#include <RSU_tracker/imm_ukf_pda.h>

ImmUkfPda::ImmUkfPda()
{
    target_id_ = 0;
    init_ = false;

    private_nh_.param<std::string>("tracking_frame", tracking_frame_, "world");
    private_nh_.param<int>("life_time_threshold", life_time_threshold_, 8);
    private_nh_.param<double>("gating_threshold", gating_threshold_, 9.22);
    private_nh_.param<double>("gate_probability", gate_probability_, 0.99);
    private_nh_.param<double>("detection_probability", detection_probability_, 0.9);
    private_nh_.param<double>("static_velocity_threshold", static_velocity_threshold_, 0.5);
    private_nh_.param<int>("static_num_history_threshold", static_num_history_threshold_, 3);
    private_nh_.param<double>("prevent_explosion_threshold", prevent_explosion_threshold_, 1000);
    private_nh_.param<double>("merge_distance_threshold", merge_distance_threshold_, 0.5);
}

void ImmUkfPda::run()
{
    pub_object_array_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/objects", 1);
    sub_detected_array_ = node_handle_.subscribe("/detection/fusion_tools/objects", 1, &ImmUkfPda::callback, this);
}

void ImmUkfPda::callback(const autoware_msgs::DetectedObjectArray &input)
{
    input_header_ = input.header;
    bool success = updateNecessaryTransform();
    if (!success) {
        ROS_INFO("Could not find coordiante transformation");
        return;
    }

    autoware_msgs::DetectedObjectArray transformed_input;
    autoware_msgs::DetectedObjectArray detected_objects_output;
    transformPoseToGlobal(input, transformed_input);
    tracker(transformed_input, detected_objects_output);
    pub_object_array_.publish(detected_objects_output);
}

void ImmUkfPda::tracker(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &detected_objects_output)
{
    double timestamp = input.header.stamp.toSec();
    std::vector<bool> matching_vec(input.objects.size(), false);

    if (!init_) {
        initTracker(input, timestamp);
        makeOutput(input, detected_objects_output);
        return;
    }

    double dt = (timestamp - timestamp_);
    timestamp_ = timestamp;

    // start UKF process
    for (size_t i = 0; i < targets_.size(); i++) {
        targets_[i].is_stable_ = false;
        targets_[i].is_static_ = false;

        if (targets_[i].tracking_num_ == TrackingState::Die)
            continue;
        // prevent ukf not to explode
        if (targets_[i].p_merge_.determinant() > prevent_explosion_threshold_ || targets_[i].p_merge_(4, 4) > prevent_explosion_threshold_) {
            targets_[i].tracking_num_ = TrackingState::Die;
            continue;
        }

        targets_[i].predictionIMMUKF(dt);

        std::vector<autoware_msgs::DetectedObject> object_vec;
        bool success = probabilisticDataAssociation(input, dt, matching_vec, object_vec, targets_[i]);
        if (!success)
            continue;

        targets_[i].updateIMMUKF(detection_probability_, gate_probability_, gating_threshold_, object_vec);
    }
    // end UKF process

    makeNewTargets(timestamp, input, matching_vec);  // making new ukf target for no data association objects
    staticClassification();                          // static dynamic classification
    makeOutput(input, detected_objects_output);      // making output for visualization
    removeUnnecessaryTarget();                       // remove unnecessary ukf object
}

void ImmUkfPda::removeUnnecessaryTarget()
{
    std::vector<UKF> temp_targets;
    for (size_t i = 0; i < targets_.size(); i++) {
        if (targets_[i].tracking_num_ != TrackingState::Die)
            temp_targets.push_back(targets_[i]);
    }
    std::vector<UKF>().swap(targets_);
    targets_ = temp_targets;
}

void ImmUkfPda::staticClassification()
{
    for (size_t i = 0; i < targets_.size(); i++) {
        // targets_[i].x_merge_(2) is referred for estimated velocity
        double current_velocity = std::abs(targets_[i].x_merge_(2));
        targets_[i].vel_history_.push_back(current_velocity);
        if (targets_[i].tracking_num_ == TrackingState::Stable && targets_[i].lifetime_ > life_time_threshold_) {
            int index = 0;
            double sum_vel = 0;
            double avg_vel = 0;
            for (auto rit = targets_[i].vel_history_.rbegin(); index < static_num_history_threshold_; ++rit) {
                index++;
                sum_vel += *rit;
            }
            avg_vel = double(sum_vel / static_num_history_threshold_);

            if (avg_vel < static_velocity_threshold_ && current_velocity < static_velocity_threshold_) {
                targets_[i].is_static_ = true;
            }
        }
    }
}

void ImmUkfPda::makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray &input, const std::vector<bool> &matching_vec)
{
    for (size_t i = 0; i < input.objects.size(); i++) {
        if (matching_vec[i] == false) {
            double px = input.objects[i].pose.position.x;
            double py = input.objects[i].pose.position.y;
            Eigen::VectorXd init_meas = Eigen::VectorXd(2);
            init_meas << px, py;

            UKF ukf;
            ukf.initialize(init_meas, timestamp, target_id_);
            ukf.object_ = input.objects[i];
            targets_.push_back(ukf);
            target_id_++;
        }
    }
}

bool ImmUkfPda::probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray &input,
                                             const double dt,
                                             std::vector<bool> &matching_vec,
                                             std::vector<autoware_msgs::DetectedObject> &object_vec,
                                             UKF &target)
{
    double det_s = 0;
    Eigen::VectorXd max_det_z;
    Eigen::MatrixXd max_det_s;
    bool success = true;

    // find maxDetS associated with predZ
    target.findMaxZandS(max_det_z, max_det_s);
    det_s = max_det_s.determinant();

    // prevent ukf not to explode
    if (std::isnan(det_s) || det_s > prevent_explosion_threshold_) {
        target.tracking_num_ = TrackingState::Die;
        success = false;
        return success;
    }

    bool is_second_init = target.tracking_num_ == TrackingState::Init ? true : false;

    measurementValidation(input, target, max_det_z, max_det_s, object_vec, matching_vec);  // measurement gating

    // second detection for a target: update v and yaw
    if (is_second_init) {
        secondInit(target, object_vec, dt);
        success = false;
        return success;
    }

    updateTargetWithAssociatedObject(object_vec, target);

    if (target.tracking_num_ == TrackingState::Die) {
        success = false;
        return success;
    }
    return success;
}

void ImmUkfPda::updateTargetWithAssociatedObject(const std::vector<autoware_msgs::DetectedObject> &object_vec, UKF &target)
{
    target.lifetime_++;
    if (!target.object_.label.empty() && target.object_.label != "unknown")
        target.label_ = target.object_.label;
    updateTrackingNum(object_vec, target);
    if (target.tracking_num_ == TrackingState::Stable || target.tracking_num_ == TrackingState::Occlusion) {
        target.is_stable_ = true;
    }
}

void ImmUkfPda::updateTrackingNum(const std::vector<autoware_msgs::DetectedObject> &object_vec, UKF &target)
{
    if (object_vec.size() > 0) {
        if (target.tracking_num_ < TrackingState::Stable)
            target.tracking_num_++;
        else if (target.tracking_num_ == TrackingState::Stable)
            target.tracking_num_ = TrackingState::Stable;
        else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
            target.tracking_num_ = TrackingState::Stable;
        else if (target.tracking_num_ == TrackingState::Lost)
            target.tracking_num_ = TrackingState::Die;
    } else {
        if (target.tracking_num_ < TrackingState::Stable)
            target.tracking_num_ = TrackingState::Die;
        else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost)
            target.tracking_num_++;
        else if (target.tracking_num_ == TrackingState::Lost)
            target.tracking_num_ = TrackingState::Die;
    }
    return;
}

void ImmUkfPda::secondInit(UKF &target, const std::vector<autoware_msgs::DetectedObject> &object_vec, double dt)
{
    if (object_vec.size() == 0) {
        target.tracking_num_ = TrackingState::Die;
        return;
    }

    // state update
    double target_x = object_vec[0].pose.position.x;
    double target_y = object_vec[0].pose.position.y;
    double target_diff_x = target_x - target.x_merge_(0);
    double target_diff_y = target_y - target.x_merge_(1);
    double target_yaw = atan2(target_diff_y, target_diff_x);
    double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
    double target_v = dist / dt;

    while (target_yaw > M_PI)
        target_yaw -= 2. * M_PI;
    while (target_yaw < -M_PI)
        target_yaw += 2. * M_PI;

    target.x_merge_(0) = target.x_cv_(0) = target.x_ctrv_(0) = target.x_rm_(0) = target_x;
    target.x_merge_(1) = target.x_cv_(1) = target.x_ctrv_(1) = target.x_rm_(1) = target_y;
    target.x_merge_(2) = target.x_cv_(2) = target.x_ctrv_(2) = target.x_rm_(2) = target_v;
    target.x_merge_(3) = target.x_cv_(3) = target.x_ctrv_(3) = target.x_rm_(3) = target_yaw;

    target.tracking_num_++;
    return;
}

void ImmUkfPda::measurementValidation(const autoware_msgs::DetectedObjectArray &input,
                                      UKF &target,
                                      const Eigen::VectorXd &max_det_z,
                                      const Eigen::MatrixXd &max_det_s,
                                      std::vector<autoware_msgs::DetectedObject> &object_vec,
                                      std::vector<bool> &matching_vec)
{
    // alert: different from original imm-pda filter, here picking up most likely measurement
    // if making it allows to have more than one measurement, you will see non semipositive definite covariance
    bool exists_smallest_nis_object = false;
    double smallest_nis = std::numeric_limits<double>::max();
    int smallest_nis_ind = 0;
    for (size_t i = 0; i < input.objects.size(); i++) {
        double x = input.objects[i].pose.position.x;
        double y = input.objects[i].pose.position.y;

        Eigen::VectorXd meas = Eigen::VectorXd(2);
        meas << x, y;

        Eigen::VectorXd diff = meas - max_det_z;
        double nis = diff.transpose() * max_det_s.inverse() * diff;

        if (nis < gating_threshold_) {
            if (nis < smallest_nis) {
                smallest_nis = nis;
                target.object_ = input.objects[i];
                smallest_nis_ind = i;
                exists_smallest_nis_object = true;
            }
        }
    }
    if (exists_smallest_nis_object) {
        matching_vec[smallest_nis_ind] = true;
        object_vec.push_back(target.object_);
    }
}

void ImmUkfPda::makeOutput(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &detected_objects_output)
{
    autoware_msgs::DetectedObjectArray tmp_objects;
    tmp_objects.header = input.header;
    std::vector<size_t> used_targets_indices;
    for (size_t i = 0; i < targets_.size(); i++) {
        double tx = targets_[i].x_merge_(0);
        double ty = targets_[i].x_merge_(1);
        double tv = targets_[i].x_merge_(2);
        double tyaw = targets_[i].x_merge_(3);
        double tyaw_rate = targets_[i].x_merge_(4);
        while (tyaw > M_PI)
            tyaw -= 2. * M_PI;
        while (tyaw < -M_PI)
            tyaw += 2. * M_PI;
        tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);

        autoware_msgs::DetectedObject dd;
        dd = targets_[i].object_;
        dd.id = targets_[i].ukf_id_;
        dd.velocity.linear.x = tv;
        dd.acceleration.linear.y = tyaw_rate;
        dd.velocity_reliable = targets_[i].is_stable_;
        dd.pose_reliable = targets_[i].is_stable_;

        // Aligh the longest side of dimentions with the estimated orientation
        if (targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y) {
            dd.dimensions.x = targets_[i].object_.dimensions.y;
            dd.dimensions.y = targets_[i].object_.dimensions.x;
        }

        if (!targets_[i].is_static_ && targets_[i].is_stable_) {
            dd.pose.position.x = tx;
            dd.pose.position.y = ty;
            if (!std::isnan(q[0]))
                dd.pose.orientation.x = q[0];
            if (!std::isnan(q[1]))
                dd.pose.orientation.y = q[1];
            if (!std::isnan(q[2]))
                dd.pose.orientation.z = q[2];
            if (!std::isnan(q[3]))
                dd.pose.orientation.w = q[3];
        }
        updateBehaviorState(targets_[i], dd);

        if (targets_[i].is_stable_ || (targets_[i].tracking_num_ >= TrackingState::Init && targets_[i].tracking_num_ < TrackingState::Stable)) {
            tmp_objects.objects.push_back(dd);
            used_targets_indices.push_back(i);
        }
    }
    detected_objects_output = removeRedundantObjects(tmp_objects, used_targets_indices);
}

autoware_msgs::DetectedObjectArray ImmUkfPda::removeRedundantObjects(const autoware_msgs::DetectedObjectArray &in_detected_objects,
                                                                     const std::vector<size_t> in_tracker_indices)
{
    if (in_detected_objects.objects.size() != in_tracker_indices.size())
        return in_detected_objects;

    autoware_msgs::DetectedObjectArray resulting_objects;
    resulting_objects.header = in_detected_objects.header;

    std::vector<geometry_msgs::Point> centroids;
    // create unique points
    for (size_t i = 0; i < in_detected_objects.objects.size(); i++) {
        if (!isPointInPool(centroids, in_detected_objects.objects[i].pose.position)) {
            centroids.push_back(in_detected_objects.objects[i].pose.position);
        }
    }
    // assign objects to the points
    std::vector<std::vector<size_t>> matching_objects(centroids.size());
    for (size_t k = 0; k < in_detected_objects.objects.size(); k++) {
        const auto &object = in_detected_objects.objects[k];
        for (size_t i = 0; i < centroids.size(); i++) {
            if (DISTANCE(object.pose.position, centroids[i]) < pow(merge_distance_threshold_, 2))
                matching_objects[i].push_back(k);  // store index of matched object to this point
        }
    }
    // get oldest object on each point
    for (size_t i = 0; i < matching_objects.size(); i++) {
        size_t oldest_object_index = 0;
        int oldest_lifespan = -1;
        std::string best_label;
        for (size_t j = 0; j < matching_objects[i].size(); j++) {
            size_t current_index = matching_objects[i][j];
            int current_lifespan = targets_[in_tracker_indices[current_index]].lifetime_;
            if (current_lifespan > oldest_lifespan) {
                oldest_lifespan = current_lifespan;
                oldest_object_index = current_index;
                if (!targets_[in_tracker_indices[current_index]].label_.empty() && targets_[in_tracker_indices[current_index]].label_ != "unknown")
                    best_label = targets_[in_tracker_indices[current_index]].label_;
            }
        }
        // delete nearby targets except for the oldest target
        for (size_t j = 0; j < matching_objects[i].size(); j++) {
            size_t current_index = matching_objects[i][j];
            if (current_index != oldest_object_index)
                targets_[in_tracker_indices[current_index]].tracking_num_ = TrackingState::Die;
        }
        autoware_msgs::DetectedObject best_object;
        best_object = in_detected_objects.objects[oldest_object_index];
        if (best_label != "unknown" && !best_label.empty()) {
            best_object.label = best_label;
        }
        resulting_objects.objects.push_back(best_object);
    }
    return resulting_objects;
}

bool ImmUkfPda::isPointInPool(const std::vector<geometry_msgs::Point> &in_pool, const geometry_msgs::Point &in_point)
{
    for (size_t i = 0; i < in_pool.size(); i++) {
        if (DISTANCE(in_pool[i], in_point) < pow(CENTROID_DISTANCE, 2))
            return true;
    }
    return false;
}

void ImmUkfPda::updateBehaviorState(const UKF &target, autoware_msgs::DetectedObject &object)
{
    if (target.mode_prob_cv_ > target.mode_prob_ctrv_ && target.mode_prob_cv_ > target.mode_prob_rm_)
        object.behavior_state = MotionModel::CV;
    else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ && target.mode_prob_ctrv_ > target.mode_prob_rm_)
        object.behavior_state = MotionModel::CTRV;
    else
        object.behavior_state = MotionModel::RM;
}

void ImmUkfPda::initTracker(const autoware_msgs::DetectedObjectArray &input, double timestamp)
{
    for (size_t i = 0; i < input.objects.size(); i++) {
        double px = input.objects[i].pose.position.x;
        double py = input.objects[i].pose.position.y;
        Eigen::VectorXd init_meas = Eigen::VectorXd(2);
        init_meas << px, py;

        UKF ukf;
        ukf.initialize(init_meas, timestamp, target_id_);
        ukf.object_ = input.objects[i];
        targets_.push_back(ukf);
        target_id_++;
    }
    timestamp_ = timestamp;
    init_ = true;
}

bool ImmUkfPda::updateNecessaryTransform()
{
    bool success = true;
    try {
        tf_listener_.waitForTransform(input_header_.frame_id, tracking_frame_, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(tracking_frame_, input_header_.frame_id, ros::Time(0), local2global_);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        success = false;
    }
    return success;
}

void ImmUkfPda::transformPoseToGlobal(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &transformed_input)
{
    transformed_input.header = input_header_;
    transformed_input.header.frame_id = tracking_frame_;
    for (auto const &object : input.objects) {
        geometry_msgs::Pose out_pose = getTransformedPose(object.pose, local2global_);

        autoware_msgs::DetectedObject dd;
        dd = object;
        dd.header.frame_id = tracking_frame_;
        dd.pose = out_pose;

        transformed_input.objects.push_back(dd);
    }
}

geometry_msgs::Pose ImmUkfPda::getTransformedPose(const geometry_msgs::Pose &in_pose, const tf::StampedTransform &tf_stamp)
{
    tf::Transform transform;
    geometry_msgs::PoseStamped out_pose;
    transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    transform.setRotation(tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    geometry_msgs::PoseStamped pose_out;
    tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
    return out_pose.pose;
}