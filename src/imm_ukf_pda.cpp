#include <rsu_tracker/imm_ukf_pda.h>

ImmUkfPda::ImmUkfPda() : private_nh_("~")
{
    target_id_ = 0;
    init_ = false;
    private_nh_.param<std::string>("tracking_frame", tracking_frame_, "world");
    private_nh_.param<std::string>("sub_topic", sub_topic_, "/detection/lidar_detector/objects_filtered");
    private_nh_.param<std::string>("pub_topic", pub_topic_, "/simulator/tracker/objects");
    private_nh_.param<int>("life_time_threshold", life_time_threshold_, 8);
    private_nh_.param<float>("gating_threshold", gating_threshold_, 9.22);
    private_nh_.param<float>("gate_probability", gate_probability_, 0.99);
    private_nh_.param<float>("detection_probability", detection_probability_, 0.9);
    private_nh_.param<float>("static_velocity_threshold", static_velocity_threshold_, 0.5);
    private_nh_.param<int>("static_num_history_threshold", static_num_history_threshold_, 3);
    private_nh_.param<float>("prevent_explosion_threshold", prevent_explosion_threshold_, 1000);
    private_nh_.param<float>("merge_distance_threshold", merge_distance_threshold_, 0.5);
    private_nh_.param<float>("lane_distance_threshold", lane_distance_threshold_, 1);
    private_nh_.param<float>("yaw_threshold", yaw_threshold_, 2.7);
    private_nh_.param<bool>("use_vector_map", use_vector_map_, false);
    private_nh_.param<bool>("debug", debug_, true);
    private_nh_.param<bool>("experiment_lilee", experiment_lilee_, true);
    private_nh_.param<bool>("output_result", output_result_, false);
    if (output_result_) {
        private_nh_.param<std::string>("groundTruth_topic", groundTruth_topic_, "/lgsvl/ground_truth/objects");
        private_nh_.param<std::string>("logfile_name", logfile_name_, "shalun_2_");
        private_nh_.param<std::string>("save_path", save_path_, "/home/kenny/catkin_ws/src/track_to_track_fusion/rsu_tracker/");
        get_logfilename();
    }
}

void ImmUkfPda::get_logfilename()
{
    logfile.open(save_path_ + logfile_name_, std::ofstream::out | std::ofstream::app);
    if (!logfile.is_open()) {
        std::cerr << RED << "failed to open " << save_path_ << logfile_name_ << RESET << '\n';
    } else {
        logfile << "Time,Computed_time,Ground_x,Ground_y,Ground_velocity,Ground_yaw,measurement_x,measurement_y,"
                << "TrackingState,Tracking_x,Tracking_y,Tracking_velocity,Tracking_yaw,Tracking_yaw_rate,"
                << "covariance_x,covariance_y,covariance_velocity,covariance_yaw,covariance_yaw_rate,"
                << "prob_CV,prob_CTRV,prob_RM\n";
        logfile.close();
        std::cout << YELLOW << "save path : " << save_path_ + logfile_name_ << RESET << std::endl;
    }
    return;
}

void ImmUkfPda::run()
{
    pub_object_array_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(pub_topic_, 1);
    sub_detected_array_ = node_handle_.subscribe(sub_topic_, 1, &ImmUkfPda::callback, this);
    if (output_result_)
        sub_ground_truth_ = node_handle_.subscribe(groundTruth_topic_, 1, &ImmUkfPda::callbackGroundTruth, this);
    if (use_vector_map_) {
        sub_lanes = node_handle_.subscribe("/vector_map_info/lane", 1, &ImmUkfPda::callbackGetVMLanes, this);
        sub_points = node_handle_.subscribe("/vector_map_info/point", 1, &ImmUkfPda::callbackGetVMPoints, this);
        sub_nodes = node_handle_.subscribe("/vector_map_info/node", 1, &ImmUkfPda::callbackGetVMNodes, this);
    }
}

void ImmUkfPda::callbackGroundTruth(const autoware_msgs::DetectedObjectArray &input)
{
    bool success = updateNecessaryTransform(local2global_ground_truth_, input, tracking_frame_);
    if (!success) {
        ROS_INFO("Could not find ground truth coordiante transformation");
        return;
    }
    transformPoseToGlobal(input, ground_truth);
}

void ImmUkfPda::callbackGetVMLanes(const vector_map_msgs::LaneArray &msg)
{
    if (m_MapRaw.pLanes == nullptr) {
        m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
        std::cout << GREEN << "Received Lanes : " << RESET << m_MapRaw.pLanes->m_data_list.size() << std::endl;
    }
}

void ImmUkfPda::callbackGetVMPoints(const vector_map_msgs::PointArray &msg)
{
    if (m_MapRaw.pPoints == nullptr) {
        m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
        std::cout << GREEN << "Received Points : " << RESET << m_MapRaw.pPoints->m_data_list.size() << std::endl;
    }
}

void ImmUkfPda::callbackGetVMNodes(const vector_map_msgs::NodeArray &msg)
{
    if (m_MapRaw.pNodes == nullptr) {
        m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
        std::cout << GREEN << "Received Nodes : " << RESET << m_MapRaw.pNodes->m_data_list.size() << std::endl;
    }
}

void ImmUkfPda::callback(const autoware_msgs::DetectedObjectArray &input)
{
    if (input.objects.size() == 0) {
        std::cout << RED << "Empty INPUT !!!!" << RESET << std::endl;
        return;
    }
    if (debug_)
        std::cout << "----- Frame : " << Frame++ << " -----" << std::endl;

    if (use_vector_map_ && (m_MapRaw.pLanes == nullptr || m_MapRaw.pPoints == nullptr || m_MapRaw.pNodes == nullptr)) {
        std::cout << YELLOW << "Loading for vector map ...." << RESET << std::endl;
        return;
    }

    input_header_ = input.header;
    bool success = updateNecessaryTransform(local2global_, input, tracking_frame_);
    if (!success) {
        ROS_INFO("Could not find coordiante transformation");
        return;
    }
    ros::Time start = ros::Time::now();
    
    autoware_msgs::DetectedObjectArray detected_objects_output;
    
    autoware_msgs::DetectedObjectArray filter_input;
    if (experiment_lilee_) {
        bool success = updateNecessaryTransform(local2vehicle_, input, "velodyne");
        if (!success) {
            ROS_INFO("Could not find velodyne and RSU coordiante transformation");
            return;
        }

        filter_input.header = input.header;
        int min_index = -1;
        double min_distance = DBL_MAX;
        for (int i = 0; i < input.objects.size(); i++) {
            geometry_msgs::Pose out_pose = getTransformedPose(input.objects[i].pose, local2vehicle_);
            if (out_pose.position.x > 2.5 && abs(out_pose.position.y) < 1) {
                if (min_distance > abs(out_pose.position.y)) {
                    min_index = i;
                    min_distance = abs(out_pose.position.y);
                }
            }
        }
        if (min_index != -1) {
            filter_input.objects.push_back(input.objects[min_index]);
            transformPoseToGlobal(filter_input, transformed_input);
        } else
            return;
    } else
        transformPoseToGlobal(input, transformed_input);

    tracker(transformed_input, detected_objects_output);
    pub_object_array_.publish(detected_objects_output);
    ros::Time end = ros::Time::now();

    if (output_result_&& detected_objects_output.objects.size() != 0 && ground_truth.objects.size() != 0) {
        saveResult(transformed_input, ground_truth, detected_objects_output, end.toSec() - start.toSec());
    }

    if (debug_ && detected_objects_output.objects.size() == 0) {
        std::cout << RED << "Tracking Size : " << detected_objects_output.objects.size() << RESET << std::endl;
        return;
    }


    if (detected_objects_output.objects[0].user_defined_info[3] == "4")
        std::cout << GREEN << "State : " << 4 << RESET << std::endl;
    else
        std::cout << RED << "State : " << detected_objects_output.objects[0].user_defined_info[3] << RESET << std::endl;
}

void ImmUkfPda::tracker(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &detected_objects_output)
{
    double timestamp = input.header.stamp.toSec();
    std::vector<bool> matching_vec(input.objects.size(), false);

    if (!init_) {
        if (debug_)
            std::cout << YELLOW << "Start initTracker" << RESET << std::endl;
        initTracker(input, timestamp);
        if (debug_)
            std::cout << YELLOW << "Start makeOutput" << RESET << std::endl;
        makeOutput(input, detected_objects_output);
        return;
    }

    double dt = (timestamp - timestamp_);
    timestamp_ = timestamp;

    // start UKF process
    for (size_t i = 0; i < targets_.size(); i++) {
        if (debug_)
            std::cout << YELLOW << "Start UKF Process" << RESET << std::endl;

        targets_[i].is_stable_ = false;
        targets_[i].is_static_ = false;

        if (targets_[i].tracking_num_ == TrackingState::Die)
            continue;
        // prevent ukf not to explode
        if (targets_[i].p_merge_.determinant() > prevent_explosion_threshold_ || targets_[i].p_merge_(4, 4) > prevent_explosion_threshold_) {
            targets_[i].tracking_num_ = TrackingState::Die;
            std::cout << YELLOW << "estimate covariance is explosion ..." << RESET << std::endl;
            continue;
        }

        if (debug_)
            std::cout << YELLOW << "Start predictionIMMUKF" << RESET << std::endl;
        targets_[i].predictionIMMUKF(dt);

        if (debug_)
            std::cout << YELLOW << "Start probabilisticDataAssociation" << RESET << std::endl;
        std::vector<autoware_msgs::DetectedObject> object_vec;
        bool success = probabilisticDataAssociation(input, dt, matching_vec, object_vec, targets_[i]);
        if (!success)
            continue;

        if (debug_)
            std::cout << YELLOW << "Start updateIMMUKF" << RESET << std::endl;
        targets_[i].updateIMMUKF(detection_probability_, gate_probability_, gating_threshold_, object_vec);
    }
    // end UKF process
    if (debug_)
        std::cout << YELLOW << "Start makeOutput" << RESET << std::endl;
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

            UKF ukf(use_vector_map_, debug_);
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
        if (std::isnan(det_s))
            std::cout << YELLOW << "measurement covariance is nan" << RESET << std::endl;
        else
            std::cout << YELLOW << "measurement covariance is explosion : " << det_s << RESET << std::endl;
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
        if (use_vector_map_) {
            double yaw = 0;
            bool nearbylane = findYawFromVectorMap(target.object_.pose.position.x, target.object_.pose.position.y, yaw);
            if (nearbylane) {
                if (target.isLaneDirectionAvailable(yaw, yaw_threshold_)) {
                    target.object_.angle = yaw;
                }
            }
        }
        object_vec.push_back(target.object_);
    }
}

void ImmUkfPda::makeOutput(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &detected_objects_output)
{
    autoware_msgs::DetectedObjectArray tmp_objects;
    tmp_objects.header = input.header;
    std::vector<size_t> used_targets_indices;
    for (size_t i = 0; i < targets_.size(); i++) {
        autoware_msgs::DetectedObject dd;
        dd = targets_[i].object_;
        dd.id = targets_[i].ukf_id_;
        dd.pose.position.x = targets_[i].x_merge_(0);
        dd.pose.position.y = targets_[i].x_merge_(1);
        dd.velocity.linear.x = targets_[i].x_merge_(2);
        dd.acceleration.linear.y = targets_[i].x_merge_(4);
        dd.velocity_reliable = targets_[i].is_stable_;
        dd.pose_reliable = targets_[i].is_stable_;

        double tyaw = targets_[i].x_merge_(3);
        while (tyaw > M_PI)
            tyaw -= 2. * M_PI;
        while (tyaw < -M_PI)
            tyaw += 2. * M_PI;
        tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);
        if (!std::isnan(q[0]))
            dd.pose.orientation.x = q[0];
        if (!std::isnan(q[1]))
            dd.pose.orientation.y = q[1];
        if (!std::isnan(q[2]))
            dd.pose.orientation.z = q[2];
        if (!std::isnan(q[3]))
            dd.pose.orientation.w = q[3];

        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++)
                dd.covariance[j * 5 + k] = targets_[i].p_merge_(j, k);
        }
        // Aligh the longest side of dimentions with the estimated orientation
        if (targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y) {
            dd.dimensions.x = targets_[i].object_.dimensions.y;
            dd.dimensions.y = targets_[i].object_.dimensions.x;
        }

        updateBehaviorState(targets_[i], dd);

        if (targets_[i].is_stable_ || (targets_[i].tracking_num_ >= TrackingState::Init && targets_[i].tracking_num_ < TrackingState::Stable)) {
            tmp_objects.objects.push_back(dd);
            used_targets_indices.push_back(i);
        }
    }
    detected_objects_output = removeRedundantObjects(tmp_objects, used_targets_indices);
}

void ImmUkfPda::saveResult(const autoware_msgs::DetectedObjectArray &ground_truth,
                           const autoware_msgs::DetectedObjectArray &measurement,
                           const autoware_msgs::DetectedObjectArray &output,
                           const double &cost_time)
{
    if ((measurement.objects.size() != 1) || (output.objects.size() != 1)) {
        std::cout << YELLOW << "The size of input and output is not same ..." << RESET << std::endl;
    }
    logfile.open(save_path_ + logfile_name_, std::ofstream::out | std::ofstream::app);
    if (!logfile.is_open()) {
        std::cerr << RED << "failed to open " << save_path_ << logfile_name_ << RESET << '\n';
    } else {
        // Time, Computed time
        logfile << std::to_string(ground_truth.header.stamp.toSec()) << "," << std::to_string(cost_time) << ",";
        // Ground x, Ground y, Ground velocity, Ground yaw, Measurement x, Measurement y
        logfile << std::to_string(ground_truth.objects[0].pose.position.x) << "," << std::to_string(ground_truth.objects[0].pose.position.y) << ","
                << std::to_string(ground_truth.objects[0].velocity.linear.x) << ","
                << std::to_string(tf::getYaw(ground_truth.objects[0].pose.orientation)) << ","
                << std::to_string(measurement.objects[0].pose.position.x) << "," << std::to_string(measurement.objects[0].pose.position.y) << ",";

        if (output.objects.size() == 0) {
            logfile << std::endl;
            logfile.close();
            return;
        }

        // Tracking State, Tracking x, Tracking y, Tracking velocity, Tracking yaw, Tracking yaw rate
        logfile << output.objects[0].user_defined_info[3] << "," << std::to_string(output.objects[0].pose.position.x) << ","
                << std::to_string(output.objects[0].pose.position.y) << "," << std::to_string(output.objects[0].velocity.linear.x) << ","
                << std::to_string(tf::getYaw(output.objects[0].pose.orientation)) << "," << std::to_string(output.objects[0].acceleration.linear.y)
                << ",";
        // covariance x, covariance y, covariance velocity, covariance yaw, covariance yaw rate
        logfile << std::to_string(output.objects[0].covariance[0]) << "," << std::to_string(output.objects[0].covariance[6]) << ","
                << std::to_string(output.objects[0].covariance[12]) << "," << std::to_string(output.objects[0].covariance[18]) << ","
                << std::to_string(output.objects[0].covariance[24]) << ",";
        // prob CV, prob CTRV, prob RM
        logfile << output.objects[0].user_defined_info[0] << "," << output.objects[0].user_defined_info[1] << ","
                << output.objects[0].user_defined_info[2] << std::endl;
        logfile.close();
    }
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
            if (current_index != oldest_object_index) {
                targets_[in_tracker_indices[current_index]].tracking_num_ = TrackingState::Die;
            }
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
        if (DISTANCE(in_pool[i], in_point) < CENTROID_DISTANCE * CENTROID_DISTANCE)
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

    object.user_defined_info.clear();
    object.user_defined_info.push_back(std::to_string(target.mode_prob_cv_));
    object.user_defined_info.push_back(std::to_string(target.mode_prob_ctrv_));
    object.user_defined_info.push_back(std::to_string(target.mode_prob_rm_));
    object.user_defined_info.push_back(std::to_string(target.tracking_num_));
}

void ImmUkfPda::initTracker(const autoware_msgs::DetectedObjectArray &input, double timestamp)
{
    for (size_t i = 0; i < input.objects.size(); i++) {
        double px = input.objects[i].pose.position.x;
        double py = input.objects[i].pose.position.y;
        Eigen::VectorXd init_meas = Eigen::VectorXd(2);
        init_meas << px, py;
        UKF ukf(use_vector_map_, debug_);
        ukf.initialize(init_meas, timestamp, target_id_);
        targets_.push_back(ukf);
        target_id_++;
    }
    timestamp_ = timestamp;
    init_ = true;
}

bool ImmUkfPda::findYawFromVectorMap(const double &pos_x, const double &pos_y, double &yaw)
{
    double min_distance = DBL_MAX;
    int min_index = -1;
    for (int i = 0; i < m_MapRaw.pLanes->m_data_list.size(); i++) {
        UtilityHNS::AisanNodesFileReader::AisanNode *N = m_MapRaw.pNodes->GetDataRowById(m_MapRaw.pLanes->m_data_list[i].BNID);
        UtilityHNS::AisanPointsFileReader::AisanPoints *P = m_MapRaw.pPoints->GetDataRowById(N->PID);
        double distance = pow(pow((pos_x - P->Ly), 2) + pow((pos_y - P->Bx), 2), 0.5);
        if (distance < lane_distance_threshold_ && distance < min_distance) {
            min_distance = distance;
            min_index = i;
        }
    }

    if (min_index == -1)
        return false;

    UtilityHNS::AisanNodesFileReader::AisanNode *n_start = m_MapRaw.pNodes->GetDataRowById(m_MapRaw.pLanes->m_data_list[min_index].BNID);
    UtilityHNS::AisanPointsFileReader::AisanPoints *p_start = m_MapRaw.pPoints->GetDataRowById(n_start->PID);
    UtilityHNS::AisanNodesFileReader::AisanNode *n_end = m_MapRaw.pNodes->GetDataRowById(m_MapRaw.pLanes->m_data_list[min_index].FNID);
    UtilityHNS::AisanPointsFileReader::AisanPoints *p_end = m_MapRaw.pPoints->GetDataRowById(n_end->PID);
    yaw = atan2((p_end->Bx - p_start->Bx), (p_end->Ly - p_start->Ly));
    return true;
}

bool ImmUkfPda::updateNecessaryTransform(tf::StampedTransform &local2global_, const autoware_msgs::DetectedObjectArray &input, const std::string frame)
{
    bool success = true;
    try {
        tf_listener_.waitForTransform(input.header.frame_id, frame, ros::Time(0), ros::Duration(3));
        tf_listener_.lookupTransform(frame, input.header.frame_id, ros::Time(0), local2global_);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        success = false;
    }
    return success;
}

void ImmUkfPda::transformPoseToGlobal(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &transformed_input)
{
    transformed_input.objects.clear();
    transformed_input.header = input.header;
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