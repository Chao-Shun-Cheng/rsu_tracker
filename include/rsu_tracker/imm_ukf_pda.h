#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector_map/vector_map.h>
#include <iostream>
#include <vector>
#include <fstream>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "op_utility/DataRW.h"
#include "ukf.h"

#define CENTROID_DISTANCE 0.2  // distance to consider centroids the same
#define DISTANCE(p1, p2) (pow(fabs(p1.x - p2.x), 2) + pow(fabs(p1.y - p2.y), 2))

class ImmUkfPda
{
private:
    bool debug_;
    bool use_vector_map_;
    bool output_result_;
    UtilityHNS::MapRaw m_MapRaw;
    int target_id_;
    bool init_;
    double timestamp_;

    std::vector<UKF> targets_;

    // probabilistic data association params
    float gating_threshold_;
    float gate_probability_;
    float detection_probability_;

    // object association param
    int life_time_threshold_;

    // static classification param
    float static_velocity_threshold_;
    int static_num_history_threshold_;

    // prevent explode param for ukf
    float prevent_explosion_threshold_;

    float merge_distance_threshold_;
    float lane_distance_threshold_;
    float yaw_threshold_;
    std::string tracking_frame_;
    std::string logfile_name_;
    std::string save_path_;
    std::string sub_topic_;
    std::string pub_topic_;
    std::ofstream logfile;
    
    tf::TransformListener tf_listener_;
    tf::StampedTransform local2global_;

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_detected_array_;
    ros::Subscriber sub_lanes;
    ros::Subscriber sub_points;
    ros::Subscriber sub_nodes;
    ros::Publisher pub_object_array_;

    std_msgs::Header input_header_;
    int Frame = 2;

    void callback(const autoware_msgs::DetectedObjectArray &input);

    void callbackGetVMLanes(const vector_map_msgs::LaneArray &msg);

    void callbackGetVMPoints(const vector_map_msgs::PointArray &msg);

    void callbackGetVMNodes(const vector_map_msgs::NodeArray &msg);

    void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &transformed_input);

    geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose &in_pose, const tf::StampedTransform &tf_stamp);

    bool updateNecessaryTransform();

    void measurementValidation(const autoware_msgs::DetectedObjectArray &input,
                               UKF &target,
                               const Eigen::VectorXd &max_det_z,
                               const Eigen::MatrixXd &max_det_s,
                               std::vector<autoware_msgs::DetectedObject> &object_vec,
                               std::vector<bool> &matching_vec);

    void updateBehaviorState(const UKF &target, autoware_msgs::DetectedObject &object);

    void initTracker(const autoware_msgs::DetectedObjectArray &input, double timestamp);

    void secondInit(UKF &target, const std::vector<autoware_msgs::DetectedObject> &object_vec, double dt);

    void updateTrackingNum(const std::vector<autoware_msgs::DetectedObject> &object_vec, UKF &target);

    bool probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray &input,
                                      const double dt,
                                      std::vector<bool> &matching_vec,
                                      std::vector<autoware_msgs::DetectedObject> &object_vec,
                                      UKF &target);

    void makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray &input, const std::vector<bool> &matching_vec);

    void staticClassification();

    void makeOutput(const autoware_msgs::DetectedObjectArray &input, autoware_msgs::DetectedObjectArray &detected_objects_output);

    void removeUnnecessaryTarget();

    void tracker(const autoware_msgs::DetectedObjectArray &transformed_input, autoware_msgs::DetectedObjectArray &detected_objects_output);

    autoware_msgs::DetectedObjectArray removeRedundantObjects(const autoware_msgs::DetectedObjectArray &in_detected_objects,
                                                              const std::vector<size_t> in_tracker_indices);

    bool isPointInPool(const std::vector<geometry_msgs::Point> &in_pool, const geometry_msgs::Point &in_point);

    void updateTargetWithAssociatedObject(const std::vector<autoware_msgs::DetectedObject> &object_vec, UKF &target);

    bool findYawFromVectorMap(const double &pos_x, const double &pos_y, double &yaw);

    void get_logfilename();

    void saveResult(const autoware_msgs::DetectedObjectArray& input, const autoware_msgs::DetectedObjectArray& output);

public:
    ImmUkfPda();
    void run();
};