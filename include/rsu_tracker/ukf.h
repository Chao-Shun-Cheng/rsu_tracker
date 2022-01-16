#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"

#include "autoware_msgs/DetectedObject.h"

// parameter for sigma weight
#define ALPHA 0.0025
#define BETA 2
#define K 0

// cout color
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

enum TrackingState : int {
    Die = 0,        // No longer tracking
    Init = 1,       // Start tracking
    Stable = 4,     // Stable tracking
    Occlusion = 5,  // Lost 1 frame possibly by occlusion
    Lost = 10,      // About to lose target
};

enum MotionModel : int {
    CV = 0,    // constant velocity
    CTRV = 1,  // constant turn rate and velocity
    RM = 2,    // random motion
};

class UKF
{
    /*
    cv: Constant Velocity
    ctrv: Constatnt Turn Rate and Velocity
    rm: Random Motion
    */
public:
    bool use_vector_map_;
    bool debug_;
    int ukf_id_;
    int num_state_;
    int RSU_state_;
    int RSU_lane_state_;
    int num_motion_model_;

    //* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::MatrixXd x_merge_;
    Eigen::MatrixXd x_cv_;
    Eigen::MatrixXd x_ctrv_;
    Eigen::MatrixXd x_rm_;

    //* state covariance matrix
    Eigen::MatrixXd p_merge_;
    Eigen::MatrixXd p_cv_;
    Eigen::MatrixXd p_ctrv_;
    Eigen::MatrixXd p_rm_;

    //* predicted sigma points matrix
    Eigen::MatrixXd x_sig_pred_cv_;
    Eigen::MatrixXd x_sig_pred_ctrv_;
    Eigen::MatrixXd x_sig_pred_rm_;

    //* process error matrix Q
    Eigen::MatrixXd q_cv_;
    Eigen::MatrixXd q_ctrv_;
    Eigen::MatrixXd q_rm_;

    //* measurement error matrix R
    Eigen::MatrixXd r_cv_;
    Eigen::MatrixXd r_ctrv_;
    Eigen::MatrixXd r_rm_;

    //* time when the state is true, in us
    long long time_;

    //* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_cv_;
    double std_a_ctrv_;
    double std_a_rm_;

    // CTRV
    double std_ctrv_yawdd_;
    // CV
    double std_cv_yawdd_;

    double std_rm_yawdd_;

    //* measurement noise standard deviation position1 in m
    double std_laspx_;
    double std_laspy_;
    double std_lane_direction_;

    //* Weights of sigma points
    Eigen::VectorXd weights_c_;
    Eigen::VectorXd weights_s_;

    //* Sigma point spreading parameter
    double lambda_;

    double mode_match_prob_cv2cv_;
    double mode_match_prob_ctrv2cv_;
    double mode_match_prob_rm2cv_;

    double mode_match_prob_cv2ctrv_;
    double mode_match_prob_ctrv2ctrv_;
    double mode_match_prob_rm2ctrv_;

    double mode_match_prob_cv2rm_;
    double mode_match_prob_ctrv2rm_;
    double mode_match_prob_rm2rm_;

    double mode_prob_cv_;
    double mode_prob_ctrv_;
    double mode_prob_rm_;

    std::vector<double> p1_;

    std::vector<double> p2_;

    std::vector<double> p3_;

    Eigen::VectorXd z_pred_cv_;
    Eigen::VectorXd z_pred_ctrv_;
    Eigen::VectorXd z_pred_rm_;

    Eigen::MatrixXd s_cv_;
    Eigen::MatrixXd s_ctrv_;
    Eigen::MatrixXd s_rm_;

    Eigen::MatrixXd k_cv_;
    Eigen::MatrixXd k_ctrv_;
    Eigen::MatrixXd k_rm_;

    // ----- for using vector map ----- //
    Eigen::MatrixXd z_pred_cv_lane;
    Eigen::MatrixXd z_pred_ctrv_lane;
    Eigen::MatrixXd z_pred_rm_lane;

    Eigen::MatrixXd s_cv_lane;
    Eigen::MatrixXd s_ctrv_lane;
    Eigen::MatrixXd s_rm_lane;

    Eigen::MatrixXd k_cv_lane;
    Eigen::MatrixXd k_ctrv_lane;
    Eigen::MatrixXd k_rm_lane;

    Eigen::MatrixXd r_cv_lane;
    Eigen::MatrixXd r_ctrv_lane;
    Eigen::MatrixXd r_rm_lane;

    bool is_direction_ctrv_available_;
    bool is_direction_cv_available_;
    // ----- for using vector map ----- //

    double pd_;
    double pg_;

    int lifetime_;
    bool is_static_;

    // object msg information
    bool is_stable_;
    autoware_msgs::DetectedObject object_;
    std::string label_;

    std::vector<double> vel_history_;

    double x_merge_yaw_;

    int tracking_num_;

    /**
     * Constructor
     */
    UKF(bool use_vector_map, bool debug);

    void updateYawWithHighProb();

    void initialize(const Eigen::VectorXd &z, const double timestamp, const int target_ind);

    void updateModeProb(const std::vector<double> &lambda_vec);

    void mergeEstimationAndCovariance();

    void mixingProbability();

    void interaction();

    void predictionIMMUKF(const double dt);

    void findMaxZandS(Eigen::VectorXd &max_det_z, Eigen::MatrixXd &max_det_s);

    void updateEachMotion(const float detection_probability,
                          const float gate_probability,
                          const float gating_threshold,
                          const std::vector<autoware_msgs::DetectedObject> &object_vec,
                          std::vector<double> &lambda_vec);

    void updateIMMUKF(const float detection_probability,
                      const float gate_probability,
                      const float gating_threshold,
                      const std::vector<autoware_msgs::DetectedObject> &object_vec);

    void
    ctrv(const double p_x, const double p_y, const double v, const double yaw, const double yawd, const double delta_t, std::vector<double> &state);

    void
    cv(const double p_x, const double p_y, const double v, const double yaw, const double yawd, const double delta_t, std::vector<double> &state);

    void randomMotion(const double p_x,
                      const double p_y,
                      const double v,
                      const double yaw,
                      const double yawd,
                      const double delta_t,
                      std::vector<double> &state);

    void initCovarQs(const double dt, const double yaw);

    void predictionMotion(const double delta_t, const int model_ind);

    void predictionLidarMeasurement(const int motion_ind, const int num_meas_state);

    // void updateKalmanGain(const int motion_ind, const int num_meas_state);
    void updateKalmanGain(const int motion_ind);

    double normalizeAngle(const double angle);

    bool isLaneDirectionAvailable(const double &angle, const float yaw_threshold);

    double calculateNIS(const double &angle, const int motion_ind);
};
