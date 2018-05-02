#ifndef RENDERER_H_
#define RENDERER_H_


#include "renderer/utils.hpp"
#include "renderer/Preprocess.h"
#include "renderer/Render.h"


namespace renderer{


class Renderer{

private:
    // messages & services
    ros::NodeHandle                 nh_;
    image_transport::ImageTransport it_;

    bool                        display_;       // true: NOTE: then enable rviz in .launch; false: w/o any unnecessay communitcatoins
    int                         seed_;          // fix the seed for repeatative experiments

    std::string                 frm_wld_;       // "/frm_wld"
    std::string                 frm_bot_;       // "/frm_bot"
    std::string                 frm_camera_;    // "/frm_camera"

    std::string                 dir_dat_;       // load unpreprocessed pcd files
    std::string                 dir_pcd_;       // save ready pcd here
    std::string                 fil_obs_;       // file to use for outer env
    std::string                 fil_tgt_;       // file to use as target

    std::string                 fil_pcl_obs_;   // the 3d model of the empty office
    std::string                 fil_pcl_tgt_;   // the 3d model of the chair as the target
    std::string                 fil_pcl_wld_;   // the 3d model of the complete env

    pcl::PointCloud<pcl::PointXYZRGB> pcl_obs_;
    std::string                 tpc_pcl_obs_;   // "/pcl_obs": the empty shell of the office
    ros::Publisher              pub_pcl_obs_;
    sensor_msgs::PointCloud2    msg_pcl_obs_;

    pcl::PointCloud<pcl::PointXYZRGB> pcl_tgt_;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_tgt_origin_;  // the one at the origin
    std::string                 tpc_pcl_tgt_;   // "/pcl_tgt": chair
    ros::Publisher              pub_pcl_tgt_;
    sensor_msgs::PointCloud2    msg_pcl_tgt_;

    pcl::PointCloud<pcl::PointXYZRGB> pcl_wld_;
    std::string                 tpc_pcl_wld_;   // "/pcl_wld": the complete cloud
    ros::Publisher              pub_pcl_wld_;
    sensor_msgs::PointCloud2    msg_pcl_wld_;

    pcl::PointCloud<pcl::PointXYZRGB> scans_;
    std::string                 tpc_scans_;     // "/scans": simulated laser scans
    ros::Publisher              pub_scans_;
    sensor_msgs::PointCloud2    msg_scans_;

    std::string                 tpc_depth_;     // "/depth": depth image from /scans
    image_transport::Publisher  pub_depth_;
    sensor_msgs::ImagePtr       msg_depth_;
    cv::Mat                     img_depth_;

    // std::string                 tpc_depth_array_;   // "/depth_array": depth image stored in array
    // ros::Publisher              pub_depth_array_;
    // std_msgs::Float32MultiArray msg_depth_array_;

    std::string                 tpc_color_;     // "/color": color image from /scans
    image_transport::Publisher  pub_color_;
    sensor_msgs::ImagePtr       msg_color_;
    cv::Mat                     img_color_;

    // std::string                 tpc_color_array_;   // "/color_array": color image stored in array
    // ros::Publisher              pub_color_array_;
    // std_msgs::Float32MultiArray msg_color_array_;

    std::string                 tpc_mkr_camera_;// "/mkr_camera"
    ros::Publisher              pub_mkr_camera_;
    visualization_msgs::Marker  msg_mkr_camera_;

    double ind2val_;       // real distance represented by each cube
    int    map_hei_ind_;   // for drawing wld config
    int    map_wid_ind_;   // for drawing wld config
    double floorZ_val_;    // to put /pcWld & /pcMap
    double tgtZ_val_;      // to put /pcObj
    double obsZ_min_val_;  // counted as obs from this height
    double cameraX_val_;   // kinect's relative x w.r.t. bot
    double cameraZ_val_;   // z for bot == kinect height

    int    num_act_;       // 3: {0: GO_STRAIGHT; 1: TURN_LEFT; 2: TURN_RIGHT}
    int    render_hei_;
    int    render_wid_;
    double angleY_min_;
    double angleY_max_;
    double angleY_increment_;
    double angleZ_min_;
    double angleZ_max_;
    double sense_min_;     // the actual sensing limit
    double sense_max_;     // the acrual sensing limit
    double value_miss_;    // range value for misses
    double range_min_;     // range returned smaller than this will be marked as value_miss_
    double range_max_;     // range returned bigger  than this will be marked as value_miss_
    bool   use_inf_;
    double render_angleY_inc_;
    double render_angleZ_inc_;

    // map parses
    std::vector<pair<int, int>> map_free_indices_; // [y, x] : #free cells
    double map_y_min_val_;
    double map_y_max_val_;
    double map_x_min_val_;
    double map_x_max_val_;

    // motion
    double          step_dist_val_;     //  the distance travelled by the agent for each time step
    double          step_deg_;          // the degree turned by the agent for each time step
    Eigen::Matrix4f go_straight_mat_;
    Eigen::Matrix4f turn_left_mat_;
    Eigen::Matrix4f turn_right_mat_;
    std::default_random_engine       y_noise_gen;
    std::default_random_engine       x_noise_gen;
    std::default_random_engine       r_noise_gen;
    std::normal_distribution<double> y_noise;
    std::normal_distribution<double> x_noise;
    std::normal_distribution<double> r_noise;

    // rl
    state                       state_;

    double reward_step_;
    double reward_collision_;
    double reward_reach_;
    double dist_reach_;

    double tgt_y_val_;
    double tgt_x_val_;
    geometry_msgs::Pose         pos_bot_;       // pose without noise
    geometry_msgs::Pose         bot_camera_;    // pose without noise
    geometry_msgs::Pose         pos_camera_;    // pose without noise
    tf::Transform               tf_wld_bot_;    // transformation between /frm_wld and /frm_bot, i.e. the current pos_bot_
    tf::Transform               tf_bot_camera_; // transformation between /frm_bot and /frm_camera
    tf::TransformBroadcaster    tfb_wld_bot_;
    tf::TransformBroadcaster    tfb_bot_camera_;
    // tf::TransformListener       listener_wld_camera_;

    // int                         hei_screen_;
    // int                         wid_screen_;

    geometry_msgs::Twist        msg_stand_still_;
    geometry_msgs::Twist        msg_turn_left_;
    geometry_msgs::Twist        msg_turn_right_;
    geometry_msgs::Twist        msg_go_straight_;

    std::string                 tpc_screen_; // "/screen"
    sensor_msgs::Image          msg_screen_;
    std_msgs::Float32MultiArray msg_screen_array_;
    image_transport::Publisher  pub_screen_;

    // services
    std::string		   tpc_preprocess_; // "/preprocess"
    ros::ServiceServer srv_preprocess_;

    std::string		   tpc_render_;     // "/render"
    ros::ServiceServer srv_render_;

public:
    // basics
	Renderer();
    ~Renderer();

    // helper funcs
    bool getIndFromAng(int& ind_y,
                       int& ind_z,
                       const double& ang_y,
                       const double& ang_z);
    bool getIndFromVal(int& y_ind,
                       int& x_ind,
                       const double& y_val,
                       const double& x_val);
    bool getValFromInd(double& y_val,
                       double& x_val,
                       const int& y_ind,
                       const int& x_ind);

    // core funcs
    bool renderImageFromPose(const geometry_msgs::Pose& pose);
    bool reset();
    bool step(const int& act_ind);
    bool act(const int& act_ind);

    // callbacks
    bool preprocessCallback(renderer::Preprocess::Request&  req,
					        renderer::Preprocess::Response& res);
    bool renderCallback(renderer::Render::Request&  req,
					    renderer::Render::Response& res);

    // init
    void initROS();

};


} // namespace


#endif
