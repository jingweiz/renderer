#include "renderer/renderer.hpp"
#include "maps.cpp"

namespace renderer{


// basics


Renderer::Renderer():
    nh_("~"),
    it_(nh_){
    nh_.param("display",         this->display_,            true);
    nh_.param("seed",            this->seed_,               123);
    nh_.param("frm_wld",         this->frm_wld_,            std::string("frm_wld"));
    nh_.param("frm_bot",         this->frm_bot_,            std::string("frm_bot"));
    nh_.param("frm_camera",      this->frm_camera_,         std::string("frm_camera"));
    // NOTE: store original unaligned clouds in this->dir_dat_, call /preprocess to preprocess them
    nh_.param("dir_dat",         this->dir_dat_,            std::string("/home/zhang/kinetic_ws/src/scanner/data/"));
    // NOTE: /preprocess stores the aligned clouds into this->dir_pcd_
    nh_.param("dir_pcd",         this->dir_pcd_,            std::string("/home/zhang/kinetic_ws/src/scanner/pcd/"));
    nh_.param("fil_obs",         this->fil_obs_,            std::string("office"));
    nh_.param("fil_tgt",         this->fil_tgt_,            std::string("chair"));
    nh_.param("tpc_pcl_obs",     this->tpc_pcl_obs_,        std::string("/pcl_obs"));
    nh_.param("tpc_pcl_tgt",     this->tpc_pcl_tgt_,        std::string("/pcl_tgt"));
    nh_.param("tpc_pcl_wld",     this->tpc_pcl_wld_,        std::string("/pcl_wld"));
    nh_.param("tpc_scans",       this->tpc_scans_,          std::string("/scans"));
    nh_.param("tpc_depth",       this->tpc_depth_,          std::string("/depth"));
    // nh_.param("tpc_depth_array", this->tpc_depth_array_,    std::string("/depth_array"));
    nh_.param("tpc_color",       this->tpc_color_,          std::string("/color"));
    // nh_.param("tpc_color_array", this->tpc_color_array_,    std::string("/color_array"));
    nh_.param("tpc_mkr_camera",  this->tpc_mkr_camera_,     std::string("/mkr_camera"));
    nh_.param("ind2val",         this->ind2val_,            0.25);
    nh_.param("map_hei_ind",     this->map_hei_ind_,        37);
    nh_.param("map_wid_ind",     this->map_wid_ind_,        16);
    nh_.param("floorZ_val",      this->floorZ_val_,         -0.08);
    nh_.param("tgtZ_val",        this->tgtZ_val_,           0.02);
    nh_.param("obsZ_min_val",    this->obsZ_min_val_,       0.10);
    nh_.param("cameraX_val_",    this->cameraX_val_,        0.135);
    nh_.param("cameraZ_val_",    this->cameraZ_val_,        0.12);
    nh_.param("num_act",         this->num_act_,            4);
    nh_.param("render_hei",      this->render_hei_,         180);
    nh_.param("render_wid",      this->render_wid_,         320);
    nh_.param("angleY_min",      this->angleY_min_,         -0.7854);
    nh_.param("angleY_max",      this->angleY_max_,         0.7854);
    nh_.param("angleZ_min",      this->angleZ_min_,         -0.4713);
    nh_.param("angleZ_max",      this->angleZ_max_,         0.4713);
    nh_.param("sense_min",       this->sense_min_,          0.01);
    nh_.param("sense_max",       this->sense_max_,          4.0);
    nh_.param("value_miss",      this->value_miss_,         0.0);
    nh_.param("range_min",       this->range_min_,          0.45);
    nh_.param("range_max",       this->range_max_,          4.0);
    nh_.param("use_inf",         this->use_inf_,            false);
    nh_.param("step_dist_val",   this->step_dist_val_,      0.3);
    nh_.param("step_deg",        this->step_deg_,           30.);
    nh_.param("reward_step",     this->reward_step_,        -0.005);
    nh_.param("reward_collision",this->reward_collision_,   -0.05);
    nh_.param("reward_reach",    this->reward_reach_,       1.);
    nh_.param("dist_reach",      this->dist_reach_,         1.);
    nh_.param("tpc_preprocess",  this->tpc_preprocess_,  	std::string("/preprocess"));
    nh_.param("tpc_render",      this->tpc_render_,  	    std::string("/render"));
    srand(this->seed_);
    this->y_noise = std::normal_distribution<double>(0., 0.01);
    this->x_noise = std::normal_distribution<double>(0., 0.01);
    this->r_noise = std::normal_distribution<double>(0., 5./180.*M_PI);
    this->render_angleY_inc_ = (this->angleY_max_ - this->angleY_min_) / this->render_wid_;
    this->render_angleZ_inc_ = (this->angleZ_max_ - this->angleZ_min_) / this->render_hei_;
    if (this->display_){
    ROS_INFO_STREAM("WID of /render min|max|inc: " << this->angleY_min_/M_PI*180. << " " << this->angleY_max_/M_PI*180. << " " << this->render_angleY_inc_/M_PI*180.);
    ROS_INFO_STREAM("HEI of /render min|max|inc: " << this->angleZ_min_/M_PI*180. << " " << this->angleZ_max_/M_PI*180. << " " << this->render_angleZ_inc_/M_PI*180.);
    }

    // 0. load in the preprocessed clouds
    this->fil_pcl_obs_ = this->dir_pcd_ + this->fil_obs_ + std::string(".pcd");
    this->fil_pcl_tgt_ = this->dir_pcd_ + this->fil_tgt_ + std::string(".pcd");
    pcl::io::loadPCDFile(this->fil_pcl_obs_, this->pcl_obs_);
    pcl::io::loadPCDFile(this->fil_pcl_tgt_, this->pcl_tgt_origin_); // the target at the origin

    // 1. parse map to get the free indices
    this->map_free_indices_.clear();
    for (int y_ind = 0; y_ind < this->map_hei_ind_; y_ind ++){
        for (int x_ind = 0; x_ind < this->map_wid_ind_; x_ind ++){
            if (upsidedown_maps[y_ind][x_ind] == 0){
                this->map_free_indices_.push_back(make_pair(y_ind, x_ind));
            }
        }
    }
    this->getValFromInd(this->map_y_min_val_,
                        this->map_x_min_val_,
                        0, 0);
    this->getValFromInd(this->map_y_max_val_,
                        this->map_x_max_val_,
                        this->map_hei_ind_-1, this->map_wid_ind_-1);
    this->map_y_min_val_ -= this->ind2val_ / 2.;
    this->map_x_min_val_ -= this->ind2val_ / 2.;
    this->map_y_max_val_ += this->ind2val_ / 2.;
    this->map_x_max_val_ += this->ind2val_ / 2.;

    // 2. bot & camera pose
    this->pos_bot_.position.x    = 0.;
    this->pos_bot_.position.y    = 0.;
    this->pos_bot_.position.z    = 0.;
    this->pos_bot_.orientation.x = 0.;
    this->pos_bot_.orientation.y = 0.;
    this->pos_bot_.orientation.z = 0.;
    this->pos_bot_.orientation.w = 1.;

    this->bot_camera_.position.x    = this->cameraX_val_;
    this->bot_camera_.position.y    = 0.;
    this->bot_camera_.position.z    = this->cameraZ_val_;
    this->bot_camera_.orientation.x = 0.;
    this->bot_camera_.orientation.y = 0.;
    this->bot_camera_.orientation.z = 0.;
    this->bot_camera_.orientation.w = 1.;

    this->pos_camera_ = this->bot_camera_;

    if (this->display_){
        // 3. tfs
        this->tf_wld_bot_.setOrigin(tf::Vector3(this->pos_bot_.position.x,
                                                this->pos_bot_.position.y,
                                                this->pos_bot_.position.z));
        this->tf_wld_bot_.setRotation(tf::Quaternion(this->pos_bot_.orientation.x,
                                                     this->pos_bot_.orientation.y,
                                                     this->pos_bot_.orientation.z,
                                                     this->pos_bot_.orientation.w));
        this->tf_bot_camera_.setOrigin(tf::Vector3(this->bot_camera_.position.x,
                                                   this->bot_camera_.position.y,
                                                   this->bot_camera_.position.z));
        this->tf_bot_camera_.setRotation(tf::Quaternion(this->bot_camera_.orientation.x,
                                                        this->bot_camera_.orientation.y,
                                                        this->bot_camera_.orientation.z,
                                                        this->bot_camera_.orientation.w));
        ROS_WARN_STREAM("Renderer Node Publishing TF " << this->frm_wld_ << " | " << this->frm_bot_);
        ROS_WARN_STREAM("Renderer Node Publishing TF " << this->frm_bot_ << " | " << this->frm_camera_);
        this->tfb_wld_bot_.sendTransform(tf::StampedTransform(this->tf_wld_bot_, ros::Time(0), this->frm_wld_, this->frm_bot_));
        this->tfb_bot_camera_.sendTransform(tf::StampedTransform(this->tf_bot_camera_, ros::Time(0), this->frm_bot_, this->frm_camera_));

        // 4. camera marker
        this->msg_mkr_camera_.ns              = "basic_shapes";
        this->msg_mkr_camera_.type            = visualization_msgs::Marker::ARROW;
        this->msg_mkr_camera_.action          = visualization_msgs::Marker::ADD;
        this->msg_mkr_camera_.header.frame_id = this->frm_wld_;
        this->msg_mkr_camera_.scale.x         = .25;
        this->msg_mkr_camera_.scale.y         = .07;
        this->msg_mkr_camera_.scale.z         = .07;
        this->msg_mkr_camera_.color.r         = 1.0;
        this->msg_mkr_camera_.color.g         = 0.0;
        this->msg_mkr_camera_.color.b         = 0.0;
        this->msg_mkr_camera_.color.a         = 1.0;
        this->msg_mkr_camera_.lifetime        = ros::Duration(0);
        this->msg_mkr_camera_.pose            = this->pos_camera_;
        this->msg_mkr_camera_.id = 0;
    }

    // 5. motion
    this->go_straight_mat_ = Eigen::Matrix4f::Identity();
    this->turn_left_mat_   = Eigen::Matrix4f::Identity();
    this->turn_right_mat_  = Eigen::Matrix4f::Identity();
    applyTranslation(this->go_straight_mat_, this->step_dist_val_, 0., 0.);
    applyRotation(this->turn_left_mat_,   deg2rad(this->step_deg_), 0, 0, 1);
    applyRotation(this->turn_right_mat_, -deg2rad(this->step_deg_), 0, 0, 1);

    // 6. rl state
    // this->msg_depth_array_.data.clear();    // TODO
    // this->msg_color_array_.data.clear();    // TODO
    this->state_.action   = this->num_act_;
    // this->state_.color    = *this->msg_color_;
    this->state_.terminal = false;
    this->state_.reward   = 0.;
}


Renderer::~Renderer(){

}


// helper funcs


bool Renderer::getIndFromAng(int& ind_y,
                             int& ind_z,
                             const double& ang_y,
                             const double& ang_z){
    ind_y = this->render_wid_ - 1 - std::floor((ang_y - this->angleY_min_) / this->render_angleY_inc_);
    ind_z = this->render_hei_ - 1 - std::floor((ang_z - this->angleZ_min_) / this->render_angleZ_inc_);

    return true;
}


bool Renderer::getIndFromVal(int& y_ind,
                             int& x_ind,
                             const double& y_val,
                             const double& x_val){
    y_ind = std::floor((y_val - (0.25)) / this->ind2val_);
    x_ind = std::floor((x_val - (-2.)) / this->ind2val_);

    return true;
}


bool Renderer::getValFromInd(double& y_val,
                             double& x_val,
                             const int& y_ind,
                             const int& x_ind){
    y_val =  0.375 + (double)y_ind * this->ind2val_;
    x_val = -1.875 + (double)x_ind * this->ind2val_;

    return true;
}


// core funcs


bool Renderer::renderImageFromPose(const geometry_msgs::Pose& pose){
    // 0. set up cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud = this->pcl_wld_;

    // 1. transform this->cloud_ the opposite direction as this->pos_bot_ to /frm_bot_
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    applyTranslation(transform, -pose.position.x, -pose.position.y, -pose.position.z);
    pcl::transformPointCloud(*cloud, *cloud, transform);    // translation
    transform = Eigen::Matrix4f::Identity();
    double _roll, _pitch, yaw;
    quaternionMsgToRPY(pose.orientation, _roll, _pitch, yaw);
    applyRotation(transform, -yaw, 0, 0, 1);
    pcl::transformPointCloud(*cloud, *cloud, transform);    // rotation

    // 2. set up this->scans_
    this->scans_ = *cloud;
    this->scans_.points.clear();

    // 3. fill in img_depth_ & img_color_: iterate through cloud
    std::map< std::pair<int, int>,  pcl::PointXYZRGB > map_depth;  // for referencing which points are actually used to render the image
    this->img_depth_ = cv::Mat(this->render_hei_, this->render_wid_, CV_32FC1, this->range_max_);
    this->img_color_ = cv::Mat::zeros(this->render_hei_, this->render_wid_, CV_8UC3);
    for (auto it = cloud->points.begin(); it != cloud->points.end(); ++ it){
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z)){ continue; }
        else{  float range = hypot(it->y, it->x);
        if (range < this->sense_min_ || range > this->sense_max_){ continue; }
        else{ double angleY = atan2(it->y, it->x);
        if (angleY < this->angleY_min_ || angleY > this->angleY_max_){ continue; }
        else{ double angleZ = atan2(it->z, it->x);
        if (angleZ < this->angleZ_min_ || angleZ > this->angleZ_max_){ continue; }
        else{
            // update the point reference for this color image pixel
            int ind_y, ind_z;
            this->getIndFromAng(ind_y, ind_z, angleY, angleZ);
            // blur
            int blur_edg = 2;
            for (int z = max(0, ind_z - blur_edg); z <= min(this->render_hei_ - 1, ind_z + blur_edg); ++ z){
                for (int y = max(0, ind_y - blur_edg); y <= min(this->render_wid_ - 1, ind_y + blur_edg); ++ y){
                    if (range < this->img_depth_.at<float>(z, y)){
                        if (map_depth.find(std::make_pair(ind_y, ind_z)) != map_depth.end()){
                            // already exists, need to first erase the old one
                            map_depth.erase(std::make_pair(ind_y, ind_z));
                        }
                        map_depth.insert(std::make_pair(std::make_pair(ind_y, ind_z), *it));
                        if (range < this->range_min_ || range > this->range_max_){
                            this->img_depth_.at<float>(z, y) = this->value_miss_;
                        }
                        else{
                            this->img_depth_.at<float>(z, y) = range;
                            pcl::PointXYZRGB point = *it;
                            uint32_t rgb = *reinterpret_cast<int*>(&point.rgb);
                            this->img_color_.ptr<uint8_t>(z, y)[0] = (rgb >> 16) & 0x0000ff;
                            this->img_color_.ptr<uint8_t>(z, y)[1] = (rgb >> 8) & 0x0000ff;
                            this->img_color_.ptr<uint8_t>(z, y)[2] = (rgb) & 0x0000ff;
                        }
                    }
                }
            }
            // no blur
            // if (range < this->img_depth_.at<float>(ind_z, ind_y)){
            //     this->img_depth_.at<float>(ind_z, ind_y) = range;
            // }
        }}}}
    }
    // insert only points that are actually used in calculating color into scans_
    for (auto it = map_depth.begin(); it != map_depth.end(); ++ it){
        this->scans_.points.push_back(it->second);
    }
    this->scans_.width = this->scans_.points.size();

    // 4. fill in msg_depth_ & msg_color_
    this->msg_depth_ = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, this->img_depth_).toImageMsg();
    // this->msg_depth_array_.data.clear();
    this->msg_color_ = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, this->img_color_).toImageMsg();
    // this->msg_color_array_.data.clear();
    // for (int z = 0; z < this->render_hei_; ++ z){
        // for (int y = 0; y < this->render_wid_; ++ y){
            // this->msg_depth_array_.data.push_back(this->img_depth_.at<float>(z, y));
            // this->msg_color_array_.data.push_back(this->img_color_.at<float>(z, y));
        // }
    // }

    // 5. publish this->msg_scans_
    if (this->display_){
        ROS_INFO_STREAM("#Points in Scans for Color " << this->scans_.points.size());
        pcl::PointXYZRGB pcWldMinPt, pcWldMaxPt;
        pcl::getMinMax3D(this->scans_, pcWldMinPt, pcWldMaxPt);
        ROS_INFO_STREAM("Min     in Scans for Render " << pcWldMinPt.x << " " << pcWldMinPt.y << " " << pcWldMinPt.z);
        ROS_INFO_STREAM("Max     in Scans for Render " << pcWldMaxPt.x << " " << pcWldMaxPt.y << " " << pcWldMaxPt.z);
        pcl::toROSMsg(this->scans_, this->msg_scans_);
        ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_scans_);
        this->msg_scans_.header.frame_id = this->frm_camera_;
        this->msg_scans_.header.stamp    = ros::Time::now();
        this->pub_scans_.publish(this->msg_scans_);
        // 6. publish this->msg_depth_
        ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_depth_);
        this->pub_depth_.publish(this->msg_depth_);
        // ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_depth_array_);
        // this->pub_depth_array_.publish(this->msg_depth_array_);
        // 7. publish this->msg_color_
        ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_color_);
        this->pub_color_.publish(this->msg_color_);
        // ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_color_array_);
        // this->pub_color_array_.publish(this->msg_color_array_);
    }

    // 8. wrap up
    cloud = NULL;
    return true;
}


bool Renderer::act(const int& act_ind){
    this->state_.action = act_ind;
    // NOTE: here we didn't implement an actual motion model
    // NOTE: so the way we add in noise is a bit hacky
    // tentatively execute the action, in val
    double _roll, _pitch, yaw;
    double x_val, y_val;
    double new_x_val, new_y_val, new_yaw;
    quaternionMsgToRPY(this->pos_bot_.orientation, _roll, _pitch, yaw);
    x_val = this->pos_bot_.position.x;
    y_val = this->pos_bot_.position.y;
    switch (act_ind){
        case 0: // go straight
            new_yaw   = yaw + this->r_noise(this->r_noise_gen);
            new_x_val = x_val + this->step_dist_val_ * cos(yaw) + this->x_noise(this->x_noise_gen);
            new_y_val = y_val + this->step_dist_val_ * sin(yaw) + this->y_noise(this->y_noise_gen);
        break;
        case 1: // turn left
            new_yaw   = yaw + deg2rad(this->step_deg_) + this->r_noise(this->r_noise_gen);
            new_x_val = x_val + this->x_noise(this->x_noise_gen);
            new_y_val = y_val + this->y_noise(this->y_noise_gen);
        break;
        case 2: // turn right
            new_yaw   = yaw - deg2rad(this->step_deg_) + this->r_noise(this->r_noise_gen);
            new_x_val = x_val + this->x_noise(this->x_noise_gen);
            new_y_val = y_val + this->y_noise(this->y_noise_gen);
        break;
        case 3: // do nothing, only called in reset()
            new_yaw   = yaw;
            new_x_val = x_val;
            new_y_val = y_val;
        break;
    }
    // collision checking, in ind
    int y_ind, x_ind;
    this->getIndFromVal(y_ind, x_ind, new_y_val, new_x_val);
    if (y_ind < 0 || y_ind >= this->map_hei_ind_ || x_ind < 0 || x_ind >= this->map_wid_ind_){ // then stay at the original place
        this->state_.terminal = false;
        this->state_.reward = this->reward_step_;
        this->pos_camera_.position.x = x_val + this->cameraX_val_ * cos(yaw);
        this->pos_camera_.position.y = y_val + this->cameraX_val_ * sin(yaw);
    }
    else{
        if (upsidedown_maps[y_ind][x_ind] == 1){    // collision
            this->state_.terminal = false;
            this->state_.reward = this->reward_collision_;
            this->pos_camera_.position.x = x_val + this->cameraX_val_ * cos(yaw);
            this->pos_camera_.position.y = y_val + this->cameraX_val_ * sin(yaw);
        }
        else{
            if (getDistance(y_val, x_val, this->tgt_y_val_, this->tgt_x_val_) < this->dist_reach_){ // reaches target
                this->state_.terminal = true;
                this->state_.reward = this->reward_reach_;
            }
            else{
                this->state_.terminal = false;
                this->state_.reward = this->reward_step_;
            }
            this->pos_bot_.position.x = new_x_val;
            this->pos_bot_.position.y = new_y_val;
            this->pos_bot_.orientation = tf::createQuaternionMsgFromRollPitchYaw(_roll, _pitch, new_yaw);
            this->pos_camera_.position.x = new_x_val + this->cameraX_val_ * cos(new_yaw);
            this->pos_camera_.position.y = new_y_val + this->cameraX_val_ * sin(new_yaw);
        }
    }
    // update pos_camera_, in val
    this->pos_camera_.position.z = this->cameraZ_val_;
    this->pos_camera_.orientation = this->pos_bot_.orientation;

    return true;
}


bool Renderer::reset(){
    // 0. randomly sample grids for target and bot
    std::random_shuffle(&this->map_free_indices_[0], &this->map_free_indices_[this->map_free_indices_.size()]);
    double bot_y_val, bot_x_val;
    this->getValFromInd(bot_y_val, bot_x_val,
                        this->map_free_indices_[0].first,
                        this->map_free_indices_[0].second);
    this->getValFromInd(this->tgt_y_val_, this->tgt_x_val_,
                        this->map_free_indices_[1].first,
                        this->map_free_indices_[1].second);

    // 1. put bot to the sampled pose
    this->pos_bot_.position.x  = bot_x_val;
    this->pos_bot_.position.y  = bot_y_val;
    this->pos_bot_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., randomRadian());

    // 2. put target to the sampled pose
    Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();
    applyRotation(transform_mat, randomRadian(), 0, 0, 1);
    pcl::transformPointCloud(this->pcl_tgt_origin_, this->pcl_tgt_, transform_mat); // translation
    applyTranslation(transform_mat, this->tgt_x_val_, this->tgt_y_val_, this->tgtZ_val_);
    pcl::transformPointCloud(this->pcl_tgt_, this->pcl_tgt_, transform_mat);        // rotation

    // 3. put obs & tgt together to form one single cloud & update collision map
    this->pcl_wld_.points.clear();
    this->pcl_wld_ = this->pcl_obs_;
    for (auto it = this->pcl_tgt_.points.begin(); it != this->pcl_tgt_.end(); ++ it){
        this->pcl_wld_.points.push_back(*it);
    }
    this->pcl_wld_.width = this->pcl_wld_.points.size();

    // 4. do a normal step
    this->step(this->num_act_);

    // 5. display wld cloud
    if (this->display_){
        pcl::PointXYZRGB min_pcl_wld;
        pcl::PointXYZRGB max_pcl_wld;
        pcl::getMinMax3D(this->pcl_wld_, min_pcl_wld, max_pcl_wld);
        ROS_INFO_STREAM("Min /pcl_wld: " << min_pcl_wld.x << " " << min_pcl_wld.y << " " << min_pcl_wld.z);
        ROS_INFO_STREAM("Max /pcl_wld: " << max_pcl_wld.x << " " << max_pcl_wld.y << " " << max_pcl_wld.z);
        ROS_INFO_STREAM("#Points in /pcl_wld: " << this->pcl_wld_.points.size());
      	ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_pcl_wld_);
        pcl::toROSMsg(this->pcl_wld_, this->msg_pcl_wld_);
        this->msg_pcl_wld_.header.frame_id = this->frm_wld_;
      	this->pub_pcl_wld_.publish(this->msg_pcl_wld_);
        ROS_ERROR_STREAM("Renderer Node Responded  To " << " Render.");
    }

    return true;
}


bool Renderer::step(const int& act_ind){
    // 1. get the new pose & reward+terminal
    this->act(act_ind);
    // 2. render the color image from the new pose
    // 2.1. update tf
    if (this->display_){
        this->tf_wld_bot_.setOrigin(tf::Vector3(this->pos_bot_.position.x,
                                                this->pos_bot_.position.y,
                                                this->pos_bot_.position.z));
        this->tf_wld_bot_.setRotation(tf::Quaternion(this->pos_bot_.orientation.x,
                                                     this->pos_bot_.orientation.y,
                                                     this->pos_bot_.orientation.z,
                                                     this->pos_bot_.orientation.w));
        this->tf_bot_camera_.setOrigin(tf::Vector3(this->bot_camera_.position.x,
                                                   this->bot_camera_.position.y,
                                                   this->bot_camera_.position.z));
        this->tf_bot_camera_.setRotation(tf::Quaternion(this->bot_camera_.orientation.x,
                                                        this->bot_camera_.orientation.y,
                                                        this->bot_camera_.orientation.z,
                                                        this->bot_camera_.orientation.w));
        ROS_WARN_STREAM("Renderer Node Publishing TF " << this->frm_wld_ << " | " << this->frm_bot_);
        ROS_WARN_STREAM("Renderer Node Publishing TF " << this->frm_bot_ << " | " << this->frm_camera_);
        // this->tfb_wld_bot_.sendTransform(tf::StampedTransform(this->tf_wld_bot_, ros::Time(0), this->frm_wld_, this->frm_bot_));
        // this->tfb_bot_camera_.sendTransform(tf::StampedTransform(this->tf_bot_camera_, ros::Time(0), this->frm_bot_, this->frm_camera_));
        this->tfb_wld_bot_.sendTransform(tf::StampedTransform(this->tf_wld_bot_, ros::Time::now(), this->frm_wld_, this->frm_bot_));
        this->tfb_bot_camera_.sendTransform(tf::StampedTransform(this->tf_bot_camera_, ros::Time::now(), this->frm_bot_, this->frm_camera_));
      	ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_mkr_camera_);
        this->msg_mkr_camera_.pose            = this->pos_camera_;
        this->msg_mkr_camera_.header.frame_id = this->frm_wld_;
        this->msg_mkr_camera_.header.stamp    = ros::Time::now();
        this->pub_mkr_camera_.publish(this->msg_mkr_camera_);
    }
    // 2.2. update msg_depth_ & msg_color_ : render images from this->pos_camera_
    this->renderImageFromPose(this->pos_camera_);

    return true;
}


// callbacks


bool Renderer::preprocessCallback(renderer::Preprocess::Request&  req,
				                  renderer::Preprocess::Response& res){
    if (this->display_){
        ROS_ERROR_STREAM("Renderer Node Responding To " << this->tpc_preprocess_ << " ...");
    }
    // 0. load the original clouds
    this->fil_pcl_obs_ = this->dir_dat_ + this->fil_obs_ + std::string(".pcd");
    this->fil_pcl_tgt_ = this->dir_dat_ + this->fil_tgt_ + std::string(".pcd");
    pcl::io::loadPCDFile(this->fil_pcl_obs_, this->pcl_obs_);
    pcl::io::loadPCDFile(this->fil_pcl_tgt_, this->pcl_tgt_origin_);

    // 1. preprocess pcl_obs to align with main axis
    pcl::PointXYZRGB min_pcl_obs;
    pcl::PointXYZRGB max_pcl_obs;
    Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();
    // 1.1 translation
    pcl::getMinMax3D(this->pcl_obs_, min_pcl_obs, max_pcl_obs);
    applyTranslation(transform_mat,
                     -(min_pcl_obs.x + max_pcl_obs.x) / 2.,
                     -(min_pcl_obs.y + max_pcl_obs.y) / 2.,
                     -(min_pcl_obs.z + max_pcl_obs.z) / 2.);
    pcl::transformPointCloud(this->pcl_obs_, this->pcl_obs_, transform_mat); // translation
    // 1.1.1. rotatation
    applyRotation(transform_mat, M_PI / 180. * 254.1, 1, 0, 0);
    pcl::transformPointCloud(this->pcl_obs_, this->pcl_obs_, transform_mat); // rotation
    applyRotation(transform_mat, M_PI / 180. * 357.3, 0, 1, 0);
    pcl::transformPointCloud(this->pcl_obs_, this->pcl_obs_, transform_mat); // rotation
    applyRotation(transform_mat, M_PI / 180. * 356, 0, 0, 1);
    pcl::transformPointCloud(this->pcl_obs_, this->pcl_obs_, transform_mat); // rotation
    // 1.1.3 translation
    transform_mat = Eigen::Matrix4f::Identity();
    pcl::getMinMax3D(this->pcl_obs_, min_pcl_obs, max_pcl_obs);
    applyTranslation(transform_mat,
                     -(min_pcl_obs.x + max_pcl_obs.x) / 2.,
                     -(min_pcl_obs.y + max_pcl_obs.y) / 2.,
                     this->floorZ_val_ - min_pcl_obs.z);
    pcl::transformPointCloud(this->pcl_obs_, this->pcl_obs_, transform_mat);// translation

    // 2. preprocess pcl_tgt to align with main axis
    pcl::PointXYZRGB min_pcl_tgt;
    pcl::PointXYZRGB max_pcl_tgt;
    transform_mat = Eigen::Matrix4f::Identity();
    // 2.1 translation
    pcl::getMinMax3D(this->pcl_tgt_origin_, min_pcl_tgt, max_pcl_tgt);
    applyTranslation(transform_mat,
                     -(min_pcl_tgt.x + max_pcl_tgt.x) / 2.,
                     -(min_pcl_tgt.y + max_pcl_tgt.y) / 2.,
                     -(min_pcl_tgt.z + max_pcl_tgt.z) / 2.);
    pcl::transformPointCloud(this->pcl_tgt_origin_, this->pcl_tgt_origin_, transform_mat); // translation
    // 2.1.1. rotatation
    applyRotation(transform_mat, M_PI / 180. * 230., 1, 0, 0);
    pcl::transformPointCloud(this->pcl_tgt_origin_, this->pcl_tgt_origin_, transform_mat); // rotation
    applyRotation(transform_mat, M_PI / 180. * 355., 0, 1, 0);
    pcl::transformPointCloud(this->pcl_tgt_origin_, this->pcl_tgt_origin_, transform_mat); // rotation
    applyRotation(transform_mat, M_PI / 180. * 64.,  0, 0, 1);
    pcl::transformPointCloud(this->pcl_tgt_origin_, this->pcl_tgt_origin_, transform_mat); // rotation
    // 2.1.3 translation
    transform_mat = Eigen::Matrix4f::Identity();
    pcl::getMinMax3D(this->pcl_tgt_origin_, min_pcl_tgt, max_pcl_tgt);
    applyTranslation(transform_mat,
                     -(min_pcl_tgt.x + max_pcl_tgt.x) / 2.,
                     -(min_pcl_tgt.y + max_pcl_tgt.y) / 2.,
                     this->floorZ_val_ - min_pcl_tgt.z);
    pcl::transformPointCloud(this->pcl_tgt_origin_, this->pcl_tgt_origin_, transform_mat);// translation

    // 3. save the aligned clouds
    this->fil_pcl_obs_ = this->dir_pcd_ + this->fil_obs_ + std::string(".pcd");
    this->fil_pcl_tgt_ = this->dir_pcd_ + this->fil_tgt_ + std::string(".pcd");
  	ROS_WARN_STREAM("Renderer Node Saving     To " << this->fil_pcl_obs_ << " ...");
    // pcl::io::savePCDFileASCII(this->fil_pcl_obs_, this->pcl_obs_);           # NOTE: this gives red patched
    pcl::io::savePCDFileBinary(this->fil_pcl_obs_, this->pcl_obs_);
  	ROS_WARN_STREAM("Renderer Node Saved      To " << this->fil_pcl_obs_ << ".");
  	ROS_WARN_STREAM("Renderer Node Saving     To " << this->fil_pcl_tgt_ << " ...");
    // pcl::io::savePCDFileASCII(this->fil_pcl_tgt_, this->pcl_tgt_origin_);    # NOTE: this gives red patches !!!
    pcl::io::savePCDFileBinary(this->fil_pcl_tgt_, this->pcl_tgt_origin_);
  	ROS_WARN_STREAM("Renderer Node Saved      To " << this->fil_pcl_tgt_ << ".");

    if (this->display_){
        // 3. recover stats
        pcl::getMinMax3D(this->pcl_obs_, min_pcl_obs, max_pcl_obs);
        ROS_INFO_STREAM("Min /pcl_obs: " << min_pcl_obs.x << " " << min_pcl_obs.y << " " << min_pcl_obs.z);
        ROS_INFO_STREAM("Max /pcl_obs: " << max_pcl_obs.x << " " << max_pcl_obs.y << " " << max_pcl_obs.z);
        ROS_INFO_STREAM("#Points in /pcl_obs: " << this->pcl_obs_.points.size());
        ROS_INFO_STREAM("#Points in /pcl_tgt: " << this->pcl_tgt_origin_.points.size());
        // 4. prepare the message
        pcl::toROSMsg(this->pcl_obs_, this->msg_pcl_obs_);
        pcl::toROSMsg(this->pcl_tgt_origin_, this->msg_pcl_tgt_);
        this->msg_pcl_obs_.header.frame_id = this->frm_wld_;
        this->msg_pcl_tgt_.header.frame_id = this->frm_wld_;
        // 5. publish
      	ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_pcl_obs_);
      	this->pub_pcl_obs_.publish(this->msg_pcl_obs_);
      	ROS_WARN_STREAM("Renderer Node Publishing To " << this->tpc_pcl_tgt_);
      	this->pub_pcl_tgt_.publish(this->msg_pcl_tgt_);

        ROS_ERROR_STREAM("Renderer Node Responded  To " << " Preprocess.");
    }

    return true;
}


bool Renderer::renderCallback(renderer::Render::Request&  req,
			                  renderer::Render::Response& res){
    if (this->display_){
        ROS_ERROR_STREAM("Renderer Node Responding To " << this->tpc_render_ << " | " << req.action << " ...");
    }

    int tmp_y_ind;
    int tmp_x_ind;
    double tmp_y_val;
    double tmp_x_val;
    // 1. execute the action
    if (req.action == 3){
        this->reset();
    }
    else{
        this->step(req.action);
    }
    // TODO: put msg_depth_ if want to use depth instead
    res.color = *this->msg_color_;
    res.terminal = this->state_.terminal;
    res.reward = this->state_.reward;

    return true;
}


// init


void Renderer::initROS(){
    if (this->display_){
        ROS_ERROR_STREAM("Renderer Node Registered.");
    }

    this->pub_pcl_obs_     = this->nh_.advertise<sensor_msgs::PointCloud2>(this->tpc_pcl_obs_, 1);
    this->pub_pcl_tgt_     = this->nh_.advertise<sensor_msgs::PointCloud2>(this->tpc_pcl_tgt_, 1);
    this->pub_pcl_wld_     = this->nh_.advertise<sensor_msgs::PointCloud2>(this->tpc_pcl_wld_, 1);
    this->pub_scans_       = this->nh_.advertise<sensor_msgs::PointCloud2>(this->tpc_scans_, 1);
    this->pub_color_       = this->it_.advertise(this->tpc_color_, 1);
    // this->pub_color_array_ = this->nh_.advertise<std_msgs::Float32MultiArray>(this->tpc_color_array_, 1);
    this->pub_depth_       = this->it_.advertise(this->tpc_depth_, 1);
    // this->pub_depth_array_ = this->nh_.advertise<std_msgs::Float32MultiArray>(this->tpc_depth_array_, 1);
    this->pub_mkr_camera_  = this->nh_.advertise<visualization_msgs::Marker>(this->tpc_mkr_camera_, 1);
    // NOTE: the preprocess service is meant to be called only once
    this->srv_preprocess_  = this->nh_.advertiseService(this->tpc_preprocess_, &Renderer::preprocessCallback, this);
    // NOTE: render rgb for interactive rl envs
    this->srv_render_      = this->nh_.advertiseService(this->tpc_render_,     &Renderer::renderCallback,     this);
}


} // end of namespace
