#ifndef UTILS_H_
#define UTILS_H_

// basic
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <string>
#include <vector>
#include <algorithm>
#include <random>

// ros
#include <ros/ros.h>
// cv
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
// messages & services
#include <std_srvs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
// tf
#include <tf/transform_broadcaster.h>
// bag
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// namespace
using namespace std;


namespace renderer{


struct state {
    int                action;
    sensor_msgs::Image color;
    float              reward;
    bool               terminal;
};


bool static quaternionMsgToRPY(const geometry_msgs::Quaternion& quaternion,
                               double& roll,
                               double& pitch,
                               double& yaw){
    tf::Quaternion tmp_quaternion(quaternion.x,
                                  quaternion.y,
                                  quaternion.z,
                                  quaternion.w);
    tf::Matrix3x3 m(tmp_quaternion);
    m.getRPY(roll, pitch, yaw);

    return true;
}


bool static applyTranslation(Eigen::Matrix4f& transform_trans,
                             const double& x,
                             const double& y,
                             const double& z,
                             bool reset=true){
    if (reset){transform_trans = Eigen::Matrix4f::Identity();}
    transform_trans(0, 3) = x;
    transform_trans(1, 3) = y;
    transform_trans(2, 3) = z;

    return true;
}


bool static applyRotation(Eigen::Matrix4f& transform_rotat,
                          double theta,
                          int l,
                          int m,
                          int n,
                          bool reset=true){
    if (reset){transform_rotat = Eigen::Matrix4f::Identity();}
    transform_rotat(0, 0) = l*l*(1-cos(theta)) +   cos(theta);
    transform_rotat(0, 1) = m*l*(1-cos(theta)) - n*sin(theta);
    transform_rotat(0, 2) = n*l*(1-cos(theta)) + m*sin(theta);
    transform_rotat(1, 0) = l*m*(1-cos(theta)) + n*sin(theta);
    transform_rotat(1, 1) = m*m*(1-cos(theta)) +   cos(theta);
    transform_rotat(1, 2) = n*m*(1-cos(theta)) - l*sin(theta);
    transform_rotat(2, 0) = l*n*(1-cos(theta)) - m*sin(theta);
    transform_rotat(2, 1) = m*n*(1-cos(theta)) + l*sin(theta);
    transform_rotat(2, 2) = n*n*(1-cos(theta)) +   cos(theta);

    return true;
}


double static randomRadian(){
    return rand() / double(RAND_MAX) * 2. * M_PI;
}


// NOTE: navive version w/o wrapping around
double static deg2rad(const double& deg){
    return deg * M_PI / 180.;
}


// NOTE: navive version w/o wrapping around
double static rad2deg(const double& rad){
    return rad / M_PI * 180.;
}


double static getDistance(const double& y_0,
                          const double& x_0,
                          const double& y_1,
                          const double& x_1){
    return sqrt(pow(y_0 - y_1, 2.0) + pow(x_0 - x_1, 2.0));
}


double static getRelativeRadian(){

}


} // end of namespace


#endif
