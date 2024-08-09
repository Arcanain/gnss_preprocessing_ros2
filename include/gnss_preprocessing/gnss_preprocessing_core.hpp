#ifndef __GNSS_PREPROCESSING_CORE_HPP__
#define __GNSS_PREPROCESSING_CORE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>

using namespace std;

class GnssPreprocessingCore : public rclcpp::Node
{
    public:
        GnssPreprocessingCore(double lat, double lon, double hig);
        ~GnssPreprocessingCore();

        double pi = 3.1415926535898;
        double a = 6378137.0;
        double ONE_F = 298.257223563;
        double E2 = ((1.0/ONE_F)*(2-(1.0/ONE_F)));

    private:
        // Local variables
        double lat0;
        double lon0;
        double hig0;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gnss_path_pub;

        // Publish data
        nav_msgs::msg::Path gnss_path;

        // Subscriber
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub;

        // TF Broadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> odom_to_baselink_broadcaster;
        geometry_msgs::msg::TransformStamped odom_to_baselink_trans;

        // Callback function
        void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg);
        
        // Conversion functions
        double deg2rad(double deg); // Convert degrees to radians
        double rad2deg(double rad); // Convert radians to degrees
        Eigen::Vector3d blh2ecef(double p_deg, double l_deg, double h);
        Eigen::Vector3d ecef2blh(double x, double y, double z);
        Eigen::Vector3d ecef2enu(Eigen::Vector3d dest, Eigen::Vector3d origin);
        Eigen::Matrix3d rotx(double theta);
        Eigen::Matrix3d roty(double theta);
        Eigen::Matrix3d rotz(double theta);

        // Different version conversion functions
        int blh2enu(double lat0, double lon0, double lat, double lon, double *x, double *y);
        double constrain(double val, double min, double max);
};

#endif  // __GNSS_PREPROCESSING_CORE_HPP__

