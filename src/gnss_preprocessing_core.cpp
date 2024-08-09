#include "gnss_preprocessing/gnss_preprocessing_core.hpp"

GnssPreprocessingCore::GnssPreprocessingCore(double lat, double lon, double hig)
: Node("gnss_preprocessing_core"), lat0(lat), lon0(lon), hig0(hig)
{
    RCLCPP_INFO(this->get_logger(), "GnssPreprocessingCore Start!");

    // Publisher
    gnss_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss_pose", 10);
    gnss_path_pub = this->create_publisher<nav_msgs::msg::Path>("/gnss_path", 10);

    // Subscriber
    gnss_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ublox/fix", 10, std::bind(&GnssPreprocessingCore::gnssCallback, this, std::placeholders::_1)
    );

    // Initialize gnss_path
    gnss_path.header.frame_id = "map";
    gnss_path.header.stamp = this->now();

    // Initialize Transform Broadcaster
    odom_to_baselink_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

GnssPreprocessingCore::~GnssPreprocessingCore()
{
    RCLCPP_INFO(this->get_logger(), "GnssPreprocessingCore Finish");
}

void GnssPreprocessingCore::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gnss_msg)
{
    // Convert lat/lon/height to ECEF
    Eigen::Vector3d ecef_origin = blh2ecef(lat0, lon0, hig0);
    Eigen::Vector3d ecef = blh2ecef(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
    Eigen::Vector3d enu = ecef2enu(ecef, ecef_origin);

    geometry_msgs::msg::PoseStamped point;
    point.header.frame_id = "map";
    point.header.stamp = this->now();
    point.pose.position.x = enu[0];
    point.pose.position.y = enu[1];
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 0;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 1.0;
    
    gnss_path.poses.push_back(point);
    gnss_pose_pub->publish(point);
    gnss_path_pub->publish(gnss_path);

    // /odom to /base_link transform broadcast
    geometry_msgs::msg::TransformStamped odom_to_baselink_trans;
    odom_to_baselink_trans.header.stamp = this->now();
    odom_to_baselink_trans.header.frame_id = "odom";
    odom_to_baselink_trans.child_frame_id = "base_link";
    odom_to_baselink_trans.transform.translation.x = enu[0];
    odom_to_baselink_trans.transform.translation.y = enu[1];
    odom_to_baselink_trans.transform.translation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.x = 0.0;
    odom_to_baselink_trans.transform.rotation.y = 0.0;
    odom_to_baselink_trans.transform.rotation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.w = 1.0;
    odom_to_baselink_broadcaster->sendTransform(odom_to_baselink_trans);
}

/***********************************************************************
 * Convert (lat, lon, alt) -> (x, y, z) 
 **********************************************************************/
Eigen::Vector3d GnssPreprocessingCore::blh2ecef(double p_deg, double l_deg, double h) {
    double p_rad = deg2rad(p_deg);
    double l_rad = deg2rad(l_deg);

    double N = (a / sqrt(1.0 - E2 * pow(sin(p_rad), 2)));

    double x = (N + h) * cos(p_rad) * cos(l_rad);
    double y = (N + h) * cos(p_rad) * sin(l_rad);
    double z = (N * (1 - E2) + h) * sin(p_rad);

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d GnssPreprocessingCore::ecef2blh(double x, double y, double z) {
    double b = (a * (1.0 - 1.0 / ONE_F));
    double ED2 = (E2 * pow(a, 2) / (pow(b, 2)));
    double p = sqrt(pow(x, 2) + pow(y, 2));
    double si = atan2(z * a, p * b);

    double p_rad2 = atan2((z + ED2 * b * pow(sin(si), 3)), (p - E2 * a * pow(cos(si), 3)));
    double l_rad2 = atan2(y, x);

    double lat = rad2deg(p_rad2);
    double lon = rad2deg(l_rad2);

    double N = (a / sqrt(1.0 - E2 * pow(sin(p_rad2), 2)));
    double hig = (p / cos(p_rad2)) - N;

    return Eigen::Vector3d(lat, lon, hig);
}

Eigen::Vector3d GnssPreprocessingCore::ecef2enu(Eigen::Vector3d dest, Eigen::Vector3d origin) {
    Eigen::Vector3d blh = ecef2blh(origin(0), origin(1), origin(2));
    
    Eigen::Matrix3d rotzp1 = rotz(90.0);
    Eigen::Matrix3d rotyp = roty(90.0 - blh(0));
    Eigen::Matrix3d rotzp2 = rotz(blh(1));

    Eigen::Matrix3d mat_conv1 = rotzp1 * rotyp;
    Eigen::Matrix3d mat_conv2 = mat_conv1 * rotzp2;

    Eigen::Vector3d mov = dest - origin;

    return mat_conv2 * mov;
}

Eigen::Matrix3d GnssPreprocessingCore::rotx(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotxa;
    rotxa << 1, 0, 0,
             0, cos(rad), sin(rad),
             0, -sin(rad), cos(rad);
    return rotxa;
}

Eigen::Matrix3d GnssPreprocessingCore::roty(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotya;
    rotya << cos(rad), 0, -sin(rad),
             0, 1, 0,
             sin(rad), 0, cos(rad);
    return rotya;
}

Eigen::Matrix3d GnssPreprocessingCore::rotz(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotza;
    rotza << cos(rad), sin(rad), 0,
             -sin(rad), cos(rad), 0,
             0, 0, 1;
    return rotza;
}

// Convert degrees to radians
double GnssPreprocessingCore::deg2rad(double deg) {
    return deg * pi / 180.0;
}

// Convert radians to degrees
double GnssPreprocessingCore::rad2deg(double rad) {
    return rad * 180.0 / pi;
}

/***********************************************************************
 * Convert (lat, lon, alt) -> (x, y, z) 
 **********************************************************************/
int GnssPreprocessingCore::blh2enu(double lat0, double lon0, double lat, double lon, double *x, double *y)
{
    static constexpr double CONSTANTS_RADIUS_OF_EARTH = 6371000; //[m]

    const double lat0_rad = lat0 * (M_PI / 180.0);
    const double lon0_rad = lon0 * (M_PI / 180.0);
    const double lat_rad = lat * (M_PI / 180.0);
    const double lon_rad = lon * (M_PI / 180.0);
    
    const double sin_lat0 = sin(lat0_rad);
    const double cos_lat0 = cos(lat0_rad);
    const double sin_lat = sin(lat_rad);
    const double cos_lat = cos(lat_rad);
    
    const double cos_d_lon = cos(lon_rad - lon0_rad);
    
    const double arg = constrain(sin_lat0 * sin_lat + cos_lat0 * cos_lat * cos_d_lon, -1.0,  1.0);
    const double c = acos(arg);
    
    double k = 1.0;

    if (fabs(c) > 0) {
        k = (c / sin(c));
    }

    *x = static_cast<double>(k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
    *y = static_cast<double>(k * cos_lat * sin(lon_rad - lon0_rad) * CONSTANTS_RADIUS_OF_EARTH);

    return 0;
}

double GnssPreprocessingCore::constrain(double val, double min, double max)
{
    if (val > max) {
        return max;
    } else if (val < min) {
        return min;
    } else {
        return val;
    }
}

