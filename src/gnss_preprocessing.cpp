#include <rclcpp/rclcpp.hpp>
#include "gnss_preprocessing/gnss_preprocessing_core.hpp"
//#include "gnss_preprocessing_component.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Initialize GPS position (latitude, longitude)
    double lat0 = 36.082887899999996;
    double lon0 = 140.0769054;
    double hig0 = 0.0;

    // Create a node using GNSS Preprocessing Core
    auto gnss_preprocessing_core = std::make_shared<GnssPreprocessingCore>(lat0, lon0, hig0);

    // Spin node (handles callbacks)
    rclcpp::spin(gnss_preprocessing_core);

    // Shutdown ROS2
    rclcpp::shutdown();

    return 0;
}

