#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <vector>
#include <cmath>

class SaveGnssPath : public rclcpp::Node
{
public:
    SaveGnssPath() : Node("save_gnss_path"), pre_x(0.0), pre_y(0.0), dist_thread(0.01)
    {
        // Subscriber to /gnss_pose topic
        gnss_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&SaveGnssPath::gnss_callback, this, std::placeholders::_1));

        // Initialize CSV file
        output_file_.open("/home/pei/ros2_ws/src/gnss_preprocessing/test_0809.csv");
        output_file_ << "x,y,z,w0,w1,w2,w3\n";
    }

    ~SaveGnssPath()
    {
        // Close the file when done
        if (output_file_.is_open())
        {
            output_file_.close();
        }
    }

private:
    void gnss_callback(const geometry_msgs::msg::PoseStamped::SharedPtr gnss_msg)
    {
        double dist = std::sqrt(std::pow(gnss_msg->pose.position.x - pre_x, 2) + std::pow(gnss_msg->pose.position.y - pre_y, 2));

        if (dist >= dist_thread)
        {
            // Save current point to CSV
            output_file_ << gnss_msg->pose.position.x << ","
                         << gnss_msg->pose.position.y << ","
                         << gnss_msg->pose.position.z << ","
                         << gnss_msg->pose.orientation.x << ","
                         << gnss_msg->pose.orientation.y << ","
                         << gnss_msg->pose.orientation.z << ","
                         << gnss_msg->pose.orientation.w << "\n";

            // Update previous position
            pre_x = gnss_msg->pose.position.x;
            pre_y = gnss_msg->pose.position.y;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;
    std::ofstream output_file_;
    double pre_x, pre_y;
    double dist_thread;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SaveGnssPath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

