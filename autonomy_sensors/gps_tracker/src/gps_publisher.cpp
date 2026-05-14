#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono>
#include <vector>
#include <array>

using namespace std::chrono_literals;

class GPSPublisher : public rclcpp::Node
{
public:
    GPSPublisher() : Node("gps_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

        timer_ = this->create_wall_timer(
            1s,
            std::bind(&GPSPublisher::publish_gps, this));
    }

private:
    void publish_gps()
    {
        sensor_msgs::msg::NavSatFix msg;

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "gps";

        msg.latitude = path_[index_][0];
        msg.longitude = path_[index_][1];
        msg.altitude = 100.0;

        publisher_->publish(msg);

        if (index_ < path_.size() - 1)
        {
            index_++;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t index_ = 0;

    std::vector<std::array<double,2>> path_ = {
        {43.656050,-79.380280},
        {43.656080,-79.380200},
        {43.656120,-79.380100},
        {43.656200,-79.380050},
        {43.656260,-79.379980},
        {43.656300,-79.379900},
        {43.656350,-79.379820},
        {43.656400,-79.379750}
    };
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GPSPublisher>());
    rclcpp::shutdown();
    return 0;
}