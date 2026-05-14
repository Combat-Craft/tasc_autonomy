#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class RouteLogger : public rclcpp::Node
{
public:
    RouteLogger() : Node("route_logger")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix",
            10,
            std::bind(&RouteLogger::gps_callback,this,std::placeholders::_1)
        );

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/gps/path",10);

        path_msg_.header.frame_id = "map";
    }

private:

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if(!origin_set_)
        {
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            origin_set_ = true;
        }

        double x,y;
        latlon_to_xy(msg->latitude,msg->longitude,x,y);

        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0;

        path_msg_.header.stamp = this->now();
        path_msg_.poses.push_back(pose);

        publisher_->publish(path_msg_);
    }

    void latlon_to_xy(double lat,double lon,double &x,double &y)
    {
        constexpr double R = 6378137.0;

        double dlat = (lat-origin_lat_) * M_PI/180.0;
        double dlon = (lon-origin_lon_) * M_PI/180.0;
        double mean_lat = ((lat+origin_lat_)/2.0) * M_PI/180.0;

        y = R * dlat;
        x = R * cos(mean_lat) * dlon;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    nav_msgs::msg::Path path_msg_;

    bool origin_set_ = false;
    double origin_lat_;
    double origin_lon_;
};

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RouteLogger>());
    rclcpp::shutdown();
    return 0;
}
