#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TF2Listener : public rclcpp::Node
{
    public:
        TF2Listener() : Node("tf2_listener")
        {
            RCLCPP_INFO(this->get_logger(), "TF2 listener node started");

            //Create TF2 Buffer and listener
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            //Subscribe to laser scan
            scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&TF2Listener::scanCallback, this, std::placeholders::_1)
            );

            //Publisher for transfomed points
            transformed_points_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
                "/transformed_scan",
                10
            );
        }
    private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try
        {
            //Look up transform from laser frame to base link
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "base_link",               //target frame
                msg->header.frame_id,      //source frame
                msg->header.stamp,         //time
                rclcpp::Duration::from_seconds(1.0)   //timeout
            );

            //Create polygon to hold transformed points
            geometry_msgs::msg::PolygonStamped transformed_polygon;
            transformed_polygon.header.stamp = msg->header.stamp;
            transformed_polygon.header.frame_id = "base_link";

            //Transform eaach laser scan point
            float angle = msg->angle_min;
            for (size_t i = 0; i < msg->ranges.size(); i++)
            {
                //Skip invalid readings
                if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                {
                    angle += msg->angle_increment;
                    continue;
                }

                //Convert polar (range, angle) to cartesion (x,y) in laser frame
                geometry_msgs::msg::PointStamped point_in_laser;
                point_in_laser.header = msg->header;
                point_in_laser.point.x = msg->ranges[i] * cos(angle);
                point_in_laser.point.y = msg->ranges[i] * sin(angle);
                point_in_laser.point.z = 0.0;

                //Transform point to nase link frame
                geometry_msgs::msg::PointStamped point_in_base;
                tf2::doTransform(point_in_laser, point_in_base, transform);

                //Add to polygom
                geometry_msgs::msg::Point32 p;
                p.x = point_in_base.point.x;
                p.y = point_in_base.point.y;
                p.z = point_in_base.point.z;
                transformed_polygon.polygon.points.push_back(p);

                angle += msg->angle_increment;
                
            }

            //Publish Transformed Points
            transformed_points_pub_->publish(transformed_polygon);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Transformed %zu laser points from %s to base link",
            transformed_polygon.polygon.points.size(),
            msg->header.frame_id.c_str());
        }

        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Could not transform : %s", ex.what());
        }
        
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr transformed_points_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TF2Listener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


