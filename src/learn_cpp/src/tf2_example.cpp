#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TF2Example : public rclcpp::Node
{
    public:
        TF2Example() : Node("tf2_example")
        {
            RCLCPP_INFO(this->get_logger(),"TF2 example node started");

            // Create statis transform broadcaster for base link -> lidar link

            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            //Create dynamic transform broadcaster for odom -> base link
            dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            //create static traansform once

            publishStaticTransform();

            //Create timr for dynamic transform
            timer_ =  this->create_wall_timer(
                100ms,
                std::bind(&TF2Example::publishDynamicTransform, this)
            );
        }
    private:
        void publishStaticTransform()
        {
            geometry_msgs::msg::TransformStamped static_transform_;
            
            static_transform_.header.stamp = this->get_clock()->now();
            static_transform_.header.frame_id = "base_link";
            static_transform_.child_frame_id = "laser_frame";
            
            //Translation (laser is 0.2m, forward and 0.1m up from base link)
            static_transform_.transform.translation.x = 0.2;
            static_transform_.transform.translation.y = 0.0;
            static_transform_.transform.translation.z = 0.1;

            //Rotation (no rotation)
            static_transform_.transform.rotation.x = 0.0;
            static_transform_.transform.rotation.y = 0.0;
            static_transform_.transform.rotation.z = 0.0;
            static_transform_.transform.rotation.w = 1.0;

            static_tf_broadcaster_->sendTransform(static_transform_);
            RCLCPP_INFO(this->get_logger(),"Static transform publisher base_link -> laser_frame");
        }

        void publishDynamicTransform()
        {
            geometry_msgs::msg::TransformStamped dynamic_transform_;

            dynamic_transform_.header.stamp = this->get_clock()->now();
            dynamic_transform_.header.frame_id = "odom";
            dynamic_transform_.child_frame_id = "base_link";

            //Translation (robot moving forward)
            dynamic_transform_.transform.translation.x = x_position_;
            dynamic_transform_.transform.translation.y = 0.0;
            dynamic_transform_.transform.translation.z = 0.0;
             //Rotation
            dynamic_transform_.transform.rotation.x  = 0.0;
            dynamic_transform_.transform.rotation.y = 0.0;
            dynamic_transform_.transform.rotation.z = sin(angle_/2.0);
            dynamic_transform_.transform.rotation.w = cos(angle_/2.0);

            dynamic_tf_broadcaster_->sendTransform(dynamic_transform_);
            RCLCPP_INFO(this->get_logger(), "Dynamic Transform publisher odom -> base_link");

            x_position_ += 0.01;
            angle_ += 0.05;

            if(x_position_ > 5.0) x_position_ = 0.0; //Reset
            if(angle_ > 2 * M_PI) angle_ = 0.0; //reste
        }

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ ;
        rclcpp::TimerBase::SharedPtr timer_;

        double x_position_ = 0.0;
        double angle_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TF2Example>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}