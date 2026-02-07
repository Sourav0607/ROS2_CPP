#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/robot_state.hpp"


class CustomInterface : public rclcpp::Node
{
public:
    CustomInterface() : Node("custom_interface_node")
    {
        RCLCPP_INFO(this->get_logger(), "Custom Interface Node Started");
        publisher_ = this->create_publisher<custom_interfaces::msg::RobotState>(
            "/robot_state",
            10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CustomInterface::timerCallback, this)
        );
    }

private:
    void timerCallback()
    {
        auto msg = custom_interfaces::msg::RobotState();
        msg.name = "Panther01";
        msg.battery = "100%";
        msg.temp = 78;
        msg.is_moving = true;

        publisher_->publish(msg);
        //RCLCPP_INFO(this->get_logger(), "Robot: %s | Battery: %s | Temp: %d | Moving: %d", 
                   // msg.name.c_str(), msg.battery.c_str(), msg.temp, msg.is_moving);
    }
    
    rclcpp::Publisher<custom_interfaces::msg::RobotState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CustomInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}