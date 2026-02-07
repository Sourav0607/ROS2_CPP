#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/bool.hpp"

using namespace std;
using namespace std::placeholders;


class BatteryWarn : public rclcpp::Node
{
    public:
        BatteryWarn() : Node("battery_warn") , battery_warn_(false)
        {
            RCLCPP_INFO(this->get_logger(),"Battery warn node has started");
            publisher_ = this->create_publisher<example_interfaces::msg::Bool>(
                "/battery_warn",
                10
            );

            timer_ =  this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&BatteryWarn::timerCallback, this)
            );

            subscriber_ =  this->create_subscription<example_interfaces::msg::Float64>(
                "/battery_status",
                10,
                std::bind(&BatteryWarn::subscriberCallback, this, _1)
            );
        }
    private:
        void subscriberCallback(const example_interfaces::msg::Float64::SharedPtr battery_percentage_)
        {
            if (battery_percentage_->data <= 20.0)
            {
                battery_warn_ = true;
            }
            else
                battery_warn_ = false;
            
        }

        void timerCallback()
        {
            if (battery_warn_ == true)
            {
                auto msg = example_interfaces::msg::Bool();
                msg.data = true;
                publisher_->publish(msg);
            }
            else
            {
                auto msg = example_interfaces::msg::Bool();
                msg.data = false;
                publisher_->publish(msg);
            }
            
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr publisher_;
        rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr subscriber_;

        bool battery_warn_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatteryWarn>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}