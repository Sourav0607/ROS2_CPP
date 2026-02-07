#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "custom_interfaces/srv/battery_charge.hpp"

using namespace std;
using namespace std::placeholders;

class BatterySimulator : public rclcpp::Node
{
    public:
        BatterySimulator() : Node("battery_simulator"), battery_percentage_(100.0)
        {
            RCLCPP_INFO(this->get_logger(), "Battery simulator has been started");
            publisher_ = this->create_publisher<example_interfaces::msg::Float64>(
                "/battery_status",
                10
            );
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&BatterySimulator::timerCallback, this)
            );
            
            // Create service server for charging
            service_ = this->create_service<custom_interfaces::srv::BatteryCharge>(
                "/charge_battery",
                std::bind(&BatterySimulator::chargeCallback, this, _1, _2)
            );
            RCLCPP_INFO(this->get_logger(), "Charge battery service created!");
        }
    private:
        void timerCallback()
        {
            auto battery_percentage = example_interfaces::msg::Float64();
            battery_percentage.data = battery_percentage_;
            battery_percentage_ -= 10;
            if (battery_percentage_ < 0)
            {
                battery_percentage_ = 0;
            }
            
            publisher_->publish(battery_percentage);
        }
        
        void chargeCallback(const std::shared_ptr<custom_interfaces::srv::BatteryCharge::Request> request,
                           std::shared_ptr<custom_interfaces::srv::BatteryCharge::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Charging battery with amount: %.1f", request->charge_amount);
            battery_percentage_ = request->charge_amount;
            
            response->battery_percentage = battery_percentage_;
            response->success = true;
            response->message = "Battery charged successfully!";
            
            RCLCPP_INFO(this->get_logger(), "Battery charged to %.1f%%", battery_percentage_);
        }
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_;
        rclcpp::Service<custom_interfaces::srv::BatteryCharge>::SharedPtr service_;
        double battery_percentage_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatterySimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}