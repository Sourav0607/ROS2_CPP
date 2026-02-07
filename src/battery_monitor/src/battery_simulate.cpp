#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"

using namespace std;

class BatterySimulator : public rclcpp::Node
{
    public:
        BatterySimulator() : Node("battery_simulator"), battery_percentage_(100)
        {
            RCLCPP_INFO(this->get_logger(), "Battery simulator has been started");
            publisher_ = this->create_publisher<example_interfaces::msg::Float64>(
                "/battery_status",
                10
            );
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10000),
                std::bind(&BatterySimulator::timerCallback, this)
            );

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
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_;
        int battery_percentage_ ;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatterySimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}