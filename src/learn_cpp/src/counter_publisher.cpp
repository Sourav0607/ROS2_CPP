// Counter publisher publisher count at every 500ms

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"


class CounterPublisher : public rclcpp::Node
{
    public:
        CounterPublisher() : Node("counter_publisher"), counter_(2)
        {
            RCLCPP_INFO(this->get_logger(), "Counter Publisher is Started");
            counterpublisher_ =  this->create_publisher<example_interfaces::msg::Int32>(
                "count_publisher",
                10
            );
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&CounterPublisher::CounterCallback, this)
            );
        }
    private:
        void CounterCallback()
        {
           auto msg = example_interfaces::msg::Int32();
           msg.data = std::int32_t(counter_);
           counterpublisher_->publish(msg);
           counter_++;
            
        }
        int counter_;
        rclcpp::Publisher<example_interfaces::msg::Int32>::SharedPtr counterpublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CounterPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}