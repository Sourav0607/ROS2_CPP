#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std;

class MyPublisher : public rclcpp::Node
{
    private:
        void PublisherCallback()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hello from publisher " + std::to_string(counter_));
            RCLCPP_INFO(this->get_logger(), "Message is : %s ", msg.data.c_str());  
            publisher_->publish(msg);
            counter_++;
            
        }
        int counter_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_ ;
    public:
        MyPublisher() : Node("publisher") ,  counter_(0)
        {
            RCLCPP_INFO(this->get_logger(), "Publisher Node Has Started");
            publisher_ = this->create_publisher<example_interfaces::msg::String>(
                "/mypub",
                10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&MyPublisher::PublisherCallback, this)
            );
        }
};

int main(int argc, char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MyPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}