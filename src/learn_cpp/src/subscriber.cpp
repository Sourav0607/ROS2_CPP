#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std;
using namespace std::placeholders;

class MySubscriber : public rclcpp::Node
{
    public:
        MySubscriber() : Node("subscriber_node")
        {
            RCLCPP_INFO(this->get_logger(), "Subscriber Node has been started");
            subscriber_ = this->create_subscription<example_interfaces::msg::String>(
                "/mypub",
                10,
                std::bind(&MySubscriber::SubscriberCallback,this,_1)

            );
        }
    private:
        void SubscriberCallback(const example_interfaces::msg::String msg)
        {
            RCLCPP_INFO(this->get_logger(), "Message is : %s", msg.data.c_str());
        }

        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
