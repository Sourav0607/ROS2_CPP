#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int32.hpp"

using namespace std;
using namespace std::placeholders;

class CounterSubscriber : public rclcpp::Node
{
    public:
        CounterSubscriber() : Node("counter_subscriber")
        {
            RCLCPP_INFO(this->get_logger(), "Counter Subcriber is Started");
            countersubscriber_ = this->create_subscription<example_interfaces::msg::Int32>(
                "count_publisher",
                10,
                std::bind(&CounterSubscriber::SubscriberCallback, this, _1)
            );
        }
    private:
        void SubscriberCallback(const example_interfaces::msg::Int32 msg)
        {
            if (msg.data % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(),"Divisble by 10 and also a Even number: %d", msg.data);
            }
            else if (msg.data % 2 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Its a Even Number : %d", msg.data);
            }
            else
                RCLCPP_INFO(this->get_logger(), "Its a ODD number : %d",msg.data);
            
            
        }

        rclcpp::Subscription<example_interfaces::msg::Int32>::SharedPtr countersubscriber_ ;


};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<CounterSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}