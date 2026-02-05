#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std;
using namespace std::placeholders ;

class TopicRemapping : public rclcpp::Node
{
    public:
        TopicRemapping() : Node("topic_remapping")
        {
            RCLCPP_INFO(this->get_logger(), "Topic Remapping has been started");
            subscriber_ = this->create_subscription<example_interfaces::msg::String>(
                "/status",
                10,
                std::bind(&TopicRemapping::subscriberCallback, this, _1)
            );
            publisher_ = this->create_publisher<example_interfaces::msg::String>(
                "/remapped_status",
                10
            );

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&TopicRemapping::publisherCallback,this)
            );
        }
    private:
        void subscriberCallback(const example_interfaces::msg::String::SharedPtr status_msg)
        {
            last_message_ = status_msg->data;
            RCLCPP_INFO(this->get_logger(),"Received message is : %s",status_msg->data.c_str());
        }

        void publisherCallback()
        {
            auto remapped_msg = example_interfaces::msg::String();
            remapped_msg.data = std::string("[RELAY] ") + last_message_;
            publisher_->publish(remapped_msg);
            RCLCPP_INFO(this->get_logger(),"Publishing: %s",remapped_msg.data.c_str());
        }

        std::string last_message_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TopicRemapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}