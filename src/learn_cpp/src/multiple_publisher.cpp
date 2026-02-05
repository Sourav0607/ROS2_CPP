#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/msg/float32.hpp"

using namespace std;

class MultiplePublisher : public rclcpp::Node
{
    private:
        void publisherCallback()
        {
            auto status_msg = example_interfaces::msg::String();
            auto temp_msg = example_interfaces::msg::Float32();
            auto battery_msg = example_interfaces::msg::Float32();

            status_msg.data = std::string("Running");
            temp_msg.data = 78.0;
            battery_msg.data = 100.0;

            /*RCLCPP_INFO(this->get_logger(),
                                        "Status is : %s, Temperature is : %f, Battery is : %f",status_msg.data.c_str(),temp_msg.data,battery_msg.data
                                        );*/
            status_publisher_->publish(status_msg);
            temperature_publisher_->publish(temp_msg);
            battery_publisher_->publish(battery_msg);
        }

        rclcpp::TimerBase::SharedPtr timer_ ;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr status_publisher_ ;
        rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr temperature_publisher_;
        rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr battery_publisher_ ;

    public:
        MultiplePublisher() : Node("multiple_publisher")
        {
            RCLCPP_INFO(this->get_logger(),"Multiple Publisher has started");

            status_publisher_ = this->create_publisher<example_interfaces::msg::String>(
                "/status",
                10
            );
            temperature_publisher_ = this->create_publisher<example_interfaces::msg::Float32>(
                "/temperature",
                10
            );
            battery_publisher_ = this->create_publisher<example_interfaces::msg::Float32>(
                "/battery",
                10
            );

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&MultiplePublisher::publisherCallback, this)
            );

        }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MultiplePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}