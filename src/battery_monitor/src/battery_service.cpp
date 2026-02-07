#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "custom_interfaces/srv/battery_charge.hpp"

using namespace std::placeholders;
using namespace std;

class BatteryChargeService : public rclcpp::Node
{
    public:
        BatteryChargeService() : Node("battery_charge_service")
        {
            RCLCPP_INFO(this->get_logger(), "Battery charge service has been started");
            
            // Subscribe to battery warning
            subscriber_ = this->create_subscription<example_interfaces::msg::Bool>(
                "/battery_warn",
                10,
                std::bind(&BatteryChargeService::warningCallback, this, _1)
            );
            
            // Create service client to call charge service
            client_ = this->create_client<custom_interfaces::srv::BatteryCharge>("/charge_battery");
        }
        
    private:
        void warningCallback(const example_interfaces::msg::Bool::SharedPtr msg)
        {
            if (msg->data == true)
            {
                RCLCPP_INFO(this->get_logger(), "Battery low! Calling charge service...");
                
                // Wait for service to be available
                while (!client_->wait_for_service(std::chrono::seconds(1)))
                {
                    if (!rclcpp::ok())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                        return;
                    }
                    RCLCPP_WARN(this->get_logger(), "Waiting for charge service...");
                }
                
                // Call the service
                auto request = std::make_shared<custom_interfaces::srv::BatteryCharge::Request>();
                request->charge_amount = 100.0;
                
                auto future = client_->async_send_request(request,
                    std::bind(&BatteryChargeService::chargeResponseCallback, this, _1));
            }
        }
        
        void chargeResponseCallback(rclcpp::Client<custom_interfaces::srv::BatteryCharge>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Charge successful! Battery: %.1f%%, Message: %s", 
                           response->battery_percentage, response->message.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Charge failed!");
            }
        }

        rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr subscriber_;
        rclcpp::Client<custom_interfaces::srv::BatteryCharge>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<BatteryChargeService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
