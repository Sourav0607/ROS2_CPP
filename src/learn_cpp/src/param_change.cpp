#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/msg/int32.hpp"

using namespace std;
using namespace std::placeholders ;


class ParamChange: public rclcpp::Node
{
    public:
        ParamChange() : Node("param_change")
        {
            //Declare Parametes with default values
            this->declare_parameter("message", "Message Changed");
            this->declare_parameter("rate", 1000);

            // Get Initial Params
            message_ = this->get_parameter("message").as_string();
            int rate = this->get_parameter("rate").as_int();


            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&ParamChange::parameterCallback,this, std::placeholders:: _1)
            );

            publisher_ = this->create_publisher<example_interfaces::msg::String>(
                "message_publisher",
                10
            );

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(rate),
                std::bind(&ParamChange::timerCallback,this)
            );

            RCLCPP_INFO(this->get_logger(), "Param Publisher has been started");
            RCLCPP_INFO(this->get_logger(), " Initial Message %s, rate: %d",message_.c_str(),rate);
        }
    private:
        void timerCallback()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = message_;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publshing: %s", msg.data.c_str());
        }
        
        rcl_interfaces::msg::SetParametersResult parameterCallback(
            const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result ;
                result.successful = true;
            
            for (const auto &param: parameters)
            {
                if (param.get_name() == "message")
                {
                    message_ = param.as_string();
                    RCLCPP_INFO(this->get_logger(), "Message Changed to : %s",message_.c_str());
                }
                else if (param.get_name() == "rate")
                {
                    rate = param.as_int();
                    
                    //Validate

                    if (rate <= 0)
                    {
                        result.successful = false;
                        result.reason = "timer rate cannot be negative or zero";
                        return result;
                    }

                    timer_->cancel();
                    timer_= this->create_wall_timer(
                        std::chrono::milliseconds(rate),
                        std::bind(&ParamChange::timerCallback, this)
                    );

                    RCLCPP_INFO(this->get_logger(), "TImer rate changed to %d", rate);
                    
                }
                
            }
            return result;
            
        }

        std::string message_;
        int rate;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_ ;
};

int main(int argc, char**argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ParamChange>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}