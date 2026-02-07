#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/set_speed.hpp"

using namespace std;
using namespace placeholders;

class RobotControl : public rclcpp::Node
{
    public:
        RobotControl() : Node("robot_control_server")
        {
            RCLCPP_INFO(this->get_logger(), "Robot Control Service has started");
            robot_server_  = this->create_service<custom_interfaces::srv::SetSpeed>(
                "/robot_control",
                std::bind(&RobotControl::serviceCallback, this, _1, _2)
            );
        }
    private:
        void serviceCallback(const custom_interfaces::srv::SetSpeed::Request::SharedPtr request,
                             const custom_interfaces::srv::SetSpeed::Response::SharedPtr response)
        {
            if (request->linear_speed <= 1.0 && request->angular_speed <= 1.0 && request->linear_speed > -1.0 && request->angular_speed > -1.0) 
            {
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "Robot is Under control ");
            }
            else
            {
                response->success = false;
                RCLCPP_INFO(this->get_logger(), "Robot is not under control  ");
            }
            
        }
        rclcpp::Service<custom_interfaces::srv::SetSpeed>::SharedPtr robot_server_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}