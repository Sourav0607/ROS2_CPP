#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/string.hpp"

using namespace rclcpp_lifecycle;
using namespace std;
using namespace rclcpp_lifecycle::node_interfaces;

class LifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        LifeCycleNode() : LifecycleNode("lifecycle_example_node")
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>(
                "/topic",
                10
            );
        }
        //Override on_configure, on_active, on_deactivte,on_sutdown

        LifecycleNodeInterface::CallbackReturn on_configure(const State &)
        {
            RCLCPP_INFO(this->get_logger(), "Coniguring ......");
            return LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        LifecycleNodeInterface::CallbackReturn on_activate(const State &)
        {
            RCLCPP_INFO(this->get_logger(), "Activating ......");
            //Activate the publisher
            publisher_->on_activate();
            //Start publishing when activate
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&LifeCycleNode::timerCallback, this)
            );
            return LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        LifecycleNodeInterface::CallbackReturn on_deactivate(const State &)
        {
            RCLCPP_INFO(this->get_logger(), "Deactivating ......");
            //Deactivate the publisher
            publisher_->on_deactivate();
            //Stop publishing when Deactivate
            timer_->cancel();
            return LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        LifecycleNodeInterface::CallbackReturn on_shutdown(const State &)
        {
            RCLCPP_INFO(this->get_logger(), "Shutting Down......");
            return LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
    private:

        void timerCallback()
        {
            auto msg = std_msgs::msg::String();
            msg.data = "Hello from lifecycle node!";
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published message %s",msg.data.c_str());
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LifeCycleNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;

}