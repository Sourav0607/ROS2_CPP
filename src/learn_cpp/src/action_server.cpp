#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
using namespace std;
using namespace placeholders;

class ActionServer : public rclcpp::Node
{
    public:
        ActionServer() : Node("fibonacci_action_server"), goal_executing_(false)
        {
            action_server_ = rclcpp_action::create_server<Fibonacci>(
                this,
                "fibonacci",
                std::bind(&ActionServer::handle_goal, this, _1, _2),
                std::bind(&ActionServer::handle_cancel, this,_1),
                std::bind(&ActionServer::handle_accepted,this,_1)
            );
            RCLCPP_INFO(this->get_logger(), "Fibonacci Server has been started");
        }
    private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const Fibonacci::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received Goal Request with order %d", goal->order);
            (void) uuid;
            
            // Reject if a goal is already executing
            if (goal_executing_)
            {
                RCLCPP_WARN(this->get_logger(), "A goal is already executing. Rejecting new goal.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            //Set flag to indicate goal is executing
            goal_executing_ = true;
            //Execute in a separate thred
            std::thread{std::bind(&ActionServer::execute, this,_1), goal_handle}.detach();
        }
        void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing Goal");
            rclcpp::Rate loop_rate(1); //1Hz
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Fibonacci::Feedback>();
            auto result = std::make_shared<Fibonacci::Result>();

            //Initialize fibonacci sequence
            feedback->sequence.push_back(0);
            feedback->sequence.push_back(1);
            
            for (int i = 1; i < goal->order &&  rclcpp::ok(); i++)
            {
                //Check if there is cancel request
                if (goal_handle->is_canceling())
                {
                    result->sequence = feedback->sequence;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
                    goal_executing_ = false; // Reset flag
                    return;
                }

                //Calculate next fibonacci sequence
                feedback->sequence.push_back(
                    feedback->sequence[i] + feedback->sequence[i-1]
                );
                //publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publishing new feedback");

                loop_rate.sleep();
            }
            //check if goal is done
            if (rclcpp::ok())
            {
                result->sequence = feedback->sequence;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal Succedd");
            }
            
            goal_executing_ = false; // Reset flag when done
        }
        
        bool goal_executing_;
        rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}