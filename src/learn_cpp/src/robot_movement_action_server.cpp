#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_interfaces/action/move_distance.hpp"

using namespace std;
using MoveDistance = custom_interfaces::action::MoveDistance;
using GoalHandleMoveDistance = rclcpp_action::ServerGoalHandle<MoveDistance>;
using namespace std::placeholders;

class RobotMovement : public rclcpp::Node
{
    public:
        RobotMovement() : Node("robot_movement_server"), goal_executing_(false)
        {
            action_server_ =  rclcpp_action::create_server<MoveDistance>(
                this,
                "move_robot",
                std::bind(&RobotMovement::handle_goal, this, _1, _2),
                std::bind(&RobotMovement::handle_cancel, this, _1),
                std::bind(&RobotMovement::handle_accepted, this, _1)

            );

            RCLCPP_INFO(this->get_logger(), "Robot Movement action server has been started");
        }
    private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                std::shared_ptr<const MoveDistance::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request to move %.2f meters", goal->move);
            (void) uuid;

            //Rejecting new goal
            if(goal_executing_)
            {
                RCLCPP_ERROR(this->get_logger(), "Rejecting new goal");
                return rclcpp_action::GoalResponse::REJECT;
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveDistance> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void) goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleMoveDistance> goal_handle)
        {
            //Set flag to indicate goal execute
            goal_executing_ = true;
            // Execute in separate thread
            std::thread{std::bind(&RobotMovement::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleMoveDistance> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Executing goal...");
            
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<MoveDistance::Feedback>();
            auto result = std::make_shared<MoveDistance::Result>();
            
            rclcpp::Rate loop_rate(1); // 1 Hz
            
            double distance_moved = 0.0;
            double step = 0.1; // Move 0.1m per step
            
            while (distance_moved < goal->move)
            {
                // Check if goal is canceling
                if (goal_handle->is_canceling())
                {
                    result->is_moved = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
                
                // Simulate robot movement
                distance_moved += step;
                if (distance_moved > goal->move)
                {
                    distance_moved = goal->move;
                }
                
                // Publish feedback
                feedback->message = "Moving... " + std::to_string(distance_moved) + " / " + std::to_string(goal->move) + " meters";
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "%s", feedback->message.c_str());
                
                loop_rate.sleep();
            }
            
            // Goal succeeded
            if (rclcpp::ok())
            {
                result->is_moved = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded! Moved %.2f meters", distance_moved);
            }
            goal_executing_ = false; //reset
        }

        bool goal_executing_;
        rclcpp_action::Server<MoveDistance>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMovement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}