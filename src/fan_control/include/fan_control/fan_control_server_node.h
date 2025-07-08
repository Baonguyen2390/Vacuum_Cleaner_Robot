#ifndef __ACTION_SERVER__
#define __ACTION_SERVER__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robot_messages/action/fan_level_control.hpp>

class ControlFanLevel : public rclcpp::Node
{
public:
    using FanLevelControl = robot_messages::action::FanLevelControl;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FanLevelControl>;

    explicit ControlFanLevel(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FanLevelControl::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp_action::Server<FanLevelControl>::SharedPtr action_server_;
};

#endif