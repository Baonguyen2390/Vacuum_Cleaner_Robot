#include "fan_control/fan_control_server_node.h"
#include "rclcpp_components/register_node_macro.hpp"

#include <wiringPi.h>

ControlFanLevel::ControlFanLevel(const rclcpp::NodeOptions & options)
: Node("control_fan_level_server", options)
{
    using namespace std::placeholders;

    // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
    wiringPiSetupGpio();

    // pin mode ..(INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK)
    // set pin 17 to input
    pinMode(17, OUTPUT);

    this->action_server_ = rclcpp_action::create_server<FanLevelControl>(
        this,
        "control_fan_level",
        std::bind(&ControlFanLevel::handle_goal, this, _1, _2),
        std::bind(&ControlFanLevel::handle_cancel, this, _1),
        std::bind(&ControlFanLevel::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse ControlFanLevel::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FanLevelControl::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ControlFanLevel::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ControlFanLevel::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ControlFanLevel::execute, this, _1), goal_handle}.detach();
}

void ControlFanLevel::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto result = std::make_shared<FanLevelControl::Result>();
    const auto goal = goal_handle->get_goal();

    digitalWrite(17, goal->level);

    RCLCPP_INFO(this->get_logger(), "Fan level has been set to %d", goal->level);

    goal_handle->succeed(result);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControlFanLevel)
