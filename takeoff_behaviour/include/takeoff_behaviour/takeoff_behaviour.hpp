#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"

#include <as2_msgs/action/take_off.hpp>
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"

#include <pluginlib/class_loader.hpp>
#include "takeoff_plugin_base/takeoff_base.hpp"

#include "as2_core/synchronous_service_client.hpp"

class TakeOffBehaviour : public as2::BasicBehaviour<as2_msgs::action::TakeOff>
{
public:
    using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;
    using PSME = as2_msgs::msg::PlatformStateMachineEvent;

    TakeOffBehaviour()
        : as2::BasicBehaviour<as2_msgs::action::TakeOff>(as2_names::actions::behaviours::takeoff),
          state_machine_event_cli_(as2_names::services::platform::set_platform_state_machine_event)
    {
        this->declare_parameter("default_takeoff_plugin");
        this->declare_parameter("default_takeoff_altitude");
        this->declare_parameter("default_takeoff_speed");
        this->declare_parameter("takeoff_height_threshold");

        loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeOffBase>>("takeoff_plugin_base", "takeoff_base::TakeOffBase");

        try
        {
            takeoff_plugin_ = loader_->createSharedInstance(this->get_parameter("default_takeoff_plugin").as_string());
            takeoff_plugin_->initialize(this, this->get_parameter("takeoff_height_threshold").as_double());
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED: %s", this->get_parameter("default_takeoff_plugin").as_string().c_str());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    };

    rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal)
    {
        if (goal->takeoff_height < 0.0f)
        {
            RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff height");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->takeoff_speed < 0.0f)
        {
            RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff speed");
            return rclcpp_action::GoalResponse::REJECT;
        }

        as2_msgs::action::TakeOff::Goal new_goal;
        new_goal.takeoff_speed = (goal->takeoff_speed != 0.0f) ? goal->takeoff_speed : this->get_parameter("default_takeoff_speed").as_double();
        new_goal.takeoff_height = (goal->takeoff_height != 0.0f) ? goal->takeoff_height : this->get_parameter("default_takeoff_altitude").as_double();

        auto _goal = std::make_shared<const as2_msgs::action::TakeOff::Goal>(new_goal);

        auto request = as2_msgs::srv::SetPlatformStateMachineEvent::Request();
        auto response = as2_msgs::srv::SetPlatformStateMachineEvent::Response();
        request.event.event = PSME::TAKE_OFF;
        if (!state_machine_event_cli_.sendRequest(request, response, 1))
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(),
                    "TakeOffBehaviour: TakeOff with speed %f and height %f",
                    _goal->takeoff_speed, _goal->takeoff_height);

        return takeoff_plugin_->onAccepted(_goal);
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        return takeoff_plugin_->onCancel(goal_handle);
    }

    void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        if (takeoff_plugin_->onExecute(goal_handle))
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff succeeded");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff canceled");
        }

        auto request = as2_msgs::srv::SetPlatformStateMachineEvent::Request();
        auto response = as2_msgs::srv::SetPlatformStateMachineEvent::Response();
        request.event.event = PSME::TOOK_OFF;
        if (!state_machine_event_cli_.sendRequest(request, response, 1))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
        }
    }

private:
    std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
    std::shared_ptr<takeoff_base::TakeOffBase> takeoff_plugin_;

    as2::SynchronousServiceClient<as2_msgs::srv::SetPlatformStateMachineEvent> state_machine_event_cli_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP