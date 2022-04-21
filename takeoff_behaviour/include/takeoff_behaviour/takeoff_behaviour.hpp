#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"

#include <as2_msgs/action/take_off.hpp>

#include <pluginlib/class_loader.hpp>
#include "takeoff_plugin_base/takeoff_base.hpp"

class TakeOffBehaviour : public as2::BasicBehaviour<as2_msgs::action::TakeOff>
{
public:
    using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;
    using PSME = as2_msgs::msg::PlatformStateMachineEvent;

    TakeOffBehaviour() : as2::BasicBehaviour<as2_msgs::action::TakeOff>(as2_names::actions::behaviours::takeoff)
    {
        this->declare_parameter("default_takeoff_plugin");
        this->declare_parameter("default_takeoff_altitude");
        this->declare_parameter("default_takeoff_speed");
        this->declare_parameter("takeoff_height_threshold");

        loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeOffBase>>("takeoff_plugin_base", "takeoff_base::TakeOffBase");

        try
        {
            takeoff_speed_ = loader_->createSharedInstance(this->get_parameter("default_takeoff_plugin").as_string());
            takeoff_speed_->initialize(this, this->get_parameter("takeoff_height_threshold").as_double());
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
 
        state_machine_event_cli_ = this->create_client<as2_msgs::srv::SetPlatformStateMachineEvent>(this->generate_global_name(as2_names::services::platform::setplatformstatemachineevent));
        if ( state_machine_event_cli_->wait_for_service() ) 
        {
            RCLCPP_INFO(this->get_logger(), "TakeOff Behaviour ready!");
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

        if ( this->callStateMachineServer(PSME::TAKE_OFF, false) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
            return rclcpp_action::GoalResponse::REJECT;
        }

        as2_msgs::action::TakeOff::Goal new_goal;
        new_goal.takeoff_speed = (goal->takeoff_speed != 0.0f) ? goal->takeoff_speed : this->get_parameter("default_takeoff_speed").as_double();
        new_goal.takeoff_height = (goal->takeoff_height != 0.0f) ? goal->takeoff_height : this->get_parameter("default_takeoff_altitude").as_double();

        auto _goal = std::make_shared<const as2_msgs::action::TakeOff::Goal>(new_goal);

        RCLCPP_INFO(this->get_logger(),
                    "TakeOffBehaviour: TakeOff with speed %f and height %f",
                    _goal->takeoff_speed, _goal->takeoff_height);

        return takeoff_speed_->onAccepted(_goal);
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        return takeoff_speed_->onCancel(goal_handle);
    }

    void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        if (takeoff_speed_->onExecute(goal_handle))
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff succeeded");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff canceled");
        }

        // TODO: Always changing to flying
        if (this->callStateMachineServer(PSME::TOOK_OFF, true) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service state_machine_event");
        }
    }

private:
    rclcpp::FutureReturnCode callStateMachineServer(const int8_t machine_event, bool is_async)
    {
        auto request = std::make_shared<as2_msgs::srv::SetPlatformStateMachineEvent::Request>();
        request->event.event = machine_event;

        if (is_async)
        {
            auto future = state_machine_event_cli_->async_send_request(request);
            return rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        }
        else
        {
            // Local aux node to call client asyncronous, since can not spin at the same node
            std::shared_ptr<rclcpp::Node> aux_node_ptr;
            aux_node_ptr = std::make_shared<rclcpp::Node>("takeoff_baheviour_aux_node");
            auto state_machine_event_cli = aux_node_ptr->create_client<as2_msgs::srv::SetPlatformStateMachineEvent>(this->generate_global_name(as2_names::services::platform::setplatformstatemachineevent));
            auto future = state_machine_event_cli->async_send_request(request);
            return rclcpp::spin_until_future_complete(aux_node_ptr, future);
        }
    }

private:
    std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
    std::shared_ptr<takeoff_base::TakeOffBase> takeoff_speed_;

    rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr state_machine_event_cli_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP