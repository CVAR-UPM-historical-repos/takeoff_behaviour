#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/take_off.hpp>

#include <pluginlib/class_loader.hpp>
#include "takeoff_plugin_base/takeoff_base.hpp"

#define DEFAULT_TAKEOFF_ALTITUDE 1.0 // [m]
#define DEFAULT_TAKEOFF_SPEED 0.4    // [m/s]
#define TAKEOFF_HEIGHT_THRESHOLD 0.1 // [m]

class TakeOffBehaviour : public as2::BasicBehaviour<as2_msgs::action::TakeOff>
{
public:
    using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;

    TakeOffBehaviour() : as2::BasicBehaviour<as2_msgs::action::TakeOff>(as2_names::actions::behaviours::takeoff)
    {
        auto loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeOffBase>>("takeoff_plugin_base", "takeoff_base::TakeOffBase");

        try
        {
            takeoff_speed_ = loader_->createSharedInstance("takeoff_plugins::TakeOffSpeed");
            takeoff_speed_->initialize(this, TAKEOFF_HEIGHT_THRESHOLD);
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
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

        // TODO: check 
        std::shared_ptr<as2_msgs::action::TakeOff::Goal> new_goal;
        new_goal->takeoff_speed = (goal->takeoff_speed != 0.0f) ? goal->takeoff_speed : DEFAULT_TAKEOFF_SPEED;
        new_goal->takeoff_height = (goal->takeoff_height != 0.0f) ? goal->takeoff_height : DEFAULT_TAKEOFF_ALTITUDE;

        RCLCPP_INFO(this->get_logger(),
                    "TakeOffBehaviour: TakeOff with speed %f and height %f",
                    new_goal->takeoff_speed, new_goal->takeoff_height);

        return takeoff_speed_->onAccepted(new_goal);
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        return takeoff_speed_->onCancel(goal_handle);
    }

    void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        return takeoff_speed_->onExecute(goal_handle);
    }

private:
    std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
    std::shared_ptr<takeoff_base::TakeOffBase> takeoff_speed_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP