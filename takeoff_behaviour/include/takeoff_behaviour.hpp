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
        pluginlib::ClassLoader<takeoff_base::TakeOffBase> loader("takeoff_plugin_base", "takeoff_base::TakeOffBase");

        try
        {
            takeoff_speed_ = loader.createSharedInstance("takeoff_plugins::TakeOffSpeed");
            takeoff_speed_->initialize(this);
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED");
        }
        catch (pluginlib::PluginlibException &ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }
    };

    rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal)
    {
        return takeoff_speed_->onAccepted(goal);
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        return takeoff_speed_->onCancel(goal_handle);
    }

    void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
    {
        return takeoff_speed_->onExecute(goal_handle);
    }

private:
    std::shared_ptr<takeoff_base::TakeOffBase> takeoff_speed_;
};

#endif // TAKE_OFF_BEHAVIOUR_HPP