#ifndef TAKEOFF_BASE_HPP
#define TAKEOFF_BASE_HPP

#include "as2_core/node.hpp"

#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include <as2_msgs/action/take_off.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define DEFAULT_TAKEOFF_ALTITUDE 1.0 // [m]
#define DEFAULT_TAKEOFF_SPEED 0.4    // [m/s]
#define TAKEOFF_HEIGHT_THRESHOLD 0.1 // [m]

namespace takeoff_base
{
    class TakeOffBase
    {
    public:
        using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;

        void initialize(as2::Node *node_ptr)
        {
            node_ptr_ = node_ptr;
            odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
                node_ptr_->generate_global_name(as2_names::topics::self_localization::odom), as2_names::topics::self_localization::qos,
                std::bind(&TakeOffBase::odomCb, this, std::placeholders::_1));

            this->ownInit(node_ptr_);
        };

        virtual rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) = 0;
        virtual rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) = 0;
        virtual void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) = 0;

        virtual ~TakeOffBase(){};

    protected:
        TakeOffBase(){};

        // To initialize needed publisher for each plugin
        virtual void ownInit(as2::Node *node_ptr){};

    private:
        // TODO: if onExecute is done with timer no atomic attributes needed
        void odomCb(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
            this->actual_heigth_ = msg->pose.pose.position.z;
            this->actual_z_speed_ = msg->twist.twist.linear.z;
        };

    protected:
        as2::Node *node_ptr_;

        std::atomic<float> actual_heigth_;
        std::atomic<float> actual_z_speed_;

        float desired_speed_ = 0.0;
        float desired_height_ = 0.0;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    }; // TakeOffBase class

}  // takeoff_base namespace

#endif // TAKEOFF_BASE_HPP