/*!*******************************************************************************************
 *  \file       takeoff_base.hpp
 *  \brief      Base class for takeoff plugins
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef TAKEOFF_BASE_HPP
#define TAKEOFF_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"
#include <as2_core/frame_utils/frame_utils.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <as2_msgs/action/take_off.hpp>

#include <Eigen/Dense>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace takeoff_base
{
    class TakeOffBase
    {
    public:
        using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;

        void initialize(as2::Node *node_ptr, float takeoff_height_threshold)
        {
            node_ptr_ = node_ptr;
            takeoff_height_threshold_ = takeoff_height_threshold;

            pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(node_ptr_, as2_names::topics::self_localization::pose, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
            twist_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>>(node_ptr_, as2_names::topics::self_localization::twist, as2_names::topics::self_localization::qos.get_rmw_qos_profile());
            synchronizer_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(approximate_policy(5), *(pose_sub_.get()), *(twist_sub_.get()));
            synchronizer_->registerCallback(&TakeOffBase::state_callback, this);

            this->ownInit(node_ptr_);
        };

        virtual rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) = 0;
        virtual rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) = 0;
        virtual bool onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) = 0;

        virtual ~TakeOffBase(){};

    protected:
        TakeOffBase(){};

        // To initialize needed publisher for each plugin
        virtual void ownInit(as2::Node *node_ptr){};
        
        bool checkGoalCondition()
        {
            if ((desired_height_ - actual_heigth_) <= 0 + this->takeoff_height_threshold_)
            {
               return true;
            }
            return false;
        };

    private:
        // TODO: if onExecute is done with timer no atomic attributes needed
        void state_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg,
                            const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
        {
            odom_received_ = true;
            pose_mutex_.lock();
            actual_position_ = {
                pose_msg->pose.position.x, 
                pose_msg->pose.position.y,
                pose_msg->pose.position.z};

            actual_speed_ = {
                twist_msg->twist.linear.x, 
                twist_msg->twist.linear.y,
                twist_msg->twist.linear.z};
            
            actual_q_ = {
                pose_msg->pose.orientation.x, 
                pose_msg->pose.orientation.y, 
                pose_msg->pose.orientation.z, 
                pose_msg->pose.orientation.w};
            
            this->actual_heigth_ = pose_msg->pose.position.z;
            this->actual_z_speed_ = twist_msg->twist.linear.z;
            pose_mutex_.unlock();
        };

    protected:
        as2::Node *node_ptr_;
        float takeoff_height_threshold_;

        std::mutex pose_mutex_;
        Eigen::Vector3d actual_position_;
        Eigen::Vector3d actual_speed_;
        tf2::Quaternion actual_q_;

        std::atomic<float> actual_heigth_;
        std::atomic<float> actual_z_speed_;

        std::atomic<bool> odom_received_ = false;

        float desired_speed_ = 0.0;
        float desired_height_ = 0.0;

    private:
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
        std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::TwistStamped>> twist_sub_;
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped> approximate_policy;
        std::shared_ptr<message_filters::Synchronizer<approximate_policy>> synchronizer_;
    }; // TakeOffBase class

}  // takeoff_base namespace

#endif // TAKEOFF_BASE_HPP
