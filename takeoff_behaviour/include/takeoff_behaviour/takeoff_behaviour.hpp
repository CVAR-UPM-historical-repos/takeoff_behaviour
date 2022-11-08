/*!*******************************************************************************************
 *  \file       takeoff_behaviour.hpp
 *  \brief      Takeoff behaviour class definition
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

#ifndef TAKE_OFF_BEHAVIOUR_HPP
#define TAKE_OFF_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/take_off.hpp>
#include "as2_msgs/srv/set_platform_state_machine_event.hpp"

#include <pluginlib/class_loader.hpp>
#include "takeoff_plugin_base/takeoff_base.hpp"

#include "rclcpp/rclcpp.hpp"

class TakeOffBehaviour : public as2::BasicBehaviour<as2_msgs::action::TakeOff> {
public:
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<as2_msgs::action::TakeOff>;
  using PSME              = as2_msgs::msg::PlatformStateMachineEvent;

  TakeOffBehaviour()
      : as2::BasicBehaviour<as2_msgs::action::TakeOff>(as2_names::actions::behaviours::takeoff) {
    try {
      this->declare_parameter<std::string>("default_takeoff_plugin");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <default_takeoff_plugin> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }
    try {
      this->declare_parameter<double>("default_takeoff_altitude");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <default_takeoff_altitude> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }
    try {
      this->declare_parameter<double>("default_takeoff_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <default_takeoff_speed> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }
    try {
      this->declare_parameter<double>("takeoff_height_threshold");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <takeoff_height_threshold> not defined or "
                   "malformed: %s",
                   e.what());
      this->~TakeOffBehaviour();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeOffBase>>(
        "takeoff_plugin_base", "takeoff_base::TakeOffBase");

    try {
      std::string plugin_name = this->get_parameter("default_takeoff_plugin").as_string();
      plugin_name += "::Plugin";
      takeoff_plugin_ = loader_->createSharedInstance(plugin_name);
      takeoff_plugin_->initialize(this,
                                  this->get_parameter("takeoff_height_threshold").as_double());
      RCLCPP_INFO(this->get_logger(), "TAKEOFF PLUGIN LOADED: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      this->~TakeOffBehaviour();
    }

    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());
    service_client_ = this->create_client<as2_msgs::srv::SetPlatformStateMachineEvent>(
        as2_names::services::platform::set_platform_state_machine_event,
        rmw_qos_profile_services_default, callback_group_);
  };

  ~TakeOffBehaviour(){};

  rclcpp_action::GoalResponse onAccepted(
      const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) {
    if (goal->takeoff_height < 0.0f) {
      RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff height");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->takeoff_speed < 0.0f) {
      RCLCPP_ERROR(this->get_logger(), "TakeOffBehaviour: Invalid takeoff speed");
      return rclcpp_action::GoalResponse::REJECT;
    }

    as2_msgs::action::TakeOff::Goal new_goal;
    new_goal.takeoff_speed  = (goal->takeoff_speed != 0.0f)
                                  ? goal->takeoff_speed
                                  : this->get_parameter("default_takeoff_speed").as_double();
    new_goal.takeoff_height = (goal->takeoff_height != 0.0f)
                                  ? goal->takeoff_height
                                  : this->get_parameter("default_takeoff_altitude").as_double();

    auto _goal = std::make_shared<const as2_msgs::action::TakeOff::Goal>(new_goal);

    this->sendEventFSME(PSME::TAKE_OFF, 3);

    RCLCPP_INFO(this->get_logger(), "TakeOffBehaviour: TakeOff with speed %f and height %f",
                _goal->takeoff_speed, _goal->takeoff_height);

    return takeoff_plugin_->onAccepted(_goal);
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
    return takeoff_plugin_->onCancel(goal_handle);
  }

  void onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) {
    if (takeoff_plugin_->onExecute(goal_handle)) {
      RCLCPP_INFO(this->get_logger(), "Takeoff succeeded");
    } else {
      RCLCPP_INFO(this->get_logger(), "Takeoff canceled");
    }

    this->sendEventFSME(PSME::TOOK_OFF, 3);
  }

private:
  bool sendEventFSME(const int8_t event, int timeout) {
    auto request         = std::make_shared<as2_msgs::srv::SetPlatformStateMachineEvent::Request>();
    request->event.event = event;

    auto result_future = service_client_->async_send_request(request);

    rclcpp::FutureReturnCode rc;
    rc = callback_group_executor_.spin_until_future_complete(result_future,
                                                             std::chrono::seconds(timeout));
    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
      return false;
    }

    if (!result_future.get()->success) {
      return false;
    }
    return true;
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeOffBase>> loader_;
  std::shared_ptr<takeoff_base::TakeOffBase> takeoff_plugin_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<as2_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr service_client_;
};

#endif  // TAKE_OFF_BEHAVIOUR_HPP
