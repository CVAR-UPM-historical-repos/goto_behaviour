/*!*******************************************************************************************
 *  \file       goto_behaviour.hpp
 *  \brief      Goto behaviour class header file
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

#ifndef GOTO_BEHAVIOUR_HPP
#define GOTO_BEHAVIOUR_HPP

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>

#include <pluginlib/class_loader.hpp>
#include "goto_plugin_base/goto_base.hpp"

class GotoBehaviour : public as2_behavior::BehaviorServer<as2_msgs::action::GoToWaypoint> {
public:
  using GoalHandleGoto = rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

  GotoBehaviour()
      : as2_behavior::BehaviorServer<as2_msgs::action::GoToWaypoint>(
            as2_names::actions::behaviours::gotowaypoint) {
    try {
      this->declare_parameter<std::string>("default_goto_plugin");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <default_goto_plugin> not defined or malformed: %s", e.what());
      this->~GotoBehaviour();
    }
    try {
      this->declare_parameter<double>("default_goto_max_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <default_goto_max_speed> not defined or "
                   "malformed: %s",
                   e.what());
      this->~GotoBehaviour();
    }
    try {
      this->declare_parameter<double>("goto_threshold");
    } catch (const rclcpp::ParameterTypeException &e) {
      RCLCPP_FATAL(this->get_logger(),
                   "Launch argument <goto_threshold> not defined or malformed: %s", e.what());
      this->~GotoBehaviour();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<goto_base::GotoBase>>("goto_plugin_base",
                                                                            "goto_base::GotoBase");

    try {
      std::string plugin_name = this->get_parameter("default_goto_plugin").as_string();
      plugin_name += "::Plugin";
      goto_plugin_ = loader_->createSharedInstance(plugin_name);
      goto_plugin_->initialize(this, this->get_parameter("default_goto_max_speed").as_double(),
                               this->get_parameter("goto_threshold").as_double());
      RCLCPP_INFO(this->get_logger(), "GOTO BEHAVIOUR PLUGIN LOADED: %s", plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n",
                   ex.what());
      this->~GotoBehaviour();
    }

    RCLCPP_DEBUG(this->get_logger(), "GoToWaypoint Behaviour ready!");
  };

  ~GotoBehaviour(){};

  bool process_goal(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal,
                    as2_msgs::action::GoToWaypoint::Goal &new_goal) {
    if ((fabs(goal->target_pose.position.x) + fabs(goal->target_pose.position.y) +
         fabs(goal->target_pose.position.z)) == 0.0f) {
      RCLCPP_ERROR(this->get_logger(), "GotoBehaviour: Target position is not set");
      return false;
    }
    // else if (goal->target_pose.position.z <= 0.0f)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "GotoBehaviour: Target height is
    //     equal or less than 0.0 m. Please set a valid target height."); return
    //     rclcpp_action::GoalResponse::REJECT;
    // }
    new_goal.max_speed       = (goal->max_speed != 0.0f)
                                   ? goal->max_speed
                                   : this->get_parameter("default_goto_max_speed").as_double();
    new_goal.target_pose     = goal->target_pose;
    new_goal.ignore_pose_yaw = goal->ignore_pose_yaw;

    RCLCPP_INFO(this->get_logger(), "GotoBehaviour: GoToWaypoint with speed %f",
                new_goal.max_speed);

    return true;
  }

  bool on_activate(
      const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override {
    as2_msgs::action::GoToWaypoint::Goal new_goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }

    auto processed_goal = std::make_shared<const as2_msgs::action::GoToWaypoint::Goal>(new_goal);

    return goto_plugin_->on_activate(processed_goal);
  }

  bool on_modify(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) override {
    as2_msgs::action::GoToWaypoint::Goal new_goal;
    if (!process_goal(goal, new_goal)) {
      return false;
    }

    auto processed_goal = std::make_shared<const as2_msgs::action::GoToWaypoint::Goal>(new_goal);

    return goto_plugin_->on_activate(processed_goal);
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    return goto_plugin_->on_deactivate(message);
  }

  bool on_pause(const std::shared_ptr<std::string> &message) override {
    return goto_plugin_->on_pause(message);
  }

  bool on_resume(const std::shared_ptr<std::string> &message) override {
    return goto_plugin_->on_resume(message);
  }

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> &goal,
      std::shared_ptr<as2_msgs::action::GoToWaypoint::Feedback> &feedback_msg,
      std::shared_ptr<as2_msgs::action::GoToWaypoint::Result> &result_msg) override {
    return goto_plugin_->on_run(goal, feedback_msg, result_msg);
  }

  void on_excution_end(const as2_behavior::ExecutionStatus &state) override {
    return goto_plugin_->on_excution_end(state);
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<goto_base::GotoBase>> loader_;
  std::shared_ptr<goto_base::GotoBase> goto_plugin_;
};

#endif  // GOTO_BEHAVIOUR_HPP
