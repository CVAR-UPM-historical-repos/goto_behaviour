#ifndef GOTO_BEHAVIOUR_HPP
#define GOTO_BEHAVIOUR_HPP

#include "as2_core/as2_basic_behaviour.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>

#include <pluginlib/class_loader.hpp>
#include "goto_plugin_base/goto_base.hpp"

class GotoBehaviour : public as2::BasicBehaviour<as2_msgs::action::GoToWaypoint>
{
public:
    using GoalHandleGoto = rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

    GotoBehaviour() : as2::BasicBehaviour<as2_msgs::action::GoToWaypoint>(as2_names::actions::behaviours::gotowaypoint)
    {
        this->declare_parameter("default_goto_plugin");
        this->declare_parameter("default_goto_max_speed");
        this->declare_parameter("goto_threshold");

        loader_ = std::make_shared<pluginlib::ClassLoader<goto_base::GotoBase>>("goto_plugin_base", "goto_base::GotoBase");

        try
        {
            goto_speed_ = loader_->createSharedInstance(this->get_parameter("default_goto_plugin").as_string());
            goto_speed_->initialize(this, this->get_parameter("default_goto_max_speed").as_double(), 
                                    this->get_parameter("goto_threshold").as_double());
            RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED: %s", this->get_parameter("default_goto_plugin").as_string().c_str());
        }
        catch (pluginlib::PluginlibException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "The plugin failed to load for some reason. Error: %s\n", ex.what());
        }

        RCLCPP_INFO(this->get_logger(), "GoToWaypoint Behaviour ready!");
    };

    rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal)
    {
        if ((fabs(goal->target_pose.position.x) +
             fabs(goal->target_pose.position.y) +
             fabs(goal->target_pose.position.z)) == 0.0f)
        {
            RCLCPP_ERROR(this->get_logger(), "GotoBehaviour: Target position is not set");
            return rclcpp_action::GoalResponse::REJECT;
        }
        else if (goal->target_pose.position.z <= 0.0f)
        {
            RCLCPP_ERROR(this->get_logger(), "GotoBehaviour: Target height is equal or less than 0.0 m. Please set a valid target height.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        as2_msgs::action::GoToWaypoint::Goal new_goal;
        new_goal.max_speed = (goal->max_speed != 0.0f) ? goal->max_speed : this->get_parameter("default_goto_max_speed").as_double();
        new_goal.target_pose = goal->target_pose;
        new_goal.ignore_pose_yaw = goal->ignore_pose_yaw;

        auto _goal = std::make_shared<const as2_msgs::action::GoToWaypoint::Goal>(new_goal);

        RCLCPP_INFO(this->get_logger(),
                    "GotoBehaviour: GoToWaypoint with speed %f", _goal->max_speed);

        return goto_speed_->onAccepted(_goal);
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleGoto> goal_handle)
    {
        return goto_speed_->onCancel(goal_handle);
    }

    void onExecute(const std::shared_ptr<GoalHandleGoto> goal_handle)
    {
        if (goto_speed_->onExecute(goal_handle))
        {
            RCLCPP_INFO(this->get_logger(), "GoTo succeeded");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GoTo canceled");
        }
    }

private:
    std::shared_ptr<pluginlib::ClassLoader<goto_base::GotoBase>> loader_;
    std::shared_ptr<goto_base::GotoBase> goto_speed_;
};

#endif // GOTO_BEHAVIOUR_HPP