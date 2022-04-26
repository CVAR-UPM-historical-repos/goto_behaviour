#ifndef GOTO_BASE_HPP
#define GOTO_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace goto_base
{
    class GotoBase
    {
    public:
        using GoalHandleGoto = rclcpp_action::ServerGoalHandle<as2_msgs::action::GoToWaypoint>;

        void initialize(as2::Node *node_ptr, float goal_threshold)
        {
            node_ptr_ = node_ptr;
            goal_threshold_ = goal_threshold;
            odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
                node_ptr_->generate_global_name(as2_names::topics::self_localization::odom), as2_names::topics::self_localization::qos,
                std::bind(&GotoBase::odomCb, this, std::placeholders::_1));

            this->ownInit(node_ptr_);
        };

        virtual rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::GoToWaypoint::Goal> goal) = 0;
        virtual rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleGoto> goal_handle) = 0;
        virtual bool onExecute(const std::shared_ptr<GoalHandleGoto> goal_handle) = 0;

        virtual ~GotoBase(){};

    protected:
        GotoBase(){};

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
        float goal_threshold_;

        std::atomic<float> actual_heigth_;
        std::atomic<float> actual_z_speed_;

        float desired_speed_ = 0.0;
        float desired_height_ = 0.0;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    }; // GotoBase class

}  // goto_base namespace

#endif // GOTO_BASE_HPP