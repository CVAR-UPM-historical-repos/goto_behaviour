#ifndef GOTO_BASE_HPP
#define GOTO_BASE_HPP

#include "as2_core/node.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/names/topics.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include <as2_msgs/action/go_to_waypoint.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>

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
            pose_mutex_.lock();
            actual_position_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                                msg->pose.pose.position.z};
            
            actual_q_ = {msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, 
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z};
            pose_mutex_.unlock();

            distance_measured_ = true;
            this->actual_distance_to_goal_ = (actual_position_ - desired_position_).norm();
            this->actual_speed_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                                    msg->twist.twist.linear.y,
                                                    msg->twist.twist.linear.z).norm();
        };

    protected:
        as2::Node *node_ptr_;
        float goal_threshold_;

        std::mutex pose_mutex_;
        Eigen::Vector3d actual_position_;
        Eigen::Quaterniond actual_q_;

        std::atomic<bool> distance_measured_;
        std::atomic<float> actual_distance_to_goal_;
        std::atomic<float> actual_speed_;

        Eigen::Vector3d desired_position_;
        float desired_speed_ = 0.0;
        bool ignore_yaw_;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    }; // GotoBase class

}  // goto_base namespace

#endif // GOTO_BASE_HPP