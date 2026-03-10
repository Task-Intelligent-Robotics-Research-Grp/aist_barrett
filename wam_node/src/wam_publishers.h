#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <barrett/math/kinematics.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

namespace wam_node
{
std::vector<std::string> kBhandJointNames =
{
    "bhand_j11_joint", "bhand_j21_joint", "bhand_j12_joint", "bhand_j22_joint",
    "bhand_j32_joint", "bhand_j13_joint", "bhand_j23_joint", "bhand_j33_joint"
};

template <size_t DOF>
class WamPublishers : public rclcpp::Node
{
  private:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    template <class MSG>
    using pub_p         = typename rclcpp::Publisher<MSG>::SharedPtr;
    using timer_p       = rclcpp::TimerBase::SharedPtr;

    using joint_state_t = sensor_msgs::msg::JointState;
    using bool_t        = std_msgs::msg::Bool;
    using pose_t        = geometry_msgs::msg::PoseStamped;
    using twist_t       = geometry_msgs::msg::TwistStamped;

    using vector3_t     = barrett::math::Vector<3>::type;

  public:
    explicit WamPublishers(barrett::systems::Wam<DOF>* wam,
                           barrett::Hand* hand)
        :Node("WamPublishers"), wam_(wam), hand_(hand), first_publish_(true),
         joint_state_pub_(
             create_publisher<joint_state_t>("/joint_states", 100)),
         joint_velocity_pub_(
             create_publisher<joint_state_t>("/wam/jointVelocity", 100)),
         tool_pose_pub_(
             create_publisher<pose_t>("/wam/ToolPose", 100)),
         tool_velocity_pub_(
             create_publisher<twist_t>("/wam/toolVelocity", 100)),
         trajectory_status_pub_(
             create_publisher<bool_t>("/wam/trajectoryStatus", 100))
    {
        const size_t    hand_dof = (hand_ ? kBhandJointNames.size() : 0);

        joint_state_.position.resize(DOF + hand_dof);
        joint_state_.effort.resize(DOF + hand_dof);
        joint_velocity_.velocity.resize(DOF + hand_dof);
        joint_state_.name.resize(DOF + hand_dof);
        for (size_t i = 0; i < DOF; ++i)
            joint_state_.name.at(i) = "q" + std::to_string(i + 1);
        for (size_t i = DOF; i < DOF + hand_dof; ++i)
            joint_state_.name.at(i) = kBhandJointNames.at(i - DOF);
    }

    void publishJointState();
    void publishCartPose();
    void publishToolVelocity();
    void publishJointVelocities();

  protected:
  /** Returns Tool Velocity.
   * Tool Velocity =
   * (current_tool_pose_-previous_tool_pose_)/(current_velocity_pub_time_-prev_velocity_pub_time_)
   */
    twist_t calcToolVelocity();
  // Simple Function for converting Quaternion to RPY
    vector3_t   toRPY(Eigen::Quaterniond inquat);

  protected:
    barrett::systems::Wam<DOF>* const   wam_;
    barrett::Hand* const                hand_;
    bool                                first_publish_;
    // cf_type cf_;
    // ct_type ct_;

    const pub_p<joint_state_t>  joint_state_pub_;
    const pub_p<joint_state_t>  joint_velocity_pub_;
    const pub_p<pose_t>         tool_pose_pub_;
    const pub_p<twist_t>        tool_velocity_pub_;
    const pub_p<bool_t>         trajectory_status_pub_;

  // Libbarrett Data Types
    pose_type                   current_tool_pose_;
    pose_type                   prev_tool_pose_;
    jp_type                     current_joint_position_;
    jv_type                     current_joint_velocity_;
    jt_type                     current_joint_torque_;

  // ROS2 Data Types
    rclcpp::Time                current_velocity_pub_time_,
                                prev_velocity_pub_time_;
    joint_state_t               joint_state_;
    joint_state_t               joint_velocity_;
    timer_t                     joint_state_pub_timer_,
                                joint_velocity_pub_timer_,
                                tool_pose_pub_timer_,
                                tool_velocity_pub_timer_;
};

template <size_t DOF> typename WamPublishers<DOF>::vector3_t
WamPublishers<DOF>::toRPY(Eigen::Quaterniond inquat)
{
    vector3_t   rpy;
    tf2::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
    tf2::Matrix3x3(q).getRPY(rpy[0], rpy[1], rpy[2]);
    return rpy;
}

template <size_t DOF> void
WamPublishers<DOF>::publishJointState()
{
    current_joint_position_ = wam_->getJointPositions();
    current_joint_torque_   = wam_->getJointTorques();
    for (int i = 0; i < (int)DOF; i++)
    {
        joint_state_.position.at(i) = current_joint_position_(i);
        joint_state_.effort.at(i) = current_joint_torque_(i);
    }

  // If hand is present, publish hand joint states
    if (hand_)
    {
        hand_->update();
        barrett::Hand::jp_type hi = hand_->getInnerLinkPosition();  // get finger positions information
        barrett::Hand::jp_type ho = hand_->getOuterLinkPosition();
        for (size_t i = 0; i < 3; ++i)
            joint_state_.position[i + 2 + DOF] = hi[i];
        for (size_t i = 0; i < 3; ++i)
            joint_state_.position[i + 5 + DOF] = ho[i];
        joint_state_.position[DOF]   =  hi[3];
        joint_state_.position[DOF+1] = -hi[3];
    }
    joint_state_.header.stamp = rclcpp::Node::now();
    joint_state_pub_->publish(joint_state_);
}

template <size_t DOF> void
WamPublishers<DOF>::publishJointVelocities()
{
    current_joint_velocity_ = wam_->getJointVelocities();
    bool_t trajectory_status_msg;
    int no_zeros = 0;
    for (size_t i = 0; i < DOF; ++i)
    {
        joint_velocity_.velocity.at(i) = current_joint_velocity_(i);
        if ((joint_velocity_.velocity.at(i) >= -0.07) &&
            (joint_velocity_.velocity.at(i) <=  0.07))
        {
            ++no_zeros;
        }
    }

    trajectory_status_msg.data = (no_zeros != DOF);
    trajectory_status_pub_->publish(trajectory_status_msg);

    joint_velocity_.header.stamp = rclcpp::Node::now();
    joint_velocity_pub_->publish(joint_velocity_);
}

template <size_t DOF> void
WamPublishers<DOF>::publishCartPose()
{
    pose_t      pose;
    pose.header.stamp = rclcpp::Node::now();
    current_tool_pose_ = wam_->getToolPose();
    pose.pose.position.x = current_tool_pose_.get<0>()(0);
    pose.pose.position.y = current_tool_pose_.get<0>()(1);
    pose.pose.position.z = current_tool_pose_.get<0>()(2);
    pose.pose.orientation.x = current_tool_pose_.get<1>().x();
    pose.pose.orientation.y = current_tool_pose_.get<1>().y();
    pose.pose.orientation.z = current_tool_pose_.get<1>().z();
    pose.pose.orientation.w = current_tool_pose_.get<1>().w();

    tool_pose_pub_->publish(pose);
}

template <size_t DOF> geometry_msgs::msg::TwistStamped
WamPublishers<DOF>::calcToolVelocity()
{
    twist_t twist;
    twist.twist.linear.x
        = (current_tool_pose_.get<0>()(0) - prev_tool_pose_.get<0>()(0)) /
          ((current_velocity_pub_time_.nanoseconds() -
            prev_velocity_pub_time_.nanoseconds())) * 1000000000; //convert nanoseconds to seconds
    twist.twist.linear.y
        = (current_tool_pose_.get<0>()(1) - prev_tool_pose_.get<0>()(1)) /
          ((current_velocity_pub_time_.nanoseconds() -
            prev_velocity_pub_time_.nanoseconds())) * 1000000000;
    twist.twist.linear.z
        = (current_tool_pose_.get<0>()(2) - prev_tool_pose_.get<0>()(2)) /
          ((current_velocity_pub_time_.nanoseconds() -
            prev_velocity_pub_time_.nanoseconds())) * 1000000000;

    const auto  current_tool_rpy  = toRPY(current_tool_pose_.get<1>());
    const auto  previous_tool_rpy = toRPY(prev_tool_pose_.get<1>());
    twist.twist.angular.x
        = (current_tool_rpy(0) - previous_tool_rpy(0)) /
          ((current_velocity_pub_time_.nanoseconds() -
            prev_velocity_pub_time_.nanoseconds())) * 1000000000;
    twist.twist.angular.y
        = (current_tool_rpy(1) - previous_tool_rpy(1)) /
          ((current_velocity_pub_time_.nanoseconds() -
            prev_velocity_pub_time_.nanoseconds())) * 1000000000;
    twist.twist.angular.z
        = (current_tool_rpy(2) - previous_tool_rpy(2)) /
          ((current_velocity_pub_time_.nanoseconds() -
            prev_velocity_pub_time_.nanoseconds())) * 1000000000;
    twist.header.stamp = rclcpp::Node::now();

    return twist;
}

template <size_t DOF> void
WamPublishers<DOF>::publishToolVelocity()
{
    twist_t twist;

    if (first_publish_)
    {
        first_publish_ = false;

        prev_tool_pose_ = current_tool_pose_;
        prev_velocity_pub_time_ = rclcpp::Node::now();
        twist.header.stamp = rclcpp::Node::now();
        twist.twist.linear.x = 0;
        twist.twist.linear.y = 0;
        twist.twist.linear.z = 0;
        twist.twist.angular.x = 0;
        twist.twist.angular.y = 0;
        twist.twist.angular.z = 0;

        tool_velocity_pub_->publish(twist);
    }
    else
    {
        current_velocity_pub_time_ = rclcpp::Node::now();

        tool_velocity_pub_->publish(calcToolVelocity());

        prev_tool_pose_ = current_tool_pose_;
        prev_velocity_pub_time_ = current_velocity_pub_time_;
    }
}
}       // namespace wam_node
