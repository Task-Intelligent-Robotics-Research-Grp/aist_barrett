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
    using vector3_t     = geometry_msgs::msg::Vector3;
    using quaternion_t  = geometry_msgs::msg::Quaternion;

  public:
    explicit
    WamPublishers(barrett::systems::Wam<DOF>* wam, barrett::Hand* hand)
        :Node("WamPublishers"), wam_(wam), hand_(hand), first_publish_(true),
         joint_state_pub_(
             create_publisher<joint_state_t>("/joint_states", 100)),
         tool_pose_pub_(
             create_publisher<pose_t>("/wam/ToolPose", 100)),
         tool_velocity_pub_(
             create_publisher<twist_t>("/wam/toolVelocity", 100)),
         trajectory_status_pub_(
             create_publisher<bool_t>("/wam/trajectoryStatus", 100))
    {
        const size_t    hand_dof = (hand_ ? kBhandJointNames.size() : 0);

        joint_state_.position.resize(DOF + hand_dof);
        joint_state_.velocity.resize(DOF + hand_dof);
        joint_state_.effort.resize(DOF + hand_dof);
        joint_state_.name.resize(DOF + hand_dof);
        for (size_t i = 0; i < DOF; ++i)
            joint_state_.name.at(i) = "q" + std::to_string(i + 1);
        for (size_t i = DOF; i < DOF + hand_dof; ++i)
            joint_state_.name.at(i) = kBhandJointNames.at(i - DOF);
    }

    void        publishJointState();
    void        publishToolPoseAndVelocity();

  private:
    static vector3_t    toRPY(const quaternion_t& q);

  private:
    barrett::systems::Wam<DOF>* const   wam_;
    barrett::Hand* const                hand_;

    bool                                first_publish_;
    joint_state_t                       joint_state_;
    pose_t                              prev_pose_;
    const pub_p<joint_state_t>          joint_state_pub_;
    const pub_p<pose_t>                 tool_pose_pub_;
    const pub_p<twist_t>                tool_velocity_pub_;
    const pub_p<bool_t>                 trajectory_status_pub_;

    timer_t                             pub_timer_;
};

template <size_t DOF> typename WamPublishers<DOF>::vector3_t
WamPublishers<DOF>::toRPY(const quaternion_t& q)
{
    vector3_t   rpy;
    tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(rpy.x,
                                                               rpy.y, rpy.z);
    return rpy;
}

template <size_t DOF> void
WamPublishers<DOF>::publishJointState()
{
  // Get arm joint states and current time
    const auto& current_joint_position = wam_->getJointPositions();
    const auto& current_joint_velocity = wam_->getJointVelocities();
    const auto& current_joint_torque   = wam_->getJointTorques();
    joint_state_.header.stamp = rclcpp::Node::now();

    int         no_zeros = 0;
    for (size_t i = 0; i < DOF; ++i)
    {
        joint_state_.position.at(i) = current_joint_position(i);
        joint_state_.velocity.at(i) = current_joint_velocity(i);
        joint_state_.effort.at(i)   = current_joint_torque(i);

        if ((joint_state_.velocity.at(i) >= -0.07) &&
            (joint_state_.velocity.at(i) <=  0.07))
        {
            ++no_zeros;
        }
    }

  // Publish trajectory status
    bool_t      trajectory_status;
    trajectory_status.data = (no_zeros != DOF);
    trajectory_status_pub_->publish(trajectory_status);

  // If hand is present, append hand joint states
    if (hand_)
    {
        hand_->update();
        const auto&     hi = hand_->getInnerLinkPosition();
        const auto&     ho = hand_->getOuterLinkPosition();
        for (size_t i = 0; i < 3; ++i)
        {
            joint_state_.position[i + 2 + DOF] = hi[i];
            joint_state_.position[i + 5 + DOF] = ho[i];
        }
        joint_state_.position[DOF]   =  hi[3];
        joint_state_.position[DOF+1] = -hi[3];
    }

  // Publish joint states
    joint_state_pub_ ->publish(joint_state_);
}

template <size_t DOF> void
WamPublishers<DOF>::publishToolPoseAndVelocity()
{
  // Create and publish cartesian pose
    pose_t              pose;
    const pose_type     tool_pose = wam_->getToolPose();
    pose.header.stamp       = rclcpp::Node::now();
    pose.pose.position.x    = tool_pose.get<0>()(0);
    pose.pose.position.y    = tool_pose.get<0>()(1);
    pose.pose.position.z    = tool_pose.get<0>()(2);
    pose.pose.orientation.x = tool_pose.get<1>().x();
    pose.pose.orientation.y = tool_pose.get<1>().y();
    pose.pose.orientation.z = tool_pose.get<1>().z();
    pose.pose.orientation.w = tool_pose.get<1>().w();
    tool_pose_pub_->publish(pose);

  // Create and publish twist
    twist_t     twist;
    twist.header.stamp = pose.header.stamp;
    if (first_publish_)
    {
        twist.twist.linear.x  = 0;
        twist.twist.linear.y  = 0;
        twist.twist.linear.z  = 0;
        twist.twist.angular.x = 0;
        twist.twist.angular.y = 0;
        twist.twist.angular.z = 0;

        first_publish_ = false;
    }
    else
    {
        using time_t = rclcpp::Time;

        const auto  dt = (time_t(pose      .header.stamp) -
                          time_t(prev_pose_.header.stamp)).seconds();
        twist.twist.linear.x  = (pose      .pose.position.x -
                                 prev_pose_.pose.position.x) / dt;
        twist.twist.linear.y  = (pose      .pose.position.y -
                                 prev_pose_.pose.position.y) / dt;
        twist.twist.linear.z  = (pose      .pose.position.z -
                                 prev_pose_.pose.position.z) / dt;
        const auto  rpy       = toRPY(pose      .pose.orientation);
        const auto  prev_rpy  = toRPY(prev_pose_.pose.orientation);
        twist.twist.angular.x = (rpy.x - prev_rpy.x) / dt;
        twist.twist.angular.y = (rpy.y - prev_rpy.y) / dt;
        twist.twist.angular.z = (rpy.z - prev_rpy.z) / dt;
    }
    tool_velocity_pub_->publish(twist);

  // Keep curerent pose for calculating next velocity
    prev_pose_ = pose;
}
}       // namespace wam_node
