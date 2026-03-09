#ifndef WAM_NODE_SRC_WAM_PUBLISHER_H_
#define WAM_NODE_SRC_WAM_PUBLISHER_H_
#include <chrono>
#include <barrett/math/kinematics.h>
using namespace std::chrono_literals;

std::vector<std::string> kBhandJointNames = {
    "bhand_j11_joint", "bhand_j21_joint", "bhand_j12_joint", "bhand_j22_joint",
    "bhand_j32_joint", "bhand_j13_joint", "bhand_j23_joint", "bhand_j33_joint"};

template <size_t DOF>
class WamPublishers : public rclcpp::Node {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
 public:
  void publishJointPositions();
  void publishCartPose();
  void publishToolVelocity();
  void publishJointVelocities();
  explicit WamPublishers(systems::Wam<DOF> &wam, ProductManager &pm, bool found_hand)
      : Node("WamPublishers"), wam_(wam), pm_(pm) {
    first_publish_ = true;
    found_hand_ = found_hand;
    joint_position_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 100);
    trajectory_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("wam/trajectoryStatus", 100);
    joint_velocity_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/wam/jointVelocity", 100);
    tool_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/wam/ToolPose", 100);
    tool_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/wam/toolVelocity", 100);
    if (found_hand_) {  // if hand is present, publish hand joint states
      hand_ = pm.getHand();
      joint_position_msg_.position.resize(DOF + kBhandJointNames.size());
      joint_position_msg_.effort.resize(DOF + kBhandJointNames.size());
      joint_velocity_msg_.velocity.resize(DOF + kBhandJointNames.size());
      joint_position_msg_.name.resize(DOF + kBhandJointNames.size());
      for (int i = 0; i < (int)DOF; i++) {
        joint_position_msg_.name.at(i) = "q" + std::to_string(i + 1);
      }
      for (int j = DOF; j < (int)(DOF+kBhandJointNames.size()); j++) { //publish Bhand joint states
        joint_position_msg_.name.at(j) = kBhandJointNames.at(j-DOF);
      }
    } else {
      joint_position_msg_.position.resize(DOF);
      joint_position_msg_.effort.resize(DOF);
      joint_velocity_msg_.velocity.resize(DOF);
      joint_position_msg_.name.resize(DOF);
      for (int i = 0; i < (int)DOF; i++) {
        joint_position_msg_.name.at(i) = "q" + std::to_string(i + 1);
      }
    }
  }

 protected:
  bool found_hand_;
  Hand *hand_;
  cf_type cf_;
  ct_type ct_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_position_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tool_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_status_pub_;
  systems::Wam<DOF> &wam_;
  ProductManager &pm_;
  // Libbarrett Data Types
  pose_type current_tool_pose_;
  pose_type prev_tool_pose_;
  jp_type current_joint_position_;
  jv_type current_joint_velocity_;
  jt_type current_joint_torque_;
  // ROS2 Data Types
  rclcpp::Time current_velocity_pub_time_, prev_velocity_pub_time_;
  sensor_msgs::msg::JointState joint_position_msg_;
  sensor_msgs::msg::JointState joint_velocity_msg_;
  rclcpp::TimerBase::SharedPtr joint_position_pub_timer_, tool_pose_pub_timer_,
      tool_velocity_pub_timer_, joint_velocity_pub_timer_;
  bool first_publish_;

  /** Returns Tool Velocity.
   * Tool Velocity =
   * (current_tool_pose_-previous_tool_pose_)/(current_velocity_pub_time_-prev_velocity_pub_time_)
   */
  geometry_msgs::msg::TwistStamped calcToolVelocity();
  // Simple Function for converting Quaternion to RPY
  math::Vector<3>::type toRPY(Eigen::Quaterniond inquat);
};

template <size_t DOF>
math::Vector<3>::type WamPublishers<DOF>::toRPY(Eigen::Quaterniond inquat) {
  math::Vector<3>::type rpy;
  tf2::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
  tf2::Matrix3x3(q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

template <size_t DOF>
void WamPublishers<DOF>::publishJointPositions() {
  current_joint_position_ = wam_.getJointPositions();
  current_joint_torque_ = wam_.getJointTorques();
  for (int i = 0; i < (int)DOF; i++) {
    joint_position_msg_.position.at(i) = current_joint_position_(i);
    joint_position_msg_.effort.at(i) = current_joint_torque_(i);
  }
  // If hand is present, publish hand joint states
  if (found_hand_) {
    hand_->update();
    Hand::jp_type hi = hand_->getInnerLinkPosition();  // get finger positions information
    Hand::jp_type ho = hand_->getOuterLinkPosition();
    for (int i = 0; i < 3; i++) {
      joint_position_msg_.position[i + 2 + DOF] = hi[i];
    }
    for (int j = 0; j < 3; j++) {
      joint_position_msg_.position[j + 5 + DOF] = ho[j];
    }
    joint_position_msg_.position[DOF] = hi[3];
    joint_position_msg_.position[DOF+1] = -hi[3];
  }
  joint_position_msg_.header.stamp = rclcpp::Node::now();
  joint_position_pub_->publish(joint_position_msg_);
}

template <size_t DOF>
void WamPublishers<DOF>::publishJointVelocities() {
  current_joint_velocity_ = wam_.getJointVelocities();
  std_msgs::msg::Bool trajectory_status_msg;
  int no_zeros = 0;
  for (int i = 0; i < (int)DOF; i++) {
    joint_velocity_msg_.velocity.at(i) = current_joint_velocity_(i);
    if ((joint_velocity_msg_.velocity.at(i) >= -0.07) && (joint_velocity_msg_.velocity.at(i) <= 0.07)) {
      no_zeros = no_zeros + 1;
    }
  }
  if (no_zeros == DOF) {
    trajectory_status_msg.data = false;
    trajectory_status_pub_->publish(trajectory_status_msg);
  } else {
    trajectory_status_msg.data = true;
    trajectory_status_pub_->publish(trajectory_status_msg);
  }
  joint_velocity_msg_.header.stamp = rclcpp::Node::now();
  joint_velocity_pub_->publish(joint_velocity_msg_);
}

template <size_t DOF>
void WamPublishers<DOF>::publishCartPose() {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = rclcpp::Node::now();
  current_tool_pose_ = wam_.getToolPose();
  pose_msg.pose.position.x = current_tool_pose_.get<0>()(0);
  pose_msg.pose.position.y = current_tool_pose_.get<0>()(1);
  pose_msg.pose.position.z = current_tool_pose_.get<0>()(2);
  pose_msg.pose.orientation.x = current_tool_pose_.get<1>().x();
  pose_msg.pose.orientation.y = current_tool_pose_.get<1>().y();
  pose_msg.pose.orientation.z = current_tool_pose_.get<1>().z();
  pose_msg.pose.orientation.w = current_tool_pose_.get<1>().w();
  tool_pose_pub_->publish(pose_msg);
}

template <size_t DOF>
geometry_msgs::msg::TwistStamped WamPublishers<DOF>::calcToolVelocity() {
  geometry_msgs::msg::TwistStamped tool_velocity_msg;
  tool_velocity_msg.twist.linear.x =
      (current_tool_pose_.get<0>()(0) - prev_tool_pose_.get<0>()(0)) /
      ((current_velocity_pub_time_.nanoseconds() -
        prev_velocity_pub_time_.nanoseconds())) * 1000000000; //convert nanoseconds to seconds
  tool_velocity_msg.twist.linear.y =
      (current_tool_pose_.get<0>()(1) - prev_tool_pose_.get<0>()(1)) /
      ((current_velocity_pub_time_.nanoseconds() -
        prev_velocity_pub_time_.nanoseconds())) * 1000000000;
  tool_velocity_msg.twist.linear.z =
      (current_tool_pose_.get<0>()(2) - prev_tool_pose_.get<0>()(2)) /
      ((current_velocity_pub_time_.nanoseconds() -
        prev_velocity_pub_time_.nanoseconds())) * 1000000000;

  math::Vector<3>::type current_tool_rpy = toRPY(current_tool_pose_.get<1>());
  math::Vector<3>::type previous_tool_rpy = toRPY(prev_tool_pose_.get<1>());
  tool_velocity_msg.twist.angular.x = (current_tool_rpy(0) - previous_tool_rpy(0)) /
                                ((current_velocity_pub_time_.nanoseconds() -
                                  prev_velocity_pub_time_.nanoseconds())) * 1000000000;
  tool_velocity_msg.twist.angular.y = (current_tool_rpy(1) - previous_tool_rpy(1)) /
                                ((current_velocity_pub_time_.nanoseconds() -
                                  prev_velocity_pub_time_.nanoseconds())) * 1000000000;
  tool_velocity_msg.twist.angular.z = (current_tool_rpy(2) - previous_tool_rpy(2)) /
                                ((current_velocity_pub_time_.nanoseconds() -
                                  prev_velocity_pub_time_.nanoseconds())) * 1000000000;
  tool_velocity_msg.header.stamp = rclcpp::Node::now();
  return tool_velocity_msg;
}

template <size_t DOF>
void WamPublishers<DOF>::publishToolVelocity() {
  geometry_msgs::msg::TwistStamped tool_velocity_msg_;
  if (first_publish_) {
    first_publish_ = false;
    prev_tool_pose_ = current_tool_pose_;
    prev_velocity_pub_time_ = rclcpp::Node::now();
    tool_velocity_msg_.header.stamp = rclcpp::Node::now();
    tool_velocity_msg_.twist.linear.x = 0;
    tool_velocity_msg_.twist.linear.y = 0;
    tool_velocity_msg_.twist.linear.z = 0;
    tool_velocity_msg_.twist.angular.x = 0;
    tool_velocity_msg_.twist.angular.y = 0;
    tool_velocity_msg_.twist.angular.z = 0;
    tool_velocity_pub_->publish(tool_velocity_msg_);
  } else {
    current_velocity_pub_time_ = rclcpp::Node::now();
    tool_velocity_pub_->publish(calcToolVelocity());
    prev_tool_pose_ = current_tool_pose_;
    prev_velocity_pub_time_ = current_velocity_pub_time_;
  }
}

#endif  // WAM_NODE_SRC_WAM_PUBLISHER_H_