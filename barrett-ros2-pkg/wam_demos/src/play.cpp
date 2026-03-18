#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <chrono>
#include <thread>
// Custom WAM msgs
#include "wam_msgs/msg/rt_angular_velocity.hpp"
#include "wam_msgs/msg/rt_cart_orientation.hpp"
#include "wam_msgs/msg/rt_cart_pose.hpp"
#include "wam_msgs/msg/rt_cart_position.hpp"
#include "wam_msgs/msg/rt_joint_positions.hpp"
#include "wam_msgs/msg/rt_joint_velocities.hpp"
#include "wam_msgs/msg/rt_linear_velocity.hpp"
#include "wam_msgs/msg/rt_linearand_angular_velocity.hpp"
#include "wam_msgs/srv/joint_move.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
//TODO(ar): Add play jointVelocities, tool pose and tool velocity playback
const double kRateLimits = 100;
std::string file_name = "test.csv";
void playJointPositions() {
  std::ifstream file;
  file.open(file_name);
  //Check if file open
  if (!file) {
    std::cout << "Error could not open file " << std::endl;
  }
  //Make a service jp_move_client for moving to start pose
  auto node = rclcpp::Node::make_shared("play");
  auto jp_move_client = node->create_client<wam_msgs::srv::JointMove>("/wam/moveToJointPosition");
   //Wait for service to appear
   while (!jp_move_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }  
  //RCLCPP Times
  rclcpp::Time current_msg_time, prev_msg_time;

  //Request message to send to service server
  auto joint_move_request = std::make_shared<wam_msgs::srv::JointMove::Request>();
  //move to start JP
  std::string start_jp;
  std::getline(file, start_jp, '\n');
  std::istringstream ss(start_jp);
  std::string temp;
  std::string seconds, n_seconds;

  //Extract time stamp, and store it in prev_msg_time
  std::getline(ss, seconds, '.');
  std::getline(ss, n_seconds, ',');
  current_msg_time = rclcpp::Time(std::stoi(seconds), std::stoi(n_seconds));
  prev_msg_time = current_msg_time;
  //Extract joint positions, and store them in service call
   while(std::getline(ss, temp, ',')) {
    joint_move_request->joint_state.position.push_back(std::stod(temp));
  }
  // Call Service
   auto result_future = jp_move_client->async_send_request(joint_move_request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Could not move WAM to start Pose");
    return;
  }
  //Check if server returned True
  auto result = result_future.get();
  if (!result->response) {
    RCLCPP_ERROR(node->get_logger(), "Could not move WAM to start Pose");
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  RCLCPP_INFO(node->get_logger(), "moved WAM to start pose");

  //bool first_check = true;
  //Create Publisher to publish messages
  auto rt_joint_position_pub_ = node->create_publisher<wam_msgs::msg::RTJointPositions>("/wam/RTJointPositionCMD", 100);
  //Play back trajectory
  rclcpp::Rate loop_rate(500);
  while (file.good()) {
    std::string line, seconds, n_seconds, temp;
    wam_msgs::msg::RTJointPositions rt_jp_cmd;
    std::getline(file, line, '\n');
    std::istringstream ss(line);
    //Extract Time stamp.
    std::getline(ss, seconds, '.');
    std::getline(ss, n_seconds, ',');
    //Not using time stamps
    current_msg_time = rclcpp::Time(std::stoi(seconds), std::stoi(n_seconds));
    //Extract Joint Positions from message
    auto previous_time = std::chrono::system_clock::now();
    while(std::getline(ss, temp, ',')) {
      rt_jp_cmd.joint_states.push_back(std::stod(temp));
      rt_jp_cmd.rate_limits.push_back(kRateLimits);
    }
    rt_joint_position_pub_->publish(rt_jp_cmd);
    loop_rate.sleep();
  }

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  playJointPositions();
}
