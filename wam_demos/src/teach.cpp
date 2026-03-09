#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <fstream>
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
//TODO(ar): Add wamJointVelocityCb, identical to wamJointPositionCb, 
//just need to change msg->position to msg->velocity and create a new subscriber for velocity
class Teach : public rclcpp::Node
{
public:
  bool jp_teach;
  bool teaching;
  void stopTeaching();
  std::ofstream file;
  Teach()
  : Node("Teach")
  {
    wam_joint_position_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 500, std::bind(&Teach::wamJointPositionCb, this, std::placeholders::_1));
    file.open("test.csv");
    jp_teach = false;
  }

private:
  void wamJointPositionCb(const sensor_msgs::msg::JointState::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wam_joint_position_sub_;
};

void Teach::wamJointPositionCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (jp_teach) {
    file << msg->header.stamp.sec;
    file << "." ;
    file << msg->header.stamp.nanosec;
    for (int i = 0; i < msg->position.size(); i++) {
      file << ",";
      file << msg->position[i]; 
    }
    file << "\n";
  }
}
void startTeaching(std::shared_ptr<Teach> node) {
  node->jp_teach = true;
  rclcpp::Rate loop_rate(500);
  node->teaching = true;
  while (rclcpp::ok() && node->teaching) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  node->file.close();
}

void Teach::stopTeaching() {
  teaching = false;
  jp_teach = false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Teach>();
  std::string input;
  RCLCPP_INFO(node->get_logger(), "Press any key to start Teaching");
  getline(std::cin, input);
  RCLCPP_INFO(node->get_logger(), "Press any key to stop Teaching");
  std::thread teach_thread(startTeaching, node);
  teach_thread.detach();
  getline(std::cin, input);
  node->stopTeaching();
  return 0;
}
