#ifndef WAM_NODE_SRC_BHAND_PUBLISHER_
#define WAM_NODE_SRC_BHAND_PUBLISHER_

class BhandPublishers : public rclcpp::Node {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  explicit BhandPublishers(Hand &hand, bool found_wam) : Node("BhandPublishers"), hand_(hand) {
    found_wam_ = found_wam;
    found_tactile_sensors_ = hand_.hasTactSensors();
    found_ft_torque_ = hand_.hasFingertipTorqueSensors();
    if (found_tactile_sensors_) {
      tactile_state_pub_ = this->create_publisher<bhand_msgs::msg::TactileStateArray>("/bhand/TactileStates", 100);
      RCLCPP_INFO(this->get_logger(), "Found Tactile Sensors"); 
    }
    if (found_ft_torque_) {
      finger_tip_torque_pub_ = this->create_publisher<bhand_msgs::msg::FingerTipTorques>("/bhand/FingertipTorques", 100);
      RCLCPP_INFO(this->get_logger(), "Found FingerTip Torque Sensors");
    }
    //Publishes sensors states every 20ms
    sensor_pub_timer_ = this->create_wall_timer(
      20ms, std::bind(&BhandPublishers::publishSensors, this));
		//kBhandJointNames defined in wam_publishers.h
    bhand_joint_position_msg_.position.resize(kBhandJointNames.size());
    bhand_joint_position_msg_.name = kBhandJointNames;
    /*Create a timer for publisher. Automatically triggers the callback
     * function every 2ms*/
    if (!found_wam) {
      //WAM Publishers handles bhand joint states. Only publish these if no WAM
      bhand_joint_position_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                             100);
      bhand_joint_position_pub_timer_ = this->create_wall_timer(
        2ms, std::bind(&BhandPublishers::publishJointPositions, this));
    }
  }

 protected:
  bool found_wam_;
  bool found_tactile_sensors_;
  bool found_ft_torque_;
  bhand_msgs::msg::TactileStateArray tactile_state_array_;
  bhand_msgs::msg::FingerTipTorques finger_tip_torque_;
  cf_type cf_;
  ct_type ct_;
  // function to publish barrett hand joint states
	Hand &hand_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr bhand_joint_position_pub_;
  rclcpp::Publisher<bhand_msgs::msg::TactileStateArray>::SharedPtr tactile_state_pub_;
  rclcpp::Publisher<bhand_msgs::msg::FingerTipTorques>::SharedPtr finger_tip_torque_pub_;
  void publishJointPositions();
  void publishSensors();
  rclcpp::TimerBase::SharedPtr bhand_joint_position_pub_timer_;
  rclcpp::TimerBase::SharedPtr sensor_pub_timer_;
  sensor_msgs::msg::JointState bhand_joint_position_msg_;
};

void BhandPublishers::publishJointPositions() {
  //hand_.update();
	Hand::jp_type hi = hand_.getInnerLinkPosition(); // get finger positions information
  Hand::jp_type ho = hand_.getOuterLinkPosition();
	for (int i = 0; i < 3; i++) {
		bhand_joint_position_msg_.position[i+2] = hi[i];
	}
	for (int j = 0; j < 3; j++) {
		bhand_joint_position_msg_.position[j+5] = ho[j];
	}
  bhand_joint_position_msg_.position[0] = -hi[3];
  bhand_joint_position_msg_.position[1] = hi[3];
  bhand_joint_position_pub_->publish(bhand_joint_position_msg_);

}

void BhandPublishers::publishSensors() {
  if (found_tactile_sensors_ || found_ft_torque_) {
    hand_.update();
  }
  if (found_tactile_sensors_) {
    std::vector<TactilePuck*> tactile_pucks = hand_.getTactilePucks();
    tactile_state_array_.tactile_states.resize(tactile_pucks.size());
    for (unsigned i = 0; i < tactile_pucks.size(); i++) {
      bhand_msgs::msg::TactileState msg;
      TactilePuck::v_type pressures(tactile_pucks[i]->getTactileData());
      for (int j = 0; j < pressures.size(); j++) {
        int value = (int)(pressures[j]*256.0)/102;
        msg.tactile_state[j] = pressures[j];
        int c = 0;
        int chunk;
        for (int z = 4; z >= 0; --z) {
            chunk = (value <= 7) ? value : 7;
            value -= chunk;
            switch (chunk)
            {
            case 0:
              c = c + 1;
              break;
            case 1:
              c = c + 2;
              break;
            case 2:
              c = c + 3;
              break;
            default:
              c = c + 4;
              break;
            }
            switch (chunk - 4) {
            case 3:
              c = c + 4;
              break;
            case 2:
              c = c+ 3;
              break;
            case 1:
              c = c + 2;
              break;
            case 0:
              c = c + 1;
              break;
            default:
              c = c + 0;
              break;
            }
          }
          msg.normalized_tactile_state[j] = c - 5;
      }
      tactile_state_array_.tactile_states[i] = msg;
    }
    tactile_state_pub_->publish(tactile_state_array_);
  }
  if (found_ft_torque_) {
    std::vector<int> finger_tip_torques = hand_.getFingertipTorque();
    finger_tip_torque_.torque = finger_tip_torques;
    finger_tip_torque_pub_->publish(finger_tip_torque_);
  }
}

#endif  // WAM_NODE_SRC_BHAND_PUBLISHER_
