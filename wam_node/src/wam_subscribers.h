#ifndef WAM_NODE_SRC_WAM_SUBSCRIBER_H_
#define WAM_NODE_SRC_WAM_SUBSCRIBER_H_
#include <boost/tuple/tuple.hpp>

#include <barrett/log.h>
static const int kPublishFrequency = 500;
rclcpp::Time kRtMsgTimeout(0,500000000);  //Timeout for RT messages: 0.5 seconds
static const double kCartesianSpeed = 0.03; // Default Cartesian Velocity
const double kDefaultRateLimits = 1;        // 1m/s or 1rad/s

template <size_t DOF>
class WamSubscribers : public rclcpp::Node {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 public:
  void updateRT();
  bool logging;
  void startLogging();
  explicit WamSubscribers(systems::Wam<DOF>& wam, ProductManager& pm)
      : Node("WamSubscribers"),
        wam_(wam),
        pm_(pm),
        ramp_(NULL, kCartesianSpeed),
        ramp1_(NULL, kCartesianSpeed) {
    linear_velocity_mag_ = kCartesianSpeed;
    angular_velocity_mag_ = kCartesianSpeed;
    pm_.getExecutionManager()->startManaging(ramp_);
    pm_.getExecutionManager()->startManaging(ramp1_);
    rt_joint_position_status_ = false;
    rt_linear_angular_velocity_status_ = false;
    rt_linear_velocity_status_ = false;
    rt_angular_velocity_status_ = false;
    rt_cart_pose_status_ = false;
    rt_cart_position_status_ = false;
    rt_cart_orientation_status_ = false;
    rt_joint_velocity_status_ = false;
    new_rt_cmd_ = false;
    rt_joint_position_sub_ =
        this->create_subscription<wam_msgs::msg::RTJointPositions>(
            "/wam/RTJointPositionCMD", 100,
            std::bind(&WamSubscribers::rtJointPositionCb, this,
                      std::placeholders::_1));
    rt_linear_angular_velocity_sub_ =
        this->create_subscription<wam_msgs::msg::RTLinearandAngularVelocity>(
            "/wam/RTLinearandAngularVelocityCMD", 100,
            std::bind(&WamSubscribers::rtLinearAngularVelocityCb, this,
                      std::placeholders::_1));
    rt_angular_velocity_sub_ = this->create_subscription<wam_msgs::msg::RTAngularVelocity>(
        "/wam/RTAngularVelocityCMD", 100,
        std::bind(&WamSubscribers::rtAngularVelocityCb, this, std::placeholders::_1));
    rt_linear_velocity_sub_ = this->create_subscription<wam_msgs::msg::RTLinearVelocity>(
        "/wam/RTLinearVelocityCMD", 100,
        std::bind(&WamSubscribers::rtLinearVelocityCb, this, std::placeholders::_1));
    rt_cart_position_sub_ = this->create_subscription<wam_msgs::msg::RTCartPosition>(
        "/wam/RTCartPositionCMD", 100,
        std::bind(&WamSubscribers::rtCartPositionCb, this, std::placeholders::_1));
    rt_cart_pose_sub_ = this->create_subscription<wam_msgs::msg::RTCartPose>(
        "/wam/RTCartPoseCMD", 100,
        std::bind(&WamSubscribers::rtCartPoseCb, this, std::placeholders::_1));
    rt_cart_orientation_sub_ =
        this->create_subscription<wam_msgs::msg::RTCartOrientation>(
            "/wam/RTCartOrientationCMD", 100,
            std::bind(&WamSubscribers::rtCartOrientationCb, this,
                      std::placeholders::_1));
    rt_joint_velocity_sub_ =
        this->create_subscription<wam_msgs::msg::RTJointVelocities>(
            "/wam/RTJointVelocityCMD", 100,
            std::bind(&WamSubscribers::rtJointVelocityCb, this,
                      std::placeholders::_1));
  }
protected:
  rclcpp::Subscription<wam_msgs::msg::RTJointPositions>::SharedPtr
      rt_joint_position_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTLinearandAngularVelocity>::SharedPtr
      rt_linear_angular_velocity_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTAngularVelocity>::SharedPtr
      rt_angular_velocity_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTLinearVelocity>::SharedPtr
      rt_linear_velocity_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTCartPosition>::SharedPtr
      rt_cart_position_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTCartPose>::SharedPtr rt_cart_pose_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTCartOrientation>::SharedPtr
      rt_cart_orientation_sub_;
  rclcpp::Subscription<wam_msgs::msg::RTJointVelocities>::SharedPtr
      rt_joint_velocity_sub_;
  /** Function to convert Quaternion to RPY, used for Angular Velocities*/
  math::Vector<3>::type toRPY(Eigen::Quaterniond inquat);
  systems::Wam<DOF> &wam_;
  ProductManager& pm_;
  //RT Status indicators
  bool rt_joint_position_status_; 
  bool rt_linear_angular_velocity_status_; 
  bool rt_linear_velocity_status_;
  bool rt_angular_velocity_status_;
  bool rt_cart_pose_status_; 
  bool rt_cart_position_status_; 
  bool rt_cart_orientation_status_; 
  bool rt_joint_velocity_status_; 
  bool new_rt_cmd_; //Indicator for new command
  double linear_velocity_mag_, angular_velocity_mag_;
  //Message Times
  rclcpp::Time rt_joint_position_msg_time_;
  rclcpp::Time prev_joint_position_msg_time_;
  rclcpp::Time rt_linear_angular_velocity_msg_time_; 
  rclcpp::Time rt_linear_velocity_msg_time_;
  rclcpp::Time rt_angular_velocity_msg_time_;
  rclcpp::Time rt_cart_pose_msg_time_; 
  rclcpp::Time rt_cart_position_msg_time_; 
  rclcpp::Time rt_cart_orientation_msg_time_;
  rclcpp::Time rt_joint_velocity_msg_time_;
  jp_type jp_cmd_, jp_rate_limits_;
  jp_type prev_jp_cmd_;
  cp_type rt_linear_velocity_cmd_, rt_cartesian_position_cmd_,
      rt_cartesian_position_rate_limits_;
  jv_type rt_joint_velocity_cmd_;
  math::Vector<3>::type rt_angular_velocity_cmd_;
  Eigen::Quaterniond rt_cartesian_orientation_cmd_;
  //Libbarrett Systems
  Multiplier<double, cp_type, cp_type> linear_velocity_mult_; //custom libbarrett system, defined in custom_systems.h
  Multiplier<double, math::Vector<3>::type, math::Vector<3>::type> angular_velocity_mult_;
  systems::TupleGrouper<cp_type, Eigen::Quaterniond> rt_pose_cmd_;
  systems::ExposedOutput<jp_type> jp_track_;
  systems::FirstOrderFilter<jp_type> jp_filter_;
  systems::TupleGrouper<double, jp_type> tg_;
  typedef boost::tuple<double, jp_type> tuple_type_;
  systems::RateLimiter<jp_type> joint_rate_limiter_;
  systems::RateLimiter<cp_type> cartesian_rate_limiter_;
  /**Rate limiter does not work with Eigen::Quaterniond*/
  //systems::RateLimiter<Eigen::Quaterniond> co_rl;
  //Eigen::Quaterniond rt_cartesian_orientation_rl_;
  systems::Ramp ramp_, ramp1_;
  systems::Summer<cp_type> cart_position_summer_;
  systems::Summer<math::Vector<3>::type> cart_orientation_summer_;
  systems::ExposedOutput<cp_type> linear_vel_direction_track_, current_cp_track_, cp_cmd_track_; 
  systems::ExposedOutput<math::Vector<3>::type> angular_vel_direction_track_, current_rpy_track_; 
  systems::ExposedOutput<Eigen::Quaterniond> current_quat_track_, quat_cmd_track_;
  systems::ExposedOutput<jv_type> jv_track_; //system for RT Joint Velocity command
  ToQuaternion to_quat_; //custom libbarrett system, defined in custom_systems.h
  //subscription callbacks. Sets target velocities and positions, used by updateRT()
  void rtJointPositionCb(const wam_msgs::msg::RTJointPositions::SharedPtr msg);
  void rtLinearAngularVelocityCb(const wam_msgs::msg::RTLinearandAngularVelocity::SharedPtr msg);
  void rtAngularVelocityCb(const wam_msgs::msg::RTAngularVelocity::SharedPtr msg);
  void rtLinearVelocityCb(const wam_msgs::msg::RTLinearVelocity::SharedPtr msg);
  void rtCartPositionCb(const wam_msgs::msg::RTCartPosition::SharedPtr msg);
  void rtCartPoseCb(const wam_msgs::msg::RTCartPose::SharedPtr msg);
  void rtCartOrientationCb(const wam_msgs::msg::RTCartOrientation::SharedPtr msg);
  void rtJointVelocityCb(const wam_msgs::msg::RTJointVelocities::SharedPtr msg);

};

template<size_t DOF>
math::Vector<3>::type WamSubscribers<DOF>::toRPY(Eigen::Quaterniond inquat)
{
  math::Vector<3>::type rpy;
  tf2::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
  tf2::Matrix3x3(q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}
template <size_t DOF>
void WamSubscribers<DOF>::rtJointPositionCb(
    const wam_msgs::msg::RTJointPositions::SharedPtr msg) {
  if (msg->joint_states.size() != DOF) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid Joint Position Command received. Please enter %ld "
                 "Joint Positions and Rate Limits",
                 DOF);
    return;
  }
  if (!rt_joint_position_status_) {
    rt_joint_position_status_ = true;
    jp_track_.setValue(wam_.getJointPositions());
    wam_.trackReferenceSignal(jp_track_.output);
    //Used for logging
/*     logging = true;
    std::thread thread(&WamSubscribers::startLogging,this);
    thread.detach(); */
  } else if (rt_joint_position_status_) {
    for (int i = 0; i < (int)DOF; i++) {
      jp_cmd_[i] = msg->joint_states[i];
      if (msg->rate_limits.size() !=
          DOF) {  // if rate limits not set, use default
        jp_rate_limits_[i] = kDefaultRateLimits;
      } else {
        jp_rate_limits_[i] = msg->rate_limits[i];
      }
    }
    jp_track_.setValue(jp_cmd_);
  }
  rt_joint_position_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtLinearAngularVelocityCb(
    const wam_msgs::msg::RTLinearandAngularVelocity::SharedPtr msg) {
  // If velocity control is activated, update values.
  if (!rt_linear_angular_velocity_status_) {
    rt_linear_angular_velocity_status_ = true;
    // start with linear velocity at 0
    cp_type start_cp;
    start_cp(0) = 0;
    start_cp(1) = 0;
    start_cp(2) = 0;
    linear_vel_direction_track_.setValue(start_cp);
    // start with angular velocity at 0
    math::Vector<3>::type start_rpy;
    start_rpy(0) = 0;
    start_rpy(1) = 0;
    start_rpy(2) = 0;
    angular_vel_direction_track_.setValue(start_rpy);
    current_cp_track_.setValue(wam_.getToolPosition());
    current_rpy_track_.setValue(toRPY(wam_.getToolOrientation()));
    /* Set ramp slope to goal linear velocity magnitude
       multiply output of ramp with linear velocity direction to get a linear
       velocity vector.*/
    ramp_.setSlope(linear_velocity_mag_);
    systems::forceConnect(ramp_.output, linear_velocity_mult_.input1);
    systems::forceConnect(linear_vel_direction_track_.output,
                          linear_velocity_mult_.input2);
    /* Set ramp1_ slope to goal angular velocity magnitude
       multiply output of ramp with angular velocity direction to get a angular
       velocity vector.*/
    ramp1_.setSlope(angular_velocity_mag_);
    systems::forceConnect(ramp1_.output, angular_velocity_mult_.input1);
    systems::forceConnect(angular_vel_direction_track_.output,
                          angular_velocity_mult_.input2);
    // Add output of multiplier to current position
    systems::forceConnect(linear_velocity_mult_.output,
                          cart_position_summer_.getInput(0));
    systems::forceConnect(current_cp_track_.output,
                          cart_position_summer_.getInput(1));
    // Add output of multiplier to current orientation
    systems::forceConnect(angular_velocity_mult_.output,
                          cart_orientation_summer_.getInput(0));
    systems::forceConnect(current_rpy_track_.output,
                          cart_orientation_summer_.getInput(1));
    // Group cartesian position and orientation
    systems::forceConnect(cart_position_summer_.output,
                          rt_pose_cmd_.getInput<0>());
    // Convert RPY to quaternion.
    systems::forceConnect(cart_orientation_summer_.output, to_quat_.input);
    systems::forceConnect(to_quat_.output, rt_pose_cmd_.getInput<1>());
    ramp_.stop();
    ramp1_.stop();
    ramp_.setOutput(0.0);  // start ramp at 0
    ramp1_.setOutput(0.0);
    ramp_.start();
    ramp1_.start();
    wam_.trackReferenceSignal(rt_pose_cmd_.output);
  } else if (rt_linear_angular_velocity_status_) {
    for (int i = 0; i < 3; i++) {
      rt_linear_velocity_cmd_[i] = msg->linear_velocity_direction[i];
      rt_angular_velocity_cmd_[i] = msg->angular_velocity_direction[i];
    }
    if (msg->linear_velocity_magnitude != 0) {
      linear_velocity_mag_ = msg->linear_velocity_magnitude;
    } else { //use default speed
      linear_velocity_mag_ = kCartesianSpeed;
    }
    if (msg->angular_velocity_magnitude != 0) {
      angular_velocity_mag_ = msg->angular_velocity_magnitude;
    } else { //use default speed
      angular_velocity_mag_ = kCartesianSpeed;
    }
    ramp_.reset();
    ramp1_.reset();
    ramp_.setSlope(linear_velocity_mag_);
    ramp1_.setSlope(angular_velocity_mag_);
    linear_vel_direction_track_.setValue(rt_linear_velocity_cmd_);
    angular_vel_direction_track_.setValue(rt_angular_velocity_cmd_);
    current_cp_track_.setValue(wam_.tpoTpController.referenceInput.getValue());
    current_rpy_track_.setValue(toRPY(wam_.tpoToController.referenceInput.getValue()));
  }
  rt_linear_angular_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtAngularVelocityCb(
    const wam_msgs::msg::RTAngularVelocity::SharedPtr msg) {
  //Initialize systems
  if (!rt_angular_velocity_status_) {
    rt_angular_velocity_status_ = true;
    angular_vel_direction_track_.setValue(math::Vector<3>::type(0, 0, 0));
    //get current RPY
    current_rpy_track_.setValue(toRPY(wam_.getToolOrientation()));
    current_cp_track_.setValue(wam_.getToolPosition());
    /* Set ramp1_ slope to goal angular velocity magnitude
    multiply output of ramp with angular velocity direction to get a angular velocity vector.*/ 
    systems::forceConnect(ramp_.output, angular_velocity_mult_.input1);
    systems::forceConnect(angular_vel_direction_track_.output, angular_velocity_mult_.input2);
    //Add output of multiplier to current orientation
    systems::forceConnect(angular_velocity_mult_.output, cart_orientation_summer_.getInput(0));
    systems::forceConnect(current_rpy_track_.output, cart_orientation_summer_.getInput(1));
    //convert to quaternion
    systems::forceConnect(cart_orientation_summer_.output, to_quat_.input);
    systems::forceConnect(current_cp_track_.output, rt_pose_cmd_.getInput<0>());
    systems::forceConnect(to_quat_.output, rt_pose_cmd_.getInput<1>());
    ramp_.setSlope(angular_velocity_mag_);
    ramp_.stop();
    ramp_.setOutput(0.0);
    ramp_.start();
    wam_.trackReferenceSignal(rt_pose_cmd_.output);
  } else if (rt_angular_velocity_status_) {
    for (int i = 0; i < 3; i++) {
      rt_angular_velocity_cmd_[i] = msg->direction[i];
    }
    if (msg->magnitude != 0) {
      angular_velocity_mag_ = msg->magnitude;
    } else { //use default speed
      angular_velocity_mag_ = kCartesianSpeed;
    }
    ramp_.reset();
    ramp_.setSlope(angular_velocity_mag_);
    angular_vel_direction_track_.setValue(rt_angular_velocity_cmd_);
    current_rpy_track_.setValue(toRPY(wam_.tpoToController.referenceInput.getValue()));
  }
  rt_angular_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtLinearVelocityCb(
    const wam_msgs::msg::RTLinearVelocity::SharedPtr msg) {
  if (!rt_linear_velocity_status_) {
    rt_linear_velocity_status_ = true;
    linear_vel_direction_track_.setValue(cp_type(0, 0, 0));
    current_cp_track_.setValue(wam_.getToolPosition());
    current_quat_track_.setValue(wam_.getToolOrientation());
/* Set ramp slope to goal linear velocity magnitude
multiply output of ramp with linear velocity direction to get a linear velocity vector.*/ 
    systems::forceConnect(ramp_.output, linear_velocity_mult_.input1);
    systems::forceConnect(linear_vel_direction_track_.output, linear_velocity_mult_.input2);
//Add output of multiplier to current orientation
    systems::forceConnect(linear_velocity_mult_.output, cart_position_summer_.getInput(0));
    systems::forceConnect(current_cp_track_.output, cart_position_summer_.getInput(1));
    systems::forceConnect(cart_position_summer_.output, rt_pose_cmd_.getInput<0>());;
    systems::forceConnect(current_quat_track_.output, rt_pose_cmd_.getInput<1>());
    ramp_.setSlope(linear_velocity_mag_);
    ramp_.stop();
    ramp_.setOutput(0.0);
    ramp_.start();
    wam_.trackReferenceSignal(rt_pose_cmd_.output);
  } else if (rt_linear_velocity_status_) {
    for (int i = 0; i < 3; i++) {
      rt_linear_velocity_cmd_[i] = msg->direction[i];
    }
    if (msg->magnitude != 0) {
      linear_velocity_mag_ = msg->magnitude;
    } else { //use default speed
      linear_velocity_mag_ = kCartesianSpeed;
    }
    ramp_.reset();
    ramp_.setSlope(linear_velocity_mag_);
    linear_vel_direction_track_.setValue(rt_linear_velocity_cmd_);
    current_cp_track_.setValue(wam_.tpoTpController.referenceInput.getValue());
  }
  rt_linear_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtCartPositionCb(
    const wam_msgs::msg::RTCartPosition::SharedPtr msg) {
  if (!rt_cart_position_status_) {
    rt_cart_position_status_ = true;
//Start with goal as current position
    cp_cmd_track_.setValue(wam_.getToolPosition());
    current_quat_track_.setValue(wam_.getToolOrientation());
//set the value of rate limiter to subscribed rate limits, and limit goal CP by that. 
    cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
    systems::forceConnect(cp_cmd_track_.output, cartesian_rate_limiter_.input);
    systems::forceConnect(cartesian_rate_limiter_.output, rt_pose_cmd_.getInput<0>());
    systems::forceConnect(current_quat_track_.output, rt_pose_cmd_.getInput<1>());
    wam_.trackReferenceSignal(rt_pose_cmd_.output);
  } else if (rt_cart_position_status_) {
    if (msg->rate_limits.size() == 3) {
      for (int i = 0; i < 3; i++) {
        rt_cartesian_position_rate_limits_[i] = msg->rate_limits[i];
      }
    } else { //if rate limits not set, use default
      for (int i = 0; i < 3; i++) {
        rt_cartesian_position_rate_limits_[i] = kDefaultRateLimits;
      }
    }
    rt_cartesian_position_cmd_ << msg->point.x, msg->point.y, msg->point.z;
    cp_cmd_track_.setValue(rt_cartesian_position_cmd_);
    cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
  }
  rt_cart_position_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtCartPoseCb(
    const wam_msgs::msg::RTCartPose::SharedPtr msg) {
  if (!rt_cart_pose_status_) {
    rt_cart_pose_status_ = true;
    //goal CP and goal orientation
    cp_cmd_track_.setValue(wam_.getToolPosition());
    quat_cmd_track_.setValue(wam_.getToolOrientation());
    //limit rates by subscribed limits
    systems::forceConnect(cp_cmd_track_.output, cartesian_rate_limiter_.input);
    systems::forceConnect(cartesian_rate_limiter_.output, rt_pose_cmd_.getInput<0>());
    systems::forceConnect(quat_cmd_track_.output, rt_pose_cmd_.getInput<1>()); //no connection to RL 
    wam_.trackReferenceSignal(rt_pose_cmd_.output);
    cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
  } else if (rt_cart_pose_status_) {
    if (msg->position_rate_limits.size() == 3) {
      for (int i = 0; i < 3; i++) {
        rt_cartesian_position_rate_limits_[i] = msg->position_rate_limits[i];
      }
    } else {  //if rate limits not set, use default
      for (int i = 0; i < 3; i++) {
        rt_cartesian_position_rate_limits_[i] = kDefaultRateLimits;
      }
    }
    rt_cartesian_position_cmd_ << msg->point.x, msg->point.y, msg->point.z;
    rt_cartesian_orientation_cmd_.x() = msg->orientation.x;
    rt_cartesian_orientation_cmd_.y() = msg->orientation.y;
    rt_cartesian_orientation_cmd_.z() = msg->orientation.z;
    rt_cartesian_orientation_cmd_.w() = msg->orientation.w;
    if (msg->orientation_rate_limits.size() == 4) { //no ratelimiter for Quaternions
/*       rt_cartesian_orientation_rl_.x() = msg->orientation_rate_limits[0];
      rt_cartesian_orientation_rl_.y() = msg->orientation_rate_limits[1];
      rt_cartesian_orientation_rl_.z() = msg->orientation_rate_limits[2];
      rt_cartesian_orientation_rl_.w() = msg->orientation_rate_limits[3]; */
    } else {
/*       rt_cartesian_orientation_rl_.x() = kDefaultRateLimits;
      rt_cartesian_orientation_rl_.y() = kDefaultRateLimits;
      rt_cartesian_orientation_rl_.z() = kDefaultRateLimits;
      rt_cartesian_orientation_rl_.w() = kDefaultRateLimits; */
    }
    cp_cmd_track_.setValue(rt_cartesian_position_cmd_);
    quat_cmd_track_.setValue(rt_cartesian_orientation_cmd_);
    cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
  }
  rt_cart_pose_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtCartOrientationCb(
    const wam_msgs::msg::RTCartOrientation::SharedPtr msg) {
  if (!rt_cart_orientation_status_) {
    rt_cart_orientation_status_ = true;
    //goal orientation
    quat_cmd_track_.setValue(wam_.getToolOrientation());
    current_cp_track_.setValue(wam_.getToolPosition());
    //no rate limiter for orientation.
    systems::forceConnect(current_cp_track_.output, rt_pose_cmd_.getInput<0>());
    systems::forceConnect(quat_cmd_track_.output, rt_pose_cmd_.getInput<1>()); //no connection to RL
    wam_.trackReferenceSignal(rt_pose_cmd_.output);
  } else if (rt_cart_orientation_status_) {
/*     rt_cartesian_orientation_rl_.x() =  msg->rate_limits[0];
    rt_cartesian_orientation_rl_.y() = msg->rate_limits[1];
    rt_cartesian_orientation_rl_.z() = msg->rate_limits[2];
    rt_cartesian_orientation_rl_.w() = msg->rate_limits[3]; */

    rt_cartesian_orientation_cmd_.x() =  msg->orientation.x;
    rt_cartesian_orientation_cmd_.y() = msg->orientation.y;
    rt_cartesian_orientation_cmd_.z() = msg->orientation.z;
    rt_cartesian_orientation_cmd_.w() = msg->orientation.w;
    quat_cmd_track_.setValue(rt_cartesian_orientation_cmd_);
  }
  rt_cart_orientation_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::rtJointVelocityCb(
    const wam_msgs::msg::RTJointVelocities::SharedPtr msg) {
  //If velocity control is activated, update values.
  if (msg->velocities.size() != DOF) {
    RCLCPP_ERROR(this->get_logger(), "Invalid Joint Velocity command. Please enter %ld velocities", DOF);
    return;
  }
  if (!rt_joint_velocity_status_) {
    rt_joint_velocity_status_ = true;
    jv_type jv_start;
    for (int i = 0; i < (int)DOF; i++) {
      jv_start(i) = 0;
    }
    jv_track_.setValue(jv_start);
    wam_.trackReferenceSignal(jv_track_.output);
  } else {
    for (int i = 0; i < (int)DOF; i++) {
      rt_joint_velocity_cmd_[i] = msg->velocities[i];
    }
    jv_track_.setValue(rt_joint_velocity_cmd_);
  }
  rt_joint_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF>
void WamSubscribers<DOF>::startLogging() {
  systems::TupleGrouper<double, jp_type, jp_type, jp_type, jv_type> tg;
	systems::forceConnect(ramp_.output, tg.template getInput<0>());
	systems::forceConnect(jp_track_.output, tg.template getInput<1>());
  systems::forceConnect(joint_rate_limiter_.output, tg.template getInput<2>());
    systems::forceConnect(wam_.jpOutput, tg.template getInput<3>());
      systems::forceConnect(wam_.jvOutput, tg.template getInput<4>());
	typedef boost::tuple<double, jp_type, jp_type, jp_type, jv_type> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
  	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
	}
	systems::PeriodicDataLogger<tuple_type> logger(
			pm_.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm_.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	ramp_.start();
	connect(tg.output, logger.input);
	printf("Logging started.\n");
  while (logging) {

  }
	logger.closeLog();
	printf("Logging stopped.\n");

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV("ros_logging.csv");
	std::remove(tmpFile);


}

template <size_t DOF>
void WamSubscribers<DOF>::updateRT() {
  // Checks if messages have been published within timeout
  if (rclcpp::Time(rt_joint_position_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_linear_angular_velocity_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_angular_velocity_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_linear_velocity_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_cart_position_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_cart_pose_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_cart_orientation_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rclcpp::Time(rt_joint_velocity_msg_time_.nanoseconds() + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now()) {
  } else if (rt_joint_position_status_ | rt_linear_angular_velocity_status_ |
             rt_linear_velocity_status_ | rt_angular_velocity_status_ |
             rt_cart_pose_status_ | rt_cart_position_status_ |
             rt_cart_orientation_status_ | rt_joint_velocity_status_) {
    //if any Real-time control is initialized, and message not received in timeout, hold position.
    rt_joint_position_status_ = rt_linear_angular_velocity_status_ = rt_linear_velocity_status_ = 
    rt_angular_velocity_status_ = rt_cart_pose_status_ = rt_cart_position_status_ = 
    rt_cart_orientation_status_ = rt_joint_velocity_status_ = false;
    wam_.moveTo(wam_.getJointPositions());
    logging = false;
    RCLCPP_ERROR(this->get_logger(),"RT Control timed-out");
  }
}

#endif // WAM_NODE_SRC_WAM_SUBSCRIBER_H_