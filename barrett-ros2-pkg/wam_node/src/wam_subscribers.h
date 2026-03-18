#pragma once

#include <boost/tuple/tuple.hpp>

#include <rclcpp/rclcpp.hpp>
#include <wam_msgs/msg/rt_angular_velocity.hpp>
#include <wam_msgs/msg/rt_cart_orientation.hpp>
#include <wam_msgs/msg/rt_cart_pose.hpp>
#include <wam_msgs/msg/rt_cart_position.hpp>
#include <wam_msgs/msg/rt_joint_positions.hpp>
#include <wam_msgs/msg/rt_joint_velocities.hpp>
#include <wam_msgs/msg/rt_linear_velocity.hpp>
#include <wam_msgs/msg/rt_linearand_angular_velocity.hpp>
#include <barrett/log.h>

namespace wam_node
{
static const int kPublishFrequency = 500;
rclcpp::Time kRtMsgTimeout(0,500000000);  //Timeout for RT messages: 0.5 seconds
static const double kCartesianSpeed = 0.03; // Default Cartesian Velocity
const double kDefaultRateLimits = 1;        // 1m/s or 1rad/s

template <size_t DOF>
class WamSubscribers : public rclcpp::Node
{
  private:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    template <class MSG>
    using sub_p = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class MSG>
    using msg_cp = typename MSG::ConstSharedPtr;

    using time_t                  = rclcpp::Time;
    using rt_joint_pos_t          = wam_msgs::msg::RTJointPositions;
    using rt_joint_vel_t          = wam_msgs::msg::RTJointVelocities;
    using rt_linear_vel_t         = wam_msgs::msg::RTLinearVelocity;
    using rt_linear_angular_vel_t = wam_msgs::msg::RTLinearandAngularVelocity;
    using rt_angular_vel_t        = wam_msgs::msg::RTAngularVelocity;
    using rt_cart_pos_t           = wam_msgs::msg::RTCartPosition;
    using rt_cart_pose_t          = wam_msgs::msg::RTCartPose;
    using rt_cart_orientation_t   = wam_msgs::msg::RTCartOrientation;

    using vector3_t               = barrett::math::Vector<3>::type;

  public:
    void updateRT();
    bool logging;
    void startLogging();

    explicit
    WamSubscribers(barrett::systems::Wam<DOF>* wam,
                   barrett::systems::RealTimeExecutionManager* em)
        :Node("WamSubscribers"),
         wam_(wam),
         em_(em),
         rt_joint_position_sub_(
             create_subscription<rt_joint_pos_t>(
                 "/wam/RTJointPositionCMD", 100,
                 std::bind(&WamSubscribers::rtJointPositionCb,
                           this, std::placeholders::_1))),
         rt_joint_velocity_sub_(
             create_subscription<rt_joint_vel_t>(
                 "/wam/RTJointVelocityCMD", 100,
                 std::bind(&WamSubscribers::rtJointVelocityCb,
                           this, std::placeholders::_1))),
         rt_linear_velocity_sub_(
             create_subscription<rt_linear_vel_t>(
                 "/wam/RTLinearVelocityCMD", 100,
                 std::bind(&WamSubscribers::rtLinearVelocityCb,
                           this, std::placeholders::_1))),
         rt_angular_velocity_sub_(
             create_subscription<rt_angular_vel_t>(
                 "/wam/RTAngularVelocityCMD", 100,
                 std::bind(&WamSubscribers::rtAngularVelocityCb,
                           this, std::placeholders::_1))),
         rt_linear_angular_velocity_sub_(
             create_subscription<rt_linear_angular_vel_t>(
                 "/wam/RTLinearandAngularVelocityCMD", 100,
                 std::bind(&WamSubscribers::rtLinearAngularVelocityCb,
                           this, std::placeholders::_1))),
         rt_cart_position_sub_(
             create_subscription<rt_cart_pos_t>(
                 "/wam/RTCartPositionCMD", 100,
                 std::bind(&WamSubscribers::rtCartPositionCb,
                           this, std::placeholders::_1))),
         rt_cart_orientation_sub_(
             create_subscription<rt_cart_orientation_t>(
                 "/wam/RTCartOrientationCMD", 100,
                 std::bind(&WamSubscribers::rtCartOrientationCb,
                           this, std::placeholders::_1))),
         rt_cart_pose_sub_(
             create_subscription<rt_cart_pose_t>(
                 "/wam/RTCartPoseCMD", 100,
                 std::bind(&WamSubscribers::rtCartPoseCb,
                           this, std::placeholders::_1))),
         rt_joint_position_status_(false),
         rt_joint_velocity_status_(false),
         rt_linear_velocity_status_(false),
         rt_angular_velocity_status_(false),
         rt_linear_angular_velocity_status_(false),
         rt_cart_position_status_(false),
         rt_cart_orientation_status_(false),
         rt_cart_pose_status_(false),
         new_rt_cmd_(false),
         linear_velocity_mag_(kCartesianSpeed),
         angular_velocity_mag_(kCartesianSpeed),
         ramp_(NULL, kCartesianSpeed),
         ramp1_(NULL, kCartesianSpeed)
    {
        em_->startManaging(ramp_);
        em_->startManaging(ramp1_);
    }

  private:
    void        rtJointPositionCb(msg_cp<rt_joint_pos_t> msg);
    void        rtLinearAngularVelocityCb(msg_cp<rt_linear_angular_vel_t> msg);
    void        rtAngularVelocityCb(msg_cp<rt_angular_vel_t> msg);
    void        rtLinearVelocityCb(msg_cp<rt_linear_vel_t> msg);
    void        rtCartPositionCb(msg_cp<rt_cart_pos_t> msg);
    void        rtCartPoseCb(msg_cp<rt_cart_pose_t> msg);
    void        rtCartOrientationCb(msg_cp<rt_cart_orientation_t> msg);
    void        rtJointVelocityCb(msg_cp<rt_joint_vel_t> msg);
    static vector3_t   toRPY(const Eigen::Quaterniond& inquat);

  private:
    barrett::systems::Wam<DOF>* const                 wam_;
    barrett::systems::RealTimeExecutionManager* const em_;

    const sub_p<rt_joint_pos_t>          rt_joint_position_sub_;
    const sub_p<rt_joint_vel_t>          rt_joint_velocity_sub_;
    const sub_p<rt_linear_vel_t>         rt_linear_velocity_sub_;
    const sub_p<rt_angular_vel_t>        rt_angular_velocity_sub_;
    const sub_p<rt_linear_angular_vel_t> rt_linear_angular_velocity_sub_;
    const sub_p<rt_cart_pos_t>           rt_cart_position_sub_;
    const sub_p<rt_cart_orientation_t>   rt_cart_orientation_sub_;
    const sub_p<rt_cart_pose_t>          rt_cart_pose_sub_;

  //RT Status indicators
    bool                                rt_joint_position_status_;
    bool                                rt_joint_velocity_status_;
    bool                                rt_linear_velocity_status_;
    bool                                rt_angular_velocity_status_;
    bool                                rt_linear_angular_velocity_status_;
    bool                                rt_cart_position_status_;
    bool                                rt_cart_orientation_status_;
    bool                                rt_cart_pose_status_;
    bool                                new_rt_cmd_; //Indicator for new command
    double                              linear_velocity_mag_;
    double                              angular_velocity_mag_;

  //Message Times
    time_t                              rt_joint_position_msg_time_;
    time_t                              prev_joint_position_msg_time_;
    time_t                              rt_joint_velocity_msg_time_;
    time_t                              rt_linear_velocity_msg_time_;
    time_t                              rt_angular_velocity_msg_time_;
    time_t                              rt_linear_angular_velocity_msg_time_;
    time_t                              rt_cart_position_msg_time_;
    time_t                              rt_cart_orientation_msg_time_;
    time_t                              rt_cart_pose_msg_time_;

    jp_type                             jp_cmd_, jp_rate_limits_;
    jp_type                             prev_jp_cmd_;
    cp_type                             rt_linear_velocity_cmd_,
                                        rt_cartesian_position_cmd_,
                                        rt_cartesian_position_rate_limits_;
    jv_type                             rt_joint_velocity_cmd_;
    vector3_t                           rt_angular_velocity_cmd_;
    Eigen::Quaterniond                  rt_cartesian_orientation_cmd_;

  //Libbarrett Systems
    Multiplier<double, cp_type, cp_type> linear_velocity_mult_; //custom libbarrett system, defined in custom_systems.h
    Multiplier<double, vector3_t, vector3_t> angular_velocity_mult_;
    barrett::systems::TupleGrouper<cp_type, Eigen::Quaterniond> rt_pose_cmd_;
    barrett::systems::ExposedOutput<jp_type> jp_track_;
    barrett::systems::FirstOrderFilter<jp_type> jp_filter_;
    barrett::systems::TupleGrouper<double, jp_type> tg_;
    typedef boost::tuple<double, jp_type> tuple_type_;
    barrett::systems::RateLimiter<jp_type> joint_rate_limiter_;
    barrett::systems::RateLimiter<cp_type> cartesian_rate_limiter_;
  /**Rate limiter does not work with Eigen::Quaterniond*/
  //systems::RateLimiter<Eigen::Quaterniond> co_rl;
  //Eigen::Quaterniond rt_cartesian_orientation_rl_;
    barrett::systems::Ramp ramp_, ramp1_;
    barrett::systems::Summer<cp_type> cart_position_summer_;
    barrett::systems::Summer<vector3_t> cart_orientation_summer_;
    barrett::systems::ExposedOutput<cp_type> linear_vel_direction_track_, current_cp_track_, cp_cmd_track_;
    barrett::systems::ExposedOutput<vector3_t> angular_vel_direction_track_, current_rpy_track_;
    barrett::systems::ExposedOutput<Eigen::Quaterniond> current_quat_track_, quat_cmd_track_;
    barrett::systems::ExposedOutput<jv_type> jv_track_; //system for RT Joint Velocity command
    ToQuaternion to_quat_; //custom libbarrett system, defined in custom_systems.h
  //subscription callbacks. Sets target velocities and positions, used by updateRT()
};

template<size_t DOF> typename WamSubscribers<DOF>::vector3_t
WamSubscribers<DOF>::toRPY(const Eigen::Quaterniond& inquat)
{
    vector3_t rpy;
    tf2::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
    tf2::Matrix3x3(q).getRPY(rpy[0], rpy[1], rpy[2]);
    return rpy;
}

template <size_t DOF> void
WamSubscribers<DOF>::rtJointPositionCb(msg_cp<rt_joint_pos_t> msg)
{
    if (msg->joint_states.size() != DOF)
    {
        RCLCPP_ERROR(get_logger(),
                     "Invalid Joint Position Command received. Please enter %ld "
                     "Joint Positions and Rate Limits",
                     DOF);
        return;
    }

    if (!rt_joint_position_status_)
    {
        rt_joint_position_status_ = true;

        jp_track_.setValue(wam_->getJointPositions());
        wam_->trackReferenceSignal(jp_track_.output);
      //Used for logging
      /*     logging = true;
             std::thread thread(&WamSubscribers::startLogging,this);
             thread.detach(); */
    }
    else
    {
        for (size_t i = 0; i < DOF; ++i)
        {
            jp_cmd_[i] = msg->joint_states[i];
            if (msg->rate_limits.size() != DOF)
            {  // if rate limits not set, use default
                jp_rate_limits_[i] = kDefaultRateLimits;
            }
            else
            {
                jp_rate_limits_[i] = msg->rate_limits[i];
            }
        }
        jp_track_.setValue(jp_cmd_);
    }
    rt_joint_position_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtJointVelocityCb(msg_cp<rt_joint_vel_t> msg)
{
  //If velocity control is activated, update values.
    if (msg->velocities.size() != DOF)
    {
        RCLCPP_ERROR(get_logger(),
                     "Invalid Joint Velocity command. Please enter %ld velocities",
                     DOF);
        return;
    }
    if (!rt_joint_velocity_status_)
    {
        rt_joint_velocity_status_ = true;

        jv_type jv_start;
        for (size_t i = 0; i < DOF; ++i)
            jv_start(i) = 0;
        jv_track_.setValue(jv_start);
        wam_->trackReferenceSignal(jv_track_.output);
    }
    else
    {
        for (size_t i = 0; i < DOF; ++i)
            rt_joint_velocity_cmd_[i] = msg->velocities[i];
        jv_track_.setValue(rt_joint_velocity_cmd_);
    }
    rt_joint_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtLinearVelocityCb(msg_cp<rt_linear_vel_t> msg)
{
    if (!rt_linear_velocity_status_)
    {
        rt_linear_velocity_status_ = true;

        linear_vel_direction_track_.setValue(cp_type(0, 0, 0));
        current_cp_track_.setValue(wam_->getToolPosition());
        current_quat_track_.setValue(wam_->getToolOrientation());

      /* Set ramp slope to goal linear velocity magnitude
         multiply output of ramp with linear velocity direction to get a linear velocity vector.*/
        barrett::systems::forceConnect(ramp_.output,
                                       linear_velocity_mult_.input1);
        barrett::systems::forceConnect(linear_vel_direction_track_.output,
                                       linear_velocity_mult_.input2);

      //Add output of multiplier to current orientation
        barrett::systems::forceConnect(linear_velocity_mult_.output,
                                       cart_position_summer_.getInput(0));
        barrett::systems::forceConnect(current_cp_track_.output,
                                       cart_position_summer_.getInput(1));
        barrett::systems::forceConnect(cart_position_summer_.output,
                                       rt_pose_cmd_.getInput<0>());;
        barrett::systems::forceConnect(current_quat_track_.output,
                                       rt_pose_cmd_.getInput<1>());
        ramp_.setSlope(linear_velocity_mag_);
        ramp_.stop();
        ramp_.setOutput(0.0);
        ramp_.start();
        wam_->trackReferenceSignal(rt_pose_cmd_.output);
    }
    else
    {
        for (size_t i = 0; i < 3; ++i)
            rt_linear_velocity_cmd_[i] = msg->direction[i];
        if (msg->magnitude != 0)
            linear_velocity_mag_ = msg->magnitude;
        else
            linear_velocity_mag_ = kCartesianSpeed;
        ramp_.reset();
        ramp_.setSlope(linear_velocity_mag_);
        linear_vel_direction_track_.setValue(rt_linear_velocity_cmd_);
        current_cp_track_.setValue(
            wam_->tpoTpController.referenceInput.getValue());
    }
    rt_linear_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtAngularVelocityCb(msg_cp<rt_angular_vel_t> msg)
{
  //Initialize systems
    if (!rt_angular_velocity_status_)
    {
        rt_angular_velocity_status_ = true;

        angular_vel_direction_track_.setValue(vector3_t(0, 0, 0));

      //get current RPY
        current_rpy_track_.setValue(toRPY(wam_->getToolOrientation()));
        current_cp_track_.setValue(wam_->getToolPosition());

      /* Set ramp1_ slope to goal angular velocity magnitude
         multiply output of ramp with angular velocity direction to get a angular velocity vector.*/
        barrett::systems::forceConnect(ramp_.output,
                                       angular_velocity_mult_.input1);
        barrett::systems::forceConnect(angular_vel_direction_track_.output,
                              angular_velocity_mult_.input2);

      //Add output of multiplier to current orientation
        barrett::systems::forceConnect(angular_velocity_mult_.output,
                              cart_orientation_summer_.getInput(0));
        barrett::systems::forceConnect(current_rpy_track_.output,
                              cart_orientation_summer_.getInput(1));

      //convert to quaternion
        barrett::systems::forceConnect(cart_orientation_summer_.output,
                                       to_quat_.input);
        barrett::systems::forceConnect(current_cp_track_.output,
                              rt_pose_cmd_.getInput<0>());
        barrett::systems::forceConnect(to_quat_.output,
                                       rt_pose_cmd_.getInput<1>());
        ramp_.setSlope(angular_velocity_mag_);
        ramp_.stop();
        ramp_.setOutput(0.0);
        ramp_.start();
        wam_->trackReferenceSignal(rt_pose_cmd_.output);
    }
    else
    {
        for (size_t i = 0; i < 3; ++i)
            rt_angular_velocity_cmd_[i] = msg->direction[i];
        if (msg->magnitude != 0)
            angular_velocity_mag_ = msg->magnitude;
        else
            angular_velocity_mag_ = kCartesianSpeed;

        ramp_.reset();
        ramp_.setSlope(angular_velocity_mag_);
        angular_vel_direction_track_.setValue(rt_angular_velocity_cmd_);
        current_rpy_track_.setValue(
            toRPY(wam_->tpoToController.referenceInput.getValue()));
    }
    rt_angular_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtLinearAngularVelocityCb(
    msg_cp<rt_linear_angular_vel_t> msg)
{
  // If velocity control is activated, update values.
    if (!rt_linear_angular_velocity_status_)
    {
        rt_linear_angular_velocity_status_ = true;

      // start with linear velocity at 0
        cp_type start_cp;
        start_cp(0) = 0;
        start_cp(1) = 0;
        start_cp(2) = 0;
        linear_vel_direction_track_.setValue(start_cp);

      // start with angular velocity at 0
        vector3_t start_rpy;
        start_rpy(0) = 0;
        start_rpy(1) = 0;
        start_rpy(2) = 0;
        angular_vel_direction_track_.setValue(start_rpy);
        current_cp_track_.setValue(wam_->getToolPosition());
        current_rpy_track_.setValue(toRPY(wam_->getToolOrientation()));

      /* Set ramp slope to goal linear velocity magnitude
         multiply output of ramp with linear velocity direction to get a linear
         velocity vector.*/
        ramp_.setSlope(linear_velocity_mag_);
        barrett::systems::forceConnect(ramp_.output,
                                       linear_velocity_mult_.input1);
        barrett::systems::forceConnect(linear_vel_direction_track_.output,
                                       linear_velocity_mult_.input2);

      /* Set ramp1_ slope to goal angular velocity magnitude
         multiply output of ramp with angular velocity direction to get a angular
         velocity vector.*/
        ramp1_.setSlope(angular_velocity_mag_);
        barrett::systems::forceConnect(ramp1_.output,
                                       angular_velocity_mult_.input1);
        barrett::systems::forceConnect(angular_vel_direction_track_.output,
                                       angular_velocity_mult_.input2);

      // Add output of multiplier to current position
        barrett::systems::forceConnect(linear_velocity_mult_.output,
                                       cart_position_summer_.getInput(0));
        barrett::systems::forceConnect(current_cp_track_.output,
                                       cart_position_summer_.getInput(1));

      // Add output of multiplier to current orientation
        barrett::systems::forceConnect(angular_velocity_mult_.output,
                                       cart_orientation_summer_.getInput(0));
        barrett::systems::forceConnect(current_rpy_track_.output,
                                       cart_orientation_summer_.getInput(1));

      // Group cartesian position and orientation
        barrett::systems::forceConnect(cart_position_summer_.output,
                                       rt_pose_cmd_.getInput<0>());

      // Convert RPY to quaternion.
        barrett::systems::forceConnect(cart_orientation_summer_.output,
                                       to_quat_.input);
        barrett::systems::forceConnect(to_quat_.output,
                                       rt_pose_cmd_.getInput<1>());
        ramp_.stop();
        ramp1_.stop();
        ramp_.setOutput(0.0);  // start ramp at 0
        ramp1_.setOutput(0.0);
        ramp_.start();
        ramp1_.start();
        wam_->trackReferenceSignal(rt_pose_cmd_.output);
    }
    else
    {
        for (size_t i = 0; i < 3; ++i)
        {
            rt_linear_velocity_cmd_[i]  = msg->linear_velocity_direction[i];
            rt_angular_velocity_cmd_[i] = msg->angular_velocity_direction[i];
        }

        if (msg->linear_velocity_magnitude != 0)
            linear_velocity_mag_ = msg->linear_velocity_magnitude;
        else
            linear_velocity_mag_ = kCartesianSpeed;
        if (msg->angular_velocity_magnitude != 0)
            angular_velocity_mag_ = msg->angular_velocity_magnitude;
        else
            angular_velocity_mag_ = kCartesianSpeed;

        ramp_.reset();
        ramp1_.reset();
        ramp_.setSlope(linear_velocity_mag_);
        ramp1_.setSlope(angular_velocity_mag_);
        linear_vel_direction_track_.setValue(rt_linear_velocity_cmd_);
        angular_vel_direction_track_.setValue(rt_angular_velocity_cmd_);
        current_cp_track_.setValue(
            wam_->tpoTpController.referenceInput.getValue());
        current_rpy_track_.setValue(
            toRPY(wam_->tpoToController.referenceInput.getValue()));
    }
    rt_linear_angular_velocity_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtCartPositionCb(msg_cp<rt_cart_pos_t> msg)
{
    if (!rt_cart_position_status_)
    {
        rt_cart_position_status_ = true;

      //Start with goal as current position
        cp_cmd_track_.setValue(wam_->getToolPosition());
        current_quat_track_.setValue(wam_->getToolOrientation());

      //set the value of rate limiter to subscribed rate limits, and limit goal CP by that.
        cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
        barrett::systems::forceConnect(cp_cmd_track_.output,
                                       cartesian_rate_limiter_.input);
        barrett::systems::forceConnect(cartesian_rate_limiter_.output,
                                       rt_pose_cmd_.getInput<0>());
        barrett::systems::forceConnect(current_quat_track_.output,
                                       rt_pose_cmd_.getInput<1>());
        wam_->trackReferenceSignal(rt_pose_cmd_.output);
    }
    else
    {
        if (msg->rate_limits.size() == 3)
        {
            for (size_t i = 0; i < 3; ++i)
                rt_cartesian_position_rate_limits_[i] = msg->rate_limits[i];
        }
        else
        { //if rate limits not set, use default
            for (size_t i = 0; i < 3; ++i)
                rt_cartesian_position_rate_limits_[i] = kDefaultRateLimits;
        }
        rt_cartesian_position_cmd_ << msg->point.x, msg->point.y, msg->point.z;
        cp_cmd_track_.setValue(rt_cartesian_position_cmd_);
        cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
    }
    rt_cart_position_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtCartOrientationCb(msg_cp<rt_cart_orientation_t> msg)
{
    if (!rt_cart_orientation_status_)
    {
        rt_cart_orientation_status_ = true;

      //goal orientation
        quat_cmd_track_.setValue(wam_->getToolOrientation());
        current_cp_track_.setValue(wam_->getToolPosition());

      //no rate limiter for orientation.
        barrett::systems::forceConnect(current_cp_track_.output,
                                       rt_pose_cmd_.getInput<0>());
        barrett::systems::forceConnect(quat_cmd_track_.output,
                                       rt_pose_cmd_.getInput<1>()); //no connection to RL
        wam_->trackReferenceSignal(rt_pose_cmd_.output);
    }
    else
    {
      /*     rt_cartesian_orientation_rl_.x() =  msg->rate_limits[0];
             rt_cartesian_orientation_rl_.y() = msg->rate_limits[1];
             rt_cartesian_orientation_rl_.z() = msg->rate_limits[2];
             rt_cartesian_orientation_rl_.w() = msg->rate_limits[3]; */

        rt_cartesian_orientation_cmd_.x() = msg->orientation.x;
        rt_cartesian_orientation_cmd_.y() = msg->orientation.y;
        rt_cartesian_orientation_cmd_.z() = msg->orientation.z;
        rt_cartesian_orientation_cmd_.w() = msg->orientation.w;
        quat_cmd_track_.setValue(rt_cartesian_orientation_cmd_);
    }
    rt_cart_orientation_msg_time_ =  rclcpp::Clock().now();
}

template <size_t DOF> void
WamSubscribers<DOF>::rtCartPoseCb(msg_cp<rt_cart_pose_t> msg)
{
    if (!rt_cart_pose_status_)
    {
        rt_cart_pose_status_ = true;

      //goal CP and goal orientation
        cp_cmd_track_.setValue(wam_->getToolPosition());
        quat_cmd_track_.setValue(wam_->getToolOrientation());

      //limit rates by subscribed limits
        barrett::systems::forceConnect(cp_cmd_track_.output,
                                       cartesian_rate_limiter_.input);
        barrett::systems::forceConnect(cartesian_rate_limiter_.output,
                                       rt_pose_cmd_.getInput<0>());
        barrett::systems::forceConnect(quat_cmd_track_.output,
                                       rt_pose_cmd_.getInput<1>()); //no connection to RL
        wam_->trackReferenceSignal(rt_pose_cmd_.output);
        cartesian_rate_limiter_.setLimit(rt_cartesian_position_rate_limits_);
    }
    else
    {
        if (msg->position_rate_limits.size() == 3)
        {
            for (size_t i = 0; i < 3; ++i)
                rt_cartesian_position_rate_limits_[i]
                    = msg->position_rate_limits[i];
        }
        else
        {  //if rate limits not set, use default
            for (size_t i = 0; i < 3; ++i)
                rt_cartesian_position_rate_limits_[i] = kDefaultRateLimits;
        }
        rt_cartesian_position_cmd_ << msg->point.x, msg->point.y, msg->point.z;
        rt_cartesian_orientation_cmd_.x() = msg->orientation.x;
        rt_cartesian_orientation_cmd_.y() = msg->orientation.y;
        rt_cartesian_orientation_cmd_.z() = msg->orientation.z;
        rt_cartesian_orientation_cmd_.w() = msg->orientation.w;
        if (msg->orientation_rate_limits.size() == 4)
        { //no ratelimiter for Quaternions
          /*       rt_cartesian_orientation_rl_.x() = msg->orientation_rate_limits[0];
                   rt_cartesian_orientation_rl_.y() = msg->orientation_rate_limits[1];
                   rt_cartesian_orientation_rl_.z() = msg->orientation_rate_limits[2];
                   rt_cartesian_orientation_rl_.w() = msg->orientation_rate_limits[3]; */
        }
        else
        {
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

template <size_t DOF> void
WamSubscribers<DOF>::startLogging()
{
    using tuple_type = boost::tuple<double, jp_type, jp_type, jp_type, jv_type>;

    barrett::systems::TupleGrouper<double, jp_type, jp_type, jp_type, jv_type> tg;
    barrett::systems::forceConnect(ramp_.output, tg.template getInput<0>());
    barrett::systems::forceConnect(jp_track_.output,
                                   tg.template getInput<1>());
    barrett::systems::forceConnect(joint_rate_limiter_.output,
                                   tg.template getInput<2>());
    barrett::systems::forceConnect(wam_->jpOutput, tg.template getInput<3>());
    barrett::systems::forceConnect(wam_->jvOutput, tg.template getInput<4>());

    const size_t PERIOD_MULTIPLIER = 1;
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1)
    {
        printf("ERROR: Couldn't create temporary file!\n");
    }
    barrett::systems::PeriodicDataLogger<tuple_type>
        logger(em_,
               new barrett::log::RealTimeWriter<tuple_type>(
                   tmpFile, PERIOD_MULTIPLIER * em_->getPeriod()),
               PERIOD_MULTIPLIER);

    ramp_.start();
    connect(tg.output, logger.input);
    printf("Logging started.\n");
    while (logging)
    {
    }
    logger.closeLog();
    printf("Logging stopped.\n");

    barrett::log::Reader<tuple_type> lr(tmpFile);
    lr.exportCSV("ros_logging.csv");
    std::remove(tmpFile);


}

template <size_t DOF> void
WamSubscribers<DOF>::updateRT()
{
  // Checks if messages have been published within timeout
    if (time_t(rt_joint_position_msg_time_.nanoseconds()
               + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_linear_angular_velocity_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_angular_velocity_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_linear_velocity_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_cart_position_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_cart_pose_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_cart_orientation_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (time_t(rt_joint_velocity_msg_time_.nanoseconds()
                    + kRtMsgTimeout.nanoseconds())  > rclcpp::Clock().now())
    {
    }
    else if (rt_joint_position_status_ | rt_linear_angular_velocity_status_ |
             rt_linear_velocity_status_ | rt_angular_velocity_status_ |
             rt_cart_pose_status_ | rt_cart_position_status_ |
             rt_cart_orientation_status_ | rt_joint_velocity_status_)
    {
      //if any Real-time control is initialized, and message not received in timeout, hold position.
        rt_joint_position_status_
            = rt_linear_angular_velocity_status_
            = rt_linear_velocity_status_
            = rt_angular_velocity_status_
            = rt_cart_pose_status_
            = rt_cart_position_status_
            = rt_cart_orientation_status_
            = rt_joint_velocity_status_
            = false;
        wam_->moveTo(wam_->getJointPositions());
        logging = false;
        RCLCPP_ERROR(get_logger(),"RT Control timed-out");
    }
}
}
