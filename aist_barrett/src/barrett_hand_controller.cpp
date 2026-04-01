// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
//
/*!
 *  \file       precision_gripper_controller.cpp
 *  \brief      controller for screw tools
 */
#include <barrett/products/product_manager.h>
#include <barrett/exception.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ddynamic_reconfigure2/ddynamic_reconfigure2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <aist_barrett_msgs/msg/tactile_states.hpp>
#include <aist_barrett_msgs/srv/set_grasp_mode.hpp>
#include <aist_barrett_msgs/srv/set_velocity.hpp>
#include <aist_barrett_msgs/srv/open_or_close.hpp>
#include <aist_barrett_msgs/action/gripper_command.hpp>

using namespace std::chrono_literals;

namespace aist_barrett
{
template <class T> inline T     square(T x)     { return x*x; }

/************************************************************************
*  static functions                                                     *
************************************************************************/
static inline rclcpp::SubscriptionOptions
create_subscription_options(const rclcpp::CallbackGroup::SharedPtr& cbg)
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = cbg;
    return options;
}

/************************************************************************
*  class BarrettHandController                                          *
************************************************************************/
class BarrettHandController : public rclcpp::Node
{
  private:
    using joint_state_t         = sensor_msgs::msg::JointState;
    using goal_uuid_t           = rclcpp_action::GoalUUID;
    using goal_response_t       = rclcpp_action::GoalResponse;
    using cancel_response_t     = rclcpp_action::CancelResponse;
    using callback_group_p      = rclcpp::CallbackGroup::SharedPtr;
    using timer_p               = rclcpp::TimerBase::SharedPtr;
    using tactile_states_t      = aist_barrett_msgs::msg::TactileStates;
    using float64_multi_array_t = std_msgs::msg::Float64MultiArray;
    using trigger_t             = std_srvs::srv::Trigger;
    using set_bool_t            = std_srvs::srv::SetBool;
    using set_grasp_mode_t      = aist_barrett_msgs::srv::SetGraspMode;
    using set_velocity_t        = aist_barrett_msgs::srv::SetVelocity;
    using open_or_close_t       = aist_barrett_msgs::srv::OpenOrClose;
    using gripper_command_t     = aist_barrett_msgs::action::GripperCommand;
    using vector_t              = std::vector<double>;

    template <class MSG>
    using msg_p         = typename MSG::UniquePtr;
    template <class MSG>
    using pub_p         = typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p         = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class ACT>
    using action_p      = typename rclcpp_action::Server<ACT>::SharedPtr;
    template <class ACT>
    using goal_cp       = std::shared_ptr<const typename ACT::Goal>;
    template <class ACT>
    using goal_handle_t = rclcpp_action::ServerGoalHandle<ACT>;
    template <class ACT>
    using goal_handle_p = std::shared_ptr<goal_handle_t<ACT> >;
    template <class SRV>
    using srv_p         = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp        = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p         = typename SRV::Response::SharedPtr;
    template <class SRV>
    using clnt_p        = typename rclcpp::Client<SRV>::SharedPtr;

    enum GraspMode      { PINCH, SCISSOR, GRIP };

  public:
    BarrettHandController(const rclcpp::NodeOptions& options)           ;

  private:
  // Service stuffs
    void        set_torque_mode_cb(req_cp<set_bool_t> req,
                                   res_p<set_bool_t>  res)              ;
    void        set_grasp_mode_cb(req_cp<set_grasp_mode_t> req,
                                  res_p<set_grasp_mode_t>)              ;
    void        set_velocity_cb(req_cp<set_velocity_t> req,
                                res_p<set_velocity_t>)                  ;
    void        open_or_close_cb(req_cp<open_or_close_t> req,
                                 res_p<open_or_close_t>)                ;
    void        idle_cb(req_cp<trigger_t>, res_p<trigger_t> res)        ;

  // Topic intput stuffs
    void        command_cb(msg_p<float64_multi_array_t> command)        ;

  // Timer stuffs
    void        tactile_states_cb()                                     ;
    void        joint_state_cb()                                        ;

  // Action stuffs
    goal_response_t
                goal_cb(const goal_uuid_t&,
                        goal_cp<gripper_command_t> goal)                ;
    cancel_response_t
                cancel_cb(const goal_handle_p<gripper_command_t>)       ;
    void        handle_accepted_cb(
                    goal_handle_p<gripper_command_t> goal_handle)       ;

    std::pair<double, double>
                actual_gap_and_effort(const joint_state_t& js)  const   ;
    bool        is_moving(const vector_t& vel)                  const   ;
    bool        reached_goal(double gap, const vector_t& vel)   const   ;
    bool        stalled(const vector_t& vel)                    const   ;
    double      goal_pos(double position)			const	;
    double      newton_meters(double torque)                    const   ;
    double      outer_finger_pos(const vector_t& pos, size_t i) const   ;

    std::pair<double, double>
                solve_for_finger_positions(GraspMode grasp_mode,
                                           double diameter,
                                           double spread_pos)   const   ;
    double      solve_for_finger_position(double r)             const   ;

  private:
  // libbarrett
    barrett::ProductManager             _pm;
    barrett::Hand* const                _hand;
    bool                                _torque_mode;
    const vector_t                      _torque_coefficients;

  // Gripper command action stuffs
    // const action_p<gripper_command_t>   _gripper_command_srv;
    // goal_handle_p<gripper_command_t>    _current_goal_handle;
    // std::mutex                          _current_goal_mtx;
    // rclcpp::Time                        _last_move_time;
    // const rclcpp::Duration              _stall_timeout;

  // Joint state stuffs
    joint_state_t                       _joint_state;
    const pub_p<joint_state_t>          _joint_state_pub;
    const callback_group_p              _joint_state_cbg;
    const timer_p                       _joint_state_timer;

  // Tactile sensor stuffs
    const pub_p<tactile_states_t>       _tactile_states_pub;
    const callback_group_p              _tactile_states_cbg;
    const timer_p                       _tactile_states_timer;

  // Command stuffs
    const sub_p<float64_multi_array_t>  _command_sub;

  // Service stuffs
    GraspMode                           _grasp_mode;
    const srv_p<set_bool_t>             _set_torque_mode_srv;
    const srv_p<set_grasp_mode_t>       _set_grasp_mode_srv;
    const srv_p<set_velocity_t>         _set_velocity_srv;
    const srv_p<open_or_close_t>        _open_or_close_srv;
    const srv_p<trigger_t>              _idle_srv;

  // Geometric dimensions required for computiong IK
    static constexpr double     _half_tread              = 0.025;
    static constexpr double     _inner_x                 = 0.050;
    static constexpr double     _inner_finger_length     = 0.070;
    static constexpr double     _outer_finger_length     = 0.058;
    static constexpr double     _outer_finger_pos_mul    = 45.0/180.0;
    static constexpr double     _outer_finger_pos_offset = 0.6109;  // 35 deg

  // Thresholds
    static constexpr double     _vel_thresh = 0.0873;  // 5 deg/sec
};

BarrettHandController::BarrettHandController(
    const rclcpp::NodeOptions& options)
    :rclcpp::Node("barrett_hand_controller", options),
     _pm(),
     _hand(_pm.foundHand() ? _pm.getHand() : nullptr),
     _torque_mode(false),
     _torque_coefficients(ddynamic_reconfigure2::declare_read_only_parameter(
                              this, "torque_coefficients",
                            // default values obtained from pyHand 1.0 Manual
                              vector_t{-2.85, 3.746e-3,
                                       -1.708e-6, 2.754e-10})),

     // _gripper_command_srv(rclcpp_action::create_server<gripper_command_t>(
     //                  this, "~/gripper_cmd",
     //                  std::bind(&BarrettHandController::goal_cb, this,
     //                            std::placeholders::_1, std::placeholders::_2),
     //                  std::bind(&BarrettHandController::cancel_cb,
     //                            this, std::placeholders::_1),
     //                  std::bind(&BarrettHandController::handle_accepted_cb,
     //                            this, std::placeholders::_1))),
     // _current_goal_handle(nullptr),
     // _current_goal_mtx(),
     // _last_move_time(now()),
     // _stall_timeout(std::chrono::duration<double>(
     //                    ddynamic_reconfigure2::declare_read_only_parameter(
     //                        this, "stall_timeout", 1.0))),

     _joint_state(),
     _joint_state_pub(create_publisher<joint_state_t>("/joint_states", 1)),
     _joint_state_cbg(create_callback_group(
                          rclcpp::CallbackGroupType::MutuallyExclusive)),
     _joint_state_timer(create_wall_timer(
                            5ms,
                            std::bind(&BarrettHandController::joint_state_cb,
                                      this),
                            _joint_state_cbg)),

     _tactile_states_pub(
         _hand->hasTactSensors() ?
         create_publisher<tactile_states_t>("~/tactile_states", 1) : nullptr),
     _tactile_states_cbg(
         _tactile_states_pub ?
         create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) :
         nullptr),
     _tactile_states_timer(
         _tactile_states_pub ?
         create_wall_timer(20ms,
                           std::bind(&BarrettHandController::tactile_states_cb,
                                     this),
                           _tactile_states_cbg) : nullptr),

     _command_sub(create_subscription<float64_multi_array_t>(
                      "~/commands", 1,
                      std::bind(&BarrettHandController::command_cb,
                                this, std::placeholders::_1))),

     _grasp_mode(PINCH),
     _set_torque_mode_srv(create_service<set_bool_t>(
                              "~/set_torque_mode",
                              std::bind(
                                  &BarrettHandController::set_torque_mode_cb,
                                  this,
                                  std::placeholders::_1,
                                  std::placeholders::_2))),
     _set_grasp_mode_srv(create_service<set_grasp_mode_t>(
                             "~/set_grasp_mode",
                             std::bind(
                                 &BarrettHandController::set_grasp_mode_cb,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2))),
     _set_velocity_srv(create_service<set_velocity_t>(
                           "~/set_velocity",
                           std::bind(&BarrettHandController::set_velocity_cb,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2))),
     _open_or_close_srv(create_service<open_or_close_t>(
                            "~/open_or_close",
                            std::bind(&BarrettHandController::open_or_close_cb,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2))),
     _idle_srv(create_service<trigger_t>(
                   "~/idle",
                   std::bind(&BarrettHandController::idle_cb, this,
                             std::placeholders::_1, std::placeholders::_2)))
{
    barrett::installExceptionHandler();
    if (!_hand)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "hand not found!");
        throw;
    }
    RCLCPP_INFO_STREAM(get_logger(), "hand found");
    _hand->initialize();
    RCLCPP_INFO_STREAM(get_logger(), "hand initialized");
    // _hand->open(barrett::Hand::GRASP, true);
    // _hand->open(barrett::Hand::SPREAD, true);
    _hand->update();
    if (_torque_mode)
        _hand->setTorqueMode(barrett::Hand::WHOLE_HAND);
    else
        _hand->setPositionMode(barrett::Hand::WHOLE_HAND);

    const auto
        device_name = ddynamic_reconfigure2::declare_read_only_parameter(
                          this, "device_name", "bhand");
    const auto
        mimic_outer_joints
            = ddynamic_reconfigure2::declare_read_only_parameter(
                this, "mimic_outer_joints", true);
    _joint_state.name.resize(mimic_outer_joints ? 4 : 8);
    _joint_state.name[0] = device_name + "_left_finger_joint";
    _joint_state.name[1] = device_name + "_right_finger_joint";
    _joint_state.name[2] = device_name + "_middle_finger_joint";
    _joint_state.name[3] = device_name + "_spread_joint";
    if (!mimic_outer_joints)
    {
        _joint_state.name[4] = device_name + "_left_finger_outer_joint";
        _joint_state.name[5] = device_name + "_right_finger_outer_joint";
        _joint_state.name[6] = device_name + "_middle_finger_outer_joint";
    }
    _joint_state.position.resize(_joint_state.name.size(), 0.0);
    _joint_state.velocity.resize(_joint_state.name.size(), 0.0);
    _joint_state.effort  .resize(_joint_state.name.size(), 0.0);
    _joint_state.header.stamp.sec     = 0;
    _joint_state.header.stamp.nanosec = 0;

    RCLCPP_INFO_STREAM(get_logger(), "controller started");
}

// Service stuffs
void
BarrettHandController::set_torque_mode_cb(req_cp<set_bool_t> req,
                                          res_p<set_bool_t>  res)
{
    _torque_mode = req->data;
    if (_torque_mode)
        _hand->setTorqueMode(barrett::Hand::GRASP);
    else
        _hand->setPositionMode(barrett::Hand::WHOLE_HAND);
    res->success = true;
    RCLCPP_INFO_STREAM(get_logger(), "torque mode "
                       << (_torque_mode ? "enabled" : "disabled"));
}

void
BarrettHandController::set_grasp_mode_cb(req_cp<set_grasp_mode_t> req,
                                         res_p<set_grasp_mode_t>  res)
{
    res->success = true;

    switch (req->mode)
    {
      case set_grasp_mode_t::Request::PINCH:
        _grasp_mode = PINCH;
        break;
      case set_grasp_mode_t::Request::SCISSOR:
        _grasp_mode = SCISSOR;
        break;
      case set_grasp_mode_t::Request::GRIP:
        _grasp_mode = GRIP;
        break;
      default:
        res->success = false;
        break;
    }

    if (res->success)
        RCLCPP_INFO_STREAM(get_logger(), "grasp mode set to"
                           << (_grasp_mode == PINCH   ? "PINCH" :
                               _grasp_mode == SCISSOR ? "SCISSOR" : "GRIP"));
    else
        RCLCPP_ERROR_STREAM(get_logger(), "unknown grasp mode");
}

void
BarrettHandController::set_velocity_cb(req_cp<set_velocity_t> req,
                                       res_p<set_velocity_t>)
{
    const auto  axis = (req->spread ? barrett::Hand::SPREAD
                                    : barrett::Hand::GRASP);
    _hand->velocityMove(barrett::Hand::jv_type(req->velocity), axis);

    RCLCPP_INFO_STREAM(get_logger(), (req->spread ? "spread" : "fingers")
                       << " velocity set to " << req->velocity);
}

void
BarrettHandController::open_or_close_cb(req_cp<open_or_close_t> req,
                                        res_p<open_or_close_t>)
{
    const auto  axis = (req->spread ? barrett::Hand::SPREAD
                                    : barrett::Hand::GRASP);
    if (req->close)
        _hand->close(axis, true);
    else
        _hand->open(axis, true);

    RCLCPP_INFO_STREAM(get_logger(), (req->spread ? "spraed" : "fingers")
                       << ' ' << (req->close  ? "closed" : "opened"));
}

void
BarrettHandController::idle_cb(req_cp<trigger_t>, res_p<trigger_t> res)
{
    _hand->idle();
    res->success = true;

    RCLCPP_INFO_STREAM(get_logger(), "idling");
}

// Topic input stuffs
void
BarrettHandController::command_cb(msg_p<float64_multi_array_t> command)
{
    if (command->data.size() != 4)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Illegal input command data size["
                            << command->data.size() << ']');
        return;
    }

    if (_torque_mode)
        _hand->setTorqueCommand(barrett::Hand::jt_type(command->data[0],
                                                       command->data[1],
                                                       command->data[2],
                                                       command->data[3]),
                                barrett::Hand::WHOLE_HAND);
    else
        _hand->setPositionCommand(barrett::Hand::jp_type(command->data[0],
                                                         command->data[1],
                                                         command->data[2],
                                                         command->data[3]),
                                  barrett::Hand::WHOLE_HAND);
}

// Timer stuffs
void
BarrettHandController::tactile_states_cb()
{
  // Get current tactile sensor values and time.
    _hand->update();
    const auto&         tactile_pucks = _hand->getTactilePucks();
    tactile_states_t    tactile_states;
    tactile_states.header.stamp = rclcpp::Node::now();

    for (size_t i = 0; i < tactile_pucks.size(); ++i)
    {
        auto& tactile_state = (i == 0 ?
                               tactile_states.left_finger_tactile_state :
                               i == 1 ?
                               tactile_states.right_finger_tactile_state :
                               i == 2 ?
                               tactile_states.middle_finger_tactile_state :
                               tactile_states.palm_tactile_state);
        const auto& pressures = tactile_pucks[i]->getTactileData();
        for (ssize_t j = 0; j < pressures.size(); ++j)
        {
            auto        value = (int)(pressures[j]*256.0)/102;
            tactile_state.tactile_state[j] = pressures[j];
            int         c = 0;
            int         chunk;

            for (int z = 4; z >= 0; --z)
            {
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

                switch (chunk - 4)
                {
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

            tactile_state.normalized_tactile_state[j] = c - 5;
        }
    }

    _tactile_states_pub->publish(tactile_states);
}

void
BarrettHandController::joint_state_cb()
{
  // Get current joint positions and time.
    _hand->update();
    const auto& hi  = _hand->getInnerLinkPosition();
    const auto& ho  = _hand->getOuterLinkPosition();
    const auto  now = rclcpp::Node::now();

  // Set joint velocities.
    if (const auto tp = rclcpp::Time(_joint_state.header.stamp).seconds())
    {
        const auto      dt = now.seconds() - tp;
        for (size_t i = 0; i < 4; ++i)
            _joint_state.velocity[i] = (hi[i] - _joint_state.position[i]) / dt;
        for (size_t i = 4; i < _joint_state.velocity.size(); ++i)
            _joint_state.velocity[i] = (ho[i-4] - _joint_state.position[i])
                                     / dt;
    }
    _joint_state.header.stamp = now;    // Update timestamp.

  // Set joint positions.
    for (size_t i = 0; i < 4; ++i)
        _joint_state.position[i] = hi[i];
    for (size_t i = 4; i < _joint_state.position.size(); ++i)
        _joint_state.position[i] = ho[i-4];

  // Set joint torques.
    if (_hand->hasFingertipTorqueSensors())
    {
        const auto&     torques = _hand->getFingertipTorque();
        for (size_t i = 0; i < 4; ++i)
            _joint_state.effort[i] = newton_meters(torques[i]);
    }

  // Publish joint state.
    _joint_state_pub->publish(_joint_state);

  // Check if the current goal is active.
  //   if (!_current_goal_handle || !_current_goal_handle->is_active())
  //       return;

  //   const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

  // // Check if the current goal is requested to be cancelled.
  //   if (_current_goal_handle->is_canceling())
  //   {
  //       auto	result = std::make_unique<gripper_command_t::Result>();
  //       result->stalled = false;
  //       _current_goal_handle->canceled(std::move(result));
  //       _current_goal_handle = nullptr;

  //       RCLCPP_WARN_STREAM(get_logger(), "goal CANCELED");
  //       return;
  //   }

  //   const auto  [gap, effort] = actual_gap_and_effort(_joint_state);

  //   if (is_moving(_joint_state.velocity))
  //       _last_move_time = now;
  //   else if (reached_goal(gap, _joint_state.velocity))
  //   {
  //       auto	result = std::make_unique<gripper_command_t::Result>();
  //       result->position     = gap;
  //       result->effort	     = effort;
  //       result->stalled	     = stalled(_joint_state.velocity);
  //       result->reached_goal = true;
  //       _current_goal_handle->succeed(std::move(result));
  //       _current_goal_handle = nullptr;

  //       RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[reached goal]");
  //       return;
  //   }
  //   else if (stalled(_joint_state.velocity))
  //   {
  //       auto	result = std::make_unique<gripper_command_t::Result>();
  //       result->position     = gap;
  //       result->effort	     = effort;
  //       result->stalled	     = true;
  //       result->reached_goal = false;
  //       _current_goal_handle->succeed(std::move(result));
  //       _current_goal_handle = nullptr;

  //       RCLCPP_INFO_STREAM(get_logger(), "goal SUCCEEDED[stalled]");
  //       return;
  //   }

  // // Publish speed and filtered current as a feedback.
  //   auto	feedback = std::make_unique<gripper_command_t::Feedback>();
  //   feedback->position	   = gap;
  //   feedback->effort	   = effort;
  //   feedback->stalled	   = stalled(_joint_state.velocity);
  //   feedback->reached_goal = reached_goal(gap, _joint_state.velocity);
  //   _current_goal_handle->publish_feedback(std::move(feedback));
}

// Action stuffs
BarrettHandController::goal_response_t
BarrettHandController::goal_cb(const goal_uuid_t&,
                               goal_cp<gripper_command_t> goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED: diameter=" << goal->diameter
                       << ", spread=" << goal->spread*180.0/M_PI
		       << " deg., max_effort=" << goal->max_effort);
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

BarrettHandController::cancel_response_t
BarrettHandController::cancel_cb(goal_handle_p<gripper_command_t>)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "accepted request for cancelling goal");
    return cancel_response_t::ACCEPT;
}

// void
// BarrettHandController::handle_accepted_cb(
//     goal_handle_p<gripper_command_t> goal_handle)
// {
//     const std::lock_guard<std::mutex>	lock(_current_goal_mtx);

//   // If any active goal exists, abort it.
//     if (_current_goal_handle != nullptr && _current_goal_handle->is_active())
//     {
//         auto	result = std::make_unique<gripper_command_t::Result>();
//     //     result->position     = actual_position(_present_pos);
//         result->effort	     = 0.0;
//         result->stalled	     = false;
//         result->reached_goal = false;
//         _current_goal_handle->abort(std::move(result));
//         _current_goal_handle = nullptr;

//         RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
//     }

//     _last_move_time = now();

//     // if (!send_move_command(goal_handle->get_goal()->command.position,
//     //     		   goal_handle->get_goal()->command.max_effort))
//     // {
//     //     auto	result = std::make_unique<gripper_command_t::Result>();
//     //     result->position     = actual_position(_present_pos);
//     //     result->effort	     = 0.0;
//     //     result->stalled	     = false;
//     //     result->reached_goal = false;
//     //     goal_handle->abort(std::move(result));

//     //     RCLCPP_ERROR_STREAM(get_logger(), "goal ABORTED");
//     //     return;
//     // }

//     _current_goal_handle = goal_handle;
// }

std::pair<double, double>
BarrettHandController::actual_gap_and_effort(const joint_state_t& js) const
{
    for (size_t i = 0; i < 3; ++i)
    {
        const auto      inner_pos = js.position[i];
        const auto      outer_pos = outer_finger_pos(js.position, i);
    }
}

bool
BarrettHandController::is_moving(const vector_t& vel) const
{
    return (std::abs(vel[0]) < _vel_thresh && std::abs(vel[1]) < _vel_thresh &&
            std::abs(vel[2]) < _vel_thresh && std::abs(vel[3]) < _vel_thresh);
}

// bool
// BarrettHandController::reached_goal(double gap, const vector_t& vel) const
// {
//     // RCLCPP_DEBUG_STREAM(get_logger(), "*** gap=" << gap << ", goal_pos="
//     //     		<< _current_goal_handle->get_goal()->command.position
//     //     		<< ", vel=" << vel);
//     return !is_moving(vel) &&
// 	   std::abs(gap - goal_pos(_current_goal_handle
// 				   ->get_goal()->command.position)) <= 0.001;
// }

// bool
// BarrettHandController::stalled(const vector_t& vel) const
// {
//     return !is_moving(vel) && now() - _last_move_time > _stall_timeout;
// }

double
BarrettHandController::goal_pos(double position) const
{
}

double
BarrettHandController::newton_meters(double torque) const
{
    double      nm = 0.0, x = 1.0;
    for (const auto coefficient : _torque_coefficients)
    {
        nm += coefficient * x;
        x  *= torque;
    }
    return nm;
}

double
BarrettHandController::outer_finger_pos(const vector_t& pos, size_t i) const
{
    if (pos.size() == 4)
        return _outer_finger_pos_mul * pos[i] + _outer_finger_pos_offset;
    else
        return pos[4 + i];
}

std::pair<double, double>
BarrettHandController::solve_for_finger_positions(GraspMode grasp_mode,
                                                  double diameter,
                                                  double spread_pos) const
{
    if (grasp_mode == SCISSOR)
        spread_pos = M_PI/2;    // 90 deg.

    const auto  r = (0.25*square(diameter) - square(_half_tread))
                  / (_half_tread*std::sin(spread_pos) +
                     std::sqrt(0.25*square(diameter) -
                               square(_half_tread*std::cos(spread_pos))));
    return {solve_for_finger_position(r - _inner_x),
            solve_for_finger_position(0.5*diameter - _inner_x)};
}

double
BarrettHandController::solve_for_finger_position(double r) const
{
    double      pos = 0.5*M_PI;   // Set initial position to 90 deg.
    for (size_t i = 10; i--; )
    {
        const auto outer_finger_pos = _outer_finger_pos_mul * pos
                                    + _outer_finger_pos_offset;
        const auto y = _inner_finger_length * std::cos(pos)
                     + _outer_finger_length * std::cos(outer_finger_pos);
        if (std::abs(y - r) < 0.0001)
            break;

        const auto s = _inner_finger_length * std::sin(pos)
                     + _outer_finger_length * std::sin(outer_finger_pos)
                     * _outer_finger_pos_mul;
        pos += y/s;
    }

    return pos;
}
}       // namespace aist_barrett

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_barrett::BarrettHandController)
