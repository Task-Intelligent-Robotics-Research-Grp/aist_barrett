// BSD 3-Clause License
//
// Copyright (c) 2023, National Institute of Industrial Science
// and Technology(AIST)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
//
/*!
 *  \file  barrett_hand_controller.cpp
 *  \brief controller for Barrett Hand
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
#include <aist_barrett_msgs/srv/set_velocity.hpp>
#include <aist_barrett_msgs/srv/open_or_close.hpp>
#include <aist_barrett_msgs/action/gripper_command.hpp>

using namespace std::chrono_literals;

namespace aist_barrett
{
template <class T> std::ostream&
operator <<(std::ostream& out, const std::vector<T>& v)
{
    for (const auto& x : v)
        out << ' ' << x;
    return out;
}

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
    using set_velocity_t        = aist_barrett_msgs::srv::SetVelocity;
    using open_or_close_t       = aist_barrett_msgs::srv::OpenOrClose;
    using gripper_command_t     = aist_barrett_msgs::action::GripperCommand;
    using vector_t              = std::vector<double>;
    using array4d               = std::array<double, 4>;

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
    template <class ACT>
    using result_p      = std::unique_ptr<typename ACT::Result>;
    template <class SRV>
    using srv_p         = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp        = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p         = typename SRV::Response::SharedPtr;
    template <class SRV>
    using clnt_p        = typename rclcpp::Client<SRV>::SharedPtr;

  public:
    BarrettHandController(const rclcpp::NodeOptions& options)           ;

  private:
  // Service stuffs
    void        set_torque_mode_cb(req_cp<set_bool_t> req,
                                   res_p<set_bool_t>  res)              ;
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
    void        handle_accepted_cb(
                    goal_handle_p<gripper_command_t> goal_handle)       ;
    cancel_response_t
                cancel_cb(const goal_handle_p<gripper_command_t>)       ;
    array4d     goal_pos(const goal_cp<gripper_command_t>& goal) const
                {
                    array4d     pos;

                    switch (goal->mode)
                    {
                      default:
                      case gripper_command_t::Goal::PINCH:
                        pos[0] = pos_from_radius(0.5*goal->gap);
                        pos[1] = pos[0];
                        pos[2] = pos[0];
                        pos[3] = std::clamp(goal->spread, 0.0, M_PI);
                        break;
                      case gripper_command_t::Goal::ENCOMPASS:
                        pos[0] = pos_from_height(goal->gap);
                        pos[1] = pos[0];
                        pos[2] = pos[0];
                        pos[3] = 0.0;
                        break;
                      case gripper_command_t::Goal::SCISSOR:
                        pos[0] = pos_from_radius(0.5*goal->gap - _half_tread);
                        pos[1] = pos[0];
                        pos[2] = 0.0;
                        pos[3] = M_PI/2;
                        break;
                      case gripper_command_t::Goal::GRIP:
                        pos[0] = pos_from_height(goal->gap);
                        pos[1] = pos[0];
                        pos[2] = pos[0];
                        pos[3] = M_PI;
                        break;
                    }

                    return pos;
                }
    static array4d
                goal_eff(const goal_cp<gripper_command_t>& goal)
                {
                    return array4d{goal->max_effort, goal->max_effort,
                                   goal->max_effort, 0.0};
                }
    void        set_result(const result_p<gripper_command_t>& result) const
                {
                    result->effort       = actual_eff(_joint_state);
                    result->stalled      = stalled(_joint_state);
                    result->reached_goal = reached_goal(_joint_state);
                }

  // Utilities
    array4d     send_move_command(const array4d& position,
                                  const array4d& effort) const
                {
                    _hand->setTorqueCommand(barrett::Hand::jt_type(
                                                effort[0], effort[1],
                                                effort[2], effort[3]),
                                            barrett::Hand::WHOLE_HAND);
                    _hand->setPositionCommand(barrett::Hand::jp_type(
                                                  position[0], position[1],
                                                  position[2], position[3]),
                                              barrett::Hand::WHOLE_HAND);
                    return position;
                }
    void        send_stop_command() const
                {
                    _hand->idle();
                }

    double      actual_eff(const joint_state_t& joint_state) const
                {
                    const auto& eff = joint_state.effort;
                    return (eff[0] + eff[1] + eff[2]) / 3.0;
                }

    bool        is_moving(const joint_state_t& joint_state) const
                {
                    const auto& vel = joint_state.velocity;
                    return (std::abs(vel[0]) > _vel_thresh ||
                            std::abs(vel[1]) > _vel_thresh ||
                            std::abs(vel[2]) > _vel_thresh ||
                            std::abs(vel[3]) > _vel_thresh);
                }
    bool        stalled(const joint_state_t& joint_state) const
                {
                    return !is_moving(joint_state) &&
                           now() > _stall_timeout + _last_move_time;
                }
    bool        reached_goal(const joint_state_t& joint_state) const
                {
                    const auto& pos = joint_state.position;
                    return (!is_moving(joint_state) &&
                            std::abs(pos[0] - _goal_pos[0]) < _pos_thresh &&
                            std::abs(pos[1] - _goal_pos[1]) < _pos_thresh &&
                            std::abs(pos[2] - _goal_pos[2]) < _pos_thresh &&
                            std::abs(pos[3] - _goal_pos[3]) < _pos_thresh);
                }

    double      newton_meters(double torque)                    const   ;
    static double
                outer_finger_pos(double p)
                {
                    return _outer_finger_pos_mul*p + _outer_finger_pos_offset;
                }
    static double
                radius_from_pos(double p)
                {
                    return _inner_x
                         + _inner_finger_length*std::cos(p)
                         + _outer_finger_length*std::cos(outer_finger_pos(p));
                }
    static double
                height_from_pos(double p)
                {
                    return _inner_finger_length*std::sin(p)
                         + _outer_finger_length*std::sin(outer_finger_pos(p));
                }
    static double
                pos_from_radius(double r)                               ;
    static double
                pos_from_height(double h)                               ;

  private:
  // libbarrett
    barrett::ProductManager             _pm;
    barrett::Hand* const                _hand;
    bool                                _torque_mode;
    const vector_t                      _torque_coefficients;

  // Joint state stuffs
    joint_state_t                       _joint_state;
    const pub_p<joint_state_t>          _joint_state_pub;
    const callback_group_p              _joint_state_cbg;
    const timer_p                       _joint_state_timer;
    std::mutex                          _joint_state_mtx;

  // Tactile sensor stuffs
    const pub_p<tactile_states_t>       _tactile_states_pub;
    const callback_group_p              _tactile_states_cbg;
    const timer_p                       _tactile_states_timer;

  // Command stuffs
    const sub_p<float64_multi_array_t>  _command_sub;

  // Service stuffs
    const srv_p<set_bool_t>             _set_torque_mode_srv;
    const srv_p<set_velocity_t>         _set_velocity_srv;
    const srv_p<open_or_close_t>        _open_or_close_srv;
    const srv_p<trigger_t>              _idle_srv;

  // Gripper command action stuffs
    array4d                             _goal_pos;
    const action_p<gripper_command_t>   _gripper_command_srv;
    goal_handle_p<gripper_command_t>    _gripper_command_goal_handle;
    std::mutex                          _gripper_command_goal_mtx;
    rclcpp::Time                        _last_move_time;
    const rclcpp::Duration              _stall_timeout;
    uint8_t                             _current_mode;

  // Geometric dimensions required for computiong IK
    static constexpr double     _half_tread              = 0.025;
    static constexpr double     _inner_x                 = 0.050;
    static constexpr double     _inner_finger_length     = 0.070;
    static constexpr double     _outer_finger_length     = 0.058;
    static constexpr double     _outer_finger_pos_mul    = 1.0 + 45.0/180.0;
    static constexpr double     _outer_finger_pos_offset = 0.733;  // 42 deg
    static constexpr double     _max_radius = _inner_x
                                            + _inner_finger_length
                                            + _outer_finger_length
                                            * std::cos(
                                                _outer_finger_pos_offset);
    static constexpr double     _max_height = _max_radius - _inner_x;

  // Thresholds
    static constexpr double     _pos_thresh = 0.01;     // 0.573 deg
    static constexpr double     _vel_thresh = 0.0873;   // 5 deg/sec
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

     _joint_state(),
     _joint_state_pub(create_publisher<joint_state_t>("/joint_states", 1)),
     _joint_state_cbg(create_callback_group(
                          rclcpp::CallbackGroupType::MutuallyExclusive)),
     _joint_state_timer(create_wall_timer(
                            5ms,
                            std::bind(&BarrettHandController::joint_state_cb,
                                      this),
                            _joint_state_cbg)),
     _joint_state_mtx(),

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

     _set_torque_mode_srv(create_service<set_bool_t>(
                              "~/set_torque_mode",
                              std::bind(
                                  &BarrettHandController::set_torque_mode_cb,
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
                             std::placeholders::_1, std::placeholders::_2))),

     _goal_pos(),
     _gripper_command_srv(rclcpp_action::create_server<gripper_command_t>(
                              this, "~/gripper_cmd",
                              std::bind(&BarrettHandController::goal_cb, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2),
                              std::bind(&BarrettHandController::cancel_cb,
                                        this, std::placeholders::_1),
                              std::bind(&BarrettHandController::
                                        handle_accepted_cb,
                                        this, std::placeholders::_1))),
     _gripper_command_goal_handle(nullptr),
     _gripper_command_goal_mtx(),
     _last_move_time(now()),
     _stall_timeout(std::chrono::duration<double>(
                        ddynamic_reconfigure2::declare_read_only_parameter(
                            this, "stall_timeout", 1.0))),
     _current_mode(gripper_command_t::Goal::PINCH)
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

    {
        const std::lock_guard<std::mutex>       lock(_joint_state_mtx);

      // Set joint velocities.
        if (const auto tp = rclcpp::Time(_joint_state.header.stamp).seconds())
        {
            const auto  dt = now.seconds() - tp;
            for (size_t i = 0; i < 4; ++i)
                _joint_state.velocity[i] = (hi[i] -
                                            _joint_state.position[i]) / dt;
            for (size_t i = 4; i < _joint_state.velocity.size(); ++i)
                _joint_state.velocity[i] = (ho[i-4] - _joint_state.position[i])
                                         / dt;
        }
        _joint_state.header.stamp = now;        // Update timestamp.

      // Set joint positions.
        for (size_t i = 0; i < 4; ++i)
            _joint_state.position[i] = hi[i];
        for (size_t i = 4; i < _joint_state.position.size(); ++i)
            _joint_state.position[i] = ho[i-4];

      // Set joint torques.
        if (_hand->hasFingertipTorqueSensors())
        {
            const auto& torques = _hand->getFingertipTorque();
            for (size_t i = 0; i < 4; ++i)
                _joint_state.effort[i] = newton_meters(torques[i]);
        }

      // Publish joint state.
        _joint_state_pub->publish(_joint_state);
    }

    const std::lock_guard<std::mutex>	lock(_gripper_command_goal_mtx);

  // Check if the current goal is active.
    if (!_gripper_command_goal_handle ||
        !_gripper_command_goal_handle->is_active())
        return;

    auto	result = std::make_unique<gripper_command_t::Result>();
    set_result(result);

  // Check if the current goal is requested to be cancelled.
    if (_gripper_command_goal_handle->is_canceling())
    {
        RCLCPP_WARN_STREAM(get_logger(), "goal CANCELED");
        send_stop_command();
        _gripper_command_goal_handle->canceled(std::move(result));
        _gripper_command_goal_handle = nullptr;
        return;
    }
    else if (is_moving(_joint_state))
    {
        _last_move_time = now;
    }
    else if (result->reached_goal || result->stalled)
    {
        RCLCPP_INFO_STREAM(get_logger(),
                           "goal SUCCEEDED[effort=" << result->effort
                           << ", reached_goal=" << std::boolalpha
                           << result->reached_goal
                           << ", stalled=" << std::boolalpha << result->stalled
                           << ']');
        _gripper_command_goal_handle->succeed(std::move(result));
        _gripper_command_goal_handle = nullptr;
        return;
    }

  // Publish speed and filtered current as a feedback.
    auto	feedback = std::make_unique<gripper_command_t::Feedback>();
    feedback->effort	   = result->effort;
    feedback->stalled	   = result->stalled;
    feedback->reached_goal = result->reached_goal;
    _gripper_command_goal_handle->publish_feedback(std::move(feedback));
}

// Action stuffs
BarrettHandController::goal_response_t
BarrettHandController::goal_cb(const goal_uuid_t&,
                               goal_cp<gripper_command_t> goal)
{
    RCLCPP_INFO_STREAM(get_logger(),
		       "goal ACCEPTED[gap=" << goal->gap
                       << ", spread=" << goal->spread*180.0/M_PI
		       << " deg., max_effort=" << goal->max_effort << ']');
    return goal_response_t::ACCEPT_AND_EXECUTE;
}

BarrettHandController::cancel_response_t
BarrettHandController::cancel_cb(goal_handle_p<gripper_command_t>)
{
    RCLCPP_DEBUG_STREAM(get_logger(), "request for cancelling goal ACCEPTED");
    return cancel_response_t::ACCEPT;
}

void
BarrettHandController::handle_accepted_cb(
    goal_handle_p<gripper_command_t> goal_handle)
{
    const std::lock_guard<std::mutex>	lock(_gripper_command_goal_mtx);

  // If any active goal exists, abort it.
    if (_gripper_command_goal_handle != nullptr &&
        _gripper_command_goal_handle->is_active())
    {
        auto	result = std::make_unique<gripper_command_t::Result>();
        set_result(result);
        _gripper_command_goal_handle->abort(std::move(result));
        _gripper_command_goal_handle = nullptr;

        RCLCPP_WARN_STREAM(get_logger(), "previous goal ABORTED");
    }
    _gripper_command_goal_handle = goal_handle;

  // If required to change mode, fully open all fingers to avoid collision.
    if (goal_handle->get_goal()->mode != _current_mode)
    {
        _current_mode = goal_handle->get_goal()->mode;
        _hand->open(barrett::Hand::GRASP, true);
    }

  // Send a move command to the gripper.
    _goal_pos = send_move_command(goal_pos(goal_handle->get_goal()),
                                  goal_eff(goal_handle->get_goal()));
    _last_move_time = now();
}

// Utilities
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
BarrettHandController::pos_from_radius(double r)
{
    if (r >= _max_radius)
        return 0.0;

    if (r < -_half_tread)
        r = -_half_tread;

    double      p = M_PI/2;   // Set initial position to 90 deg.
    double      l = 1.0e-4;
    double      e = radius_from_pos(p) - r;
    for (size_t i = 100; i--; )
    {
        const auto s = _inner_finger_length*std::sin(p)
                     + _outer_finger_length*std::sin(outer_finger_pos(p))
                     * _outer_finger_pos_mul;
        for (;;)
        {
            const auto p_new = p + e/(l + s);
            const auto e_new = radius_from_pos(p_new) - r;

            // std::cerr << i << ": e_new=" << e_new << ", p_new=" << p_new
            //           << std::endl;

            if (std::abs(e_new - e) < 1.0e-7)
                return p_new;
            else if (std::abs(e_new) < std::abs(e))
            {
                p = p_new;
                e = e_new;
                l *= 0.1;
                break;
            }

            l *= 10.0;
        }
    }

    return p;
}

double
BarrettHandController::pos_from_height(double h)
{
    if (h >= _max_height)
        return 0.0;

    double      p = M_PI/2;   // Set initial position to 90 deg.
    double      l = 1.0e-4;
    double      e = height_from_pos(p) - h;
    for (size_t i = 100; i--; )
    {
        const auto s = _inner_finger_length * std::cos(p)
                     + _outer_finger_length * std::cos(outer_finger_pos(p))
                     * _outer_finger_pos_mul;
        for (;;)
        {
            const auto p_new = p - e/(l + s);
            const auto e_new = height_from_pos(p_new) - h;

            // std::cerr << i << ": e_new=" << e_new << ", p_new=" << p_new
            //           << std::endl;

            if (std::abs(e_new - e) < 1.0e-7)
                return p_new;
            else if (std::abs(e_new) < std::abs(e))
            {
                p = p_new;
                e = e_new;
                l *= 0.1;
                break;
            }

            l *= 10.0;
        }
    }

    return p;
}
}       // namespace aist_barrett

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_barrett::BarrettHandController)
