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
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <aist_barrett_msgs/msg/tactile_state_array.hpp>
#include <aist_barrett_msgs/srv/finger_position.hpp>
#include <aist_barrett_msgs/srv/finger_velocity.hpp>
#include <aist_barrett_msgs/srv/grasp_position.hpp>
#include <aist_barrett_msgs/srv/grasp_velocity.hpp>
#include <aist_barrett_msgs/srv/spread_position.hpp>
#include <aist_barrett_msgs/srv/spread_velocity.hpp>

using namespace std::chrono_literals;

namespace aist_barrett
{
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
    using gripper_command_t     = control_msgs::action::GripperCommand;
    using goal_uuid_t           = rclcpp_action::GoalUUID;
    using goal_response_t       = rclcpp_action::GoalResponse;
    using cancel_response_t     = rclcpp_action::CancelResponse;
    using callback_group_p      = rclcpp::CallbackGroup::SharedPtr;
    using timer_p               = rclcpp::TimerBase::SharedPtr;
    using tactile_states_t      = aist_barrett_msgs::msg::TactileStateArray;
    using float64_multi_array_t = std_msgs::msg::Float64MultiArray;
    using trigger_t             = std_srvs::srv::Trigger;
    using set_bool_t            = std_srvs::srv::SetBool;
    using finger_pos_t          = aist_barrett_msgs::srv::FingerPosition;
    using grasp_pos_t           = aist_barrett_msgs::srv::GraspPosition;
    using spread_pos_t          = aist_barrett_msgs::srv::SpreadPosition;
    using finger_vel_t          = aist_barrett_msgs::srv::FingerVelocity;
    using grasp_vel_t           = aist_barrett_msgs::srv::GraspVelocity;
    using spread_vel_t          = aist_barrett_msgs::srv::SpreadVelocity;

    template <class MSG>
    using msg_p         = typename MSG::UniquePtr;
    template <class MSG>
    using pub_p         = typename rclcpp::Publisher<MSG>::SharedPtr;
    template <class MSG>
    using sub_p         = typename rclcpp::Subscription<MSG>::SharedPtr;
    template <class SRV>
    using srv_p         = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp        = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p         = typename SRV::Response::SharedPtr;
    template <class SRV>
    using clnt_p        = typename rclcpp::Client<SRV>::SharedPtr;
    template <class ACT>
    using action_p      = typename rclcpp_action::Server<ACT>::SharedPtr;
    template <class ACT>
    using goal_cp       = std::shared_ptr<const typename ACT::Goal>;
    template <class ACT>
    using goal_handle_t = rclcpp_action::ServerGoalHandle<ACT>;
    template <class ACT>
    using goal_handle_p = std::shared_ptr<goal_handle_t<ACT> >;

  public:
    BarrettHandController(const rclcpp::NodeOptions& options)           ;

  private:
    void        joint_state_cb()                                        ;
    void        tactile_state_cb()                                      ;

    void        command_cb(msg_p<float64_multi_array_t> command)        ;

    void        finger_position_cb(req_cp<finger_pos_t> req,
                                   res_p<finger_pos_t>  res)            ;
    void        grasp_position_cb(req_cp<grasp_pos_t> req,
                                  res_p<grasp_pos_t>  res)              ;
    void        spread_position_cb(req_cp<spread_pos_t> req,
                                   res_p<spread_pos_t>  res)            ;
    void        finger_velocity_cb(req_cp<finger_vel_t> req,
                                   res_p<finger_vel_t>  res)            ;
    void        grasp_velocity_cb(req_cp<grasp_vel_t> req,
                                  res_p<grasp_vel_t>  res)              ;
    void        spread_velocity_cb(req_cp<spread_vel_t> req,
                                   res_p<spread_vel_t>  res)            ;
    void        idle_cb(req_cp<trigger_t>, res_p<trigger_t> res)        ;
    void        open_close_cb(req_cp<set_bool_t> req,
                              res_p<set_bool_t>  res, bool spread)      ;

    goal_response_t
                goal_cb(const goal_uuid_t&,
                        goal_cp<gripper_command_t> goal)                ;
    cancel_response_t
                cancel_cb(const goal_handle_p<gripper_command_t>)       ;
    void        handle_accepted_cb(
                    goal_handle_p<gripper_command_t> goal_handle)       ;

  private:
  // libbarrett
    barrett::ProductManager             _pm;
    barrett::Hand* const                _hand;

  // Joint state stuffs
    joint_state_t                       _joint_state;
    const pub_p<joint_state_t>          _joint_state_pub;
    const callback_group_p              _joint_state_cbg;
    const timer_p                       _joint_state_timer;

  // Tactile sensor stuffs
    const pub_p<tactile_states_t>       _tactile_state_pub;
    const callback_group_p              _tactile_state_cbg;
    const timer_p                       _tactile_state_timer;

  // Command stuffs
    const sub_p<float64_multi_array_t>  _command_sub;

  // Service stuffs
    const srv_p<finger_pos_t>           _finger_position_srv;
    const srv_p<grasp_pos_t>            _grasp_position_srv;
    const srv_p<spread_pos_t>           _spread_position_srv;
    const srv_p<finger_vel_t>           _finger_velocity_srv;
    const srv_p<grasp_vel_t>            _grasp_velocity_srv;
    const srv_p<spread_vel_t>           _spread_velocity_srv;
    const srv_p<trigger_t>              _idle_srv;
    const srv_p<set_bool_t>             _grasp_srv;
    const srv_p<set_bool_t>             _spread_srv;

  // Gripper command action stuffs
    // const action_p<gripper_command_t>        _command_srv;
    // goal_handle_p<gripper_command_t> _current_goal_handle;
    // std::mutex                               _current_goal_mtx;
    // rclcpp::Time                     _last_move_time;
};

BarrettHandController::BarrettHandController(
    const rclcpp::NodeOptions& options)
    :rclcpp::Node("barrett_hand_controller", options),
     _pm(),
     _hand(_pm.foundHand() ? _pm.getHand() : nullptr),

     _joint_state(),
     _joint_state_pub(create_publisher<joint_state_t>("/joint_states", 1)),
     _joint_state_cbg(create_callback_group(
                          rclcpp::CallbackGroupType::MutuallyExclusive)),
     _joint_state_timer(create_wall_timer(
                            2ms,
                            std::bind(&BarrettHandController::joint_state_cb,
                                      this),
                            _joint_state_cbg)),

     _tactile_state_pub(
         _hand->hasTactSensors() ?
         create_publisher<tactile_states_t>("~/tactile_states", 1) : nullptr),
     _tactile_state_cbg(
         _tactile_state_pub ?
         create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) :
         nullptr),
     _tactile_state_timer(
         _tactile_state_pub ?
         create_wall_timer(20ms,
                           std::bind(&BarrettHandController::tactile_state_cb,
                                     this),
                           _tactile_state_cbg) : nullptr),

     _command_sub(create_subscription<float64_multi_array_t>(
                      "~/commands", 1,
                      std::bind(&BarrettHandController::command_cb,
                                this, std::placeholders::_1))),

     _finger_position_srv(create_service<finger_pos_t>(
                              "~/move_to_finger_positions",
                              std::bind(
                                  &BarrettHandController::finger_position_cb,
                                  this,
                                  std::placeholders::_1,
                                  std::placeholders::_2))),
     _grasp_position_srv(create_service<grasp_pos_t>(
                             "~/move_to_grasp_position",
                             std::bind(
                                 &BarrettHandController::grasp_position_cb,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2))),
     _spread_position_srv(create_service<spread_pos_t>(
                              "~/move_to_spread_position",
                              std::bind(
                                  &BarrettHandController::spread_position_cb,
                                  this,
                                  std::placeholders::_1,
                                  std::placeholders::_2))),

     _finger_velocity_srv(create_service<finger_vel_t>(
                              "~/move_to_finger_velocities",
                              std::bind(
                                  &BarrettHandController::finger_velocity_cb,
                                  this,
                                  std::placeholders::_1,
                                  std::placeholders::_2))),
     _grasp_velocity_srv(create_service<grasp_vel_t>(
                             "~/move_to_grasp_velocities",
                             std::bind(
                                 &BarrettHandController::grasp_velocity_cb,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2))),
     _spread_velocity_srv(create_service<spread_vel_t>(
                              "~/move_to_spread_velocity",
                              std::bind(
                                  &BarrettHandController::spread_velocity_cb,
                                  this,
                                  std::placeholders::_1,
                                  std::placeholders::_2))),
     _idle_srv(create_service<trigger_t>(
                   "~/idle",
                   std::bind(&BarrettHandController::idle_cb, this,
                             std::placeholders::_1, std::placeholders::_2))),
     _grasp_srv(create_service<set_bool_t>(
                    "~/grasp",
                    std::bind(&BarrettHandController::open_close_cb, this,
                              std::placeholders::_1,
                              std::placeholders::_2, false))),
     _spread_srv(create_service<set_bool_t>(
                     "~/spread",
                     std::bind(&BarrettHandController::open_close_cb, this,
                               std::placeholders::_1,
                               std::placeholders::_2, true)))

   // _command_srv(rclcpp_action::create_server<gripper_command_t>(
     //                       this, "~/gripper_cmd",
     //                       std::bind(&BarrettHandController::goal_cb, this,
     //                                 std::placeholders::_1, std::placeholders::_2),
     //                       std::bind(&BarrettHandController::cancel_cb, this,
     //                                 std::placeholders::_1),
     //                       std::bind(&BarrettHandController::
     //                                 handle_accepted_cb, this,
     //                                 std::placeholders::_1))),
     // _current_goal_handle(nullptr),
     // _current_goal_mtx(),
     // _last_move_time(now())
{
    barrett::installExceptionHandler();
    if (!_hand)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "hand not found!");
        throw;
    }
    _hand->initialize();
    _hand->update();

    const auto
        device_name = ddynamic_reconfigure2::declare_read_only_parameter(
                          this, "device_name", "bhand");
    _joint_state.name.resize(4);
    _joint_state.name[0] = device_name + "_left_finger_joint";
    _joint_state.name[1] = device_name + "_right_finger_joint";
    _joint_state.name[2] = device_name + "_middle_finger_joint";
    _joint_state.name[3] = device_name + "_spread_joint";
    _joint_state.position.resize(_joint_state.name.size(), 0.0);
    _joint_state.velocity.resize(_joint_state.name.size(), 0.0);
    _joint_state.effort  .resize(_joint_state.name.size(), 0.0);

    RCLCPP_INFO_STREAM(get_logger(), "controller started");
}

void
BarrettHandController::joint_state_cb()
{
    _hand->update();
    _joint_state.header.stamp = rclcpp::Node::now();

    const auto& hi = _hand->getInnerLinkPosition();
    for (size_t i = 0; i < 4; ++i)
        _joint_state.position[i] = hi[i];

    if (_hand->hasFingertipTorqueSensors())
    {
        const auto&     torques = _hand->getFingertipTorque();
        for (size_t i = 0; i < 4; ++i)
            _joint_state.effort[i] = torques[i];
    }

    _joint_state_pub->publish(_joint_state);
}

void
BarrettHandController::tactile_state_cb()
{
    _hand->update();

    const auto&         tactile_pucks = _hand->getTactilePucks();
    tactile_states_t    tactile_states;
    tactile_states.header.stamp = rclcpp::Node::now();
    tactile_states.tactile_states.resize(tactile_pucks.size());

    for (size_t i = 0; i < tactile_pucks.size(); ++i)
    {
        auto&           tactile_state = tactile_states.tactile_states[i];
        const auto&     pressures = tactile_pucks[i]->getTactileData();
        for (ssize_t j = 0; j < pressures.size(); ++j)
        {
            int value = (int)(pressures[j]*256.0)/102;
            tactile_state.tactile_state[j] = pressures[j];
            int c = 0;
            int chunk;

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

    _tactile_state_pub->publish(tactile_states);
}

void
BarrettHandController::command_cb(msg_p<float64_multi_array_t> command)
{
    if (command->data.size() != 4)
    {
        RCLCPP_ERROR(get_logger(), "Illegal input command data size[%ld]",
                     command->data.size());
        return;
    }

    _hand->trapezoidalMove(barrett::Hand::jp_type(command->data[0],
                                                  command->data[1],
                                                  command->data[2],
                                                  command->data[3]),
                           barrett::Hand::WHOLE_HAND, true);
}

void
BarrettHandController::finger_position_cb(req_cp<finger_pos_t> req,
                                          res_p<finger_pos_t>  res)
{
    RCLCPP_INFO(get_logger(),
                "Moving BarrettHand to Finger Positions %.3f, %.3f, %.3f radians",
                req->position[0], req->position[1], req->position[2]);
    _hand->trapezoidalMove(barrett::Hand::jp_type(req->position[0],
                                                  req->position[1],
                                                  req->position[2],
                                                  0.0),
                           barrett::Hand::GRASP, true);
    res->response = true;
}

void
BarrettHandController::grasp_position_cb(req_cp<grasp_pos_t> req,
                                         res_p<grasp_pos_t>  res)
{
    RCLCPP_INFO(get_logger(), "Moving BarrettHand Grasp: %.3f radians",
                req->position);

    _hand->trapezoidalMove(barrett::Hand::jp_type(req->position),
                           barrett::Hand::GRASP, true);
    res->response = true;
}

void
BarrettHandController::spread_position_cb(req_cp<spread_pos_t> req,
                                          res_p<spread_pos_t>  res)
{
    RCLCPP_INFO(get_logger(), "Moving BarrettHand Spread: %.3f radians",
                req->position);

    _hand->trapezoidalMove(barrett::Hand::jp_type(req->position),
                           barrett::Hand::SPREAD, false);
    res->response = true;
}

void
BarrettHandController::finger_velocity_cb(req_cp<finger_vel_t> req,
                                          res_p<finger_vel_t>  res)
{
    RCLCPP_INFO(get_logger(),
                "Moving BarrettHand Finger Velocities: %.3f, %.3f, %3.f rad/s",
                req->velocity[0], req->velocity[1], req->velocity[2]);

    _hand->velocityMove(barrett::Hand::jv_type(req->velocity[0],
                                               req->velocity[1],
                                               req->velocity[2],
                                               0.0),
                        barrett::Hand::GRASP);
    res->response = true;
}

void
BarrettHandController::grasp_velocity_cb(req_cp<grasp_vel_t> req,
                                         res_p<grasp_vel_t>  res)
{
    RCLCPP_INFO(get_logger(), "Moving BarrettHand Grasp Velocity: %.3f rad/s",
                req->velocity);

    _hand->velocityMove(barrett::Hand::jv_type(req->velocity),
                        barrett::Hand::GRASP);
    res->response = true;
}

void
BarrettHandController::spread_velocity_cb(req_cp<spread_vel_t> req,
                                          res_p<spread_vel_t>  res)
{
    RCLCPP_INFO(get_logger(),
                "Moving BarrettHand Spread Velocity: %.3f rad/s",
                req->velocity);

    _hand->velocityMove(barrett::Hand::jv_type(req->velocity),
                        barrett::Hand::SPREAD);
    res->response = true;
}

void
BarrettHandController::idle_cb(req_cp<trigger_t>, res_p<trigger_t> res)
{
    RCLCPP_INFO(get_logger(), "Idling Barrett Hand");

    _hand->idle();
    res->success = true;
}

void
BarrettHandController::open_close_cb(req_cp<set_bool_t> req,
                                     res_p<set_bool_t>  res, bool spread)
{
    RCLCPP_INFO(get_logger(), "%s the BarrettHand %s",
                (req->data ? "closing" : "opening"),
                (spread    ? "SPREAD"  : "GRASP"));

    const auto  axis = (spread ? barrett::Hand::SPREAD : barrett::Hand::GRASP);
    if (req->data)
        _hand->close(axis, true);
    else
        _hand->open(axis, true);
    res->success = true;
}
}       // namespace aist_barrett

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(aist_barrett::BarrettHandController)
