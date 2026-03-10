#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <bhand_msgs/msg/finger_tip_torques.hpp>
#include <bhand_msgs/msg/tactile_state.hpp>
#include <bhand_msgs/msg/tactile_state_array.hpp>
#include <barrett/systems.h>
#include <barrett/systems/wam.h>
#include <barrett/units.h>

namespace wam_node
{
class BhandPublishers : public rclcpp::Node
{
  private:
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    template <class MSG>
    using pub_p                 = typename rclcpp::Publisher<MSG>::SharedPtr;
    using timer_p               = rclcpp::TimerBase::SharedPtr;
    using joint_state_t         = sensor_msgs::msg::JointState;
    using tactile_states_t      = bhand_msgs::msg::TactileStateArray;
    using finger_tip_torque_t   = bhand_msgs::msg::FingerTipTorques;

  public:
    explicit BhandPublishers(barrett::Hand* hand, bool found_wam)
        :Node("BhandPublishers"),
         hand_(hand),
         joint_state_(),
         joint_state_pub_(
             found_wam ?
             create_publisher<joint_state_t>("/joint_states", 100) :
             nullptr),
         joint_state_pub_timer_(
             found_wam ?
             create_wall_timer(
                 2ms,
                 std::bind(&BhandPublishers::publishJointPositions, this)) :
             nullptr),
         tactile_state_pub_(
             hand_->hasTactSensors() ?
             create_publisher<tactile_states_t>("/bhand/TactileStates", 100) :
             nullptr),
         finger_tip_torque_pub_(
             hand_->hasFingertipTorqueSensors() ?
             create_publisher<finger_tip_torque_t>("/bhand/FingertipTorques",
                                                   100) :
             nullptr),
         sensor_pub_timer_(
             create_wall_timer(20ms,
                               std::bind(&BhandPublishers::publishSensors,
                                         this)))
    {
      //kBhandJointNames defined in wam_publishers.h
        joint_state_.position.resize(kBhandJointNames.size());
        joint_state_.name = kBhandJointNames;

        if (tactile_state_pub_)
            RCLCPP_INFO(get_logger(), "Found Tactile Sensors");
        if (finger_tip_torque_pub_)
            RCLCPP_INFO(get_logger(), "Found FingerTip Torque Sensors");
    }

  private:
    void        publishJointPositions();
    void        publishSensors();

  private:
    barrett::Hand* const                hand_;

    joint_state_t                       joint_state_;
    const pub_p<joint_state_t>          joint_state_pub_;
    const timer_p                       joint_state_pub_timer_;

    const pub_p<tactile_states_t>       tactile_state_pub_;
    const pub_p<finger_tip_torque_t>    finger_tip_torque_pub_;
    const timer_p                       sensor_pub_timer_;
};

void
BhandPublishers::publishJointPositions()
{
  //hand_->update();
  // barrett::Hand::jp_type hi = hand_->getInnerLinkPosition(); // get finger positions information
  // barrett::Hand::jp_type ho = hand_->getOuterLinkPosition();
    const auto& hi = hand_->getInnerLinkPosition();
    const auto& ho = hand_->getOuterLinkPosition();
    for (size_t i = 0; i < 3; ++i)
    {
        joint_state_.position[i+2] = hi[i];
        joint_state_.position[i+5] = ho[i];
    }
    joint_state_.position[0] = -hi[3];
    joint_state_.position[1] =  hi[3];

    joint_state_pub_->publish(joint_state_);
}

void
BhandPublishers::publishSensors()
{
    if (tactile_state_pub_ || finger_tip_torque_pub_)
        hand_->update();

    if (tactile_state_pub_)
    {
        // std::vector<barrett::TactilePuck*> tactile_pucks = hand_->getTactilePucks();
        const auto&             tactile_pucks = hand_->getTactilePucks();
        tactile_states_t        tactile_states;
        tactile_states.tactile_states.resize(tactile_pucks.size());

        for (size_t i = 0; i < tactile_pucks.size(); ++i)
        {
            auto&       tactile_state = tactile_states.tactile_states[i];
            // barrett::TactilePuck::v_type pressures(tactile_pucks[i]->getTactile
            //                                        Data());
            const auto& pressures = tactile_pucks[i]->getTactileData();
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

        tactile_state_pub_->publish(tactile_states);
    }

    if (finger_tip_torque_pub_)
    {
        finger_tip_torque_t     finger_tip_torque;
        finger_tip_torque.torque = hand_->getFingertipTorque();
        finger_tip_torque_pub_->publish(finger_tip_torque);
    }
}
}       // namespace wam_node
