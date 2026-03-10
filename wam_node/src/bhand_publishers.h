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
    using tactile_state_t       = bhand_msgs::msg::TactileState;
    using tactile_states_t      = bhand_msgs::msg::TactileStateArray;
    using finger_tip_torques_t  = bhand_msgs::msg::FingerTipTorques;

  public:
    explicit BhandPublishers(barrett::Hand& hand, bool found_wam)
        :Node("BhandPublishers"),
         hand_(hand),
         found_wam_(found_wam),
         found_tactile_sensors_(hand_.hasTactSensors()),
         found_ft_torque_(hand_.hasFingertipTorqueSensors()),
         bhand_joint_position_msg_(),
         tactile_state_array_(),
         finger_tip_torque_(),
         // cf_(),
         // ct_()
         bhand_joint_position_pub_(
             found_wam_ ?
             create_publisher<joint_state_t>("/joint_states", 100) :
             nullptr),
         bhand_joint_position_pub_timer_(
             found_wam_ ?
             create_wall_timer(
                 2ms,
                 std::bind(&BhandPublishers::publishJointPositions, this)) :
             nullptr),
         tactile_state_pub_(
             found_tactile_sensors_ ?
             create_publisher<tactile_states_t>("/bhand/TactileStates", 100) :
             nullptr),
         finger_tip_torque_pub_(
             found_ft_torque_ ?
             create_publisher<finger_tip_torques_t>("/bhand/FingertipTorques",
                                                    100) :
             nullptr),
         sensor_pub_timer_(
             create_wall_timer(20ms,
                               std::bind(&BhandPublishers::publishSensors,
                                         this)))
    {
      //kBhandJointNames defined in wam_publishers.h
        bhand_joint_position_msg_.position.resize(kBhandJointNames.size());
        bhand_joint_position_msg_.name = kBhandJointNames;

        if (found_tactile_sensors_)
        {
            RCLCPP_INFO(get_logger(), "Found Tactile Sensors");
        }
        if (found_ft_torque_)
        {
            RCLCPP_INFO(get_logger(), "Found FingerTip Torque Sensors");
        }
    }

  private:
    void        publishJointPositions();
    void        publishSensors();

  private:
    barrett::Hand&                      hand_;
    const bool                          found_wam_;
    const bool                          found_tactile_sensors_;
    const bool                          found_ft_torque_;
    joint_state_t                       bhand_joint_position_msg_;
    tactile_states_t                    tactile_state_array_;
    finger_tip_torques_t                finger_tip_torque_;
    // cf_type                             cf_;
    // ct_type                             ct_;

  // function to publish barrett hand joint states
    const pub_p<joint_state_t>          bhand_joint_position_pub_;
    const timer_p                       bhand_joint_position_pub_timer_;

  // function to publish barrett hand joint states
    const pub_p<tactile_states_t>       tactile_state_pub_;
    const pub_p<finger_tip_torques_t>   finger_tip_torque_pub_;
    const timer_p                       sensor_pub_timer_;
};

void
BhandPublishers::publishJointPositions()
{
  //hand_.update();
  // barrett::Hand::jp_type hi = hand_.getInnerLinkPosition(); // get finger positions information
  // barrett::Hand::jp_type ho = hand_.getOuterLinkPosition();
    const auto& hi = hand_.getInnerLinkPosition();
    const auto& ho = hand_.getOuterLinkPosition();
    for (int i = 0; i < 3; i++)
    {
        bhand_joint_position_msg_.position[i+2] = hi[i];
    }
    for (int j = 0; j < 3; j++)
    {
        bhand_joint_position_msg_.position[j+5] = ho[j];
    }
    bhand_joint_position_msg_.position[0] = -hi[3];
    bhand_joint_position_msg_.position[1] =  hi[3];

    bhand_joint_position_pub_->publish(bhand_joint_position_msg_);
}

void
BhandPublishers::publishSensors()
{
    if (found_tactile_sensors_ || found_ft_torque_)
    {
        hand_.update();
    }

    if (found_tactile_sensors_)
    {
        // std::vector<barrett::TactilePuck*> tactile_pucks = hand_.getTactilePucks();
        const auto& tactile_pucks = hand_.getTactilePucks();
        tactile_state_array_.tactile_states.resize(tactile_pucks.size());
        for (unsigned i = 0; i < tactile_pucks.size(); i++)
        {
            tactile_state_t msg;
            // barrett::TactilePuck::v_type pressures(tactile_pucks[i]->getTactile
            //                                        Data());
            const auto& pressures = tactile_pucks[i]->getTactileData();
            for (int j = 0; j < pressures.size(); j++)
            {
                int value = (int)(pressures[j]*256.0)/102;
                msg.tactile_state[j] = pressures[j];
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
                msg.normalized_tactile_state[j] = c - 5;
            }
            tactile_state_array_.tactile_states[i] = msg;
        }
        tactile_state_pub_->publish(tactile_state_array_);
    }

    if (found_ft_torque_)
    {
        finger_tip_torque_.torque = hand_.getFingertipTorque();
        finger_tip_torque_pub_->publish(finger_tip_torque_);
    }
}
}       // namespace wam_node
