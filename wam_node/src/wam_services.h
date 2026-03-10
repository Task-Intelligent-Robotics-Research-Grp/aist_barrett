#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <wam_msgs/srv/cart_orientation_move.hpp>
#include <wam_msgs/srv/cart_pose_move.hpp>
#include <wam_msgs/srv/cart_position_move.hpp>
#include <wam_msgs/srv/joint_move.hpp>
#include <wam_msgs/srv/velocity_limit.hpp>

namespace wam_node
{
template <size_t DOF>
class WamServices : public rclcpp::Node
{
  private:
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    template <class SRV>
    using srv_p  = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p  = typename SRV::Response::SharedPtr;

    using trigger_t               = std_srvs::srv::Trigger;
    using set_bool_t              = std_srvs::srv::SetBool;
    using cart_position_move_t    = wam_msgs::srv::CartPositionMove;
    using cart_orientation_move_t = wam_msgs::srv::CartOrientationMove;
    using cart_pose_move_t        = wam_msgs::srv::CartPoseMove;
    using joint_move_t            = wam_msgs::srv::JointMove;
    using velocity_limit_t        = wam_msgs::srv::VelocityLimit;

  public:
    explicit
    WamServices(barrett::systems::Wam<DOF>* const wam,
                barrett::ProductManager& pm, barrett::Hand* hand)
        :Node("WamServices"),
         wam_(wam), pm_(pm), hand_(hand)
    {
        auto move_home_cb =
            [this](req_cp<trigger_t>, res_p<trigger_t> res)
            {
              /*To avoid compiler warning for unused variable*/
                RCLCPP_INFO(get_logger(), "Moving WAM Home");
                if (hand_)
                {
                    hand_->open(barrett::Hand::GRASP, true);
                    hand_->close(barrett::Hand::SPREAD, true);
                }
                wam_->moveHome();
                res->success = true;
            };

        auto gravity_compensate_cb =
            [this](req_cp<set_bool_t> req, res_p<set_bool_t> res)
            {
                if (req->data == true)
                {
                    RCLCPP_INFO(get_logger(),
                                "Gravity Compensation turned on");
                    wam_->gravityCompensate();
                    res->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Gravity Compensation turned off");
                    wam_->gravityCompensate(false);
                    res->success = true;
                }
            };

        auto idle_cb =
            [this](req_cp<trigger_t>, res_p<trigger_t> res)
            {
                RCLCPP_INFO(get_logger(), "Idling WAM");
                wam_->idle();
                res->success = true;
            };

        auto hold_cart_position_cb =
            [this](req_cp<set_bool_t> req, res_p<set_bool_t> res)
            {
                if (req->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Tool Position");
                    wam_->moveTo(wam_->getToolPosition(), false);
                    res->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Releasing WAM Tool Position hold");
                    wam_->idle();
                    res->success = true;
                }
            };

        auto hold_joint_positioin_cb =
            [this](req_cp<set_bool_t> req, res_p<set_bool_t> res)
            {
                if (req->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Joint Positions");
                    wam_->moveTo(wam_->getJointPositions(), false);
                    res->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Releasing WAM Joint Positions hold");
                    wam_->idle();
                    res->success = true;
                }
            };

        auto hold_cart_orientation_ =
            [this](req_cp<set_bool_t> req, res_p<set_bool_t> res)
            {
                if (req->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Tool Orientation");
                    wam_->moveTo(wam_->getToolOrientation(), false);
                    res->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Releasing WAM Tool Orientation hold");
                    wam_->idle();
                    res->success = true;
                }
            };

        auto hold_cart_pose_ =
            [this](req_cp<set_bool_t> req, res_p<set_bool_t> res)
            {
                if (req->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Tool Pose");
                    wam_->moveTo(wam_->getToolPose(), false);
                    res->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Releasing WAM Tool Pose hold");
                    wam_->idle();
                    res->success = true;
                }
            };

        auto joint_move_cb =
            [this](req_cp<joint_move_t> req, res_p<joint_move_t> res)
            {
                jp_type jp;
                if (DOF != req->joint_state.position.size())
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Invalid Command. Please enter %lu Joint Positions", DOF);
                    res->response = false; //notify client that move not initialized
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving WAM to commanded Joint Positions");
                    for (int i = 0; i < (int)DOF; i++)
                    {
                        jp(i) = req->joint_state.position[i];
                    }
                    wam_->moveTo(jp);
                    res->response = true;
                }
            };

        auto cart_position_move_cb =
            [this](req_cp<cart_position_move_t> req,
                   res_p<cart_position_move_t>  res)
            {
                cp_type cp;
                RCLCPP_INFO(get_logger(),
                            "Moving WAM to commanded Cartesian Positions");
                cp << req->position.x,
                      req->position.y,
                      req->position.z;
                wam_->moveTo(cp, false);
                res->response = true;
            };

        auto cart_orientation_move_cb =
            [this](req_cp<cart_orientation_move_t> req,
                   res_p<cart_orientation_move_t>  res)
            {
                Eigen::Quaterniond goal_ortn;
                RCLCPP_INFO(get_logger(),
                            "Moving WAM to commanded Cartesian Orientation");
                goal_ortn.x() = req->orientation.x;
                goal_ortn.y() = req->orientation.y;
                goal_ortn.z() = req->orientation.z;
                goal_ortn.w() = req->orientation.w;
                wam_->moveTo(goal_ortn, false);
                res->response = true;
            };

        auto cart_pose_move_cb =
            [this](req_cp<cart_pose_move_t> req, res_p<cart_pose_move_t> res)
            {
                pose_type pose;
                RCLCPP_INFO(get_logger(),
                            "Moving WAM to commanded Cartesian Pose");
                pose.get<0>() << req->pose.position.x,
                                 req->pose.position.y,
                                 req->pose.position.z;
                res->response = true;
                pose.get<1>().x() = req->pose.orientation.x;
                pose.get<1>().y() = req->pose.orientation.y;
                pose.get<1>().z() = req->pose.orientation.z;
                pose.get<1>().w() = req->pose.orientation.w;
                wam_->moveTo(pose, false);
                res->response = true;
            };

        auto set_velocity_limit_cb =
            [this](req_cp<velocity_limit_t> req, res_p<velocity_limit_t>)
            {
                RCLCPP_INFO(get_logger(), "Setting WAM Velocity Limit");
                pm_.getSafetyModule()->setVelocityLimit(req->velocity_limit);
            };

        move_to_home_srv_
            = create_service<trigger_t>("/wam/moveHome", move_home_cb);
        gravity_comp_srv_
            = create_service<set_bool_t>("/wam/gravityCompensate",
                                         gravity_compensate_cb);
        idle_srv_
            = create_service<trigger_t>("/wam/idle", idle_cb);
        hold_cart_position_srv_
            = create_service<set_bool_t>("/wam/holdCartPosition",
                                         hold_cart_position_cb);
        hold_joint_position_srv_
            = create_service<set_bool_t>("/wam/holdJointPosition",
                                         hold_joint_positioin_cb);
        hold_cart_orientation_srv_
            = create_service<set_bool_t>("/wam/holdCartOrientation",
                                         hold_cart_orientation_);
        hold_cart_pose_srv_
            = create_service<set_bool_t>("/wam/holdCartPose", hold_cart_pose_);
        joint_move_srv_
            = create_service<joint_move_t>("/wam/moveToJointPosition",
                                           joint_move_cb);
        cart_position_move_srv_
            = create_service<cart_position_move_t>("/wam/moveToCartPosition",
                                                   cart_position_move_cb);
        cart_orientation_move_srv_
            = create_service<cart_orientation_move_t>(
                "/wam/moveToCartOrientation", cart_orientation_move_cb);
        cart_pose_move_srv_
            = create_service<cart_pose_move_t>("/wam/moveToCartPose",
                                               cart_pose_move_cb);
        set_velocity_limit_srv_
            = create_service<velocity_limit_t>("/wam/setVelocityLimit",
                                               set_velocity_limit_cb);
    }

  private:
    barrett::systems::Wam<DOF>* const   wam_;
    barrett::ProductManager&            pm_;
    barrett::Hand* const                hand_;
    srv_p<trigger_t>                    move_to_home_srv_;
    srv_p<set_bool_t>                   gravity_comp_srv_;
    srv_p<trigger_t>                    idle_srv_;
    srv_p<set_bool_t>                   hold_joint_position_srv_;
    srv_p<set_bool_t>                   hold_cart_position_srv_;
    srv_p<set_bool_t>                   hold_cart_orientation_srv_;
    srv_p<set_bool_t>                   hold_cart_pose_srv_;
    srv_p<joint_move_t>                 joint_move_srv_;
    srv_p<cart_position_move_t>         cart_position_move_srv_;
    srv_p<cart_orientation_move_t>      cart_orientation_move_srv_;
    srv_p<cart_pose_move_t>             cart_pose_move_srv_;
    srv_p<velocity_limit_t>             set_velocity_limit_srv_;
};
}
