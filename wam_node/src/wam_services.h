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
    using srv_p = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_p = std::shared_ptr<typename SRV::Request>;
    template <class SRV>
    using res_p = std::shared_ptr<typename SRV::Response>;
    using hdr_p = std::shared_ptr<rmw_request_id_t>;

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
            [this](const hdr_p request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void)request;
                RCLCPP_INFO(get_logger(), "Moving WAM Home");
                if (hand_)
                {
                    hand_->open(barrett::Hand::GRASP, true);
                    hand_->close(barrett::Hand::SPREAD, true);
                }
                wam_->moveHome();
                response->success = true;
            };

        auto gravity_compensate_cb =
            [this](const hdr_p request_header,
                   const req_p<set_bool_t> request, res_p<set_bool_t> response)
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(get_logger(),
                                "Gravity Compensation turned on");
                    wam_->gravityCompensate();
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Gravity Compensation turned off");
                    wam_->gravityCompensate(false);
                    response->success = true;
                }
            };

        auto idle_cb =
            [this](const hdr_p request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
                (void)request_header;
                (void)request;
                RCLCPP_INFO(get_logger(), "Idling WAM");
                wam_->idle();
                response->success = true;
            };

        auto hold_cart_position_cb =
            [this](const hdr_p request_header,
                   const req_p<set_bool_t> request, res_p<set_bool_t> response)
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Tool Position");
                    wam_->moveTo(wam_->getToolPosition(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Releasing WAM Tool Position hold");
                    wam_->idle();
                    response->success = true;
                }
            };

        auto hold_joint_positioin_cb =
            [this](const hdr_p request_header,
                   const req_p<set_bool_t> request, res_p<set_bool_t> response)
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Joint Positions");
                    wam_->moveTo(wam_->getJointPositions(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Releasing WAM Joint Positions hold");
                    wam_->idle();
                    response->success = true;
                }
            };

        auto hold_cart_orientation_ =
            [this](const hdr_p request_header,
                   const req_p<set_bool_t> request, res_p<set_bool_t> response)
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Tool Orientation");
                    wam_->moveTo(wam_->getToolOrientation(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Releasing WAM Tool Orientation hold");
                    wam_->idle();
                    response->success = true;
                }
            };

        auto hold_cart_pose_ =
            [this](const hdr_p request_header,
                   const req_p<set_bool_t> request, res_p<set_bool_t> response)
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(get_logger(), "Holding WAM Tool Pose");
                    wam_->moveTo(wam_->getToolPose(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Releasing WAM Tool Pose hold");
                    wam_->idle();
                    response->success = true;
                }
            };

        auto joint_move_cb =
            [this](const hdr_p request_header,
                   const req_p<joint_move_t> request,
                   res_p<joint_move_t> response)
            {
                (void)request_header;
                jp_type jp;
                if (DOF != request->joint_state.position.size())
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Invalid Command. Please enter %lu Joint Positions", DOF);
                    response->response = false; //notify client that move not initialized
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving WAM to commanded Joint Positions");
                    for (int i = 0; i < (int)DOF; i++)
                    {
                        jp(i) = request->joint_state.position[i];
                    }
                    wam_->moveTo(jp);
                    response->response = true;
                }
            };

        auto cart_position_move_cb =
            [this](const hdr_p request_header,
                   const req_p<cart_position_move_t> request,
                   res_p<cart_position_move_t> response)
            {
                (void)request_header;
                cp_type cp;
                RCLCPP_INFO(get_logger(),
                            "Moving WAM to commanded Cartesian Positions");
                cp << request->position.x,
                      request->position.y,
                      request->position.z;
                wam_->moveTo(cp, false);
                response->response = true;
            };

        auto cart_orientation_move_cb =
            [this](const hdr_p request_header,
                   const req_p<cart_orientation_move_t> request,
                   res_p<cart_orientation_move_t> response)
            {
                (void)request_header;
                Eigen::Quaterniond goal_ortn;
                RCLCPP_INFO(get_logger(),
                            "Moving WAM to commanded Cartesian Orientation");
                goal_ortn.x() = request->orientation.x;
                goal_ortn.y() = request->orientation.y;
                goal_ortn.z() = request->orientation.z;
                goal_ortn.w() = request->orientation.w;
                wam_->moveTo(goal_ortn, false);
                response->response = true;
            };

        auto cart_pose_move_cb =
            [this](const hdr_p request_header,
                   const req_p<cart_pose_move_t> request,
                   res_p<cart_pose_move_t> response)
            {
                (void)request_header;
                pose_type pose;
                RCLCPP_INFO(get_logger(),
                            "Moving WAM to commanded Cartesian Pose");
                pose.get<0>() << request->pose.position.x,
                                 request->pose.position.y,
                                 request->pose.position.z;
                response->response = true;
                pose.get<1>().x() = request->pose.orientation.x;
                pose.get<1>().y() = request->pose.orientation.y;
                pose.get<1>().z() = request->pose.orientation.z;
                pose.get<1>().w() = request->pose.orientation.w;
                wam_->moveTo(pose, false);
                response->response = true;
            };

        auto set_velocity_limit_cb =
            [this](const hdr_p request_header,
                   const req_p<velocity_limit_t> request,
                   res_p<velocity_limit_t> response)
            {
                (void)request_header;
                RCLCPP_INFO(get_logger(), "Setting WAM Velocity Limit");
                pm_.getSafetyModule()->setVelocityLimit(request->velocity_limit);
                (void)response;
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
