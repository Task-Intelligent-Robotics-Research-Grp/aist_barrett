#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <bhand_msgs/srv/finger_position.hpp>
#include <bhand_msgs/srv/finger_velocity.hpp>
#include <bhand_msgs/srv/grasp_position.hpp>
#include <bhand_msgs/srv/grasp_velocity.hpp>
#include <bhand_msgs/srv/spread_position.hpp>
#include <bhand_msgs/srv/spread_velocity.hpp>

namespace wam_node
{
class BhandServices : public rclcpp::Node
{
  private:
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    template <class SRV>
    using srv_p         = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_p         = std::shared_ptr<typename SRV::Request>;
    template <class SRV>
    using res_p         = std::shared_ptr<typename SRV::Response>;
    using hdr_p         = std::shared_ptr<rmw_request_id_t>;

    using trigger_t     = std_srvs::srv::Trigger;
    using finger_pos_t  = bhand_msgs::srv::FingerPosition;
    using grasp_pos_t   = bhand_msgs::srv::GraspPosition;
    using spread_pos_t  = bhand_msgs::srv::SpreadPosition;
    using finger_vel_t  = bhand_msgs::srv::FingerVelocity;
    using grasp_vel_t   = bhand_msgs::srv::GraspVelocity;
    using spread_vel_t  = bhand_msgs::srv::SpreadVelocity;

  public:
    explicit BhandServices(barrett::Hand* hand)
        :Node("BhandServices"),
         hand_(hand)
    {
      // BarrettHand Idle callback
        auto bhand_idle_cb =
            [this](const hdr_p request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void)request;
                RCLCPP_INFO(get_logger(), "Idling Barrett Hand");
                hand_->idle();
                response->success = true;
            };

      // BarrettHand Finger Position Callback
        auto finger_position_cb =
            [this](const hdr_p request_header,
                   const req_p<finger_pos_t> request,
                   res_p<finger_pos_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(get_logger(),
                            "Moving BarrettHand to Finger Positions %.3f, %.3f, %.3f radians",
                            request->position[0], request->position[1],
                            request->position[2]);
                hand_->trapezoidalMove(
                    barrett::Hand::jp_type(request->position[0],
                                           request->position[1],
                                           request->position[2],
                                           0.0),
                    barrett::Hand::GRASP, false);
                response->response = true;
            };

      // BarrettHand Grasp Position Callback
        auto grasp_position_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const req_p<grasp_pos_t> request,
                   res_p<grasp_pos_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(get_logger(),
                            "Moving BarrettHand Grasp: %.3f radians",
                            request->position);
                hand_->trapezoidalMove(
                    barrett::Hand::jp_type(request->position),
                    barrett::Hand::GRASP, false);
                response->response = true;
            };

      // BarrettHand Spread Position Callback
        auto spread_position_cb =
            [this](const hdr_p request_header,
                   const req_p<spread_pos_t> request,
                   res_p<spread_pos_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(get_logger(), "Moving BarrettHand Spread: %.3f radians",
                            request->position);
                hand_->trapezoidalMove(
                    barrett::Hand::jp_type(request->position),
                    barrett::Hand::SPREAD, false);
                response->response = true;
            };

      // BarrettHand Finger Velocity Callback
        auto finger_velocity_cb =
            [this](const hdr_p request_header,
                   const req_p<finger_vel_t> request,
                   res_p<finger_vel_t> response)
             {
               /*To avoid compiler warning for unused variable*/
                 (void)request_header;
                 RCLCPP_INFO(get_logger(),
                             "Moving BarrettHand Finger Velocities: %.3f, %.3f, %3.f rad/s",
                             request->velocity[0], request->velocity[1],
                             request->velocity[2]);
                 hand_->velocityMove(
                     barrett::Hand::jv_type(request->velocity[0],
                                            request->velocity[1],
                                            request->velocity[2],
                                            0.0),
                     barrett::Hand::GRASP);
                response->response = true;
            };

      // BarrettHand Grasp Velocity Callback
        auto grasp_velocity_cb =
            [this](const hdr_p request_header,
                   const req_p<grasp_vel_t> request,
                   res_p<grasp_vel_t> response) -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(get_logger(),
                            "Moving BarrettHand Grasp Velocity: %.3f rad/s",
                            request->velocity);
                hand_->velocityMove(barrett::Hand::jv_type(request->velocity),
                                   barrett::Hand::GRASP);
                response->response = true;
            };

      // BarrettHand Spread Velocity Callback
        auto spread_velocity_cb =
            [this](const hdr_p request_header,
                   const req_p<spread_vel_t> request,
                   res_p<spread_vel_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(get_logger(),
                            "Moving BarrettHand Spread Velocity: %.3f rad/s",
                            request->velocity);
                hand_->velocityMove(barrett::Hand::jv_type(request->velocity),
                                    barrett::Hand::SPREAD);
                response->response = true;
            };

      // BarrettHand Open Grasp Callback
        auto open_grasp_cb =
            [this](const hdr_p request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void)request;
                RCLCPP_INFO(get_logger(), "Opening the BarrettHand Grasp");
                hand_->open(barrett::Hand::GRASP, false);
                response->success = true;
            };

        auto close_grasp_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void) request;
                RCLCPP_INFO(get_logger(), "Closing the BarrettHand Grasp");
                hand_->close(barrett::Hand::GRASP, false);
                response->success = true;
            };

      // BarrettHand Open Spread Callback
        auto open_spread_cb =
            [this](const hdr_p request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void) request;
                RCLCPP_INFO(get_logger(), "Opening the BarrettHand Spread");
                hand_->open(barrett::Hand::SPREAD, false);
                response->success = true;
            };

      // BarrettHand Close Spread Callback
        auto close_spread_cb =
            [this](const hdr_p request_header,
                   const req_p<trigger_t> request, res_p<trigger_t> response)
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void) request;
                RCLCPP_INFO(get_logger(), "Closing the BarrettHand Spread");
                hand_->close(barrett::Hand::SPREAD, false);
                response->success = true;
            };

      // Create Services
        finger_position_srv_
            = create_service<finger_pos_t>("/bhand/moveToFingerPositions",
                                           finger_position_cb);
        grasp_position_srv_
            = create_service<grasp_pos_t>("/bhand/moveToGraspPosition",
                                          grasp_position_cb);
        spread_position_srv_
            = create_service<spread_pos_t>("/bhand/moveToSpreadPosition",
                                           spread_position_cb);
        finger_velocity_srv_
            = create_service<finger_vel_t>("/bhand/moveToFingerVelocities",
                                           finger_velocity_cb);
        grasp_velocity_srv_
            = create_service<grasp_vel_t>("/bhand/moveToGraspVelocity",
                                          grasp_velocity_cb);
        spread_velocity_srv_
            = create_service<spread_vel_t>("/bhand/moveToSpreadVelocity",
                                           spread_velocity_cb);
        idle_srv_ = create_service<trigger_t>("/bhand/idle", bhand_idle_cb);
        open_grasp_srv_ = create_service<trigger_t>("/bhand/openGrasp",
                                                    open_grasp_cb);
        close_grasp_srv_ = create_service<trigger_t>("/bhand/closeGrasp",
                                                     close_grasp_cb);
        open_spread_srv_ = create_service<trigger_t>("/bhand/openSpread",
                                                     open_spread_cb);
        close_spread_srv_ = create_service<trigger_t>("/bhand/closeSpread",
                                                      close_spread_cb);
    }

  private:
    barrett::Hand* const        hand_;
    srv_p<finger_pos_t>         finger_position_srv_;
    srv_p<grasp_pos_t>          grasp_position_srv_;
    srv_p<spread_pos_t>         spread_position_srv_;
    srv_p<finger_vel_t>         finger_velocity_srv_;
    srv_p<grasp_vel_t>          grasp_velocity_srv_;
    srv_p<spread_vel_t>         spread_velocity_srv_;
    srv_p<trigger_t>            idle_srv_;
    srv_p<trigger_t>            open_grasp_srv_;
    srv_p<trigger_t>            close_grasp_srv_;
    srv_p<trigger_t>            open_spread_srv_;
    srv_p<trigger_t>            close_spread_srv_;
};
}       // namespace wam_node
