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
    using req_cp        = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p         = typename SRV::Response::SharedPtr;

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
         hand_(hand),
         finger_position_srv_(create_service<finger_pos_t>(
                                  "/bhand/moveToFingerPositions",
                                  std::bind(&BhandServices::finger_position_cb,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2))),
         grasp_position_srv_(create_service<grasp_pos_t>(
                                 "/bhand/moveToGraspPosition",
                                 std::bind(&BhandServices::grasp_position_cb,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2))),
         spread_position_srv_(create_service<spread_pos_t>(
                                  "/bhand/moveToSpreadPosition",
                                  std::bind(&BhandServices::spread_position_cb,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2))),

         finger_velocity_srv_(create_service<finger_vel_t>(
                                  "/bhand/moveToFingerVelocities",
                                  std::bind(&BhandServices::finger_velocity_cb,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2))),
         grasp_velocity_srv_(create_service<grasp_vel_t>(
                                 "/bhand/moveToGraspVelocity",
                                 std::bind(&BhandServices::grasp_velocity_cb,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2))),
         spread_velocity_srv_(create_service<spread_vel_t>(
                                  "/bhand/moveToSpreadVelocity",
                                  std::bind(&BhandServices::spread_velocity_cb,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2))),
         idle_srv_(create_service<trigger_t>(
                       "/bhand/idle",
                       std::bind(&BhandServices::idle_cb, this,
                                 std::placeholders::_1,
                                 std::placeholders::_2))),
         open_grasp_srv_(create_service<trigger_t>(
                             "/bhand/openGrasp",
                             std::bind(&BhandServices::open_close_cb, this,
                                       std::placeholders::_1,
                                       std::placeholders::_2, false, false))),
         close_grasp_srv_(create_service<trigger_t>(
                              "/bhand/closeGrasp",
                              std::bind(&BhandServices::open_close_cb, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2, true, false))),
         open_spread_srv_(create_service<trigger_t>(
                              "/bhand/openSpread",
                              std::bind(&BhandServices::open_close_cb, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2, false, true))),
         close_spread_srv_(create_service<trigger_t>(
                               "/bhand/closeSpread",
                               std::bind(&BhandServices::open_close_cb, this,
                                         std::placeholders::_1,
                                         std::placeholders::_2, true, true)))
    {
    }

  private:
    void        finger_position_cb(req_cp<finger_pos_t> req,
                                   res_p<finger_pos_t>  res)
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving BarrettHand to Finger Positions %.3f, %.3f, %.3f radians",
                                req->position[0], req->position[1],
                                req->position[2]);
                    hand_->trapezoidalMove(
                        barrett::Hand::jp_type(req->position[0],
                                               req->position[1],
                                               req->position[2],
                                               0.0),
                        barrett::Hand::GRASP, false);
                    res->response = true;
                }
    void        grasp_position_cb(req_cp<grasp_pos_t> req,
                                  res_p<grasp_pos_t>  res)
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving BarrettHand Grasp: %.3f radians",
                                req->position);

                    hand_->trapezoidalMove(
                        barrett::Hand::jp_type(req->position),
                        barrett::Hand::GRASP, false);
                    res->response = true;
                }
    void        spread_position_cb(req_cp<spread_pos_t> req,
                                   res_p<spread_pos_t>  res)
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving BarrettHand Spread: %.3f radians",
                                req->position);

                    hand_->trapezoidalMove(
                        barrett::Hand::jp_type(req->position),
                        barrett::Hand::SPREAD, false);
                    res->response = true;
                }
    void        finger_velocity_cb(req_cp<finger_vel_t> req,
                                   res_p<finger_vel_t>  res)
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving BarrettHand Finger Velocities: %.3f, %.3f, %3.f rad/s",
                                req->velocity[0], req->velocity[1],
                                req->velocity[2]);

                    hand_->velocityMove(
                        barrett::Hand::jv_type(req->velocity[0],
                                               req->velocity[1],
                                               req->velocity[2],
                                               0.0),
                        barrett::Hand::GRASP);
                    res->response = true;
                }
    void        grasp_velocity_cb(req_cp<grasp_vel_t> req,
                                  res_p<grasp_vel_t>  res)
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving BarrettHand Grasp Velocity: %.3f rad/s",
                                req->velocity);

                    hand_->velocityMove(barrett::Hand::jv_type(req->velocity),
                                        barrett::Hand::GRASP);
                    res->response = true;
                }
    void        spread_velocity_cb(req_cp<spread_vel_t> req,
                                   res_p<spread_vel_t>  res)
                {
                    RCLCPP_INFO(get_logger(),
                                "Moving BarrettHand Spread Velocity: %.3f rad/s",
                                req->velocity);

                    hand_->velocityMove(barrett::Hand::jv_type(req->velocity),
                                        barrett::Hand::SPREAD);
                    res->response = true;
                }
    void        idle_cb(req_cp<trigger_t>, res_p<trigger_t> res)
                {
                    RCLCPP_INFO(get_logger(), "Idling Barrett Hand");

                    hand_->idle();
                    res->success = true;
                }
    void        open_close_cb(req_cp<trigger_t>, res_p<trigger_t> res,
                              bool close, bool spread)
                {
                    RCLCPP_INFO(get_logger(), "%s the BarrettHand %s",
                                (close  ? "closing" : "opening"),
                                (spread ? "SPREAD"  : "GRASP"));

                    const auto  axis = (spread ? barrett::Hand::SPREAD
                                               : barrett::Hand::GRASP);
                    if (close)
                        hand_->close(axis, false);
                    else
                        hand_->open(axis, false);
                    res->success = true;
                }

  private:
    barrett::Hand* const        hand_;
    const srv_p<finger_pos_t>   finger_position_srv_;
    const srv_p<grasp_pos_t>    grasp_position_srv_;
    const srv_p<spread_pos_t>   spread_position_srv_;
    const srv_p<finger_vel_t>   finger_velocity_srv_;
    const srv_p<grasp_vel_t>    grasp_velocity_srv_;
    const srv_p<spread_vel_t>   spread_velocity_srv_;
    const srv_p<trigger_t>      idle_srv_;
    const srv_p<trigger_t>      open_grasp_srv_;
    const srv_p<trigger_t>      close_grasp_srv_;
    const srv_p<trigger_t>      open_spread_srv_;
    const srv_p<trigger_t>      close_spread_srv_;
};
}       // namespace wam_node
