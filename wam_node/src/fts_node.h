#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace wam_node
{
class FtsNode : public rclcpp::Node
{
  private:
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    template <class MSG>
    using pub_p         = typename rclcpp::Publisher<MSG>::SharedPtr;
    using timer_p       = rclcpp::TimerBase::SharedPtr;
    template <class SRV>
    using srv_p         = typename rclcpp::Service<SRV>::SharedPtr;
    template <class SRV>
    using req_cp        = typename SRV::Request::ConstSharedPtr;
    template <class SRV>
    using res_p         = typename SRV::Response::SharedPtr;

    using trigger_t     = std_srvs::srv::Trigger;
    using wrench_t      = geometry_msgs::msg::Wrench;

  public:
    explicit    FtsNode(barrett::ForceTorqueSensor* fts)
                  :Node("FtsNode"),
                   fts_(fts),
                   wrench_pub_(create_publisher<wrench_t>("/FTS/States", 100)),
                   wrench_pub_timer_(create_wall_timer(
                                         2ms,
                                         std::bind(&FtsNode::wrench_cb,
                                                   this))),
                   tare_srv_(create_service<trigger_t>(
                                 "/FTS/Tare",
                                 std::bind(&FtsNode::tare_cb, this,
                                           std::placeholders::_1,
                                           std::placeholders::_2)))
                {
                    fts_->tare();
                }

  private:
    void        wrench_cb()
                {
                    fts_->update();
                    const auto  cf = barrett::math::saturate(fts_->getForce(),
                                                             99.99);
                    const auto  ct = barrett::math::saturate(fts_->getTorque(),
                                                             9.999);
                    wrench_t    wrench;
                    wrench.force.x  = cf[0];
                    wrench.force.y  = cf[1];
                    wrench.force.z  = cf[2];
                    wrench.torque.x = ct[0];
                    wrench.torque.y = ct[1];
                    wrench.torque.z = ct[2];
                    wrench_pub_->publish(wrench);
                }
    void        tare_cb(req_cp<trigger_t>, res_p<trigger_t> res)
                {
                    RCLCPP_INFO(get_logger(), "Taring FTS");
                    fts_->tare();
                    res->success = true;
                }

  private:
    barrett::ForceTorqueSensor* const   fts_;
    const pub_p<wrench_t>               wrench_pub_;
    const timer_p                       wrench_pub_timer_;
    const srv_p<trigger_t>              tare_srv_;
};
}       // namespace wam_node
