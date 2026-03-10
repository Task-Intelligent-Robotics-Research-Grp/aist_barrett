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
    using req_p         = std::shared_ptr<typename SRV::Request>;
    template <class SRV>
    using res_p         = std::shared_ptr<typename SRV::Response>;
    using hdr_p         = std::shared_ptr<rmw_request_id_t>;

    using trigger_t     = std_srvs::srv::Trigger;
    using wrench_t      = geometry_msgs::msg::Wrench;

  public:
    explicit    FtsNode(barrett::ForceTorqueSensor* fts)
                  :Node("FtsNode"), fts_(fts)
                {
                    fts_->tare();
                    fts_pub_ = create_publisher<wrench_t>("/FTS/States", 100);
                    fts_pub_timer_
                        = create_wall_timer(2ms,
                                            std::bind(&FtsNode::publishFTS,
                                                      this));
                    auto tare_srv_cb =
                        [this](const hdr_p request_header,
                               const req_p<trigger_t> request,
                               res_p<trigger_t> response)
                        {
                          /*To avoid compiler warning for unused variable*/
                            (void)request_header;
                            (void)request;
                            RCLCPP_INFO(get_logger(), "Taring FTS");
                            fts_->tare();
                            response->success = true;
                        };
                    tare_srv_ = create_service<trigger_t>("/FTS/Tare",
                                                          tare_srv_cb);
                }

  private:
    void        publishFTS()
                {
                    fts_->update();
                    cf_ = barrett::math::saturate(fts_->getForce(), 99.99);
                    ct_ = barrett::math::saturate(fts_->getTorque(), 9.999);
                    fts_state_.force.x = cf_[0];
                    fts_state_.force.y = cf_[1];
                    fts_state_.force.z = cf_[2];
                    fts_state_.torque.x = ct_[0];
                    fts_state_.torque.y = ct_[1];
                    fts_state_.torque.z = ct_[2];
                    fts_pub_->publish(fts_state_);
                }

  private:
    barrett::ForceTorqueSensor* const   fts_;
    srv_p<trigger_t>                    tare_srv_;
    wrench_t                            fts_state_;
    pub_p<wrench_t>                     fts_pub_;
    timer_p                             fts_pub_timer_;

    cf_type                             cf_;
    ct_type                             ct_;
};
}       // namespace wam_node
