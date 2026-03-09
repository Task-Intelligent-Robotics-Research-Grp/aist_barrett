#ifndef WAM_NODE_SRC_FTS_NODE_
#define WAM_NODE_SRC_FTS_NODE_

class FtsNode : public rclcpp::Node {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  explicit FtsNode(ForceTorqueSensor* fts__)
      : Node("FtsNode"), fts_(fts__) {
    fts_->tare();
    fts_pub_ =
        this->create_publisher<geometry_msgs::msg::Wrench>("/FTS/States", 100);
    fts_pub_timer_ =
        this->create_wall_timer(2ms, std::bind(&FtsNode::publishFTS, this));
    auto tare_srv_cb =
        [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
      /*To avoid compiler warning for unused variable*/
      (void)request_header;
      (void)request;
        RCLCPP_INFO(this->get_logger(), "Taring FTS");
        fts_->tare();
        response->success = true;
    };
    tare_srv_ = create_service<std_srvs::srv::Trigger>("/FTS/Tare", tare_srv_cb);
  }

 protected:
  ForceTorqueSensor* fts_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_srv_;
  geometry_msgs::msg::Wrench fts_state_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fts_pub_;
  void publishFTS();
  cf_type cf_;
  ct_type ct_;
  rclcpp::TimerBase::SharedPtr fts_pub_timer_;
};

void FtsNode::publishFTS() {
  fts_->update();
  cf_ = math::saturate(fts_->getForce(), 99.99);
  ct_ = math::saturate(fts_->getTorque(), 9.999);
  fts_state_.force.x = cf_[0];
  fts_state_.force.y = cf_[1];
  fts_state_.force.z = cf_[2];
  fts_state_.torque.x = ct_[0];
  fts_state_.torque.y = ct_[1];
  fts_state_.torque.z = ct_[2];
  fts_pub_->publish(fts_state_);
}

#endif  // WAM_NODE_SRC_FTS_NODE_
