#ifndef WAM_NODE_SRC_BHAND_SERVICES_H_
#define WAM_NODE_SRC_BHAND_SERVICES_H_

class BhandServices : public rclcpp::Node
{
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
  public:
    explicit BhandServices(Hand& hand)
        :Node("BhandServices"), hand_(hand)
    {
      // BarrettHand Idle callback
        auto bhand_idle_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request>
                   request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void)request;
                RCLCPP_INFO(this->get_logger(), "Idling Barrett Hand");
                this->hand_.idle();
                response->success = true;
            };

      // BarrettHand Finger Position Callback
        auto finger_position_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<bhand_msgs::srv::FingerPosition::Request>
                   request,
                   std::shared_ptr<bhand_msgs::srv::FingerPosition::Response>
                   response) -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(this->get_logger(),
                            "Moving BarrettHand to Finger Positions %.3f, %.3f, %.3f radians",
                            request->position[0], request->position[1],
                            request->position[2]);
                this->hand_.trapezoidalMove(Hand::jp_type(request->position[0],
                                                          request->position[1],
                                                          request->position[2],
                                                          0.0),
                                            Hand::GRASP, false);
                response->response = true;
            };

      // BarrettHand Grasp Position Callback
        auto grasp_position_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<bhand_msgs::srv::GraspPosition::Request>
                   request,
                   std::shared_ptr<bhand_msgs::srv::GraspPosition::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(this->get_logger(),
                            "Moving BarrettHand Grasp: %.3f radians",
                            request->position);
                this->hand_.trapezoidalMove(Hand::jp_type(request->position),
                                            Hand::GRASP, false);
                response->response = true;
            };

      // BarrettHand Spread Position Callback
        auto spread_position_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<bhand_msgs::srv::SpreadPosition::Request>
                   request,
                   std::shared_ptr<bhand_msgs::srv::SpreadPosition::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(this->get_logger(), "Moving BarrettHand Spread: %.3f radians",
                            request->position);
                this->hand_.trapezoidalMove(Hand::jp_type(request->position),
                                            Hand::SPREAD, false);
                response->response = true;
            };

      // BarrettHand Finger Velocity Callback
        auto finger_velocity_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<bhand_msgs::srv::FingerVelocity::Request>
                   request,
                   std::shared_ptr<bhand_msgs::srv::FingerVelocity::Response>
                   response) -> void
             {
               /*To avoid compiler warning for unused variable*/
                 (void)request_header;
                 RCLCPP_INFO(this->get_logger(),
                             "Moving BarrettHand Finger Velocities: %.3f, %.3f, %3.f rad/s",
                             request->velocity[0], request->velocity[1],
                             request->velocity[2]);
                 this->hand_.velocityMove(Hand::jv_type(request->velocity[0],
                                                        request->velocity[1],
                                                        request->velocity[2],
                                                        0.0),
                                          Hand::GRASP);
                response->response = true;
            };

      // BarrettHand Grasp Velocity Callback
        auto grasp_velocity_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<bhand_msgs::srv::GraspVelocity::Request>
                   request,
                   std::shared_ptr<bhand_msgs::srv::GraspVelocity::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(this->get_logger(),
                            "Moving BarrettHand Grasp Velocity: %.3f rad/s",
                            request->velocity);
                this->hand_.velocityMove(Hand::jv_type(request->velocity),
                                         Hand::GRASP);
                response->response = true;
            };

      // BarrettHand Spread Velocity Callback
        auto spread_velocity_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<bhand_msgs::srv::SpreadVelocity::Request>
                   request,
                   std::shared_ptr<bhand_msgs::srv::SpreadVelocity::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                RCLCPP_INFO(this->get_logger(),
                            "Moving BarrettHand Spread Velocity: %.3f rad/s",
                            request->velocity);
                this->hand_.velocityMove(Hand::jv_type(request->velocity),
                                         Hand::SPREAD);
                response->response = true;
            };

      // BarrettHand Open Grasp Callback
        auto open_grasp_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void)request;
                RCLCPP_INFO(this->get_logger(),
                            "Opening the BarrettHand Grasp");
                this->hand_.open(Hand::GRASP, false);
                response->success = true;
            };

        auto close_grasp_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void) request;
                RCLCPP_INFO(this->get_logger(),
                            "Closing the BarrettHand Grasp");
                this->hand_.close(Hand::GRASP, false);
                response->success = true;
            };

      // BarrettHand Open Spread Callback
        auto open_spread_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void) request;
                RCLCPP_INFO(this->get_logger(),
                            "Opening the BarrettHand Spread");
                this->hand_.open(Hand::SPREAD, false);
                response->success = true;
            };

      // BarrettHand Close Spread Callback
        auto close_spread_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request>
                   request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void) request;
                RCLCPP_INFO(this->get_logger(),
                            "Closing the BarrettHand Spread");
                this->hand_.close(Hand::SPREAD, false);
                response->success = true;
            };

      // Create Services
        finger_position_srv_
            = create_service<bhand_msgs::srv::FingerPosition>(
                "/bhand/moveToFingerPositions", finger_position_cb);
        grasp_position_srv_
            = create_service<bhand_msgs::srv::GraspPosition>(
                "/bhand/moveToGraspPosition", grasp_position_cb);
        spread_position_srv_
            = create_service<bhand_msgs::srv::SpreadPosition>(
                "/bhand/moveToSpreadPosition", spread_position_cb);
        finger_velocity_srv_
            = create_service<bhand_msgs::srv::FingerVelocity>(
                "/bhand/moveToFingerVelocities", finger_velocity_cb);
        grasp_velocity_srv_
            = create_service<bhand_msgs::srv::GraspVelocity>(
                "/bhand/moveToGraspVelocity", grasp_velocity_cb);
        spread_velocity_srv_
            = create_service<bhand_msgs::srv::SpreadVelocity>(
                "/bhand/moveToSpreadVelocity", spread_velocity_cb);
        idle_srv_
            = create_service<std_srvs::srv::Trigger>(
                "/bhand/idle", bhand_idle_cb);
        open_grasp_srv_
            = create_service<std_srvs::srv::Trigger>(
                "/bhand/openGrasp", open_grasp_cb);
        close_grasp_srv_
            = create_service<std_srvs::srv::Trigger>(
                "/bhand/closeGrasp", close_grasp_cb);
        open_spread_srv_
            = create_service<std_srvs::srv::Trigger>(
                "/bhand/openSpread", open_spread_cb);
        close_spread_srv_
            = create_service<std_srvs::srv::Trigger>(
                "/bhand/closeSpread", close_spread_cb);
    }

  protected:
    Hand& hand_;
    rclcpp::Service<bhand_msgs::srv::FingerPosition>::SharedPtr finger_position_srv_;
    rclcpp::Service<bhand_msgs::srv::GraspPosition>::SharedPtr grasp_position_srv_;
    rclcpp::Service<bhand_msgs::srv::SpreadPosition>::SharedPtr spread_position_srv_;
    rclcpp::Service<bhand_msgs::srv::FingerVelocity>::SharedPtr finger_velocity_srv_;
    rclcpp::Service<bhand_msgs::srv::GraspVelocity>::SharedPtr grasp_velocity_srv_;
    rclcpp::Service<bhand_msgs::srv::SpreadVelocity>::SharedPtr spread_velocity_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr idle_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_grasp_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_grasp_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_spread_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_spread_srv_;
};
#endif  // WAM_NODE_SRC_BHAND_SERVICES_H_
