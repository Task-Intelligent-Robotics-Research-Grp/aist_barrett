#ifndef WAM_NODE_SRC_WAM_SERVICES_H_
#define WAM_NODE_SRC_WAM_SERVICES_H_

template <size_t DOF>
class WamServices : public rclcpp::Node
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  public:
    explicit
    WamServices(systems::Wam<DOF> &wam, ProductManager &pm, Hand* hand,
                bool found_hand)
        :Node("WamServices"),
         wam_(wam), pm_(pm), hand_(hand), found_hand_(found_hand)
    {
        auto move_home_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
            {
              /*To avoid compiler warning for unused variable*/
                (void)request_header;
                (void)request;
                RCLCPP_INFO(this->get_logger(), "Moving WAM Home");
                if (found_hand_)
                {
                    this->hand_->open(Hand::GRASP, true);
                    this->hand_->close(Hand::SPREAD, true);
                }
                this->wam_.moveHome();
                response->success = true;
            };

        auto gravity_compensate_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request>
                   request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            -> void
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(this->get_logger(), "Gravity Compensation turned on");
                    this->wam_.gravityCompensate();
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Gravity Compensation turned off");
                    this->wam_.gravityCompensate(false);
                    response->success = true;
                }
            };

        auto idle_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void
            {
                (void)request_header;
                (void)request;
                RCLCPP_INFO(this->get_logger(), "Idling WAM");
                this->wam_.idle();
                response->success = true;
            };

        auto hold_cart_position_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Holding WAM Tool Position");
                    this->wam_.moveTo(wam_.getToolPosition(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Releasing WAM Tool Position hold");
                    this->wam_.idle();
                    response->success = true;
                }
            };

        auto hold_joint_positioin_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Holding WAM Joint Positions");
                    this->wam_.moveTo(wam_.getJointPositions(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Releasing WAM Joint Positions hold");
                    this->wam_.idle();
                    response->success = true;
                }
            };

        auto hold_cart_orientation_ =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Holding WAM Tool Orientation");
                    this->wam_.moveTo(wam_.getToolOrientation(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Releasing WAM Tool Orientation hold");
                    this->wam_.idle();
                    response->success = true;
                }
            };

        auto hold_cart_pose_ =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            -> void
            {
                (void)request_header;
                if (request->data == true)
                {
                    RCLCPP_INFO(this->get_logger(), "Holding WAM Tool Pose");
                    this->wam_.moveTo(wam_.getToolPose(), false);
                    response->success = true;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Releasing WAM Tool Pose hold");
                    this->wam_.idle();
                    response->success = true;
                }
            };
        auto joint_move_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<wam_msgs::srv::JointMove::Request> request,
                   std::shared_ptr<wam_msgs::srv::JointMove::Response> response) -> void
            {
                (void)request_header;
                jp_type jp;
                if (DOF != request->joint_state.position.size())
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Invalid Command. Please enter %lu Joint Positions", DOF);
                    response->response = false; //notify client that move not initialized
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Moving WAM to commanded Joint Positions");
                    for (int i = 0; i < (int)DOF; i++)
                    {
                        jp(i) = request->joint_state.position[i];
                    }
                    this->wam_.moveTo(jp);
                    response->response = true;
                }
            };
        auto cart_position_move_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<wam_msgs::srv::CartPositionMove::Request> request,
                   std::shared_ptr<wam_msgs::srv::CartPositionMove::Response> response)
            -> void
            {
                (void)request_header;
                cp_type cp;
                RCLCPP_INFO(this->get_logger(),
                            "Moving WAM to commanded Cartesian Positions");
                cp << request->position.x, request->position.y, request->position.z;
                this->wam_.moveTo(cp, false);
                response->response = true;
            };

        auto cart_orientation_move_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<wam_msgs::srv::CartOrientationMove::Request>
                   request,
                   std::shared_ptr<wam_msgs::srv::CartOrientationMove::Response> response)
            -> void
            {
                (void)request_header;
                Eigen::Quaterniond goal_ortn;
                RCLCPP_INFO(this->get_logger(),
                            "Moving WAM to commanded Cartesian Orientation");
                goal_ortn.x() = request->orientation.x;
                goal_ortn.y() = request->orientation.y;
                goal_ortn.z() = request->orientation.z;
                goal_ortn.w() = request->orientation.w;
                this->wam_.moveTo(goal_ortn, false);
                response->response = true;
            };

        auto cart_pose_move_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<wam_msgs::srv::CartPoseMove::Request> request,
                   std::shared_ptr<wam_msgs::srv::CartPoseMove::Response> response)
            -> void
            {
                (void)request_header;
                pose_type pose;
                RCLCPP_INFO(this->get_logger(),
                            "Moving WAM to commanded Cartesian Pose");
                pose.get<0>() << request->pose.position.x, request->pose.position.y, request->pose.position.z;
                response->response = true;
                pose.get<1>().x() = request->pose.orientation.x;
                pose.get<1>().y() = request->pose.orientation.y;
                pose.get<1>().z() = request->pose.orientation.z;
                pose.get<1>().w() = request->pose.orientation.w;
                this->wam_.moveTo(pose, false);
                response->response = true;
            };

        auto set_velocity_limit_cb =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<wam_msgs::srv::VelocityLimit::Request> request,
                   std::shared_ptr<wam_msgs::srv::VelocityLimit::Response> response)
            -> void
            {
                (void)request_header;
                RCLCPP_INFO(this->get_logger(),
                            "Setting WAM Velocity Limit");
                this->pm_.getSafetyModule()->setVelocityLimit(request->velocity_limit);
                (void)response;
            };

        move_to_home_srv_
            = create_service<std_srvs::srv::Trigger>("/wam/moveHome",
                                                   move_home_cb);
        gravity_comp_srv_
            = create_service<std_srvs::srv::SetBool>(
                "/wam/gravityCompensate", gravity_compensate_cb);
        idle_srv_
            = create_service<std_srvs::srv::Trigger>("/wam/idle", idle_cb);
        hold_cart_position_srv_
            = create_service<std_srvs::srv::SetBool>(
                "/wam/holdCartPosition", hold_cart_position_cb);
        hold_joint_position_srv_
            = create_service<std_srvs::srv::SetBool>(
                "/wam/holdJointPosition", hold_joint_positioin_cb);
        hold_cart_orientation_srv_
            = create_service<std_srvs::srv::SetBool>(
                "/wam/holdCartOrientation", hold_cart_orientation_);
        hold_cart_pose_srv_
            = create_service<std_srvs::srv::SetBool>(
                "/wam/holdCartPose", hold_cart_pose_);
        joint_move_srv_
            = create_service<wam_msgs::srv::JointMove>(
                "/wam/moveToJointPosition", joint_move_cb);
        cart_position_move_srv_
            = create_service<wam_msgs::srv::CartPositionMove>(
                "/wam/moveToCartPosition", cart_position_move_cb);
        cart_orientation_move_srv_
            = create_service<wam_msgs::srv::CartOrientationMove>(
                "/wam/moveToCartOrientation", cart_orientation_move_cb);
        cart_pose_move_srv_
            = create_service<wam_msgs::srv::CartPoseMove>(
                "/wam/moveToCartPose", cart_pose_move_cb);
        set_velocity_limit_srv_
            = create_service<wam_msgs::srv::VelocityLimit>(
                "/wam/setVelocityLimit", set_velocity_limit_cb);
    }

  protected:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_home_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    gravity_comp_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr idle_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    hold_joint_position_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    hold_cart_position_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    hold_cart_orientation_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr hold_cart_pose_srv_;
    rclcpp::Service<wam_msgs::srv::JointMove>::SharedPtr joint_move_srv_;
    rclcpp::Service<wam_msgs::srv::CartPositionMove>::SharedPtr
    cart_position_move_srv_;
    rclcpp::Service<wam_msgs::srv::CartOrientationMove>::SharedPtr
    cart_orientation_move_srv_;
    rclcpp::Service<wam_msgs::srv::CartPoseMove>::SharedPtr cart_pose_move_srv_;
    rclcpp::Service<wam_msgs::srv::VelocityLimit>::SharedPtr
    set_velocity_limit_srv_;
    systems::Wam<DOF> &wam_;
    ProductManager &pm_;
    Hand *hand_;
    bool found_hand_;
};
#endif // WAM_NODE_SRC_WAM_SERVICES_H_
