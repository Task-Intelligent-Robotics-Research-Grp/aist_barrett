#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// Standard ROS messages
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
// Custom WAM srvs
#include "wam_msgs/srv/cart_orientation_move.hpp"
#include "wam_msgs/srv/cart_pose_move.hpp"
#include "wam_msgs/srv/cart_position_move.hpp"
#include "wam_msgs/srv/joint_move.hpp"
#include "wam_msgs/srv/velocity_limit.hpp"
// Custom WAM msgs
#include "wam_msgs/msg/rt_angular_velocity.hpp"
#include "wam_msgs/msg/rt_cart_orientation.hpp"
#include "wam_msgs/msg/rt_cart_pose.hpp"
#include "wam_msgs/msg/rt_cart_position.hpp"
#include "wam_msgs/msg/rt_joint_positions.hpp"
#include "wam_msgs/msg/rt_joint_velocities.hpp"
#include "wam_msgs/msg/rt_linear_velocity.hpp"
#include "wam_msgs/msg/rt_linearand_angular_velocity.hpp"
//Custom BarrettHand srvs
#include "bhand_msgs/srv/finger_position.hpp"
#include "bhand_msgs/srv/finger_velocity.hpp"
#include "bhand_msgs/srv/grasp_position.hpp"
#include "bhand_msgs/srv/grasp_velocity.hpp"
#include "bhand_msgs/srv/spread_position.hpp"
#include "bhand_msgs/srv/spread_velocity.hpp"
//Custom BarrettHand msgs
#include "bhand_msgs/msg/finger_tip_torques.hpp"
#include "bhand_msgs/msg/tactile_state.hpp"
#include "bhand_msgs/msg/tactile_state_array.hpp"
// Barrett Headers
#include <barrett/detail/stl_utils.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/systems/wam.h>
#include <barrett/units.h>
// For custom main function, needed for standalone BarrettHand
#include <barrett/exception.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>

#include "tf2/LinearMath/Matrix3x3.h"
// Custom Headers
#include "custom_systems.h"
#include "wam_publishers.h"
#include "wam_services.h"
#include "wam_subscribers.h"
#include "bhand_publishers.h"
#include "bhand_services.h"
#include "fts_node.h"

using namespace barrett;

// Wait until desired mode reached
void customWaitForMode(enum SafetyModule::SafetyMode mode, SafetyModule *sm,
                       std::shared_ptr<rclcpp::Node> node,
                       double pollingPeriod_s = 0.25) {
  if (sm->getMode() == mode) {
    return;
  }
  if (sm->getMode() == SafetyModule::ESTOP) {
    RCLCPP_INFO(node->get_logger(), "WAM is currently E-Stopped");
  }
  RCLCPP_INFO(node->get_logger(), "Please %s the WAM.\n",
              SafetyModule::getSafetyModeStr(mode));
  do {
    btsleep(pollingPeriod_s);
  } while (sm->getMode() != mode);
}

// Check for WAM, and prompt on zeroing. Return false if wam not present
bool customWaitForWAM(ProductManager &pm, std::shared_ptr<rclcpp::Node> node) {
  if (!pm.foundSafetyModule()) {
    return false;
  }
  SafetyModule *sm = pm.getSafetyModule();
  customWaitForMode(SafetyModule::IDLE, sm, node);
  if (!pm.foundWam()) {
    pm.enumerate();
    if (!pm.foundWam()) {
      return false;
    }
  }
  if (!sm->wamIsZeroed()) {
    RCLCPP_INFO(node->get_logger(),
                "The WAM needs to be zeroed. Please move it to its "
                "home position, then press [Enter].");
    detail::waitForEnter();
  }
  return true;
}

template <size_t DOF>
void updateRTThreadCb(std::shared_ptr<WamSubscribers<DOF>> node, double frequency) {
  rclcpp::Rate loop_rate(frequency);
  while (rclcpp::ok()) {
    node->updateRT();
    loop_rate.sleep();
  }
}

template <size_t DOF>
int wam_main(ProductManager &pm, systems::Wam<DOF> *wam,
             bool found_wam, bool found_hand, bool found_fts) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  rclcpp::Rate loop_rate(kPublishFrequency);
  SafetyModule *sm = pm.getSafetyModule();
  if (found_wam) {
    wam->gravityCompensate(true);
    pm.getSafetyModule()->setVelocityLimit(2);
    pm.getSafetyModule()->setTorqueLimit(3.0);
  }
  if (found_wam && found_hand) { //found wam and hand.
    Hand *hand = pm.getHand();
    jp_type jp_init = wam->getJointPositions();
    jp_init[3] -= 0.35;  // move j3 to allow room for BarrettHand initialization
    btsleep(0.5);
    wam->moveTo(jp_init);
    btsleep(0.5);
    hand->initialize();
    hand->update();
    //WAMPublisher publishes hand and WAM joint states.
    auto publish_node = std::make_shared<WamPublishers<DOF>>(*wam, pm, found_hand);
    auto service_node = std::make_shared<WamServices<DOF>>(*wam, pm, pm.getHand(), found_hand);
    auto bhand_publisher_node = std::make_shared<BhandPublishers>(*pm.getHand(), found_wam);
    auto subscriber_node = std::make_shared<WamSubscribers<DOF>>(*wam, pm);
    auto bhand_services_node = std::make_shared<BhandServices>(*pm.getHand());
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<FtsNode> fts_node;
    if (found_fts) {
      fts_node = std::make_shared<FtsNode>(pm.getForceTorqueSensor());
      executor.add_node(fts_node);
    }
    executor.add_node(service_node);
    executor.add_node(subscriber_node);
    executor.add_node(bhand_services_node);
    executor.add_node(bhand_publisher_node);
    std::thread update_rt_thread(updateRTThreadCb<DOF>, subscriber_node, kPublishFrequency);
    update_rt_thread.detach();
    rclcpp::Rate loop_rate(kPublishFrequency);
    while (rclcpp::ok()) {
      executor.spin_some();
      publish_node->publishJointPositions();
      publish_node->publishCartPose();
      publish_node->publishToolVelocity();
      publish_node->publishJointVelocities();
      loop_rate.sleep();
    }
    customWaitForMode(SafetyModule::IDLE, sm, publish_node);
    rclcpp::shutdown();
  } else if (found_wam && !found_hand) { //found only wam, no hand
    auto publish_node = std::make_shared<WamPublishers<DOF>>(*wam, pm, found_hand);
    auto service_node = std::make_shared<WamServices<DOF>>(*wam, pm, pm.getHand(), found_hand);
    auto subscriber_node = std::make_shared<WamSubscribers<DOF>>(*wam, pm);
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<FtsNode> fts_node;
    if (found_fts) {
      fts_node = std::make_shared<FtsNode>(pm.getForceTorqueSensor());
      executor.add_node(fts_node);
    }
    executor.add_node(service_node);
    executor.add_node(subscriber_node);
    std::thread update_rt_thread(updateRTThreadCb<DOF>, subscriber_node, kPublishFrequency);
    update_rt_thread.detach();
    rclcpp::Rate loop_rate(kPublishFrequency);
    while (rclcpp::ok()) {
      executor.spin_some();
      publish_node->publishJointPositions();
      publish_node->publishCartPose();
      publish_node->publishToolVelocity();
      publish_node->publishJointVelocities();
      loop_rate.sleep();
    }
    customWaitForMode(SafetyModule::IDLE, sm, publish_node);
    rclcpp::shutdown();
  } else if (found_hand && !found_wam) { //found only hand. Launch a BhandPublish node
    Hand *hand = pm.getHand();
    hand->initialize();
    hand->update();
    auto bhand_services_node = std::make_shared<BhandServices>(*pm.getHand());
    auto bhand_publisher_node = std::make_shared<BhandPublishers>(*pm.getHand(), found_wam);
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<FtsNode> fts_node;
    if (found_fts) {
      fts_node = std::make_shared<FtsNode>(pm.getForceTorqueSensor());
      executor.add_node(fts_node);
    }
    executor.add_node(bhand_services_node);
    executor.add_node(bhand_publisher_node);
    executor.spin();
    rclcpp::shutdown();
  } else if (!found_hand && !found_wam && found_fts) {
    //no hand or WAM, only FTS found
      auto fts_node = std::make_shared<FtsNode>(pm.getForceTorqueSensor());
      rclcpp::executors::MultiThreadedExecutor executor;
      executor.add_node(fts_node);
      executor.spin();
      rclcpp::shutdown();
  }
  return 0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ProductManager pm;
  ::barrett::installExceptionHandler();
  auto node = rclcpp::Node::make_shared("wam_node");
  bool found_hand = false;
  bool found_fts = false;
  //Check if WAM present
  if (customWaitForWAM(pm, node)) {
    if (pm.foundHand()) {
      found_hand = true;
      RCLCPP_INFO(node->get_logger(), "Found Barrett Hand");
    }
    if (pm.foundForceTorqueSensor()) {
      found_fts = true;
      RCLCPP_INFO(node->get_logger(), "Found FTS");
    }
    pm.wakeAllPucks();
    if (pm.foundWam3()) {
      RCLCPP_INFO(node->get_logger(), "Found 3DOF WAM");
      return wam_main<3>(pm, pm.getWam3(true, NULL), true, found_hand, found_fts);
    } else if (pm.foundWam4()) {
      RCLCPP_INFO(node->get_logger(), "Found 4DOF WAM");
      return wam_main<4>(pm, pm.getWam4(true, NULL), true, found_hand, found_fts);
    } else if (pm.foundWam7()) {
      RCLCPP_INFO(node->get_logger(), "Found 7DOF WAM");
      return wam_main<7>(pm, pm.getWam7(true, NULL), true, found_hand, found_fts);
    } else {
      RCLCPP_FATAL(node->get_logger(), "No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam().");
    }
  } else if (pm.foundHand()) {
    // no wam found, only hand
    found_fts = pm.foundForceTorqueSensor();
    RCLCPP_INFO(node->get_logger(), "Found BarrettHand");
    rclcpp::spin_some(node);
    return wam_main<3>(pm, NULL, false, true, found_fts);
  } else if (pm.foundForceTorqueSensor()) {
    //no wam, no Bhand, only FTS
    RCLCPP_INFO(node->get_logger(), "Found FTS");
    return wam_main<3>(pm, NULL, false, false, true);
  } else {
    // no hand or wam found
      RCLCPP_ERROR(node->get_logger(),
                   "ERROR: No WAM or BarrettHand found.");
  }
  return 0;
}
