#include <barrett/products/product_manager.h>
#include <barrett/exception.h>

// Custom Headers
#include "custom_systems.h"
#include "wam_publishers.h"
#include "wam_services.h"
#include "wam_subscribers.h"
#include "bhand_publishers.h"
#include "bhand_services.h"
#include "fts_node.h"

namespace wam_node
{
// Wait until desired mode reached
static void
customWaitForMode(enum barrett::SafetyModule::SafetyMode mode,
                  barrett::SafetyModule* sm,
                  std::shared_ptr<rclcpp::Node> node,
                  double pollingPeriod_s = 0.25)
{
    if (sm->getMode() == mode)
        return;

    if (sm->getMode() == barrett::SafetyModule::ESTOP)
        RCLCPP_INFO(node->get_logger(), "WAM is currently E-Stopped");

    RCLCPP_INFO(node->get_logger(), "Please %s the WAM.\n",
                barrett::SafetyModule::getSafetyModeStr(mode));

    do
    {
        barrett::btsleep(pollingPeriod_s);
    } while (sm->getMode() != mode);
}

// Check for WAM, and prompt on zeroing. Return false if wam not present
static bool
customWaitForWAM(barrett::ProductManager& pm,
                 std::shared_ptr<rclcpp::Node> node)
{
    if (!pm.foundSafetyModule())
        return false;

    const auto  sm = pm.getSafetyModule();
    customWaitForMode(barrett::SafetyModule::IDLE, sm, node);
    if (!pm.foundWam())
    {
        pm.enumerate();
        if (!pm.foundWam())
            return false;
    }
    if (!sm->wamIsZeroed())
    {
        RCLCPP_INFO(node->get_logger(),
                    "The WAM needs to be zeroed. Please move it to its "
                    "home position, then press [Enter].");
        barrett::detail::waitForEnter();
    }
    return true;
}

template <size_t DOF> static void
updateRTThreadCb(std::shared_ptr<WamSubscribers<DOF>> node, double frequency)
{
    rclcpp::Rate loop_rate(frequency);
    while (rclcpp::ok())
    {
        node->updateRT();
        loop_rate.sleep();
    }
}

template <size_t DOF> static int
wam_main(barrett::ProductManager& pm, barrett::systems::Wam<DOF>* wam,
         barrett::Hand* hand, barrett::ForceTorqueSensor* fts)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    rclcpp::Rate loop_rate(kPublishFrequency);

    if (wam)
    {
        const auto   sm = pm.getSafetyModule();

        wam->gravityCompensate(true);
        sm->setVelocityLimit(2);
        sm->setTorqueLimit(3.0);

        if (hand)        //found wam and hand.
        {
          // move j3 to allow room for BarrettHand initialization
            auto    jp_init = wam->getJointPositions();
            jp_init[3] -= 0.35;
            barrett::btsleep(0.5);
            wam->moveTo(jp_init);
            barrett::btsleep(0.5);

            hand->initialize();
            hand->update();

          //WAMPublisher publishes hand and WAM joint states.
            auto publish_node = std::make_shared<WamPublishers<DOF>>(wam,
                                                                     hand);
            auto service_node = std::make_shared<WamServices<DOF>>(wam, pm,
                                                                   hand);
            auto bhand_publisher_node = std::make_shared<BhandPublishers>(hand,
                                                                          wam);
            auto subscriber_node = std::make_shared<WamSubscribers<DOF>>(wam,
                                                                         pm);
            auto bhand_services_node = std::make_shared<BhandServices>(hand);
            rclcpp::executors::MultiThreadedExecutor executor;
            std::shared_ptr<FtsNode> fts_node;
            if (fts)
            {
                fts_node = std::make_shared<FtsNode>(fts);
                executor.add_node(fts_node);
            }
            executor.add_node(service_node);
            executor.add_node(subscriber_node);
            executor.add_node(bhand_services_node);
            executor.add_node(bhand_publisher_node);
            std::thread update_rt_thread(updateRTThreadCb<DOF>,
                                         subscriber_node, kPublishFrequency);
            update_rt_thread.detach();
            rclcpp::Rate loop_rate(kPublishFrequency);
            while (rclcpp::ok())
            {
                executor.spin_some();
                publish_node->publishJointState();
                publish_node->publishCartPose();
                publish_node->publishToolVelocity();
                loop_rate.sleep();
            }
            customWaitForMode(barrett::SafetyModule::IDLE, sm, publish_node);
            rclcpp::shutdown();
        }
        else    //found only wam, no hand
        {
            auto publish_node = std::make_shared<WamPublishers<DOF>>(wam,
                                                                     hand);
            auto service_node = std::make_shared<WamServices<DOF>>(wam, pm,
                                                                   hand);
            auto subscriber_node = std::make_shared<WamSubscribers<DOF>>(wam,
                                                                         pm);
            rclcpp::executors::MultiThreadedExecutor executor;
            std::shared_ptr<FtsNode> fts_node;
            if (fts)
            {
                fts_node = std::make_shared<FtsNode>(fts);
                executor.add_node(fts_node);
            }
            executor.add_node(service_node);
            executor.add_node(subscriber_node);
            std::thread update_rt_thread(updateRTThreadCb<DOF>,
                                         subscriber_node, kPublishFrequency);
            update_rt_thread.detach();
            rclcpp::Rate loop_rate(kPublishFrequency);
            while (rclcpp::ok())
            {
                executor.spin_some();
                publish_node->publishJointState();
                publish_node->publishCartPose();
                publish_node->publishToolVelocity();
                loop_rate.sleep();
            }
            customWaitForMode(barrett::SafetyModule::IDLE, sm, publish_node);
            rclcpp::shutdown();
        }
    }
    else if (hand)      //found only hand. Launch a BhandPublish node
    {
        hand->initialize();
        hand->update();
        auto bhand_services_node = std::make_shared<BhandServices>(hand);
        auto bhand_publisher_node = std::make_shared<BhandPublishers>(hand,
                                                                      wam);
        rclcpp::executors::MultiThreadedExecutor executor;
        std::shared_ptr<FtsNode> fts_node;
        if (fts)
        {
            fts_node = std::make_shared<FtsNode>(fts);
            executor.add_node(fts_node);
        }
        executor.add_node(bhand_services_node);
        executor.add_node(bhand_publisher_node);
        executor.spin();
        rclcpp::shutdown();
    }
    else if (fts)       //no hand or WAM, only FTS found
    {
        auto fts_node = std::make_shared<FtsNode>(fts);
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(fts_node);
        executor.spin();
        rclcpp::shutdown();
    }

    return 0;
}
}       // namespace wam_node

int
main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::cerr << "### OK0" << std::endl;
    barrett::ProductManager pm;
    std::cerr << "### OK1" << std::endl;
    barrett::installExceptionHandler();
    std::cerr << "### OK2" << std::endl;
    auto node = rclcpp::Node::make_shared("wam_node");
    std::cerr << "### OK3" << std::endl;
    const auto  hand = (pm.foundHand() ? pm.getHand() : nullptr);
    const auto  fts  = (pm.foundForceTorqueSensor() ?
                        pm.getForceTorqueSensor() : nullptr);

    if (hand)
        RCLCPP_INFO(node->get_logger(), "Found Barrett Hand");

    if (fts)
        RCLCPP_INFO(node->get_logger(), "Found FTS");

  //Check if WAM present
    if (wam_node::customWaitForWAM(pm, node))
    {
        pm.wakeAllPucks();
        if (pm.foundWam3())
        {
            RCLCPP_INFO(node->get_logger(), "Found 3DOF WAM");
            return wam_node::wam_main<3>(pm,
                                         pm.getWam3(true, NULL), hand, fts);
        }
        else if (pm.foundWam4())
        {
            RCLCPP_INFO(node->get_logger(), "Found 4DOF WAM");
            return wam_node::wam_main<4>(pm,
                                         pm.getWam4(true, NULL), hand, fts);
        }
        else if (pm.foundWam7())
        {
            RCLCPP_INFO(node->get_logger(), "Found 7DOF WAM");
            return wam_node::wam_main<7>(pm,
                                         pm.getWam7(true, NULL), hand, fts);
        }
        else
        {
            RCLCPP_FATAL(node->get_logger(),
                         "No WAM was found. Perhaps you have found a bug in ProductManager::waitForWam().");
        }
    }
    else if (hand)       // no wam found, only hand
    {
        rclcpp::spin_some(node);
        return wam_node::wam_main<3>(pm, nullptr, hand, fts);
    }
    else if (fts)       //no wam, no Bhand, only FTS
    {
        return wam_node::wam_main<3>(pm, nullptr, hand, fts);
    }
    else                // no hand or wam found
    {
        RCLCPP_ERROR(node->get_logger(),
                     "ERROR: No WAM or BarrettHand found.");
    }

    return 0;
}
