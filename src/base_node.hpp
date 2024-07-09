/// @file base_node.hpp - Base class for DexHand ROS 2 nodes
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#pragma once
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "dexhand_connect.hpp"
#include "dexhandv2_control/msg/discovered_hands.hpp"
#include "dexhandv2_control/srv/reset.hpp"

#include "servo_vars_subscriber.hpp"
#include "firmware_version_subscriber.hpp"


/// @brief Container class for holding instances of DexHand devices discovered by the node
class HandInstance {

    public:
        HandInstance(std::string deviceID, rclcpp::Node* parent) : hand(), id(deviceID), servoVars(deviceID, parent), firmware(deviceID,parent) {
            hand.subscribe(&servoVars);
            hand.subscribe(&firmware);
        }

        virtual ~HandInstance() {
            hand.unsubscribe(&servoVars);
            hand.unsubscribe(&firmware);
        }

        inline dexhand_connect::DexhandConnect& getHand() { return hand; }
        inline std::string getID() { return id; }
        

    private:

        dexhand_connect::DexhandConnect hand;
        std::string id;
        ServoVarsSubscriber servoVars;
        FirmwareVersionSubscriber firmware;
};
using HandInstanceFactory = std::function<std::shared_ptr<HandInstance>(std::string, rclcpp::Node*)>;
        

/// @brief Base class for DexHand ROS 2 nodes. This class provides common functions such as enumerating and connecting to DexHand devices,
/// as well as providing common services and publishers that would be common to all DexHand nodes.
class DexHandBase : public rclcpp::Node
{
    public:

        /// @brief Constructor
        /// @param node_name Name of the node
        DexHandBase(const std::string& node_name);

        /// @brief Destructor
        virtual ~DexHandBase();

    protected:
        inline std::vector<std::shared_ptr<HandInstance>>& getHands() { return hands; }

        /// @brief This method must be called from a derived class at initialization time
        /// in order to populate the hands vector with instances of DexHand devices. 
        /// @note The factory parameter allows the derived class to pass through it's own
        /// container objects.
        void enumerate_devices(HandInstanceFactory factory);
        
    private:

        void reset_callback(const std::shared_ptr<dexhandv2_control::srv::Reset::Request> request,
                        std::shared_ptr<dexhandv2_control::srv::Reset::Response> response);


        void close_devices();

        std::vector<std::shared_ptr<HandInstance>> hands;
        
        rclcpp::Publisher<dexhandv2_control::msg::DiscoveredHands>::SharedPtr dh_publisher;
        rclcpp::Service<dexhandv2_control::srv::Reset>::SharedPtr reset_service;


};
