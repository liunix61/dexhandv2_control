
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "dexhand_connect.hpp"
#include "dexhandv2_control/msg/discovered_hands.hpp"
#include "dexhandv2_control/srv/reset.hpp"

#include "servo_vars_subscriber.hpp"
#include "firmware_version_subscriber.hpp"


/// @brief Base class for DexHand nodes
class DexHandBase : public rclcpp::Node
{
    public:

        /// @brief Constructor
        /// @param node_name Name of the node
        DexHandBase(const std::string& node_name);

        /// @brief Destructor
        virtual ~DexHandBase();

    protected:

        
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

        inline std::vector<std::shared_ptr<HandInstance>>& getHands() { return hands; }

    private:

        void reset_callback(const std::shared_ptr<dexhandv2_control::srv::Reset::Request> request,
                        std::shared_ptr<dexhandv2_control::srv::Reset::Response> response);


        void enumerate_devices();
        void close_devices();

        std::vector<std::shared_ptr<HandInstance>> hands;
        

        rclcpp::Publisher<dexhandv2_control::msg::DiscoveredHands>::SharedPtr dh_publisher;

        rclcpp::Service<dexhandv2_control::srv::Reset>::SharedPtr reset_service;


};
