/// @file firmware_version_subscriber.hpp - Subscriber for DexHand firmware version messages
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dexhand_connect.hpp"
#include "dexhandv2_control/msg/firmware_version.hpp"


/// @brief Subscriber for DexHand firmware version messages. Receives firmware version messages from DexHand and publishes them as ROS 2 messages.
class FirmwareVersionSubscriber : public dexhand_connect::IDexhandMessageSubscriber<dexhand_connect::FirmwareVersionMessage> {
    public:
        FirmwareVersionSubscriber(std::string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()) {
            fw_publisher = parent->create_publisher<dexhandv2_control::msg::FirmwareVersion>("dexhandv2/firmware_version", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        }
        ~FirmwareVersionSubscriber() = default;

        void messageReceived(const dexhand_connect::FirmwareVersionMessage& message) override {
            RCLCPP_INFO(logger, "Firmware version received for device: %s", deviceID.c_str());
            RCLCPP_INFO(logger, "Firmware version name: %s", message.getVersionName().c_str());
            RCLCPP_INFO(logger, "Firmware version: %d.%d", (int)message.getMajorVersion(), (int)message.getMinorVersion());

            dexhandv2_control::msg::FirmwareVersion fw_msg;
            fw_msg.id = deviceID;
            fw_msg.version_name = message.getVersionName();
            fw_msg.major = message.getMajorVersion();
            fw_msg.minor = message.getMinorVersion();
            fw_publisher->publish(fw_msg);
        }

    private:
        std::string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::FirmwareVersion>::SharedPtr fw_publisher;
};