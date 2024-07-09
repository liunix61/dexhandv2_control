/// @file base_node.cpp - Base class for DexHand ROS 2 nodes
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#include "base_node.hpp"

using namespace std::chrono_literals;
using namespace dexhand_connect;
using namespace std;

DexHandBase::DexHandBase(const std::string& node_name) : Node(node_name) {
    // Create service for resetting the hand
    reset_service = this->create_service<dexhandv2_control::srv::Reset>(
        "dexhandv2/reset", 
        std::bind(&DexHandBase::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Create a publisher for discovered hands
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    dh_publisher = this->create_publisher<dexhandv2_control::msg::DiscoveredHands>("dexhandv2/discovered_hands", qos_profile);

}

DexHandBase::~DexHandBase() {
    close_devices();
}

 
void DexHandBase::reset_callback(const std::shared_ptr<dexhandv2_control::srv::Reset::Request> request,
                std::shared_ptr<dexhandv2_control::srv::Reset::Response> response)
{
    response->success = false;

    for (auto& hand : hands) {
        if (request->id == "all" || request->id == hand->getID()) {
            RCLCPP_INFO(this->get_logger(), "Resetting hand device %s", hand->getID().c_str());
            hand->getHand().resetHand();
            response->success = true;
        }
    }
    
}


void DexHandBase::enumerate_devices(HandInstanceFactory factory) {
    DexhandConnect hand;
    vector<DexhandConnect::DexhandUSBDevice> devices = hand.enumerateDevices();
    if (devices.size() > 0){
        RCLCPP_INFO(this->get_logger(), "Found %ld Dexhand devices", (devices.size()));

        // Construct a message to publish the discovered hands
        dexhandv2_control::msg::DiscoveredHands discoHands;
        
        for (auto& device : devices) {
            if (hand.openSerial(device.port)) {
                RCLCPP_INFO(this->get_logger(), "Manufacturer: %s Product: %s Serial: %s", device.  manufacturer.c_str(), device.product.c_str(), device.serial.c_str());
                RCLCPP_INFO(this->get_logger(), "Opened serial port %s for device %s", device.port.c_str(), device.serial.c_str());

                // Create a new hand instance and open the serial port
                auto hi = factory(device.serial, this);
                hands.push_back(hi);
                hi->getHand().openSerial(device.port);
                
                // Add it to the discovered hands message
                dexhandv2_control::msg::HardwareDescription hd;
                hd.manufacturer = device.manufacturer;
                hd.port = device.port;
                hd.product = device.product;
                hd.id = device.serial;
                discoHands.hands.push_back(hd);
                
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s for device %s", device.port.c_str(), device.serial.c_str());
            }
        }

        // Publish the discovered hands message as a latched message
        dh_publisher->publish(discoHands);


    }
    else {
        RCLCPP_ERROR(this->get_logger(), "No Dexhand devices found");
    }
}

void DexHandBase::close_devices() {

    RCLCPP_INFO(this->get_logger(), "Shutting down hand devices.");
    for (auto& hand : hands) {
        hand->getHand().closeSerial();
    }
    hands.clear();
}

