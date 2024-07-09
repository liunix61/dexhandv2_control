#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dexhand_connect.hpp"

#include "dexhandv2_control/msg/servo_vars.hpp"
#include "dexhandv2_control/msg/servo_vars_table.hpp"


class ServoVarsSubscriber : public dexhand_connect::IDexhandMessageSubscriber<dexhand_connect::ServoVarsListMessage> {
    public:
        ServoVarsSubscriber(std::string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()) {
            sv_publisher = parent->create_publisher<dexhandv2_control::msg::ServoVarsTable>("dexhandv2/servo_vars", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        }
        ~ServoVarsSubscriber() = default;

        void messageReceived(const dexhand_connect::ServoVarsListMessage& message) override {
            RCLCPP_INFO(logger, "Servo Vars message received for device: %s", deviceID.c_str());
            RCLCPP_INFO(logger, "Num servos: %ld", message.getNumServos());
            RCLCPP_INFO(logger, "ID\tHWMin\tHWMax\tSWMin\tSWMax\tHome\tMaxLoad\tMaxTemp");
            RCLCPP_INFO(logger, "------------------------------------------------------------------");
            
            dexhandv2_control::msg::ServoVarsTable sv_msg;
            sv_msg.id = deviceID;

            
            // Iterate over each servo and print out the vars
            for (const auto& vars : message.getServoVars()){
                RCLCPP_INFO(logger, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", (int)vars.first, vars.second.getHWMinPosition(), vars.second.getHWMaxPosition(), vars.second.getSWMinPosition(), vars.second.getSWMaxPosition(), vars.second.getHomePosition(), (int)vars.second.getMaxLoadPct(), (int)vars.second.getMaxTemp());
                
                dexhandv2_control::msg::ServoVars sv;
                sv.servo_id = vars.first;
                sv.hw_min = vars.second.getHWMinPosition();
                sv.hw_max = vars.second.getHWMaxPosition();
                sv.sw_min = vars.second.getSWMinPosition();
                sv.sw_max = vars.second.getSWMaxPosition();
                sv.home = vars.second.getHomePosition();
                sv.max_load_pct = vars.second.getMaxLoadPct();
                sv.max_temp = vars.second.getMaxTemp();
                sv_msg.servo_table.push_back(sv);
            }
            
            sv_publisher->publish(sv_msg);
        }
    private:
        std::string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::ServoVarsTable>::SharedPtr sv_publisher;
};
