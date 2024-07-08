#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dexhandv2_control/msg/firmware_version.hpp"
#include "dexhandv2_control/msg/discovered_hands.hpp"
#include "dexhandv2_control/msg/hardware_description.hpp"
#include "dexhandv2_control/msg/servo_vars.hpp"
#include "dexhandv2_control/msg/servo_vars_table.hpp"
#include "dexhandv2_control/msg/servo_dynamics.hpp"
#include "dexhandv2_control/msg/servo_dynamics_table.hpp"
#include "dexhandv2_control/msg/servo_status.hpp"
#include "dexhandv2_control/msg/servo_target.hpp"
#include "dexhandv2_control/msg/servo_targets_table.hpp"

#include "dexhandv2_control/srv/reset.hpp"



#include "dexhand_connect.hpp"

using namespace std::chrono_literals;


using namespace dexhand_connect;
using namespace std;



class FullServoStatusSubscriber : public IDexhandMessageSubscriber<ServoFullStatusMessage> {
    public:

        FullServoStatusSubscriber(string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()) {
            // Define QoS policy - best effort, keep last message, volatile
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            qos.best_effort();

            ss_publisher = parent->create_publisher<dexhandv2_control::msg::ServoStatus>("dexhandv2/servo_status", qos);
        }
        ~FullServoStatusSubscriber() = default;

        void messageReceived(const ServoFullStatusMessage& message) override {
            RCLCPP_DEBUG(logger, "Full status message received for device: %s", deviceID.c_str());
            RCLCPP_DEBUG(logger, "Servo ID: %d", (int)message.getServoID());
            RCLCPP_DEBUG(logger, "Status: %d", message.getStatus());
            RCLCPP_DEBUG(logger, "Position: %d", message.getPosition());
            RCLCPP_DEBUG(logger, "Speed: %d", message.getSpeed());
            RCLCPP_DEBUG(logger, "Load: %d", message.getLoad());
            RCLCPP_DEBUG(logger, "Voltage: %d", (int)message.getVoltage());
            RCLCPP_DEBUG(logger, "Temperature: %d", (int)message.getTemperature());

            dexhandv2_control::msg::ServoStatus ss_msg;
            ss_msg.id = deviceID;
            ss_msg.servo_id = message.getServoID();
            ss_msg.status = message.getStatus();
            ss_msg.position = message.getPosition();
            ss_msg.speed = message.getSpeed();
            ss_msg.load = message.getLoad();
            ss_msg.voltage = message.getVoltage();
            ss_msg.temp = message.getTemperature();
            ss_publisher->publish(ss_msg);
        }

    private:
        string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::ServoStatus>::SharedPtr ss_publisher;
};

class DynamicsSubscriber : public IDexhandMessageSubscriber<ServoDynamicsMessage> {
    public:
        DynamicsSubscriber(string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()){
            // Define QoS policy - best effort, keep last message, volatile
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            qos.best_effort();

            sd_publisher = parent->create_publisher<dexhandv2_control::msg::ServoDynamicsTable>("dexhandv2/servo_dynamics", qos);
        }
        ~DynamicsSubscriber() = default;

        void messageReceived(const ServoDynamicsMessage& message) override {
            RCLCPP_DEBUG(logger, "Dynamics message received for device: %s", deviceID.c_str());
            RCLCPP_DEBUG(logger, "Num servos: %ld", message.getNumServos());
            RCLCPP_DEBUG(logger, "ID\tStatus\tPos\tSpd\tLoad");
            RCLCPP_DEBUG(logger, "------------------------------------");
            

            dexhandv2_control::msg::ServoDynamicsTable sd_msg;
            sd_msg.id = deviceID;

            for (const auto& status : message.getServoStatus()){
                RCLCPP_DEBUG(logger, "%d\t%d\t%d\t%d\t%d", (int)status.first, status.second.getStatus(), status.second.getPosition(), status.second.getSpeed(), status.second.getLoad());
                
                dexhandv2_control::msg::ServoDynamics sd;
                sd.servo_id = status.first;
                sd.status = status.second.getStatus();
                sd.position = status.second.getPosition();
                sd.speed = status.second.getSpeed();
                sd.load = status.second.getLoad();
                sd_msg.servo_table.push_back(sd);
            }
            
            sd_publisher->publish(sd_msg);
        }
    private:
        string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::ServoDynamicsTable>::SharedPtr sd_publisher;
};

class ServoVarsSubscriber : public IDexhandMessageSubscriber<ServoVarsListMessage> {
    public:
        ServoVarsSubscriber(string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()) {
            sv_publisher = parent->create_publisher<dexhandv2_control::msg::ServoVarsTable>("dexhandv2/servo_vars", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        }
        ~ServoVarsSubscriber() = default;

        void messageReceived(const ServoVarsListMessage& message) override {
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
        string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::ServoVarsTable>::SharedPtr sv_publisher;
};

class FirmwareVersionSubscriber : public IDexhandMessageSubscriber<FirmwareVersionMessage> {
    public:
        FirmwareVersionSubscriber(string deviceID, rclcpp::Node* parent) : deviceID(deviceID), logger(parent->get_logger()) {
            fw_publisher = parent->create_publisher<dexhandv2_control::msg::FirmwareVersion>("dexhandv2/firmware_version", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        }
        ~FirmwareVersionSubscriber() = default;

        void messageReceived(const FirmwareVersionMessage& message) override {
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
        string deviceID;
        rclcpp::Logger logger;
        rclcpp::Publisher<dexhandv2_control::msg::FirmwareVersion>::SharedPtr fw_publisher;
};




/// @brief Main class for the node to poll hand events and publish to ROS2 messages
class DexHandNode : public rclcpp::Node
{
  public:
    DexHandNode(): Node("dexhandv2_control")
    {
        // Create service for resetting the hand
        reset_service = this->create_service<dexhandv2_control::srv::Reset>(
            "dexhandv2/reset", 
            std::bind(&DexHandNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Create a publisher for discovered hands
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        dh_publisher = this->create_publisher<dexhandv2_control::msg::DiscoveredHands>("dexhandv2/discovered_hands", qos_profile);

        // Create subscriber for servo position messages
        st_subscriber = this->create_subscription<dexhandv2_control::msg::ServoTargetsTable>(
            "dexhandv2/servo_targets", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            std::bind(&DexHandNode::servo_targets_callback, this, std::placeholders::_1));
    

        // Set up the Dexhand devices
        enumerate_devices();

        // Schedule timer for updating hand devices
        timer_ = this->create_wall_timer(10ms, std::bind(&DexHandNode::timer_callback, this));
    }

    ~DexHandNode() {
        close_devices();
    }

  private:

    void reset_callback(const std::shared_ptr<dexhandv2_control::srv::Reset::Request> request,
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

    void servo_targets_callback(const dexhandv2_control::msg::ServoTargetsTable::SharedPtr msg) {
        for (auto& hand : hands) {
            if (msg->id == hand->getID()) {
                SetServoPositionsCommand cmd;
                for (const auto& target : msg->servo_table) {
                    RCLCPP_DEBUG(this->get_logger(), "Setting servo %d to position %d", target.servo_id, target.position);
                    cmd.setServoPosition(target.servo_id, target.position);
                }
                hand->getHand().sendCommand(cmd);
            }
        }
    }

    class HandInstance {

        public:
            HandInstance(string deviceID, rclcpp::Node* parent) : hand(), id(deviceID), fullStatus(deviceID, parent), dynamics(deviceID, parent), servoVars(deviceID, parent), firmware(deviceID,parent) {
                hand.subscribe(&fullStatus);
                hand.subscribe(&dynamics);
                hand.subscribe(&servoVars);
                hand.subscribe(&firmware);
            }

            ~HandInstance() {
                hand.unsubscribe(&fullStatus);
                hand.unsubscribe(&dynamics);
                hand.unsubscribe(&servoVars);
                hand.unsubscribe(&firmware);
            }

            inline DexhandConnect& getHand() { return hand; }
            inline string getID() { return id; }

        private:

            DexhandConnect hand;
            string id;
            FullServoStatusSubscriber fullStatus;
            DynamicsSubscriber dynamics;
            ServoVarsSubscriber servoVars;
            FirmwareVersionSubscriber firmware;
    };


    void enumerate_devices() {
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
                    hands.push_back(std::make_shared<HandInstance>(device.serial, this));
                    HandInstance* hi = hands.back().get();
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

    void close_devices() {

        RCLCPP_INFO(this->get_logger(), "Shutting down hand devices.");
        for (auto& hand : hands) {
            hand->getHand().closeSerial();
        }
        hands.clear();
    }


    void timer_callback()
    {
        // Update all the hands
        for (auto& hand : hands) {
            hand->getHand().update();
        }
    }

    std::vector<shared_ptr<HandInstance>> hands;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<dexhandv2_control::msg::DiscoveredHands>::SharedPtr dh_publisher;
    rclcpp::Subscription<dexhandv2_control::msg::ServoTargetsTable>::SharedPtr st_subscriber;

    rclcpp::Service<dexhandv2_control::srv::Reset>::SharedPtr reset_service;

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DexHandNode>());
  rclcpp::shutdown();
  return 0;
}