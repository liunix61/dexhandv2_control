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

#include "dexhand_connect.hpp"

using namespace std::chrono_literals;


using namespace dexhand_connect;
using namespace std;



class FullServoStatusSubscriber : public IDexhandMessageSubscriber<ServoFullStatusMessage> {
    public:

        FullServoStatusSubscriber(string deviceID) : deviceID(deviceID) {}
        ~FullServoStatusSubscriber() = default;

        void messageReceived(const ServoFullStatusMessage& message) override {
            cout << "Full Status for Device:" << deviceID << " Servo ID: " << (int)message.getServoID() << endl;
            cout << "--------------------------------" << endl;
            cout << "Status: " << (int)message.getStatus() << endl;
            cout << "Position: " << message.getPosition() << endl;
            cout << "Speed: " << message.getSpeed() << endl;
            cout << "Load: " << message.getLoad() << endl;
            cout << "Voltage: " << (int)message.getVoltage() << endl;
            cout << "Temperature: " << (int)message.getTemperature() << endl;
            cout << "--------------------------------" << endl << endl;
        }

    private:
        string deviceID;
};

class DynamicsSubscriber : public IDexhandMessageSubscriber<ServoDynamicsMessage> {
    public:
        DynamicsSubscriber(string deviceID) : deviceID(deviceID) {}
        ~DynamicsSubscriber() = default;

        void messageReceived(const ServoDynamicsMessage& message) override {
            cout << "Dynamics message received for device:" << deviceID << endl;
            cout << "Num servos: " << message.getNumServos() << endl;
            cout << "ID:\tStatus\tPos\tSpd\tLoad" << endl;
            cout << "------------------------------------" << endl;

            for (const auto& status : message.getServoStatus()){
                cout << (int)status.first << "\t";
                cout << (int)status.second.getStatus() << "\t";
                cout << status.second.getPosition() << "\t";
                cout << status.second.getSpeed() << "\t";
                cout << status.second.getLoad() << endl;
            }
            cout << "------------------------------------" << endl << endl;
        }
    private:
        string deviceID;
};

class ServoVarsSubscriber : public IDexhandMessageSubscriber<ServoVarsListMessage> {
    public:
        ServoVarsSubscriber(string deviceID) : deviceID(deviceID) {}
        ~ServoVarsSubscriber() = default;

        void messageReceived(const ServoVarsListMessage& message) override {
            cout << "Servo Vars message received for device:" << deviceID << endl;
            cout << "Num servos: " << message.getNumServos() << endl;
            cout << "ID\tHWMin\tHWMax\tSWMin\tSWMax\tHome\tMaxLoad\tMaxTemp" << endl;
            cout << "------------------------------------------------------------------" << endl;


            // Iterate over each servo and print out the vars
            for (const auto& vars : message.getServoVars()){
                cout << (int)vars.first << "\t";
                cout << vars.second.getHWMinPosition() << "\t";
                cout << vars.second.getHWMaxPosition() << "\t";
                cout << vars.second.getSWMinPosition() << "\t";
                cout << vars.second.getSWMaxPosition() << "\t";
                cout << vars.second.getHomePosition() << "\t";
                cout << (int)vars.second.getMaxLoadPct() << "\t";
                cout << (int)vars.second.getMaxTemp() << endl;
            }
            
            cout << "------------------------------------" << endl << endl;
        }
    private:
        string deviceID;
};

class FirmwareVersionSubscriber : public IDexhandMessageSubscriber<FirmwareVersionMessage> {
    public:
        FirmwareVersionSubscriber(string deviceID, rclcpp::Logger log) : deviceID(deviceID), logger(log) {}
        ~FirmwareVersionSubscriber() = default;

        void messageReceived(const FirmwareVersionMessage& message) override {
            RCLCPP_INFO(logger, "Firmware version received for device: %s", deviceID.c_str());
            RCLCPP_INFO(logger, "Firmware: %s", message.getVersionName().c_str());
            RCLCPP_INFO(logger, "Firmware version: %d.%d", (int)message.getMajorVersion(), (int)message.getMinorVersion());
        }

    private:
        string deviceID;
        rclcpp::Logger logger;
};




/// @brief Main class for the node to poll hand events and publish to ROS2 messages
class DexHandPublisher : public rclcpp::Node
{
  public:
    DexHandPublisher()
    : Node("DexHand_publisher"), count_(0)
    {
        // Set up the Dexhand devices
        enumerate_devices();
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&DexHandPublisher::timer_callback, this));
    }

    ~DexHandPublisher() {
        close_devices();
    }

  private:

    class HandInstance {

        public:
            HandInstance(string deviceID, rclcpp::Node* parent) : hand(), fullStatus(deviceID), dynamics(deviceID), servoVars(deviceID), firmware(deviceID,parent->get_logger()) {
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

        private:

            DexhandConnect hand;
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
            
            for (auto& device : devices) {
                RCLCPP_INFO(this->get_logger(), "Manufacturer: %s Product: %s Serial: %s", device.  manufacturer.c_str(), device.product.c_str(), device.serial.c_str());
            }

            for (auto& device : devices) {
                if (hand.openSerial(device.port)) {
                    RCLCPP_INFO(this->get_logger(), "Opened serial port %s for device %s", device.port.c_str(), device.serial.c_str());

                    // Create a new hand instance and open the serial port
                    hands.push_back(std::make_shared<HandInstance>(device.serial, this));
                    HandInstance* hi = hands.back().get();
                    hi->getHand().openSerial(device.port);
                    hi->getHand().resetHand();
                    
                }
            }
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
        
      //auto message = std_msgs::msg::String();
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //publisher_->publish(message);
    }

    std::vector<shared_ptr<HandInstance>> hands;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DexHandPublisher>());
  rclcpp::shutdown();
  return 0;
}