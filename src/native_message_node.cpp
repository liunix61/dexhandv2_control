#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "dexhandv2_control/msg/discovered_hands.hpp"
#include "dexhandv2_control/msg/hardware_description.hpp"
#include "dexhandv2_control/msg/servo_dynamics.hpp"
#include "dexhandv2_control/msg/servo_dynamics_table.hpp"
#include "dexhandv2_control/msg/servo_status.hpp"
#include "dexhandv2_control/msg/servo_target.hpp"
#include "dexhandv2_control/msg/servo_targets_table.hpp"

#include "dexhand_connect.hpp"

#include "base_node.hpp"

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







/// @brief Main class for the node to poll hand events and publish to ROS2 messages
class DexHandNode : public DexHandBase
{
  public:
    DexHandNode(): DexHandBase("dexhandv2_control")
    {
        // Create subscriber for servo position messages
        st_subscriber = this->create_subscription<dexhandv2_control::msg::ServoTargetsTable>(
            "dexhandv2/servo_targets", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            std::bind(&DexHandNode::servo_targets_callback, this, std::placeholders::_1));

        // Schedule timer for updating hand devices
        timer_ = this->create_wall_timer(10ms, std::bind(&DexHandNode::timer_callback, this));
    }

    ~DexHandNode() override {
        
    }

  private:

    
    void servo_targets_callback(const dexhandv2_control::msg::ServoTargetsTable::SharedPtr msg) {
        for (auto& hand : getHands()) {
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

    class NMHandInstance : public HandInstance {

        public:
            NMHandInstance(string deviceID, rclcpp::Node* parent) : HandInstance(deviceID, parent), fullStatus(deviceID, parent), dynamics(deviceID, parent) {
                getHand().subscribe(&fullStatus);
                getHand().subscribe(&dynamics);
            }

            ~NMHandInstance() {
                getHand().unsubscribe(&fullStatus);
                getHand().unsubscribe(&dynamics);
            }

        private:

            FullServoStatusSubscriber fullStatus;
            DynamicsSubscriber dynamics;
            
    };

    void timer_callback()
    {
        // Update all the hands
        for (auto& hand : getHands()) {
            hand->getHand().update();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<dexhandv2_control::msg::ServoTargetsTable>::SharedPtr st_subscriber;
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DexHandNode>());
  rclcpp::shutdown();
  return 0;
}