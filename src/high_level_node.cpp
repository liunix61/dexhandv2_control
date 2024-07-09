/// @file high_level_node.cpp - Dexhand ROS2 Control Node for Servo Abstraction Layer and Approximate Joint Angle Control
/// @copyright 2024 IoT Design Shop Inc. All rights reserved.

#include <string>
#include "base_node.hpp"
#include "dexhand_servomgr.hpp"

using namespace dexhand_connect;
using namespace std;

class HighLevelControlNode : public DexHandBase {
    public:
        HighLevelControlNode() : DexHandBase("dexhandv2_hlc") {
        }

        ~HighLevelControlNode() override {}

    private:

        /// @brief Container class for holding instances of DexHand devices discovered by the node. HLC includes a ServoManager and 
        /// (optionally) a JointAngleController object for each instance
        class HLCHandInstance : public HandInstance {

        public:
            const static int SM_TX_RATE = 100; // 100hz
            const static int SM_RX_RATE = 50; // 50hz
            
            HLCHandInstance(string deviceID, rclcpp::Node* parent) : HandInstance(deviceID, parent), servoMgr(getHand()) {
                servoMgr.start(SM_TX_RATE, SM_RX_RATE);
            }

            ~HLCHandInstance() {
                servoMgr.stop();
            }

        private:

            ServoManager servoMgr; 
    };

};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HighLevelControlNode>());
    rclcpp::shutdown();
    return 0;
}