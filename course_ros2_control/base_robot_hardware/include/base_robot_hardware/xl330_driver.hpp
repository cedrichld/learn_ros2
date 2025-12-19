#ifndef XL330_DRIVER_HPP
#define XL330_DRIVER_HPP

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 57600

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

#define RAD_TO_DXL_POSITION 651.088636364 // 1 / 0.0174533 / 0.088
#define RAD_S_TO_RPM 9.549 // 60 / (2*PI)
#define RPM_TO_DXL_VELOCITY 4.366812227 // 1 / 0.229

#define OPERATING_MODE_VELOCITY 1
#define OPERATING_MODE_POSITION 3

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <iostream>

class XL330Driver {
    public:
        XL330Driver(std::string device_name){
            portHandler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());
            packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        }
    
        int init() {
            std::cout << "Initializing connection with robot." << std::endl;
    
            // Open port
            if (portHandler_->openPort()) {
                std::cout << "Succeeded to open the port!" << std::endl;
            }
            else {
                std::cout << "Failed to open the port!" << std::endl;
                return -1;
            }
    
            // Set port baudrate
            if (portHandler_->setBaudRate(BAUDRATE)) {
                std::cout << "Succeeded to change the baudrate!" << std::endl;
            }
            else {
                std::cout << "Failed to change the baudrate!" << std::endl;
                return -1;
            } 
            return 0;
        }
    
        void activateWithPositionMode(int dxl_id)
        {
            std::cout << "Activate motor" << std::endl;
    
            // Set Position Control Mode
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION);
            
            // Enable Torque
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1);
        }

        void activateWithVelocityMode(int dxl_id)
        {
            std::cout << "Activate motor" << std::endl;
    
            // Set Velocity Control Mode
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_VELOCITY);
            
            // Enable Torque
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 1);
        }
    
        void deactivate(int dxl_id)
        {
            std::cout << "Deactivate motor" << std::endl;
    
            // Disable Torque
            packetHandler_->write1ByteTxRx(portHandler_, dxl_id, ADDR_TORQUE_ENABLE, 0);
        }
    
        void setTargetPositionRadian(int dxl_id, double command) 
        {
            int dxl_cmd = command * RAD_TO_DXL_POSITION + 2048;
            packetHandler_->write4ByteTxRx(portHandler_, dxl_id, ADDR_GOAL_POSITION, dxl_cmd);
        }
    
        void setTargetVelocityRadianPerSec(int dxl_id, double command)
        {
            int dxl_cmd = command * RAD_S_TO_RPM * RPM_TO_DXL_VELOCITY;
            packetHandler_->write4ByteTxRx(portHandler_, dxl_id, ADDR_GOAL_VELOCITY, dxl_cmd);
        }
    
        double getPositionRadian(int dxl_id) 
        {
            int32_t dxl_present_position = 0;
            packetHandler_->read4ByteTxRx(portHandler_, dxl_id, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position);
            return (double)(dxl_present_position - 2048) / RAD_TO_DXL_POSITION;
        }
    
        double getVelocityRadianPerSec(int dxl_id)
        {
            int32_t dxl_present_velocity = 0;
            packetHandler_->read4ByteTxRx(portHandler_, dxl_id, ADDR_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity);
            double velocity = (double)dxl_present_velocity / RPM_TO_DXL_VELOCITY / RAD_S_TO_RPM ;
            return velocity;
        }
    
    private:
        dynamixel::PortHandler *portHandler_;
        dynamixel::PacketHandler *packetHandler_;
};

#endif