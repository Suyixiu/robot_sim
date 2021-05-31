/**
 * @file dh_rgi.h
 * @brief Driver for DH-Robotics RGI Gripper
 * @author Shelling Ding (Renjie.ding@DH-Robotics.com)
 * @version 1.0
 * @date 2020-11-17
 * 
 * @copyright Copyright (c) 2020  Shenzhen DH-Robotics Inc.
 * 
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-11-17 <td>1.0     <td>Shelling Ding     <td>内容
 * </table>
 */
#ifndef __dh_rgi__
#define __dh_rgi__

#include "dh_modbus_gripper.h"

class DH_RGI: public DH_Modbus_Gripper
{
public:
    DH_RGI(int id, std::string Portname, int parameter);
    ~DH_RGI();

    // set gripper target rotation angle -32767~32767 degree
    bool SetTargetRotation(int angle);
    // set gripper target rotation torque  20~100  %
    bool SetTargetRotationTorque(int torque);
    // set gripper target rotation speed  1~100 %
    bool SetTargetRotationSpeed(int speed);

    // get gripper current rotation angle 
    bool GetCurrentRotation(int &curAngle);
    // get gripper target rotation angle 
    bool GetCurrentTargetRotation(int &curAngle);
    // get gripper target rotation torque (Notice: Not actual force)
    bool GetCurrentTargetRotationTorque(int &curTarRotTorque);
    // get gripper target rotation speed (Notice: Not actual speed)
    bool GetCurrentTargetRotationSpeed(int &curTarRotSpeed);

    // get gripper rotation state
    bool GetRotationState(int &r_state);
    // get gripper rotation initialization state
    bool GetRotationInitializationState(int &ri_state);
    //get states: initialization, grip state, current position, target position, target force
    //                          rotation state, current angle, target angle, target torque
    bool GetRunStates(int states[]);

    enum S_ROTATION_STATES
    {
        S_ROT_MOVING = 0,       // Rotating
        S_ROT_ARRIVED = 1,      // Arrived target angle
        S_ROT_BLOCKED = 2,      // Rotation is blocked
        S_ROT_HAD_BLOCKED = 3,  // Had been blocked
    };

};

#endif //__dh_rgi__