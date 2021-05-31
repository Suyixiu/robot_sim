/**
 * @file dh_dh3_can.h
 * @brief Driver for AG-95 Gripper Communication
 * @author Shelling Ding (Renjie.ding@DH-Robotics.com)
 * @version 1.0
 * @date 2020-11-17
 * 
 * @copyright Copyright (c) 2020  Shenzhen DH-Robotics Inc.
 * 
 * @par HeadFile For AG95 Driver
 *      inherit from DH_Lagacy_Gripper
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-11-17 <td>1.0     <td>Shelling Ding     <td>内容
 * </table>
 */
#ifndef __dh_dh3_can__
#define __dh_dh3_can__

#include "dh_lagacy_gripper.h"

class DH_DH3_CAN: public DH_Lagacy_Gripper
{
public:
    DH_DH3_CAN(int id, std::string Portname, int parameter);
    ~DH_DH3_CAN();
    // set gripper target position 0-100 %
    bool SetTargetPosition(int refpos);
    // set gripper target force 20-100 %
    bool SetTargetForce(int g_force);

    // set gripper target force 0-100 %
    bool SetTargetRotation(int rot);
    
    // get gripper current rotarion 
    bool GetCurrentRotation(int &cur_rot);
    // get gripper target rotarion 
    bool GetCurrentTargetRotation(int &tar_rot);


    // get gripper rotation state
    bool GetRotationState(int &r_state);
        //get states: initialization, grip state, current position, target position, target force
    //                          rotation state, current angle, target angle, target torque
    bool GetRunStates(int states[]);

    enum S_ROTATION_STATES
    {
        S_ROT_MOVING = 0,       // Rotating
        S_ROT_UNDEFINE = 1,      // UNDEFINE
        S_ROT_ARRIVED = 2,      // Rotation is blocked
        S_ROT_BLOCKED = 3,  // Had been blocked
    };
protected:
    int _target_rotation;
};

#endif //__dh_dh3_can__