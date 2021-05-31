/**
 * @file dh_gripper.h
 * @brief Virtual BaseClass for DH-Robotics Gripper
 * @author Shelling Ding (Renjie.ding@DH-Robotics.com)
 * @version 1.0
 * @date 2020-11-17
 * 
 * @copyright Copyright (c) 2020  Shenzhen DH-Robotics Inc.
 * 
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2020-11-17 <td>1.0     <td>Shelling Ding     <td>
 * </table>
 */

#ifndef __dh_gripper__
#define __dh_gripper__

class DH_Gripper
{

public:
    DH_Gripper(){}

    // connect to the gripper
    virtual int open() = 0;;
    // disconnect
    virtual void close() = 0;

    // initialization the gripper 
    virtual bool Initialization() = 0;
    
    virtual bool SetTargetPosition(int refpos) = 0;
    virtual bool SetTargetForce(int force) = 0;
    virtual bool SetTargetSpeed(int speed) = 0;

    virtual bool GetCurrentPosition(int &curpos) = 0;
    virtual bool GetTargetPosition(int &curpos) = 0;
    virtual bool GetTargetForce(int &curTarforce) = 0;
    virtual bool GetTargetSpeed(int &curTarpos) = 0;

    virtual bool GetInitState(int &i_state) = 0;
    virtual bool GetGripState(int &g_state) = 0;

    virtual bool GetRunStates(int states[]) =0;


    int GetGripperAxiNumber() { return gripper_axis;}
    
    enum S_INIT_STATES
    {
        S_INIT_NOT = 0,         // Need to be initialized    
        S_INIT_FINISHED = 1,    // Initialize finished
        S_INIT_DOING = 2,       // Initializing
    };

    enum S_GRIP_STATES
    {
        S_GRIP_MOVING = 0,      // gripper finger is moving
        S_GRIP_ARRIVED = 1,     // gripper finger arrived target position
        S_GRIP_CAUGHT = 2,      // gripper caught a object
        S_GRIP_DROPPED = 3,     // object dropped
    };

protected:
    int gripper_axis;
};

#endif //__dh_gripper__