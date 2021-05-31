/**
 * @file dh_lagacy_gripper.h
 * @brief Driver for DH-Robotics gripper communication
 *         AG95 CAN2.0 Version and DH-3 CAN2.0 version
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
#ifndef __dh_lagacy_gripper__
#define __dh_lagacy_gripper__

#include "dh_gripper.h"
#include <iostream>

class DH_Lagacy_Gripper : public DH_Gripper
{
public:

    DH_Lagacy_Gripper(int id, std::string Portname, int Baudrate);
    ~DH_Lagacy_Gripper();

    // connect to the gripper
    int open();
    // disconnect
    void close();

    // initialization the gripper 
    bool Initialization();
    
    // set gripper target position 0-1000
    bool SetTargetPosition(int refpos);
    // set gripper target force 20-100 %
    bool SetTargetForce(int force);

    //Nouse in AG95 and DH3
    bool SetTargetSpeed(int speed) {return true;}

    // get gripper current position 
    bool GetCurrentPosition(int &curpos);
    // get gripper target position 
    bool GetTargetPosition(int &curpos);
    // get gripper current target force (Notice: Not actual force)
    bool GetTargetForce(int &curTarforce);

    //Nouse in AG95 and DH3
    bool GetTargetSpeed(int &curTarpos){return true;}

    // get gripper initialization state
    bool GetInitState(int &i_state);
    // get gripper grip state
    bool GetGripState(int &g_state);

    //get states: initialize, grip, position, target position, target force
    bool GetRunStates(int states[]);

    enum S_INIT_STATES
    {
        S_INIT_NOT = 0,         // Need to be initialized    
        S_INIT_FINISHED = 1,    // Initialize finished
    };

    enum S_GRIP_STATES
    {
        S_GRIP_MOVING = 0,      // gripper finger is moving
        S_GRIP_UNDEFINE = 1,    // Undefine
        S_GRIP_ARRIVED = 2,     // gripper finger arrived target position
        S_GRIP_CAUGHT = 3,      // gripper caught a object
    };

protected:
    //Modbus WriteRegisterFunc
    //para      :   index : register address ;
    //              value : write value
    //return    :   false : write failed ;
    //              true  : write successed
    bool WriteRegisterFunc(int index, int value);
    //Modbus ReadRegisterFunc
    //para      :   index : register address ;
    //              value : readed value
    //return    :   false : readed failed ;
    //              true  : readed successed
    bool ReadRegisterFunc(int index,int &value);

    int _gripper_id;
    std::string _PortName;
    int _BaudRate;
    int _Serialhandle;



};

#endif //__dh_lagacy_gripper__