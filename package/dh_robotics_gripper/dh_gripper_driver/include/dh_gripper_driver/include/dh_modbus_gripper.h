/**
 * @file dh_modbus_gripper.h
 * @brief Driver for DH-Robotics Modbus-RTU Gripper ( PGC/PGE/PGI/RGI/CGC Serial and AG95 Modbus Version)
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
#ifndef __dh_modbus_gripper__
#define __dh_modbus_gripper__

#include "dh_gripper.h"
#include <iostream>

class DH_Modbus_Gripper : public DH_Gripper
{
public:

    DH_Modbus_Gripper(int id, std::string Portname, int Baudrate);
    ~DH_Modbus_Gripper();

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
    // set gripper target speed 1-100 %
    bool SetTargetSpeed(int speed);

    // get gripper current position 
    bool GetCurrentPosition(int &curpos);
    // get gripper Target position 
    bool GetTargetPosition(int &curpos);
    // get gripper current target force (Notice: Not actual force)
    bool GetTargetForce(int &curTarforce);
    // get gripper current target speed (Notice: Not actual speed)
    bool GetTargetSpeed(int &curTarpos);

    // get gripper initialization state
    bool GetInitState(int &i_state);
    // get gripper grip state
    bool GetGripState(int &g_state);
    //get states: initialize, grip, position, target position, target force
    bool GetRunStates(int states[]);

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
    //Modbus CRC16 
    unsigned short CRC16(const unsigned char *nData, unsigned short wLength);

    int _gripper_id;
    std::string _PortName;
    int _BaudRate;
    int _Serialhandle;



};

#endif //__dh_modbus_gripper__