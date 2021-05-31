/**
 * @file dh_ag95_can.h
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
 * <tr><td>2020-11-17 <td>1.0     <td>Shelling Ding     <td>
 * </table>
 */
#ifndef __dh_ag95_can__
#define __dh_ag95_can__

#include "dh_lagacy_gripper.h"

class DH_AG95_CAN: public DH_Lagacy_Gripper
{
public:
    DH_AG95_CAN(int id, std::string Portname, int parameter);
    ~DH_AG95_CAN();
    // set gripper target position 0-100 %
    bool SetTargetPosition(int refpos);
    // set gripper target force 20-100 %
    bool SetTargetForce(int force);

};

#endif //__dh_ag95_can__



