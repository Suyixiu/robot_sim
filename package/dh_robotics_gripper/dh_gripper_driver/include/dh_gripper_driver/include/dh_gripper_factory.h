/**
 * @file dh_gripper_factory.h
 * @brief  Factory Class to generate the specify gripper control pointer
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
#ifndef __dh_gripper_factory__
#define __dh_gripper_factory__

#include "dh_ag95_can.h"
#include "dh_dh3_can.h"
#include "dh_modbus_gripper.h"
#include "dh_rgi.h"

#include <iostream>

class DH_Gripper_Factory  
{  
public:  

    DH_Gripper* CreateGripper(const std::string _gripper_model)  
    {  
        if(_gripper_model.find("AG95_CAN")!=_gripper_model.npos)
            return new DH_AG95_CAN(_gripper_id,_port_name,_parameter); 
        else if(_gripper_model.find("DH3")!=_gripper_model.npos)
            return new DH_DH3_CAN(_gripper_id,_port_name,_parameter);   
        else if(_gripper_model.find("AG95_MB")!=_gripper_model.npos
                ||_gripper_model.find("PGE")!=_gripper_model.npos
                ||_gripper_model.find("PGC")!=_gripper_model.npos
                ||_gripper_model.find("CGC")!=_gripper_model.npos)
            return new DH_Modbus_Gripper(_gripper_id,_port_name,_parameter);  
        else if(_gripper_model.find("RGI")!=_gripper_model.npos)
            return new DH_RGI(_gripper_id,_port_name,_parameter);  
        else
        {
            return NULL;
        }
        
    }  

    void Set_Parameter(int id, std::string Portname, int parameter)
    {
        _gripper_id = id;
        _port_name  = Portname;
        _parameter  = parameter;
    }
private:
    int _gripper_id;
    std::string _port_name;
    int _parameter;


}; 

#endif //__dh_gripper_factory__


