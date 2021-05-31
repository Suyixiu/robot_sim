#include "dh_dh3_can.h"

DH_DH3_CAN::DH_DH3_CAN(int id, std::string Portname, int Baudrate):
    DH_Lagacy_Gripper(id,Portname, Baudrate)
{
    this->gripper_axis = 1;
}

DH_DH3_CAN::~DH_DH3_CAN()
{

}

bool DH_DH3_CAN::SetTargetPosition(int refpos)
{
    if(refpos<0)
        refpos = 0;
    if(refpos>95)
        refpos = 95;

    return WriteRegisterFunc(0x0602,refpos);
}

bool DH_DH3_CAN::SetTargetForce(int force)
{
    if(force<15)
        force = 15;
    if(force>90)
        force = 90;
    return WriteRegisterFunc(0x0502,force);
}

bool DH_DH3_CAN::SetTargetRotation(int rot)
{
    if(rot<0)
        rot = 0;
    if(rot>100)
        rot = 100;
    _target_rotation = rot;
    
    return WriteRegisterFunc(0x0702,rot);  
}
   
bool DH_DH3_CAN::GetCurrentRotation(int &cur_rot)
{
    return ReadRegisterFunc(0x0702,cur_rot);
}

bool DH_DH3_CAN::GetCurrentTargetRotation(int &tar_rot)
{
    tar_rot = _target_rotation;
    return true;
}

bool DH_DH3_CAN::GetRotationState(int &r_state)
{
    return ReadRegisterFunc(0x0f02,r_state);
}

bool DH_DH3_CAN::GetRunStates(int states[])
{
    if(this->GetInitState(states[0])) 
        if(this->GetGripState(states[1])) 
            if(this->GetCurrentPosition(states[2]))
                if(this->GetTargetPosition(states[3]))
                     if(this->GetTargetForce(states[4]))
                        if(this->GetRotationState(states[5]))
                            if(this->GetCurrentRotation(states[6]))
                                if(this->GetCurrentTargetRotation(states[7]))
                                    {
                                        states[8] = 100;
                                        return true;
                                    }
    return false;
}