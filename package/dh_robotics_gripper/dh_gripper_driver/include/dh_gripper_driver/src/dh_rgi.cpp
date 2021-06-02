#include "dh_rgi.h"

DH_RGI::DH_RGI(int id, std::string Portname, int Baudrate):
    DH_Modbus_Gripper(id,Portname, Baudrate)
{
    this->gripper_axis = 2;
}

DH_RGI::~DH_RGI()
{

}


bool DH_RGI::SetTargetRotation(int angle)
{
     return WriteRegisterFunc(0x0105,angle);
}

bool DH_RGI::SetTargetRotationTorque(int torque)
{
    return WriteRegisterFunc(0x0108,torque);
}

bool DH_RGI::SetTargetRotationSpeed(int speed)
{
    return WriteRegisterFunc(0x0107,speed);
}


bool DH_RGI::GetCurrentRotation(int &curAngle)
{
    return ReadRegisterFunc(0x0208,curAngle);
}

bool DH_RGI::GetCurrentTargetRotation(int &curAngle)
{
    return ReadRegisterFunc(0x0105,curAngle);
}

bool DH_RGI::GetCurrentTargetRotationTorque(int &curTarRotTorque)
{
    return ReadRegisterFunc(0x0108,curTarRotTorque);
}

bool DH_RGI::GetCurrentTargetRotationSpeed(int &curTarRotSpeed)
{
    return ReadRegisterFunc(0x0107,curTarRotSpeed);
}

bool DH_RGI::GetRotationState(int &r_state)
{
    return ReadRegisterFunc(0x020B,r_state);
}

bool DH_RGI::GetRotationInitializationState(int &ri_state)
{
    return ReadRegisterFunc(0x020A,ri_state);
}

bool DH_RGI::GetRunStates(int states[])
{
    if(this->GetInitState(states[0])) 
        if(this->GetGripState(states[1])) 
            if(this->GetCurrentPosition(states[2]))
                if(this->GetTargetPosition(states[3]))
                     if(this->GetTargetForce(states[4]))
                        if(this->GetRotationState(states[5]))
                            if(this->GetCurrentRotation(states[6]))
                                if(this->GetCurrentTargetRotation(states[7]))
                                    if(this->GetCurrentTargetRotationTorque(states[8]))
                                        return true;
    return false;
}