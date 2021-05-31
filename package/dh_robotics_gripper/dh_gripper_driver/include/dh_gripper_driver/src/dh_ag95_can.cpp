#include "dh_ag95_can.h"

DH_AG95_CAN::DH_AG95_CAN(int id, std::string Portname, int Baudrate):
    DH_Lagacy_Gripper(id,Portname, Baudrate)
{
    this->gripper_axis = 1;
}

DH_AG95_CAN::~DH_AG95_CAN()
{

}

bool DH_AG95_CAN::SetTargetPosition(int refpos)
{
    if(refpos<0)
        refpos = 0;
    if(refpos>100)
        refpos = 100;

    return WriteRegisterFunc(0x0602,refpos);
}

bool DH_AG95_CAN::SetTargetForce(int force)
{
    if(force<20)
        force = 20;
    if(force>100)
        force = 100;
    return WriteRegisterFunc(0x0502,force);
}