#include "dh_lagacy_gripper.h"
#include "dh_device.h"

DH_Lagacy_Gripper::DH_Lagacy_Gripper(int id, std::string Portname, int Baudrate)
    :DH_Gripper(), _gripper_id(id),_PortName(Portname),_BaudRate(Baudrate),_Serialhandle(-1)
{
    this->gripper_axis = 1;
}

DH_Lagacy_Gripper::~DH_Lagacy_Gripper()
{

}

int DH_Lagacy_Gripper::open()
{
    _Serialhandle = connect_device(_PortName.c_str(), _BaudRate);

    if(_Serialhandle < 0)
    {
        std::cout << "open failed"<<std::endl;
        return -1;
    }
    else
    {
        std::cout << "open successful"<<std::endl;
        return _Serialhandle;
    }
}

void DH_Lagacy_Gripper::close()
{
    disconnect_device(_Serialhandle);
}


bool DH_Lagacy_Gripper::Initialization()
{
   return WriteRegisterFunc(0x0802,0x00);
}


bool DH_Lagacy_Gripper::SetTargetPosition(int refpos)
{
    return WriteRegisterFunc(0x0602,refpos);
}

bool DH_Lagacy_Gripper::SetTargetForce(int force)
{
    return WriteRegisterFunc(0x0502,force);
}


bool DH_Lagacy_Gripper::GetCurrentPosition(int &curpos)
{
    return ReadRegisterFunc(0x0602,curpos);
}

bool DH_Lagacy_Gripper::GetTargetPosition(int &tarpos)
{
    return ReadRegisterFunc(0x0602,tarpos);
}

bool DH_Lagacy_Gripper::GetTargetForce(int &curTarforce)
{
    return ReadRegisterFunc(0x0502,curTarforce);
}


bool DH_Lagacy_Gripper::GetInitState(int &i_state)
{
     return ReadRegisterFunc(0x0802,i_state);
}

bool DH_Lagacy_Gripper::GetGripState(int &g_state)
{
     return ReadRegisterFunc(0x0F01,g_state);
}

bool DH_Lagacy_Gripper::GetRunStates(int states[])
{
    if(this->GetInitState(states[0])) 
        if(this->GetGripState(states[1])) 
            if(this->GetCurrentPosition(states[2]))
                if(this->GetTargetPosition(states[3]))
                    if(this->GetTargetForce(states[3]))
                        return true;
    return false;
}

bool DH_Lagacy_Gripper::WriteRegisterFunc(int index, int value)
{
    unsigned char send_buf[14];
    send_buf[0] = 0xFF;
    send_buf[1] = 0xFE;
    send_buf[2] = 0xFD;
    send_buf[3] = 0xFC;

    send_buf[4] = _gripper_id;

    send_buf[5] = (index>>8)&0xff;
    send_buf[6] = index&0xff;
    send_buf[7] = 0x01;
    send_buf[8] = 0x00;

    send_buf[9] = value&0xff;
    send_buf[10] = (value>>8)&0xff;
    send_buf[11] = 0x00;
    send_buf[12] = 0x00;

    send_buf[13] = 0xFB;

    bool ret = false;
    int retrycount = 3;
    do
    {
        ret = false;
        retrycount -- ;
        if(retrycount<0)
        {
            break;
        }
        int wdlen = device_wrire(_Serialhandle, (char *)send_buf, sizeof(send_buf));
        if(sizeof(send_buf) != wdlen)
            continue; 

        char rev_buf[32];
        int rdlen = device_read(_Serialhandle, rev_buf, sizeof(rev_buf) - 1);
        // std::cout << "sed " ;
        // for(int i=0; i< wdlen;i++)
        //     std::cout <<(unsigned int)(unsigned char) send_buf[i]<<" "; 
        // std::cout << std::endl;
        // std::cout << "rev " ;
        // for(int i=0; i< rdlen;i++)
        //     std::cout <<(unsigned int)(unsigned char) rev_buf[i]<<" "; 
        // std::cout << std::endl;
        if (rdlen == sizeof(send_buf)) {
             bool checkrev = true;
            for(int i=0;i<rdlen;i++)
            {
                if(send_buf[i] != (unsigned char)rev_buf[i])
                {
                    // std::cout << "def " <<(unsigned int)(unsigned char) send_buf[i]<<" "<<(unsigned int)(unsigned char) rev_buf[i];
                    checkrev = false;     
                    break;
                }
            }
            if(checkrev)
                ret = true;
        }
        
    } while(!ret);
    
    return ret;
}


bool DH_Lagacy_Gripper::ReadRegisterFunc(int index,int &value)
{
    unsigned char send_buf[14];
    send_buf[0] = 0xFF;
    send_buf[1] = 0xFE;
    send_buf[2] = 0xFD;
    send_buf[3] = 0xFC;

    send_buf[4] = _gripper_id;

    send_buf[5] = (index>>8)&0xff;
    send_buf[6] = index&0xff;
    send_buf[7] = 0x00;
    send_buf[8] = 0x00;

    send_buf[9] = 0x00;
    send_buf[10] = 0x00;
    send_buf[11] = 0x00;
    send_buf[12] = 0x00;
    
    send_buf[13] = 0xFB;


    bool ret = false;
    int retrycount = 3;
    do
    {
        ret = false;
        retrycount -- ;
        if(retrycount<0)
        {
            break;
        }
        // std::cout << "--------read --------"<<std::endl;
        int wdlen = device_wrire(_Serialhandle, (char *)send_buf, sizeof(send_buf));
        if(sizeof(send_buf) != wdlen)
            continue; 

        // std::cout <<retrycount<<  " sed " ;
        // for(int i=0; i< wdlen;i++)
        //     std::cout <<(unsigned int)(unsigned char) send_buf[i]<<" "; 
        // std::cout << std::endl;

        char rev_buf[32];
        int rdlen = device_read(_Serialhandle, rev_buf, sizeof(rev_buf) - 1);
        
        // std::cout <<retrycount<<" rev " ;
        // for(int i=0; i< rdlen;i++)
        //     std::cout <<(unsigned int)(unsigned char) rev_buf[i]<<" "; 
        // std::cout << std::endl;
        
        if (rdlen == sizeof(send_buf)) {

            if ((unsigned char)rev_buf[0] == send_buf[0] &&
                (unsigned char)rev_buf[1] == send_buf[1] &&
                (unsigned char)rev_buf[2] == send_buf[2] &&
                (unsigned char)rev_buf[3] == send_buf[3] &&
                (unsigned char)rev_buf[4] == send_buf[4] &&
                (unsigned char)rev_buf[5] == send_buf[5] &&
                (unsigned char)rev_buf[6] == send_buf[6] &&
                // rev_buf[7] == send_buf[7] &&
                (unsigned char)rev_buf[8] == send_buf[8] &&
                // rev_buf[9] == send_buf[9] &&
                (unsigned char)rev_buf[10] == send_buf[10] &&
                (unsigned char)rev_buf[11] == send_buf[11] &&
                (unsigned char)rev_buf[12] == send_buf[12] &&
                (unsigned char)rev_buf[13] == send_buf[13] )
             {
                 value = ((rev_buf[9]&0xff)|(rev_buf[10]<<8));
                ret = true;
             }
            // std::cout <<"get " << *value << " ";
            // for(int i=0;i< sizeof(send_buf);i++)
            //     std::cout<< std::hex<<(unsigned int)(unsigned char)  rev_buf[i] << " ";
            // std::cout << std::endl;
        }
        
    } while(!ret);
    return ret;
}



