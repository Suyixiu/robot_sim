#include <iostream>
#include "ros/ros.h"
#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"
#include "dh_gripper_msgs/GripperRotCtrl.h"
#include "dh_gripper_msgs/GripperRotState.h"


ros::Subscriber _sub_grip_state ;
ros::Publisher _gripper_ctrl_pub;
ros::Subscriber _sub_rot_state ;
ros::Publisher _rot_ctrl_pub;

dh_gripper_msgs::GripperState _g_state;
dh_gripper_msgs::GripperRotState _r_state;

void _update_gripper_state(const dh_gripper_msgs::GripperState::ConstPtr& msg)
{
        _g_state.header = msg->header;
        _g_state.is_initialized = msg->is_initialized;
        
        _g_state.grip_state  =  msg->grip_state;
        _g_state.position = msg->position;
        _g_state.target_position = msg->target_position;
        _g_state.target_force = msg->target_force;
         //ROS_INFO("state : %ld %lf %lf %lf %lf", _g_state.is_initialized, _g_state.grip_state, _g_state.position, _g_state.target_position, _g_state.target_force);
}

void _update_gripper_rot_state(const dh_gripper_msgs::GripperRotState::ConstPtr& msg)
{
        _r_state.header = msg->header;
        _r_state.rot_state = msg->rot_state;
        _r_state.angle = msg->angle;
        _r_state.target_angle = msg->target_angle;
        _r_state.target_force = msg->target_force;
}

void AG95_Test()
{
        dh_gripper_msgs::GripperCtrl msg_g_ctrl;
        msg_g_ctrl.initialize = true;
        msg_g_ctrl.position = 100;
        msg_g_ctrl.force = 100;
        msg_g_ctrl.speed = 100;
        _gripper_ctrl_pub.publish(msg_g_ctrl);
        
        while(!_g_state.is_initialized)
        {
                ros::spinOnce();
        }

        int test_state = 0;
       while (ros::ok())
        {
                switch(test_state)
                {
                        case 0:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 100;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 1;
                                break;
                        case 1:
                                if(_g_state.grip_state == 0);
                                        test_state = 2;
                                break;
                        case 2:
                                if(_g_state.grip_state != 0);
                                        test_state = 3;
                                break;
                        case 3:
                                ros::Duration(0.2).sleep();
                                test_state = 4;
                        case 4:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 0;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 5;
                                break;
                        case 5:
                                if(_g_state.grip_state == 0);
                                        test_state = 6;
                                break;
                        case 6:
                                if(_g_state.grip_state != 0);
                                        test_state = 7;
                                break;
                        case 7:
                                ros::Duration(0.2).sleep();
                                test_state = 0;
                }
                ros::spinOnce();
        }
}

void DH3_Test()
{
        dh_gripper_msgs::GripperCtrl msg_g_ctrl;
        msg_g_ctrl.initialize = true;
        msg_g_ctrl.position = 1000;
        msg_g_ctrl.force = 100;
        msg_g_ctrl.speed = 100;
        _gripper_ctrl_pub.publish(msg_g_ctrl);
        
        while(!_g_state.is_initialized)
        {
                ros::spinOnce();
        }

        int test_state = 0;
       while (ros::ok())
        {
                switch(test_state)
                {
                        case 0:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 1000;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 1;
                                break;
                        case 1:
                                if(_g_state.grip_state == 0);
                                        test_state = 2;
                                break;
                        case 2:
                                if(_g_state.grip_state != 0);
                                        test_state = 3;
                                break;
                        case 3:
                                ros::Duration(0.2).sleep();
                                test_state = 4;
                        case 4:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 0;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 5;
                                break;
                        case 5:
                                if(_g_state.grip_state == 0);
                                        test_state = 6;
                                break;
                        case 6:
                                if(_g_state.grip_state != 0);
                                        test_state = 7;
                                break;
                        case 7:
                                ros::Duration(0.2).sleep();
                                test_state = 0;
                }
                ros::spinOnce();
        }
}

void ModbusGripper_test()
{
        dh_gripper_msgs::GripperCtrl msg_g_ctrl;
        msg_g_ctrl.initialize = true;
        msg_g_ctrl.position = 1000;
        msg_g_ctrl.force = 100;
        msg_g_ctrl.speed = 100;
        _gripper_ctrl_pub.publish(msg_g_ctrl);
        
        while(!_g_state.is_initialized)
        {
                ros::spinOnce();
        }

        int test_state = 0;
       while (ros::ok())
        {
                switch(test_state)
                {
                        case 0:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 1000;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 1;
                                break;
                        case 1:
                                if(_g_state.grip_state == 0);
                                        test_state = 2;
                                break;
                        case 2:
                                if(_g_state.grip_state != 0);
                                        test_state = 3;
                                break;
                        case 3:
                                ros::Duration(0.2).sleep();
                                test_state = 4;
                        case 4:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 0;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 5;
                                break;
                        case 5:
                                if(_g_state.grip_state == 0);
                                        test_state = 6;
                                break;
                        case 6:
                                if(_g_state.grip_state != 0);
                                        test_state = 7;
                                break;
                        case 7:
                                ros::Duration(0.2).sleep();
                                test_state = 0;
                }
                ros::spinOnce();
        }
}

void RGI_test()
{
        dh_gripper_msgs::GripperCtrl msg_g_ctrl;
        msg_g_ctrl.initialize = true;
        msg_g_ctrl.position = 1000;
        msg_g_ctrl.force = 100;
        msg_g_ctrl.speed = 100;
        _gripper_ctrl_pub.publish(msg_g_ctrl);
        
        while(!_g_state.is_initialized)
        {
                ros::spinOnce();
        }

        int test_state = 0;
       while (ros::ok())
        {
                switch(test_state)
                {
                        case 0:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 1000;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 1;
                                break;
                        case 1:
                                if(_g_state.grip_state == 0);
                                        test_state = 2;
                                break;
                        case 2:
                                if(_g_state.grip_state != 0);
                                        test_state = 3;
                                break;
                        case 3:
                                ros::Duration(0.2).sleep();
                                test_state = 4;
                        case 4:
                                msg_g_ctrl.initialize = false;
                                msg_g_ctrl.position = 0;
                                msg_g_ctrl.force = 100;
                                msg_g_ctrl.speed = 100;
                                _gripper_ctrl_pub.publish(msg_g_ctrl);
                                test_state = 5;
                                break;
                        case 5:
                                if(_g_state.grip_state == 0);
                                        test_state = 6;
                                break;
                        case 6:
                                if(_g_state.grip_state != 0);
                                        test_state = 7;
                                break;
                        case 7:
                                ros::Duration(0.2).sleep();
                                test_state = 0;
                }
                ros::spinOnce();
        }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "dh_gripper_tester");
    ros::NodeHandle n("~");
    std::string _gripper_model;
    n.param<std::string>("Gripper_Model", _gripper_model,"AG95_MB");

    ROS_INFO("Gripper_model : %s", _gripper_model.c_str());
    _sub_grip_state = n.subscribe("/gripper/states", 50, _update_gripper_state);
    _gripper_ctrl_pub = n.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 50);

    if(_gripper_model.find("AG95_CAN")!=_gripper_model.npos)
    {
        AG95_Test();
    }
    else if(_gripper_model.find("DH3")!=_gripper_model.npos)
    {
        _sub_rot_state = n.subscribe("/gripper/rot_states", 50, _update_gripper_state);
        _rot_ctrl_pub = n.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/rot_ctrl", 50);
        DH3_Test();
    }
    else if(_gripper_model.find("AG95_MB")!=_gripper_model.npos
                ||_gripper_model.find("PGE")!=_gripper_model.npos
                ||_gripper_model.find("PGC")!=_gripper_model.npos
                ||_gripper_model.find("CGC")!=_gripper_model.npos)
    {
        ModbusGripper_test();
    }
    else if(_gripper_model.find("RGI")!=_gripper_model.npos)
    {
        _sub_rot_state = n.subscribe("/gripper/rot_states", 50, _update_gripper_state);
        _rot_ctrl_pub = n.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/rot_ctrl", 50);
        RGI_test();
    }
    else
    {
            return -1;
    }
    return 0;
}