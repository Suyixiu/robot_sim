#ifndef _DH_GRIPPER_SERIAL_DRIVER_H_
#define _DH_GRIPPER_SERIAL_DRIVER_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define cmd_read 0x00
#define cmd_write 0x01

typedef struct
{
	uint8_t frame_head[4];
	uint8_t DeviceID;
	uint8_t Register1;
	uint8_t Register2;
	uint8_t option;
	uint8_t reserve;
	uint8_t data[4];
	uint8_t frame_end;
} gripper_data_t;

extern serial::Serial ser;

void set_motor_force(uint8_t target_force);
bool set_motor_position(uint8_t target_position);
void gripper_init();

#endif