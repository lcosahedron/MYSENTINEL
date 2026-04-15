#ifndef UNDERPAN_H
#define UNDERPAN_H

#include "canbus.h"

typedef struct
{
    int16_t x;   //最大值1684，最小值364，偏移量1024，中心位置1024，范围660
    int16_t y;   //前进后退
    int16_t r;   //顺时针为正
} underpan_speed_t;


typedef struct 
{
    int16_t fl;   //前左轮
    int16_t fr;   //前右轮速度
    int16_t bl;   //后左轮速度
    int16_t br;   //后右轮速度
} wheel_speed_t;

extern uint16_t ZeroAnglePoint; //6020电机的零位
extern uint16_t revove_speed; //底盘旋转速度
extern uint16_t max_speed; //单个向量分量的最大速度，直接初始化为比例系数
extern double deviation_angle; //底盘与小yaw正方向的偏差角，顺时针为正，单位为弧度
extern CAN_RxFrame_t can2_6020_msg; //接收6020电机的消息，包含当前角度信息

extern underpan_speed_t underpan_speed;//发给大yaw作为修正的速度向量

#endif // UNDERPAN_H
