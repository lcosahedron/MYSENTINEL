#include "pid.h"
#include "canbus.h"
#include "rc.h"
#include <math.h>
/*******************************************************************************
    * @file    tripod.c
    * @date    2026/4/13
    * @brief   云台控制，有pitch和yaw两个轴，非小陀螺模式下将yaw轴与底盘角度作为error传给底盘pid.
    *******************************************************************************/
double tripod_yaw = 0; //云台当前yaw角，单位为弧度
double tripod_pitch = 0; //云台当前pitch角，单位为弧度
double tripod_yaw_setpoint = 0; //云台yaw轴目标角度，单位为弧度
double tripod_pitch_setpoint = 0; //云台pitch轴目标角度，单位为弧度
