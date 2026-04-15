#include "pid.h"
#include "canbus.h"
#include "rc.h"
#include "cmsis_os.h"
#include "bsp_imu.h"
#include "underpan.h"
#include <math.h>
/*******************************************************************************
    * @file    tripod.c
    * @date    2026/4/15
    * @brief   云台控制，有pitch和大小yaw三个轴，
    *******************************************************************************/
PID_Controller downyaw_pid = {1000.0f, 0.0f, 0.0f}; //大yaw的pid，比例系数先初始化为1000，之后调试时再修改
PID_Controller upyaw_pid = {1000.0f, 0.0f, 0.0f}; //小yaw的pid，比例系数先初始化为1000，之后调试时再修改
PID_Controller pitch_pid = {1000.0f, 0.0f, 0.0f}; //pitch的pid，比例系数先初始化为1000，之后调试时再修改
RC_Ctl_t RC_CtrlData;
CAN_RxFrame_t Upyaw_motor_msg;
CAN_RxFrame_t Downyaw_motor_msg;
CAN_RxFrame_t pitch_motor_msg;

uint16_t upyaw_zeropoint = 0; //小yaw6020电机的零位
uint16_t downyaw_zeropoint = 0; //大yaw6020电机的零位
//速度系数
uint16_t upyaw_speed = 1;
uint16_t downyaw_speed = 1;
uint16_t pitch_speed = 1;
//角度数据
uint16_t upyaw_angle = 0;
uint16_t downyaw_angle = 0;
uint16_t pitch_angle = 0;
//发送数据
uint16_t upyaw_data = 0;
uint16_t downyaw_data = 0;
uint16_t pitch_data = 0;

void GetTripodData(void)
{
    CAN2_FindLatestById(0x201, &Upyaw_motor_msg); //接收6020电机的角度信息
    upyaw_angle = (Upyaw_motor_msg.data[1] << 8) | Upyaw_motor_msg.data[0]; //角度值，范围0-8191
    CAN2_FindLatestById(0x202, &pitch_motor_msg); 
    pitch_angle = (pitch_motor_msg.data[1] << 8) | pitch_motor_msg.data[0];
    CAN2_FindLatestById(0x203, &Downyaw_motor_msg);
    downyaw_angle = (Downyaw_motor_msg.data[1] << 8) | Downyaw_motor_msg.data[0];

    RC_CtrlData = *RC_GetData();
    upyaw_pid.setpoint += (RC_CtrlData.rc.ch0 - RC_CH_VALUE_OFFSET + RC_CtrlData.mouse.x) * upyaw_speed;
    pitch_pid.setpoint += (RC_CtrlData.rc.ch1 - RC_CH_VALUE_OFFSET + RC_CtrlData.mouse.z) * pitch_speed;
}

void StartTripod(void *argument)
{
  /* USER CODE BEGIN StartTripod */
  
  /* Infinite loop */
  for(;;)
  {
    GetTripodData();
    upyaw_data = (uint16_t)PID_Compute(&upyaw_pid, upyaw_angle - upyaw_zeropoint);
    downyaw_data = (uint16_t)PID_Compute(&downyaw_pid, downyaw_angle - downyaw_zeropoint);
    downyaw_data -= upyaw_data; //小yaw的转动会带动大yaw转动，所以大yaw的目标速度要减去小yaw的目标速度
    downyaw_data -= underpan_speed.r*downyaw_speed; //底盘旋转也会带动大yaw转动，所以大yaw的目标速度还要减去底盘旋转速度
    pitch_data = (uint16_t)PID_Compute(&pitch_pid, pitch_angle);//这里之后应该再写个重力补偿
    uint8_t send_data[8] = {0};
    
    // if(RC_CtrlData.rc.s2 != RC_SW_MID) //安全模式，遥控器s2开关不在中间时才会发送云台控制指令，否则发送0
    // {
    //     send_data[0] = (uint8_t)(upyaw_data & 0xFF);
    //     send_data[1] = (uint8_t)((upyaw_data >> 8) & 0xFF);
    //     send_data[2] = (uint8_t)(pitch_data & 0xFF);
    //     send_data[3] = (uint8_t)((pitch_data >> 8) & 0xFF);
    //     send_data[4] = (uint8_t)(downyaw_data & 0xFF);
    //     send_data[5] = (uint8_t)((downyaw_data >> 8) & 0xFF);
    // }
    CAN2_Send(0x200, send_data); //发送给电机控制器
    osDelay(10);
  }
  /* USER CODE END StartTripod */
}
