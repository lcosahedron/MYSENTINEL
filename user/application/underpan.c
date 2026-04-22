#include "pid.h"
#include "canbus.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "underpan.h"
#include <math.h>
//TODO:根据实际安装情况修改ZeroAnglePoint的值，使其对应6020电机的零位角度值，单位为角度值（0-8191）
uint16_t ZeroAnglePoint = 0; //6020电机的零位
uint16_t revove_speed = 1000; //底盘旋转速度
uint16_t max_speed = 1000/660; //单个向量分量的最大速度，直接初始化为比例系数
/*******************************************************************************
  * @file    underpan.c
  * @date    2026/4/12
  * @brief   底盘控制，暂时还没有加入pid控制，之后再改。上面三个参数等待调试后再修改
  ******************************************************************************/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
CAN_RxFrame_t can2_6020_msg; //接收6020电机的消息，包含当前角度信息
double deviation_angle = 0; //底盘与大yaw正方向的偏差角
underpan_speed_t underpan_speed = {0};
static PID_Controller chassis_pid = {0};

wheel_speed_t GetUnderpanWheelSpeed(void)
{
    // 直接使用 remote_control.h 的全局 rc_data
    underpan_speed.x = 0;
    underpan_speed.y = 0;
    underpan_speed.r = 0;
    if(rc_data.Switch.switch1 == RC_SW_DOWN)//小陀螺开关，之后再叠加其他向量时会加上这个旋转向量
    {
        //现在将摇杆与键盘输入叠在一起，ch3是前后，ch2是左右，键盘的WASD分别对应前后左右，偏移660后乘以max_speed映射到实际速度
        underpan_speed.x = (rc_data.Channels.ch2 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.D_pressed - rc_data.Keyboard.A_pressed)*660) *cos(deviation_angle) + (rc_data.Channels.ch3 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.W_pressed - rc_data.Keyboard.S_pressed)*660) *sin(deviation_angle); 
        underpan_speed.y = (rc_data.Channels.ch3 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.W_pressed - rc_data.Keyboard.S_pressed)*660) *cos(deviation_angle) - (rc_data.Channels.ch2 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.D_pressed - rc_data.Keyboard.A_pressed)*660) *sin(deviation_angle);
        underpan_speed.r = revove_speed;
    }

    if(rc_data.Switch.switch1 == RC_SW_UP)//安全模式，遥控器s2开关向上时才会根据遥控器输入计算轮速，否则轮速为0
    {
        underpan_speed.x = (rc_data.Channels.ch2 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.D_pressed - rc_data.Keyboard.A_pressed)*660) *cos(deviation_angle) + (rc_data.Channels.ch3 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.W_pressed - rc_data.Keyboard.S_pressed)*660) *sin(deviation_angle); 
        underpan_speed.y = (rc_data.Channels.ch3 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.W_pressed - rc_data.Keyboard.S_pressed)*660) *cos(deviation_angle) - (rc_data.Channels.ch2 - RC_CH_VALUE_OFFSET + (rc_data.Keyboard.D_pressed - rc_data.Keyboard.A_pressed)*660) *sin(deviation_angle);
        underpan_speed.r = PID_Compute(&chassis_pid, deviation_angle); //非小陀螺模式，底盘转速由pid输出，error为底盘与大yaw正方向的偏差角，setpoint为0
    }

    wheel_speed_t wheel_speed = {0};
    wheel_speed.fl = (underpan_speed.y + underpan_speed.x + underpan_speed.r)*max_speed;
    wheel_speed.fr = (underpan_speed.x - underpan_speed.y + underpan_speed.r)*max_speed;
    wheel_speed.bl = (underpan_speed.y - underpan_speed.x + underpan_speed.r)*max_speed;
    wheel_speed.br = (-underpan_speed.x - underpan_speed.y + underpan_speed.r)*max_speed;
    return wheel_speed;
}

void StartUnderpan(void *argument)
{
  /* USER CODE BEGIN StartUnderpan */
  /* Infinite loop */
  for(;;)
  {
    //TODO:安装时配置6020电机编码，修改can2_6020_msg的ID为对应的ID
    CAN2_FindLatestById(0x206, &can2_6020_msg); //接收6020电机的角度信息
    uint16_t anglereceive = (can2_6020_msg.data[1] << 8) | can2_6020_msg.data[0]; //角度值，范围0-8191
    deviation_angle = (double)(anglereceive - ZeroAnglePoint) * 2.0 * M_PI / 8192.0; //计算偏差角，单位为弧度
    wheel_speed_t wheel_speed = GetUnderpanWheelSpeed();
    uint8_t data[8] = {0};
    if(rc_data.Switch.switch2 != RC_SW_MID) //安全模式，遥控器s2开关不在中间时才会发送底盘控制指令，否则发送0
    {
        data[0] = (uint8_t)(wheel_speed.fr & 0xFF);
        data[1] = (uint8_t)((wheel_speed.fr >> 8) & 0xFF);
        data[2] = (uint8_t)(wheel_speed.fl & 0xFF);
        data[3] = (uint8_t)((wheel_speed.fl >> 8) & 0xFF);
        data[4] = (uint8_t)(wheel_speed.bl & 0xFF);
        data[5] = (uint8_t)((wheel_speed.bl >> 8) & 0xFF);
        data[6] = (uint8_t)(wheel_speed.br & 0xFF);
        data[7] = (uint8_t)((wheel_speed.br >> 8) & 0xFF);
    }
    else //安全模式，遥控器s2开关在中间时发送0，停止底盘
    {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
    }
    CAN1_Send(0x200, data);

    osDelay(10);
  }
  /* USER CODE END StartUnderpan */
}
