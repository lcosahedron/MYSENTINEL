#include "pid.h"
#include "canbus.h"
#include "rc.h"
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
  // max_speed 已初始化为 1000/660，无需再缩放
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "cmsis_os.h"
typedef struct
{
    int16_t x;   //最大值1684，最小值364，偏移量1024，中心位置1024，范围660
    int16_t y;   //前进后退
    int16_t r;   //顺时针为正
} underpan_speed_t;
double deviation_angle = 0; //底盘与小yaw正方向的偏差角，顺时针为正，单位为弧度
CAN_RxFrame_t can2_6020_msg; //接收6020电机的消息，包含当前角度信息
typedef struct 
{
    int16_t fl;   //前左轮
    int16_t fr;   //前右轮速度
    int16_t bl;   //后左轮速度
    int16_t br;   //后右轮速度
} wheel_speed_t;

wheel_speed_t GetUnderpanWheelSpeed(void)
{
    RC_Ctl_t RC_CtrlData = *RC_GetData();
    underpan_speed_t underpan_speed;
    if(RC_CtrlData.rc.s1 == RC_SW_DOWN)//小陀螺开关，之后再叠加其他向量时会加上这个旋转向量
    {
        underpan_speed.r = revove_speed;
    }

    wheel_speed_t wheel_speed = {0};
    if(RC_CtrlData.rc.s2 == RC_SW_UP||RC_CtrlData.rc.s2 == RC_SW_DOWN)//安全模式，遥控器s2开关向上时才会根据遥控器输入计算轮速，否则轮速为0
    {
    //现在将摇杆与键盘输入叠在一起，ch3是前后，ch2是左右，键盘的WASD分别对应前后左右，偏移660后乘以max_speed映射到实际速度
    underpan_speed.x = (RC_CtrlData.rc.ch2 - RC_CH_VALUE_OFFSET + (RC_CtrlData.key.v&KEY_PRESSED_OFFSET_D - RC_CtrlData.key.v&KEY_PRESSED_OFFSET_A)*660) *cos(deviation_angle) + (RC_CtrlData.rc.ch3 - RC_CH_VALUE_OFFSET + (RC_CtrlData.key.v&KEY_PRESSED_OFFSET_W - RC_CtrlData.key.v&KEY_PRESSED_OFFSET_S)*660) *sin(deviation_angle); 
    underpan_speed.y = (RC_CtrlData.rc.ch3 - RC_CH_VALUE_OFFSET + (RC_CtrlData.key.v&KEY_PRESSED_OFFSET_W - RC_CtrlData.key.v&KEY_PRESSED_OFFSET_S)*660) *cos(deviation_angle) - (RC_CtrlData.rc.ch2 - RC_CH_VALUE_OFFSET + (RC_CtrlData.key.v&KEY_PRESSED_OFFSET_D - RC_CtrlData.key.v&KEY_PRESSED_OFFSET_A)*660) *sin(deviation_angle);
    underpan_speed.r += RC_CtrlData.rc.ch0 - RC_CH_VALUE_OFFSET;
    
    wheel_speed.fl = (underpan_speed.y + underpan_speed.x + underpan_speed.r)*max_speed;
    wheel_speed.fr = (underpan_speed.x - underpan_speed.y + underpan_speed.r)*max_speed;
    wheel_speed.bl = (underpan_speed.y - underpan_speed.x + underpan_speed.r)*max_speed;
    wheel_speed.br = (-underpan_speed.x - underpan_speed.y + underpan_speed.r)*max_speed;
    }
    return wheel_speed;
}

void StartUnderpan(void *argument)
{
  /* USER CODE BEGIN StartUnderpan */
  /* Infinite loop */
  for(;;)
  {
    //TODO:安装时配置6020电机编码，修改can2_6020_msg的ID为对应的ID
    CAN2_FindLatestById(0x201, &can2_6020_msg); //接收6020电机的角度信息
    uint16_t anglereceive = (can2_6020_msg.data[1] << 8) | can2_6020_msg.data[0]; //角度值，范围0-8191
    deviation_angle = (double)(anglereceive - ZeroAnglePoint) * 2.0 * M_PI / 8192.0; //计算偏差角，单位为弧度
    wheel_speed_t wheel_speed = GetUnderpanWheelSpeed();
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(wheel_speed.fl & 0xFF);
    data[1] = (uint8_t)((wheel_speed.fl >> 8) & 0xFF);
    data[2] = (uint8_t)(wheel_speed.fr & 0xFF);
    data[3] = (uint8_t)((wheel_speed.fr >> 8) & 0xFF);
    data[4] = (uint8_t)(wheel_speed.bl & 0xFF);
    data[5] = (uint8_t)((wheel_speed.bl >> 8) & 0xFF);
    data[6] = (uint8_t)(wheel_speed.br & 0xFF);
    data[7] = (uint8_t)((wheel_speed.br >> 8) & 0xFF);
    CAN1_Send(0x200, data);

    osDelay(1);
  }
  /* USER CODE END StartUnderpan */
}
