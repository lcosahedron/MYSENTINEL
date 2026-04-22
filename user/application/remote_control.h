#ifndef RC_CH_VALUE_OFFSET
#define RC_CH_VALUE_OFFSET 1024
#endif
#ifndef RC_SW_UP
#define RC_SW_UP           1
#endif
#ifndef RC_SW_MID
#define RC_SW_MID          3
#endif
#ifndef RC_SW_DOWN
#define RC_SW_DOWN         2
#endif
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "usart.h"

//老图传为DR16 新图传为VT03
#define DR16 0x01
#define VT03 0x02
// 选择控制器
#define Controller DR16

#define IRT_unpack 0x01
#define FREERTOS_unpack 0x02
// 选择解包方式
#define UnpackRoute IRT_unpack

#if Controller == DR16
#define RX_BUFFER_SIZE 18
#endif

#define ctrl_uart_port huart1

typedef struct
{
    uint16_t ch0;
    uint16_t ch1;           
    uint16_t ch2;
    uint16_t ch3;
    uint16_t thumb_wheel;   //新图传拨轮
} channel_data_t;

typedef struct
{
    uint8_t switch1;
    uint8_t switch2;
} switch_data_t;

typedef struct 
{
    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
    uint8_t button4;
    uint8_t button5;
} button_data_t;

typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t left_pressed;
    uint8_t right_pressed;
    uint8_t middle_pressed;
} mouse_data_t;

typedef struct
{
    uint16_t key_buffer;
    uint8_t W_pressed;
    uint8_t S_pressed;
    uint8_t A_pressed;
    uint8_t D_pressed;
    uint8_t shift_pressed;
    uint8_t ctrl_pressed;
    uint8_t Q_pressed;
    uint8_t E_pressed;
    uint8_t R_pressed;
    uint8_t F_pressed;
    uint8_t G_pressed;
    uint8_t Z_pressed;
    uint8_t X_pressed;
    uint8_t C_pressed;
    uint8_t V_pressed;
    uint8_t B_pressed;
}keyboard_data_t;

typedef struct
{
    channel_data_t Channels;
    switch_data_t Switch;
    button_data_t Buttons;
    mouse_data_t Mouse;
    keyboard_data_t Keyboard;
} rc_data_t;

extern rc_data_t rc_data;

void RC_Init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void RC_data_unpack(void);

#endif /* REMOTE_CONTROL_H */
