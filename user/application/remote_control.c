#include "remote_control.h"
#include "usart.h"

uint8_t rx_buffer[RX_BUFFER_SIZE];
rc_data_t rc_data;
#if UnpackRoute == FREERTOS_unpack
uint8_t rc_it_flag = 0;
#endif

void RC_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&ctrl_uart_port, rx_buffer, sizeof(rx_buffer));
    __HAL_DMA_DISABLE_IT(ctrl_uart_port.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == ctrl_uart_port.Instance)
    {
        if (Size == RX_BUFFER_SIZE)
        {
        #if UnpackRoute == IRT_unpack
            RC_data_unpack();
        #elif UnpackRoute == FREERTOS_unpack
            rc_it_flag = 1;
        #endif
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&ctrl_uart_port, rx_buffer, sizeof(rx_buffer));
        __HAL_DMA_DISABLE_IT(ctrl_uart_port.hdmarx, DMA_IT_HT);
    }
}

void RC_data_unpack(void)
{
#if Controller == DR16
    /* DR16 摇杆通道解包 */
    rc_data.Channels.ch0 = ((rx_buffer[0] | (rx_buffer[1] << 8)) & 0x07FF);
    rc_data.Channels.ch1 = (((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07FF);
    rc_data.Channels.ch2 = (((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07FF);
    rc_data.Channels.ch3 = (((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07FF);
    rc_data.Channels.thumb_wheel = 0; // DR16无拨轮

    /* DR16 拨杆解包 */
    rc_data.Switch.switch1 = ((rx_buffer[5] >> 6) & 0x03);
    rc_data.Switch.switch2 = ((rx_buffer[5] >> 4) & 0x03);

    /* DR16 鼠标解包 */
    rc_data.Mouse.mouse_x = (int16_t)(rx_buffer[6] | (rx_buffer[7] << 8));
    rc_data.Mouse.mouse_y = (int16_t)(rx_buffer[8] | (rx_buffer[9] << 8));
    rc_data.Mouse.mouse_z = (int16_t)(rx_buffer[10] | (rx_buffer[11] << 8));
    rc_data.Mouse.left_pressed = rx_buffer[12];
    rc_data.Mouse.right_pressed = rx_buffer[13];
    rc_data.Mouse.middle_pressed = 0; // DR16无中键

    /* DR16 键盘解包 */
    rc_data.Keyboard.key_buffer = (uint16_t)(rx_buffer[14] | (rx_buffer[15] << 8));

#elif Controller == VT03
    /* VT03(VTM) 摇杆及拨轮解包 */
    rc_data.Channels.ch0 = ((rx_buffer[2] >> 0) | (rx_buffer[3] << 8)) & 0x07FF;
    rc_data.Channels.ch1 = ((rx_buffer[3] >> 3) | (rx_buffer[4] << 5)) & 0x07FF;
    rc_data.Channels.ch2 = ((rx_buffer[4] >> 6) | (rx_buffer[5] << 2) | (rx_buffer[6] << 10)) & 0x07FF;
    rc_data.Channels.ch3 = ((rx_buffer[6] >> 1) | (rx_buffer[7] << 7)) & 0x07FF;
    rc_data.Channels.thumb_wheel = ((rx_buffer[8] >> 1) | (rx_buffer[9] << 7)) & 0x07FF;

    /* VT03(VTM) 拨杆解包 */
    rc_data.Switch.switch1 = (rx_buffer[7] >> 4) & 0x03;
    rc_data.Switch.switch2 = 0U; // 无sw2

    /* VT03(VTM) 图传链路独立按键解包 */
    rc_data.Buttons.button1 = (rx_buffer[7] >> 6) & 0x01; // 对应原 pause
    rc_data.Buttons.button2 = (rx_buffer[7] >> 7) & 0x01; // 对应原 btn_l
    rc_data.Buttons.button3 = (rx_buffer[8] >> 0) & 0x01; // 对应原 btn_r
    rc_data.Buttons.button4 = (rx_buffer[9] >> 4) & 0x01; // 对应原 trigger
    rc_data.Buttons.button5 = 0U;

    /* VT03(VTM) 鼠标解包 */
    rc_data.Mouse.mouse_x = (int16_t)(rx_buffer[10] | (rx_buffer[11] << 8));
    rc_data.Mouse.mouse_y = (int16_t)(rx_buffer[12] | (rx_buffer[13] << 8));
    rc_data.Mouse.mouse_z = (int16_t)(rx_buffer[14] | (rx_buffer[15] << 8));
    rc_data.Mouse.left_pressed = rx_buffer[16] & 0x01;
    rc_data.Mouse.right_pressed = (rx_buffer[16] >> 1) & 0x01;
    rc_data.Mouse.middle_pressed = (rx_buffer[16] >> 2) & 0x01;

    /* VT03(VTM) 键盘解包 */
    rc_data.Keyboard.key_buffer = (uint16_t)(rx_buffer[17] | (rx_buffer[18] << 8));

#endif

    /* 通用键盘位姿解包合并（解出具体按键状态） */
    rc_data.Keyboard.W_pressed      = (rc_data.Keyboard.key_buffer >> 0) & 0x01;
    rc_data.Keyboard.S_pressed      = (rc_data.Keyboard.key_buffer >> 1) & 0x01;
    rc_data.Keyboard.A_pressed      = (rc_data.Keyboard.key_buffer >> 2) & 0x01;
    rc_data.Keyboard.D_pressed      = (rc_data.Keyboard.key_buffer >> 3) & 0x01;
    #if Controller == DR16
    rc_data.Keyboard.shift_pressed  = (rc_data.Keyboard.key_buffer >> 6) & 0x01;
    rc_data.Keyboard.ctrl_pressed   = (rc_data.Keyboard.key_buffer >> 7) & 0x01;
    rc_data.Keyboard.Q_pressed      = (rc_data.Keyboard.key_buffer >> 4) & 0x01;
    rc_data.Keyboard.E_pressed      = (rc_data.Keyboard.key_buffer >> 5) & 0x01;
    #elif Controller == VT03
    rc_data.Keyboard.shift_pressed  = (rc_data.Keyboard.key_buffer >> 4) & 0x01;
    rc_data.Keyboard.ctrl_pressed   = (rc_data.Keyboard.key_buffer >> 5) & 0x01;
    rc_data.Keyboard.Q_pressed      = (rc_data.Keyboard.key_buffer >> 6) & 0x01;
    rc_data.Keyboard.E_pressed      = (rc_data.Keyboard.key_buffer >> 7) & 0x01;
    rc_data.Keyboard.R_pressed      = (rc_data.Keyboard.key_buffer >> 8) & 0x01;
    rc_data.Keyboard.F_pressed      = (rc_data.Keyboard.key_buffer >> 9) & 0x01;
    rc_data.Keyboard.G_pressed      = (rc_data.Keyboard.key_buffer >> 10) & 0x01;
    rc_data.Keyboard.Z_pressed      = (rc_data.Keyboard.key_buffer >> 11) & 0x01;
    rc_data.Keyboard.X_pressed      = (rc_data.Keyboard.key_buffer >> 12) & 0x01;
    rc_data.Keyboard.C_pressed      = (rc_data.Keyboard.key_buffer >> 13) & 0x01;
    rc_data.Keyboard.V_pressed      = (rc_data.Keyboard.key_buffer >> 14) & 0x01;
    rc_data.Keyboard.B_pressed      = (rc_data.Keyboard.key_buffer >> 15) & 0x01;
    #endif
}
