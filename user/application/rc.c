#include "rc.h"

#include <string.h>
/*******************************************************************************
  * @file    rc.c
  * @date    2026/4/11
  * @brief   遥控器数据接收处理，使用uart3的空闲中断+DMA接收方式，接收完成后会调用回调函数进行数据解析
  * 可调用const RC_Ctl_t *RC_GetData(void)函数获取最新的遥控器数据，函数返回RC_Ctl_t的指针，数据结构详见rc.h
  ******************************************************************************/

volatile uint8_t sbus_rx_buffer[2][RC_FRAME_LENGTH];

static RC_Ctl_t RC_CtrlData;
static uint8_t rc_active_buffer_index = 0u;

void RC_ProcessFrame(const uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}

	RC_CtrlData.rc.ch0 = ((uint16_t)pData[0] | ((uint16_t)pData[1] << 8)) & 0x07FF;
	RC_CtrlData.rc.ch1 = (((uint16_t)pData[1] >> 3) | ((uint16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((uint16_t)pData[2] >> 6) | ((uint16_t)pData[3] << 2) | ((uint16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((uint16_t)pData[4] >> 1) | ((uint16_t)pData[5] << 7)) & 0x07FF;

	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000Cu) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003u);

	RC_CtrlData.mouse.x = (int16_t)((uint16_t)pData[6] | ((uint16_t)pData[7] << 8));
	RC_CtrlData.mouse.y = (int16_t)((uint16_t)pData[8] | ((uint16_t)pData[9] << 8));
	RC_CtrlData.mouse.z = (int16_t)((uint16_t)pData[10] | ((uint16_t)pData[11] << 8));
	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];

	RC_CtrlData.key.v = (uint16_t)pData[14] | ((uint16_t)pData[15] << 8);
}

const RC_Ctl_t *RC_GetData(void)
{
	return &RC_CtrlData;
}

void RC_Init(void)
{
	memset((void *)&RC_CtrlData, 0, sizeof(RC_CtrlData));
	memset((void *)sbus_rx_buffer, 0, sizeof(sbus_rx_buffer));
	rc_active_buffer_index = 0u;

	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)sbus_rx_buffer[rc_active_buffer_index], RC_FRAME_LENGTH);
	if (huart3.hdmarx != NULL)
	{
		__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t done_buffer = 0u;

	if (huart == NULL || huart->Instance != USART3)
	{
		return;
	}

	done_buffer = rc_active_buffer_index;

	if (Size == RC_FRAME_LENGTH)
	{
		RC_ProcessFrame((const uint8_t *)sbus_rx_buffer[done_buffer]);
	}

	rc_active_buffer_index ^= 1u;
	(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)sbus_rx_buffer[rc_active_buffer_index], RC_FRAME_LENGTH);
	if (huart3.hdmarx != NULL)
	{
		__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
	}
}
