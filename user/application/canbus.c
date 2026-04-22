#include "canbus.h"

#include <string.h>
/*******************************************************************************
  * @file    canbus.c
  * @date    2026/4/11
  * @brief   CAN总线数据收发处理，使用中断方式接收数据，接收完成后会将最新的消息保存在内部表格中
  * 可调用
  *CAN1_Send(uint16_t std_id, const uint8_t data[8])或
  *CAN2_Send(uint16_t std_id, const uint8_t data[8])函数发送消息，
  *返回值为HAL_OK表示发送成功，为HAL_ERROR表示发送失败.HAL_BUSY表示总线忙，HAL_TIMEOUT表示发送超时.
  *CAN1_FindLatestById(uint16_t std_id, CAN_RxFrame_t *out_msg)或
  *CAN2_FindLatestById(uint16_t std_id, CAN_RxFrame_t *out_msg)函数获取最新的消息，
  *返回值为1表示找到对应id的消息并成功复制到out_msg指向的结构体中，返回值为0表示未找到对应id的消息.
  *out_msg参数为CAN_RxFrame_t类型的指针，调用时需要传入一个CAN_RxFrame_t类型的变量地址，
  *CAN_RxFrame_t结构体定义详见canbus.h.
  *如需修改保存的消息（最大发送者）数量，请修改canbus.h中CAN_SENDER_MAX_COUNT的值，修改后需要重新编译.
  ******************************************************************************/

typedef struct
{
	uint8_t used;
	uint16_t std_id;
	CAN_RxFrame_t frame;
} CAN_RxLatestSlot_t;

typedef struct
{
	CAN_RxLatestSlot_t slots[CAN_SENDER_MAX_COUNT];
	uint8_t count;
} CAN_RxLatestTable_t;

static CAN_RxLatestTable_t can1_rx_latest;
static CAN_RxLatestTable_t can2_rx_latest;

static void CAN_RxLatestClear(CAN_RxLatestTable_t *table)
{
	if (table == NULL)
	{
		return;
	}

	memset(table, 0, sizeof(CAN_RxLatestTable_t));
}

static int16_t CAN_RxLatestFindIndex(const CAN_RxLatestTable_t *table, uint16_t std_id)

{
	uint8_t i = 0u;

	if (table == NULL)
	{
		return -1;
	}

	for (i = 0u; i < CAN_SENDER_MAX_COUNT; i++)
	{
		if ((table->slots[i].used != 0u) && (table->slots[i].std_id == std_id))
		{
			return (int16_t)i;
		}
	}

	return -1;
}

static int16_t CAN_RxLatestFindEmptyIndex(const CAN_RxLatestTable_t *table)
{
	uint8_t i = 0u;

	if (table == NULL)
	{
		return -1;
	}

	for (i = 0u; i < CAN_SENDER_MAX_COUNT; i++)
	{
		if (table->slots[i].used == 0u)
		{
			return (int16_t)i;
		}
	}

	return -1;
}

static void CAN_RxLatestStore(CAN_RxLatestTable_t *table, const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
	int16_t index = -1;

	if (table == NULL || header == NULL || data == NULL)
	{
		return;
	}

	if (header->IDE != CAN_ID_STD)
	{
		return;
	}

	index = CAN_RxLatestFindIndex(table, (uint16_t)header->StdId);
	if (index < 0)
	{
		index = CAN_RxLatestFindEmptyIndex(table);
		if (index < 0)
		{
			return;
		}

		table->slots[index].used = 1u;
		table->slots[index].std_id = (uint16_t)header->StdId;
		table->count++;
	}

	table->slots[index].frame.header = *header;
	memcpy(table->slots[index].frame.data, data, 8u);
}

HAL_StatusTypeDef CAN1_BusStart(void)
{
	CAN_FilterTypeDef filter = {0};

	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;

	extern volatile uint8_t error_code;
	CAN1_RxLatestClear();

	if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
	{
		error_code = 11;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET); // Green LED on
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Green LED off
		return HAL_ERROR;
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		error_code = 12;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		return HAL_ERROR;
	}

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		error_code = 13;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		return HAL_ERROR;
	}

	// if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	// if (HAL_CAN_Start(&hcan1) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	// if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	return HAL_OK;
}

HAL_StatusTypeDef CAN2_BusStart(void)
{
	CAN_FilterTypeDef filter = {0};

	filter.FilterBank = 14;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 14;

	extern volatile uint8_t error_code;
	CAN2_RxLatestClear();

	if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
	{
		error_code = 21;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		return HAL_ERROR;
	}

	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		error_code = 22;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		return HAL_ERROR;
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		error_code = 23;
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
		return HAL_ERROR;
	}

	// if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	// if (HAL_CAN_Start(&hcan2) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	// if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	// {
	// 	return HAL_ERROR;
	// }

	return HAL_OK;
}

HAL_StatusTypeDef CAN1_Send(uint16_t std_id, const uint8_t data[8])
{
	CAN_TxHeaderTypeDef tx_header = {0};
	uint32_t tx_mailbox = 0u;

	if (data == NULL)
	{
		return HAL_ERROR;
	}

	tx_header.StdId = (uint32_t)(std_id & 0x07FFu);
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8u;
	tx_header.TransmitGlobalTime = DISABLE;

	return HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &tx_mailbox);
}

HAL_StatusTypeDef CAN2_Send(uint16_t std_id, const uint8_t data[8])
{
	CAN_TxHeaderTypeDef tx_header = {0};
	uint32_t tx_mailbox = 0u;

	if (data == NULL)
	{
		return HAL_ERROR;
	}

	tx_header.StdId = (uint32_t)(std_id & 0x07FFu);
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8u;
	tx_header.TransmitGlobalTime = DISABLE;

	return HAL_CAN_AddTxMessage(&hcan2, &tx_header, (uint8_t *)data, &tx_mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	CAN_RxLatestTable_t *table = NULL;

	if (hcan == NULL)
	{
		return;
	}

	if (hcan->Instance == CAN1)
	{
		table = &can1_rx_latest;
	}
	else if (hcan->Instance == CAN2)
	{
		table = &can2_rx_latest;
	}
	else
	{
		return;
	}

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
		{
			CAN_RxLatestStore(table, &rx_header, rx_data);
		}
		else
		{
			break;
		}
	}
}
//查找对应ID的最新消息并复制到out_msg指向的结构体中，返回值为1表示找到对应id的消息并成功复制到out_msg指向的结构体中，返回值为0表示未找到对应id的消息.
uint8_t CAN1_FindLatestById(uint16_t std_id, CAN_RxFrame_t *out_msg)
{
	int16_t index = -1;

	if (out_msg == NULL)
	{
		return 0u;
	}

	index = CAN_RxLatestFindIndex(&can1_rx_latest, std_id);
	if (index < 0)
	{
		return 0u;
	}

	*out_msg = can1_rx_latest.slots[index].frame;

	return 1u;
}

uint8_t CAN2_FindLatestById(uint16_t std_id, CAN_RxFrame_t *out_msg)
{
	int16_t index = -1;

	if (out_msg == NULL)
	{
		return 0u;
	}

	index = CAN_RxLatestFindIndex(&can2_rx_latest, std_id);
	if (index < 0)
	{
		return 0u;
	}

	*out_msg = can2_rx_latest.slots[index].frame;

	return 1u;
}
//清空缓存
void CAN1_RxLatestClear(void)
{
    CAN_RxLatestClear(&can1_rx_latest);
}

void CAN2_RxLatestClear(void)
{
    CAN_RxLatestClear(&can2_rx_latest);
}
