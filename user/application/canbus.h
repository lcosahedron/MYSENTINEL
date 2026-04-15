#ifndef CANBUS_H
#define CANBUS_H

#include "can.h"

#define CAN_SENDER_MAX_COUNT 8u

typedef struct
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_RxFrame_t;

HAL_StatusTypeDef CAN1_BusStart(void);
HAL_StatusTypeDef CAN2_BusStart(void);

HAL_StatusTypeDef CAN1_Send(uint16_t std_id, const uint8_t data[8]);
HAL_StatusTypeDef CAN2_Send(uint16_t std_id, const uint8_t data[8]);

uint8_t CAN1_FindLatestById(uint16_t std_id, CAN_RxFrame_t *out_msg);
uint8_t CAN2_FindLatestById(uint16_t std_id, CAN_RxFrame_t *out_msg);

void CAN1_RxLatestClear(void);
void CAN2_RxLatestClear(void);

#endif /* CANBUS_H */
