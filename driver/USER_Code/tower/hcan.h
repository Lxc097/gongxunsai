#ifndef __HCAN_H__
#define __HCAN_H__

#include "stm32f4xx_hal.h"
#include "can.h"
#include "tower.h"

#define	CAN_NUM &hcan1

uint8_t CAN_Start(CAN_HandleTypeDef *hcan);
uint8_t CAN_SendEXDataChecked(CAN_HandleTypeDef *hcan, uint32_t ID,
                              const uint8_t *pData, uint8_t Len);
void Can_SendCmd(uint16_t ID,uint8_t *cmd, uint8_t len);
void CAN_SendEXData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_SendData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);








#endif
