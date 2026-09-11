#ifndef __DYNAMIC_MAP_LINK_H
#define __DYNAMIC_MAP_LINK_H

#include "main.h"

#define DYNAMIC_MAP_PROTOCOL_VERSION  1U
#define DYNAMIC_MAP_WATCHDOG_DEFAULT  300U

/* Default build is chassis-only Jetson commissioning. Legacy RC, route and
 * tower homing are isolated until a feedback-based manipulator is integrated.
 * Set to 0 explicitly only for legacy bench testing, never a competition run. */
#ifndef DYNAMIC_MAP_NAVIGATION_ONLY
#define DYNAMIC_MAP_NAVIGATION_ONLY  1
#endif

/* Static vehicle bring-up may route the host protocol through the proven
 * UART5/U47 connector. Production and default builds stay on USART6/H27. */
#ifndef DYNAMIC_MAP_LINK_UART5
#define DYNAMIC_MAP_LINK_UART5  0
#endif

void DynamicMap_Link_Init(void);
void DynamicMap_Link_Process(void);
/* Persistent actuator fault; a host STOP may not re-arm a failed motor bus. */
void DynamicMap_Link_ReportMotorFault(void);

/* 返回 1 表示速度有效；返回 0 时调用方必须停车。 */
uint8_t DynamicMap_Link_GetVelocity(int16_t *x, int16_t *y, int16_t *z);

/* 由 USART6_IRQHandler 调用，只做无阻塞入队。 */
void DynamicMap_Link_IRQHandler(void);

/* Feed a DMA-received byte span from UART5 when the static U47 build is used. */
#if DYNAMIC_MAP_LINK_UART5
void DynamicMap_Link_ReceiveBytes(const uint8_t *data, uint16_t length);
#endif

#endif
