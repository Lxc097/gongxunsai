#ifndef __SOFTSPI_OLED_H
#define __SOFTSPI_OLED_H

#include "main.h"
#include "stdint.h"
#include "stdlib.h"

// 引脚定义（需与CubeMX配置一致）
#define SOFTSPI_OLED_SCL_PORT    GPIOB
#define SOFTSPI_OLED_SCL_PIN     GPIO_PIN_13
#define SOFTSPI_OLED_SDA_PORT    GPIOC
#define SOFTSPI_OLED_SDA_PIN     GPIO_PIN_3
#define SOFTSPI_OLED_RES_PORT    GPIOE
#define SOFTSPI_OLED_RES_PIN     GPIO_PIN_2
#define SOFTSPI_OLED_DC_PORT     GPIOE
#define SOFTSPI_OLED_DC_PIN      GPIO_PIN_3
#define SOFTSPI_OLED_CS_PORT     GPIOE
#define SOFTSPI_OLED_CS_PIN      GPIO_PIN_4

// 引脚操作宏定义（HAL库风格）
#define SoftSPI_OLED_SCL_Clr()   HAL_GPIO_WritePin(SOFTSPI_OLED_SCL_PORT, SOFTSPI_OLED_SCL_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_SCL_Set()   HAL_GPIO_WritePin(SOFTSPI_OLED_SCL_PORT, SOFTSPI_OLED_SCL_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_SDA_Clr()   HAL_GPIO_WritePin(SOFTSPI_OLED_SDA_PORT, SOFTSPI_OLED_SDA_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_SDA_Set()   HAL_GPIO_WritePin(SOFTSPI_OLED_SDA_PORT, SOFTSPI_OLED_SDA_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_RES_Clr()   HAL_GPIO_WritePin(SOFTSPI_OLED_RES_PORT, SOFTSPI_OLED_RES_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_RES_Set()   HAL_GPIO_WritePin(SOFTSPI_OLED_RES_PORT, SOFTSPI_OLED_RES_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_DC_Clr()    HAL_GPIO_WritePin(SOFTSPI_OLED_DC_PORT, SOFTSPI_OLED_DC_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_DC_Set()    HAL_GPIO_WritePin(SOFTSPI_OLED_DC_PORT, SOFTSPI_OLED_DC_PIN, GPIO_PIN_SET)
#define SoftSPI_OLED_CS_Clr()    HAL_GPIO_WritePin(SOFTSPI_OLED_CS_PORT, SOFTSPI_OLED_CS_PIN, GPIO_PIN_RESET)
#define SoftSPI_OLED_CS_Set()    HAL_GPIO_WritePin(SOFTSPI_OLED_CS_PORT, SOFTSPI_OLED_CS_PIN, GPIO_PIN_SET)

// 命令/数据定义
#define SOFTSPI_OLED_CMD  0
#define SOFTSPI_OLED_DATA 1

// 全局显存数组
extern uint8_t SoftSPI_OLED_GRAM[144][8];

// 函数声明
void SoftSPI_OLED_ColorTurn(uint8_t i);
void SoftSPI_OLED_DisplayTurn(uint8_t i);
void SoftSPI_OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void SoftSPI_OLED_DisPlay_On(void);
void SoftSPI_OLED_DisPlay_Off(void);
void SoftSPI_OLED_Refresh(void);
void SoftSPI_OLED_Clear(void);
void SoftSPI_OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void SoftSPI_OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);
void SoftSPI_OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void SoftSPI_OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode);
void SoftSPI_OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode);
uint32_t SoftSPI_OLED_Pow(uint8_t m, uint8_t n);
void SoftSPI_OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode);
void SoftSPI_OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode);
void SoftSPI_OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode);
void SoftSPI_OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode);
void SoftSPI_OLED_Init(void);

void SoftSPI_OLED_ShowBN(uint8_t x, uint8_t y, uint8_t num, uint8_t mode);

#endif
