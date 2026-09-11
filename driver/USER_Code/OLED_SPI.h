/*
 * oled.h
 *
 *  Created on: 2022年9月30日
 *      Author: Unicorn_Yihao
 */
 
#ifndef __OLED_H__
#define __OLED_H__
 
#include "main.h"
#include "oledfont.h"
 
extern SPI_HandleTypeDef  hspi1;
 
#define OLED_RES_Clr() HAL_GPIO_WritePin(SPI1_RST_GPIO_Port,SPI1_RST_Pin,GPIO_PIN_RESET)//RES
#define OLED_RES_Set() HAL_GPIO_WritePin(SPI1_RST_GPIO_Port,SPI1_RST_Pin,GPIO_PIN_SET)
 
#define OLED_DC_Clr() HAL_GPIO_WritePin(SPI1_DC_GPIO_Port,SPI1_DC_Pin,GPIO_PIN_RESET)//DC
#define OLED_DC_Set() HAL_GPIO_WritePin(SPI1_DC_GPIO_Port,SPI1_DC_Pin,GPIO_PIN_SET)
 		     
#define OLED_CS_Clr()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET)//CS
#define OLED_CS_Set()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET)
 
 
void SPI_OLED_WR_CMD(uint8_t cmd);
void SPI_OLED_WR_DATA(uint8_t data);
void SPI_OLED_Init(void);
void SPI_OLED_Clear(void);
void SPI_OLED_Display_On(void);
void SPI_OLED_Display_Off(void);
void SPI_OLED_Set_Pos(uint8_t x, uint8_t y);
void SPI_OLED_On(void);
void SPI_OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2,uint8_t Color_Turn);
void SPI_OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn);
void SPI_OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size,uint8_t Color_Turn);
void SPI_OLED_ShowString(uint8_t x,uint8_t y,char*chr,uint8_t Char_Size,uint8_t Color_Turn);
void SPI_OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t Color_Turn);
void SPI_OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t Color_Turn);
void SPI_OLED_DrawGIF(uint8_t x0, uint8_t  y0, uint8_t  x1, uint8_t y1, uint8_t  k, int m, unsigned char GIF[][m]);
void SPI_OLED_HorizontalShift(uint8_t direction);
void SPI_OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end);
void SPI_OLED_VerticalAndHorizontalShift(uint8_t direction);
void SPI_OLED_DisplayMode(uint8_t mode);
void SPI_OLED_IntensityControl(uint8_t intensity);
 
void SPI_OLED_DrawDot(uint8_t x,uint8_t y);
void SPI_OLED_Draw_XLine(uint8_t x,uint8_t y,int length);
void SPI_SPI_OLED_Draw_YLine(uint8_t x,uint8_t y,int length);
void SPI_System_message(char message[16]);
 
 
#endif 
//mian.c
///* USER CODE BEGIN 2 */
// OLED_Init();                           //OLED初始
// OLED_Clear();                         //清屏
////上面的初始化以及清屏的代码在一开始处一定要写
// OLED_ShowString(0,0,"UNICORN_LI",16, 1);    //反相显示8X16字符串
// OLED_ShowString(0,2,"unicorn_li_123",12,0);//正相显示6X8字符串
// 
// OLED_ShowCHinese(0,4,0,1); //反相显示汉字“独”
// OLED_ShowCHinese(16,4,1,1);//反相显示汉字“角”
// OLED_ShowCHinese(32,4,2,1);//反相显示汉字“兽”
// OLED_ShowCHinese(0,6,0,0); //正相显示汉字“独”
// OLED_ShowCHinese(16,6,1,0);//正相显示汉字“角”
// OLED_ShowCHinese(32,6,2,0);//正相显示汉字“兽”
// 
// OLED_ShowNum(48,4,6,1,16, 0);//正相显示1位8X16数字“6”
// OLED_ShowNum(48,7,77,2,12, 1);//反相显示2位6X8数字“77”
// OLED_DrawBMP(90,0,122, 4,BMP1,0);//正相显示图片BMP1
// OLED_DrawBMP(90,4,122, 8,BMP1,1);//反相显示图片BMP1
// 
// OLED_HorizontalShift(0x26);//全屏水平向右滚动播放
// 
////显示正负浮点数的代码
////  float num1=-231.24;
////  float num2=23.375;
// 
////OLED_ShowString(0,0,"Show Decimal",12,0);
////OLED_Showdecimal(0,4,num1,3,2,12, 0);
////OLED_Showdecimal(0,6,num2,2,3,16, 1);
// 
// 
//  /* USER CODE END 2 */
