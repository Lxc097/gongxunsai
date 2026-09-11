/********************************************************/
/*   2022/9/30                                     
*   Unicorn_Yihao                                       
*   STM32 7针硬件SPI 0.96 OLED HAL库显示驱动   
*********************************************************
*   引脚定义：                                         
*   OLED_CS OLED_RES OLED_DC OUTPUT    Output push pull
*********************************************************
*   SPI定义：                                          
*   Mode:Transmit only Master                          
*   Hardware Nss Signal:Disable                        
*   Data Size : 8Bits                                  
*   First Bit : MSB First                              
*   CPOL : Low                                         
*   CPHA : 1 Edge	 
*********************************************************
*   接线：                                             
*   GND ---> GND                                       
*   VCC ---> 3.3V                                      
*   DO  ---> SPI_SCK       PA5                            
*   D1  ---> SPI_MOSI      PA7                            
*   RES ---> OLED_RES      PB0                            
*   DC  ---> OLED_DC       PB1                            
*   CS  ---> OLED_CS       PB10                            
*                                                      
*/                                                     
/********************************************************/
 
#include "OLED_SPI.h"
 
/**********************************************************
 * 初始化命令,根据芯片手册书写
 ***********************************************************/
uint8_t OLED_Init_CMD_Data[]={
0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,0xA1, 0xC8, 0xDA,
 
0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,0x8D, 0x14,
 
0xAF};
 
 
 
/**
 * @function: void OLED_Init(void)
 * @description: OLED初始化
 * @return {*}
 */
void SPI_OLED_Init(void)
{
 
	OLED_RES_Clr();
	HAL_Delay(200);
	OLED_RES_Set();
	
	uint8_t i = 0;  
	for(i=0; i<23; i++)
	{
		SPI_OLED_WR_CMD(OLED_Init_CMD_Data[i]);
	}
 
}
 
 
/**
 * @function: void OLED_WR_CMD(uint8_t cmd)
 * @description: 向设备写控制命令
 * @param {uint8_t} cmd 芯片手册规定的命令
 * @return {*}
 */
void SPI_OLED_WR_CMD(uint8_t cmd)
{
	 OLED_DC_Clr();
	 OLED_CS_Clr();
	 HAL_SPI_Transmit(&hspi1,&cmd,1,100); //需根据实际情况修改 
	 OLED_CS_Set();
	 OLED_DC_Set();   	  
}
 
/**
 * @function: void OLED_WR_DATA(uint8_t data)
 * @description: 向设备写控制数据
 * @param {uint8_t} data 数据
 * @return {*}
 */
void SPI_OLED_WR_DATA(uint8_t data)
{
	 OLED_DC_Set();  
	 OLED_CS_Clr();
	 HAL_SPI_Transmit(&hspi1,&data,1,100); //需根据实际情况修改 
	 OLED_CS_Set();
	 OLED_DC_Set();   	  
}
 
/**
 * @function: void OLED_On(void)
 * @description: 更新显示
 * @return {*}
 */
void SPI_OLED_On(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		SPI_OLED_WR_CMD(0xb0+i);    //设置页地址（0~7）
		SPI_OLED_WR_CMD(0x00);      //设置显示位置—列低地址
		SPI_OLED_WR_CMD(0x10);      //设置显示位置—列高地址
		for(n=0;n<128;n++)
			SPI_OLED_WR_DATA(1);
	}
}
 
 
/**
 * @function: OLED_Clear(void)
 * @description: 清屏,整个屏幕是黑色的!和没点亮一样!!!
 * @return {*}
 */
void SPI_OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		SPI_OLED_WR_CMD(0xb0+i);    //设置页地址（0~7）
		SPI_OLED_WR_CMD(0x00);      //设置显示位置—列低地址
		SPI_OLED_WR_CMD(0x10);      //设置显示位置—列高地址
		for(n=0;n<128;n++)
			SPI_OLED_WR_DATA(0);
	}
}
 
/**
 * @function: void OLED_Display_On(void)
 * @description: 开启OLED显示
 * @return {*}
 */
void SPI_OLED_Display_On(void)
{
	SPI_OLED_WR_CMD(0X8D);  //SET DCDC命令
	SPI_OLED_WR_CMD(0X14);  //DCDC ON
	SPI_OLED_WR_CMD(0XAF);  //DISPLAY ON,打开显示
}
 
 
/**
 * @function: void OLED_Display_Off(void)
 * @description: 关闭OLED显示
 * @return {*}
 */
void SPI_OLED_Display_Off(void)
{
	SPI_OLED_WR_CMD(0X8D);  //SET DCDC命令
	SPI_OLED_WR_CMD(0X10);  //DCDC OFF
	SPI_OLED_WR_CMD(0XAE);  //DISPLAY OFF，关闭显示
}
 
/**
 * @function: void OLED_Set_Pos(uint8_t x, uint8_t y)
 * @description: 坐标设置
 * @param {uint8_t} x,y
 * @return {*}
 */
void SPI_OLED_Set_Pos(uint8_t x, uint8_t y)
{
	SPI_OLED_WR_CMD(0xb0+y);	//设置页地址（0~7）
	SPI_OLED_WR_CMD(((x&0xf0)>>4)|0x10); //设置显示位置—列高地址
	SPI_OLED_WR_CMD(x&0x0f);	//设置显示位置—列低地址
}
 
 
/**
 * @function: unsigned int oled_pow(uint8_t m,uint8_t n)
 * @description: m^n函数
 * @param {uint8_t} m,n
 * @return {unsigned int} result
 */
unsigned int SPI_oled_pow(uint8_t m,uint8_t n)
{
	unsigned int result=1;
	while(n--)result*=m;
	return result;
}
 
/**
 * @function: void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size,uint8_t Color_Turn)
 * @description: 在OLED12864特定位置开始显示一个字符
 * @param {uint8_t} x字符开始显示的横坐标
 * @param {uint8_t} y字符开始显示的纵坐标
 * @param {uint8_t} chr待显示的字符
 * @param {uint8_t} Char_Size待显示字符的字体大小,选择字体 16/12
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void SPI_OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size,uint8_t Color_Turn)
{
	unsigned char c=0,i=0;
		c=chr-' ';//得到偏移后的值
		if(x>128-1){x=0;y=y+2;}
		if(Char_Size ==16)
		{
			SPI_OLED_Set_Pos(x,y);
			for(i=0;i<8;i++)
				{
				  if(Color_Turn)
					  SPI_OLED_WR_DATA(~F8X16[c*16+i]);
				  else
					  SPI_OLED_WR_DATA(F8X16[c*16+i]);
				}
			SPI_OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			    {
				  if(Color_Turn)
					  SPI_OLED_WR_DATA(~F8X16[c*16+i+8]);
				  else
					  SPI_OLED_WR_DATA(F8X16[c*16+i+8]);
			    }
 
			}
	     else
	     {
				SPI_OLED_Set_Pos(x,y);
				for(i=0;i<6;i++)
			    {
				  if(Color_Turn)
					  SPI_OLED_WR_DATA(~F6x8[c][i]);
				  else
					  SPI_OLED_WR_DATA(F6x8[c][i]);
			    }
		  }
}
 
/**
 * @function: void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_tChar_Size, uint8_t Color_Turn)
 * @description: 在OLED12864特定位置开始显示字符串
 * @param {uint8_t} x待显示字符串的开始横坐标x:0~127
 * @param {uint8_t} y待显示字符串的开始纵坐标 y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {uint8_t} *chr待显示的字符串
 * @param {uint8_t} Char_Size待显示字符串的字体大小,选择字体 16/12，16为8X16，12为6x8
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void SPI_OLED_ShowString(uint8_t x,uint8_t y,char *chr,uint8_t Char_Size, uint8_t Color_Turn)
{
	uint8_t  j=0;
	while (chr[j]!='\0')
	{		
		SPI_OLED_ShowChar(x,y,chr[j],Char_Size, Color_Turn);
			if (Char_Size == 12) //6X8的字体列加6，显示下一个字符
				x += 6;
			else  //8X16的字体列加8，显示下一个字符
				x += 8;
 
			if (x > 122 && Char_Size==12) //TextSize6x8如果一行不够显示了，从下一行继续显示
			{
				x = 0;
				y++;
			}
			if (x > 120 && Char_Size== 16) //TextSize8x16如果一行不够显示了，从下一行继续显示
			{
				x = 0;
				y++;
			}
			j++;
	}
}
 
/**
 * @function: void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2, Color_Turn)
 * @description: 显示数字
 * @param {uint8_t} x待显示的数字起始横坐标,x:0~126
 * @param {uint8_t} y待显示的数字起始纵坐标, y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {unsigned int} num:输入的数据
 * @param {uint8_t } len:输入的数据位数
 * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void SPI_OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2, uint8_t Color_Turn)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/SPI_oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				SPI_OLED_ShowChar(x+(size2/2)*t,y,' ',size2, Color_Turn);
				continue;
			}else enshow=1;
 
		}
	 	SPI_OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2, Color_Turn);
	}
}
 
 
/**
 * @function: void OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn)
 * @description: 显示正负浮点数
 * @param {uint8_t} x待显示的数字起始横坐标,x:0~126
 * @param {uint8_t} y待显示的数字起始纵坐标, y:0~7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {float} num:输入的浮点型数据
 * @param {uint8_t } z_ len:整数部分的位数
 * @param {uint8_t } f_len: 小数部分的位数
 * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void SPI_OLED_Showdecimal(uint8_t x,uint8_t y,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t Color_Turn)
{
	uint8_t t,temp,i=0;//i为负数标志位
	uint8_t enshow=0;
	int z_temp,f_temp;
	z_len+=1;
	if(num<0)
	{
		i=1;
		num=-num;
	}
	z_temp=(int)num;
	//整数部分
	for(t=0;t<z_len;t++)
	{
		temp=(z_temp/SPI_oled_pow(10,z_len-t-1))%10;
		if(enshow==0 && t<(z_len-1))
		{
			if(temp==0)
			{
				SPI_OLED_ShowChar(x+(size2/2)*t,y,' ',size2, Color_Turn);
				continue;
			}
			else
			enshow=1;
		}
		SPI_OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2, Color_Turn);
	}
	//小数点
	SPI_OLED_ShowChar(x+(size2/2)*(z_len),y,'.',size2, Color_Turn);
 
	f_temp=(int)((num-z_temp)*(SPI_oled_pow(10,f_len)));
  //小数部分
	for(t=0;t<f_len;t++)
	{
		temp=(f_temp/SPI_oled_pow(10,f_len-t-1))%10;
		SPI_OLED_ShowChar(x+(size2/2)*(t+z_len)+5,y,temp+'0',size2, Color_Turn);
	}
	if(i==1)//如果为负，就将最前的一位赋值‘-’
	{
		SPI_OLED_ShowChar(x,y,'-',size2, Color_Turn);
		i=0;
	}
}
 
 
 
/**
 * @function: void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no, uint8_t Color_Turn)
 * @description: 在OLED特定位置开始显示16X16汉字
 * @param {uint8_t} x待显示的汉字起始横坐标x: 0~112，两列汉字之间需要间隔16
 * @param {uint8_t} y待显示的汉字起始纵坐标 y: 0~6 , 两行汉字之间需要间隔2
 * @param {uint8_t} no待显示的汉字编号
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void SPI_OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no, uint8_t Color_Turn)
{
	uint8_t t=0;
	SPI_OLED_Set_Pos(x,y);
    for(t=0;t<16;t++)
		{
				if (Color_Turn)
					SPI_OLED_WR_DATA(~Hzk[2*no][t]); //显示汉字的上半部分
				else
					SPI_OLED_WR_DATA(Hzk[2*no][t]); //显示汉字的上半部分
        }
 
		SPI_OLED_Set_Pos(x,y+1);
    for(t=0;t<16;t++)
		{
				if (Color_Turn)
					SPI_OLED_WR_DATA(~Hzk[2*no+1][t]); //显示汉字的上半部分
				else
					SPI_OLED_WR_DATA(Hzk[2*no+1][t]);//显示汉字的上半部分
 
         }
}
 
/**
 * @function: void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t Color_Turn)
 * @description: 在OLED特定区域显示BMP图片
 * @param {uint8_t} x0图像开始显示横坐标  x0:0~127
 * @param {uint8_t} y0图像开始显示纵坐标  y0:0~7
 * @param {uint8_t} x1图像结束显示横坐标  x1:1~128
 * @param {uint8_t} y1图像结束显示纵坐标  y1:1~8
 * @param {uint8_t} *BMP待显示的图像数据
 * @param {uint8_t} Color_Turn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void SPI_OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t Color_Turn)
{
   uint32_t j = 0;
   uint8_t x = 0, y = 0;
 
  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		SPI_OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
		{
			if (Color_Turn)
				SPI_OLED_WR_DATA(~BMP[j++]);//显示反相图片
			else
				SPI_OLED_WR_DATA(BMP[j++]);//显示图片
 
		}
	}
}
 
/*
	@brief			显示动图
	@param			x0：起始列地址
				y0：起始页地址
				x1：终止列地址
				y1：终止页地址
				k: 帧个数
				m: 单帧数组大小
				BMP[][m]：存放动图代码的数组
	@retval			无
 */
void SPI_OLED_DrawGIF(uint8_t x0, uint8_t  y0, uint8_t  x1, uint8_t y1, uint8_t  k, int m, unsigned char GIF[][m])
{
	unsigned int j=0; //定义变量
 	unsigned char x,y,i; //定义变量
  
 	if(y1%8==0) y=y1/8;   //判断终止页是否为8的整数倍
 	 else y=y1/8+1;
	for (i=0;i<k;i++) //从第一帧开始画
	{
		j = 0;
		for(y=y0;y<y1;y++) //从起始页开始，画到终止页
		{
			SPI_OLED_Set_Pos(x0,y);//在页的起始列开始画
			
			for(x=x0;x<x1;x++) //画x1 - x0 列
	    		{
						SPI_OLED_WR_DATA(GIF[i][j++]);		//画图片的点  
	    		}
		}
		//delay_ms(80);//人为制造卡顿？？？
 
	}
}
 
 
 
/**
 * @function: void OLED_HorizontalShift(uint8_t direction)
 * @description: 屏幕内容水平全屏滚动播放
 * @param {uint8_t} direction			LEFT	   0x27     	RIGHT  0x26
 * @return {*}
 */
void SPI_OLED_HorizontalShift(uint8_t direction)
 
{
	SPI_OLED_WR_CMD(0x2e);//停止滚动
	SPI_OLED_WR_CMD(direction);//设置滚动方向
	SPI_OLED_WR_CMD(0x00);//虚拟字节设置，默认为0x00
	SPI_OLED_WR_CMD(0x00);//设置开始页地址
	SPI_OLED_WR_CMD(0x07);//设置每个滚动步骤之间的时间间隔的帧频
    //  0x00-5帧， 0x01-64帧， 0x02-128帧， 0x03-256帧， 0x04-3帧， 0x05-4帧， 0x06-25帧， 0x07-2帧，
	SPI_OLED_WR_CMD(0x07);//设置结束页地址
	SPI_OLED_WR_CMD(0x00);//虚拟字节设置，默认为0x00
	SPI_OLED_WR_CMD(0xff);//虚拟字节设置，默认为0xff
	SPI_OLED_WR_CMD(0x2f);//开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}
 
/**
 * @function: void OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end)
 * @description: 屏幕部分内容水平滚动播放
 * @param {uint8_t} direction			LEFT	   0x27     	RIGHT  0x26
 * @param {uint8_t} start 开始页地址  0x00~0x07
 * @param {uint8_t} end  结束页地址  0x01~0x07
 * @return {*}
 */
void SPI_OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end)
{
	SPI_OLED_WR_CMD(0x2e);//停止滚动
	SPI_OLED_WR_CMD(direction);//设置滚动方向
	SPI_OLED_WR_CMD(0x00);//虚拟字节设置，默认为0x00
	SPI_OLED_WR_CMD(start);//设置开始页地址
	SPI_OLED_WR_CMD(0x07);//设置每个滚动步骤之间的时间间隔的帧频,0x07即滚动速度2帧
	SPI_OLED_WR_CMD(end);//设置结束页地址
	SPI_OLED_WR_CMD(0x00);//虚拟字节设置，默认为0x00
	SPI_OLED_WR_CMD(0xff);//虚拟字节设置，默认为0xff
	SPI_OLED_WR_CMD(0x2f);//开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
 
}
 
/**
 * @function: void OLED_VerticalAndHorizontalShift(uint8_t direction)
 * @description: 屏幕内容垂直水平全屏滚动播放
 * @param {uint8_t} direction				右上滚动	 0x29
 *                                                            左上滚动   0x2A
 * @return {*}
 */
void SPI_OLED_VerticalAndHorizontalShift(uint8_t direction)
{
	SPI_OLED_WR_CMD(0x2e);//停止滚动
	SPI_OLED_WR_CMD(direction);//设置滚动方向
	SPI_OLED_WR_CMD(0x01);//虚拟字节设置
	SPI_OLED_WR_CMD(0x00);//设置开始页地址
	SPI_OLED_WR_CMD(0x07);//设置每个滚动步骤之间的时间间隔的帧频，即滚动速度
	SPI_OLED_WR_CMD(0x07);//设置结束页地址
	SPI_OLED_WR_CMD(0x01);//垂直滚动偏移量
	SPI_OLED_WR_CMD(0x00);//虚拟字节设置，默认为0x00
	SPI_OLED_WR_CMD(0xff);//虚拟字节设置，默认为0xff
	SPI_OLED_WR_CMD(0x2f);//开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}
 
/**
 * @function: void OLED_DisplayMode(uint8_t mode)
 * @description: 屏幕内容取反显示
 * @param {uint8_t} direction			ON	0xA7  ，
 *                                                          OFF	0xA6	默认此模式，设置像素点亮
 * @return {*}
 */
void SPI_OLED_DisplayMode(uint8_t mode)
{
	SPI_OLED_WR_CMD(mode);
}
 
/**
 * @function: void OLED_IntensityControl(uint8_t intensity)
 * @description: 屏幕亮度调节
 * @param  {uint8_t} intensity	0x00~0xFF,RESET=0x7F
 * @return {*}
 */
void SPI_OLED_IntensityControl(uint8_t intensity)
{
	SPI_OLED_WR_CMD(0x81);
	SPI_OLED_WR_CMD(intensity);
}

/**
 * @function: void System_message(char message[16])
 * @description: 系统消息
 * @param  char message[16]
 * @return {*}
 */
char message_1 [16];
char message_2 [16];
char message_3 [16];
uint16_t message_num = 0;	
void SPI_System_message(char message[16])
{
	if (message_num<3)
	  message_num++;
	
	  if (message_num == 1)
		{	    
      for(int i=0;i<16;i++)
      {
        message_1[i] = message[i];
      }
		}
		else if (message_num == 2)
		{	    
      for(int i=0;i<16;i++)
      {
        message_2[i] = message[i];
      }
		}
		else if (message_num == 3)
		{	    
      for(int i=0;i<16;i++)
      {
        message_3[i] = message[i];
      }
		}
		else
	  {
      for(int i=0;i<16;i++)
      {
        message_1[i] = message_2[i];
      }
      for(int i=0;i<16;i++)
      {
        message_2[i] = message_3[i];
      }
      for(int i=0;i<16;i++)
      {
        message_3[i] = message[i];
      }
		}
		SPI_OLED_ShowString(8,2,(char *)message_1,16,0);
		SPI_OLED_ShowString(8,4,(char *)message_2,16,0);
		SPI_OLED_ShowString(8,6,(char *)message_3,16,0);
}
