#include "position.h"

/**
 * @brief 数据解析函数  如更换MCU平台或更换软件库，只需将串口接收到的值传入该函数即可解析
 * @param  rec 串口接收到的字节数据
 */
const uint8_t reset[]="ACT0";
 
//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 定位系统DMA方式接收解包
//  @return     		 /
//  @param					 uint8_t pos_data[32]  传入接收到的数据包
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void POS_Data_Unpack(uint8_t pos_data[32])
{
	if (pos_data[0]==0x0d && pos_data[1]==0x0a && pos_data[26]==0x0a && pos_data[27]==0x0d)
  {
  	static union
  	{
	  	uint8_t date[24];
  		float ActVal[6];
  	}posture;
		
		memcpy(&posture.date, pos_data+2, 24);
  
	  zangle = posture.ActVal[0];
	  xangle = posture.ActVal[1];
	  yangle = posture.ActVal[2];
	  pos_x = posture.ActVal[3];
	  pos_y = posture.ActVal[4];
	  w_z =posture.ActVal[5];
  }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 坐标置零
//  @return     		 /
//  @param					 /
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void pos_reset(void)
{
  HAL_UART_Transmit(&huart2,reset,sizeof(reset),100);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 字符串拼接
//  @return     		 /
//  @param					 char strDestination[]  原字符串
//  @param					 char strSource[]       尾部加入字符串
//  @param					 int num                加入长度
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void Stract(char strDestination[], char strSource[], int num)
{
  int i = 0,j = 0;
  while(strDestination[i]!='\0')  i++;
  for(j = 0;j < num;j++)
  strDestination[i++] = strSource[j];
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 更新OPS坐标
//  @return     		 /
//  @param					 float New_X  需上传的X坐标
//  @param					 float New_Y  需上传的Y坐标
//  @param					 float New_Z  需上传的Z坐标
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void Update_OPS(float New_X, float New_Y, float New_Z)
{
	osDelay(5);
  Update_X(New_X);
	osDelay(5);
	Update_Y(New_Y);
	osDelay(5);
	Update_Z(New_Z);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 更新X坐标
//  @return     		 /
//  @param					 float New_X  需上传的X坐标
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void Update_X(float New_X)
{
	int i = 0;
	char Update_x[8] = "ACTX";
	static union
		{
			float X;
			char data[4];
		}New_set;
		New_set.X = New_X;
		Stract(Update_x,New_set.data,4);
		for(i = 0;i < 8;i++)
		{  
			HAL_UART_Transmit(&huart2,(const uint8_t *)&Update_x[i],1,100);
		}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 更新Y坐标
//  @return     		 /
//  @param					 float New_Y  需上传的Y坐标
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void Update_Y(float New_Y)
{
	int i = 0;
	char Update_y[8] = "ACTY";
	static union
		{
			float Y;
			char data[4];
		}New_set;
		New_set.Y = New_Y;
		Stract(Update_y,New_set.data,4);
		for(i = 0;i < 8;i++)
		{  
			HAL_UART_Transmit(&huart2,(const uint8_t *)&Update_y[i],1,100);
		}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 更新Z坐标
//  @return     		 /
//  @param					 float New_Z  需上传的Z坐标
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void Update_Z(float New_Z)
{
	int i = 0;
	char Update_z[8] = "ACTJ";
	static union
		{
			float Z;
			char data[4];
		}New_set;
		New_set.Z = New_Z;
		Stract(Update_z,New_set.data,4);
		for(i = 0;i < 8;i++)
		{  
			HAL_UART_Transmit(&huart2,(const uint8_t *)&Update_z[i],1,100);
		}
}
