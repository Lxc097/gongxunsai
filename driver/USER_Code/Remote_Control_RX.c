#include "Remote_Control_RX.h"
#include "dynamic_map_link.h"

uint8_t count=0,data[10];

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 遥控器DMA方式解包
//  @return     		 /
//  @param					 uint8_t rc_data[16]  传入接收到的数据包
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void RC_Data_Unpack(uint8_t rc_data[16])
{
#if DYNAMIC_MAP_NAVIGATION_ONLY
    (void)rc_data;
    return; /* No remote mode takeover, reset, or readiness bypass. */
#else
    	//if ((data[1] + data[2] + data[3] + data[4] + data[5]) == data[9])
    	//和校验：计算所有数据帧之和
    	if (rc_data[0]==0xFA && rc_data[9]==0xFB)                     //校验帧头帧尾
  	  {
	    	RC_RX.LX=rc_data[1];                  //左摇杆 X
		  	RC_RX.LY=-rc_data[2];                 //左摇杆 Y
		  	RC_RX.RX=rc_data[3];                    //右摇杆 X
		  	RC_RX.RY=-rc_data[4];                 //右摇杆 Y
		  	RC_RX.KNOB_L=rc_data[5];                //左旋钮
	  		RC_RX.KNOB_R=rc_data[6];                //右旋钮
		  	RC_RX.KEY_L1=rc_data[7] >> 3 & 1;       //按键 L1
		  	RC_RX.KEY_L2=rc_data[7] >> 2 & 1;	     //按键 L2
			  RC_RX.KEY_R1=rc_data[7] >> 1 & 1;       //按键 R1
			  RC_RX.KEY_R2=rc_data[7] & 1;            //按键 R2
		  	RC_RX.TOG_L1=rc_data[8] >> 3 & 1;       //拨杆 L1
			  RC_RX.TOG_L2=rc_data[8] >> 2 & 1;       //拨杆 L2
		  	RC_RX.TOG_R1=rc_data[8] >> 1 & 1;       //拨杆 R1
		  	RC_RX.TOG_R2=rc_data[8] & 1;            //拨杆 R2
				
				control_mode = RC_RX.TOG_L1 ? 1 : 0;		//手动接管
				if (RC_RX.KEY_L1)  NVIC_SystemReset();  //远程复位
				if (RC_RX.KEY_L2)  ready=1;             //跳过初始化
	  	}         	                
#endif
}
