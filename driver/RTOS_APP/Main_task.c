/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   主任务
  ******************************************************************************
  * @attention
  * 邮箱：sakura.mail@qq.com
  ******************************************************************************
  */

#include "Main_task.h"
#include "dynamic_map_link.h"

//	V2.6.1 解决了位置环抖动问题
//				 删除无效代码
//				 改变了限幅
//         动作时间微调
//	V2.6.2 重调塔吊
//				 优化出界问题
//				 提速至2分15秒
//         动作时间微调
//	V2.6.3 加入决赛代码
//	V2.6.4 决赛抓取物料
//	V2.6.5 扫码超时处理
//         变量整理
//         决赛物料台放置
//  V2.7.4 加入二次校准
//         决赛夹爪规划补偿


 //LED_1  绿色  NANO已就位
 //LED_2  蓝色  底盘视觉矫正
 //LED_3  红色  塔吊运行

uint8_t GET_TASK_FLAGE = 0;
uint8_t GET_TASK_TIME_OUT = 0;

void Main_init(void)
{
	// OLED初始化
	SoftSPI_OLED_Init();
	// 清屏
	SoftSPI_OLED_Clear();
	// 设置显示方向（可选）
	SoftSPI_OLED_DisplayTurn(0); // 0-正常显示，1-旋转180度 
	visual_idle();
	pos_reset();
}

void Get_Task_Code(void)
{
	for(uint8_t i=0;i<6;i++)
	  color_task[i]=0;
	GET_TASK_FLAGE = 1;
	while(color_task[0] + color_task[5] == 0)  
  {
		identify_QR_code();    //识别二维码
		osDelay(50); 
	  GET_TASK_TIME_OUT++;
		if (GET_TASK_TIME_OUT >= 80)  //超时处理
		{
			if (d_X == 0 || d_X == -30)
			  d_X = 30;
			else 
				d_X = -30;
			GET_TASK_TIME_OUT = 0;
		}
	}
	GET_TASK_FLAGE = 0;
	visual_idle();
	Chassis_d_zero();
}

void Show_Task_Code(void)
{
	SoftSPI_OLED_Clear();
	for(uint8_t i=0;i<3;i++)
	{
		SoftSPI_OLED_ShowBN(8+i*16,16, color_task[i], 1);
	}
	SoftSPI_OLED_ShowBN(56,16, 0, 1);
	for(uint8_t i=3;i<6;i++)
	{
		SoftSPI_OLED_ShowBN(24+i*16,16, color_task[i], 1);
	}
	SoftSPI_OLED_Refresh();
}

void CRing_Dual_Calibration()
{
  	Chassis_Visual_Pos(4, 320, 222, 4, 0);   //底盘视觉微调函数（目标号，目标x，目标y，保持角度） 正-左小右大
		
		identify_posture(1500, 1200);  //识别姿态
		osDelay(100);
		
		Chassis_Visual_Pos(2, 322, 228, 4, 0);   //底盘视觉微调函数（目标号，目标x，目标y，保持角度） x增大车往左 y增大车往前 正-左小右大
		
		identify_posture(1500, 1700);  //识别姿态
		osDelay(200);
}

void Main_task(void)
{
#if DYNAMIC_MAP_NAVIGATION_ONLY
	/* Do not run the old six-digit route or issue any arm commands. */
	while(1) { osDelay(100); }
#else
	osDelay(100);
	Main_init();
	
//	while(1)  //舵机测试
//	{
//	  while(!(HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin))) {osDelay(50);}
//	  LFD_01M_OP_C;    //夹爪复位
//		osDelay(200);
//		while(!(HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin))) {osDelay(50);}
//		LFD_01M_CL;    //夹爪关闭
//		osDelay(200);
//	}
//	while(1){osDelay(50);};
	
  SoftSPI_OLED_ShowString(16,0,(uint8_t *)"SW Ver 2.7.4",16,1);
  SoftSPI_OLED_ShowString(8,16,(uint8_t *)"OLED INTI PASS",16,1);
	SoftSPI_OLED_Refresh();
	while(!ready) { osDelay(10);}
	osDelay(100);
	buzzer_rings(100, 0, 1);
	
	SoftSPI_OLED_Clear();

	pos_reset();
	
	while(!(HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin))) {osDelay(50);}
	
	chassis_mode=1;    //底盘模式
	
	#ifdef __PRELIMINARY_ROUND__
	
	while(1)
	{	
//		color_task[0]=1;
//		color_task[1]=3;
//		color_task[2]=2;

//		identify_posture(1500, 1700);  //识别姿态
//		osDelay(500);
//		
//		CRing_Dual_Calibration();  //色环双重校准

//			grab_materials_car(1, 0);     //车上抓取物料  地面放置物料
//			put_materials_ground(color_task[0],0);
//			grab_materials_car(2, 0);     //车上抓取物料
//			put_materials_ground(color_task[1],0);
//			grab_materials_car(3, 0);     //车上抓取物料
//			put_materials_ground(color_task[2],0);
//			reset_posture();	
//		while(1){osDelay(10);};
		
    Chassis_Go_Pos(150, 165, -45, 0, 300);  //出库
		
		reset_posture();
		
		Chassis_Go_Pos(730, 130, -90, 0, 10);  //扫码
		
		Get_Task_Code();
    Show_Task_Code();
		
		/* 第一轮 */
		
		Chassis_Go_Pos(1450, 110, -90, 0, 100);  //物料台
		
			grab_turntable_B(color_task[0],1);
			grab_turntable_B(color_task[1],2);
			grab_turntable_B(color_task[2],3);
		
		Chassis_Go_Pos(1050, 165, 0, 1, 300);  //T1路口
		
		Chassis_Go_Pos(1050, 1900, 0, 1, 300);  //粗加工区1
		
		Chassis_Go_Pos(1050, 1900, 90, 1, 50);  //组加工区2
		
		identify_posture(1500, 1700);  //识别姿态

		Chassis_Go_Pos(1000, 1950, 90, 0, 300);
		
		CRing_Dual_Calibration();  //色环双重校准
		
			grab_materials_car(1, 0);     //车上抓取物料  地面放置物料
			put_materials_ground(color_task[0],0);
			grab_materials_car(2, 0);     //车上抓取物料
			put_materials_ground(color_task[1],0);
			grab_materials_car(3, 0);     //车上抓取物料
			put_materials_ground(color_task[2],0);
			reset_posture();
		
			grab_materials_ground(color_task[0],1);     //地面抓取物料
			put_materials_car(1);
			grab_materials_ground(color_task[1],0);     //地面抓取物料
			put_materials_car(2);
			grab_materials_ground(color_task[2],0);     //地面抓取物料
			put_materials_car(3);
		
		Chassis_Go_Pos(1000, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 90, 1, 300);
				
		Chassis_Go_Pos(1860, 1880, 0, 1, 300);
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1900, 1020, 0, 0, 100);
			
		CRing_Dual_Calibration();  //色环双重校准
		Update_OPS(1940, 1050, 0);

			grab_materials_car(1, 0);     //车上抓取物料  地面放置物料
			put_materials_ground(color_task[0],0);
			grab_materials_car(2, 0);     //车上抓取物料
			put_materials_ground(color_task[1],0);
			grab_materials_car(3, 0);     //车上抓取物料
			put_materials_ground(color_task[2],0);
			reset_posture();
		
		Chassis_Go_Pos(1900, 165, -90, 1, 300);
		
		/* 第二轮 */
		
		Chassis_Go_Pos(1450, 120, -90, 0, 100);  //物料台
		
			grab_turntable_B(color_task[3],1);
			grab_turntable_B(color_task[4],2);
			grab_turntable_B(color_task[5],3);	
				
		Chassis_Go_Pos(1050, 165, 0, 1, 300);  //T1路口
		
		Chassis_Go_Pos(1050, 1900, 0, 1, 300);  //粗加工区1
		
		Chassis_Go_Pos(1050, 1900, 90, 1, 200);  //组加工区2
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1000, 1950, 90, 0, 300);
		
		CRing_Dual_Calibration();  //色环双重校准
		
			grab_materials_car(1, 0);     //车上抓取物料  地面放置物料
			put_materials_ground(color_task[3],0);
			grab_materials_car(2, 0);     //车上抓取物料
			put_materials_ground(color_task[4],0);
			grab_materials_car(3, 0);     //车上抓取物料
			put_materials_ground(color_task[5],0);
		
			grab_materials_ground(color_task[3],1);     //地面抓取物料
			put_materials_car(1);
			grab_materials_ground(color_task[4],0);     //地面抓取物料
			put_materials_car(2);
			grab_materials_ground(color_task[5],0);     //地面抓取物料
			put_materials_car(3);
		
		Chassis_Go_Pos(1000, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 0, 1, 300);
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1900, 1040, 0, 0, 100);
			
		Chassis_Visual_Pos(3, 328, 230, 6, 0);    //底盘视觉微调函数（目标号，目标x，目标y，保持角度）
		Update_OPS(1950, 1050, 0);

			grab_materials_car(1, 0);     //车上抓取物料  码垛放置物料
			put_materials_ground(color_task[3],1);
			grab_materials_car(2, 0);     //车上抓取物料
			put_materials_ground(color_task[4],1);
			grab_materials_car(3, 0);     //车上抓取物料
			put_materials_ground(color_task[5],1);
			reset_posture();
		
		Chassis_Go_Pos(1900, 165, -90, 1, 300);
		
		Chassis_Go_Pos(200, 165, -90, 0, 300);
		
		Chassis_Go_Pos(0, 0, 0, 0, 3000);
		
		chassis_mode=2;    //底盘模式
	
		while(1){osDelay(100);}
		
	}
	
	#else
	
	while(1)
	{		
	  uint8_t block_pos[3] = {0};

    Chassis_Go_Pos(150, 165, -45, 0, 300);  //出库
		
		reset_posture();
		
		Chassis_Go_Pos(730, 135, -90, 0, 10);  //扫码
		
		Get_Task_Code();
    Show_Task_Code();
		
		/* 第一轮 */
		
		Chassis_Go_Pos(1050, 165, 0, 1, 300);  //T1路口
		
		Chassis_Go_Pos(1050, 1900, 0, 1, 300);  //粗加工区1
		
		Chassis_Go_Pos(1050, 1900, 90, 1, 200);  //组加工区2
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1000, 1950, 90, 0, 300);
		
		Chassis_Visual_Pos(3, 328, 230, 6, 0);    //底盘视觉微调函数（目标号，目标x，目标y，保持角度）
		
		switch(VIS_RX.block_order)  //[颜色]对应色环号
		{
			case 1:  block_pos[0] = 3; block_pos[1] = 2; block_pos[2] = 1; break;
			case 2:  block_pos[0] = 3; block_pos[1] = 1; block_pos[2] = 2; break;
			case 3:  block_pos[0] = 2; block_pos[1] = 3; block_pos[2] = 1; break;
			case 4:  block_pos[0] = 1; block_pos[1] = 3; block_pos[2] = 2; break;
			case 5:  block_pos[0] = 2; block_pos[1] = 1; block_pos[2] = 3; break;
			case 6:  block_pos[0] = 1; block_pos[1] = 2; block_pos[2] = 3; break;
		}
		
			grab_materials_ground(block_pos[color_task[0]-1],1);     //地面抓取物料
			put_materials_car(1);
			grab_materials_ground(block_pos[color_task[1]-1],0);     //地面抓取物料
			put_materials_car(2);
			grab_materials_ground(block_pos[color_task[2]-1],0);     //地面抓取物料
			put_materials_car(3);
			
		Chassis_Go_Pos(1000, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 0, 1, 300);
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1900, 1040, 0, 0, 100);
			
		CRing_Dual_Calibration();  //色环双重校准
		Update_OPS(1940, 1050, 0);
		
			grab_materials_car(1, 0);     //车上抓取物料  地面放置物料
			put_materials_ground(color_task[0],0);
			grab_materials_car(2, 0);     //车上抓取物料
			put_materials_ground(color_task[1],0);
			grab_materials_car(3, 0);     //车上抓取物料
			put_materials_ground(color_task[2],0);
			reset_posture();
		
			grab_materials_ground(color_task[0],1);     //地面抓取物料
			put_materials_car(1);
			grab_materials_ground(color_task[1],0);     //地面抓取物料
			put_materials_car(2);
			grab_materials_ground(color_task[2],0);     //地面抓取物料
			put_materials_car(3);
		
		Chassis_Go_Pos(1900, 165, -90, 1, 300);

/////////////////////////////////////////////////////////////////////////////////////////
//		color_task[0]=1;
//		color_task[1]=2;
//		color_task[2]=3;
//		color_task[3]=3;
//		color_task[4]=2;
//		color_task[5]=1;
//    Show_Task_Code();
/////////////////////////////////////////////////////////////////////////////////////////
		
		identify_posture(2100, 2000);     //识别姿态
		
		Chassis_Go_Pos(1450, 120, -90, 0, 100);  //物料台
		
		Chassis_Visual_Pos(1, 313, 190, -90, 0);    //底盘视觉微调函数（目标号，目标x，目标y，保持角度）
		
		put_block_turntable(1, color_task[0]);     //决赛物料台放置
		put_block_turntable(2, color_task[1]);     //决赛物料台放置
		put_block_turntable(3, color_task[2]);     //决赛物料台放置
		reset_posture();
		
		/* 第二轮 */
		
		Chassis_Go_Pos(1050, 165, 0, 1, 300);  //T1路口
		
		Chassis_Go_Pos(1050, 1900, 0, 1, 300);  //粗加工区1
		
		Chassis_Go_Pos(1050, 1900, 90, 1, 200);  //组加工区2
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1000, 1950, 90, 0, 300);
		
		Chassis_Visual_Pos(3, 328, 230, 6, 0);    //底盘视觉微调函数（目标号，目标x，目标y，保持角度）
		
		switch(VIS_RX.block_order)  //[颜色]对应色环号
		{
			case 1:  block_pos[0] = 3; block_pos[1] = 2; block_pos[2] = 1; break;
			case 2:  block_pos[0] = 3; block_pos[1] = 1; block_pos[2] = 2; break;
			case 3:  block_pos[0] = 2; block_pos[1] = 3; block_pos[2] = 1; break;
			case 4:  block_pos[0] = 1; block_pos[1] = 3; block_pos[2] = 2; break;
			case 5:  block_pos[0] = 2; block_pos[1] = 1; block_pos[2] = 3; break;
			case 6:  block_pos[0] = 1; block_pos[1] = 2; block_pos[2] = 3; break;
		}
		
			grab_materials_ground(block_pos[color_task[3]-1],1);     //地面抓取物料
			put_materials_car(1);
			grab_materials_ground(block_pos[color_task[4]-1],0);     //地面抓取物料
			put_materials_car(2);
			grab_materials_ground(block_pos[color_task[5]-1],0);     //地面抓取物料
			put_materials_car(3);
			
		Chassis_Go_Pos(1000, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 90, 1, 300);
		
		Chassis_Go_Pos(1860, 1880, 0, 1, 300);
		
		identify_posture(1500, 1700);  //识别姿态
		
		Chassis_Go_Pos(1900, 1040, 0, 0, 100);
			
		CRing_Dual_Calibration();  //色环双重校准
		Update_OPS(1950, 1050, 0);
		
			grab_materials_car(1, 0);     //车上抓取物料  地面放置物料
			put_materials_ground(color_task[3],0);
			grab_materials_car(2, 0);     //车上抓取物料
			put_materials_ground(color_task[4],0);
			grab_materials_car(3, 0);     //车上抓取物料
			put_materials_ground(color_task[5],0);
			reset_posture();
		
			grab_materials_ground(color_task[3],1);     //地面抓取物料
			put_materials_car(1);
			grab_materials_ground(color_task[4],0);     //地面抓取物料
			put_materials_car(2);
			grab_materials_ground(color_task[5],0);     //地面抓取物料
			put_materials_car(3);
		
		Chassis_Go_Pos(1900, 165, -90, 1, 300);

/////////////////////////////////////////////////////////////////////////////////////////
//		color_task[0]=1;
//		color_task[1]=2;
//		color_task[2]=3;
//		color_task[3]=3;
//		color_task[4]=2;
//		color_task[5]=1;
//    Show_Task_Code();
/////////////////////////////////////////////////////////////////////////////////////////
		
		identify_posture(2100, 2000);     //识别姿态
		
		Chassis_Go_Pos(1450, 120, -90, 0, 100);  //物料台
		
		Chassis_Visual_Pos(1, 310, 190, -90, 0);    //底盘视觉微调函数（目标号，目标x，目标y，保持角度）
		
		put_block_turntable(1, color_task[3]);     //决赛物料台放置
		put_block_turntable(2, color_task[4]);     //决赛物料台放置
		put_block_turntable(3, color_task[5]);     //决赛物料台放置
		reset_posture();
		
		Chassis_Go_Pos(200, 165, -90, 0, 300);
		
		Chassis_Go_Pos(0, 0, 0, 0, 3000);
		
		chassis_mode=2;    //底盘模式
	
		while(1){osDelay(100);}
		
	}
	#endif
#endif
}
