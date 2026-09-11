/**
  ******************************************************************************
  * @author  PINK SAKURA
  * @version V1.0
  * @date    2024-08-08
  * @brief   底盘控制 C 文件
  ******************************************************************************
  * @attention
  * 邮箱：sakura.mail@qq.com
  ******************************************************************************
  */

#include "chassis_control.h"
#include "zdt_motor.h"
#include "dynamic_map_link.h"

extern uint8_t control_mode;
static int16_t motor_batch_x, motor_batch_y, motor_batch_z;
static uint8_t motor_batch_mode;

static uint8_t Chassis_MotorBatchValid(void)
{
    int16_t x, y, z;
    DynamicMap_Link_Process(); /* STOP/expiry can arrive during four-wheel staging. */
    if(control_mode != motor_batch_mode) return 0U;
    if(control_mode != 2U) return 1U;
    return DynamicMap_Link_GetVelocity(&x, &y, &z) &&
           x == motor_batch_x && y == motor_batch_y && z == motor_batch_z;
}

float rcKpx	= 8, rcKpy = 8, rcKpz = 6;       //move函数Kp
float mKpx	= 2.3, mKpy = 2.3, mKpz = 9;     //move函数Kp
float vKpx	= 1.2, vKpy =1.2, vKpz = 2.4;    //调整函数Kp
float cvKpz	= 0.08;    //z轴调整函数Kp
int last_Speed[4]={0};

float XYVmax;
float ZVmax;
float XYVmin;
float ZVmin;

uint8_t in_pos = 1;      //就位指示
uint8_t v_in_pos = 1;    //视觉就位指示
uint8_t near_pos = 1;    //接近指示
uint8_t delay_pos=0;
uint8_t delay_turn=0;
uint8_t delay_visual=0;
	
	float devx=0;
	float devy=0;
	float devz=0;

void SpeedTarget_stop(void)  //电机停转
{
	SpeedTarget[0]=0;
	SpeedTarget[1]=0;
	SpeedTarget[2]=0;
	SpeedTarget[3]=0;
	/* A later move must not reuse the pre-stop acceleration history. */
	for(uint8_t i=0U; i<4U; ++i) last_Speed[i]=0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 电机转速和方向控制函数
//  @return     		 /
//  @param					 int MotorSpeed1  电机1目标速度
//  @param					 int MotorSpeed2  电机2目标速度
//  @param					 int MotorSpeed3  电机3目标速度
//  @param					 int MotorSpeed4  电机4目标速度
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void SetMotorVoltageAndDirection(int MotorSpeed1, int MotorSpeed2, int MotorSpeed3, int MotorSpeed4)  //电机转速和方向控制函数
{
    const int rpm[4] = {MotorSpeed1, MotorSpeed2, MotorSpeed3, MotorSpeed4};
    motor_batch_mode = control_mode;
    (void)DynamicMap_Link_GetVelocity(&motor_batch_x, &motor_batch_y, &motor_batch_z);
    if(!ZdtMotor_SetSpeeds(rpm, Chassis_MotorBatchValid)) {
        SpeedTarget_stop();
        DynamicMap_Link_ReportMotorFault();
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 速度用限幅
//  @return     		 /
//  @param					 float* value  输入值指针
//  @param					 float max     最大值
//  @param					 float min     最小值
//  @param					 float dead_zone     死区/不响应区域（防止抖动）
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void numerical_limit(float* value, float max, float min , float dead_zone)
{
	if (*value > dead_zone)
		*value += min;
	else if (*value < -dead_zone)
	  *value -= min;
	if (*value > max)
		*value = max;
	else if (*value <- max)
	  *value = -max;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 底盘遥控函数
//  @return     		 /
//  @param					 short x  X轴速度
//  @param					 short y  Y轴速度
//  @param					 short z  Z轴速度
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void chassis_RC(short x,short y,short z)      //底盘遥控函数
{
  XYVmax=800;
	ZVmax=600;
	
//	  float length = sqrt( x * x + y * y );      //归一化处理
//       x = ( x / length ) * 100;
//       y = ( y / length ) * 100;

		int Speed[4]={0};
		devx = x;
		devy = y;
		devz = z;
		
		float vx1 = 0,vy1 = 0,vz = 0;

		vx1 =1 * rcKpx * devx;
		numerical_limit(&vx1, XYVmax, 0, 5);
			
		vy1 =1 * rcKpy * devy;
		numerical_limit(&vy1, XYVmax, 0, 5);

		vz = rcKpz * devz;
		numerical_limit(&vz, ZVmax, 0, 5);

		float v_ratio = 1;
			
    Speed[0] =   ( ( vy1 + vx1 ) * v_ratio + vz);
  	Speed[1] = - ( ( vy1 - vx1 ) * v_ratio - vz);
  	Speed[2] =   ( ( vy1 - vx1 ) * v_ratio + vz);
  	Speed[3] = - ( ( vy1 + vx1 ) * v_ratio - vz);

    for(uint8_t i=0;i<4;i++)       //速度提交
	  {
			SpeedTarget[i]=(int)(Speed[i] * 0.238);
	  }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 底盘定位系统位置环
//  @return     		 /
//  @param					 int x  X轴目标位置
//  @param					 int y  Y轴目标位置
//  @param					 int z  Z轴目标位置
//  @Sample usage:   略
//-------------------------------------------------------------------------------------------------------------------
void chassis_move(int x,int y,int z)      //底盘移动函数
{
  XYVmax=1600;
	ZVmax=750;
	XYVmin=5;
	ZVmin=5;

		int Speed[4]={0};
	  devx=pos_x-x;
	  devy=pos_y-y;
	  if(z*zangle<0&&abs(z)+fabs(zangle)>180)
      devz=-(360-abs(z)-fabs(zangle));
		else 
      devz=z-zangle;
		
		float vx1 = 0,vy1 = 0,vx2 = 0,vy2 = 0,vz = 0;
		
		vy1 =cos(zangle*3.141f/180) * mKpy * devy;
		vx2 =sin(zangle*3.141f/180) * mKpy * devy;
		numerical_limit(&vy1, XYVmax, XYVmin, 5);
		numerical_limit(&vx2, XYVmax, XYVmin, 5);
			
		vy2 =sin(zangle*3.141f/180) * mKpx * devx;
		vx1 =cos(zangle*3.141f/180) * mKpx * devx;
		numerical_limit(&vy2, XYVmax, XYVmin, 5);
		numerical_limit(&vx1, XYVmax, XYVmin, 5);

		vz = mKpz * devz;
    numerical_limit(&vz, ZVmax, 0, 5);
	
    Speed[0] = - (  vy1  -vy2  +  vx1  +vx2  +  vz);
  	Speed[1] =   (  vy1  -vy2  -  vx1  -vx2  -  vz);
  	Speed[2] = - (  vy1  -vy2  -  vx1  -vx2  +  vz);
  	Speed[3] =   (  vy1  -vy2  +  vx1  +vx2  -  vz);

    for(uint8_t i=0;i<4;i++)       //速度提交
	  {
			if (Speed[i]>0&&Speed[i]>last_Speed[i])
				Speed[i]=last_Speed[i]+20;
			if (Speed[i]<0&&Speed[i]<last_Speed[i])
				Speed[i]=last_Speed[i]-20;
			SpeedTarget[i]=(int)(Speed[i] * 0.238);
			last_Speed[i]=Speed[i];
	  }
		if((devx<100&&devx>-100)&&(devy<100&&devy>-100)&&(devz<30&&devz>-30))
			near_pos = 1;
    if((devx<60&&devx>-60)&&(devy<60&&devy>-60)&&(devz<15&&devz>-15))
			delay_pos++;
		else
			delay_pos=0;
		
		if (delay_pos>10)
			in_pos = 1;
}


void visual_pos_adj(uint8_t target,int x,int y,int z)      //底盘视觉微调函数（目标号，目标x，目标y，保持角度（物料盘使用陀螺仪角度））
{
	XYVmax=80;
	ZVmax=35;
	XYVmin=3;
	ZVmin=2;

		int Speed[4]={0};
		
		switch(target)    //目标坐标处理
		{
    case 1:  //物料盘
		    devy = VIS_RX.tray_center_x - x;
	      devx = VIS_RX.tray_center_y - y;
        break;
    case 2:  //色环
	      devy = VIS_RX.rgb_circle_x - x;
	      devx = VIS_RX.rgb_circle_y - y;
        break;
    case 3:  //码垛时色环
	      devy = VIS_RX.rgb_circle_x - x;
	      devx = VIS_RX.rgb_circle_y - y;
        break;
		case 4:  //色环粗校准
	      devy = VIS_RX.rgb_circle_x - x;
	      devx = VIS_RX.rgb_circle_y - y;
        break;
	  }
		
		if (target>=2)
      devz = VIS_RX.rgb_circle_z - z;
		else 
		{
			if(z*zangle<0&&abs(z)+fabs(zangle)>180)
        devz=-(360-abs(z)-fabs(zangle));
		  else 
        devz=z-zangle;
		}
		
		float vx1 = 0,vy1 = 0,vx2 = 0,vy2 = 0,vz = 0;
		
		vy1 = vKpy * devy;
		numerical_limit(&vy1, XYVmax, XYVmin, 0);
		numerical_limit(&vx2, XYVmax, XYVmin, 0);

		vx1 = vKpx * devx;
		numerical_limit(&vy2, XYVmax, XYVmin, 0);
		numerical_limit(&vx1, XYVmax, XYVmin, 0);

		vz = vKpz * devz;
    numerical_limit(&vz, ZVmax, 0, 0);
	
    Speed[0] = - (  vy1  -vy2  +  vx1  +vx2  +  vz);
  	Speed[1] =   (  vy1  -vy2  -  vx1  -vx2  -  vz);
  	Speed[2] = - (  vy1  -vy2  -  vx1  -vx2  +  vz);
  	Speed[3] =   (  vy1  -vy2  +  vx1  +vx2  -  vz);

    for(uint8_t i=0;i<4;i++)       //速度提交
	  {
			if (Speed[i]>0&&Speed[i]>last_Speed[i])
				Speed[i]=last_Speed[i]+20;
			if (Speed[i]<0&&Speed[i]<last_Speed[i])
				Speed[i]=last_Speed[i]-20;
			SpeedTarget[i]=(int)(Speed[i] * 0.238);
			last_Speed[i]=Speed[i];
	  }
		switch(target)    //目标坐标处理
		{
			case 1:  //物料盘
				if((devx<4&&devx>-4)&&(devy<4&&devy>-4)&&(devz<2&&devz>-2))
					delay_visual+=2;
				else
					delay_visual=0;
			break;
			case 2:  //色环
				if((devx<2&&devx>-2)&&(devy<2&&devy>-2)&&(devz<2&&devz>-2))
					delay_visual++;
				else
					delay_visual=0;
			break;
			case 3:  //码垛时色环
				if((devx<4&&devx>-4)&&(devy<4&&devy>-4)&&(devz<3&&devz>-3))
					delay_visual+=2;
				else
					delay_visual=0;
			break;
			case 4:  //色环粗校准
				if((devx<10&devx>-10)&&(devy<10&&devy>-10)&&(devz<6&&devz>-6))
					delay_visual+=20;
				else
					delay_visual=0;
			break;
	  }
		
		if (delay_visual>200)
			v_in_pos = 1;
}
