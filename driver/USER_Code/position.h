#include "main.h"
#include "usart.h"
#include <string.h>

extern float pos_x;
extern float pos_y;
extern float zangle;
extern float xangle;
extern float yangle;
extern float w_z;

/**
 * @brief 数据解析函数  如更换MCU平台或更换软件库，只需将串口接收到的值传入该函数即可解析
 * @param  rec 串口接收到的字节数据
 */

void POS_Data_Unpack(uint8_t rec[32]);
/*更新OPS坐标*/
void Update_OPS(float New_X, float New_Y, float New_Z);
/*坐标清零*/
void pos_reset(void);

/*更新X坐标*/
void Update_X(float New_X);
/*更新Y坐标*/
void Update_Y(float New_Y);
/*更新Z坐标*/
void Update_Z(float New_Z);
