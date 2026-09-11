#include "OLED_SoftSPI.h"
#include "ZJY_oledfont.h"
#include "main.h"

// 全局显存（128x64分辨率，按列存储）
uint8_t SoftSPI_OLED_GRAM[144][8];

// 反显函数
void SoftSPI_OLED_ColorTurn(uint8_t i)
{
    if(i == 0)
    {
        SoftSPI_OLED_WR_Byte(0xA6, SOFTSPI_OLED_CMD); // 正常显示
    }
    if(i == 1)
    {
        SoftSPI_OLED_WR_Byte(0xA7, SOFTSPI_OLED_CMD); // 反色显示
    }
}

// 屏幕旋转180度
void SoftSPI_OLED_DisplayTurn(uint8_t i)
{
    if(i == 0)
    {
        SoftSPI_OLED_WR_Byte(0xC8, SOFTSPI_OLED_CMD); // 正常显示
        SoftSPI_OLED_WR_Byte(0xA1, SOFTSPI_OLED_CMD);
    }
    if(i == 1)
    {
        SoftSPI_OLED_WR_Byte(0xC0, SOFTSPI_OLED_CMD); // 反转显示
        SoftSPI_OLED_WR_Byte(0xA0, SOFTSPI_OLED_CMD);
    }
}

// 软件SPI写入字节（命令/数据）
void SoftSPI_OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;
    // 区分命令/数据（DC引脚控制）
    if(cmd)
        SoftSPI_OLED_DC_Set();
    else
        SoftSPI_OLED_DC_Clr();
    
    SoftSPI_OLED_CS_Clr(); // 片选使能
    
    // 软件SPI时序（高位先发）
    for(i = 0; i < 8; i++)
    {
        SoftSPI_OLED_SCL_Clr(); // 时钟拉低
        if(dat & 0x80)          // 发送高位
            SoftSPI_OLED_SDA_Set();
        else
            SoftSPI_OLED_SDA_Clr();
        SoftSPI_OLED_SCL_Set(); // 时钟拉高（数据锁存）
        dat <<= 1;              // 移位准备下一位
    }
    
    SoftSPI_OLED_CS_Set(); // 片选关闭
    SoftSPI_OLED_DC_Set(); // 恢复DC为高（默认数据模式）
}

// 开启OLED显示
void SoftSPI_OLED_DisPlay_On(void)
{
    SoftSPI_OLED_WR_Byte(0x8D, SOFTSPI_OLED_CMD); // 电荷泵使能
    SoftSPI_OLED_WR_Byte(0x14, SOFTSPI_OLED_CMD); // 开启电荷泵
    SoftSPI_OLED_WR_Byte(0xAF, SOFTSPI_OLED_CMD); // 点亮屏幕
}

// 关闭OLED显示
void SoftSPI_OLED_DisPlay_Off(void)
{
    SoftSPI_OLED_WR_Byte(0x8D, SOFTSPI_OLED_CMD); // 电荷泵使能
    SoftSPI_OLED_WR_Byte(0x10, SOFTSPI_OLED_CMD); // 关闭电荷泵
    SoftSPI_OLED_WR_Byte(0xAE, SOFTSPI_OLED_CMD); // 关闭屏幕
}

// 更新显存到OLED
void SoftSPI_OLED_Refresh(void)
{
    uint8_t i, n;
    for(i = 0; i < 8; i++) // 共8页（0-7）
    {
        SoftSPI_OLED_WR_Byte(0xB0 + i, SOFTSPI_OLED_CMD); // 设置页地址
        SoftSPI_OLED_WR_Byte(0x00, SOFTSPI_OLED_CMD);     // 列地址低4位
        SoftSPI_OLED_WR_Byte(0x10, SOFTSPI_OLED_CMD);     // 列地址高4位
        for(n = 0; n < 128; n++)                          // 写入128列数据
            SoftSPI_OLED_WR_Byte(SoftSPI_OLED_GRAM[n][i], SOFTSPI_OLED_DATA);
    }
}

// 清屏函数（显存清零）
void SoftSPI_OLED_Clear(void)
{
    uint8_t i, n;
    for(i = 0; i < 8; i++)
        for(n = 0; n < 128; n++)
            SoftSPI_OLED_GRAM[n][i] = 0; // 全部置0
    SoftSPI_OLED_Refresh(); // 刷新显示
}

// 画点（x:0~127, y:0~63, t:1-填充,0-清空）
void SoftSPI_OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t i, m, n;
    i = y / 8;      // 计算所在页（每8行1页）
    m = y % 8;      // 计算页内行号
    n = 1 << m;     // 计算位掩码
    if(t)
        SoftSPI_OLED_GRAM[x][i] |= n; // 置位
    else
    {
        SoftSPI_OLED_GRAM[x][i] &= ~n; // 清位
    }
}

// 画线（Bresenham算法）
void SoftSPI_OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    
    delta_x = x2 - x1; // 计算x增量
    delta_y = y2 - y1; // 计算y增量
    uRow = x1;         // 起点x
    uCol = y1;         // 起点y
    
    // 设置方向增量
    if(delta_x > 0) incx = 1;
    else if(delta_x == 0) incx = 0; // 垂直线
    else { incx = -1; delta_x = -delta_x; }
    
    if(delta_y > 0) incy = 1;
    else if(delta_y == 0) incy = 0; // 水平线
    else { incy = -1; delta_y = -delta_y; }
    
    // 选择长轴作为步长基准
    if(delta_x > delta_y) distance = delta_x;
    else distance = delta_y;
    
    // 逐点绘制
    for(t = 0; t < distance + 1; t++)
    {
        SoftSPI_OLED_DrawPoint(uRow, uCol, mode);
        
        xerr += delta_x;
        yerr += delta_y;
        
        // x方向步进
        if(xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        // y方向步进
        if(yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

// 画圆（中点圆算法）
void SoftSPI_OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a, b, num;
    a = 0;
    b = r;
    while(2 * b * b >= r * r)
    {
        // 绘制8个对称点
        SoftSPI_OLED_DrawPoint(x + a, y - b, 1);
        SoftSPI_OLED_DrawPoint(x - a, y - b, 1);
        SoftSPI_OLED_DrawPoint(x - a, y + b, 1);
        SoftSPI_OLED_DrawPoint(x + a, y + b, 1);
        
        SoftSPI_OLED_DrawPoint(x + b, y + a, 1);
        SoftSPI_OLED_DrawPoint(x + b, y - a, 1);
        SoftSPI_OLED_DrawPoint(x - b, y - a, 1);
        SoftSPI_OLED_DrawPoint(x - b, y + a, 1);
        
        a++;
        num = (a * a + b * b) - r * r; // 距离判断
        if(num > 0) // 超出圆范围，y方向减1
        {
            b--;
            a--;
        }
    }
}

// 显示单个字符
void SoftSPI_OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size1, uint8_t mode)
{
    uint8_t i, m, temp, size2, chr1;
    uint8_t x0 = x, y0 = y;
    
    // 计算字体字节数
    if(size1 == 8) size2 = 6;
    else size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    
    chr1 = chr - ' '; // 计算偏移（从空格开始）
    
    for(i = 0; i < size2; i++)
    {
        // 取字模数据
        if(size1 == 8) temp = asc2_0806[chr1][i];    // 6x8
        else if(size1 == 12) temp = asc2_1206[chr1][i]; // 6x12
        else if(size1 == 16) temp = asc2_1608[chr1][i]; // 8x16
        else if(size1 == 24) temp = asc2_2412[chr1][i]; // 12x24
        else return;
        
        // 逐位绘制
        for(m = 0; m < 8; m++)
        {
            if(temp & 0x01) SoftSPI_OLED_DrawPoint(x, y, mode);
            else SoftSPI_OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        
        x++;
        // 换行处理
        if((size1 != 8) && ((x - x0) == size1 / 2))
        {
            x = x0;
            y0 += 8;
        }
        y = y0;
    }
}

// 显示字符串
void SoftSPI_OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size1, uint8_t mode)
{
    while((*chr >= ' ') && (*chr <= '~')) // 只显示可打印字符
    {
        SoftSPI_OLED_ShowChar(x, y, *chr, size1, mode);
        // 计算下一个字符x坐标
        if(size1 == 8) x += 6;
        else x += size1 / 2;
        chr++;
    }
}

// 求m^n（用于数字显示）
uint32_t SoftSPI_OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while(n--)
    {
        result *= m;
    }
    return result;
}

// 显示数字
void SoftSPI_OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size1, uint8_t mode)
{
    uint8_t t, temp, m = 0;
    if(size1 == 8) m = 2; // 6x8字体偏移调整
    
    for(t = 0; t < len; t++)
    {
        temp = (num / SoftSPI_OLED_Pow(10, len - t - 1)) % 10; // 取每一位
        if(temp == 0)
        {
            SoftSPI_OLED_ShowChar(x + (size1 / 2 + m) * t, y, '0', size1, mode);
        }
        else
        {
            SoftSPI_OLED_ShowChar(x + (size1 / 2 + m) * t, y, temp + '0', size1, mode);
        }
    }
}

/**
 * 显示16×32点阵汉字（阴码、列行式、逆向）
 * x,y: 起始坐标（x:0~127, y:0~63）
 * num: 汉字在Hzk数组中的索引
 * size1: 固定为32（表示16×32点阵）
 * mode: 0-正常显示；1-反色（叠加阴码逻辑）
 */
void SoftSPI_OLED_ShowBN(uint8_t x, uint8_t y, uint8_t num, uint8_t mode)
{ 
    uint8_t page, x_col, m;    // page:0~3（4页，每页8行）；x_col:0~15（16列）；m:0~7（每页内的行）
    uint8_t byte_data;         // 当前字节数据
    uint8_t real_bit;          // 处理逆向的实际位值
    int y_pos;                 // 实际显示的y坐标

    // 遍历4页（32行 = 4页×8行）
    for (page = 0; page < 4; page++)
    {
        // 遍历16列（x方向）
        for (x_col = 0; x_col < 16; x_col++)
        {
            // 获取当前列、当前页的字模数据（阴码）
            byte_data = Hzk[4 * num + page][x_col];
            
            // 遍历当前字节的8位（每页内的8行）
            for (m = 0; m < 8; m++)
            {
                // 处理逆向：原字模位顺序反转（根据取模"逆向"调整，此处假设低位在前→反转后高位在前）
                real_bit = (byte_data >> m) & 0x01; 
                // 若逆向是高位在前→低位在前，可改为：real_bit = (byte_data >> (7 - m)) & 0x01;
                
                // 阴码逻辑：0表示显示，1表示不显示，叠加mode反色
                uint8_t draw = (real_bit == 0) ? 1 : 0; // 阴码反转
                draw = (mode) ? !draw : draw;           // 叠加用户反色模式
                
                // 计算实际显示坐标
                y_pos = y + page * 8 + m; // 总y坐标 = 起始y + 页偏移 + 页内偏移
                if (y_pos >= 64) break;   // 超出屏幕范围则跳过
                
                // 绘制点（x + x_col为实际x坐标）
                SoftSPI_OLED_DrawPoint(x + x_col, y_pos, draw);
            }
        }
    }
}

// 显示汉字
void SoftSPI_OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t num, uint8_t size1, uint8_t mode)
{
    uint8_t m, temp;
    uint16_t i;
    uint8_t x0 = x, y0 = y;
    uint8_t size3 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * size1; // 汉字字节数
    
    for(i = 0; i < size3; i++)
    {
        // 取汉字字模
        if(size1 == 16) temp = Hzk1[num][i];  // 16x16
        else if(size1 == 24) temp = Hzk2[num][i]; // 24x24
        else if(size1 == 32) temp = Hzk3[num][i]; // 32x32
        else if(size1 == 64) temp = Hzk4[num][i]; // 64x64
        else return;
        
        // 逐位绘制
        for(m = 0; m < 8; m++)
        {
            if(temp & 0x01) SoftSPI_OLED_DrawPoint(x, y, mode);
            else SoftSPI_OLED_DrawPoint(x, y, !mode);
            temp >>= 1;
            y++;
        }
        
        x++;
        // 换行处理
        if((x - x0) == size1)
        {
            x = x0;
            y0 += 8;
        }
        y = y0;
    }
}

// 滚动显示
void SoftSPI_OLED_ScrollDisplay(uint8_t num, uint8_t space, uint8_t mode)
{
    uint8_t i, n, t = 0, m = 0, r;
    while(1)
    {
        if(m == 0)
        {
            // 从右侧写入新汉字
            SoftSPI_OLED_ShowChinese(128, 24, t, 16, mode);
            t++;
        }
        
        if(t == num)
        {
            // 间隔停顿
            for(r = 0; r < 16 * space; r++)
            {
                // 左移一列
                for(i = 1; i < 144; i++)
                    for(n = 0; n < 8; n++)
                        SoftSPI_OLED_GRAM[i - 1][n] = SoftSPI_OLED_GRAM[i][n];
                SoftSPI_OLED_Refresh();
            }
            t = 0;
        }
        
        m++;
        if(m == 16) m = 0; // 控制写入速度
        
        // 左移一列
        for(i = 1; i < 144; i++)
            for(n = 0; n < 8; n++)
                SoftSPI_OLED_GRAM[i - 1][n] = SoftSPI_OLED_GRAM[i][n];
        
        SoftSPI_OLED_Refresh();
    }
}

// 显示图片
void SoftSPI_OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[], uint8_t mode)
{
    uint16_t j = 0;
    uint8_t i, n, temp, m;
    uint8_t x0 = x, y0 = y;
    
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0); // 计算页数
    
    for(n = 0; n < sizey; n++)
    {
        for(i = 0; i < sizex; i++)
        {
            temp = BMP[j];
            j++;
            
            // 逐位绘制
            for(m = 0; m < 8; m++)
            {
                if(temp & 0x01) SoftSPI_OLED_DrawPoint(x, y, mode);
                else SoftSPI_OLED_DrawPoint(x, y, !mode);
                temp >>= 1;
                y++;
            }
            
            x++;
            // 换行处理
            if((x - x0) == sizex)
            {
                x = x0;
                y0 += 8;
            }
            y = y0;
        }
    }
}

// 初始化OLED
void SoftSPI_OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOA时钟（根据实际引脚修改）
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置SPI引脚（SCL/SDA/RES/DC/CS）为推挽输出
    GPIO_InitStruct.Pin = SOFTSPI_OLED_SCL_PIN | SOFTSPI_OLED_SDA_PIN | 
                          SOFTSPI_OLED_RES_PIN | SOFTSPI_OLED_DC_PIN | SOFTSPI_OLED_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFTSPI_OLED_SCL_PORT, &GPIO_InitStruct);
    
    // 硬件复位
    SoftSPI_OLED_RES_Clr();
    HAL_Delay(200); // 延时确保复位完成
    SoftSPI_OLED_RES_Set();
    
    // OLED初始化命令（基于SSD1306控制器）
    SoftSPI_OLED_WR_Byte(0xFD, SOFTSPI_OLED_CMD); // 解锁命令
    SoftSPI_OLED_WR_Byte(0x12, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0xAE, SOFTSPI_OLED_CMD); // 关闭显示
    SoftSPI_OLED_WR_Byte(0xD5, SOFTSPI_OLED_CMD); // 设置时钟分频
    SoftSPI_OLED_WR_Byte(0xA0, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0xA8, SOFTSPI_OLED_CMD); // 设置多路复用率
    SoftSPI_OLED_WR_Byte(0x3F, SOFTSPI_OLED_CMD); // 1/64占空比
    SoftSPI_OLED_WR_Byte(0xD3, SOFTSPI_OLED_CMD); // 设置显示偏移
    SoftSPI_OLED_WR_Byte(0x00, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0x40, SOFTSPI_OLED_CMD); // 设置起始行
    SoftSPI_OLED_WR_Byte(0xA1, SOFTSPI_OLED_CMD); // 列映射（正常）
    SoftSPI_OLED_WR_Byte(0xC8, SOFTSPI_OLED_CMD); // 行扫描方向（正常）
    SoftSPI_OLED_WR_Byte(0xDA, SOFTSPI_OLED_CMD); // COM引脚配置
    SoftSPI_OLED_WR_Byte(0x12, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0x81, SOFTSPI_OLED_CMD); // 对比度设置
    SoftSPI_OLED_WR_Byte(0x7F, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0xD9, SOFTSPI_OLED_CMD); // 预充电周期
    SoftSPI_OLED_WR_Byte(0x82, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0xDB, SOFTSPI_OLED_CMD); // VCOMH设置
    SoftSPI_OLED_WR_Byte(0x34, SOFTSPI_OLED_CMD);
    SoftSPI_OLED_WR_Byte(0xA4, SOFTSPI_OLED_CMD); // 正常显示（非全亮）
    SoftSPI_OLED_WR_Byte(0xA6, SOFTSPI_OLED_CMD); // 非反显
    
    SoftSPI_OLED_Clear(); // 清屏
    SoftSPI_OLED_WR_Byte(0xAF, SOFTSPI_OLED_CMD); // 开启显示
}
