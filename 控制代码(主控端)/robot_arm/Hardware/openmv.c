#include "openmv.h"

uint16_t center_x = 0;
uint16_t center_y = 0;   
uint16_t color_type = 0;

double center_x_cm = 0;
double center_y_cm = 0;


/**
  * @brief  接收openmv发送的 颜色、x轴和y轴的像素坐标
  * @param  com_data : openmv发送过;来的一帧数据
  * @retval None
  */
void openmv_receive_data(uint16_t com_data)
{
    uint8_t i;
    static uint8_t rx_counter_1 = 0; // 记录接收到的数据字节数量
    static uint16_t rx_buffer_1[10] = {0}; 
    static uint8_t rx_state = 0;

    //char buffer[50];  
        
    if (rx_state == 0 && com_data == 0x2C) 
    {
        rx_state = 1; 
        rx_buffer_1[rx_counter_1++] = com_data;
    }
    else if (rx_state == 1 && com_data == 0x12)
    {
        rx_state = 2;
        rx_buffer_1[rx_counter_1++] = com_data; 
    }
    else if (rx_state == 2)  // 在状态2时，接收并存储颜色类型、X坐标和Y坐标
    {
        rx_buffer_1[rx_counter_1++] = com_data; // 将接收到的字节存储到缓冲区
        // 如果接收到8个字节或者遇到结束字节 0x5B，说明数据接收完成
        if (rx_counter_1 >= 8 || com_data == 0x5B)   
        {
            rx_state = 3; // 切换到状态3，表示数据接收完成
            // 解析接收到的数据
            color_type = rx_buffer_1[2]; // 获取颜色类型
            center_x = (rx_buffer_1[3] << 8) | rx_buffer_1[4]; // 合并高低字节为X坐标
            center_y = (rx_buffer_1[5] << 8) | rx_buffer_1[6]; // 合并高低字节为Y坐标

            // 打印接收到的颜色种类和坐标数据
            //sprintf(buffer,"color_type = %c, x = %d, y = %d\r\n", color_type, center_x, center_y);
            //HAL_UART_Transmit(&huart3, (const uint8_t *)buffer, strlen(buffer), 0xFFFF);

            rx_counter_1 = 0; // 重置计数器
            rx_state = 0;     // 切换状态回到初始状态，准备接收下一帧数据
        }
    }
    else if (rx_state == 3) // 在状态3时，检查是否接收到结束字节 0x5B
    {
        if (com_data == 0x5B)  // 检查最后一个字节是否是结束标志
        {
            // 数据已成功接收并处理，进入初始状态
            rx_counter_1 = 0; // 重置计数器
            rx_state = 0;     // 切换状态回到初始状态
        }
        else   // 如果没有接收到正确的结束标志，说明数据接收错误
        {
            rx_state = 0; // 重置状态机
            rx_counter_1 = 0; // 重置计数器
            // 清空缓冲区中的数据
            for (i = 0; i < 10; i++)
            {
                rx_buffer_1[i] = 0x00; // 将存放数据的数组清零
            }
        }
    }
    else   // 其他情况，接收异常
    {
        rx_state = 0; // 重置状态机
        rx_counter_1 = 0; // 重置计数器
        // 清空缓冲区中的数据
        for (i = 0; i < 10; i++)
        {
            rx_buffer_1[i] = 0x00; // 将存放数据的数组清零
        }
    }
}

/**
  * @brief  将 像素坐标 转换到 物理坐标
  * @param  center_x_temp 目标在x轴的像素坐标
  * @param  center_x_temp 目标在y轴的像素坐标
  * @retval None, 但计算出的物理坐标会被存储在全局变量中
  * @note 根据自己选取的参考点进行转换(注意数据类型)
  */
void coordinate_transformation(uint16_t center_x_temp, uint16_t center_y_temp)
{
    char buffer[50]; 
    double ratio_x = 42.0 / 640;
    double ratio_y = 30.0 / 480;
    center_x_cm = (int16_t)(175 - center_x_temp) * ratio_x;  
    center_y_cm = center_y_temp * ratio_y; 
    sprintf(buffer," real: x = %.2f, y = %.2f\r\n",center_x_cm,center_y_cm);
    HAL_UART_Transmit(&huart3,(const uint8_t *)buffer,strlen(buffer),0xFFFF);  
}
