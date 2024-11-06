#include "openmv.h"

uint16_t center_x = 0;
uint16_t center_y = 0;   
uint16_t color_type = 0;

double center_x_cm = 0;
double center_y_cm = 0;


/**
  * @brief  ����openmv���͵� ��ɫ��x���y�����������
  * @param  com_data : openmv���͹�;����һ֡����
  * @retval None
  */
void openmv_receive_data(uint16_t com_data)
{
    uint8_t i;
    static uint8_t rx_counter_1 = 0; // ��¼���յ��������ֽ�����
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
    else if (rx_state == 2)  // ��״̬2ʱ�����ղ��洢��ɫ���͡�X�����Y����
    {
        rx_buffer_1[rx_counter_1++] = com_data; // �����յ����ֽڴ洢��������
        // ������յ�8���ֽڻ������������ֽ� 0x5B��˵�����ݽ������
        if (rx_counter_1 >= 8 || com_data == 0x5B)   
        {
            rx_state = 3; // �л���״̬3����ʾ���ݽ������
            // �������յ�������
            color_type = rx_buffer_1[2]; // ��ȡ��ɫ����
            center_x = (rx_buffer_1[3] << 8) | rx_buffer_1[4]; // �ϲ��ߵ��ֽ�ΪX����
            center_y = (rx_buffer_1[5] << 8) | rx_buffer_1[6]; // �ϲ��ߵ��ֽ�ΪY����

            // ��ӡ���յ�����ɫ�������������
            //sprintf(buffer,"color_type = %c, x = %d, y = %d\r\n", color_type, center_x, center_y);
            //HAL_UART_Transmit(&huart3, (const uint8_t *)buffer, strlen(buffer), 0xFFFF);

            rx_counter_1 = 0; // ���ü�����
            rx_state = 0;     // �л�״̬�ص���ʼ״̬��׼��������һ֡����
        }
    }
    else if (rx_state == 3) // ��״̬3ʱ������Ƿ���յ������ֽ� 0x5B
    {
        if (com_data == 0x5B)  // ������һ���ֽ��Ƿ��ǽ�����־
        {
            // �����ѳɹ����ղ����������ʼ״̬
            rx_counter_1 = 0; // ���ü�����
            rx_state = 0;     // �л�״̬�ص���ʼ״̬
        }
        else   // ���û�н��յ���ȷ�Ľ�����־��˵�����ݽ��մ���
        {
            rx_state = 0; // ����״̬��
            rx_counter_1 = 0; // ���ü�����
            // ��ջ������е�����
            for (i = 0; i < 10; i++)
            {
                rx_buffer_1[i] = 0x00; // ��������ݵ���������
            }
        }
    }
    else   // ��������������쳣
    {
        rx_state = 0; // ����״̬��
        rx_counter_1 = 0; // ���ü�����
        // ��ջ������е�����
        for (i = 0; i < 10; i++)
        {
            rx_buffer_1[i] = 0x00; // ��������ݵ���������
        }
    }
}

/**
  * @brief  �� �������� ת���� ��������
  * @param  center_x_temp Ŀ����x�����������
  * @param  center_x_temp Ŀ����y�����������
  * @retval None, �����������������ᱻ�洢��ȫ�ֱ�����
  * @note �����Լ�ѡȡ�Ĳο������ת��(ע����������)
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
