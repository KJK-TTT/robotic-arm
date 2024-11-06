#include "servo.h"                  // Device header

double pulse_1;
double pulse_2;
double pulse_3;
double pulse_4;
double pulse_5;
double pulse_6;

/* ��̬�����ÿ�����ת�ĽǶ� */
double target_angle_1;
double target_angle_2;
double target_angle_3;
double target_angle_4;
double target_angle_5;
double target_angle_6;

/* ��ǰ��������ĽǶ� */
double now_angle_1;
double now_angle_2;
double now_angle_3;
double now_angle_4;
double now_angle_5;
double now_angle_6;

double pi = 3.1415927;

/**
  * @brief  ����Ŀ��λ�ã������е�۵ĸ����ؽڽǶ�
  * @param  target_x Ŀ���� x ����
  * @param  target_y Ŀ���� y ����
  * @param  target_z Ŀ���� z ����
  *         This parameter can be a value of @ref target_x, target_y, target_z ��Ϊ������
  * @retval �޷���ֵ����������ĹؽڽǶȻᱻ�洢��ȫ�ֱ��� target_angle_1, target_angle_2, target_angle_3, target_angle_4 ��
  * @note   �ú��������е�۵ĵ�����ԭ�㣬Ŀ��㲻Ӧ������е�۵��˶���Χ��target_y �������� 3 �� 20 ֮�䣬�Ա����е�۳��������ռ�
  */
void servo_angle_calculate(float target_x, float target_y, float target_z)
{
    if (target_y >= 20)
        target_y = 20;
    else if(target_y <= 3)
        target_y = 3;

    // ��ʼ����е�۵Ĳ�����������λ�е�۵ĳ���
    float len_1, len_2, len_3, len_4;   // len_1Ϊ�ײ��߶ȣ�len_2, len_3, len_4Ϊ��е�۸��εĳ���
    float j1, j2, j3, j4;               // �ؽڽǶ� j1Ϊ�ײ���ת�Ƕȣ�j2, j3, j4Ϊ����е�۹ؽڽǶ�
    float L, H, bottom_r;               // LΪˮƽͶӰ���ȣ�HΪ��ֱ�߶ȣ�bottom_rΪ�ײ��뾶
																				// L =	a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); PΪ�ײ�Բ�̰뾶R
    float j_sum;                        // j2, j3, j4֮��
    float len, high;                    // Ŀ��λ�õ��ܳ��Ⱥ͸߶�
    float cos_j3, sin_j3;               // �ؽ�3�����Һ�����ֵ�������򻯼���
    float cos_j2, sin_j2;               // �ؽ�2�����Һ�����ֵ�������򻯼���
    float k1, k2;                       // �м���������ڼ���ؽ�2�Ƕ�
    int i;                              // �������������ڱ����Ƕȷ�Χ
    float n, m;                         // nΪ��Ч���������mΪѡȡ�Ľ���м�ֵ

    n = 0;
    m = 0;

    // �����ʼֵ
    bottom_r = 8.00;    // �ײ�Բ�̵İ뾶 8 cm
    len_1 = 8.20;       // �ײ��߶�8.5cm
    len_2 = 10.50;      // ��е�۵ĵ�һ�γ���10.5cm
    len_3 = 10.00;      // ��е�۵ĵڶ��γ���10cm
    len_4 = 16.00;      // ��е�۵�ĩ�˳���16.0cm

    // ����ײ���ת�Ƕ�j1������target_x��target_y������ת�Ƕ�
    if (target_x == 0)
        j1 = 90;  // ���ˮƽ����Ϊ0����е�۲���Ҫ��ת
    else
        // ʹ�÷����к���atan����Ƕȣ���������ת��Ϊ�Ƕ�
        j1 = 90 - atan(target_x / (target_y + bottom_r)) * 57.3;

    // ��һ��ѭ�����������п��ܵĹؽڽǶ����
    for (i = 0; i <= 180; i++)
    {
        j_sum = 3.1415927 * i / 180; // ���Ƕȴ�0�ȵ�180��ת��Ϊ������

        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x); // ����Ŀ��㵽ԭ���ˮƽ����
        high = target_z; 

        L = len - len_4 * sin(j_sum); 
        H = high - len_4 * cos(j_sum) - len_1; 

        // ʹ�����Ҷ������ؽ�3�ĽǶ�
        cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
        sin_j3 = sqrt(1 - (cos_j3) * (cos_j3)); // ����sin(j3)

        j3 = atan(sin_j3 / cos_j3) * 57.3; // �ؽ�3�ĽǶ�

        // ʹ�ü��ι�ϵ����ؽ�2�ĽǶ�
        k2 = len_3 * sin(j3 / 57.3);
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2); // ����ؽ�2�Ƕȵ�����
        sin_j2 = sqrt(1 - (cos_j2) * (cos_j2)); // ����sin(j2)

        j2 = atan(sin_j2 / cos_j2) * 57.3; // �ؽ�2�ĽǶ�
        j4 = j_sum * 57.3 - j2 - j3; // �ؽ�4�ĽǶ�

        // ȷ����������ĽǶ�����Ч��Χ��
        if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
        {
            n++; // ��¼��Ч�������
        }
    }

    // �ڶ���ѭ����Ѱ����ӽ��м�Ľ�
    for (i = 0; i <= 180; i++)
    {
        j_sum = 3.1415927 * i / 180; // ����ʹ�û�����

        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x); // ���¼���ˮƽ����
        high = target_z; // Ŀ�����

        L = len - len_4 * sin(j_sum); // ˮƽͶӰ����
        H = high - len_4 * cos(j_sum) - len_1; // ��ֱ�߶�

        // ���¼���ؽ�3�����Һ�����ֵ
        cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
        sin_j3 = sqrt(1 - (cos_j3) * (cos_j3)); // sin(j3)

        j3 = atan(sin_j3 / cos_j3) * 57.3; // �ؽ�3�ĽǶ�

        k2 = len_3 * sin(j3 / 57.3);
			
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2); // ����ؽ�2�Ƕ�
        sin_j2 = sqrt(1 - (cos_j2) * (cos_j2)); // sin(j2)

        j2 = atan(sin_j2 / cos_j2) * 57.3; // �ؽ�2�Ƕ�
        j4 = j_sum * 57.3 - j2 - j3; // �ؽ�4�Ƕ�

        // ȷ���Ƕ�����Ч��Χ��
        if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
        {
            m++; // ��¼�м�������
            if (m == n / 2 || m == (n + 1) / 2) // �ҵ��м��
                break;
        }
    }

    // �������õ��ĸ����ؽڽǶ�
    target_angle_1 = j1; // �ؽ�1�Ƕ�
    target_angle_2 = j2; // �ؽ�2�Ƕ�
    target_angle_3 = j3; // �ؽ�3�Ƕ�
    target_angle_4 = j4; // �ؽ�4�Ƕ�

    // �������ͽǶȣ����ڵ���
    //printf("center_x: %f\r\n center_y: %f\r\n taret_angle_1: %f\r\n taret_angle_2: %f\r\n taret_angle_3: %f\r\n taret_angle_4: %f\r\n",
    //       target_x, target_y, j1, j2, j3, j4);
}

/**
  * @brief  ����PWM���
  * @param  None
  * @retval None
	*	@note 	None
  */
void robot_arm_init(void)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}


/**
  * @brief  ���Ƕ�ת��Ϊ��Ӧ������
  * @param  angle
  *   This parameter can be a value of @ref -90 ~ 90.
  * @retval None
	*	@note 
	*	      ע��ÿ���������ת��Χ	     
	*	      ���ݰ�װλ��,���е���			      
  */
void angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6)
{
	pulse_1 = (angle_1+10) *2000/180+500;// 0~180��
	pulse_2 = (-angle_2+90)*2000/180+500;// -90~90��
	pulse_3 = (angle_3+90)*2000/180+500; // -90~90��
	pulse_4 = (angle_4+90)*2000/180+500; // -90~90��
	pulse_5 = (angle_5+90)*2000/180+500; // -90~90��
	pulse_6 = (angle_6+90)*2000/180+500; // -90~90��
}

/**
  * @brief  �ײ�PWM���
  * @param  ���Ŀ��Ƕ�
  *   angle_1 parameter can be a value of @ref 0 ~ 180.
  *   other   parameter can be a value of @ref -90 ~ 90.
  * @retval None
	*	@note 
	*			�������Ƶ��Ϊ50Hz������Ϊ20ms��PSC = 72 - 1��ARR = 200 - 1��f = 72MHz /( PSC + 1 )( ARR +1 )
	*			ռ�ձ� = pulse / ARR
	*			���ߵ�ƽʱ��Ϊ0.5msʱ������Ƕ�Ϊ0��
	*			���ߵ�ƽʱ��Ϊ1.5msʱ������Ƕ�Ϊ90��
	*			���ߵ�ƽʱ��Ϊ2.5msʱ������Ƕ�Ϊ180��
	*/
void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5,  double angle_6)
{
	angle_to_pulse(angle_1,angle_2,angle_3,angle_4,angle_5,angle_6);
	
	if(pulse_1 != NULL)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse_1);
	}
	if(pulse_2 != NULL)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_2);
	}
	if(pulse_3 != NULL)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pulse_3);
	}
	if(pulse_4 != NULL)
	{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse_4);
	}
	if(pulse_5 != NULL)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pulse_5);
	}
	if(pulse_6 != NULL)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pulse_6);
	}
}

/**
  * @brief  ϵͳ��λ
  * @param  None
  * @retval None
	*/
void servo_reset_begin(void)
{
    now_angle_1 = 90;
    now_angle_2 = 0;
    now_angle_3 = 0;
    now_angle_4 = 0;
    now_angle_5 = 0;
    now_angle_6 = -90;
    //smooth_servo_control(90,0,0,0);
    smooth_servo_control(0,45,45,0);
}


/**
  * @brief  �ö��ƽ�ȵ��ƶ���Ŀ��λ��,���ץȡ�ĸ���,�������˶��
  * @param  ���Ŀ��Ƕ�
  *   angle_1 parameter can be a value of @ref 0 ~ 180.
  *   other   parameter can be a value of @ref -90 ~ 90.
  * @retval None
	*/
void smooth_servo_control(double target_angle_1, double target_angle_2, double target_angle_3,double target_angle_4) 
{
    double current_angles[4] = {now_angle_1, now_angle_2, now_angle_3, now_angle_4};
    double angle_increment = 1.0; // ����ÿ�ε����ĽǶ�����
    uint8_t all_reached = 0; // ��־λ����ʼΪδ����

    while (all_reached == 0)
		{
        all_reached = 1; // �������ж�����ѵ���Ŀ��Ƕ�

        for (int i = 0; i < 4; i++)
				{
            double target_angle = 0;
            switch (i) 
						{
                case 0: target_angle = target_angle_1; break;
                case 1: target_angle = target_angle_2; break;
                case 2: target_angle = target_angle_3; break;
                case 3: target_angle = target_angle_4; break;
            }

            // ���㵱ǰ�����Ŀ����֮������
            double error = target_angle - current_angles[i];
            if (fabs(error) > angle_increment)
						{
                // �𲽵�������Ƕ�
                current_angles[i] += (error > 0) ? angle_increment : -angle_increment;
                all_reached = 0; // ���Ϊδ����Ŀ��Ƕ�
            }
						else 
						{
                // ������С����������ֱ�ӵ���Ŀ��Ƕ�
                current_angles[i] = target_angle;
            }
        }
        // �����ǰ����ĽǶ�
        pwm_out(current_angles[0], current_angles[1], current_angles[2], current_angles[3],0,-90);
        HAL_Delay(20); // ��ʱ��������ʱ���ƶ�
    }
		pwm_out(current_angles[0], current_angles[1], current_angles[2], current_angles[3],0,-10);
		HAL_Delay(1000);
}


/**
  * @brief  ץȡ(ֻ�Ǽ�סĿ��)
  * @param  None
  * @retval None
	*/
void catch_box(void)
{
  coordinate_transformation(center_x,center_y);
  if ((center_y_cm <= 8) || (center_y_cm >= 16) || (center_x_cm <= -3.0))
  {
    center_y_cm += 3.5;
  }
  else
  {
    center_y_cm += 3;
  }
  servo_angle_calculate(center_x_cm,center_y_cm,2); // ��̬����
  smooth_servo_control(target_angle_1,target_angle_2,target_angle_3,target_angle_4); //
}

/**
  * @brief  ץȡ �� ���� ��ɫ����
  * @param  None
  * @retval None
	*/
void task_blue(void)
{
  catch_box();
  pwm_out(170,45,45,0,0,-10);  // ת�����ɿ��Ϸ�
  HAL_Delay(1000);
  pwm_out(170,45,45,45,0,-10); // ת�����ɿ��Ϸ�
  HAL_Delay(1000);
  pwm_out(170,45,45,45,0,-90); // ����
  HAL_Delay(1000);
}

/**
  * @brief  ץȡ �� ���� ��ɫ����
  * @param  None
  * @retval None
	*/
void task_green(void)
{
  catch_box();
  pwm_out(150,45,45,0,0,-10); // ת�����ɿ��Ϸ�
  HAL_Delay(1000);
  pwm_out(150,45,45,0,0,-90); // ����
  HAL_Delay(1000);
}
