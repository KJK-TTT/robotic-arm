#include "servo.h"                  // Device header

double pulse_1;
double pulse_2;
double pulse_3;
double pulse_4;
double pulse_5;
double pulse_6;

/* 姿态解算后每个舵机转的角度 */
double target_angle_1;
double target_angle_2;
double target_angle_3;
double target_angle_4;
double target_angle_5;
double target_angle_6;

/* 当前舵机所处的角度 */
double now_angle_1;
double now_angle_2;
double now_angle_3;
double now_angle_4;
double now_angle_5;
double now_angle_6;

double pi = 3.1415927;

/**
  * @brief  根据目标位置，计算机械臂的各个关节角度
  * @param  target_x 目标点的 x 坐标
  * @param  target_y 目标点的 y 坐标
  * @param  target_z 目标点的 z 坐标
  *         This parameter can be a value of @ref target_x, target_y, target_z 均为浮点型
  * @retval 无返回值，但计算出的关节角度会被存储在全局变量 target_angle_1, target_angle_2, target_angle_3, target_angle_4 中
  * @note   该函数假设机械臂的底座在原点，目标点不应超出机械臂的运动范围。target_y 被限制在 3 到 20 之间，以避免机械臂超出工作空间
  */
void servo_angle_calculate(float target_x, float target_y, float target_z)
{
    if (target_y >= 20)
        target_y = 20;
    else if(target_y <= 3)
        target_y = 3;

    // 初始化机械臂的参数，定义各段机械臂的长度
    float len_1, len_2, len_3, len_4;   // len_1为底部高度，len_2, len_3, len_4为机械臂各段的长度
    float j1, j2, j3, j4;               // 关节角度 j1为底部旋转角度，j2, j3, j4为各机械臂关节角度
    float L, H, bottom_r;               // L为水平投影长度，H为垂直高度，bottom_r为底部半径
																				// L =	a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); P为底部圆盘半径R
    float j_sum;                        // j2, j3, j4之和
    float len, high;                    // 目标位置的总长度和高度
    float cos_j3, sin_j3;               // 关节3的余弦和正弦值，用来简化计算
    float cos_j2, sin_j2;               // 关节2的余弦和正弦值，用来简化计算
    float k1, k2;                       // 中间变量，用于计算关节2角度
    int i;                              // 迭代变量，用于遍历角度范围
    float n, m;                         // n为有效解的数量，m为选取的解的中间值

    n = 0;
    m = 0;

    // 输入初始值
    bottom_r = 8.00;    // 底部圆盘的半径 8 cm
    len_1 = 8.20;       // 底部高度8.5cm
    len_2 = 10.50;      // 机械臂的第一段长度10.5cm
    len_3 = 10.00;      // 机械臂的第二段长度10cm
    len_4 = 16.00;      // 机械臂的末端长度16.0cm

    // 计算底部旋转角度j1，根据target_x和target_y决定旋转角度
    if (target_x == 0)
        j1 = 90;  // 如果水平距离为0，机械臂不需要旋转
    else
        // 使用反正切函数atan计算角度，并将弧度转换为角度
        j1 = 90 - atan(target_x / (target_y + bottom_r)) * 57.3;

    // 第一遍循环，计算所有可能的关节角度组合
    for (i = 0; i <= 180; i++)
    {
        j_sum = 3.1415927 * i / 180; // 将角度从0度到180度转换为弧度制

        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x); // 计算目标点到原点的水平距离
        high = target_z; 

        L = len - len_4 * sin(j_sum); 
        H = high - len_4 * cos(j_sum) - len_1; 

        // 使用余弦定理计算关节3的角度
        cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
        sin_j3 = sqrt(1 - (cos_j3) * (cos_j3)); // 计算sin(j3)

        j3 = atan(sin_j3 / cos_j3) * 57.3; // 关节3的角度

        // 使用几何关系计算关节2的角度
        k2 = len_3 * sin(j3 / 57.3);
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2); // 计算关节2角度的余弦
        sin_j2 = sqrt(1 - (cos_j2) * (cos_j2)); // 计算sin(j2)

        j2 = atan(sin_j2 / cos_j2) * 57.3; // 关节2的角度
        j4 = j_sum * 57.3 - j2 - j3; // 关节4的角度

        // 确保计算出来的角度在有效范围内
        if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
        {
            n++; // 记录有效解的数量
        }
    }

    // 第二遍循环，寻找最接近中间的解
    for (i = 0; i <= 180; i++)
    {
        j_sum = 3.1415927 * i / 180; // 继续使用弧度制

        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x); // 重新计算水平距离
        high = target_z; // 目标深度

        L = len - len_4 * sin(j_sum); // 水平投影长度
        H = high - len_4 * cos(j_sum) - len_1; // 垂直高度

        // 重新计算关节3的余弦和正弦值
        cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
        sin_j3 = sqrt(1 - (cos_j3) * (cos_j3)); // sin(j3)

        j3 = atan(sin_j3 / cos_j3) * 57.3; // 关节3的角度

        k2 = len_3 * sin(j3 / 57.3);
			
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2); // 计算关节2角度
        sin_j2 = sqrt(1 - (cos_j2) * (cos_j2)); // sin(j2)

        j2 = atan(sin_j2 / cos_j2) * 57.3; // 关节2角度
        j4 = j_sum * 57.3 - j2 - j3; // 关节4角度

        // 确保角度在有效范围内
        if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
        {
            m++; // 记录中间解的数量
            if (m == n / 2 || m == (n + 1) / 2) // 找到中间解
                break;
        }
    }

    // 输出计算得到的各个关节角度
    target_angle_1 = j1; // 关节1角度
    target_angle_2 = j2; // 关节2角度
    target_angle_3 = j3; // 关节3角度
    target_angle_4 = j4; // 关节4角度

    // 输出坐标和角度，便于调试
    //printf("center_x: %f\r\n center_y: %f\r\n taret_angle_1: %f\r\n taret_angle_2: %f\r\n taret_angle_3: %f\r\n taret_angle_4: %f\r\n",
    //       target_x, target_y, j1, j2, j3, j4);
}

/**
  * @brief  开启PWM输出
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
  * @brief  将角度转换为对应的脉冲
  * @param  angle
  *   This parameter can be a value of @ref -90 ~ 90.
  * @retval None
	*	@note 
	*	      注意每个舵机的旋转范围	     
	*	      根据安装位置,自行调节			      
  */
void angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6)
{
	pulse_1 = (angle_1+10) *2000/180+500;// 0~180°
	pulse_2 = (-angle_2+90)*2000/180+500;// -90~90°
	pulse_3 = (angle_3+90)*2000/180+500; // -90~90°
	pulse_4 = (angle_4+90)*2000/180+500; // -90~90°
	pulse_5 = (angle_5+90)*2000/180+500; // -90~90°
	pulse_6 = (angle_6+90)*2000/180+500; // -90~90°
}

/**
  * @brief  底层PWM输出
  * @param  舵机目标角度
  *   angle_1 parameter can be a value of @ref 0 ~ 180.
  *   other   parameter can be a value of @ref -90 ~ 90.
  * @retval None
	*	@note 
	*			舵机控制频率为50Hz，周期为20ms，PSC = 72 - 1，ARR = 200 - 1，f = 72MHz /( PSC + 1 )( ARR +1 )
	*			占空比 = pulse / ARR
	*			当高电平时间为0.5ms时，舵机角度为0度
	*			当高电平时间为1.5ms时，舵机角度为90度
	*			当高电平时间为2.5ms时，舵机角度为180度
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
  * @brief  系统复位
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
  * @brief  让舵机平稳的移动到目标位置,提高抓取的概率,避免损伤舵机
  * @param  舵机目标角度
  *   angle_1 parameter can be a value of @ref 0 ~ 180.
  *   other   parameter can be a value of @ref -90 ~ 90.
  * @retval None
	*/
void smooth_servo_control(double target_angle_1, double target_angle_2, double target_angle_3,double target_angle_4) 
{
    double current_angles[4] = {now_angle_1, now_angle_2, now_angle_3, now_angle_4};
    double angle_increment = 1.0; // 设置每次调整的角度增量
    uint8_t all_reached = 0; // 标志位，初始为未到达

    while (all_reached == 0)
		{
        all_reached = 1; // 假设所有舵机都已到达目标角度

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

            // 计算当前舵机与目标舵机之间的误差
            double error = target_angle - current_angles[i];
            if (fabs(error) > angle_increment)
						{
                // 逐步调整舵机角度
                current_angles[i] += (error > 0) ? angle_increment : -angle_increment;
                all_reached = 0; // 标记为未到达目标角度
            }
						else 
						{
                // 如果误差小于增量，则直接到达目标角度
                current_angles[i] = target_angle;
            }
        }
        // 输出当前舵机的角度
        pwm_out(current_angles[0], current_angles[1], current_angles[2], current_angles[3],0,-90);
        HAL_Delay(20); // 延时，给予舵机时间移动
    }
		pwm_out(current_angles[0], current_angles[1], current_angles[2], current_angles[3],0,-10);
		HAL_Delay(1000);
}


/**
  * @brief  抓取(只是夹住目标)
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
  servo_angle_calculate(center_x_cm,center_y_cm,2); // 姿态解算
  smooth_servo_control(target_angle_1,target_angle_2,target_angle_3,target_angle_4); //
}

/**
  * @brief  抓取 并 搬运 蓝色物体
  * @param  None
  * @retval None
	*/
void task_blue(void)
{
  catch_box();
  pwm_out(170,45,45,0,0,-10);  // 转到收纳筐上方
  HAL_Delay(1000);
  pwm_out(170,45,45,45,0,-10); // 转到收纳筐上方
  HAL_Delay(1000);
  pwm_out(170,45,45,45,0,-90); // 放下
  HAL_Delay(1000);
}

/**
  * @brief  抓取 并 搬运 绿色物体
  * @param  None
  * @retval None
	*/
void task_green(void)
{
  catch_box();
  pwm_out(150,45,45,0,0,-10); // 转到收纳筐上方
  HAL_Delay(1000);
  pwm_out(150,45,45,0,0,-90); // 放下
  HAL_Delay(1000);
}
