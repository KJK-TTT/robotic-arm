#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"
#include "tim.h"
#include "openmv.h"

extern double pulse_1;
extern double pulse_2;
extern double pulse_3;
extern double pulse_4;
extern double pulse_5;
extern double pulse_6;

extern double target_angle_1;
extern double target_angle_2;
extern double target_angle_3;
extern double target_angle_4;
extern double target_angle_5;
extern double target_angle_6;

extern double now_angle_1;
extern double now_angle_2;
extern double now_angle_3;
extern double now_angle_4;
extern double now_angle_5;
extern double now_angle_6;

extern double pi;


void robot_arm_init(void);
void angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4,double angle_5,double angle_6);
void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4, double angle_5,  double angle_6);
void servo_angle_calculate(float target_x, float target_y, float target_z);
void servo_reset_begin(void);
void smooth_servo_control(double target_angle_1, double target_angle_2, double target_angle_3,double target_angle_4);

void catch_box(void);
void task_blue(void);
void task_green(void);

#endif
