#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "main.h"
#include "usart.h"

extern uint16_t center_x;
extern uint16_t center_y;   
extern uint16_t color_type;

extern double center_x_cm;
extern double center_y_cm;

void openmv_receive_data(uint16_t com_data);
void coordinate_transformation(uint16_t center_x_temp, uint16_t center_y_temp);

#endif

