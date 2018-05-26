#ifndef _get_car_angle_
#define _get_car_angle_

#include "common.h"
#include "include.h"
#include "bsp_my_oled.h" 

extern float gyro_zero_x;
extern float gyro_zero_y;
extern float gyro_zero_z;

extern float angle_of_z;
extern float speed_of_z;

extern void get_gyro_zero(void);
extern void get_car_angle(void);
extern void KalmanFilter(void);
#endif  /*_get_car_angle_*/