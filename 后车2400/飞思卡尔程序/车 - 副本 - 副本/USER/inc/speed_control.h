#ifndef _speed_control_
#define _speed_control_

#include "common.h"
#include "include.h"

extern float Ratio_Encoder_Left;   // 左轮速度=counter*左轮周长(mm)/(左轮转一圈对应的脉冲数*程序周期)
extern float Ratio_Encoder_Righ;   //  右轮速度=counter*右轮周长(mm)/(右轮转一圈对应的脉冲数*程序周期)
extern int16 error[3];
extern int16 pwm;

extern void speed_control(void);
extern int16 get_left_speed(void);
extern int16 get_righ_speed(void);

void speed_control_change(void);
extern int16 pid_left(int16 expect_speed,int16 really_speed);
extern int16 pid_righ(int16 expect_speed,int16 really_speed);
#endif /*_speed_control_*/