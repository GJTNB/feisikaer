#include "steer_control.h"
#include "get_car_angle.h"

int16 angle=0;
int16 angle_last=0;
void steer_control(void)
{
  static float k_last[8]={0}; //前几次的微分

  static int16 buff=0;
  
  buff=(int16)(car_p.diff*100);
  buff=((buff/10)*10);
  car_p.diff=((float)buff/100.0f);  
  
  if((car_p.diff-k_last[0])>=10){
    car_p.diff=k_last[0]+10;
  }
  
  if((car_p.diff-k_last[0])<=(-10)){
    car_p.diff=k_last[0]-10;
  }//10
  
  k_last[7]=k_last[6];
  k_last[6]=k_last[5];
  k_last[5]=k_last[4];
  k_last[4]=k_last[3];
  k_last[3]=k_last[2];
  k_last[2]=k_last[1];
  k_last[1]=k_last[0];
  k_last[0]=car_p.diff;  
  
//  static float diff=0;
//  diff=(k_last[0]-k_last[3]);
//  diff=(diff>=10?10:diff);
//  diff=(diff<=(-10)?(-10):diff); 
  

  angle=(int16)(float01*(k_last[0])); //角度  speed_of_z陀螺仪角度  + float02*speed_of_z
  //400
  angle=(angle<=(-180)?(-180):angle);
  angle=(angle>=180?180:angle);   
  
//  if(car_p.back_flag==1){   //倒车标志位
//    if(car_p.ten_left==1){
//      angle=160;
//    }
//    if(car_p.ten_righ==1){
//      angle=-160;
//    }
//  }
  
  ftm_pwm_duty(FTM1,FTM_CH0,int03-angle); //160右转  -160左转
  
  angle_last=angle;

}