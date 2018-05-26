#include "speed_control.h"
#include "picture_deal.h"
#include "steer_control.h"
#include "common.h"
#include "include.h"
#include "init.h"

float Ratio_Encoder_Left = 204/(1163*0.02);// 左轮速度=counter*左轮周长(mm)/(左轮转一圈对应的脉冲数*程序周期)
float Ratio_Encoder_Righ = 204/(1163*0.02);//  右轮速度=counter*右轮周长(mm)/(右轮转一圈对应的脉冲数*程序周期)

float k_speed[17]={1.0f,1.0f,1.0f,1.0f,1.0f,1.0f,0.985f,0.929f,0.893f,0.864f,0.845f,0.828f,0.753f,0.735f,0.683f,0.654f,0.601f};
extern uint16 cur_R_ready_rest_flag;    //右圆环预判断复位变量
extern uint16 cur_L_ready_rest_flag;    //右圆环预判断复位变量
extern uint16 cur_L_real_delay_flag;    //左圆环准确弛懈识别标志
extern uint16 cur_R_real_delay_flag;    //左圆环准确弛懈识别标志
int16 error[3]={0,0,0};
int16 pwm=4000;
int16 expect_speed_last=4000;
int16 really_speed=4000;
extern int chaochequ;
void speed_control(void)
{
  int16 l_speed=0;
  int16 r_speed=0;
  int16 global_diff=0;
  
  l_speed=get_left_speed();
  r_speed=get_righ_speed();
  static int16 expect_speed=0;

  
  car_p.diff=((car_p.diff/10)*10);
  global_diff=(car_p.diff>=60?60:(int16)car_p.diff);
  global_diff=(car_p.diff<=(-60)?(-60):(int16)car_p.diff);
  
  
  //偏差小的时候不处理
  if(global_diff>=0){//去除小于10的
    if(global_diff<=10){
      global_diff=0;
    }
  }
  
  if(global_diff<0){//去除大于-10的
    if(global_diff>=(-10)){
      global_diff=0;
    }
  }  
  
  
  expect_speed=(int16)(int04 - (global_diff*global_diff )*(int08/100.0f));
  //2200  60 根据偏差降速
  
  /*超车程序 暂时屏蔽
  int16 length=car_p.distance/10;//超声波距离
  
  length=(length>=140?140:length);
  length=(length<=0?0:length);
  
  if(car_p.car_c==non_circle&&car_p.car_t==non_ten&&car_p.ten_left==0&&car_p.ten_righ==0){
    if(car_p.car_fob==front){
      expect_speed=(int16)(int04 - ( global_diff*global_diff )*int08/100.0f );
    }
    if(car_p.car_fob==behind){
      if(length>=80){
        expect_speed=(int16)((int04+(length-80)*5) - ( global_diff*global_diff )*int08/100.0f );
      }
      else{
        expect_speed=expect_speed-(80-length)*5;
      }
    }
  }
  */
  
  //期盼速度限甫
  expect_speed=(expect_speed<=2800?2800:expect_speed);
  expect_speed=(expect_speed>=3800?3800:expect_speed);
  
  
  if((expect_speed-expect_speed_last)>=0){
    expect_speed=((expect_speed-expect_speed_last)>=200?(expect_speed_last+200):expect_speed);
  }
  else{
    expect_speed=((expect_speed-expect_speed_last)<=(-200)?(expect_speed_last-200):expect_speed);
  }
  
  really_speed=((l_speed+r_speed)/2);
  really_speed=(really_speed>=3800?3800:really_speed);
  really_speed=(really_speed<=-500?(-500):really_speed);
  
      if(cur_L_real_delay_flag)//准确识别后
  {
          expect_speed =2500; 
  }
  
  if(cur_R_real_delay_flag)
  {
     expect_speed =2500; 
  }
  
  
    if(chaochequ&&!cur_R_real_delay_flag&&!cur_L_real_delay_flag&&!cur_R_ready_rest_flag&&!cur_L_ready_rest_flag)
  {
    expect_speed=0;
  }
//  if(chaochequ)
//  {
//    expect_speed=0;
//  }
  //static float distance=0;
  
  /*终点判断
  if(int11==1&&car_p.car_fob==behind){  
    if(car_p.starting_l_p.starting_line_flag==1&&car_p.starting_l_p.starting_line_count==1){
      car_p.end_line_stop_flag=1;
      car_p.starting_l_p.starting_line_flag=0;
    }
  
    if(car_p.end_line_stop_flag==1){
      distance+=really_speed*0.02f;
    }

    if(distance>=int10){
      distance=0;
      car_p.end_line_stop_flag=0;//终点
      car_p.car_s=motor_stop;
    }
    
  }
  */
  
  /*
  if(int11==1&&car_p.car_fob==front){
    if(car_p.starting_l_p.starting_line_flag==1&&car_p.starting_l_p.starting_line_count==1){
      car_p.end_line_stop_flag=1;
      car_p.starting_l_p.starting_line_flag=0;
    }
  
    if(car_p.end_line_stop_flag==1){
      distance+=really_speed*0.02f;
    }

    if(distance>=int10){
      int09=8000;
    }
    if(distance>=(int10+500)){
      distance=0;
      car_p.end_line_stop_flag=0;    
      uart_putchar(UART1,0x70);  
      uart_putchar(UART1,0x70);
    }    
  }*/
  

  /*
  if(car_p.start_flag==1){//开车标志位
    if(car_p.car_fob==front&&car_p.starting_l_p.starting_line_count==1){
      if(car_p.starting_l_p.starting_line_flag==1&&car_p.starting_l_p.starting_line_count==1){
        car_p.end_line_stop_flag=1;
        car_p.starting_l_p.starting_line_flag=0;
      }
    
      if(car_p.end_line_stop_flag==1){
        distance+=really_speed*0.02f;
      }
      if(distance>=150){
        distance=0;
        car_p.end_line_stop_flag=0; 
        car_p.start_flag=0;
        uart_putchar(UART1,0x72);  
        uart_putchar(UART1,0x72);
      }       
    }
  }*/
  
  
  /*
  if(car_p.starting_l_p.starting_line_count==2&&car_p.starting_l_p.starting_line_flag==1){
    static float stop_distance=0;
    stop_distance+=really_speed*0.02f;
    if(car_p.car_fob==front){
      if(stop_distance>=900){
        car_p.car_s=motor_stop;
        car_p.starting_l_p.starting_line_flag=0;
      }
    }
    if(car_p.car_fob==behind){
      if(stop_distance>=300){
        car_p.car_s=motor_stop;
        car_p.starting_l_p.starting_line_flag=0;
      }
    }    
  }*/
  
  
//  if(car_p.far_end_line==60){//坡道
//    expect_speed=3000;
//  }
  
  /*
  if(car_p.ten_left==1||car_p.ten_righ==1){
    if(car_p.out_ten_flag==0){
      expect_speed=2000;
    }
  }*/
  
  /*
  if(car_p.out_ten_flag==1&&car_p.car_s==motor_run){
    if(car_p.car_fob==behind){
      expect_speed=2000;
    }
    if(car_p.car_fob==front){
      expect_speed=4000;
    }
  }*/
  
  /*
  if(car_p.car_s==motor_add){
    static uint8 count=0;
    count++;
    if(count==3){
      car_p.car_s=motor_stop;
      count=0;
    }
  }*/
 
  
  
//  if(car_p.car_s!=motor_close){
//    if(car_p.car_s==motor_stop){
//        expect_speed=0;
//    }
//  }
  
  
  
 /* if(car_p.back_flag==1){//倒车
    
    static uint8 count=0;
    count++;
    
    if(count<5){
      speed_of_z=0;
    }
    
    if(count>=5){
      count=5;
    }
    
    if(count==5){
      static float ten_out_angle=0;
      
      ten_out_angle+=(speed_of_z*0.02f);
      expect_speed=-3800;
      
      if( abs((int)(ten_out_angle))<30 ){
        //expect_speed=-700;
      }
      else{
        car_p.car_s=motor_run;
        car_p.back_flag=0;   
        ten_out_angle=0;
        count=0;
      }
    }
    
  }*/
  
  
//  if(car_p.car_s==motor_close){
//    expect_speed=0;
//  }

  
  expect_speed_last=expect_speed;
  
  error[0]=(expect_speed-really_speed);
  
  
  
  static int16 pwm_p=0;
  static int16 pwm_i=0;
  static int16 pwm_d=0;
  
  pwm_p=0;
  pwm_i=0;
  pwm_d=0;
  
  
  //计算PD
  if((error[0]-error[1])>=0){//误差越来越大
    pwm_p=(int16)(  float09*(error[0]-error[1])>=9500?9500:(float09*(error[0]-error[1]))  );
  }//1200 
  else{//误差越来越小
    pwm_p=(int16)(  float09*(error[0]-error[1])<=(-9500)?(-9500):(float09*(error[0]-error[1]))  );
  }
  
  if(( error[0] - 2*error[1] + error[2] ) >=0 ){
    pwm_d=(int16)( (float11*( error[0] - 2*error[1] + error[2]  ))>=9500?9500:(float11*( error[0] - 2*error[1] + error[2]  )) );
  }
  else{
    pwm_d=(int16)( (float11*( error[0] - 2*error[1] + error[2]  ))<=(-9500)?(-9500):(float11*( error[0] - 2*error[1] + error[2]  )) );
  }
  
  
  int8 index=0;
  
  if(abs(error[0])<=200){
    index=1;
  }
  else{
    index=0;
  }
  
  //计算i
  if(error[0]>=0){
    pwm_i=(int16)( (float10*error[0])>=500?500:(float10*error[0]) );
  }
  else{
    pwm_i=(int16)( (float10*error[0])<=(-500)?(-500):(float10*error[0]) );
  }
  
  pwm=pwm+pwm_p+pwm_d+index*pwm_i;//累计
  
  if(error[0]>500){
    pwm=3800;
  }
  if(error[0]<(-500)){
    pwm=-3800;
  }  
    
  
  error[2]=error[1];
  error[1]=error[0];
  
  pwm=(pwm>=3800?3800:pwm);
  pwm=(pwm<=(-3800)?(-3800):pwm);
  
  int16 left_speed=0;
  int16 righ_speed=0;

  if(pwm>=0){
      left_speed=pwm;
      righ_speed=pwm;
    left_speed=(left_speed>=3800?3800:left_speed);
    righ_speed=(righ_speed>=3800?3800:righ_speed);
    left_speed=(left_speed<=200?200:left_speed);
    righ_speed=(righ_speed<=200?200:righ_speed);   
    
    ftm_pwm_duty(FTM0, FTM_CH4,left_speed);
    ftm_pwm_duty(FTM0, FTM_CH5,0);
    ftm_pwm_duty(FTM0, FTM_CH6,righ_speed);
    ftm_pwm_duty(FTM0, FTM_CH7,0);        
  }
  else{
      left_speed=pwm;
      righ_speed=pwm;
    left_speed=(left_speed<=(-3800)?(-3800):left_speed);
    righ_speed=(righ_speed<=(-3800)?(-3800):righ_speed);
    ftm_pwm_duty(FTM0, FTM_CH4,0);
    ftm_pwm_duty(FTM0, FTM_CH5,(-left_speed));
    ftm_pwm_duty(FTM0, FTM_CH6,0);
    ftm_pwm_duty(FTM0, FTM_CH7,(-righ_speed));            
  }
}







int16 get_left_speed(void)
{
  static int16 speed_left=0;
  static int16 speed_left_last=0;
  
  speed_left=-(int16)(Ratio_Encoder_Left*ftm_quad_get(FTM2))/2;
  if(speed_left>-200&&speed_left<200)
    speed_left=0;
  //oled_print_short(70,2,speed_left);
  speed_left=(speed_left>9000?9000:speed_left);
  if(speed_left>speed_left_last){
    speed_left=((speed_left-speed_left_last)>200?(speed_left_last+200):speed_left);
  }
  else{
    speed_left=((speed_left-speed_left_last)<(-200)?(speed_left_last-200):speed_left);
  }
  speed_left_last=speed_left;
   
  ftm_quad_clean(FTM2);  
  
  return speed_left;
}

int16 get_righ_speed(void)
{
  static int16 speed_righ=0;
  static int16 speed_righ_last=0;  
  
  uint16 data;
  data=lptmr_pulse_get();
  
  speed_righ=(int16)(Ratio_Encoder_Righ*(data));
  //OutData[3]=speed_righ;
  if(PTA24_IN!=0){
    speed_righ=-speed_righ;
  }
  //oled_print_short(70,4,speed_righ);
    
  speed_righ=(speed_righ>9000?9000:speed_righ);
  if(speed_righ>speed_righ_last){
    speed_righ=((speed_righ-speed_righ_last)>200?(speed_righ_last+200):speed_righ);
  }
  else{
    speed_righ=((speed_righ-speed_righ_last)<(-200)?(speed_righ_last-200):speed_righ);
  }
  speed_righ_last=speed_righ;
   
  lptmr_pulse_clean();  
  
  return speed_righ;
}

void speed_control_change(void)
{
  int16 l_speed=0;
  int16 r_speed=0;
  int16 global_diff=0;
  int16 expect_speed=0;
  int16 really_speed=0;  
  int16 expect_speed_l=0;
  int16 expect_speed_r=0;
  
  l_speed=get_left_speed();
  r_speed=get_righ_speed();
  
  l_speed=(l_speed>=4000?4000:l_speed);
  l_speed=(l_speed<=-500?(-500):l_speed); 
  r_speed=(r_speed>=4000?4000:r_speed);
  r_speed=(r_speed<=-500?(-500):r_speed);   
    
  global_diff=(car_p.diff>=60?60:(int16)car_p.diff);
  global_diff=(car_p.diff<=(-60)?(-60):(int16)car_p.diff);
  expect_speed=(int16)(int04 - ( global_diff*global_diff )*int08/100.0f );
  
  expect_speed=(expect_speed/50)*50;
  expect_speed=(expect_speed<=2500?2500:expect_speed);
  expect_speed=(expect_speed>=4000?4000:expect_speed); 
  
  really_speed=((l_speed+r_speed)/2);
  really_speed=(really_speed>=4000?4000:really_speed);
  really_speed=(really_speed<=-500?(-500):really_speed);      

  
  if(angle>=80){
    expect_speed_l=expect_speed+l_speed-expect_speed;
    expect_speed_r=expect_speed-l_speed+expect_speed;
    
    expect_speed_l=(expect_speed_l>=6000?6000:expect_speed_l);
    expect_speed_l=(expect_speed_l<=2500?(2500):expect_speed_l);     
    expect_speed_r=(expect_speed_r>=6000?6000:expect_speed_r);
    expect_speed_r=(expect_speed_r<=2500?(2500):expect_speed_r);         
  }
  else if(angle<=(-80)){
    expect_speed_l=expect_speed-r_speed+expect_speed;
    expect_speed_r=expect_speed+r_speed-expect_speed;    
    
    expect_speed_l=(expect_speed_l>=6000?6000:expect_speed_l);
    expect_speed_l=(expect_speed_l<=2500?(2500):expect_speed_l);     
    expect_speed_r=(expect_speed_r>=6000?6000:expect_speed_r);
    expect_speed_r=(expect_speed_r<=2500?(2500):expect_speed_r);        
  }
  else{
    expect_speed_l=expect_speed;
    expect_speed_r=expect_speed;        
  }
  
  
  
  if(car_p.car_s==motor_close){
    expect_speed_l=0;
    expect_speed_r=0;
  }  
  
  int16 l_pwm;
  int16 r_pwm;
  
  l_pwm=pid_left(expect_speed_l,l_speed);
  r_pwm=pid_righ(expect_speed_r,r_speed);
  
  if(angle>=80||angle<=(-80)){
    if(l_pwm>=0){
      ftm_pwm_duty(FTM0, FTM_CH4,0);
      ftm_pwm_duty(FTM0, FTM_CH5,l_pwm);      
    }
    else{
      ftm_pwm_duty(FTM0, FTM_CH4,-l_pwm);
      ftm_pwm_duty(FTM0, FTM_CH5,0);          
    }
    
    if(r_pwm>=0){
      ftm_pwm_duty(FTM0, FTM_CH6,0);
      ftm_pwm_duty(FTM0, FTM_CH7,r_pwm);         
    }
    else{
      ftm_pwm_duty(FTM0, FTM_CH6,-r_pwm);
      ftm_pwm_duty(FTM0, FTM_CH7,0);          
    }       
  }
  else{
    if( ((l_pwm+r_pwm)/2)>=0 ){
      ftm_pwm_duty(FTM0, FTM_CH4,0);
      ftm_pwm_duty(FTM0, FTM_CH5,(l_pwm+r_pwm)/2);      
      ftm_pwm_duty(FTM0, FTM_CH6,0);
      ftm_pwm_duty(FTM0, FTM_CH7,(l_pwm+r_pwm)/2);          
    }
    else{
      ftm_pwm_duty(FTM0, FTM_CH4,-(l_pwm+r_pwm)/2);
      ftm_pwm_duty(FTM0, FTM_CH5,0);    
      ftm_pwm_duty(FTM0, FTM_CH6,-(l_pwm+r_pwm)/2);
      ftm_pwm_duty(FTM0, FTM_CH7,0);         
    }  
  }
  
}

int16 pid_left(int16 expect_speed,int16 really_speed)
{
  static int16 error_l[3]={0,0,0};
  
  error_l[0]=(expect_speed-really_speed);
  
  static int16 pwm_p=0;
  static int16 pwm_i=0;
  static int16 pwm_d=0;
  
  pwm_p=0;
  pwm_i=0;
  pwm_d=0;
  
  if((error_l[0]-error_l[1])>=0){
    pwm_p=(int16)(  float09*(error_l[0]-error_l[1])>=9500?9500:(float09*(error_l[0]-error_l[1]))  );
  }
  else{
    pwm_p=(int16)(  float09*(error_l[0]-error_l[1])<=(-9500)?(-9500):(float09*(error_l[0]-error_l[1]))  );
  }
  
  if( ( error_l[0] - 2*error_l[1] + error_l[2]  ) >=0 ){
    pwm_d=(int16)( (float11*( error_l[0] - 2*error_l[1] + error_l[2]  ))>=9500?9500:(float11*( error_l[0] - 2*error_l[1] + error_l[2]  )) );
  }
  else{
    pwm_d=(int16)( (float11*( error_l[0] - 2*error_l[1] + error_l[2]  ))<=(-9500)?(-9500):(float11*( error_l[0] - 2*error_l[1] + error_l[2]  )) );
  }
  
  
  int8 index=0;
  
  if(abs(error_l[0])<=200){
    index=1;
  }
  else{
    index=0;
  }
  
  if(error_l[0]>=0){
    pwm_i=(int16)( (float10*error_l[0])>=500?500:(float10*error_l[0]) );
  }
  else{
    pwm_i=(int16)( (float10*error_l[0])<=(-500)?(-500):(float10*error_l[0]) );
  }
  
  static int16 pwm_left=0;
  pwm_left=pwm_left+pwm_p+pwm_d+index*pwm_i;
  
  
  if(error_l[0]>500){
    pwm_left=9500;
  }
  if(error_l[0]<(-800)){
    pwm_left=-9500;
  }  
    
  
  error_l[2]=error_l[1];
  error_l[1]=error_l[0];
  
  pwm_left=(pwm_left>=9500?9500:pwm_left);
  pwm_left=(pwm_left<=(-9500)?(-9500):pwm_left);  
  
  return pwm_left;
}

int16 pid_righ(int16 expect_speed,int16 really_speed)
{
  static int16 error_r[3]={0,0,0};
  
  error_r[0]=(expect_speed-really_speed);
  
  static int16 pwm_p=0;
  static int16 pwm_i=0;
  static int16 pwm_d=0;
  
  pwm_p=0;
  pwm_i=0;
  pwm_d=0;
  
  if((error_r[0]-error_r[1])>=0){
    pwm_p=(int16)(  float09*(error_r[0]-error_r[1])>=9500?9500:(float09*(error_r[0]-error_r[1]))  );
  }
  else{
    pwm_p=(int16)(  float09*(error_r[0]-error_r[1])<=(-9500)?(-9500):(float09*(error_r[0]-error_r[1]))  );
  }
  
  if( ( error_r[0] - 2*error_r[1] + error_r[2]  ) >=0 ){
    pwm_d=(int16)( (float11*( error_r[0] - 2*error_r[1] + error_r[2]  ))>=9500?9500:(float11*( error_r[0] - 2*error_r[1] + error_r[2]  )) );
  }
  else{
    pwm_d=(int16)( (float11*( error_r[0] - 2*error_r[1] + error_r[2]  ))<=(-9500)?(-9500):(float11*( error_r[0] - 2*error_r[1] + error_r[2]  )) );
  }
  
  
  int8 index=0;
  
  if(abs(error_r[0])<=200){
    index=1;
  }
  else{
    index=0;
  }
  
  if(error_r[0]>=0){
    pwm_i=(int16)( (float10*error_r[0])>=500?500:(float10*error_r[0]) );
  }
  else{
    pwm_i=(int16)( (float10*error_r[0])<=(-500)?(-500):(float10*error_r[0]) );
  }
  
  static int16 pwm_righ=0;
  pwm_righ=pwm_righ+pwm_p+pwm_d+index*pwm_i;
  
  
  if(error_r[0]>500){
    pwm_righ=9500;
  }
  if(error_r[0]<(-800)){
    pwm_righ=-9500;
  }  
    
  
  error_r[2]=error_r[1];
  error_r[1]=error_r[0];
  
  pwm_righ=(pwm_righ>=9500?9500:pwm_righ);
  pwm_righ=(pwm_righ<=(-9500)?(-9500):pwm_righ);  
  
  return pwm_righ;
}