#include "infrared.h"

uint8 user_code=0;
uint8 ircode[3]={0};
uint8 irkey_value=0;

debug_param car_debug;


void infrared_init(void)
{
  port_init(IR_PIN, IRQ_FALLING |PF | ALT1 | PULLUP );
}

void ir_decoding(void)
{
  uint8 i=0,j=0;
  uint16 time=0;
  uint8 byte=0;
  
  time = get_low_time();               //红外获取低电平时间
  if((time<3700) || (time>4200)){
    return;
  }
  
  time = get_high_time();               //红外获取低电平时间
  if((time<3700) || (time>4200)){
    return;
  }
  
  for(i=0;i<=2;i++){
    for(j=0;j<=7;j++){
      time = get_low_time();            
      time = get_high_time();
      if((time>900)&&(time<1200)){
        byte >>= 1;
      }
      else{
        byte >>= 1; 
        byte |= 0x80; 
      }     
    }
    ircode[i] = byte; 
  }
  
  user_code = ircode[1] + ircode[2];
  
  systick_delay_ms(10);
  
#if 0
oled_fill(0);
oled_print_16x8short(0,0,user_code);  
#else
  code_mapping();                     //红外键码解算
#endif
  
}

uint16 get_high_time(void)
{
  uint16 hightime;
  pit_time_start(PIT2);
  while(gpio_get(IR_PIN)!=0);
  hightime = pit_time_get_us(PIT2);

  return hightime;    
}


uint16 get_low_time(void)
{
  uint16 lowtime;
  pit_time_start(PIT2);
  while(gpio_get(IR_PIN)==0);
  lowtime = pit_time_get_us(PIT2);

  return lowtime;    
}


//红外键码解算
void code_mapping(void)
{
  if(user_code==176){
    car_p.car_s=motor_run;
    //car_p.picture_flag=2;
  }
  else if(user_code==123){
    car_debug.steer_run_flag=~car_debug.steer_run_flag;
  }
  else if(user_code==81){
    car_debug.debug_flag=~car_debug.debug_flag;
  }
  else if(user_code==110){
    irkey_value = BT_UP_PRESS;
  }  
  else if(user_code==210){
    irkey_value = BT_OK_PRESS;
  }
  else if(user_code==155){
    irkey_value = BT_LE_PRESS;
  }
  else if(user_code==35){
    irkey_value = BT_RI_PRESS;
  }
  else if(user_code==230){
    irkey_value = BT_DN_PRESS;
  }
  else if(user_code==183){
    irkey_value = BT_UP_PRESS;
  }  
  else if(user_code==165){
    //uart_putchar(UART1,0x68);
    car_p.back_flag=1;
  }
  else if(user_code==18){
    car_p.car_s=motor_close;
    ftm_pwm_duty(FTM0, FTM_CH4,0);
    ftm_pwm_duty(FTM0, FTM_CH5,0);
    ftm_pwm_duty(FTM0, FTM_CH6,0);
    ftm_pwm_duty(FTM0, FTM_CH7,0);       
  }
}

//获得红外按键
uint8 GetIRKey(void)
{
  return irkey_value; 
}

//设置红外按键
void SetIRKey(unsigned char key)
{
    irkey_value = key;
}