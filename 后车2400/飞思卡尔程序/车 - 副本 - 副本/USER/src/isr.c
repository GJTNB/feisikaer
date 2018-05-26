#include "isr.h"
extern uint8*    ov7725_eagle_img_buff;
extern uint8 flag_imgbuff;


//如果你是初学者建议跳过下面这个函数直接看PORTA_IRQHandler  
void Infrared_Handler(void)
{
  uint8  n = 0;   
  n = 10;
  if(PORTD_ISFR & (1 << n))       
  {
    PORTD_ISFR  = (1 << n);    
    ir_decoding();            // 红外调整参数  红外键码
  } 
  
  if(PORTD_ISFR & (1 << 0))       
  {
    PORTD_ISFR  = (1 << 0);    
    SetIRKey(5);
    DELAY_MS(20);
  }  
}


void PORTA_IRQHandler(void)         // 图像采集处理部分
{
  int i,j;
  int p[120]=0;
  uint8  n = 0;    //引脚号
  uint32 flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                //清中断标志位

  n = 29;                                         //场中断
  if(flag & (1 << n)){                            //PTA29触发中断
    static int16 count=0;
    camera_vsync(); 						      //开始采集图像(设置图像地址)
    
    flag_imgbuff != 0?img_extract(img, imgbuff_1,CAMERA_SIZE):img_extract(img,imgbuff_2,CAMERA_SIZE); 
    

    if(car_p.picture_flag!=0){      //图像识别标志
      picture();              // 图像处理 赛道识别
    }
    
    
    
    if(car_debug.picture_run_flag==1){  
     steer_control();        // 舵机控制
      count++;
      
      if(count==3){    
        speed_control();     // 速度控制   
        count=0;
      }      
    }
}

}


//摄像头DMA0中断服务函数
void DMA0_IRQHandler(void)
{
    if(car_debug.picture_run_flag==1){ 
      
      
      if(car_p.car_c==non_circle){
        if(car_p.car_fob==front){
          disable_irq(PORTB_IRQn);
          PTE27_OUT=0;
        }
        if(car_p.car_fob==behind){
          disable_irq(PORTB_IRQn);
          PTE27_OUT=0;
        }
      }
      else{
        disable_irq(PORTB_IRQn);
        PTE27_OUT=0;
      }
    }
    
    camera_dma();      // 摄像头传输图像函数  摄像头函数库自带  直接使用
    camera_get_img();  // 内有循环等待采集完成   摄像头采集图像函数  摄像头函数库自带  直接使用
    flag_imgbuff =! flag_imgbuff;
    ov7725_eagle_img_buff = (flag_imgbuff == 0?imgbuff_1:imgbuff_2);//初始化图像地址     
}





void UART1_IRQHandler(void)     // 通讯部分 初学者可以不用管
{
  char ch=0;
  static char last_ch=0;
  
  
  if(uart_query(UART1) == 1)   //接收数据寄存器满
  {
    uart_getchar(UART1, &ch);                    //无限等待接受1个字节
    
    if(ch==0x68&&last_ch==0x68){
      last_ch=0;
      return;
    }
    if(ch==0x69&&last_ch==0x69){
      last_ch=0;
      return;
    }
    if(ch==0x71&&last_ch==0x71){
      last_ch=0;
      return;
    } 
    if(ch==0x72&&last_ch==0x72){
      last_ch=0;
      return;
    }        
    if(ch==0x73&&last_ch==0x73){
      last_ch=0;
      return;
    }         
    
    if(ch==0x68){
      car_p.picture_flag=2; 
      uart_rx_irq_dis(UART1);
    }
    if(ch==0x69){
      car_p.two_way_find_circle=0;
      car_p.circle_buff_flag=1;
      uart_rx_irq_dis(UART1);
    }
    if(ch==0x71){
      car_p.obstacle_flag=1;
      car_p.obstacle_buff_flag=1;
    }
    if(ch==0x72){
      param_reset();
      car_p.picture_flag=1;
      car_p.car_s=motor_run;
    }    
    if(ch==0x73){
      car_p.back_flag=1;
    }
    
    
    last_ch=ch;
  }  
}

void PORTC_IRQHandler(void)
{ 
  if(PORTC_ISFR & (1 << 16))       
  {
    PORTC_ISFR  = (1 << 16);    
    SetIRKey(1);//设置红外按键
    DELAY_MS(20);
  }
  
  if(PORTC_ISFR & (1 << 18))       
  {
    PORTC_ISFR  = (1 << 18);    
    SetIRKey(2);
    DELAY_MS(20);    
  }  

  if(PORTC_ISFR & (1 << 17))       
  {
    PORTC_ISFR  = (1 << 17);   
    SetIRKey(3);
    DELAY_MS(20);    
  }  

  if(PORTC_ISFR & (1 << 19))       
  {
    PORTC_ISFR  = (1 << 19); 
    SetIRKey(4);
    DELAY_MS(20);
  }  
  
}


void PORTB_IRQHandler(void)
{
  uint32 chaoshengboTime=0;
  static uint32 ABDistance=0;
  
  if(PORTB_ISFR & (1 << 20)){       
    PORTB_ISFR  = (1 << 20);
    
    if(PTB20_IN==1){
      pit_time_start(PIT3); 
    }
    if(PTB20_IN==0){
      chaoshengboTime=pit_time_get_us(PIT3);
      if(chaoshengboTime>5000){
        car_p.distance=1400;
        return;
      }
      else{
        ABDistance=(int16)((float)chaoshengboTime*340/1000.0f);
      }
      if(PTD12_IN!=0){
        oled_fill(0);
        oled_print_16x8short(0,0,ABDistance/10); 
      }      
      
      if(ABDistance>=car_p.distance){
        car_p.distance=((ABDistance-car_p.distance>=20)?(car_p.distance+20):ABDistance);
      }
      else{
        car_p.distance=ABDistance;
      }

    }
    
    
  }
      
}