#include "init.h"

extern volatile IMG_STATUS_e  ov7725_eagle_img_flag;
void car_init(void)
{
  DisableInterrupts;             //禁止总中断        
  
  NVIC_SetPriorityGrouping(4);      //设置中断优先级
  
  NVIC_SetPriority(UART1_RX_TX_IRQn,0);   //串口通讯中断
  NVIC_SetPriority(PORTA_IRQn,4);         //配置优先级  摄像头采集中端
  NVIC_SetPriority(PORTB_IRQn,0);         //配置优先级  超声波中断
  NVIC_SetPriority(DMA0_IRQn,2);         //配置优先级     
  NVIC_SetPriority(PORTD_IRQn,1);         //配置优先级    红外中断
  NVIC_SetPriority(PORTC_IRQn,1);         //配置优先级    开关中断
  
  /*红外中断
  infrared_init();  
  set_vector_handler(PORTD_VECTORn,Infrared_Handler);
  enable_irq(PORTD_IRQn);  
  */
  
  //flash_init();     //flash初始化 芯片好像有问题
 
  
  //oled初始化
  LCD_Init();       
   
  
  //摄像头初始化
  camera_init(imgbuff_1);  
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    
  set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);   
  
  
  gpio_init(PTD11,GPO,1);     // led初始化
  
  //电机初始化
  ftm_pwm_init(FTM0, FTM_CH4,15000,3000); //左1   
  ftm_pwm_init(FTM0, FTM_CH5,15000,0);
  ftm_pwm_init(FTM0, FTM_CH6,15000,3000);
  ftm_pwm_init(FTM0, FTM_CH7,15000,0); 
  
 
  /* 通讯还没用
  set_vector_handler(UART1_RX_TX_VECTORn,UART1_IRQHandler);
  uart_rx_irq_en(UART1);
  */      
  
  ftm_quad_init(FTM2);                             //左编码器初始化
  lptmr_pulse_init(LPT0_ALT1,0xFFFF,LPT_Falling);  //右编码器初始化
  gpio_init(PTA24,GPI,0); //判断编码器正负
  
  
  gpio_init(PTE10,GPO,0);     //蜂鸣器
  
  gpio_init(PTD15,GPI,1);     //拨码开关初始化
  gpio_init(PTD14,GPI,1);
  gpio_init(PTD13,GPI,1);
  gpio_init(PTD12,GPI,1);     
  
  /*无线模块
  gpio_init(PTE27,GPO,0);         
  gpio_init(PTE28,GPO,1); 
  */
  
  /*超声波
  gpio_init(PTB20,GPI,0);       
  gpio_init(PTB21,GPI,0);   
  */
  
  /* 超声波初始化
  port_init(PTB20,IRQ_RISING|IRQ_FALLING |PF | ALT1 | PULLUP ); 
  set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);  
  disable_irq(PORTB_IRQn);
  */
  
  
  //五项开关
  port_init(PTC16, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTC18, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTC17, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTC19, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTD0, IRQ_FALLING |PF | ALT1 | PULLUP );  
  
  //开关
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);
  enable_irq(PORTC_IRQn);
  
  param_reset();           //初始化写入参数
  //param_in();           //参数导入 芯片问题
  
  //img_sd_init();        //sd卡初始化
  
  EnableInterrupts;
  
  ov7725_eagle_img_flag = IMG_START;      //开始采集图像
  enable_irq(PORTA_IRQn);                 //允许PTA的中断    
  
  ftm_pwm_init(FTM1, FTM_CH0,100,1490);  //舵机初始化
  
  /*没用
  if(int05==1){                      //是否关闭红外中断
    disable_irq(PORTD_IRQn);
  }
  */
  
  pit_delay_ms(PIT3,1000);
  disable_irq(PORTB_IRQn);//超声波
}