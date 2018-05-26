#include "init.h"

extern volatile IMG_STATUS_e  ov7725_eagle_img_flag;
void car_init(void)
{
  DisableInterrupts;             //��ֹ���ж�        
  
  NVIC_SetPriorityGrouping(4);      //�����ж����ȼ�
  
  NVIC_SetPriority(UART1_RX_TX_IRQn,0);   //����ͨѶ�ж�
  NVIC_SetPriority(PORTA_IRQn,4);         //�������ȼ�  ����ͷ�ɼ��ж�
  NVIC_SetPriority(PORTB_IRQn,0);         //�������ȼ�  �������ж�
  NVIC_SetPriority(DMA0_IRQn,2);         //�������ȼ�     
  NVIC_SetPriority(PORTD_IRQn,1);         //�������ȼ�    �����ж�
  NVIC_SetPriority(PORTC_IRQn,1);         //�������ȼ�    �����ж�
  
  /*�����ж�
  infrared_init();  
  set_vector_handler(PORTD_VECTORn,Infrared_Handler);
  enable_irq(PORTD_IRQn);  
  */
  
  //flash_init();     //flash��ʼ�� оƬ����������
 
  
  //oled��ʼ��
  LCD_Init();       
   
  
  //����ͷ��ʼ��
  camera_init(imgbuff_1);  
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    
  set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);   
  
  
  gpio_init(PTD11,GPO,1);     // led��ʼ��
  
  //�����ʼ��
  ftm_pwm_init(FTM0, FTM_CH4,15000,3000); //��1   
  ftm_pwm_init(FTM0, FTM_CH5,15000,0);
  ftm_pwm_init(FTM0, FTM_CH6,15000,3000);
  ftm_pwm_init(FTM0, FTM_CH7,15000,0); 
  
 
  /* ͨѶ��û��
  set_vector_handler(UART1_RX_TX_VECTORn,UART1_IRQHandler);
  uart_rx_irq_en(UART1);
  */      
  
  ftm_quad_init(FTM2);                             //���������ʼ��
  lptmr_pulse_init(LPT0_ALT1,0xFFFF,LPT_Falling);  //�ұ�������ʼ��
  gpio_init(PTA24,GPI,0); //�жϱ���������
  
  
  gpio_init(PTE10,GPO,0);     //������
  
  gpio_init(PTD15,GPI,1);     //���뿪�س�ʼ��
  gpio_init(PTD14,GPI,1);
  gpio_init(PTD13,GPI,1);
  gpio_init(PTD12,GPI,1);     
  
  /*����ģ��
  gpio_init(PTE27,GPO,0);         
  gpio_init(PTE28,GPO,1); 
  */
  
  /*������
  gpio_init(PTB20,GPI,0);       
  gpio_init(PTB21,GPI,0);   
  */
  
  /* ��������ʼ��
  port_init(PTB20,IRQ_RISING|IRQ_FALLING |PF | ALT1 | PULLUP ); 
  set_vector_handler(PORTB_VECTORn ,PORTB_IRQHandler);  
  disable_irq(PORTB_IRQn);
  */
  
  
  //�����
  port_init(PTC16, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTC18, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTC17, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTC19, IRQ_FALLING |PF | ALT1 | PULLUP ); 
  port_init(PTD0, IRQ_FALLING |PF | ALT1 | PULLUP );  
  
  //����
  set_vector_handler(PORTC_VECTORn ,PORTC_IRQHandler);
  enable_irq(PORTC_IRQn);
  
  param_reset();           //��ʼ��д�����
  //param_in();           //�������� оƬ����
  
  //img_sd_init();        //sd����ʼ��
  
  EnableInterrupts;
  
  ov7725_eagle_img_flag = IMG_START;      //��ʼ�ɼ�ͼ��
  enable_irq(PORTA_IRQn);                 //����PTA���ж�    
  
  ftm_pwm_init(FTM1, FTM_CH0,100,1490);  //�����ʼ��
  
  /*û��
  if(int05==1){                      //�Ƿ�رպ����ж�
    disable_irq(PORTD_IRQn);
  }
  */
  
  pit_delay_ms(PIT3,1000);
  disable_irq(PORTB_IRQn);//������
}