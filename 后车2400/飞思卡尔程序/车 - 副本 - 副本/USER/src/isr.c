#include "isr.h"
extern uint8*    ov7725_eagle_img_buff;
extern uint8 flag_imgbuff;


//������ǳ�ѧ�߽������������������ֱ�ӿ�PORTA_IRQHandler  
void Infrared_Handler(void)
{
  uint8  n = 0;   
  n = 10;
  if(PORTD_ISFR & (1 << n))       
  {
    PORTD_ISFR  = (1 << n);    
    ir_decoding();            // �����������  �������
  } 
  
  if(PORTD_ISFR & (1 << 0))       
  {
    PORTD_ISFR  = (1 << 0);    
    SetIRKey(5);
    DELAY_MS(20);
  }  
}


void PORTA_IRQHandler(void)         // ͼ��ɼ�������
{
  int i,j;
  int p[120]=0;
  uint8  n = 0;    //���ź�
  uint32 flag = PORTA_ISFR;
  PORTA_ISFR  = ~0;                                //���жϱ�־λ

  n = 29;                                         //���ж�
  if(flag & (1 << n)){                            //PTA29�����ж�
    static int16 count=0;
    camera_vsync(); 						      //��ʼ�ɼ�ͼ��(����ͼ���ַ)
    
    flag_imgbuff != 0?img_extract(img, imgbuff_1,CAMERA_SIZE):img_extract(img,imgbuff_2,CAMERA_SIZE); 
    

    if(car_p.picture_flag!=0){      //ͼ��ʶ���־
      picture();              // ͼ���� ����ʶ��
    }
    
    
    
    if(car_debug.picture_run_flag==1){  
     steer_control();        // �������
      count++;
      
      if(count==3){    
        speed_control();     // �ٶȿ���   
        count=0;
      }      
    }
}

}


//����ͷDMA0�жϷ�����
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
    
    camera_dma();      // ����ͷ����ͼ����  ����ͷ�������Դ�  ֱ��ʹ��
    camera_get_img();  // ����ѭ���ȴ��ɼ����   ����ͷ�ɼ�ͼ����  ����ͷ�������Դ�  ֱ��ʹ��
    flag_imgbuff =! flag_imgbuff;
    ov7725_eagle_img_buff = (flag_imgbuff == 0?imgbuff_1:imgbuff_2);//��ʼ��ͼ���ַ     
}





void UART1_IRQHandler(void)     // ͨѶ���� ��ѧ�߿��Բ��ù�
{
  char ch=0;
  static char last_ch=0;
  
  
  if(uart_query(UART1) == 1)   //�������ݼĴ�����
  {
    uart_getchar(UART1, &ch);                    //���޵ȴ�����1���ֽ�
    
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
    SetIRKey(1);//���ú��ⰴ��
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