#include "common.h"
#include "include.h"
#include "init.h"
#include "picture_deal.h"

/*Բ��Ԥ�ж�*/
extern uint16 cur_L_ready_flag;         //��Բ��Ԥ�жϳ�ʼʶ���־
extern uint16 cur_L_ready_delay_flag;   //��Բ��Ԥ�жϳ�и��־
extern uint16 cur_L_ready_time_flag;   //��Բ��Ԥ�жϼ�ʱ����
extern uint16 cur_L_ready_rest_flag;    //��Բ��Ԥ�жϸ�λ����
extern uint16 cur_R_ready_flag;         //��Բ��Ԥ�жϳ�ʼʶ���־
extern uint16 cur_R_ready_delay_flag;   //��Բ��Ԥ�жϳ�и��־
extern uint16 cur_R_ready_time_flag;   //��Բ��Ԥ�жϼ�ʱ����
extern uint16 cur_R_ready_rest_flag;    //��Բ��Ԥ�жϸ�λ����
/*Բ��׼ȷʶ��*/
extern uint16 cur_L_real_flag;          //��Բ��׼ȷ�ж�ʶ���־
extern uint16 cur_L_real_delay_flag;    //��Բ��׼ȷ��иʶ���־
extern uint16 cur_L_real_rest_flag;     //��Բ��׼ȷ��λʶ���־
extern uint16 cur_L_real_time_flag;    //��Բ��׼ȷ��иʶ���־
extern uint16 cur_L_real_time_flag1;   //��Բ��׼ȷ��иʶ���־1
extern uint16 cur_L_real_delay_flag2;
extern uint8 cur_R_real_flag;          //��Բ��׼ȷ�ж�ʶ���־
extern uint8 cur_R_real_delay_flag;    //��Բ��׼ȷ��иʶ���־
extern uint8 cur_R_real_rest_flag;     //��Բ��׼ȷ��λʶ���־
extern uint16 cur_R_real_time_flag;    //��Բ��׼ȷ��иʶ���־
extern uint16 cur_R_real_time_flag1;   //��Բ��׼ȷ��иʶ���־1
// ������ͨ���ж�ʵ�ֵģ����еĿ��Ƴ����� user/isr/isr.c�ļ���
// �ڸսӴ����ܳ�ʱ����Ҫʵ�ֵľ����������ܹ���������
// �����������������Ҫ�����Զ������������˵����Ҫ�Ŀ��ľ���ͼ������
// ͼ��������Ҫ���������ߵ���Ѱ ��Ҳ���Ǳ��ߵ���Ѱ����Ѱ��ƫ�Ȼ�������ڶ������
// ����˵���˱��ߵ���Ѱ��������ƣ��������в����㶼����Ҫ��
// �����ܹ�ʵ������ͨ���������������ٿ��ҵ�����ʶ��  ��ͨ������ָ������Բ����ʮ���䣩

// ����ʵ������Ĳ��ֺ�����������������̵�������ԶҪ�ǵõ�һ�仰���ǣ�
// ��������ô��������Ҫ������ƫ��ƽ���Ļ����ϣ�ƫ��ƽ����Զ�ǻ���������û��ƽ����ƫ���ٶ඼��û�õ�

// Ӳ�����ã�
// k60DN512 ���κ�һ���̼ҵ�K60�����������������������û���κ����� ��
// ɽ��ӥ������ͷ
// 0.96��oled  ���ܹ�I2C�����Ļ���GPIOģ��I2C�����ģ�
// ���뿪��
// ������
// ������������еĺ��ⶼ��ָ�������ͷ��Ҳ������ƽʱʹ�õ�ң����ң����һ�����õ�����Ա���



void  main(void)
{
  int k;  //����ͼ���л��ı�־λ
  car_init();             //������ʼ��
  car_debug.picture_run_flag=1;  //����ͷͼ�������ɼ�

  while(1)
  {
   // flag_imgbuff != 0?img_sd_save(imgbuff_1,CAMERA_SIZE):img_sd_save(imgbuff_2,CAMERA_SIZE);//�ڴ濨��ȡ
    
    if(PTD12_IN==0){                  //���뿪�ز���
      Draw_BMP(0,0,127,7,bmp_buff);    //oled��ʾ����ͼ��
      k=1;
    }
    else{  
        oled_print_short(50,1,cur_L_ready_flag);
        oled_print_short(50,2,cur_L_ready_delay_flag);
        oled_print_short(50,3,cur_L_ready_rest_flag);
        oled_print_short(50,5,cur_L_real_flag);
        oled_print_short(50,7,cur_L_real_rest_flag);
      if(k==1)
      {
      oled_fill(0x00);
      k=0;
      }
    }
  }
}

