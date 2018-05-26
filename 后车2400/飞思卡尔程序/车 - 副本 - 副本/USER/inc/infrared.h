#ifndef _infrared_
#define _infrared_

/*
 * ����ͷ�ļ�
 */
#include "common.h"
#include "include.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_PIT.h"
#include "picture_deal.h"
#include "VCAN_NRF24L0.h"  
#include "bsp_my_oled.h"  
#define  IR_PIN    PTD10

//����ID����
#define BT_UP_PRESS  (1)
#define BT_DN_PRESS  (2)
#define BT_LE_PRESS  (3)
#define BT_RI_PRESS  (4)
#define BT_OK_PRESS  (5)

typedef struct
{
  uint8 debug_flag;               //����ģʽ��־λ
  uint8 motor_run_flag;           //������б�־λ
  uint8 steer_run_flag;           //������б�־λ
  uint8 picture_run_flag;         //ͼ��ɼ���־λ
}debug_param;


extern uint8 irkey_value;                  //�����ֵ
extern debug_param car_debug;              //��������

extern void infrared_init(void);
extern void ir_decoding(void);
uint16 get_high_time(void);
uint16 get_low_time(void);
void code_mapping(void);

extern uint8 GetIRKey(void);
extern void SetIRKey(unsigned char key);
#endif  /*_infrared_*/