#ifndef _isr_
#define _isr_

#include "common.h"
#include "infrared.h"
#include "VCAN_camera.h" 
#include "picture_deal.h"
#include "VCAN_NRF24L0.h"
#include "bsp_my_oled.h"
#include "bsp_oled.h"
#include "steer_control.h"
#include "speed_control.h"

extern void Infrared_Handler(void);
extern void DMA0_IRQHandler(void);
extern void PORTA_IRQHandler(void);
extern void UART1_IRQHandler(void);
extern void PORTC_IRQHandler(void);
extern void PORTB_IRQHandler(void);
#endif  /*_isr_*/