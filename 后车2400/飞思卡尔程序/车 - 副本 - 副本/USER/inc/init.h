#ifndef _init_
#define _init_

#include "common.h"
#include "include.h"
#include "isr.h"
#include "infrared.h"
#include "VCAN_camera.h" 
#include "picture_deal.h"
#include "bsp_oled.h"
#include "bsp_my_oled.h"
#include "mpu6050.h"
#include "flash_param.h"
#include "VCAN_NRF24L0.h"
#include "ui_debug.h"
#include "vcan_img2sd.h"
#include "get_car_angle.h"

extern void car_init(void);

#endif  /*_init_*/