/******************************************************************************
*@file:oledµ÷²ÎÒ³Ãæ
*@author:zrobot
*@doc:
*******************************************************************************/
#ifndef _ui_setpage_
#define _ui_setpage_ 

#include "common.h"
#include "include.h"
#include "ui_debug_common.h"
#include "ui_debug.h"
#include "flash_param.h"
#include "bsp_my_oled.h" 
#include "infrared.h"

extern uint8 set_page_1(void);


extern uint8 set_page_2(struct PARAM *p_param);
extern uint8 set_page_3(struct PARAM *p_param);
extern uint8 set_page_4(struct PARAM *p_param);
extern uint8 set_page_5(struct PARAM *p_param);
extern uint8 set_page_6(struct PARAM *p_param);
extern uint8 set_page_7(struct PARAM *p_param);

extern uint8 set_page_8(struct PARAM *p_param);

#endif  /*_ui_setpage_*/
