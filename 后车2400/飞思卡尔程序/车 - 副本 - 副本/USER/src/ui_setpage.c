/******************************************************************************
*@file:
*@author:zrobot
*@doc:
*******************************************************************************/
#include"ui_setpage.h"
#include"button.h"

uint8 set_page_1(void)
{
	uint8 page_num = PAGE_MIN;
	int8  exit_flag = 0;
	int8  button_value = 0;

	oled_fill(0x00);
	oled_display_16x8str(40,0,"FC");
	oled_display_16x8str(40,2,"GZ");
	oled_display_16x8str(40,4,"XZL");
	oled_display_16x8str(40,6,"ZY");

	while(!exit_flag)
	{
        
        //button_value = get_button_value();
        button_value = GetIRKey();
        
        SetIRKey(0);
        
		switch(button_value)
		{
			case BT_UP_PRESS: break;
			case BT_DN_PRESS: break;
			case BT_RI_PRESS: page_num++;
			                  exit_flag = 1;
							  break;
			case BT_LE_PRESS: page_num = PAGE_MAX;
			                  exit_flag = 1;
							  break;
			default:          break;
		}
	}

    return page_num;  

}

uint8 set_page_2(struct PARAM *p_param)
{
	 return set_page_16x8common("s_or_d",   &(p_param->int01),
							    "ramp",   &(p_param->int02),
							    "ster_mid",     &(p_param->int03),
							    "expe_spd",        &(p_param->int04),
							    2
							    );
}

uint8 set_page_3(struct PARAM *p_param)
{
	 return set_page_16x8common("obs_num",   &(p_param->int05),
							    "ten_num",   &(p_param->int06),
							    "circ_num",     &(p_param->int07),
							    "sp_beilv",        &(p_param->int08),
							    3
							    );
}

uint8 set_page_4(struct PARAM *p_param)
{
	 return set_page_16x8common("int09",   &(p_param->int09),
							    "int10",   &(p_param->int10),
							    "int11",     &(p_param->int11),
							    "int12",        &(p_param->int12),
							    4
							    );
}

uint8 set_page_5(struct PARAM *p_param)
{
	 return set_page_16x8common("float01",   &(p_param->float01),
							    "float02",   &(p_param->float02),
							    "float03",     &(p_param->float03),
							    "float04",        &(p_param->float04),
							    5
							    );
}

uint8 set_page_6(struct PARAM *p_param)
{
	 return set_page_16x8common("float05",   &(p_param->float05),
							    "float06",   &(p_param->float06),
							    "float07",     &(p_param->float07),
							    "float08",        &(p_param->float08),
							    6
							    );
}

uint8 set_page_7(struct PARAM *p_param)
{
	 return set_page_16x8common("float09",   &(p_param->float09),
							    "float10",   &(p_param->float10),
							    "float11",     &(p_param->float11),
							    "float12",        &(p_param->float12),
							    7
							    );
}

uint8 set_page_8(struct PARAM *p_param)
{
	uint8 page_num = PAGE_MAX;
	int8  exit_flag = 0;
	int8  button_value = 0;

	oled_fill(0x00);
	oled_display_16x8str(5,1,"Exit set param ?");

	while(!exit_flag)
	{
          //button_value = get_button_value();
          button_value = GetIRKey();
          
          SetIRKey(0);
		switch(button_value)
		{
			case BT_UP_PRESS: break;
			case BT_DN_PRESS: break;
			case BT_RI_PRESS: page_num = PAGE_MIN;
			                  exit_flag = 1;
							  break;
			case BT_LE_PRESS: page_num--;
			                  exit_flag = 1;
							  break;
			case BT_OK_PRESS: page_num = EXIT_SET;
							  exit_flag = 1;
                              param_save(p_param);
							  break;
			default:          break;
		}
	}

    return page_num;  
    
}

