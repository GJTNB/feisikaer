/******************************************************************************
*@file:从flash里面读取参数
*@author:zrobot
*@doc:
*******************************************************************************/
#include"flash_param.h"

void param_in(void)
{
  param_load(&parameter);  //从flash里读数据
  set_param(&parameter);
}

void param_load(struct PARAM * p_param)
{
    p_param->int01  = flash_read(120,0,uint32);
    p_param->int02  = flash_read(120,4,uint32);
    p_param->int03  = flash_read(120,8,uint32);
    p_param->int04  = flash_read(120,12,uint32);
    p_param->int05  = flash_read(120,16,uint32);
    p_param->int06  = flash_read(120,20,uint32);
    p_param->int07  = flash_read(120,24,uint32);
    p_param->int08  = flash_read(120,28,uint32);
    p_param->int09  = flash_read(120,32,uint32);
    p_param->int10  = flash_read(120,36,uint32);
    p_param->int11  = flash_read(120,40,uint32);
    p_param->int12  = flash_read(120,44,uint32);    
    
    p_param->float01  = flash_read(120,48,uint32);
    p_param->float02  = flash_read(120,52,uint32);
    p_param->float03  = flash_read(120,56,uint32);
    p_param->float04  = flash_read(120,60,uint32);
    p_param->float05  = flash_read(120,64,uint32);
    p_param->float06  = flash_read(120,68,uint32);
    p_param->float07  = flash_read(120,72,uint32);
    p_param->float08  = flash_read(120,76,uint32);
    p_param->float09  = flash_read(120,80,uint32);
    p_param->float10  = flash_read(120,84,uint32);
    p_param->float11  = flash_read(120,88,uint32);
    p_param->float12  = flash_read(120,92,uint32);
    
    p_param->param_flag =flash_read(119,0,uint32);
    
}

void set_param(struct PARAM *p_param)
{
    int01=(p_param->int01);
    int02=(p_param->int02);
    int03=(p_param->int03);
    int04=(p_param->int04);
    int05=(p_param->int05);
    int06=(p_param->int06);
    int07=(p_param->int07);
    int08=(p_param->int08);
    int09=(p_param->int09);
    int10=(p_param->int10);
    int11=(p_param->int11);
    int12=(p_param->int12);
    
    float01=(float)((p_param->float01)*0.01);
    float02=(float)((p_param->float02)*0.01);
    float03=(float)((p_param->float03)*0.01);
    float04=(float)((p_param->float04)*0.01);
    float05=(float)((p_param->float05)*0.01);
    float06=(float)((p_param->float06)*0.01);
    float07=(float)((p_param->float07)*0.01);
    float08=(float)((p_param->float08)*0.01);
    float09=(float)((p_param->float09)*0.01);
    float10=(float)((p_param->float10)*0.01);
    float11=(float)((p_param->float11)*0.01);
    float12=(float)((p_param->float12)*0.01);
    
}

void param_save(struct PARAM * p_param)
{
    flash_erase_sector(120); //擦除第255页数据
    flash_erase_sector(119);

    flash_write( 120,0,(p_param->int01));
    flash_write( 120,4,(p_param->int02));
    flash_write( 120,8,(p_param->int03));
    flash_write( 120,12,(p_param->int04));
    flash_write( 120,16,(p_param->int05));
    flash_write( 120,20,(p_param->int06));
    flash_write( 120,24,(p_param->int07));
    flash_write( 120,28,(p_param->int08));
    flash_write( 120,32,(p_param->int09));
    flash_write( 120,36,(p_param->int10));
    flash_write( 120,40,(p_param->int11));
    flash_write( 120,44,(p_param->int12));

    flash_write( 120,48,(p_param->float01));
    flash_write( 120,52,(p_param->float02));
    flash_write( 120,56,(p_param->float03));
    flash_write( 120,60,(p_param->float04));
    flash_write( 120,64,(p_param->float05));
    flash_write( 120,68,(p_param->float06));
    flash_write( 120,72,(p_param->float07));
    flash_write( 120,76,(p_param->float08));
    flash_write( 120,80,(p_param->float09));
    flash_write( 120,84,(p_param->float10));
    flash_write( 120,88,(p_param->float11));
    flash_write( 120,92,(p_param->float12));
    flash_write( 119,0,(p_param->param_flag));

}




