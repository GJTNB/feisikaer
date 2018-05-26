#include "common.h"
#include "include.h"
#include "init.h"
#include "picture_deal.h"

/*圆环预判断*/
extern uint16 cur_L_ready_flag;         //左圆环预判断初始识别标志
extern uint16 cur_L_ready_delay_flag;   //左圆环预判断弛懈标志
extern uint16 cur_L_ready_time_flag;   //左圆环预判断计时变量
extern uint16 cur_L_ready_rest_flag;    //左圆环预判断复位变量
extern uint16 cur_R_ready_flag;         //右圆环预判断初始识别标志
extern uint16 cur_R_ready_delay_flag;   //右圆环预判断弛懈标志
extern uint16 cur_R_ready_time_flag;   //右圆环预判断计时变量
extern uint16 cur_R_ready_rest_flag;    //右圆环预判断复位变量
/*圆环准确识别*/
extern uint16 cur_L_real_flag;          //左圆环准确判断识别标志
extern uint16 cur_L_real_delay_flag;    //左圆环准确弛懈识别标志
extern uint16 cur_L_real_rest_flag;     //左圆环准确复位识别标志
extern uint16 cur_L_real_time_flag;    //左圆环准确弛懈识别标志
extern uint16 cur_L_real_time_flag1;   //左圆环准确弛懈识别标志1
extern uint16 cur_L_real_delay_flag2;
extern uint8 cur_R_real_flag;          //右圆环准确判断识别标志
extern uint8 cur_R_real_delay_flag;    //右圆环准确弛懈识别标志
extern uint8 cur_R_real_rest_flag;     //右圆环准确复位识别标志
extern uint16 cur_R_real_time_flag;    //右圆环准确弛懈识别标志
extern uint16 cur_R_real_time_flag1;   //右圆环准确弛懈识别标志1
// 控制是通过中断实现的，所有的控制程序在 user/isr/isr.c文件中
// 在刚接触智能车时首先要实现的就是让他先能够沿中线走
// 沿中线走是最基本的要求，所以对于这个程序来说你需要的看的就是图像处理部分
// 图像处理部分需要看的是中线的找寻 ，也就是边线的找寻，找寻主偏差，然后作用于舵机上面
// 可以说除了边线的找寻，舵机控制，其他所有部分你都不需要看
// 等你能够实现在普通赛道上沿中线走再看我的赛道识别  普通赛道是指（除了圆环，十字弯）

// 等你实现上面的部分后，在你调车的整个过程当中你永远要记得的一句话就是：
// 无论你怎么调车，都要建立在偏差平滑的基础上，偏差平滑永远是基础！！！没有平滑的偏差再多都是没用的

// 硬件配置：
// k60DN512 （任何一个商家的K60都可以用这个程序来驱动，没有任何问题 ）
// 山外鹰眼摄像头
// 0.96寸oled  （能够I2C驱动的或者GPIO模拟I2C驱动的）
// 拨码开关
// 蜂鸣器
// 另外程序中所有的红外都是指红外接收头，也就是你平时使用的遥控器遥控另一端所用的这个淘宝有



void  main(void)
{
  int k;  //下面图像切换的标志位
  car_init();             //车辆初始化
  car_debug.picture_run_flag=1;  //摄像头图像正常采集

  while(1)
  {
   // flag_imgbuff != 0?img_sd_save(imgbuff_1,CAMERA_SIZE):img_sd_save(imgbuff_2,CAMERA_SIZE);//内存卡读取
    
    if(PTD12_IN==0){                  //拨码开关拨下
      Draw_BMP(0,0,127,7,bmp_buff);    //oled显示赛道图像
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

