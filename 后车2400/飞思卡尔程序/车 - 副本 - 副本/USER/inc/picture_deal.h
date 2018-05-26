#ifndef _picture_deal_
#define _picture_deal_

#include "common.h"
#include "include.h"
#include "VCAN_camera.h" 
#include "get_car_angle.h"

extern uint8 imgbuff_1[CAMERA_SIZE];
extern uint8 imgbuff_2[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W];
extern uint8 flag_imgbuff;
extern uint8 bmp_buff[1024];
extern uint8 compress_buff[120][128];

extern int16 left_line[120];
extern int16 righ_line[120];
extern int16 start_line[120];
extern int16 center_line[120];

#define LL(i)  left_line[(i)]
#define RL(i)  righ_line[(i)]
#define SL(i)  start_line[(i)]
#define CL(i)  center_line[(i)]
#define PP(i,j)  img[(i)][(j)]

#define BLACK_P    (0)
#define WHITE_P    (1)

typedef struct
{
  int16 starting_line_flag;
  int16 starting_line_count;
}starting_line_param;

typedef enum
{
  motor_close=0,
  motor_stop,
  motor_run,
  motor_add,
}car_status;

typedef enum
{
  non_circle=1,                  //没有圆环
  pre_circle_left,               //圆环未入
  bef_circle_left,               //圆环前
  mid_circle_left,               //圆环中
  add_circle_left,               //圆环出口接近
  aft_circle_left,               //圆环出口后
  pre_circle_righ,               //同上
  bef_circle_righ,
  mid_circle_righ,
  add_circle_righ,
  aft_circle_righ,    
  
}car_circle;

typedef enum
{
  non_ten=1,               //没有十字弯
  bef_ten,                 //十字弯前
  mid_ten,                 //十字弯中
  add_ten_left_one,        
  add_ten_left_two,
  add_ten_righ_one,
  add_ten_righ_two,
  aft_ten,
}car_ten;

typedef enum
{
  left_single=1,                  //单独左边
  righ_single,                    //单独右边
  left_and_righ,                  //左右同时
}find_line_way;                   //找寻边线方式

typedef enum
{
  front=1,                        //前车
  behind,                         //后车
}car_front_or_behind;             //前后车区分

typedef struct
{
  int16 circle_num;               //圆环数目
  int16 circle_count;             //圆环计数
  int16 turn_way;                 //转向方向
}circle_turn_param;

typedef struct
{
  int16 ten_num;                  //十字弯数目             
  int16 ten_count;                //十字弯计数
}ten_overtake_param;

typedef struct
{
  uint8 near_start_line;                //接近起跑线
  uint8 judge_start_distance;           //判断起跑距离
  uint8 far_end_line;                   //终点线
  find_line_way fd_line_way;            ////找寻边线方式
  uint8 effect_flag;                    //边线有效标志位1
  uint8 single_distance;
  uint8 end_effect_line;
  uint8 near_lost_count_left;            //丢线次数
  uint8 near_lost_count_righ;            //丢线次数
  uint8 circle_pre_flag;                 //
  uint8 circle_flag;                     //圆环标志位
  car_front_or_behind car_fob;           //前后车标志位
  circle_turn_param circle_t_p;          //
  ten_overtake_param ten_o_p;            //十字弯超车
  uint8 two_way_find_circle;             //第二种方式识别圆环
  car_circle car_c;
  car_ten car_t;
  uint8 circle_stop_flag;
  float diff;
  car_status car_s;
  starting_line_param starting_l_p;
  uint8 end_line_stop_flag;               //终点线结束
  uint8 left_lost_count;                  //左边丢失数
  uint8 righ_lost_count;                  //右边丢失数
  uint8 left_obstacle;                    //左障碍
  uint8 righ_obstacle;                    //右障碍
  uint8 obstacle_flag;                    //障碍标志位
  uint8 obstacle_count;                   //障碍数目
  uint8 obstacle_buff_flag;               //障碍buff
  uint8 picture_flag;                     //图像识别标志
  uint32 distance;                        //超声波距离
  uint8 ramp_flag;                        //坡道标志位
  
  uint8 ten_flag;                         //十字弯标志位
  uint8 back_flag;                        //倒车标志位
  uint8 ten_left;                         //左边的十字弯
  uint8 ten_righ;                         //右边的十字弯
  uint8 out_ten_flag;                     //出十字弯标志位
  
  uint8 left_flag;                        //十字弯左转向超车标志位
  uint8 righ_flag;                        //十字弯右转向标志位
  
  uint8 start_flag;                       //开车标志位
  uint8 circle_buff_flag;                 //通讯圆环信息
  
}car_param;

extern car_param car_p;






















void HuiChejudge();
extern void picture(void);
void line_reset(void);
extern void param_reset(void);
void find_side_line(void);
void find_single_left(void);
void find_single_righ(void);
void find_left_and_righ_line(void);
void yupangduan();
int16 gb_recursion(int16 start,int16 end,int16 line);
void find_line_lr(int16 line,int16 start);
void find_line_l(int16 line,int16 start);
void find_line_r(int16 line,int16 start);
int16 judge_start_line(int16 sline,int16 eline);
int16 judge_init_start(int16 line,int16 eline);
int16 judge_column(int16 line);
uint8 search_black(int16 line,int16 start);
void confirm_circle(void);
void judge_ten(void);
void judge_circle_position(void);
void judge_circle_stop(void);
void deal_center(void);
float get_average_diff(int16 i,int16 j,int16* arry);
int16 partition(int16 arr[], int16 low, int16 high);
void quick_sort(int16 arr[], int16 start, int16 end);
void judge_obstacle(void);
void check_starting_line(int16 sline,int16 eline);
void judge_ramp(void);
void ten_over_tack(void);
void left_or_righ_flag(void);
void EVENT_Duty(void);
void mid_Point_DUTY(uint8 Mid_len,uint8 Mid_y);          //补线算法
void street_duty(void);
#endif  /*_picture_deal_*/