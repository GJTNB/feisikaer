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
  non_circle=1,                  //û��Բ��
  pre_circle_left,               //Բ��δ��
  bef_circle_left,               //Բ��ǰ
  mid_circle_left,               //Բ����
  add_circle_left,               //Բ�����ڽӽ�
  aft_circle_left,               //Բ�����ں�
  pre_circle_righ,               //ͬ��
  bef_circle_righ,
  mid_circle_righ,
  add_circle_righ,
  aft_circle_righ,    
  
}car_circle;

typedef enum
{
  non_ten=1,               //û��ʮ����
  bef_ten,                 //ʮ����ǰ
  mid_ten,                 //ʮ������
  add_ten_left_one,        
  add_ten_left_two,
  add_ten_righ_one,
  add_ten_righ_two,
  aft_ten,
}car_ten;

typedef enum
{
  left_single=1,                  //�������
  righ_single,                    //�����ұ�
  left_and_righ,                  //����ͬʱ
}find_line_way;                   //��Ѱ���߷�ʽ

typedef enum
{
  front=1,                        //ǰ��
  behind,                         //��
}car_front_or_behind;             //ǰ������

typedef struct
{
  int16 circle_num;               //Բ����Ŀ
  int16 circle_count;             //Բ������
  int16 turn_way;                 //ת����
}circle_turn_param;

typedef struct
{
  int16 ten_num;                  //ʮ������Ŀ             
  int16 ten_count;                //ʮ�������
}ten_overtake_param;

typedef struct
{
  uint8 near_start_line;                //�ӽ�������
  uint8 judge_start_distance;           //�ж����ܾ���
  uint8 far_end_line;                   //�յ���
  find_line_way fd_line_way;            ////��Ѱ���߷�ʽ
  uint8 effect_flag;                    //������Ч��־λ1
  uint8 single_distance;
  uint8 end_effect_line;
  uint8 near_lost_count_left;            //���ߴ���
  uint8 near_lost_count_righ;            //���ߴ���
  uint8 circle_pre_flag;                 //
  uint8 circle_flag;                     //Բ����־λ
  car_front_or_behind car_fob;           //ǰ�󳵱�־λ
  circle_turn_param circle_t_p;          //
  ten_overtake_param ten_o_p;            //ʮ���䳬��
  uint8 two_way_find_circle;             //�ڶ��ַ�ʽʶ��Բ��
  car_circle car_c;
  car_ten car_t;
  uint8 circle_stop_flag;
  float diff;
  car_status car_s;
  starting_line_param starting_l_p;
  uint8 end_line_stop_flag;               //�յ��߽���
  uint8 left_lost_count;                  //��߶�ʧ��
  uint8 righ_lost_count;                  //�ұ߶�ʧ��
  uint8 left_obstacle;                    //���ϰ�
  uint8 righ_obstacle;                    //���ϰ�
  uint8 obstacle_flag;                    //�ϰ���־λ
  uint8 obstacle_count;                   //�ϰ���Ŀ
  uint8 obstacle_buff_flag;               //�ϰ�buff
  uint8 picture_flag;                     //ͼ��ʶ���־
  uint32 distance;                        //����������
  uint8 ramp_flag;                        //�µ���־λ
  
  uint8 ten_flag;                         //ʮ�����־λ
  uint8 back_flag;                        //������־λ
  uint8 ten_left;                         //��ߵ�ʮ����
  uint8 ten_righ;                         //�ұߵ�ʮ����
  uint8 out_ten_flag;                     //��ʮ�����־λ
  
  uint8 left_flag;                        //ʮ������ת�򳬳���־λ
  uint8 righ_flag;                        //ʮ������ת���־λ
  
  uint8 start_flag;                       //������־λ
  uint8 circle_buff_flag;                 //ͨѶԲ����Ϣ
  
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
void mid_Point_DUTY(uint8 Mid_len,uint8 Mid_y);          //�����㷨
void street_duty(void);
#endif  /*_picture_deal_*/