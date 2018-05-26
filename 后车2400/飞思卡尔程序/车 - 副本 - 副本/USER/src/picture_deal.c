#include "picture_deal.h"
int op=32;
int or=105;
int RL2[120];
uint8 imgbuff_1[CAMERA_SIZE];
uint8 imgbuff_2[CAMERA_SIZE];
uint8 img[CAMERA_H][CAMERA_W];
uint8 flag_imgbuff;
uint8 bmp_buff[1024];
uint8 compress_buff[120][128];
uint16 point_len[120];            //ʵʱ�������
int changdu;
int y=0;
float LS[10];
int chaochequ=0;
/*Բ��Ԥ�ж�*/
uint16 cur_L_ready_flag=0;         //��Բ��Ԥ�жϳ�ʼʶ���־
uint16 cur_L_ready_delay_flag=0;   //��Բ��Ԥ�жϳ�и��־
uint16 cur_L_ready_time_flag=0;   //��Բ��Ԥ�жϼ�ʱ����
uint16 cur_L_ready_rest_flag=0;    //��Բ��Ԥ�жϸ�λ����
uint16 cur_R_ready_flag=0;         //��Բ��Ԥ�жϳ�ʼʶ���־
uint16 cur_R_ready_delay_flag=0;   //��Բ��Ԥ�жϳ�и��־
uint16 cur_R_ready_time_flag=0;   //��Բ��Ԥ�жϼ�ʱ����
uint16 cur_R_ready_rest_flag=0;    //��Բ��Ԥ�жϸ�λ����
/*Բ��׼ȷʶ��*/
uint16 cur_L_real_flag=0;          //��Բ��׼ȷ�ж�ʶ���־
uint16 cur_L_real_delay_flag=0;    //��Բ��׼ȷ��иʶ���־
uint16 cur_L_real_delay_flag2=0;    //����
uint16 cur_L_real_rest_flag=0;     //��Բ��׼ȷ��λʶ���־
uint16 cur_L_real_time_flag=0;    //��Բ��׼ȷ��иʶ���־
uint16 cur_L_real_time_flag1=0;   //��Բ��׼ȷ��иʶ���־1

uint8 cur_R_real_flag=0;          //��Բ��׼ȷ�ж�ʶ���־
uint8 cur_R_real_delay_flag=0;    //��Բ��׼ȷ��иʶ���־
uint8 cur_R_real_rest_flag=0;     //��Բ��׼ȷ��λʶ���־
uint16 cur_R_real_time_flag=0;    //��Բ��׼ȷ��иʶ���־
uint16 cur_R_real_time_flag1=0;   //��Բ��׼ȷ��иʶ���־1

int16 left_line[120]={0};
int16 righ_line[120]={0};
int16 start_line[120]={0};
int16 center_line[120]={0};
uint16 lost_left[120];            //���߱�־
uint16 lost_right[120];            //�Ҷ��߱�־
int16 compensation_line[10]={49,50,50,50,51,51,52,52,52,54};//������
uint8 jinhuang=0;
int s=0;
//��������
int16 cp_line[120]={-2,0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,34,34,36,38,41,42,44,46,47,50,51,\
6,6,8,10,12,3,7,10,12,15,18,19,21,25,27,29,31,34,35,37,38,41,42,44,46,47,\
50,52,54,55,57,58,58,59,61,63,65,66,68,69,71,73,74,79,79,79,79,79,79,79,79,79,78,\
84,85,86,87,79,79,79,79,79,79,94,95,96,97,98,99,100,102,103,104,105,106,107,108,\
109,110,111,112,113,114,115,116,117,118,119,120};


//ʮ���еĲ���
int16 sp_line[120]={8,8,8,9,9,10,10,10,11,11,12,12,12,13,13,14,14,14,15,15,16,16,17,17,17,18,18,19,19,\
19,20,20,21,21,21,22,22,23,23,24,24,24,25,25,26,26,26,27,27,28,28,28,29,29,30,\
30,31,31,31,32,32,33,33,33,34,34,35,35,35,36,36,37,37,38,38,38,39,39,40,40,40,41,\
41,42,42,42,43,43,44,44,44,45,45,46,46,47,47,47,48,48,49,49,49,50,50,51,51,51,52,\
52,53,53,54,54,54,55,55,56,56,56};

car_param car_p;

void picture(void)
{
  int i;
  line_reset();     // ���г�ʼ��
  
  left_or_righ_flag();  // ���ߴ���  �õ��ı�־λ����judge_ten
  //ƽ��ʮ�������  ��ʱ����û������
  
  find_left_and_righ_line();//��Ѱ���߽߱�
  
  yupangduan();  //Ԥ�ж�Բ��
  
  EVENT_Duty();//Բ������
  
    HuiChejudge();
    
    if(cur_L_real_delay_flag || cur_R_real_delay_flag)//׼ȷʶ���
  {
   gpio_init(PTE10,GPO,1);     //������
//  while(i>=car_p.far_end_line)
//  {
//  //mid_Point_DUTY(2*cp_line[i],i);          //�����㷨
//  }

  }
  else
    gpio_init(PTE10,GPO,0);     //������
    
  deal_center();        // ����ͷ����
  
  judge_ten();          // ʮ����ʶ��
   
   /*�����߼��   ��ѧ�߿��Բ���
  if(car_p.starting_l_p.starting_line_count==0&&car_p.car_c==non_circle&&car_p.car_t==non_ten){
    check_starting_line( car_p.near_start_line, (car_p.near_start_line-4) );
  }
  if(car_p.starting_l_p.starting_line_count==1&&car_p.car_c==non_circle&&car_p.car_t==non_ten){
    check_starting_line( car_p.near_start_line-20, (car_p.near_start_line-24) );
  }
  */
  
  if(car_p.ten_o_p.ten_count!=0){//ʮ�������
    ten_over_tack();
     //gpio_init(PTE10,GPO,1);     //������
  }
  
}

void HuiChejudge()
{
    int i,j=0,l=0;
    int d1[28],d2[28];
    for(i=42;i<=65;i++)
    {
      d1[i-42]=left_line[i]-left_line[i+1];//��߽��
      d2[i-42]=righ_line[i+1]-righ_line[i];
      if(righ_line[i]>=157 || left_line[i]<=3)
      {
        l=0;
        j=0;
        break;
      }
      if(d1[i-42]>=5&&d1[i-42]<=8)
      {
        j++;
      }
      if(j>=2)
      {
        if((d1[i-42]-d1[i-41])>=3 && (d1[i-42]-d1[i-43])>=3)
        { 
         j=j;
         break;
        }
        else
       {
        j=0;
        }
      }
      
      if(d2[i-42]>=5 && d2[i-42]<=8)
      {
        l++;
      }
//        if(l>=2)
//      {
//        if((d2[i-42]-d2[i-41])>=3 && (d2[i-42]-d2[i-43])>=3)
//        { 
//         l=l;
//         break;
//        }
//        else
//        {
//        l=1;
//        }
//      }
    }
    
    if(j>=2&&l>=2&&!cur_R_real_delay_flag&&!cur_L_real_delay_flag&&!cur_R_ready_rest_flag&&!cur_L_ready_rest_flag)
    {
      chaochequ=1;
    }
}


/*******************************************************************************/
//                      ��һ���֣�ͼ������
//
//                      ��һ���֣��¼�������
/*******************************************************************************/
void EVENT_Duty(void)
{
  /************����ʱ��*****************/
 
  /*************Բ��Ԥʶ��****************/
  /*��Ԥʶ��*/
  if(cur_R_ready_flag||cur_R_ready_delay_flag)
  { 
      cur_R_ready_time_flag++;//��Բ��Ԥ�жϼ�ʱ����
      cur_R_ready_delay_flag=1;//����Բ���������
    
    if((!lost_right[63]&&!lost_right[65])&&!lost_left[63]&&!lost_left[65]&&cur_R_ready_time_flag>10)//&&street_len>50)
    {  //1����  56 60
      cur_R_ready_rest_flag=1;//��Բ��Ԥ�жϸ�λ���� 23 25���Ҳ�����   ��Բ���б�Ե
    }
    else;
    
    if(cur_R_ready_time_flag> 150)//?
    {
      cur_R_ready_delay_flag=0;
      cur_R_ready_rest_flag=0;
    }
    else if(cur_R_ready_rest_flag&&lost_right[55]&&lost_right[54]&&!lost_left[55]&&!lost_left[54])//&&street_len>50)
    { //23 25�Ҷ����󲻶�  ��Բ����Ե 50 52
      cur_R_ready_delay_flag=0;
      cur_R_ready_rest_flag=0;
      cur_R_real_flag=1; 
    }
    else;
  }
  else
  {
    cur_R_ready_delay_flag=0;
    cur_R_ready_time_flag=0;
    cur_R_ready_rest_flag=0;
  }
  
  /*��Ԥʶ��*/
  if(cur_L_ready_flag||cur_L_ready_delay_flag)
  {
      cur_L_ready_time_flag++;//��Բ��Ԥ�жϼ�ʱ����
      cur_L_ready_delay_flag=1;
    
    if((!lost_left[56]&&!lost_left[60])&&!lost_right[56]&&!lost_right[60]&&cur_L_ready_time_flag>10)//&&street_len>50)
    {
      cur_L_ready_rest_flag=1;
    }
    else;
    
    if(cur_L_ready_time_flag>200)
    {
      cur_L_ready_delay_flag=0;
      cur_L_ready_rest_flag=0;
    }
    else if(cur_L_ready_rest_flag&&lost_left[50]&&lost_left[52]&&!lost_right[50]&&!lost_right[52]&&point_len[60]>95&&(righ_line[70]-righ_line[45])<50&&righ_line[50]<150)//&&street_len>50)
    { 
      cur_L_ready_delay_flag=0;
      cur_L_ready_rest_flag=0;
      cur_L_real_flag=1;
    }
    else;
    
  }
  else
  { 
    cur_L_ready_delay_flag=0;
    cur_L_ready_time_flag=0;
    cur_L_ready_rest_flag=0;
  }
  
  
  /*************Բ��׼ȷʶ��****************/
  /*��׼ȷʶ��*/
  if(cur_R_real_flag||cur_R_real_delay_flag)
  { 
    cur_R_real_time_flag++;//��Բ��׼ȷ��иʶ���־
    cur_R_real_flag=0;
    cur_R_real_delay_flag=1;
    
    if(cur_R_real_time_flag>40&&(lost_left[58]&&lost_left[56])&&lost_left[58]&&lost_left[56])//�ѵ�������  23 25�Ҷ���  58 60
    {
      cur_R_real_rest_flag=1;//����
      y=-8;
      or=105;
      s=1;
    }
    else;
    
    if(cur_R_real_rest_flag)
      cur_R_real_time_flag1++;
    else;
    
      if(cur_R_real_time_flag1>29)
      s=0;
      
    if(cur_R_real_time_flag1>100)
      cur_R_real_time_flag1=100;
    else;
    
    
    if(cur_R_real_time_flag1>20&&!lost_left[48]&&!lost_left[50]&&!lost_right[50]&&!lost_right[48]&&lost_right[60])
    {  //56 60
      cur_R_real_delay_flag=0;//�жϽ���
       or=105;
       y=0;
       s=0;
       chaochequ=0;
    }
    else;
  }
  else
  {
    cur_R_real_delay_flag=0;
    cur_R_real_time_flag=0;
    cur_R_real_time_flag1=0;
    cur_R_real_rest_flag=0;
  }
  
  
  /*��׼ȷʶ��*/
  if(cur_L_real_flag||cur_L_real_delay_flag)
  { 
    cur_L_real_time_flag++;//��Բ��׼ȷ��иʶ���־
    cur_L_real_flag=0;
    cur_L_real_delay_flag=1;
    if((cur_L_real_time_flag>40&&righ_line[54]>140&&righ_line[56]>140&&lost_left[56]&&lost_left[54]))//&&lost_right[58]&&lost_left[56]
    {
      cur_L_real_rest_flag=1;
      y=-9;
      op=35;
      s=1;
    }
    else;
    
    if(cur_L_real_rest_flag)
      cur_L_real_time_flag1++;
    else;
    
     if(cur_L_real_time_flag1>30)
      s=0;
      
    if(cur_L_real_time_flag1>100)
      cur_L_real_time_flag1=100;
    
    if(cur_L_real_time_flag1>40&&!lost_left[40]&&!lost_left[42]&&!lost_right[40]&&!lost_right[42]&&lost_left[60]&&!lost_right[60])
    {
      cur_L_real_delay_flag=0;
      op=35;
       y=0;
       s=0;
       chaochequ=0;
    }
    else;
  }
  else
  {
    cur_L_real_delay_flag=0;
    cur_L_real_time_flag=0;
    cur_L_real_time_flag1=0;
    cur_L_real_rest_flag=0;
    
  }
  
}


/********************����ֱ�����Ȳ���**********************/
void street_duty(void)                //����ֱ�����Ȳ���
{
//  /*����ֱ�����Ȳ���*/
//  for(img_y=55;img_y>=1;img_y--)
//  {
//    if(img[40+img_y*80]==0)
//      break;
//    else;
//  }
//  street_len=59-img_y;
//  
//  /*��Բ����������*/
//  for(img_y=55;img_y>=1;img_y--)
//  {
//    if(img[right_point[img_y_max-3]-3+img_y*80]==0)//right_point�ұ߽�
//      break;
//    else;
//  }
//  R_Cur_K=10*(right_point[img_y_max-3]-35)/(59-img_y);
//  
//  /*��Բ����������*/
//  for(img_y=55;img_y>=1;img_y--)
//  {
//    if(img[left_point[img_y_max-3]+3+img_y*80]==0)
//      break;
//    else;
//  }
//  
//  L_Cur_K=10*(50-left_point[img_y_max-3])/(59-img_y);
}


///********************���ߴ�����*************************/
//void mid_Point_DUTY(uint8 Mid_len,uint8 Mid_y)          //�����㷨
//{  
//  /*********Բ�����ߴ���***********/
//  
//  /*�뻷*/
//
//    if(cur_R_real_delay_flag&&cur_R_real_time_flag<20)//��
//    {
//      left_line[Mid_y]=(uint8)(R_Cur_K*(59-Mid_y)/10);
//      if(left_line[Mid_y]+Mid_len/2<79)
//        center_line[Mid_y]=left_line[Mid_y]+Mid_len/2;
//      else 
//        center_line[Mid_y]=79;
//    }
//    else if(cur_L_real_delay_flag&&cur_L_real_time_flag<20)//��
//    {
//      right_point[Mid_y]=(uint8)(80-L_Cur_K*(59-Mid_y)/10);
//      if(right_point[Mid_y]-Mid_len/2>1)
//        mid_point[Mid_y]=right_point[Mid_y]-Mid_len/2;
//      else 
//        mid_point[Mid_y]=1;
//    }
//    
//     /*����*/
//    else if(cur_R_real_rest_flag&&cur_R_real_delay_flag)
//    {
//      if(mid_point_His[9]+20<79)
//        mid_point[Mid_y]=mid_point_His[9]+20;
//      else 
//        mid_point[Mid_y]=mid_point_His[9];
//    }
//    else if(cur_L_real_rest_flag&&cur_L_real_delay_flag)
//    {
//      if(mid_point_His[9]>21)
//        mid_point[Mid_y]=mid_point_His[9]-20;
//      else 
//        mid_point[Mid_y]=mid_point_His[9];
//    }
//    
//    /*����*/
//    else
//    {
//      if(left_point[Mid_y]<5 && !lost_flag)           //��߶��߲���
//      {
//        if(right_point[Mid_y]>30)
//          mid_point[Mid_y]=right_point[Mid_y]-Mid_len/2;
//        else mid_point[Mid_y]=1;
//      }
//      else if(right_point[Mid_y]>75 && !lost_flag)   //�ұ߶��߲���
//      {
//        if(left_point[Mid_y]<50)
//          mid_point[Mid_y]=left_point[Mid_y]+Mid_len/2;
//        else mid_point[Mid_y]=79;
//      }
//      else
//        mid_point[Mid_y]=(left_point[Mid_y]+right_point[Mid_y])/2; //���߼���
//    }
//    
//    
// 
//  
//}


void param_reset(void)
{
  uart_rx_irq_dis(UART1);
  car_p.circle_t_p.circle_num=int07;
  car_p.circle_t_p.circle_count=0;  
  car_p.ten_o_p.ten_num=int06;
  car_p.ten_o_p.ten_count=0;
  car_p.near_start_line=80;
  car_p.far_end_line=37;
  car_p.fd_line_way=left_and_righ;
  car_p.single_distance=20;
  car_p.judge_start_distance=5;
  car_p.car_t=non_ten;
  car_p.circle_flag=0;
  car_p.circle_stop_flag=0;
  car_p.end_line_stop_flag=0;
  car_p.starting_l_p.starting_line_flag=0;
  car_p.starting_l_p.starting_line_count=0;  
  car_p.diff=0;
  car_p.car_c=non_circle;
  
  car_p.left_obstacle=0;
  car_p.righ_obstacle=0; 
  car_p.obstacle_flag=1;
  car_p.obstacle_count=int05;
  car_p.obstacle_buff_flag=0;
  
  car_p.picture_flag=1;
  car_p.distance=0;
  car_p.ramp_flag=0;
  
  car_p.back_flag=0;
  car_p.ten_flag=1;
  car_p.ten_left=0;
  car_p.ten_righ=0;
  car_p.out_ten_flag=0;
  
  car_p.left_flag=0;
  car_p.righ_flag=0;
  
  car_p.start_flag=0;
  
  car_p.circle_buff_flag=0;
  
  car_p.car_fob=front;
  
  
  if(car_p.car_fob==front){//ǰ�󳵱�־λ
    car_p.two_way_find_circle=0;
    uart_rx_irq_dis(UART1);
    car_p.obstacle_flag=1;
    car_p.car_s=motor_run;
    car_p.picture_flag=1;
    car_p.start_flag=1;
  }
  
  
  if(car_p.car_fob==behind){
    car_p.two_way_find_circle=1;
    uart_rx_irq_en(UART1);
    car_p.obstacle_flag=0;
    car_p.car_s=motor_close;
    car_p.picture_flag=0;
    car_p.start_flag=0;
  }  
  
  
  
int01=19;
int02=4000;
int03=1490;
int04=3000;
int05=0;
int06=3000;
int07=0;
int08=20;
int09=8000;
int10=600;
int11=0;
int12=0;


float01=5.5;
float02=1000;
float03=100;
float04=100;
float05=40;
float06=40;
float07=20;
float08=90;
float09=50;
float10=5;
float11=300;
float12=0;
}



void line_reset(void)
{
  int16 i;
  for(i=car_p.near_start_line;i>=car_p.far_end_line;i--)//��ʼ����Ч����
  {//near_start_line�ӽ������߳�ʼ��Ϊ90//far_end_line�յ��߳�ʼ��Ϊ20
    LL(i)=0;     //�����
    RL(i)=159;  //�ұ���
    SL(i)=80;   //��ʼ��
    CL(i)=80;   //������
  }
  
  car_p.effect_flag=0;            //������Ч��־λ1
  car_p.end_effect_line=0;        //�����Ч��־λ
  
  car_p.near_lost_count_left=0;  //����
  car_p.near_lost_count_righ=0;  //����
  
  car_p.circle_pre_flag=0;       //֮ǰԲ�ı�־λ
  
  car_p.left_lost_count=0;      //��߶�ʧ��
  car_p.righ_lost_count=0;      //�ұ߶�ʧ��
  
  car_p.left_flag=0;            //ʮ������ת�򳬳���־λ
  car_p.righ_flag=0;           //ʮ������ת���־λ
}


void find_side_line(void)
{
//  if(car_p.fd_line_way==left_single){//fd_line_wayΪ�ı��߳�ʼ��Ϊ���Ҷ�û����
//    find_single_left();
//  }
//  if(car_p.fd_line_way==righ_single){
//    find_single_righ();
//  }
//  if(car_p.fd_line_way==left_and_righ){
    find_left_and_righ_line();
//  }
}

void find_single_left(void)
{
  //�����Ҷ�����һ��
  int16 start=80;
  start=judge_start_line(car_p.near_start_line,(car_p.near_start_line-car_p.judge_start_distance+1));  
  
  if(start==(-1)){
    car_p.effect_flag=0;
    return;  
  }
  
  int16 i;
  i=car_p.near_start_line-car_p.judge_start_distance;
  while(i>=car_p.far_end_line){
    if(PP(i,start)==BLACK_P){
      break;
    }
    else{
      find_line_l(i,start);
      start=LL(i)+car_p.single_distance;
      PP(i,start)=0;//������
    }
        
    i--;
  }
  
  car_p.effect_flag=1;
  car_p.end_effect_line=i+1;
}



void find_single_righ(void)
{
  int16 start=80;
  start=judge_start_line(car_p.near_start_line,(car_p.near_start_line-car_p.judge_start_distance+1));  
  
  if(start==(-1)){
    car_p.effect_flag=0;
    return;  
  }
  
  int16 i;
  i=car_p.near_start_line-car_p.judge_start_distance;
  while(i>=car_p.far_end_line){
    if(PP(i,start)==BLACK_P){
      break;
    }
    else{
      find_line_r(i,start);
      start=RL(i)-car_p.single_distance;
    }
    PP(i,start)=0;  //������  
    i--;
  }
  
  car_p.effect_flag=1;
  car_p.end_effect_line=i+1;  
}


void yupangduan()//Ԥ�ж�Բ��
{
    /**********Բ��Ԥ�ж�************/
  if(lost_right[55]&&lost_right[57]&&!lost_right[60]&&!lost_left[55]&&!lost_left[57]&&point_len[60]>95&&point_len[60]<159)//&&(left_line[70]-left_line[45])<70&&(left_line[70]-left_line[45])>20&&left_line[50]<150)
  {
    cur_R_ready_flag=1;//��Բ��Ԥʶ��  1���� 0������   �Ҷ���   ����
  }
  else 
 {
    cur_R_ready_flag=0;
  }

  
  if(lost_left[54]&&lost_left[52]&&!lost_left[63]&&!lost_right[54]&&!lost_right[52]&&point_len[55]>95&&(righ_line[65]-righ_line[45]>20)&&(righ_line[65]-righ_line[45])<50&&righ_line[50]<150)//&&(righ_line[70]-righ_line[45]>20)&&(righ_line[70]-righ_line[45])<50&&righ_line[50]<150)//&&street_len>47
  {
    cur_L_ready_flag=1;//��Բ��Ԥʶ��       ����  �Ҷ���
  }
  else 
  {
    cur_L_ready_flag=0;
  }
}

void find_left_and_righ_line(void)
{
  int u=0;
  //ǰ5�������ж���û�ж���  ��׼��
  static int16 start=80;
  //�ж���û�ж��� ���迿����ͷ�����������ģ�����������ǵ�i�С���ô���ҽ���i�е���i-4��5�����м��������������ķ��������5�е��е㣬�����5������3�л����϶��ҵ������ˣ���ô�ⳡͼ������Ч�ģ�������ȥ��һ��ͼ��
  start=judge_start_line(car_p.near_start_line,(car_p.near_start_line-car_p.judge_start_distance+1));  
  //��׼��
 
  for(u=0;u<120;u++)
  {
    lost_left[u]=1;
    lost_right[u]=1;
  }
  
  if(start==(-1))
  {
    car_p.effect_flag=0;//������Ч��־λ
    return;  //����ͼƬ����
  }
  
  SL(car_p.near_start_line-car_p.judge_start_distance+1)=start;//��׼��
  CL(car_p.near_start_line-car_p.judge_start_distance+1)=start;//��׼��
  
  static int16 i;
  i=car_p.near_start_line-car_p.judge_start_distance;//��׼��
  
  while(i>=car_p.far_end_line){//��ѯ��ΧΪ85�е�40��
    if(PP(i,start)==BLACK_P){
      break;//��������
    }
    else{
      find_line_lr(i,start);//�ó�RL��LL
      start=judge_column(i);//��������SL[]
      point_len[i]=righ_line[i]-left_line[i];//�����������
      
    if(left_line[i]<11)
      lost_left[i]=1;           //�����ж�
    else 
      lost_left[i]=0;
    
    if(righ_line[i]>157)
      lost_right[i]=1;        //�Ҷ����ж�
    else 
      lost_right[i]=0;
    }
    i--;
  }
  
  car_p.effect_flag=1;//ͼ����Ч
  car_p.end_effect_line=i+1;//�����Ч��
}




//start>end  return left_column
int16 gb_recursion(int16 start,int16 end,int16 line)
{
  if(abs(start-end)==1){
    return end;
  }
  if(PP(line,((start+end)/2)) == BLACK_P){
    return gb_recursion(start,((start+end)/2),line);
  }
  if(PP(line,((start+end)/2)) == WHITE_P){
    return gb_recursion(((start+end)/2),end,line);
  }
  return 0;
}


void find_line_lr(int16 line,int16 start)
{
  find_line_l(line,start);
  find_line_r(line,start);
}

void find_line_l(int16 line,int16 start)
{
  static int16 i; 
  static int8 flag=0;

  i=start;//�ӵڼ��п�ʼ��  line�ڼ���
  flag=0;
  
  
  if(i==0){//�߽綪ʧ
    LL(line)=0;
  }
  else if(i>8){
    do{
      i-=8;
      if(PP(line,i) == BLACK_P){
        LL(line)=gb_recursion((i+7),i,line);//�ݹ��ҵ�����
        flag=1;
        break;
      }      
    }while(i>8);
    if(i<=8&&flag==0){
      LL(line)=gb_recursion((i+8),0,line);
    }
  }
  else if(i<=8){
    LL(line)=gb_recursion((i+8),0,line);
  }  
  
}

void find_line_r(int16 line,int16 start)
{
  static int16 i; 
  static int8 flag=0;

  i=start;
  flag=0;
  if(i==159){
    RL(line)=159;
  }
  else if(i<151){
    do{
      i+=8;
      if(PP(line,i)== BLACK_P ){
        RL(line)=gb_recursion((i-7),i,line);
        flag=1;
        break;
      }      
    }while(i<151);
    if(i>=151&&flag==0){
      RL(line)=gb_recursion((i-8),159,line);
    }
  }
  else if(i>=151){
    RL(line)=gb_recursion((i-8),159,line);
  }  
}

int16 judge_start_line(int16 sline,int16 eline)//��ʼΪ90 86
{
  static uint8 i=0;
  int16 sum=0;
  uint8 start=80;
  
  for(i=sline;i>=eline;i--)//90 86
  {//��ʼ����  ��ʼ���߼����ܾ����1
    
    if(PP(i,start)==BLACK_P){//�м�Ϊ���ǾͲ���������ͼƬ����
      return (-1);
    }
    
    if(PP(i,start)==WHITE_P){//�м��Ϊ��������
      find_line_lr(i,start);  //�����ұ���
      start=judge_init_start(i,eline);//��һ��Ҫ��ʼ���� �����ϴ��ҵ�������
    }
    
    start=(start>=120?120:start);       
    start=(start<=40?40:start);         
    
  }
  
  for(i=sline;i>=eline;i--){
    sum+=SL(i);//SL������
  }
  
  return (int16)((float)sum/(float)(sline-eline+1));//5�����ߵ�ƽ��ֵ
}



int16 judge_init_start(int16 line,int16 eline)
{
  if(LL(line)!=0&&RL(line)!=159){
    SL(line)=(LL(line)+RL(line))/2;//���Ҷ��ҵ�
  }
  if(LL(line)!=0&&RL(line)==159){
    SL(line)=LL(line)+compensation_line[line-eline];//�ұ߶���  ���ϸ���������һ��
    car_p.near_lost_count_righ++;
  }
  if(LL(line)==0&&RL(line)!=159){
    SL(line)=RL(line)-compensation_line[line-eline];//��߶���  ��ȥ����������һ��
    car_p.near_lost_count_left++;
  }
  if(LL(line)==0&&RL(line)==159){
    SL(line)=80;                       //��������Ϊ�м�
    car_p.near_lost_count_righ++;
    car_p.near_lost_count_left++;
  }
  
  SL(line)=(SL(line)>=120?120:SL(line));
  SL(line)=(SL(line)<= 40? 40:SL(line));
  
  return SL(line);
}



int16 judge_column(int16 line)//�ж�������
{
  
  //��һƽ������
  if(LL(line)!=0&&RL(line)!=159){//���߶��б���
    if(((LL(line)+RL(line))/2-SL(line+1))>=0){//��֮ǰ�����߱Ƚ�
      SL(line)=SL(line+1)+(((LL(line)+RL(line))/2-SL(line+1))>=1?1:((LL(line)+RL(line))/2-SL(line+1)));
    }//���ڵ����߱�֮ǰ������֮�����0������֮ǰ�����߼Ӳ�
    if(((LL(line)+RL(line))/2-SL(line+1))<0){
      SL(line)=SL(line+1)+(((LL(line)+RL(line))/2-SL(line+1))<=(-1)?(-1):((LL(line)+RL(line))/2-SL(line+1)));
    }//���ڵ����߱�֮ǰ������֮��С��0������֮ǰ�����߼Ӳ�
  }
  
  
  //�ڶ�
  if(LL(line)!=0&&RL(line)==159){//���ұ���
    if((LL(line)-LL(line+1))>=0){
      SL(line)=SL(line+1)+((LL(line)-LL(line+1))>=1?1:(LL(line)-LL(line+1)));
    }
    else{
      if(line%2==0){
        SL(line)=SL(line+1);
      }
      if(line%2==1){
        SL(line)=SL(line+1)+1;
      }
      
      /*û��
      if(car_p.car_fob==behind){
        if(car_p.ten_left==1||car_p.ten_righ==1){
          SL(line)=SL(line+1)+((LL(line)-LL(line+1))<=(-1)?(-1):(LL(line)-LL(line+1)));
        }
      }
      */
      
    }
    car_p.righ_lost_count++;
  }
  
  
  //����
  if(LL(line)==0&&RL(line)!=159){//�������
    if((RL(line)-RL(line+1))<=0){
      SL(line)=SL(line+1)+((RL(line)-RL(line+1))<=(-1)?(-1):(RL(line)-RL(line+1)));
    }
    else{
      if(line%2==0){
        SL(line)=SL(line+1);
      }
      if(line%2==1){
        SL(line)=SL(line+1)-1;
      }
      if(car_p.car_fob==behind){
        if(car_p.ten_left==1||car_p.ten_righ==1){
          SL(line)=SL(line+1)+((LL(line)-LL(line+1))>=1?1:(LL(line)-LL(line+1)));
        }
      }       
    }   
    car_p.left_lost_count++;
  }
  
  
  //����
  if(LL(line)==0&&RL(line)==159){//���߶�����
    SL(line)=SL(line+1);
    car_p.righ_lost_count++;
    car_p.left_lost_count++;
    if(car_p.left_lost_count>=20&&car_p.righ_lost_count>=20){
      SL(line)=80;
    }
  }  
  
  SL(line)=SL(line)>=159?159:SL(line);
  SL(line)=SL(line)<=  0?  0:SL(line);  
  
  return SL(line);
}




uint8 search_black(int16 line,int16 start)
{
  static int16 i,j;
  int16 diff[4]={0,159,0,159};  
  int16 return_flag_left=0;
  int16 return_flag_righ=0;  
  
  
  for(i=start;i>=11;i--){
    if(PP(line,i)==BLACK_P&&PP(line,i-1)==WHITE_P){
      diff[0]=i;
      for(j=(i-1);j>=(i-10);j--){
        if(PP(line,j)==WHITE_P&&PP(line,j-1)==BLACK_P){
          return_flag_left=1;
          break;
        }
      }      
      break;
    }
  }
  
  
  for(i=start;i<=148;i++)
  {
    if(PP(line,i)==BLACK_P&&PP(line,i+1)==WHITE_P)
    {
      diff[1]=i;
      for(j=(i+1);j<=(i+10);j++)
      {
        if(PP(line,j)==WHITE_P&&PP(line,j+1)==BLACK_P)
        {
          return_flag_righ=1;
          break;
        }
      }      
      break;
    }      
  }
  
  
  for(i=start;i>=5;i--)
  {
    if(PP(line-1,i)==BLACK_P&&PP(line-1,i-1)==WHITE_P)
    {
      diff[2]=i;
      break;
    }
  }    
  
  
  for(i=start;i<=155;i++)
  {
    if(PP(line-1,i)==BLACK_P&&PP(line-1,i+1)==WHITE_P)
    {
      diff[3]=i;
      break;
    }      
  }    
  
  if(return_flag_left==1&&return_flag_righ==1)
  {
    return 0;
  }  
  
  if(diff[0]!=0&&diff[1]!=159&&diff[2]!=0&&diff[3]!=159)
  {
    if((diff[0]>=diff[2]+1)&&(diff[1]<=diff[3]-1))
    {
      return 1;
    }
  }
  
  
  return 0;
}




void confirm_circle(void)
{
  if(car_p.circle_pre_flag==1){
    uint8 left_flag=0;
    uint8 righ_flag=0;
    
    int16 l_line=0;
    int16 r_line=159;
    
    int16 i=0;
    
    for(i=car_p.near_start_line-car_p.judge_start_distance-5;i>=(car_p.end_effect_line+5);i--)
    {
      if(LL(i)>=(LL(i-5)+2)&&LL(i)>=(LL(i+5)+2)){
        left_flag=1;
        l_line=i;
        break;
      }
    }
    
    for(i=car_p.near_start_line-car_p.judge_start_distance-5;i>=(car_p.end_effect_line+5);i--)
    {
      if(RL(i)<=(RL(i-5)-2)&&RL(i)<=(RL(i+5)-2)){
        righ_flag=1;
        r_line=i;
        break;
      }    
    }
    
    
    //��Բ��
    if(left_flag==1&&righ_flag==1&&abs(l_line-r_line)<=20&&LL(l_line)>=LL(car_p.near_start_line)&&RL(r_line)<=RL(car_p.near_start_line))
    {
      
      if(car_p.circle_t_p.circle_count>5){
        car_p.circle_t_p.circle_count=0;
      }
      
      if( (car_p.circle_t_p.circle_num&(1<<car_p.circle_t_p.circle_count)) == 0){
        car_p.car_c=pre_circle_left;//��Բ��������
        car_p.circle_t_p.turn_way=0;
        car_p.circle_flag=1;
        car_p.fd_line_way=left_single;
//        if(car_p.car_fob==front){
//          uart_putchar(UART1,0x69);
//          uart_putchar(UART1,0x69);
//          if(int01==1){
//            car_p.two_way_find_circle=1;
//          }
//        }       
      }
      
      if( (car_p.circle_t_p.circle_num&(1<<car_p.circle_t_p.circle_count)) != 0){
        car_p.car_c=pre_circle_righ;//��Բ��������
        car_p.circle_t_p.turn_way=1;
        car_p.circle_flag=1;
        car_p.fd_line_way=righ_single;   
//        if(car_p.car_fob==front){
//          uart_putchar(UART1,0x69);
//          uart_putchar(UART1,0x69);
//          if(int01==1){
//            car_p.two_way_find_circle=1;
//          }
//        }       
      }    
      
      car_p.circle_t_p.circle_count++;
    }    
  }  
}





void judge_ten(void)
{
     //֮ǰ�ĺ�����λ��left_flag��righ_flag
  
    //��һ
    if(car_p.ten_left==0&&car_p.ten_righ==0){
      if(car_p.car_t==non_ten&&car_p.left_flag==1&&car_p.righ_flag==1){
        car_p.car_t=bef_ten;//��ʮ����ǰ
      }
      if(car_p.car_t==bef_ten&&car_p.left_flag==2&&car_p.righ_flag==2){
        car_p.car_t=mid_ten;//��ʮ������
        car_p.ten_o_p.ten_count++;//����ʮ�ֱ��
        if(car_p.ten_o_p.ten_count>7){
          car_p.ten_o_p.ten_count=1;
        }
      }
      if(car_p.car_t==mid_ten&&car_p.left_flag==1&&car_p.righ_flag==1){
        car_p.car_t=aft_ten;//��ʮ�����
      }
      if(car_p.car_t==aft_ten&&car_p.left_flag==2&&car_p.righ_flag==2){
        car_p.car_t=non_ten;//����ʮ����
      }
    }
    
    
    
    //�ڶ�
    if(car_p.ten_left==1){      //��ߵ�ʮ����
      if(car_p.car_t==mid_ten){
        int16 i=0;
        if(car_p.end_effect_line<70){
          for(i=80;i>=78;i--){
            if(RL(i)<(RL(i-8)-3)&&RL(i)<(RL(i+8)-3)){
              car_p.car_t=add_ten_left_one;
              break;
            }    
          }   
        }
      }    
      if(car_p.car_t==add_ten_left_one&&car_p.left_flag==1&&car_p.righ_flag==2){
        car_p.car_t=add_ten_left_two;
      }
    }
    
    
    
    //����
    if(car_p.ten_righ==1){      //�ұߵ�ʮ����
      
      if(car_p.car_t==mid_ten){
        int16 i=0;
        if(car_p.end_effect_line<70){
          for(i=80;i>=78;i--){
            if(LL(i)>(LL(i-8)+3)&&LL(i)>(LL(i+8)+3)){
              car_p.car_t=add_ten_righ_one;
              break;
            }
          }   
        }
      }  
      if(car_p.car_t==add_ten_righ_one&&car_p.left_flag==2&&car_p.righ_flag==1){
        car_p.car_t=add_ten_righ_two;
      }
    }
    
    
  }
  


void judge_circle_position(void)
{
  if(car_p.circle_flag==1){
    
    
    if(car_p.circle_t_p.turn_way==0){
      if(car_p.car_c==pre_circle_left&&car_p.left_flag==2&&car_p.righ_flag==1){
        car_p.car_c=bef_circle_left;
      }
      if(car_p.car_c==bef_circle_left&&car_p.left_flag==2&&car_p.righ_flag==2){
        car_p.car_c=mid_circle_left;
        car_p.fd_line_way=left_and_righ;
      }     
      if(car_p.car_c==mid_circle_left&&car_p.circle_stop_flag==1){
        car_p.car_c=add_circle_left;
        car_p.fd_line_way=left_single;
        if(car_p.car_fob==front){
          uart_rx_irq_en(UART1);
        }
      }
      if(car_p.car_c==add_circle_left&&car_p.left_flag==2&&car_p.righ_flag==1){
        car_p.car_c=aft_circle_left;
      }
      if(car_p.car_s==motor_run){
        if(car_p.car_c==aft_circle_left&&car_p.left_flag==2&&car_p.righ_flag==2){
          if(car_p.car_fob==behind){
            car_p.car_c=non_circle;
            car_p.circle_stop_flag=0;
            car_p.circle_flag=0;
            car_p.fd_line_way=left_and_righ;
            car_p.car_fob=front;
            uart_putchar(UART1,0x68);
            uart_putchar(UART1,0x68);     
            car_p.obstacle_flag=1;
          }
          else{
            car_p.car_c=non_circle;
            car_p.circle_stop_flag=0;
            car_p.circle_flag=0;
            car_p.fd_line_way=left_and_righ;
            car_p.car_fob=behind;      
            uart_rx_irq_en(UART1);
            if(car_p.obstacle_buff_flag==1){
              car_p.obstacle_flag=1;
              car_p.obstacle_buff_flag=0;
            }
            else{
              car_p.obstacle_flag=0;
            }
          }
        }
      }
      
    }
    
    
    
    if(car_p.circle_t_p.turn_way==1){
      if(car_p.car_c==pre_circle_righ&&car_p.left_flag==1&&car_p.righ_flag==2){
        car_p.car_c=bef_circle_righ;
      }
      if(car_p.car_c==bef_circle_righ&&car_p.left_flag==2&&car_p.righ_flag==2){
        car_p.car_c=mid_circle_righ;
        car_p.fd_line_way=left_and_righ;
      }     
      if(car_p.car_c==mid_circle_righ&&car_p.circle_stop_flag==1){
        car_p.car_c=add_circle_righ;
        car_p.fd_line_way=righ_single;
        if(car_p.car_fob==front){
          uart_rx_irq_en(UART1);
        }        
      }
      if(car_p.car_c==add_circle_righ&&car_p.left_flag==1&&car_p.righ_flag==2){
        car_p.car_c=aft_circle_righ;
      }
      if(car_p.car_s==motor_run){
        if(car_p.car_c==aft_circle_righ&&car_p.left_flag==2&&car_p.righ_flag==2){
          if(car_p.car_fob==behind){
            car_p.car_c=non_circle;
            car_p.circle_stop_flag=0;
            car_p.circle_flag=0;
            car_p.fd_line_way=left_and_righ;
            car_p.car_fob=front;
            uart_putchar(UART1,0x68);
            uart_putchar(UART1,0x68);  
            car_p.obstacle_flag=1;
          }
          else{
            car_p.car_c=non_circle;
            car_p.circle_stop_flag=0;
            car_p.circle_flag=0;
            car_p.fd_line_way=left_and_righ;
            car_p.car_fob=behind;        
            uart_rx_irq_en(UART1);
            if(car_p.obstacle_buff_flag==1){
              car_p.obstacle_flag=1;
              car_p.obstacle_buff_flag=0;
            }
            else{
              car_p.obstacle_flag=0;
            }
          }
        }
      }
    }
    
    
  }
}

void judge_circle_stop(void)
{
  uint8 get_flag=0;
  int16 i=0;
  
  if(car_p.car_c==mid_circle_left&&car_p.circle_stop_flag==0){
    for(i=80;i>=car_p.end_effect_line+8;i--){
      if(LL(i)>(LL(i-8)+3)&&LL(i)>(LL(i+8)+3)){
        get_flag=1;
        break;
      }
    }
    if(get_flag==1){
      if(car_p.car_fob==front){
        if(int01==1){
          car_p.car_s=motor_stop;
        }
      }
      car_p.circle_stop_flag=1;
    }
  }
  
  if(car_p.car_c==mid_circle_righ&&car_p.circle_stop_flag==0){
    for(i=80;i>=car_p.end_effect_line+8;i--){
      if(RL(i)<(RL(i-8)-3)&&RL(i)<(RL(i+8)-3)){
        get_flag=1;
        break;
      }    
    }
    if(get_flag==1){
      if(car_p.car_fob==front){
        if(int01==1){
          car_p.car_s=motor_stop;
        }
      }
      car_p.circle_stop_flag=1; 
    }
  }
}

void deal_center(void)
{
  if(car_p.effect_flag==0){//���ͼƬû��
    car_p.diff=0;
     static int16 count=0;
      count++;
      //ͼ������Ч
      if(count>=110){
        car_p.car_s=motor_close;
        ftm_pwm_init(FTM0, FTM_CH4,15000,0);    //����ر�
        ftm_pwm_init(FTM0, FTM_CH5,15000,0);
        ftm_pwm_init(FTM0, FTM_CH6,15000,0);
        ftm_pwm_init(FTM0, FTM_CH7,15000,0);
      }
    return;
  }
  else{        //ͼƬ��Ч
    int16 i;
    i=car_p.near_start_line;//90
      
    if(cur_R_real_delay_flag&&y<21)
    {
      y++;
       i=65;
       while(i>=50){
           CL(i)=RL(i)-(RL(65)-or);
           i--;
       }
    }
    else if(cur_L_real_delay_flag&&y<21)
    {
      y++;
       i=65;
       while(i>=50){
           CL(i)=op-LL(65)+LL(i);
           i--;
       }
    }
    else if(car_p.ten_left==1){//��ߵ�ʮ����ten_over_tack����λ
        while(i>=car_p.end_effect_line){
          CL(i)=RL(i)-sp_line[i]-8;
          i--;
        }                  
      }
      else if(car_p.ten_righ==1){//�ұߵ�ʮ����
        while(i>=car_p.end_effect_line){
          CL(i)=LL(i)+sp_line[i]+8;
          i--;
        }          
      }
      else{
        while(i>=car_p.end_effect_line){
          if(LL(i)!=0&&RL(i)!=159){//���ұ߽綼��
            CL(i)=(LL(i)+RL(i))/2;
          }        
          if(LL(i)!=0&&RL(i)==159){//���ұ߽�
            CL(i)=LL(i)+cp_line[i];
          }
          if(LL(i)==0&&RL(i)!=159){//����߽�
            CL(i)=RL(i)-cp_line[i];
          }        
          if(LL(i)==0&&RL(i)==159){//ȫ��
            CL(i)=SL(i);
          }   
          PP(i,CL(i))=0;//������
          i--;
        }
      }
    }
    
  
 
  if(car_p.end_effect_line<=30){
    car_p.end_effect_line=30;
  }    

  float a=0,b=0,c=0;
   //CL����center_line
  if((!cur_R_real_delay_flag)&&(!cur_L_real_delay_flag) || (y>=21&&s==0))
  {
  a=get_average_diff(car_p.end_effect_line,car_p.end_effect_line+((car_p.near_start_line+1-car_p.end_effect_line)/3)-1,center_line);
  b=get_average_diff(car_p.end_effect_line+(car_p.near_start_line+1-car_p.end_effect_line)/3,car_p.end_effect_line+2*((car_p.near_start_line+1-car_p.end_effect_line)/3)-1,center_line);
  c=get_average_diff(car_p.end_effect_line+2*(car_p.near_start_line+1-car_p.end_effect_line)/3,car_p.near_start_line,center_line);    
  car_p.diff=(float05*a+float06*b+float07*c-(float)int09)/100.0f;  //΢��
  }
  else
  {
    if(s==1&&(cur_L_real_time_flag1<=30 || cur_R_real_time_flag1<=29))
  {
    a=get_average_diff(50,65,center_line);
    car_p.diff=(100*a-(float)int09)/100.0f;  //΢��
    car_p.diff=(((LS[0]+LS[1]+LS[2]+LS[3]+LS[4]+LS[5]+LS[6]+LS[7]+LS[8]+LS[9]+LS[10]+LS[11]+LS[12]+LS[13]+LS[14]+LS[15]+LS[16]+LS[17]+LS[18]+LS[19])/20)*1+0*car_p.diff);//+(LS[10]+LS[11]+LS[12]+LS[13]+LS[14]+LS[15]+LS[16]+LS[17]+LS[18]+LS[19])*0.6+(LS[20]+LS[21]+LS[22]+LS[23]+LS[24]+LS[25]+LS[26]+LS[27]+LS[28]+LS[29])*0.2)/30;
  }
    else
    {
     if(cur_R_real_delay_flag)
    {
    a=get_average_diff(50,65,center_line);
    car_p.diff=(100*a-(float)int09)/100.0f;  //΢��
    LS[19]=LS[18];
    LS[18]=LS[17];
    LS[17]=LS[16];
    LS[16]=LS[15];
    LS[15]=LS[14];
    LS[14]=LS[13];
    LS[13]=LS[12];
    LS[12]=LS[11];
    LS[11]=LS[10];
    LS[10]=LS[9];
    LS[9]=LS[8];
    LS[8]=LS[7];
    LS[7]=LS[6];
    LS[6]=LS[5];
    LS[5]=LS[4];
    LS[4]=LS[3];
    LS[3]=LS[2];
    LS[2]=LS[1];
    LS[1]=LS[0];
    LS[0]=car_p.diff;
    }
    else
    {
    a=get_average_diff(50,65,center_line);
    car_p.diff=(100*a-(float)int09)/100.0f;  //΢��
    LS[19]=LS[18];
    LS[18]=LS[17];
    LS[17]=LS[16];
    LS[16]=LS[15];
    LS[15]=LS[14];
    LS[14]=LS[13];
    LS[13]=LS[12];
    LS[12]=LS[11];
    LS[11]=LS[10];
    LS[10]=LS[9];
    LS[9]=LS[8];
    LS[8]=LS[7];
    LS[7]=LS[6];
    LS[6]=LS[5];
    LS[5]=LS[4];
    LS[4]=LS[3];
    LS[3]=LS[2];
    LS[2]=LS[1];
    LS[1]=LS[0];
    LS[0]=car_p.diff;
    }
    }
    

    
  }
    
  
  //40 40 20 7900
}



float get_average_diff(int16 i,int16 j,int16* arry)
{
  static int16 a;
  static int32 sum;
  sum=0;

  
  quick_sort(arry,i,j);//��С������
  
  for(a=i+1;a<=j-1;a++)
  {
    sum+=arry[a];
  }
  
  return (sum/(float)(j-i-1));
}



int16 partition(int16 arr[], int16 low, int16 high)
{
    int16 key;
    key = arr[low];
    while(low<high){
        while(low <high && arr[high]>= key )
            high--;
        if(low<high)
            arr[low++] = arr[high];
        while( low<high && arr[low]<=key )
            low++;
        if(low<high)
            arr[high--] = arr[low];
    }
    arr[low] = key;
    return low;
}

void quick_sort(int16 arr[], int16 start, int16 end)
{
    int16 pos;
    if (start<end){
        pos = partition(arr, start, end);
        quick_sort(arr,start,pos-1);
        quick_sort(arr,pos+1,end);
    }
    return;
}



void judge_obstacle(void)
{
  uint8 i=0;
  uint8 flag=0;
  if(car_p.obstacle_flag==1&&car_p.car_c==non_circle&&car_p.car_t==non_ten&&car_p.left_lost_count==0&&car_p.righ_lost_count==0&&car_p.end_effect_line<=60&&car_p.starting_l_p.starting_line_count==1&&car_p.ten_left==0&&car_p.ten_righ==0&&car_p.obstacle_count!=0){
    if(car_p.left_obstacle==0&&car_p.righ_obstacle==0){
      for(i=80;i>=car_p.end_effect_line+5;i--){
        if(abs(LL(i)-RL(i))>abs(LL(i-1)-RL(i-1))+15){
          if(abs(LL(i)-RL(i))>abs(LL(i-2)-RL(i-2))+15){
            if(LL(i)<LL(car_p.end_effect_line)&&RL(i)>RL(car_p.end_effect_line)){
              flag=1;
              break;
            }
          }
        }
      }
      
      if(flag==1){
        if(abs(LL(i)-LL(i-1))>=15&&abs(RL(i)-RL(i-1))<=4&&LL(i)<LL(i-1)&&LL(i)>LL(car_p.near_start_line)){
          if(abs(LL(i)-LL(i-2))>=15&&abs(RL(i)-RL(i-2))<=4&&LL(i)<LL(i-2)){
            car_p.left_obstacle=1;
            PTE10_OUT=1;
            int09=7400;
          }
        }
        if(abs(RL(i)-RL(i-1))>=15&&abs(LL(i)-LL(i-1))<=4&&RL(i)>RL(i-1)&&RL(i)<RL(car_p.near_start_line)){
          if(abs(RL(i)-RL(i-2))>=15&&abs(LL(i)-LL(i-2))<=4&&RL(i)>RL(i-2)){
            car_p.righ_obstacle=1;
            PTE10_OUT=1;
            int09=8600;
          }
        }        
      }
      
    }   
  }
  
  if(car_p.left_obstacle==1||car_p.righ_obstacle==1){
    int16 distance=0;
    static int16 flag=0;    
    
    distance=abs(RL(car_p.near_start_line)-LL(car_p.near_start_line));

//    if(PTD12_IN!=0){
//      oled_fill(0);
//      oled_print_16x8short(0,0,distance);       
//    }
    if(flag==0){
      if(distance>=80&&distance<=100){
        if(car_p.car_fob==front){
          uart_putchar(UART1,0x71);  
          uart_putchar(UART1,0x71);      
        }            
        flag=1;
      }
    }
    if(flag==1){
      if(distance>=50&&distance<=70){
        flag=2;
      }      
    }
    if(flag==2){
      if(distance>=80&&distance<=100){
        flag=3;
      }      
    }    
    
    
    if(flag==3){
      int09=8000;
      car_p.left_obstacle=0;
      car_p.righ_obstacle=0;
      distance=0;
      flag=0;
      PTE10_OUT=0;
      if(car_p.car_fob==behind){
        if(car_p.obstacle_buff_flag==1){
          car_p.obstacle_flag=1;
          car_p.obstacle_buff_flag=0;
        }
        else{
          car_p.obstacle_flag=0;
        }
      }
      car_p.obstacle_count--;
    }    
  }
}


void check_starting_line(int16 sline,int16 eline)
{
  if(car_p.car_c==non_circle){
    static uint8 this_get=0;
    static uint8 last_get=0;
    static uint8 i=0,j=0,k=0;
    uint8 left_flag=0,righ_flag=0;
    uint8 get_count=0;
    for(i=sline;i>=eline;i--){
      if(PP(i,80)==BLACK_P){
        for(j=80;j>=60;j--){
          if(PP(i,j)==BLACK_P&&PP(i,j-1)==WHITE_P){
            for(k=j-1;k>=j-20;k--){
              if(PP(i,k)==BLACK_P&&PP(i,k-1)==WHITE_P){
                left_flag=1;
                goto segement_1;
              }
            }
          }
        }
      segement_1:      
        for(j=80;j<=100;j++){
          if(PP(i,j)==BLACK_P&&PP(i,j+1)==WHITE_P){
            for(k=j+1;k<=j+20;k++){
              if(PP(i,k)==BLACK_P&&PP(i,k+1)==WHITE_P){
                righ_flag=1;
                goto segement_2;
              }
            }
          }
        }
      segement_2:
        if(left_flag==1&&righ_flag==1){
          get_count++;
        }
        left_flag=0;
        righ_flag=0;      
      }
      if(PP(i,80)==WHITE_P){
        for(j=80;j>=60;j--){
          if(PP(i,j)==WHITE_P&&PP(i,j-1)==BLACK_P){
            for(k=j-1;k>=j-20;k--){
              if(PP(i,k)==WHITE_P&&PP(i,k-1)==BLACK_P){
                left_flag=1;
                goto segement_3;
              }
            }
          }
        }
      segement_3:      
        for(j=80;j<=100;j++){
          if(PP(i,j)==WHITE_P&&PP(i,j+1)==BLACK_P){
            for(k=j+1;k<=j+20;k++){
              if(PP(i,k)==WHITE_P&&PP(i,k+1)==BLACK_P){
                righ_flag=1;
                goto segement_4;
              }
            }
          }
        }
      segement_4:
        if(left_flag==1&&righ_flag==1){
          get_count++;
        }    
        left_flag=0;
        righ_flag=0;      
      }
    }
    
    if(get_count>=3){
      this_get=1;
    }
    else if(get_count==0){
      this_get=0;
    }
    
    if(this_get==0&&last_get==1){
      car_p.starting_l_p.starting_line_flag=1;
      car_p.starting_l_p.starting_line_count++;    
      //PTE10_OUT=1;
    }
//    else{
//      PTE10_OUT=0;
//    }
    
    last_get=this_get;
  }
}

void judge_ramp(void)
{
  if(car_p.car_c==non_circle&&car_p.car_t==non_ten&&car_p.left_lost_count==0&&car_p.righ_lost_count==0&&car_p.left_obstacle==0&&car_p.righ_obstacle==0&&car_p.ramp_flag==0){
    if(LL(70)>LL(car_p.near_start_line)&&LL(60)>LL(70)&&RL(70)<RL(car_p.near_start_line)&&RL(60)<RL(70)){
      int16 temp=abs( (int16)get_average_diff(60,70,left_line)-(int16)get_average_diff(60,70,righ_line) );
//      if(PTD12_IN!=0){
//        oled_fill(0);
//        oled_print_16x8short(0,0,temp); 
//      }  
      if(temp>=80){
        car_p.ramp_flag=1;
      }
    }
  }
  
  if(car_p.ramp_flag==1){
    PTE10_OUT=1;
    static int16 count=0;
    count++;
    if(count<=120){
      car_p.far_end_line=60;
    }
    if(count<=160&&count>120){
      car_p.far_end_line=19;
    }
    if(count==160){
      count=0;
      car_p.ramp_flag=0;     
      PTE10_OUT=0;
    }
  }
  else{
    car_p.far_end_line=19;
  }
}

void ten_over_tack(void)
{
  if(car_p.ten_flag==1&&((car_p.ten_o_p.ten_num&(1<<(car_p.ten_o_p.ten_count-1))) != 0)){
    
    if(car_p.ten_left==0&&car_p.ten_righ==0){
      if(car_p.car_t==mid_ten){
        static uint8 count=0;
        count++;
        if(count==2){
          if(car_p.diff>=0){
            car_p.ten_righ=1;
          }
          else if(car_p.diff<=0){
            car_p.ten_left=1;
          }
          count=0;
        }
      }
    }
    
    if(car_p.ten_left==1){
      if(car_p.car_fob==front){
        if(car_p.car_t==add_ten_left_two&&car_p.out_ten_flag==0){
          car_p.car_s=motor_add;
          uart_rx_irq_en(UART1);
          car_p.out_ten_flag=1;
        }
        if(car_p.out_ten_flag==1){
          if(car_p.car_s==motor_run){
            static uint8 flag=0;
            uint8 get=0;
            
            if(flag==0){
              if(car_p.left_flag==1&&car_p.righ_flag==1){
                flag=1;
              }
            }
            
            if(flag==1){
              if(car_p.left_flag==2&&car_p.righ_flag==2){
                flag=0;
                get=1;
              }         
            }
            
            
            
            if(get==1){
              flag=0;
              car_p.car_t=non_ten;
              car_p.ten_left=0;
              car_p.ten_righ=0;
              car_p.out_ten_flag=0;
              car_p.car_fob=behind;      
              if(car_p.circle_buff_flag==1){
                car_p.two_way_find_circle=0;
                car_p.circle_buff_flag=0;
              }
              else{
                car_p.two_way_find_circle=1;
              }
              car_p.car_c=non_circle;
              if(car_p.obstacle_buff_flag==1){
                car_p.obstacle_flag=1;
                car_p.obstacle_buff_flag=0;
              }
              else{
                car_p.obstacle_flag=0;
              }          
              return;
            }
            
          }
        }
      }
      if(car_p.car_fob==behind){
        if(car_p.out_ten_flag==0){
          uint8 get_flag=0;
          int16 i=0;
          for(i=80;i>=car_p.end_effect_line+8;i--){
            if(RL(i)<(RL(i-8)-3)&&RL(i)<(RL(i+8)-3)){
              get_flag=1;
              break;
            }    
          }         
          if(get_flag==1){
            car_p.out_ten_flag=1;
          }
        }
        if(car_p.out_ten_flag==1){
          static uint8 flag=0;
          uint8 get=0;
          
          if(flag!=1){
            if(car_p.left_flag==1&&car_p.righ_flag==2){
              flag=1;
              uart_putchar(UART1,0x73);
              uart_putchar(UART1,0x73);               
            }
          }
          if(flag==1){
            if(car_p.left_flag==2&&car_p.righ_flag==2){
              flag=0;
              get=1;
            }
          }
             
          if(get==1){
            flag=0;
            car_p.car_t=non_ten;
            car_p.ten_left=0;
            car_p.ten_righ=0;
            car_p.out_ten_flag=0;
            car_p.car_fob=front;              
            car_p.two_way_find_circle=0;
            car_p.car_c=non_circle;
            car_p.obstacle_flag=1;   
            uart_rx_irq_dis(UART1);
//            uart_putchar(UART1,0x73);
//            uart_putchar(UART1,0x73);  
            return;
          }              
        }
      }
    }
    
    
    if(car_p.ten_righ==1){
      if(car_p.car_fob==front){
        if(car_p.car_t==add_ten_righ_two&&car_p.out_ten_flag==0){
          car_p.car_s=motor_add;
          uart_rx_irq_en(UART1);
          car_p.out_ten_flag=1;
        }
        if(car_p.out_ten_flag==1){
          if(car_p.car_s==motor_run){
            static uint8 flag=0;
            uint8 get=0;
            
            if(flag==0){
              if(car_p.left_flag==1&&car_p.righ_flag==1){
                flag=1;
              }
            }
            
            if(flag==1){
              if(car_p.left_flag==2&&car_p.righ_flag==2){
                flag=0;
                get=1;
              }         
            }
            
            
            if(get==1){
              flag=0;
              car_p.car_t=non_ten;
              car_p.ten_left=0;
              car_p.ten_righ=0;
              car_p.out_ten_flag=0;
              car_p.car_fob=behind;              
              if(car_p.circle_buff_flag==1){
                car_p.two_way_find_circle=0;
                car_p.circle_buff_flag=0;
              }
              else{
                car_p.two_way_find_circle=1;
              }
              car_p.car_c=non_circle;
              if(car_p.obstacle_buff_flag==1){
                car_p.obstacle_flag=1;
                car_p.obstacle_buff_flag=0;
              }
              else{
                car_p.obstacle_flag=0;
              }            
              return;
            }
            
          }
        }
      }
      if(car_p.car_fob==behind){
        if(car_p.out_ten_flag==0){
          uint8 get_flag=0;
          int16 i=0;
          for(i=80;i>=car_p.end_effect_line+8;i--){
            if(LL(i)>(LL(i-8)+3)&&LL(i)>(LL(i+8)+3)){
              get_flag=1;
              break;
            }
          }
          if(get_flag==1){
            car_p.out_ten_flag=1;
          }
        }
        if(car_p.out_ten_flag==1){
          static uint8 flag=0;
          uint8 get=0;
          
          if(flag!=1){
            if(car_p.left_flag==2&&car_p.righ_flag==1){
              flag=1;
              uart_putchar(UART1,0x73);
              uart_putchar(UART1,0x73);  
            }
          }
          if(flag==1){
            if(car_p.left_flag==2&&car_p.righ_flag==2){
              flag=0;
              get=1;
            }
          }
          
          
          
          if(get==1){
            flag=0;
            car_p.car_t=non_ten;
            car_p.ten_left=0;
            car_p.ten_righ=0;
            car_p.out_ten_flag=0;
            car_p.car_fob=front;              
            car_p.two_way_find_circle=0;
            car_p.car_c=non_circle;
            car_p.obstacle_flag=1;    
            uart_rx_irq_dis(UART1);
//            uart_putchar(UART1,0x73);
//            uart_putchar(UART1,0x73);              
            return;
          }
        }
      }
    }
    
  }
}

void left_or_righ_flag(void)
{
  int16 i=0,j=0;

  //left
  for(i=119;i>=115;i--){
    
    if(PP(i,40)==BLACK_P){//��0��1  PP����img
      LL(i)=40;  //�����
      continue;
    }
    else{
      for(j=40;j>=1;j--){
        if(PP(i,j)==WHITE_P&&PP(i,j-1)==BLACK_P){
          LL(i)=j-1;
          break;
        }
      }
      
      if(j==0){
        LL(i)=0;
      }
    }
    
  }

  //righ
  for(i=119;i>=115;i--){
    if(PP(i,120)==BLACK_P){
      RL(i)=120;
      continue;
    }
    else{
      for(j=120;j<=158;j++){
        if(PP(i,j)==WHITE_P&&PP(i,j+1)==BLACK_P){
          RL(i)=j+1;
          break;
        }
      }
      if(j==159){
        RL(i)=159;
      }
    }
  }
  
  
  if(LL(119)==0&&LL(118)==0&&LL(117)==0&&LL(116)==0&&LL(115)==0){
    car_p.left_flag=1;//ʮ������ת�򳬳���־λ
  }
  if(LL(119)!=0&&LL(118)!=0&&LL(117)!=0&&LL(116)!=0&&LL(115)!=0){
    car_p.left_flag=2;
  }    
  
  if(RL(119)==159&&RL(118)==159&&RL(117)==159&&RL(116)==159&&RL(115)==159){
    car_p.righ_flag=1;//ʮ������ת���־λ
  }    
  if(RL(119)!=159&&RL(118)!=159&&RL(117)!=159&&RL(116)!=159&&RL(115)!=159){
    car_p.righ_flag=2;
  }   
  
}