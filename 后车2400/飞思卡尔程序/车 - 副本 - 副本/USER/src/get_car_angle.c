#include "get_car_angle.h"
#include "mpu6050.h"
#include "steer_control.h"

float gyro_zero_x=0;
float gyro_zero_y=0;
float gyro_zero_z=0;

float Peried = 1/150.0f;
float KalmanGain = 1.0;//卡尔曼增益
float Q = 2.0;//过程噪声2.0		越小积分越慢，跟踪加速度计越慢越平滑
float R = 5000.0;//测量噪声5000.0	越小跟踪加速度计越快
float g_AngleOfCar=0;

float g_fAngleSpeed=0;
float g_fAngleOfAcce=0;

float angle_of_z=0;
float speed_of_z=0;


void get_gyro_zero(void)
{
  int16 i=0;
  int32 sum_x=0,sum_y=0,sum_z=0;
  int16 gyro[3]={0,0,0};
  for(i=0;i<=999;i++){
    MPU_Get_Gyroscope(&gyro[0],&gyro[1],&gyro[2]);
    sum_x+=gyro[0];
    sum_y+=gyro[1];
    sum_z+=gyro[2];
  }
  gyro_zero_x=(float)(sum_x/1000.0f);    
  gyro_zero_y=(float)(sum_y/1000.0f);
  gyro_zero_z=(float)(sum_z/1000.0f);    
}

void get_car_angle(void)
{
  static int16 gyro[3]={0,0,0};
  static int16 acce[3]={0,0,0};
  
  MPU_Get_Gyroscope(&gyro[0],&gyro[1],&gyro[2]);
  MPU_Get_Accelerometer(&acce[0],&acce[1],&acce[2]);//加速度
  
  speed_of_z=((float)((float)gyro[2]-gyro_zero_z)/16.4f);//实际值减初始值
   
}


void KalmanFilter(void)
{
	//卡尔曼滤波局部参量
    static float Priori_Estimation = 0;//先验估计
    static float Posterior_Estimation = 0;//后验估计
    static float Priori_Convariance = 0;//先验方差
    static float Posterior_Convariance = 0;//后验方差
		
	//卡尔曼滤波
    //1.时间更新(预测) : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k) 
    Priori_Estimation = Posterior_Estimation + g_fAngleSpeed*Peried;		//先验估计，积分获得角度
    //2.更新先验协方差  : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k) 
    Priori_Convariance = (float)sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );
	
    //  卡尔曼后验估计：测量更新  
    // 1.计算卡尔曼增益  : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) /
    KalmanGain = (float)sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
    //2.测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1)) 
    Posterior_Estimation  = Priori_Estimation + KalmanGain * (g_fAngleOfAcce - Priori_Estimation );
    // 3.更新后验协方差  : P(k|k) =（I-K(k)*H(k)）*P(k|k-1) 
    Posterior_Convariance = (float)sqrt(( 1 - KalmanGain ) * Priori_Convariance * Priori_Convariance );
    //得到最终角度 
    g_AngleOfCar = Posterior_Estimation;
  
}
