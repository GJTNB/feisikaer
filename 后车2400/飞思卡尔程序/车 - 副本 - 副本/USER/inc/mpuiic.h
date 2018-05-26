#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "common.h"
#include "MK60_SysTick.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK MiniSTM32F103������
//MPU6050 IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/4/18
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
	   		   
//IO��������
#define MPU_SDA_IN()  PTC13_DDR=0
#define MPU_SDA_OUT() PTC13_DDR=1

//IO��������	 
#define MPU_IIC_SCL    PTC14_OUT 		//SCL
#define MPU_IIC_SDA    PTC13_OUT 		//SDA	 
#define MPU_READ_SDA   PTC13_IN 		        //����SDA 

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(uint8 txd);			//IIC����һ���ֽ�
uint8 MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8 MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�

void IMPU_IC_Write_One_Byte(uint8 daddr,uint8 addr,uint8 data);
uint8 MPU_IIC_Read_One_Byte(uint8 daddr,uint8 addr);	  
#endif
















