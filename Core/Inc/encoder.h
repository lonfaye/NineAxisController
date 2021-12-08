/*
 * encoder.h
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

//#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>
#include "PID.h"

#define U16_MAX    ((uint16_t)65535u)
#define U32_MAX    ((uint32_t)4294967295uL)


#define SPEED_SAMPLING_TIME  9    // (9+1)*500usec = 5MS   ,200hz
#define SPEED_BUFFER_SIZE 3       //�������ٶȻ��������С

#define ENCODER2_PPR           (uint16_t)(500*30*4)  // ���2��������
#define ENCODER1_PPR           (uint16_t)(500*30*4)  // ���2��������

#define ICx_FILTER      (uint8_t) 6 // 6<-> 670nsec   ������ģʽ���ò���

#define SPEED_SAMPLING_FREQ (uint16_t)(10000/(SPEED_SAMPLING_TIME+1))  //1000hz��С���ٶȲ���Ƶ��
//#define SPEED_SAMPLING_FREQ (u16)(10000/(SPEED_SAMPLING_TIME+1)) //1000hz

//static unsigned short int hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;//����������ɼ�ʱ����

void ENC_Init(void);//��������ʼ��

void ENC_Init1(void);//���õ��B TIM3������ģʽPA6 PA7 ����
void ENC_Init2(void);//���õ��A TIM4������ģʽPB6 PB7 �ҵ��

int16_t ENC_Calc_Rot_Speed1(void);//������B�ı�����
int16_t ENC_Calc_Rot_Speed2(void);//������A�ı�����

void ENC_Clear_Speed_Buffer(void);//�ٶȴ洢������
void ENC_Calc_Average_Speed(void);//�������ε����ƽ��������

float Gain1(void);//���õ��B PID���� PA1
float Gain2(void);//���õ��A PID���� PA2

#endif /* INC_ENCODER_H_ */
