/*
 * PID.h
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdio.h>

struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float Kc;//anti windup
    float error_0;//��������
    float error_1;//һ��г������
    float error_2;//����г������
    float SatErr;
    float  Sum_error;
    float OutPreSat;
    float OutputValue;//ʵ�������
    float OwenValue;//�����ʱ�ı�׼�����
    float Value_Kp;//��������
	float Value_Ki;//���ַ���
	float Value_Kd;//΢�ַ���

};

float PID_P_calculate( struct PID *Control,float CurrentValue);      //λ��PID����
float PID_I_calculate( struct PID *Control,float CurrentValue);      //λ��PID����

#endif /* INC_PID_H_ */
