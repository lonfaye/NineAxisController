/*
 * PID.c
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */

#include "PID.h"
#include <math.h>

//extern int span;

float MaxValue=10000;//�������޷�
float MinValue=-10000;//-9000;//�����С�޷�

float OutputValue = 0;//PID����ݴ����,���ڻ��ֱ�������


float PID_P_calculate(struct PID *Control,float CurrentValue )// 位置式PID with anti windup
{

//	float Value_Kp;//��������
//	long Value_Ki;//���ַ���
//	float Value_Kd;//΢�ַ���
//	float OutPreSat;
//	uint16_t Out;

	if(Control->OwenValue<0.1 && Control->OwenValue>=-0.1)
	{
		Control->Value_Kp = 0;
		Control->Value_Ki = 0;
		Control->Value_Kd = 0;
		Control->Sum_error = 0;
		Control->OutputValue = 0;
	}


	Control->error_0 = Control->OwenValue - CurrentValue; //+ 0*span;//����������Control->OwenValueΪ��Ҫ���ٶȣ�CurrentValue_leftΪ�����ʵ�ٶ�

	Control->Value_Kp = Control->Kp * Control->error_0 ;
//	Control->Sum_error += Control->error_0;

	Control->Value_Ki = Control->Ki * Control->error_0 + Control->Sum_error;

	Control->Value_Kd = Control->Kd * ( Control->error_0 - Control->error_1);

	Control->OutPreSat = Control->Value_Kp + Control->Value_Ki + Control->Value_Kd + Control->Kc*Control->SatErr;

	if (Control->OutPreSat>MaxValue){
		Control->OutputValue = MaxValue;
	}
	else if (Control->OutPreSat<MinValue){
		Control->OutputValue = MinValue;
	}
	else
		Control->OutputValue = Control->OutPreSat;

	Control->SatErr = Control->OutputValue - Control->OutPreSat;
	Control->Sum_error = Control->Value_Ki;
	Control->error_1 = Control->error_0;//����һ��г��
	Control->error_2 = Control->error_1;

	if(Control->OutputValue<0)
		Control->OutputValue = -Control->OutputValue;
//	Out = (uint16_t)Control->OutputValue;

	return (Control->OutputValue) ;

}

/*uint16_t PID_P_calculate(struct PID *Control,float CurrentValue )// 位置式PID with anti windup

{
	float OutTemp;
	uint16_t Out;

	Control->error_0 = Control->OwenValue - CurrentValue;

	if(Control->OutputValue>2200)
	{
	  if(Control->error_0<=0)
	  {
		  Control->Sum_error+=Control->error_0;
	  }
	}
	else if(Control->OutputValue<-2200)
	{
	  if(Control->error_0>=0)
	  {
		  Control->Sum_error+=Control->error_0;
	  }
	}
	else
	{
	Control->Sum_error+=Control->error_0;
	}

	Control->OutputValue=Control->Kp*Control->error_0+Control->Ki*Control->Sum_error+Control->Kd*(Control->error_0-Control->error_1);

	Control->error_1=Control->error_0;

	if (Control->OutPreSat>MaxValue){
		Control->OutputValue = MaxValue;
	}
	else if (Control->OutPreSat<MinValue){
		Control->OutputValue = MinValue;
	}

	OutTemp = Control->OutputValue;
	if(OutTemp<0)
		OutTemp = -OutTemp;
	Out = (uint16_t)OutTemp;

	return Out;

}*/

float PID_I_calculate(struct PID *Control,float CurrentValue )//λ��PID����B 位置式PID
{

	float Value_Kp;//��������
	float Value_Ki;//���ַ���
	float Value_Kd;//΢�ַ���

	Control->error_0 = Control->OwenValue - CurrentValue; //+ 0*span;//����������Control->OwenValueΪ��Ҫ���ٶȣ�CurrentValue_leftΪ�����ʵ�ٶ�

	Value_Kp = Control->Kp * (Control->error_0-Control->error_1) ;
//	Control->Sum_error += Control->error_0;

	Value_Ki = Control->Ki * Control->error_0;

	Value_Kd = Control->Kd * ( Control->error_0 - 2*Control->error_1 + Control->error_2);

	Control->OutputValue = Value_Kp  + Value_Ki + Value_Kd;//���ֵ���㣬ע��Ӽ�

//	if(Control->OutputValue>10000 || Control->OutputValue<-10000)//-5000)
//	{
//		Control->Ki = -10;
////		Control->Kd = 10;
//	}
//	else {
//		Control->Ki = 100;
//		//Control->Kd = 0;
//	}

	Control->error_1 = Control->error_0;//����һ��г��
	Control->error_2 = Control->error_1;

	OutputValue = Control->OutputValue;

	if(OutputValue>MaxValue){
		OutputValue = MaxValue;
	}
	else if(OutputValue<MinValue){
		OutputValue = MinValue;
	}


	if(OutputValue<0)
		OutputValue = -OutputValue;

	return (OutputValue) ;

}

