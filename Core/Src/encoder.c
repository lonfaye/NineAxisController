/*
 * encoder.c
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */


#include "encoder.h"
#include <math.h>

/****************************************************************************************************************/

int16_t hSpeed_Buffer2[SPEED_BUFFER_SIZE]={0}, hSpeed_Buffer1[SPEED_BUFFER_SIZE]={0};//�������ٶȻ�������
static float hRot_Speed2;//���Aƽ��ת�ٻ���
static float hRot_Speed1;//���Bƽ��ת�ٻ���
//unsigned int Speed2=0; //���Aƽ��ת�� r/min��PID����
//unsigned int Speed1=0; //���Bƽ��ת�� r/min��PID����

float spd1PITuning,spd2PITuning;

static volatile uint16_t hEncoder_Timer_Overflow1;//���B�������ɼ�
static volatile uint16_t hEncoder_Timer_Overflow2;//���A�������ɼ�

//float A_REMP_PLUS;//���APID���ں��PWMֵ����
float pulse = 0;//���A PID���ں��PWMֵ����
float pulse1 = 0;//���B PID���ں��PWMֵ����

//int span;//�ɼ��������������ٶȲ�ֵ

static bool bIs_First_Measurement2 = true;//���A������ٶȻ��������־λ
static bool bIs_First_Measurement1 = true;//���B������ٶȻ��������־λ

struct PID Control_left  ={30.5,1.5,0,0,0,0,0,0,0,0,0,0,0,0,0};//����PID�����������µ��4096
struct PID Control_right ={30.5,1.5,0,0,0,0,0,0,0,0,0,0,0,0,0};//����PID�����������µ��4096
//struct PID Control_right ={0.01,0.1,0.75,0,0,0,0,0,0};//����PID�����������µ��4096

/****************************************************************************************************************/

int32_t hPrevious_angle2, hPrevious_angle1;

/****************************************************************************************************************/

/****************************************************************************************************************/

void ENC_Clear_Speed_Buffer(void)//�ٶȴ洢������
{
    uint8_t i;

    //����������ٶȻ�������
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
        hSpeed_Buffer1[i] = 0;
        hSpeed_Buffer2[i] = 0;
    }

    bIs_First_Measurement2 = true;//���A������ٶȻ��������־λ
    bIs_First_Measurement1 = true;//���B������ٶȻ��������־λ
}

void ENC_Calc_Average_Speed(void)//�������ε����ƽ��������
{
    uint8_t i;
	int32_t wtemp3=0;
	int32_t wtemp4=0;

    //�ۼӻ�������ڵ��ٶ�ֵ
	for (i=0;i<SPEED_BUFFER_SIZE;i++)
	{
		wtemp3 += hSpeed_Buffer1[i];//right
		wtemp4 += hSpeed_Buffer2[i];//left
	}

    //ȡƽ����ƽ����������λΪ ��/s
	wtemp3 /= (SPEED_BUFFER_SIZE);
	wtemp4 /= (SPEED_BUFFER_SIZE); //ƽ�������� ��/s

    //��ƽ����������λתΪ r/min
//	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER1_PPR);
//	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(4*ENCODER2_PPR);
	wtemp3 = (wtemp3 * SPEED_SAMPLING_FREQ)*60/(ENCODER1_PPR);
	wtemp4 = (wtemp4 * SPEED_SAMPLING_FREQ)*60/(ENCODER2_PPR);

	hRot_Speed2= (float)(wtemp4);//ƽ��ת�� r/min//left
	hRot_Speed1= (float)(wtemp3);//ƽ��ת�� r/min//right
	hRot_Speed1 = fabs(hRot_Speed1);
	hRot_Speed2 = fabs(hRot_Speed2);

//	Speed2=hRot_Speed2;//ƽ��ת�� r/min
//	Speed1=hRot_Speed1;//ƽ��ת�� r/min
	spd1PITuning=hRot_Speed1;//ƽ��ת�� r/min
	spd2PITuning=hRot_Speed2;//ƽ��ת�� r/min
}

/****************************************************************************************************************/

float Gain2(void)//���õ��A PID���� PA2//left
{
	//static float pulse = 0;

//	span=1*(Speed1-Speed2);//�ɼ��������������ٶȲ�ֵ
//	pulse= pulse + PID_calculate(&Control_right,hRot_Speed2);//PID����
	pulse= PID_P_calculate(&Control_left,hRot_Speed2);//PID����

    //pwm��������
	if(pulse > 10000) pulse = 10000;
	if(pulse < 0) pulse = 0;

	return pulse;
	//A_REMP_PLUS=pulse;//���APID���ں��PWMֵ����
}


float Gain1(void)//���õ��B PID���� PA1//right
{
	//static float pulse1 = 0;

//	span=1*(Speed2-Speed1);//�ɼ��������������ٶȲ�ֵ
//	pulse1= pulse1 + PID_calculate(&Control_left,hRot_Speed1);//PID����
	pulse1= PID_P_calculate(&Control_right,hRot_Speed1);//PID����

    ////pwm ��������
	if(pulse1 > 10000) pulse1 = 10000;
	if(pulse1 < 0) pulse1 = 0;

	return pulse1;

}
/****************************************************************************************************************/

