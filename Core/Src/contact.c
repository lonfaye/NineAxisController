/*
 * contact.c
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */


#include "contact.h"

/***********************************************  ���  *****************************************************************/

/***********************************************  ����  *****************************************************************/

extern struct PID Control_left;//����PID�����������µ��4096
extern struct PID Control_right;//����PID�����������µ��4096

/***********************************************  ����  *****************************************************************/

/*******************************************************************************************************************/

void LeftMovingSpeedW(unsigned int val)//���ַ�����ٶȿ��ƺ���
{
    if(val>10000)
    {
//        GPIO_SetBits(GPIOC, GPIO_Pin_6);
//        GPIO_ResetBits(GPIOC, GPIO_Pin_7);

        Control_left.OwenValue=(val-10000);//PID���ڵ�Ŀ�������
    }
    else if(val<10000)
    {
//        GPIO_SetBits(GPIOC, GPIO_Pin_7);
//        GPIO_ResetBits(GPIOC, GPIO_Pin_6);

        Control_left.OwenValue=(10000-val);//PID���ڵ�Ŀ�������
    }
    else
    {
//         GPIO_SetBits(GPIOC, GPIO_Pin_6);
//         GPIO_SetBits(GPIOC, GPIO_Pin_7);

         Control_left.OwenValue=0;//PID���ڵ�Ŀ�������
    }
}

void RightMovingSpeedW(unsigned int val2)//���ַ�����ٶȿ��ƺ���
{
    if(val2>10000)
    {
        /* motor A ��ת*/
//        GPIO_SetBits(GPIOC, GPIO_Pin_10);
//        GPIO_ResetBits(GPIOC, GPIO_Pin_11);

        Control_right.OwenValue=(val2-10000);//PID���ڵ�Ŀ�������
    }
    else if(val2<10000)
    {
        /* motor A ��ת*/
//        GPIO_SetBits(GPIOC, GPIO_Pin_11);
//        GPIO_ResetBits(GPIOC, GPIO_Pin_10);

        Control_right.OwenValue=(10000-val2);//PID���ڵ�Ŀ�������
    }
    else
    {
//        GPIO_SetBits(GPIOC, GPIO_Pin_10);
//        GPIO_SetBits(GPIOC, GPIO_Pin_11);

        Control_right.OwenValue=0;//PID���ڵ�Ŀ�������
    }
}

void car_control(float rightspeed,float leftspeed)//С���ٶ�ת���Ϳ��ƺ���
{
//    float k2=17.179;         //�ٶ�ת������,ת/����
//
//    //���Ӵ��ڽ��յ����ٶ�ת����ʵ�ʿ���С�����ٶȣ�����PWM��
//    int right_speed=(int)k2*rightspeed;
//    int left_speed=(int)k2*leftspeed;
//
//    RightMovingSpeedW(right_speed+10000);
//    LeftMovingSpeedW(left_speed+10000);
	Control_right.OwenValue = rightspeed;
	Control_left.OwenValue = leftspeed;
}

//void Contact_Init(void)//�����ַ�����ٶȳ�ʼ��
//{
//	LeftMovingSpeedW(12000); //���B
//	RightMovingSpeedW(12000);//���A
//}

