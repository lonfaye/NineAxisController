/*
 * contact.h
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */

#ifndef INC_CONTACT_H_
#define INC_CONTACT_H_

//#include "stm32f10x.h"
#include "stm32f1xx_it.h"

#include "PID.h"
#include "encoder.h"

#include "math.h"
#include <stdio.h>
//#include "cstring"


void LeftMovingSpeedW(unsigned int val);//���ַ�����ٶȿ��ƺ���
void RightMovingSpeedW(unsigned int val2);//���ַ�����ٶȿ��ƺ���

void car_control(float rightspeed,float leftspeed);//С���ٶ�ת���Ϳ��ƺ���

//void Contact_Init(void);//�����ַ�����ٶȳ�ʼ��

#endif /* INC_CONTACT_H_ */
