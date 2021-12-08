/*
 * odometry.c
 *
 *  Created on: 2021��9��16��
 *      Author: wlf
 */
#include "odometry.h"

/***********************************************  ���  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

/***********************************************  ����  *****************************************************************/

extern float odometry_right,odometry_left;//���ڵõ����������ٶ�

//extern union odometry vel_left,vel_right;

/***********************************************  ����  *****************************************************************/

float wheel_interval= 0.177f;//268.0859f;//    272.0f;        //  1.0146 ��λm
//float wheel_interval=276.089f;    //���У��ֵ=ԭ���/0.987

float multiplier=4.0f;           //��Ƶ��
float deceleration_ratio=30.0f;  //���ٱ�
float wheel_diameter=0.068f;     //����ֱ������λm
float pi_1_2=1.570796f;			 //��/2
float pi=3.141593f;              //��
float pi_3_2=4.712389f;			 //��*3/2
float pi_2_1=6.283186f;			 //��*2
float dt=0.001f;                 //����ʱ����1ms
float line_number=500.0f;       //��������
float oriention_interval=0;  //dtʱ���ڷ���仯ֵ

float sin_=0;        //�Ƕȼ���ֵ
float cos_=0;
float theta = 0;
float delta_distance=0,delta_oriention=0;   //����ʱ�������˶��ľ���

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0,linear_vel=0,angular_vel=0;;

float oriention_1=0;

unsigned char  once=1;

/****************************************************************************************************************/

//��̼Ƽ��㺯��
void odometry(float right,float left)
{
	if(once)  //����������һ��
	{
//		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio);
		const_frame = wheel_diameter*pi/60; //
		const_angle=const_frame/wheel_interval;
		once=0;
	}
//	right = right*1000;
//	left = left*1000;//1ms

//	vel_left = left*const_frame;  //unit:m/s
//	vel_right = right*const_frame; //unit:m/sd
	linear_vel = 0.5f*(right+left)*const_frame; //unit:m/s
	angular_vel = (right-left)*const_angle; //unit:rad/s //逆时针为正

	theta = theta+angular_vel*dt;
//	theta = fmod(theta,pi_2_1);

	/*
	 * if angular_vel>0 right>left,car turn right angular is negative,else if angular_vel<0 right<left,can turn left,angular is positive
	 * */
	sin_ = sin(theta);//���������ʱ����y����
	cos_ = cos(theta);//���������ʱ����x����

	position_x = position_x + linear_vel*cos_*dt;//�������̼�x����
	position_y = position_y + linear_vel* sin_*dt;//�������̼�y����

//	distance_sum = 0.5f*(right+left);//�ں̵ܶ�ʱ���ڣ�С����ʻ��·��Ϊ�����ٶȺ�
//	distance_diff = right-left;//�ں̵ܶ�ʱ���ڣ�С����ʻ�ĽǶ�Ϊ�����ٶȲ�
//
//    //���������ֵķ��򣬾�����ʱ���ڣ�С����ʻ��·�̺ͽǶ���������
//	if((odometry_right>0)&&(odometry_left>0))            //���Ҿ���
//	{
//		delta_distance = distance_sum;
//		delta_oriention = distance_diff;
//	}
//	else if((odometry_right<0)&&(odometry_left<0))       //���Ҿ���
//	{
//		delta_distance = -distance_sum;
//		delta_oriention = -distance_diff;
//	}
//	else if((odometry_right<0)&&(odometry_left>0))       //�����Ҹ�
//	{
//		delta_distance = -distance_diff;
//		delta_oriention = -2.0f*distance_sum;
//	}
//	else if((odometry_right>0)&&(odometry_left<0))       //������
//	{
//		delta_distance = distance_diff;
//		delta_oriention = 2.0f*distance_sum;
//	}
//	else
//	{
//		delta_distance=0;
//		delta_oriention=0;
//	}
//
//	oriention_interval = delta_oriention * const_angle;//����ʱ�����ߵĽǶ�
//	oriention = oriention + oriention_interval;//�������̼Ʒ����
//	oriention_1 = oriention + 0.5f * oriention_interval;//��̼Ʒ��������λ���仯���������Ǻ�������
//
//    sin_ = sin(oriention_1);//���������ʱ����y����
//	cos_ = cos(oriention_1);//���������ʱ����x����
//
//    position_x = position_x + delta_distance * cos_ * const_frame;//�������̼�x����
//	position_y = position_y + delta_distance * sin_ * const_frame;//�������̼�y����
//
//	velocity_linear = delta_distance*const_frame / dt;//�������̼����ٶ�
//	velocity_angular = oriention_interval / dt;//�������̼ƽ��ٶ�
//
//    //����ǽǶȾ���
//	if(oriention > pi)
//	{
//		oriention -= pi_2_1;
//	}
//	else
//	{
//		if(oriention < -pi)
//		{
//			oriention += pi_2_1;
//		}
//	}
}
