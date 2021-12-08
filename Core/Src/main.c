/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "contact.h"
#include "odometry.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "storage_manager.h"
#include "inv_mpu.h"
#include "IOI2C.h"
//#include "../MPL/mpu6050.h"
#include "../delay/delay.h"
#include "mpu9250.h"
#include "W25QXX.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USART_REC_LEN  			16//200  	//定义接收字节长度
#define USART_SEN_LEN           84         //定义发�?�字节数
//#define DEFAULT_MPU_HZ 200
#define q30  1073741824.0f
//与IMU�??螺仪设置的量程有关，量程±500°，对应数据范围为±32768
//�??螺仪原始数据转换位弧�??(rad)单位�??1/65.5/57.30=0.00026644
//#define GYROSCOPE_RATIO   0.00026644f
//与IMU加�?�度计设置的量程有关，量程�?2g，对应数据范围�?32768
//加�?�度计原始数据转换位m/s^2单位�??32768/2g=32768/19.6=1671.84
//#define ACCEl_RATIO 	  1671.84f

// ---------------- POPULAR POLYNOMIALS ----------------
// CCITT:      x^16 + x^12 + x^5 + x^0                 (0x1021)
// CRC-16:     x^16 + x^15 + x^2 + x^0                 (0x8005)
#define         CRC_16_POLYNOMIALS      0x8005

/*****printf redirection*****/
#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
 return ch;
}
#endif
/*******************************/

// --------------------------------------------------------------
//      CRC16计算方法1:使用 256长度的校验表
// --------------------------------------------------------------
const uint8_t chCRCHTalbe[] =                                 // CRC 高位字节值表
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40
};

const uint8_t chCRCLTalbe[] =                                 // CRC 低位字节值表
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
0x41, 0x81, 0x80, 0x40
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t keynumX; //exit var



uint32_t cr1its;

uint32_t icnt ;
uint16_t pulseR,pulseL;

extern struct PID Control_left;//
extern struct PID Control_right;//
float spdref;

uint8_t txBuf[4];
uint8_t ACKBuf[] = "The PWM duty setting.\r\n";
uint8_t rxBuf[6];
uint8_t aRxBuffer[1];			//HAL库USART接收Buffer
uint8_t flag = 0; //接收结束标志
static uint8_t state = 0; //消息接收状�??
static uint8_t recv_cnt = 0; //
static uint16_t header = 0; //
static uint8_t pkt_len; //消息长度
static uint16_t checksum = 0; //CRC

/***flash spi params*/
uint8_t wData[0x10];
uint8_t rData[0x10];
uint8_t ID[2];
static uint8_t  mpudateBuff[128]={0};
uint8_t calflg = 0;
uint8_t calQuatFlg = 0;
size_t size=0;
float yawavg[10]={};
float yawoffset = 0;

//encoder params
int16_t SpdSum1;
int16_t SpdSum2;
int16_t oldSpdSum1;
int16_t oldSpdSum2;
float Spd1;
float Spd2;
uint8_t dir1;
uint8_t dir2;
uint8_t dirEnc1;
uint8_t dirEnc2;
bool DirectionL;
bool DirectionR;
extern int16_t hSpeed_Buffer1[],hSpeed_Buffer2[];//左右轮�?�度缓存数组
extern float spd1PITuning,spd2PITuning; //PI tuning
uint8_t bSpeed_Buffer_Index = 0;//缓存左右轮编码数到数组变�??
uint8_t ret1,ret2;
//DMP setting orientation
//static signed char gyro_orientation[9] = {0, -1, 0, 1,0, 0, 0, 0, 1};
//short gyro[3], accel[3], sensors;
float fAccel[3],fGyro[3],fQuat[4];
//9轴IMU数据

unsigned char Send_Count;
//odometry params
//communication protocol define for receive data  to upper system
uint16_t sndCRCVal;
uint8_t odometry_data[USART_SEN_LEN]={0};   //发�?�给串口的里程计数组

float odometry_right=0,odometry_left=0;//串口得到的左右轮速度
extern float position_x,position_y,oriention,velocity_linear,velocity_angular,theta,linear_vel,angular_vel;         //计算得到的里程计数�??

uint8_t USART_RX_BUF[USART_REC_LEN];     //串口接收缓冲
extern uint16_t USART_RX_STA;                   //串口接收状�?�标志位

float Milemeter_L_Motor,Milemeter_R_Motor;     //dt时间内的左右轮�?�度,用于里程计计�??

union pusleData  //接收到的数据
{
	uint32_t pulse_d;    //左右轮脉�??
	uint8_t pulse_data[4];
}leftrightpulse;       //接收的左右轮数据

union imudata  //接收到的数据
{
	float imu_d;    //euler angle
	uint8_t imu_data[4];
}Pitch,Roll,Yaw,YawOS,Pitch1,Roll1,Yaw1;       //接收的左右轮数据

union quatdata  //接收到的数据
{
	float quat_d;    //四元�??
	uint8_t qua_data[4];
}q0,q1,q2,q3,q0_o,q1_o,q2_o,q3_o,q0_,q1_,q2_,q3_;

union quatdata_inv  //
{
	float quat_d;    //
	uint8_t qua_data[4];
}q0_inv,q1_inv,q2_inv,q3_inv;

union accdata  //
{
	float acc_d;    //
	uint8_t acc_data[4];
}x_acc,y_acc,z_acc;

union gyrodata  //
{
	float gyro_d;    //
	uint8_t gyro_data[4];
}x_gyro,y_gyro,z_gyro;

union recieveData  //接收到的数据
{
	float d;    //左右轮�?�度
	uint8_t data[4];
}leftdata,rightdata;       //接收的左右轮数据

union odometry  //里程计数据共用体
{
	float odoemtry_float;
	uint8_t odometry_char[4];
}x_data,y_data,vel_left,vel_right,theta_data,vel_linear,vel_angular;     //要发布的里程计数据，分别为：X，Y方向移动的距离，左轮速度，右轮�?�度，当前角度，线�?�度，角速度

uint8_t stateHandle=0; //用作处理主函数各种if，去掉多余的flag

/****************************************************************************************************************/

int32_t hPrevious_angle2, hPrevious_angle1;

/****************************************************************************************************************/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*****************************************************
 * *************** DMP init function *****************
 * ***************************************************/
#define SAMPLING_FREQ 20.0f // 采样频率
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
void Quaternion_Solution(float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float twoKp = 1.0f;     // 2 * proportional gain (Kp)
  float twoKi = 0.0f;     // 2 * integral gain (Ki)
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的�??(三维向量)转化为单位向量，即向量除以模
    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    ax /= recipNorm;
    ay /= recipNorm;
    az /= recipNorm;
    // 把四元数换算成方向余弦中的第三行的三个元�??
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    //误差是估计的重力方向和测量的重力方向的交叉乘积之�??
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用�??
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
      gx += integralFBx;        // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;       // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / SAMPLING_FREQ));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / SAMPLING_FREQ));
  gz *= (0.5f * (1.0f / SAMPLING_FREQ));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  // Normalise quaternion
  recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= recipNorm;
  q1 /= recipNorm;
  q2 /= recipNorm;
  q3 /= recipNorm;
  q0_.quat_d = q0;
  q1_.quat_d = q1;
  q2_.quat_d = q2;
  q3_.quat_d = q3;
}

void Quat2Euler(void)
{
//	unsigned long sensor_timestamp;
//	unsigned char more;
//	long quat[4];

	 Pitch1.imu_d = asin(-2 * q1_inv.quat_d * q3_inv.quat_d + 2 * q0_inv.quat_d* q2_inv.quat_d)* 57.3;
	 Roll1.imu_d = atan2(2 * q2_inv.quat_d * q3_inv.quat_d + 2 * q0_inv.quat_d * q1_inv.quat_d, -2 * q1_inv.quat_d * q1_inv.quat_d - 2 * q2_inv.quat_d* q2_inv.quat_d + 1)* 57.3; // roll
	 Yaw1.imu_d = 	atan2(2*(q1_inv.quat_d*q2_inv.quat_d + q0_inv.quat_d*q3_inv.quat_d),q0_inv.quat_d*q0_inv.quat_d+q1_inv.quat_d*q1_inv.quat_d-q2_inv.quat_d*q2_inv.quat_d-q3_inv.quat_d*q3_inv.quat_d) * 57.3;//yaw
	 Roll1.imu_d = -Roll1.imu_d;
	 Yaw1.imu_d = -Yaw1.imu_d;

//	while(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more)!=0);
//	if (sensors & INV_WXYZ_QUAT )
//	{
//		 q0.quat_d=quat[0] / q30;   //w
//		 q1.quat_d=quat[1] / q30;   //x
//		 q2.quat_d=quat[2] / q30;   //y
//		 q3.quat_d=quat[3] / q30;   //z
//		 Pitch1.imu_d = asin(-2 * q1.quat_d * q3.quat_d + 2 * q0.quat_d* q2.quat_d)* 57.3;
//		 Roll1.imu_d = atan2(2 * q2.quat_d * q3.quat_d + 2 * q0.quat_d * q1.quat_d, -2 * q1.quat_d * q1.quat_d - 2 * q2.quat_d* q2.quat_d + 1)* 57.3; // roll
//		 Yaw1.imu_d = 	atan2(2*(q1.quat_d*q2.quat_d + q0.quat_d*q3.quat_d),q0.quat_d*q0.quat_d+q1.quat_d*q1.quat_d-q2.quat_d*q2.quat_d-q3.quat_d*q3.quat_d) * 57.3;//yaw
//		 Roll1.imu_d = -Roll.imu_d;
//		 Yaw1.imu_d = -Yaw.imu_d;
//	}
//	if (sensors & INV_XYZ_ACCEL )
//	{
//		 x_acc.acc_d = accel[0]/ACCEl_RATIO;
//		 y_acc.acc_d = accel[1]/ACCEl_RATIO;
//		 z_acc.acc_d = accel[2]/ACCEl_RATIO;
//	}
//	if (sensors & INV_XYZ_GYRO )
//	{
//		 x_gyro.gyro_d = gyro[0]*GYROSCOPE_RATIO;
//		 y_gyro.gyro_d = gyro[1]*GYROSCOPE_RATIO;
//		 z_gyro.gyro_d = gyro[2]*GYROSCOPE_RATIO;
//	}

}

void Euler2Quat(float roll,float pitch,float yaw)
{
	roll  = roll*M_PI/180;
	pitch  = pitch*M_PI/180;
	yaw  = yaw*M_PI/180;
	q0.quat_d = cosf(roll/2)*cosf(pitch/2)*cosf(yaw/2)+sinf(roll/2)*sinf(pitch/2)*sinf(yaw/2);
	q1.quat_d = sinf(roll/2)*cosf(pitch/2)*cosf(yaw/2)-cosf(roll/2)*sinf(pitch/2)*sinf(yaw/2);
	q2.quat_d = cosf(roll/2)*sinf(pitch/2)*cosf(yaw/2)+sinf(roll/2)*cosf(pitch/2)*sinf(yaw/2);
	q3.quat_d = cosf(roll/2)*cosf(pitch/2)*sinf(yaw/2)-sinf(roll/2)*sinf(pitch/2)*cosf(yaw/2);
}

void Euler2Quat_O(float roll,float pitch,float yaw)
{
	roll  = roll*M_PI/180;
	pitch  = pitch*M_PI/180;
	yaw  = yaw*M_PI/180;
	q0_o.quat_d = cosf(roll/2)*cosf(pitch/2)*cosf(yaw/2)+sinf(roll/2)*sinf(pitch/2)*sinf(yaw/2);
	q1_o.quat_d = sinf(roll/2)*cosf(pitch/2)*cosf(yaw/2)-cosf(roll/2)*sinf(pitch/2)*sinf(yaw/2);
	q2_o.quat_d = cosf(roll/2)*sinf(pitch/2)*cosf(yaw/2)+sinf(roll/2)*cosf(pitch/2)*sinf(yaw/2);
	q3_o.quat_d = cosf(roll/2)*cosf(pitch/2)*sinf(yaw/2)-sinf(roll/2)*sinf(pitch/2)*cosf(yaw/2);
}

unsigned char DataScope_OutPut_Buffer[42];

void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
  }
}

unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //
  else
  {
	 DataScope_OutPut_Buffer[0] = '$';  //帧头

	 switch(Channel_Number)
   {
		 case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;
		 case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
		 case 3:   DataScope_OutPut_Buffer[13] = 13; return 14;
		 case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
		 case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;
		 case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
		 case 7:   DataScope_OutPut_Buffer[29] = 29; return 30;
		 case 8:   DataScope_OutPut_Buffer[33] = 33; return 34;
		 case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
     case 10:  DataScope_OutPut_Buffer[41] = 41; return 42;
   }
  }
	return 0;
}

void DataScope(void)
{
//	unsigned char i;
	DataScope_Get_Channel_Data( Pitch.imu_d, 1 );
	DataScope_Get_Channel_Data( Roll.imu_d, 2 );
	DataScope_Get_Channel_Data( Yaw.imu_d, 3 );
	DataScope_Get_Channel_Data( YawOS.imu_d, 4 );
//	DataScope_Get_Channel_Data( Yaw1.imu_d, 5 );
//	DataScope_Get_Channel_Data( spd1PITuning , 1 );
//	DataScope_Get_Channel_Data(spdref, 2 ); //����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0 , 6 );//����Ҫ��ʾ�������滻0������
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 );
//		DataScope_Get_Channel_Data(0, 9 );
//		DataScope_Get_Channel_Data( 0 , 10);
	Send_Count = DataScope_Data_Generate(4);//parameter is channel numbers
	HAL_UART_Transmit_IT(&huart1,(uint8_t*)&DataScope_OutPut_Buffer,Send_Count);
//	for( i = 0 ; i < Send_Count; i++)
//	{
//		unsigned char val;
//		val = DataScope_OutPut_Buffer[i];
//		HAL_UART_Transmit_IT(&huart1,val,sizeof(unsigned char));
//	}
}

/*******************************************************
 * *****************************************************/

uint16_t CRC16(uint8_t* pchMsg, uint16_t wDataLen)
{
	uint8_t chCRCHi = 0xFF; // 高CRC字节初始�??
	uint8_t chCRCLo = 0xFF; // 低CRC字节初始�??
	uint16_t wIndex;            // CRC循环中的索引

	while (wDataLen--)
	{
			// 计算CRC
			wIndex = chCRCLo ^ *pchMsg++ ;
			chCRCLo = chCRCHi ^ chCRCHTalbe[wIndex];
			chCRCHi = chCRCLTalbe[wIndex] ;
	}

	return ((chCRCHi << 8) | chCRCLo) ;
}



//EXINT Callback PA0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//外部中断
{
	//TO-DO encoder process
 /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
		   the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
//  if(GPIO_Pin==GPIO_PIN_0)//编码器计数，�?????????????????????????????????????????????????测到PB15(A�?????????????????????????????????????????????????)跳变�?????????????????????????????????????????????????
//  {
//	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==1)//PB3(B�?????????????????????????????????????????????????)高电平正�?????????????????????????????????????????????????
//		SpdSum1++;
//
//	else
//		SpdSum1--;
//  }
//  if(GPIO_Pin==GPIO_PIN_6)//编码器计数，�?????????????????????????????????????????????????测到PB15(A�?????????????????????????????????????????????????)跳变�?????????????????????????????????????????????????
//  {
//	if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)==1)//PB3(B�?????????????????????????????????????????????????)高电平正�?????????????????????????????????????????????????
//		SpdSum2++;
//	else
//		SpdSum2--;
//  }

    if(GPIO_Pin==GPIO_PIN_0)//
    {
    	calflg = 1;
//    	 HAL_UART_Transmit(&huart1,ACKBuf,sizeof(ACKBuf),0xff);

/*********************************************************************************/
/*********test motor control ************************/
//    	switch(keynumX)
//    	{
//			case 0:
////				 Control_right.Sum_error = 0;
//				 spdref=0.5;
//				 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
//				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//				 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//				 keynumX++;
////				HAL_UART_Transmit(&huart1,"a",1,0xff);
//				 break;
//			case 1:
////				  Control_right.Sum_error = 0;
//				  spdref =-0.5;
//				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//				  keynumX++;
////				HAL_UART_Transmit(&huart1,"b",1,0xff);
//				  break;
//			case 2:
////				Control_right.Sum_error = 0;
//				  spdref = 0;
//				  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
//				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
//				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//				  keynumX = 0;
////				HAL_UART_Transmit(&huart1,"c",1,0xff);
//				  break;
//			default:
//				  break;
//    	}

    }
}

/*
 * Encoder interrupt callback
 * */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{

		oldSpdSum1++;
		if(htim->Instance->CR1==0x01)//顺时�??
			dirEnc1=1;
		if(htim->Instance->CR1==0x11)//逆时�??
			dirEnc1=2;
	}
	if(htim->Instance == TIM3)
	{
		oldSpdSum2++;
		if(htim->Instance->CR1==0x01)//顺时�??
			dirEnc2=1;
		if(htim->Instance->CR1==0x11)//逆时�??
			dirEnc2=2;
	}
}

/*
 * UART Error CallBack
 * */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		__HAL_UNLOCK(&huart1);
		HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
	}
}

/*
 * UART Receive Callback to jetson NX
 * */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//TO-DO uart receive process
/**
 * communication protocol define for receive data  from upper system
 * frame header 0x55 0xAA
 * function code 0x01->startup,0x02->receive spd cmd,0x03->stop,0x04->others
 * length
 * data
 * CRC 2 bytes
 * startup frame example:0x55 0xAA 0x01 0x0a,dir1,dir2,spd1_B1,spd1_B2 spd1_B3 spd1_B4 spd2_B1 spd2_B2 spd2_B3 spd2_B4 CRCL,CRCH (dir.x=1->forward,dir.x=2->backward)
 * spd cmd frame example:0x55 0xAA 0x01 0x0a,spd1_B1,spd1_B2 spd1_B3 spd1_B4 spd2_B1 spd2_B2 spd2_B3 spd2_B4 0x00 CRCL,CRCH (spd.x_B.x:float)
 * stop  frame  example: 0x55 0xAA 0x01 0x0a,0x00,0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 CRCL,CRCH
 */
/* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
		   the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  uint8_t t =0;
  if(huart->Instance == USART1)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
//	  usart1_tim = 1;
	  if(flag!=1)
	  {
		  if(state == 0)
		  {
			  if(aRxBuffer[0] ==0x55)
			      header = 0x0055;
			  else if(aRxBuffer[0] ==0xAA)
			  {
				  header=(header<<8)|aRxBuffer[0];
				  if(header==0x55AA)
				  {//收到包头
					  state=1;
					  USART_RX_BUF[0]=0x55;
					  USART_RX_BUF[1]=0xAA;
					  recv_cnt=2;
				  }
			  }
			  else
				  header = 0;
		  }
		  else if(state ==1)
		  {
			  USART_RX_BUF[recv_cnt++]=aRxBuffer[0];
			  state =2;
		  }
		  else if(state ==2)
		  {
			  pkt_len=aRxBuffer[0];
			  USART_RX_BUF[recv_cnt++]=pkt_len;
			  if(pkt_len==10)
				  state=3;
			  else
				  state=0;
		  }
		  else if(state ==3)
		  {
			 USART_RX_BUF[recv_cnt++]=aRxBuffer[0];
		     if(recv_cnt-4==pkt_len) //pkt_len
		    	 state=4;
//		     else if(pkt_len!=10)
//		    	 state=0;

		  }
		  else if(state == 4)
		  {
			  USART_RX_BUF[recv_cnt++]=aRxBuffer[0];
//			  checksum = USART_RX_BUF[recv_cnt];
			  state=5;

		  }
		  else if(state == 5)
		  {
			  USART_RX_BUF[recv_cnt++]=aRxBuffer[0];
//			  checksum = checksum<<8|USART_RX_BUF[recv_cnt];
			  checksum = CRC16(USART_RX_BUF,USART_REC_LEN-2);
			  if (checksum == (uint16_t)(USART_RX_BUF[14]<<8|USART_RX_BUF[15]))  //ros 14<<8 sscom 15<<8
			  {
				  if(USART_RX_BUF[2]==0x01)
				  {
					  //startup motor
					 // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
					 // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		  //			  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		  //			  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
					  dir1 = USART_RX_BUF[4];
					  dir2 = USART_RX_BUF[5];
					  for(t=0;t<4;t++)
					  {
						  rightdata.data[t]=USART_RX_BUF[t+6];
						  leftdata.data[t]=USART_RX_BUF[t+10];
					  }

					  //储存左右轮�?�度
					  odometry_right=rightdata.d;//单位mm/s
					  odometry_left=leftdata.d;//单位mm/s
					  if(dir1==1)
					  {
						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		  //				  rightdata.d = -rightdata.d;

					  }
					  if(dir1==2)
					  {
						  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					  }
					  if(dir2==1)
					  {
						  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
						  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  //				  leftdata.d = -leftdata.d;
					  }
					  else if(dir2==2)
					  {
						  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
						  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					  }
					  else
					  {

					  }

//					  HAL_UART_Transmit_IT(&huart1,ACKBuf,sizeof(ACKBuf));
				  }
				  if(USART_RX_BUF[2]==0x02)
				  {
					  //receive spd cmd
					  //接收左右轮�?�度
					  for(t=0;t<4;t++)
					  {
						  rightdata.data[t]=USART_RX_BUF[t+4];
						  leftdata.data[t]=USART_RX_BUF[t+8];
					  }

					  //储存左右轮�?�度
					  odometry_right=rightdata.d;//单位mm/s
					  odometry_left=leftdata.d;//单位mm/s
//					  HAL_UART_Transmit_IT(&huart1,ACKBuf,sizeof(ACKBuf));

				  }
				  if(USART_RX_BUF[2]==0x03)
				  {
					  // stop motor
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		  //			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		  //			  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//					  HAL_UART_Transmit_IT(&huart1,ACKBuf,sizeof(ACKBuf));
				  }
			  }
//			  flag=1;
			  state=0;
			  recv_cnt=0;

		  }
	  }
	  HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
//      usart1_tim = 0;
  }

}

// timer interrupt callback 1ms period

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//		tim1_tim = 1;
		icnt++;
		stateHandle |= 0x01;

		//encoder1
		SpdSum1 = (int16_t)(__HAL_TIM_GET_COUNTER(&htim4));//获取定时器的�??(divided by 4 for both rise and fall edge counting)
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		Spd1 = (float)SpdSum1;
		hSpeed_Buffer1[bSpeed_Buffer_Index] = SpdSum1;//temp2;right

		//encoder2
		SpdSum2 = (int16_t)(__HAL_TIM_GET_COUNTER(&htim3));//获取定时器的�??
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		Spd2 = (float)SpdSum2;
		hSpeed_Buffer2[bSpeed_Buffer_Index] = SpdSum2;//left

		//储存编码数（脉冲数），用于里程计计算
		Milemeter_L_Motor= Spd2;//(float)temp2; //储存脉冲数组
		Milemeter_R_Motor= Spd1;//float)temp4;
		odometry(Milemeter_R_Motor,Milemeter_L_Motor);//计算里程�??

		bSpeed_Buffer_Index++;//数组移位
		//缓存左右轮编码数到数组结束判�??
		if(bSpeed_Buffer_Index >=SPEED_BUFFER_SIZE)
		{
			bSpeed_Buffer_Index=0;//缓存左右轮编码数到数组变量清�??
		}

		ENC_Calc_Average_Speed();//计算三次电机的平均编码数

		pulseR = (uint16_t)Gain1(); //电机A转动 PID调节控制right
		htim2.Instance->CCR3 = pulseR;
//		tim1_tim = 0;
		pulseL = (uint16_t)Gain2(); //电机B转动 PID调节控制left
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pulseR); //PB7 ch1
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pulseL); //PB8 ch2
		htim2.Instance->CCR4 = pulseL;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//		tim1_tim = 0;
		leftrightpulse.pulse_d = pulseL<<16|pulseR;
	}
	if(htim->Instance == TIM5){
		if(calQuatFlg){
			if(mpu_mpl_get_data(&Pitch.imu_d,&Roll.imu_d,&Yaw.imu_d,&x_acc.acc_d,&y_acc.acc_d,&z_acc.acc_d,&x_gyro.gyro_d,&y_gyro.gyro_d,&z_gyro.gyro_d,&q0_inv.quat_d,&q1_inv.quat_d,&q2_inv.quat_d,&q3_inv.quat_d)==0)
			{
			  Yaw.imu_d = Yaw.imu_d+180;
			  YawOS.imu_d = Yaw.imu_d-yawoffset;
			  YawOS.imu_d = fmod(YawOS.imu_d+360,360);
			  Euler2Quat(Roll.imu_d,Pitch.imu_d,YawOS.imu_d);
//			  Euler2Quat_O(Roll.imu_d,Pitch.imu_d,Yaw.imu_d);
//			  Quaternion_Solution(x_gyro.gyro_d, y_gyro.gyro_d, z_gyro.gyro_d, x_acc.acc_d, y_acc.acc_d, z_acc.acc_d);

//			  Quat2Euler();
//			  DataScope();
	//		  printf("Pitch: %.2f,Roll: %.2f,Yaw: %.2f.\r\n",Pitch.imu_d,Roll.imu_d,Yaw.imu_d);
//					  printf("q0_o: %.2f,q1_o: %.2f,q2_o: %.2f,q3_o: %.2f.\r\n",q0_o.quat_d,q1_o.quat_d,q2_o.quat_d,q3_o.quat_d);
//					  printf("q0_: %.2f,q1_: %.2f,q2_: %.2f,q3_: %.2f.\r\n",q0_.quat_d,q1_.quat_d,q2_.quat_d,q3_.quat_d);
	//		  printf("Pitch: %.2f,Roll: %.2f,YawOS: %.2f.\r\n",Pitch.imu_d,Roll.imu_d,YawOS.imu_d);
//					  printf("q0: %.2f,q1: %.2f,q2: %.2f,q3: %.2f.\r\n",q0.quat_d,q1.quat_d,q2.quat_d,q3.quat_d);
	////
	//		  printf("q0_inv: %.2f,q1_inv: %.2f,q2_inv: %.2f,q3_inv: %.2f.\r\n",q0_inv.quat_d,q1_inv.quat_d,q2_inv.quat_d,q3_inv.quat_d);
			}
			else{
			  delay_ms(100);
			//		  printf("get data error.\r\n");
			}
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  keynumX = 0;
   spdref = 0;
 //  mainloop_tim = 0;
 //  tim1_tim = 0;
 //  usart1_tim = 0;

   pulseR = 0;
   pulseL = 0;

   SpdSum1 = 0;
   SpdSum2 = 0;
 //  oldSpdSum1 = -10;
 //  oldSpdSum2 = 0;
   Spd1 = 0.0;
   Spd2 = 0.0;
   dir1=0;
   dir2=0;
   dirEnc1=0;
   dirEnc2=0;
   uint8_t j=0;

   q0.quat_d=1.0f,q1.quat_d=0.0f,q2.quat_d=0.0f,q3.quat_d=0.0f;

   ENC_Clear_Speed_Buffer();

   /**************clear counter of encoder timer*/
 //  __HAL_TIM_SetCounter(&htim2,0);
 //  __HAL_TIM_SetCounter(&htim3,0);

 //  IIC_Init();                     //=====模拟IIC初始�??
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);//定时�??1�??
  HAL_TIM_Base_Start_IT(&htim5);//定时�??5�??
  ret1 = HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    /***********start up IT to cal direction in callback function************/
  //  ret2 = HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_ALL);//这里TIM_CHANNEL_1/TIM_CHANNEL_ALL均可
  //  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);//这里TIM_CHANNEL_1/TIM_CHANNEL_ALL均可
   /*********flash spi init**************/
    BSP_W25Qx_Init();
    BSP_W25Qx_Read_ID(ID);
    printf("W25Qxxx ID is : 0x%02X 0x%02X \r\n",ID[0],ID[1]);

   /************************************/
    delay_init(72);
    //disable LN298
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)USART_RX_BUF, sizeof(USART_RX_BUF));
  //  HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /*****MPU9250 init*******/
    if(!MPU9250_Init())             	//初始化MPU9250
  	  printf("mpu init is ok.\r\n");
    else
  	  printf("mpu9250 init failed.\r\n");
    if(!mpu_dmp_init()){
  	  printf("dmp int is ok.\r\n");
  	  delay_ms(200);
    }
    else
  	  printf("dmp int faied.\r\n");

    if(!BSP_W25Qx_Read(mpudateBuff,0x00,sizeof(mpudateBuff))){//读取mpu9250校准数据
		printf("read mpl states success.\r\n");
		printf("read states: %d,%d,%d,%d......\r\n",mpudateBuff[0],mpudateBuff[1],mpudateBuff[2],mpudateBuff[3]);
	}
    else{
    	printf("read mpl states fail.\r\n");
    }
	if(inv_load_mpl_states(mpudateBuff,sizeof(mpudateBuff))==INV_SUCCESS)
	{
		printf("=======inv_load_mpl_states ok====================\r\n");
	}
	else
	{
		printf("=======inv_load_mpl_states error====================\r\n");
	}

	//init euler angle
	uint8_t avgcnt = 0;
	do{
		calQuatFlg = 0;
		 if(mpu_mpl_get_data(&Pitch.imu_d,&Roll.imu_d,&Yaw.imu_d,&x_acc.acc_d,&y_acc.acc_d,&z_acc.acc_d,&x_gyro.gyro_d,&y_gyro.gyro_d,&z_gyro.gyro_d,&q0_inv.quat_d,&q1_inv.quat_d,&q2_inv.quat_d,&q3_inv.quat_d)==0)
		  {
			 yawavg[avgcnt] = Yaw.imu_d+180;
			 avgcnt++;
		  }
		  else{
			  delay_ms(100);
	//		  printf("get data error.\r\n");
		  }
	}while(avgcnt<=10);
	calQuatFlg = 1;
	yawoffset = (yawavg[5]+yawavg[6]+yawavg[7]+yawavg[8]+yawavg[9])/5;
    /***********************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//	  mainloop_tim = 1;
//	  Read_DMP();
	  delay_ms(20);
	  if(calflg == 1)
	  {
	/*********save mpl calibration params**********************************************/
		if(inv_get_mpl_state_size(&size)==0)
		{
			printf("=======inv_get_mpl_state_size ok====================\r\n");
			printf("mpl state size:%d\r\n",size);
			if(inv_save_mpl_states(mpudateBuff,sizeof(mpudateBuff))==0)
			{
				printf("=======inv_save_mpl_states ok====================\r\n");
				printf("write states: %d,%d,%d,%d......\r\n",mpudateBuff[0],mpudateBuff[1],mpudateBuff[2],mpudateBuff[3]);
				  if(BSP_W25Qx_Erase_Block(0) == W25Qx_OK)
					  printf(" SPI Erase Block ok\r\n");
				  else
					  printf(" SPI Erase Block fail.\r\n");
				if(!BSP_W25Qx_Write(mpudateBuff,0x00,sizeof(mpudateBuff)))
					printf("flash write success.\r\n");
				else
					printf("flash write fail.\r\n");

			}
			else
			{
				printf("=======inv_save_mpl_states error====================\r\n");
			}
			calflg =0;
		}
	  }
//	  if(mpu_mpl_get_data(&Pitch.imu_d,&Roll.imu_d,&Yaw.imu_d,&x_acc.acc_d,&y_acc.acc_d,&z_acc.acc_d,&x_gyro.gyro_d,&y_gyro.gyro_d,&z_gyro.gyro_d,&q0_inv.quat_d,&q1_inv.quat_d,&q2_inv.quat_d,&q3_inv.quat_d)==0)
//	  {
//		  Yaw.imu_d = Yaw.imu_d+180;
//		  YawOS.imu_d = Yaw.imu_d-yawoffset;
////		  YawOS.imu_d = YawOS.imu_d+180;
//		  Euler2Quat(Roll.imu_d,Pitch.imu_d,YawOS.imu_d);
//		  Euler2Quat_O(Roll.imu_d,Pitch.imu_d,Yaw.imu_d);
//
////		  Quat2Euler();
////		  DataScope();
////		  printf("Pitch: %.2f,Roll: %.2f,Yaw: %.2f.\r\n",Pitch.imu_d,Roll.imu_d,Yaw.imu_d);
////		  printf("q0_o: %.2f,q1_o: %.2f,q2_o: %.2f,q3_o: %.2f.\r\n",q0_o.quat_d,q1_o.quat_d,q2_o.quat_d,q3_o.quat_d);
////		  printf("Pitch: %.2f,Roll: %.2f,YawOS: %.2f.\r\n",Pitch.imu_d,Roll.imu_d,YawOS.imu_d);
////		  printf("q0: %.2f,q1: %.2f,q2: %.2f,q3: %.2f.\r\n",q0.quat_d,q1.quat_d,q2.quat_d,q3.quat_d);
//////
////		  printf("q0_inv: %.2f,q1_inv: %.2f,q2_inv: %.2f,q3_inv: %.2f.\r\n",q0_inv.quat_d,q1_inv.quat_d,q2_inv.quat_d,q3_inv.quat_d);
//	  }
//	  else{
//		  delay_ms(100);
////		  printf("get data error.\r\n");
//	  }

	  /*
	   * uart overrun error self recovery
	   * */
	  cr1its  = READ_REG(huart1.Instance->CR1);
	  if((cr1its & USART_CR1_RXNEIE) ==0)
		  HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  if(stateHandle&0x01)
	  {
		 x_data.odoemtry_float=position_x;//单位mm
		 y_data.odoemtry_float=position_y;//单位mm
		 theta_data.odoemtry_float=theta;//单位rad
		 vel_linear.odoemtry_float=linear_vel;//单位mm/s
		 vel_angular.odoemtry_float=angular_vel;//单位rad/s

	   //将所有里程计数据存到要发送的数组
		odometry_data[0] =0xAA;
		odometry_data[1] =0x55;
		odometry_data[2] =0x01;
		odometry_data[3] =0x4c; //

		for(j=0;j<4;j++)
		{
			odometry_data[j+4]=x_data.odometry_char[j];
			odometry_data[j+8]=y_data.odometry_char[j];
			odometry_data[j+12]=theta_data.odometry_char[j];
			odometry_data[j+16]=vel_linear.odometry_char[j];
			odometry_data[j+20]=vel_angular.odometry_char[j];
			odometry_data[j+24]=leftrightpulse.pulse_data[j];
			odometry_data[j+28]=Pitch.imu_data[j];
			odometry_data[j+32]=Roll.imu_data[j];
			odometry_data[j+36]=Yaw.imu_data[j];
			odometry_data[j+40]=q0.qua_data[j];
			odometry_data[j+44]=q1.qua_data[j];
			odometry_data[j+48]=q2.qua_data[j];
			odometry_data[j+52]=q3.qua_data[j];
			odometry_data[j+56]=x_acc.acc_data[j];
			odometry_data[j+60]=y_acc.acc_data[j];
			odometry_data[j+64]=z_acc.acc_data[j];
			odometry_data[j+68]=x_gyro.gyro_data[j];
			odometry_data[j+72]=y_gyro.gyro_data[j];
			odometry_data[j+76]=z_gyro.gyro_data[j];
		}
         sndCRCVal = CRC16(odometry_data,USART_SEN_LEN-4);
		 odometry_data[80] =sndCRCVal;
		 odometry_data[81] =sndCRCVal>>8;
		 odometry_data[82]=0x0d;//添加结束�??
		 odometry_data[83]=0x0a;//添加结束�??

		//发�?�至上位�??
//		HAL_UART_Transmit_IT(&huart1,odometry_data,sizeof(odometry_data));
		HAL_UART_Transmit(&huart1,odometry_data,sizeof(odometry_data),0xfff); //
		stateHandle&=0xFE;//state machine
	  }

	  car_control(rightdata.d,leftdata.d);	 //将接收到的左右轮速度赋给小车
//	  car_control(-30,30);	 //将接收到的左右轮速度赋给小车
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//	  mainloop_tim = 0;
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
