/************************************************************************/
/*********                      MPU6050.H                       *************/
/**********          Written By LYM---20160916              *************/
/**********                  Version 1.0			      ***************/
/************************************************************************/
#ifndef __MPU9250_H__
#define __MPU9250_H__
//#include "sys.h"
#include "ioi2c.h"
#include "delay.h"
#include <stdio.h>
#include <math.h>
//#include "usart.h"
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------�궨��-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================
//ע�ʹ˺�ɹرմ�ӡ��Ϣ
//#define MPU9250_DUBUG 1

#define MPU_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z


#define MPU_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG				0X1A	//���üĴ���
#define MPU_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG		0X41	//�¶�ֵ��8λ�Ĵ���
#define MPU_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2
#define MPU_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		0X75	//����ID�Ĵ���0x71

#define MPU_ADDR				0x68 //0xD0	//ADO��GND�����λΪ0
#define	GYRO_ADDRESS   			0x68 //0xD0	//�����ǵ�ַ
#define ACCEL_ADDRESS  			0x68 //0xD0   	//���ٶȼƵ�ַ

#define MAG_ADDRESS   			0x0c //0x18   	//�����Ƶ�ַ
#define AKM8963_DEVICE_ID_REG	0x00	//AKM8963����ID�Ĵ���  0x48
#define AKM8963_INFO_REG		0x01	//AKM8963������Ϣ�Ĵ���
#define AKM8963_STATUS1_REG		0x02	//AKM8963״̬�Ĵ���1

#define AKM8963_MAG_XOUTL_REG	0x03	//AKM8963������ֵ,X���8λ�Ĵ���
#define AKM8963_MAG_XOUTH_REG	0x04	//AKM8963������ֵ,X���8λ�Ĵ���
#define AKM8963_MAG_YOUTL_REG	0x05	//AKM8963������ֵ,Y���8λ�Ĵ���
#define AKM8963_MAG_YOUTH_REG	0x06	//AKM8963������ֵ,Y���8λ�Ĵ���
#define AKM8963_MAG_ZOUTL_REG	0x07	//AKM8963������ֵ,Z���8λ�Ĵ���
#define AKM8963_MAG_ZOUTH_REG	0x08	//AKM8963������ֵ,Z���8λ�Ĵ���

#define AKM8963_STATUS2_REG		0x09	//AKM8963״̬�Ĵ���2
#define	AKM8963_CNTL1_REG		0x0A	//AKM8963���ƼĴ���1
#define	AKM8963_CNTL2_REG		0x0B	//AKM8963���ƼĴ���2
#define AKM8963_SELF_TEST_REG	0x0C	//AKM8963�Լ���ƼĴ���
#define AKM8963_TEST1_REG		0x0D	//AKM8963���ԼĴ���1
#define AKM8963_TEST2_REG		0x0E	//AKM8963���ԼĴ���2
#define AKM8963_I2C_DISABLE_REG	0x0F	//AKM8963-I2Cʧ�ܼĴ���
#define AKM8963_ASAX_REG		0x10	//AKM8963������X��������У���Ĵ���
#define AKM8963_ASAY_REG		0x11	//AKM8963������Y��������У���Ĵ���
#define AKM8963_ASAZ_REG		0x12	//AKM8963������Z��������У���Ĵ���

enum _MPU9250_STATUS
{
	MPU9250_OK = 0,
	MPU9250_FAIL = 1,
};

typedef enum _MPU9250_STATUS MPU9250_STATUS;
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------�ⲿ�ӿں���-------------------------------------
//=======================================================================
MPU9250_STATUS MPU9250_Init(void);
MPU9250_STATUS MPU9250_Get_Temperature(float *temp);
MPU9250_STATUS MPU9250_Get_Gyroscope(short *gx,short *gy,short *gz);
MPU9250_STATUS MPU9250_Get_Accelerometer(short *ax,short *ay,short *az);
MPU9250_STATUS MPU9250_Get_Mag(short *mx,short *my,short *mz);

void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short mx,short my,short mz,short roll,short pitch,short yaw);
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------�ڲ����ú���-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================
u8 MPU9250_Read_Byte(u8 addr,u8 reg);													//��һ���ֽ�
MPU9250_STATUS MPU9250_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);		//����������
MPU9250_STATUS MPU9250_Write_Byte(u8 addr,u8 reg,u8 data);				//дһ���ֽ�
MPU9250_STATUS MPU9250_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);	//����д����
//-------------------------------------------------------------------------MPU6050 �����Ƿ�Χ���ú���
MPU9250_STATUS MPU9250_Set_Gyro(u8 fsr);	//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//-------------------------------------------------------------------------MPU6050 ���ٶȼƷ�Χ���ú���
MPU9250_STATUS MPU9250_Set_Accel(u8 fsr); //fsr:0,��2g;1,��4g;2,��8g;3,��16g
//-------------------------------------------------------------------------MPU6050 ����Ƶ�����ú���
MPU9250_STATUS MPU9250_Set_Rate(u16 rate);
#endif
//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------����Ӧ��ʵ��-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================

//=======================================================================
//--------------------------End of MPU6050.H---------------------------------
//=======================================================================






