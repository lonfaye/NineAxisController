#include "MPU9250.h"
#include "ioi2c.h"

/************************************************************************/
/*********                     MPU6050.C                       *************/
/**********          Written By LYM---20160916              *************/
/**********                  Version 1.0			      ***************/
/************************************************************************/

//-----------------------------------------------------------------------
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//-----------------------------------------------------------------------
//----------------------���ݱ�������-------------------------------------
//-----------------------------------------------------------------------
//=======================================================================
//-------------------------------------------------------------------------MPU9250��ʼ������
MPU9250_STATUS MPU9250_Init(void)
{
	u8 get_ID=0;
	MPU9250_Write_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x80);//��λMPU9250
	delay_ms(100);
	MPU9250_Write_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x00);//����MPU9250
	MPU9250_Set_Gyro(3);				//�����Ǵ�����,��250dps;1,��500dps;2,��1000dps;3,��2000dps
	MPU9250_Set_Accel(0);				//���ٶȴ�����,��2g->0 ��4->1 ��8->2
	MPU9250_Set_Rate(50);			//���ò���Ƶ��
	MPU9250_Write_Byte(MPU_ADDR,MPU_INT_EN_REG,0x00);		//�ر������ж�
	MPU9250_Write_Byte(MPU_ADDR,MPU_USER_CTRL_REG,0x00);//I2C��ģʽ�رգ�HCM588L������������
	MPU9250_Write_Byte(MPU_ADDR,MPU_FIFO_EN_REG,0x00);	//�ر�FIFO
	MPU9250_Write_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0x82);//INT���ŵ͵�ƽ��Ч //0x80
	delay_ms(3);
	get_ID=MPU9250_Read_Byte(MPU_ADDR,MPU_DEVICE_ID_REG);
	#ifdef MPU9250_DUBUG
		printf("MPU9250-ID = 0x%02x\n",id);
	#endif
	if(get_ID==0x71)
	{
		MPU9250_Write_Byte(MPU_ADDR,MPU_PWR_MGMT1_REG,0x01);		//����CLKSEL,PLL X��Ϊ�ο�
		MPU9250_Write_Byte(MPU_ADDR,MPU_PWR_MGMT2_REG,0x00);		//���ٶ��������Ƕ�����
		MPU9250_Set_Rate(50);											//���ò���Ƶ��
//		return MPU9250_OK;
	}
	else
		return MPU9250_FAIL;

	get_ID= MPU9250_Read_Byte(MAG_ADDRESS,AKM8963_DEVICE_ID_REG);
	if(get_ID==0x48){
//		return MPU9250_OK;
//		MPU9250_Write_Byte(MAG_ADDRESS,AKM8963_CNTL2_REG,0x01);
		MPU9250_Write_Byte(MAG_ADDRESS,AKM8963_CNTL1_REG,0x11);
	}
	else
		return MPU9250_FAIL;
	return MPU9250_OK;
}

//-------------------------------------------------------------------------MPU9250 ��ȡAKM8963����ID
MPU9250_STATUS MPU9250_Status_Read_Akm8963_ID(u16 *id)//����ֵ: 0���ɹ�  1��ʧ��
{
	MPU9250_Write_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0x02);
	delay_ms(10);	
	MPU9250_Write_Byte(MAG_ADDRESS,AKM8963_CNTL1_REG,0x01);
	delay_ms(10);
  *id=MPU9250_Read_Byte(MAG_ADDRESS,AKM8963_DEVICE_ID_REG);
	#ifdef MPU9250_DUBUG
		printf("AKM8963-ID = 0x%02x\n",*id);
	#endif
	if(*id  == 0x48)
	{
		return  MPU9250_OK;
	}
	else
	{
		return  MPU9250_FAIL;
	}	
}
//=========================================================================================
//*******************************MPU9250��������********************************************
//=========================================================================================
//-------------------------------------------------------------------------MPU9250 �¶Ⱥ���
MPU9250_STATUS MPU9250_Get_Temperature(float *temp)
{
	u8 buf[2];
	short raw;
	MPU9250_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
	raw=(buf[0]<<8)|buf[1];
	//temp=36.53+((double)raw)/340;
	*temp=21+((double)raw)/333.87;
	return MPU9250_OK;
}
//-------------------------------------------------------------------------MPU9250 �����Ǻ���
MPU9250_STATUS MPU9250_Get_Gyroscope(short *gx,short *gy,short *gz)
{
	u8 buf[6];
	if(MPU9250_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf))
	{
		return MPU9250_FAIL;
	}
	else
	{
		*gx=((u16)buf[0]<<8)|buf[1];
		*gy=((u16)buf[2]<<8)|buf[3];
		*gz=((u16)buf[4]<<8)|buf[5];
	}
	return MPU9250_OK;
}
//-------------------------------------------------------------------------MPU9250 ���ٶȼƺ���
MPU9250_STATUS MPU9250_Get_Accelerometer(short *ax,short *ay,short *az)
{
	u8  buf[6];
	if(MPU9250_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf))
	{
		return MPU9250_FAIL;
	}
	else
	{
		*ax=((u16)buf[0]<<8)|buf[1];
		*ay=((u16)buf[2]<<8)|buf[3];
		*az=((u16)buf[4]<<8)|buf[5];
	}
	return MPU9250_OK;
}
//-------------------------------------------------------------------------MPU9250 �����ƺ���
MPU9250_STATUS MPU9250_Get_Mag(short *mx,short *my,short *mz)
{
	u8 buf[6];
	MPU9250_Write_Byte(MPU_ADDR,MPU_INTBP_CFG_REG,0x02);
	delay_ms(10);
	MPU9250_Write_Byte(MAG_ADDRESS,AKM8963_CNTL1_REG,0x01);
	delay_ms(10);
	if(MPU9250_Read_Len(MAG_ADDRESS,AKM8963_MAG_XOUTL_REG,6,buf))
	{
		return MPU9250_FAIL;
	}
	else
	{
		*mx=((u16)buf[1]<<8)|buf[0];
		*my=((u16)buf[3]<<8)|buf[2];
		*mz=((u16)buf[5]<<8)|buf[4];
	}
	return MPU9250_OK;
}

//=========================================================================================
//*******************************MPU6050����***********************************************
//=========================================================================================
//-------------------------------------------------------------------------MPU9250 �����Ƿ�Χ���ú���
MPU9250_STATUS MPU9250_Set_Gyro(u8 fsr)	//0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
{
	return MPU9250_Write_Byte(MPU_ADDR,MPU_GYRO_CFG_REG,fsr<<3); //����д���� ����0�ɹ�
}
//-------------------------------------------------------------------------MPU9250 ���ٶȼƷ�Χ���ú���
MPU9250_STATUS MPU9250_Set_Accel(u8 fsr) //0,��2g;1,��4g;2,��8g;3,��16g
{
	return MPU9250_Write_Byte(MPU_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);
}
//-------------------------------------------------------------------------MPU9250 �������ֵ�ͨ�˲���
MPU9250_STATUS MPU9250_Set_LPF(u8 lpf)
{
	u8 data=0;
	if(lpf>184)			data=1;
	else if(lpf>92) data=2;
	else if(lpf>41) data=3;
	else if(lpf>20) data=4;
	else if(lpf>10) data=5;
	else 						data=6;
	return MPU9250_Write_Byte(MPU_ADDR,MPU_CFG_REG,data);
}
//-------------------------------------------------------------------------MPU9250 ����Ƶ�����ú���
MPU9250_STATUS MPU9250_Set_Rate(u16 rate)
{
	u8 data=0;
	if(rate>1000) rate=1000;
	if(rate<4) 		rate=4;
	data=1000/rate-1;
	data=MPU9250_Write_Byte(MPU_ADDR,MPU_SAMPLE_RATE_REG,data);
	return MPU9250_Set_LPF(rate/2);      //**�Զ��������ֵ�ͨ�˲�Ϊ����Ƶ�ʵ�һ��
}
//=========================================================================================
//*******************************MPU9250 IIC����*******************************************
//=========================================================================================
//-------------------------------------------------------------------------MPU9250����������
// addr MPU9250��ַ
// reg  MPU9250�Ĵ�����ַ
// len  ��ȡ����
// *buf ���ݴ洢��
MPU9250_STATUS MPU9250_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
//	II2C_Start();
//	II2C_Send_Byte(addr);
//	if(II2C_Wait_Ack())
//	{
//		II2C_Stop();
//		return MPU9250_FAIL;
//	}
//	II2C_Send_Byte(reg);
//	II2C_Wait_Ack();
//	II2C_Start();
//	II2C_Send_Byte(addr+1);
//	II2C_Wait_Ack();
//	while(len)
//	{
//		if(len == 1)//len=1��ʾֻ��ȡһ���Ĵ���
//		{
//			*buf = II2C_Read_Byte(0);//��ȡһ���ֽڲ��ط�NACK
//		}
//		else
//		{
//			*buf = II2C_Read_Byte(1);//��ȡһ���ֽڲ��ط�ACK
//		}
//		buf++;
//		len--;
//	}
//	II2C_Stop();
//	return MPU9250_OK;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);
	if(!IIC_Wait_Ack())
	{
		IIC_Stop();
		return MPU9250_FAIL;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte((addr<<1)|1);
	IIC_Wait_Ack();
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);
		else *buf=IIC_Read_Byte(1);
		len--;
		buf++;
	}
	IIC_Stop();
	return MPU9250_OK;

//	if(i2cRead(addr, reg, len,  buf))
//		return MPU9250_FAIL;
//	else
//		return MPU9250_OK;
}

//-------------------------------------------------------------------------MPU9250����д����
MPU9250_STATUS MPU9250_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
//	u8 i=0;
//	II2C_Start();
//	II2C_Send_Byte(addr);
//	if(II2C_Wait_Ack())
//	{
//		II2C_Stop();
//		return MPU9250_FAIL;
//	}
//	II2C_Send_Byte(reg);
//	II2C_Wait_Ack();
//	for(i=0;i<len;i++)
//	{
//		II2C_Send_Byte(buf[i]);
//		if(II2C_Wait_Ack())
//		{
//			II2C_Stop();
//			return MPU9250_FAIL;
//		}
//	}
//	II2C_Stop();
//	return MPU9250_OK;

	 u8 i;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);
	if(!IIC_Wait_Ack())
	{
		IIC_Stop();
		return MPU9250_FAIL;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);
		if(!IIC_Wait_Ack())
		{
			IIC_Stop();
			return MPU9250_FAIL;
		}
	}
	IIC_Stop();
	return MPU9250_OK;

//	if(i2cWrite(addr,reg,len,buf))
//		return MPU9250_FAIL;
//	else
//		return MPU9250_OK;
}

//==========================================================================================
//-------------------------------------------------------------------------MPU9250��һ�ֽں���
u8 MPU9250_Read_Byte(u8 addr,u8 reg)
{
//	u8 rcv;
//	II2C_Start();
//	II2C_Send_Byte(MPU_ADDR);
//	II2C_Wait_Ack();
//	II2C_Send_Byte(reg);
//	II2C_Wait_Ack();
//	II2C_Start();
//	II2C_Send_Byte(MPU_ADDR+1);
//	II2C_Wait_Ack();
//	Return=II2C_Read_Byte(0);//��ȡ����
//	II2C_Stop();
//	return Return;

	u8 res;
	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte((addr<<1)|1);
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);
	IIC_Stop();
	return res;

//	if(i2cRead(MPU_ADDR, reg, 1,  &rcv))
//		return MPU9250_FAIL;
//	else
//		return rcv;
}
//-------------------------------------------------------------------------MPU6050дһ�ֽں���
MPU9250_STATUS MPU9250_Write_Byte(u8 addr,u8 reg,u8 data)
{
//	II2C_Start();
//	II2C_Send_Byte(addr);
//	if(II2C_Wait_Ack())
//	{
//		II2C_Stop();
//		return MPU9250_FAIL;
//	}
//	II2C_Send_Byte(reg);
//	II2C_Wait_Ack();
//	II2C_Send_Byte(data); //��������
//	if(II2C_Wait_Ack())
//	{
//		II2C_Stop();
//		return MPU9250_FAIL;
//	}
//	II2C_Stop();
//	return MPU9250_OK;

	IIC_Start();
	IIC_Send_Byte((addr<<1)|0);
	if(!IIC_Wait_Ack())
	{
		IIC_Stop();
		return MPU9250_FAIL;
	}
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	if(!IIC_Wait_Ack())
	{
		IIC_Stop();
		return MPU9250_FAIL;
	}
	IIC_Stop();
	return MPU9250_OK;

//	if(i2cWrite(addr,reg,1,&data))
//		return MPU9250_FAIL;
//	else
//		return MPU9250_OK;
}

//========================================================================================= MPU9250 ��λ������
void usart1_send_char(u8 c)
{
	while((USART1->SR&0X40)==0);//�ȴ���һ�η������
	USART1->DR=c;   	
} 
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ�����
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1
}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short mx,short my,short mz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=(mx>>8)&0XFF;
	tbuf[13]=mx&0XFF;
	tbuf[14]=(my>>8)&0XFF;
	tbuf[15]=my&0XFF;
	tbuf[16]=(mz>>8)&0XFF;
	tbuf[17]=mz&0XFF;		
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
} 





