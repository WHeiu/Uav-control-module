#include "mmpu9250.h"
#include "mp9250.h"
#include "stm32l4xx_ll_utils.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "myiic.h" 
#include "delay.h"
#include "usart.h"
#include "string.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//MPU9250��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/7/19
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved	
//********************************************************************************
//�޸�˵��
//V1.1 20160902
//�޸�MPU6500_IDΪ: MPU6500_ID1,MPU6500_ID2,��Ϊ9205������2��ID!!!								  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��MPU9250
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU9250_Init(void)
{
    uint8_t res=0;
//  IIC_Init();     //��ʼ��IIC����     �ĺ�Ӧ�ó�ʼ��SPI1
//  MPU9250_Write_Reg(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
	  MPU9250_Write_Reg(MPU_PWR_MGMT1_REG,0X80);
    LL_mDelay(100);  //��ʱ100ms
    MPU9250_Write_Reg(MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
	  MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
    MPU_Set_Rate(50);						       	 	//���ò�����50Hz
  	MPU9250_Write_Reg(MPU_USER_CTRL_REG,0X00);  //I2C��ģʽ�ر�
	
//	  MPU9250_Write_Reg(MPU_USER_CTRL_REG,0X20);  //I2C��ģʽ����
//    MPU9250_Write_Reg(I2C_SLV0_ADDR,0x98);//MAG address 0x18��and Transfer is a read
//    MPU9250_Write_Reg(I2C_SLV0_CTRL,0x81);//Enable reading data from this slave at the sample rate && 1 Byte length
	
  	MPU9250_Write_Reg(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	
	/************  ���������� ����Ӧ����ֱͨģʽ�� Ӧ���� I2C Master Mode ģʽ  ****************/
    MPU9250_Write_Reg(MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������    
	
  	/**���޸�**/
	/*****************************************************/
//	  MPU9250_Write_Reg(MPU_I2CMST_STA_REG,0X82);//
//		MPU9250_Write_Reg(INT_PIN_CFG ,0x30);// INT Pin / Bypass Enable Configuration  
//	  MPU9250_Write_Reg(I2C_MST_CTRL,0x4d);//I2C MAster mode and Speed 400 kHz
//  	MPU9250_Write_Reg(USER_CTRL ,0x20); // I2C_MST_EN 
//	  MPU9250_Write_Reg(I2C_MST_DELAY_CTRL ,0x01);//��ʱʹ��I2C_SLV0 _DLY_ enable 	
//	  MPU9250_Write_Reg(I2C_SLV0_CTRL ,0x81); //enable IIC	and EXT_SENS_DATA==1 Byte
		
	
	/*********************************************/
	
    res=MPU9250_Read_Reg(MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) 				//����ID��ȷ
    {
			  printff(USART3, "Read MPU6500 ID OK!!!\r\n",strlen("Read MPU6500 ID OK!!!\r\n"));	   
        MPU9250_Write_Reg(MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        MPU9250_Write_Reg(MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
	    	MPU_Set_Rate(50);						       			//���ò�����Ϊ50Hz   
    }else return 1;
 
			/**********************Init MAG **********************************/
//	i2c_Mag_write(AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
//	i2c_Mag_write(AK8963_CNTL1_REG,0x12); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	

   res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
			  printff(USART3, "Read AK8963 ID OK!!!\r\n",strlen("Read AK8963 ID OK!!!\r\n"));	   
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//��λAK8963
	    	delay_ms(50);
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���
    }else return 1;

    return 0;
}

//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU9250_Write_Reg(MPU_GYRO_CFG_REG,(fsr<<3)|3);//���������������̷�Χ  
}
//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU9250_Write_Reg(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU9250_Write_Reg(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU9250_Write_Reg(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_LEN(MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_LEN(MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_LEN(MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//�õ�������ֵ(ԭʼֵ)
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
uint8_t MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    uint8_t buf[6],res;  
 	res=MPU_Read_LEN(MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((uint16_t)buf[1]<<8)|buf[0];  
		*my=((uint16_t)buf[3]<<8)|buf[2];  
		*mz=((uint16_t)buf[5]<<8)|buf[4];
	} 	 
	MPU9250_Write_Reg(MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
//uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
//{
//    uint8_t i;
//    IIC_Start();
//    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
//    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
//    {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//    for(i=0;i<len;i++)
//    {
//        IIC_Send_Byte(buf[i]);  //��������
//        if(IIC_Wait_Ack())      //�ȴ�ACK
//        {
//            IIC_Stop();
//            return 1;
//        }
//    }
//    IIC_Stop();
//    return 0;
//} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
//uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
//{ 
//    IIC_Start();
//    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
//    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
//    {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//	IIC_Start();                
//    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//    while(len)
//    {
//        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
//		else *buf=IIC_Read_Byte(1);		//������,����ACK  
//		len--;
//		buf++;  
//    }
//    IIC_Stop();                 //����һ��ֹͣ����
//    return 0;       
//}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
//uint8_t MPU9250_Write_Reg(uint8_t addr,uint8_t reg,uint8_t data)
//{
//   // IIC_Start();
//    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
//    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
//    {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//    IIC_Send_Byte(data);        //��������
//    if(IIC_Wait_Ack())          //�ȴ�ACK
//    {
//        IIC_Stop();
//        return 1;
//    }
//    IIC_Stop();
//    return 0;
//}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
//uint8_t MPU9250_Read_Reg(uint8_t addr,uint8_t reg)
//{
//    uint8_t res;
//    IIC_Start();
//    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//	IIC_Start();                
//    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
//    IIC_Wait_Ack();             //�ȴ�Ӧ��
//    res=IIC_Read_Byte(0);		//������,����nACK  
//    IIC_Stop();                 //����һ��ֹͣ����
//    return res;  
//}
