#include "stm32l4xx_hal.h"
#include "mp9250.h"
#include "usart.h"
#include <string.h>
#include <invmpuu.h>
MPU_value mpu_value;          //9轴数据
unsigned char BUF[6];       //接收数据缓存区

extern	char Pitch[100];
extern  char Roll[100];
extern	char Yaw[100];
extern  float pitch,roll,yaw; 	        		//欧拉角
extern SPI_HandleTypeDef hspi1;
//SPIx 
//TxData:发送一个字节
//返回值:data
/***************************************************************/
static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{	
  HAL_StatusTypeDef ret;
  uint8_t Rxdata;
  ret = HAL_SPI_TransmitReceive(&hspi1, &TxData,&Rxdata, 1, 100);

  return Rxdata;
}
#if 0
static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{		
	uint8_t retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //接收数据					    
}
#endif
/***************************************************************/
//SPI发送
//reg: addr
//value:数据
/***************************************************************/
uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value)
{
//	uint8_t status;
	MPU_9250_ENABLE;   //	MPU9250_CS=0;  //片选MPU9250
//	status=SPI1_ReadWriteByte(reg); //发送reg地址
	SPI1_ReadWriteByte(reg); //发送reg地址
	SPI1_ReadWriteByte(value);//发送数据
	MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return 0;//
}
uint8_t MPU9250_Write_LEN(uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i;
	MPU_9250_ENABLE;   //	MPU9250_CS=0;  //片选MPU9250
//	status=SPI1_ReadWriteByte(reg); //发送reg地址
	SPI1_ReadWriteByte(reg); //发送reg地址
	for(i = 0; i < len ; i++)
	SPI1_ReadWriteByte(buf[i]);//发送数据
	MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return 0;//
}

//---------------------------------------------------------------//
//SPI读取
//reg: addr
uint8_t MPU9250_Read_Reg(uint8_t reg)
{
	  uint8_t reg_val;
		MPU_9250_ENABLE;//	MPU9250_CS=0;  //片选MPU9250
	  SPI1_ReadWriteByte(reg|0x80); //reg地址+读命令
	  reg_val=SPI1_ReadWriteByte(0xff);//任意数据
		MPU_9250_DISENABLE;//	MPU9250_CS=1;  //失能MPU9250
	return(reg_val);
}
//spi方式，从寄存器reg中读一个长度为len的数据，存放在buf中
uint8_t MPU_Read_LEN(uint8_t reg,uint8_t len,uint8_t *buf)
{ 
    uint8_t i;
    MPU_9250_ENABLE;   
    SPI1_ReadWriteByte(reg|0x80); //发送读命令+寄存器号
    for(i=0;i<len;i++)  //一共读取len字节数据
    {
        buf[i]=SPI1_ReadWriteByte(0xff);   //循环读取
    }
    MPU_9250_DISENABLE;
    return 0;   
}
/***************************************************************/
// MPU内部i2c 写入
//I2C_SLVx_ADDR:  MPU9250_AK8963_ADDR
//I2C_SLVx_REG:   reg
//I2C_SLVx_Data out:  value
/***************************************************************/
 void i2c_Mag_write(uint8_t reg,uint8_t value)
{
	uint16_t j=500;
	MPU9250_Write_Reg(I2C_SLV0_ADDR ,MPU9250_AK8963_ADDR);//设置磁力计地址,mode: write
	MPU9250_Write_Reg(I2C_SLV0_REG ,reg);//set reg addr
	MPU9250_Write_Reg(I2C_SLV0_DO ,value);//send value	
	while(j--);//此处因为MPU内部I2C读取速度较慢，延时等待内部写完毕
}
/***************************************************************/
// MPU内部i2c 读取
//I2C_SLVx_ADDR:  MPU9250_AK8963_ADDR
//I2C_SLVx_REG:   reg
//return value:   EXT_SENS_DATA_00 register value
/***************************************************************/
static uint8_t i2c_Mag_read(uint8_t reg)
{
	uint16_t j=5000;
	MPU9250_Write_Reg(I2C_SLV0_ADDR ,MPU9250_AK8963_ADDR|0x80); //设置磁力计地址，mode：read
	MPU9250_Write_Reg(I2C_SLV0_REG ,reg);// set reg addr
	MPU9250_Write_Reg(I2C_SLV0_DO ,0xff);//read
	while(j--);//此处因为MPU内部I2C读取速度较慢，必须延时等待内部读取完毕
	return MPU9250_Read_Reg(EXT_SENS_DATA_00);
}

//****************初始化MPU9250，根据需要请参考pdf进行修改************************
void Init_MPU9250(void)
{	
	uint8_t WhoAmI=0;
	MPU9250_Write_Reg(PWR_MGMT_1, 0x00);	//解除休眠状态
	MPU9250_Write_Reg(CONFIG, 0x07);      //低通滤波频率，典型值：0x07(3600Hz)此寄存器内决定Internal_Sample_Rate==8K
	
/**********************Init SLV0 i2c**********************************/	
//Use SPI-bus read slave0
	MPU9250_Write_Reg(INT_PIN_CFG ,0x30);// INT Pin / Bypass Enable Configuration  
	MPU9250_Write_Reg(I2C_MST_CTRL,0x4d);//I2C MAster mode and Speed 400 kHz
	MPU9250_Write_Reg(USER_CTRL ,0x20); // I2C_MST_EN 
	MPU9250_Write_Reg(I2C_MST_DELAY_CTRL ,0x01);//延时使能I2C_SLV0 _DLY_ enable 	
	MPU9250_Write_Reg(I2C_SLV0_CTRL ,0x81); //enable IIC	and EXT_SENS_DATA==1 Byte
	
/*******************Init GYRO and ACCEL******************************/	
	MPU9250_Write_Reg(SMPLRT_DIV, 0x07);  //陀螺仪采样率，典型值：0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	MPU9250_Write_Reg(GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	MPU9250_Write_Reg(ACCEL_CONFIG, 0x18);//加速计自检、测量范围及高通滤波频率，典型值：0x18(不自检，16G)
	MPU9250_Write_Reg(ACCEL_CONFIG_2, 0x08);//加速计高通滤波频率 典型值 ：0x08  （1.13kHz）	
		
/**********************Init MAG **********************************/
	i2c_Mag_write(AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
	i2c_Mag_write(AK8963_CNTL1_REG,0x12); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output	


	//验证初始化完成
	WhoAmI=MPU9250_Read_Reg(WHO_AM_I); 
	//printf("WhoAmI:0x%X  \r\n",WhoAmI);

}

//************************加速度读取**************************/
void READ_MPU9250_ACCEL(void)//
{ 

   BUF[0]=MPU9250_Read_Reg(ACCEL_XOUT_L); 
   BUF[1]=MPU9250_Read_Reg(ACCEL_XOUT_H);
   mpu_value.Accel[0]=	(BUF[1]<<8)|BUF[0];
   mpu_value.Accel[0]/=164; 						   //读取计算X轴数据
   BUF[2]=MPU9250_Read_Reg(ACCEL_YOUT_L);
   BUF[3]=MPU9250_Read_Reg(ACCEL_YOUT_H);
   mpu_value.Accel[1]=	(BUF[3]<<8)|BUF[2];
   mpu_value.Accel[1]/=164; 						   //读取计算Y轴数据
   BUF[4]=MPU9250_Read_Reg(ACCEL_ZOUT_L); 
   BUF[5]=MPU9250_Read_Reg(ACCEL_ZOUT_H);
   mpu_value.Accel[2]=  (BUF[5]<<8)|BUF[4];
   mpu_value.Accel[2]/=164; 					      //读取计算Z轴数据 
}
/**********************陀螺仪读取*****************************/
void READ_MPU9250_GYRO(void)
{ 

   BUF[0]=MPU9250_Read_Reg(GYRO_XOUT_L); 
   BUF[1]=MPU9250_Read_Reg(GYRO_XOUT_H);
   mpu_value.Gyro[0]=	(BUF[1]<<8)|BUF[0];
   mpu_value.Gyro[0]/=16.4; 						   //读取计算X轴数据

   BUF[2]=MPU9250_Read_Reg(GYRO_YOUT_L);
   BUF[3]=MPU9250_Read_Reg(GYRO_YOUT_H);
   mpu_value.Gyro[1]=	(BUF[3]<<8)|BUF[2];
   mpu_value.Gyro[1]/=16.4; 						   //读取计算Y轴数据
   BUF[4]=MPU9250_Read_Reg(GYRO_ZOUT_L);
   BUF[5]=MPU9250_Read_Reg(GYRO_ZOUT_H);
   mpu_value.Gyro[2]=	(BUF[5]<<8)|BUF[4];
   mpu_value.Gyro[2]/=16.4; 					       //读取计算Z轴数据
}


/**********************磁力计读取***************************/
//i2c_Mag_read(AK8963_ST2_REG) 此步读取不可省略
//数据读取结束寄存器，reading this register means data reading end
//AK8963_ST2_REG 同时具有数据非正常溢出检测功能
//详情参考 MPU9250 PDF
/**********************************************************/
void READ_MPU9250_MAG(void)
{ 	
	uint8_t x_axis,y_axis,z_axis; 
	
	x_axis=i2c_Mag_read(AK8963_ASAX);// X轴灵敏度调整值
	y_axis=i2c_Mag_read(AK8963_ASAY);
	z_axis=i2c_Mag_read(AK8963_ASAZ);
	
	if((i2c_Mag_read(AK8963_ST1_REG)&AK8963_ST1_DOR)==0)//data ready
	{
			//读取计算X轴数据
		 BUF[0]=i2c_Mag_read(MAG_XOUT_L); //Low data	
		 if((i2c_Mag_read(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register & check Magnetic sensor overflow occurred 
		 {
			 BUF[0]=i2c_Mag_read(MAG_XOUT_L);//reload data
		 } 
		 BUF[1]=i2c_Mag_read(MAG_XOUT_H); //High data	
		 if((i2c_Mag_read(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
		 {
			 BUF[1]=i2c_Mag_read(MAG_XOUT_H);
		 }
		 mpu_value.Mag[0]=((BUF[1]<<8)|BUF[0])*(((x_axis-128)>>8)+1);		//灵敏度纠正 公式见/RM-MPU-9250A-00 PDF/ 5.13	
		 
		//读取计算Y轴数据
			BUF[2]=i2c_Mag_read(MAG_YOUT_L); //Low data	
		 if((i2c_Mag_read(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
		 {
			 BUF[2]=i2c_Mag_read(MAG_YOUT_L);
		 }		 
		 BUF[3]=i2c_Mag_read(MAG_YOUT_H); //High data	
		 if((i2c_Mag_read(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
		 {
			 BUF[3]=i2c_Mag_read(MAG_YOUT_H);
		 }
		  mpu_value.Mag[1]=((BUF[3]<<8)|BUF[2])*(((y_axis-128)>>8)+1);	
		 
		//读取计算Z轴数据
		 BUF[4]=i2c_Mag_read(MAG_ZOUT_L); //Low data	
		 if((i2c_Mag_read(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
		 {
			 BUF[4]=i2c_Mag_read(MAG_ZOUT_L);
		 }	 
		 BUF[5]=i2c_Mag_read(MAG_ZOUT_H); //High data	
		 if((i2c_Mag_read(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
		 {
			 BUF[5]=i2c_Mag_read(MAG_ZOUT_H);
		 }
		  mpu_value.Mag[2]=((BUF[5]<<8)|BUF[4])*(((z_axis-128)>>8)+1);	
	}					       
}
void MPU9250_Get_Data(void)
{
	
	        printff(USART3,"\r\n",strlen("\r\n"));
          printff(USART3,"\r\n",strlen("\r\n"));
        	printff(USART3, "-----------------------------------\r\n", strlen("-----------------------------------\r\n"));
	        printff(USART3,"**************MPU9250**************\r\n",strlen("**************MPU9250**************\r\n"));
	        while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0 );   //这里是while()空循环，等待读取数据
          //while(mpu_mpl_get_data(&pitch,&roll,&yaw)!=0 );   //这里是while()空循环，等待读取数据
     					
					sprintf( Pitch, "%f", pitch ); 
					sprintf( Roll, "%f", roll ); 
					sprintf( Yaw, "%f", yaw );
					
					printff(USART3, "Pitch:\r\n",strlen("Pitch:\r\n"));
					printff(USART3, Pitch,strlen(Pitch));
					printff(USART3, "\r\n\r\nRoll:\r\n",strlen("\r\n\r\nRoll:\r\n"));
					printff(USART3, Roll,strlen(Roll));
					printff(USART3, "\r\n\r\nYaw:\r\n",strlen("\r\n\r\nYaw:\r\n"));
					printff(USART3, Yaw,strlen(Yaw));
					printff(USART3, "\r\n", strlen("\r\n"));
		
					printff(USART3, "-----------------------------------\r\n\r\n", strlen("-----------------------------------\r\n\r\n"));
}


