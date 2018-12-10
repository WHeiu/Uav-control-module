#ifndef _SPI_H_
#define _SPI_H_
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "main.h"


#define  uint8_t unsigned char
#define  uint16_t unsigned short
#define  u32 unsigned int
// ����MPU9250�ڲ���ַ
/*****************************************************************/
#define	SMPLRT_DIV		                      0x19	//�����ǲ�����
#define	CONFIG			                        0x1A	
#define	GYRO_CONFIG		                      0x1B	
#define	ACCEL_CONFIG	                      0x1C	
#define	ACCEL_CONFIG_2                      0x1D 

#define INT_PIN_CFG                         0x37 //�ж�����
#define USER_CTRL                           0x6a
#define I2C_MST_CTRL                        0x24
#define I2C_MST_DELAY_CTRL                  0x67
//--------------------i2c slv0-------------------------------//
#define I2C_SLV0_ADDR                       0x25  
#define I2C_SLV0_REG                        0x26
#define I2C_SLV0_CTRL                       0x27 
#define I2C_SLV0_DO                         0x63 //output reg
//--------------------AK8963 reg addr------------------------//
#define MPU9250_AK8963_ADDR                 0x0C  //AKM addr
#define AK8963_WHOAMI_REG                   0x00  //AKM ID addr
#define AK8963_WHOAMI_ID                    0x48  //ID
#define AK8963_ST1_REG                      0x02  //Data Status1
#define AK8963_ST2_REG                      0x09  //Data reading end register & check Magnetic sensor overflow occurred
#define AK8963_ST1_DOR                      0x02
#define AK8963_ST1_DRDY                     0x01 //Data Ready
#define AK8963_ST2_BITM                     0x10
#define AK8963_ST2_HOFL                     0x08 // Magnetic sensor overflow 
#define AK8963_CNTL1_REG                    0x0A
#define AK8963_CNTL2_REG                    0x0B
#define AK8963_CNTL2_SRST                   0x01 //soft Reset
#define AK8963_ASAX                         0x10 //X-axis sensitivity adjustment value 
#define AK8963_ASAY                         0x11 //Y-axis sensitivity adjustment value
#define AK8963_ASAZ                         0x12 //Z-axis sensitivity adjustment value
//--------------------9axis  reg addr-----------------------//
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41   //temperture
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
		
#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08
//--------------------other reg addr-----------------------//
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	//ID��ַ�Ĵ���(��ȷ��ֵ0x71��ֻ��)

#define EXT_SENS_DATA_00    0x49  //MPU9250 IIC���������ȡ���ؼĴ���00
#define EXT_SENS_DATA_01    0x4a  //MPU9250 IIC���������ȡ���ؼĴ���01
#define EXT_SENS_DATA_02    0x4b  //MPU9250 IIC���������ȡ���ؼĴ���02
#define EXT_SENS_DATA_03    0x4c  //MPU9250 IIC���������ȡ���ؼĴ���03

/************************SPI CS ********************************/
#define MPU_9250_DISENABLE  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);//Ƭѡ
#define MPU_9250_ENABLE  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

typedef struct{
	short Accel[3];//Accel X,Y,Z
	short Gyro[3];//Gyro X,Y,Z
	short Mag[3];	//Mag X,Y,Z	
}MPU_value;

void Init_MPU9250(void);
uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value);//SPIд
uint8_t MPU9250_Read_Reg(uint8_t reg);//SPI��
uint8_t SPI1_ReadWriteByte(uint8_t TxData);
void READ_MPU9250_ACCEL(void);//��ȡ���ٶ�
void READ_MPU9250_GYRO(void);//��ȡ������
void READ_MPU9250_MAG(void);//��ȡ�شż�
void MPU9250_Get_Data(void);
void i2c_Mag_write(uint8_t reg,uint8_t value);
uint8_t MPU9250_Write_LEN(uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_LEN(uint8_t reg,uint8_t len,uint8_t *buf);
#endif
