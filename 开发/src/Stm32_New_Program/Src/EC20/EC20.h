#ifndef __EC20_H
#define __EC20_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "usart.h"
#include "user.h"


//#define EC20_PWRKEY_UP   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
//#define EC20_PWRKEY_DOWN HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)

//#define EC20_RST_UP      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
//#define EC20_RST_DOWN    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)

//#define EC20_DIS_UP      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
//#define EC20_DIS_DOWN    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)

//#define EC20_RDY_UP      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
//#define EC20_RDY_DOWN    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

//#define EC20_NET_MODE_UP      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
//#define EC20_NET_MODE_DOWN    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

//#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
extern __IO ITStatus UartReady;
#define EC20_Sendbyte(x)       HAL_UART_Transmit(&huart4,&x,1,10)//USART3_SendByte                  //几个重要接口的定义
//#define EC20_USART_IRQHandler //USART3_IRQHandler
//#define EC20_Receivebyte      //USART_ReceiveData(USART3)
//#define EC20_USART            //USART3

typedef struct    
{
		unsigned char status;//?????
		unsigned char IMEI[20];//IMEI??
		unsigned char ipstatus;
		unsigned char tcpstaus;
		int sendcount;
		unsigned char datastatus;
		unsigned char ackstatus;
		unsigned char getcount;
		unsigned char getok;
		unsigned char tcpcount;
		unsigned char tcpflag;
		unsigned char enable;
		unsigned char dataunm;
}GSM_init;

typedef struct    
{

		unsigned char Realtime[25];//????
		unsigned char Qcell[25];//?????
		unsigned char GPS[30];//?????
		unsigned char sendplus[100];
		unsigned char senddata[100];
		unsigned char sendackdata[100];	
		unsigned char sendgpsdata[100];	
		unsigned char count;
		unsigned char starnum[5];//卫星数量
		unsigned char speed[10];//速度值
		unsigned char getnum;
		unsigned char hardfalut;
}GPS_DATA;
typedef struct    
{
		char Current_Card_IP[20];   //当前卡IP
		char IMEI[30];   //模块IMEI码
		char Module_version_number[100];   //模块版本号
		char Operators[30];    //营运商
		char sendcount;
		char datastatus;
		char ackstatus;
		char getcount;
		char getok;
		char tcpcount;
		char tcpflag;
		char enable;
		char dataunm;
}GSM_Status;

void Get_IMEI(void);
void  EC20_GPS_Init(void);
void EC20Send_StrData(char *bufferdata);
void EC20Send_RecAccessMode(void);
void EC20_GetGps_Send(void);     //EC20获取GPS信息并发送给服务器
void EC20_Init(void);   
void  EC20_TR_Init(void);
void EC20_Get_Data(void);
void EC20_Change_Server_IP(void);
void EC20Send_ChangeMode(int Mode);
void EC20_Change_Server_Port(void);
void EC20_Change_Server_IP_ByServer(char Change_IP_buff[]);
void EC20_Change_Server_Port_ByServer(char Change_Port_buff[]);
void Set_IP_Port_ByServer(void);


#endif



