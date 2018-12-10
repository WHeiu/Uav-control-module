#include "user.h"
#include "EC20.h"
#include "adc.h"
#define RXLENN 200
extern char ReceiveBuff[];
extern char UART4_ReceiveBuff[];
extern 	int UART4_Receive_Count;
extern 	int Receive_Count;
char *sstrx;

void USART3_CLR_RecvBuf(void)
{
	memset(ReceiveBuff, 0, RXLENN);    //清除USART3接收缓存区
	Receive_Count = 0;
}

 void UART4_CLR_RecvBuf(void)
{
	memset(UART4_ReceiveBuff, 0, RXLENN);    //清除UART4接收缓存区
	UART4_Receive_Count=0;
}


void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen)
{
	int i = 0;
	for(i = 0; i < ucLen; i++)
	{
		//格式化输str,每unsigned char 转换字符占两位置%x写输%X写输
		sprintf(str + i * 2, "%02x", UnChar[i]);
	}

}
//串口交互处理函数
void USART3_Ack_Interaction(void)
{
			
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"BMP280?");  //显示气压计BMP280测量值
		if(sstrx)
		{		
			BMP280_Get_Data();		  //获取子模块气压计BMP280采集值
			USART3_CLR_RecvBuf();
		}			
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"MPU9250?");   //显示九轴传感器MPU9250数据
		if(sstrx)
		{		
			MPU9250_Get_Data();		//获取子模块九轴传感器MPU9250采集数据
			USART3_CLR_RecvBuf();
		}
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"EC20?");
		if(sstrx)
		{		
			EC20_Get_Data();		//获取子模块EC20状态		
			USART3_CLR_RecvBuf();
		}
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"ADC?");
		if(sstrx)
		{		
			ADC_Get_Data();    //获取电池电压
			USART3_CLR_RecvBuf();
	  }
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"Change the IP");  //改变服务器IP地址或端口后需要重新启动EC20
		if(sstrx)
		{		
			EC20_Change_Server_IP();		//更改IP地址		
			USART3_CLR_RecvBuf();
		}
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"Change the Port");  //改变服务器IP地址或端口后需要重新启动EC20
		if(sstrx)
		{		
			EC20_Change_Server_Port();		//更改服务器端口号		
			USART3_CLR_RecvBuf();
		}
		sstrx=strstr((const char*)ReceiveBuff,(const char*)"Restart EC20");    //改变服务器IP地址或端口后需要重新启动EC20
		if(sstrx)
		{		
			EC20_TR_Init();    //重新对EC20进行初始化  
    	printff(USART3, "Restarted the inition of EC20.  \r\n",strlen("Restarted the inition of EC20.  \r\n"));	
			USART3_CLR_RecvBuf();
		}
}

















