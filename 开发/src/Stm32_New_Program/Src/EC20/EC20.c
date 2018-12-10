#include "EC20.h"
#include "main.h"
#include "usart.h"
#include "user.h"
#include <stdlib.h>
GPS_DATA gspdata;
GSM_init GSMinit;
GSM_Status GSMdata;

extern char ReceiveBuff[];
extern char UART4_ReceiveBuff[];
extern char AT_QIOPEN[];

char EC20_rx_buff[512];
uint8_t EC20_rx_flag=0;
uint8_t EC20_rx_num=0;
uint8_t EC20_rx_tempcount=0;

char *gps_strx,*gps_extstrx,*gps_Readystrx;
char *strx0;
char *Re_strx;

//获取IMEI
void Get_IMEI(void)
{
    char i=0,*strx;
    //   printf("AT\r\n"); //同步EC20开机
	  //EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST0);
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT \r\n",strlen("AT\r\n"));
    LL_mDelay(300);
    strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    while(strx==NULL)
    {
  	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
       LL_mDelay(300);
        strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    }	
 		 printff(USART3, "Get_IMEI_START_1\r\n",strlen("Get_IMEI_START_1\r\n"));
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	   LL_mDelay(300);
  while(1)
    {
		      if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//获取模块IMEI的值
							 	         break;
			      else
        {
         		UART4_CLR_RecvBuf();
            printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
					  LL_mDelay(300);		 
        }
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		  for(i=0;i<20;i++)
    {
       GSMdata.IMEI[i]=UART4_ReceiveBuff[i];
    }	
		//printff(USART3, "Get_IMEI_START_2\r\n",strlen("Get_IMEI_START_2\r\n"));
    for(i=0;i<30;i++)
    {
        GSMinit. IMEI[i]=UART4_ReceiveBuff[i+2];
        GSMinit.IMEI[15]=',';
    }	
   for(i=0;i<16;i++)
	 gspdata.senddata[i]=GSMinit.IMEI[i];//获取到IMEI值
   printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n")); //打开GPS电源
   LL_mDelay(300);	
	 printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //关闭上一次连接
   LL_mDelay(300);	
}

/*  获取GPS上报服务器 EC20初始化*/
void  EC20_GPS_Init(void)
{
	  int i;
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
			  printff(UART4, "AT\r\n",strlen("AT\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //查询当前状态
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//返回已经上电
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //如果没上电就上电，上电就不要重复上电
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //对GNSS上电
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
 	  printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));   //关闭回显
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
    LL_mDelay(500);
	  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));  //检查CSQ
   	printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
    LL_mDelay(500);
 	  printff(USART3, "ATI\r\n",strlen("ATI\r\n"));  //检查模块的版本号
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
		for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    /////////////////////////////////
	  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //检查SIM卡是否在位
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//查看是否返回ready
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();
		  	printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		  	printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//检查SIM卡是否在位，等待卡在位，如果卡识别不到，剩余的工作就没法做了
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    ///////////////////////////////////
		printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //查看是否注册GSM网络
		printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//返回正常
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//返回正常，漫游
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //查看是否注册GSM网络
			  printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//返回正常
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//返回正常，漫游
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //查看是否注册GPRS网络
		printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //查看是否注册GPRS网络
		   	printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //查看注册到哪个运营商，支持移动 联通 电信 
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
    LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //关闭socket连接
		printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
    //接入APN，无用户名和密码
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////开启成功
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //去激活
		printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));  //激活
		printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //获取当前卡的IP地址
		printff(UART4, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n"));
    LL_mDelay(500);
			for(i=0;i<15;i++)
    {
			if(UART4_ReceiveBuff[i+17]== '"')
				break;
       GSMdata.Current_Card_IP[i]=UART4_ReceiveBuff[i+17];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
    //这里是需要登陆的IP号码，采用直接吐出模式
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
//		printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
    printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN)); //这里是需要登陆的IP号码，采用直接吐出模式
		printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));
		LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//检查是否登陆成功
    while(gps_strx==NULL)
    {
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//检查是否登陆成功
        LL_mDelay(100);
       
    }
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
}		

///发送字符型数据给服务器
void EC20Send_StrData(char *bufferdata)
{
    uint8_t untildata=0xff;
	  printff(USART3, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
		printff(UART4, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
    LL_mDelay(100);
		printff(UART4, bufferdata,strlen(bufferdata));
    LL_mDelay(100);	
	  LL_USART_TransmitData8(UART4, (uint8_t) 0x1a); //发送完成函数
		while(LL_USART_IsActiveFlag_TC(UART4) == RESET)
		{
		}
		LL_mDelay(100);
		gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//是否正确发送
		while(gps_strx==NULL)
		{
				gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//是否正确发送
				LL_mDelay(10);
		}
    LL_mDelay(100);
	  printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 	  printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
    LL_mDelay(200);
    gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//发送剩余字节数据
		while(untildata)
    {
 				printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		    printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
        LL_mDelay(200);
        gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//发送剩余字节数据
        gps_strx=strstr((char*)gps_strx,(char*)",");//获取第一个,
        gps_strx=strstr((char*)(gps_strx+1),(char*)",");//获取第二个,
        untildata=*(gps_strx+1)-0x30;
        UART4_CLR_RecvBuf();
    }
    printff(USART3, "Send Flish!!!!!\r\n",strlen("Send Flish!!!!!\r\n"));
}

///透传模式下接受数据
void EC20Send_RecAccessMode(void)
{   
	    char Data_SendIMEI_Buffer[15]={0}; 
			int i;
		  Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x01");  //登录
			if(Re_strx)
			{		
				unsigned short Str_Len = 0;
				printff(UART4, "78780A01",strlen("78780A01"));              //串口数据透传至服务器  
				for(i = 0; i < 15; i++)
				   Data_SendIMEI_Buffer[i] = GSMdata.IMEI[i+1];    //剔除第一个换行符
				printff(UART4, Data_SendIMEI_Buffer,strlen(Data_SendIMEI_Buffer));
				printff(UART4, "010D0A",strlen("010D0A"));
				printff(UART4, "\r\n",strlen("\r\n"));
		  	UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x08");  //发送心跳包
			if(Re_strx)
			{				
				printff(UART4, "787801080D0A",strlen("787801080D0A"));     //设备向服务器发送心跳包，发送周期待定
				printff(UART4, "\r\n",strlen("\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x10");  //发送GPS定位数据包
			if(Re_strx)
			{		
				EC20Send_ChangeMode(1);    //切换模式为命令模式，因为透传模式冲突，产生的问题未知
			 	EC20_GetGps_Send();  //获取GPS数据包并发送
				EC20Send_ChangeMode(0);     //切换为透传模式
				UART4_CLR_RecvBuf();
      }
		   Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x13");  //状态包
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
	
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x14");  //设备休眠指令
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x15");  //恢复出厂设置
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x16");  //白名单总数
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x30");  //更新时间
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x40");  //远程监听号码
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x41");  //SOS号码
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x42");  //爸爸号码
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x43");  //妈妈号码
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x44");  //停止数据上传
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x46");  //GPS时间段设置
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x47");  //勿扰时间段设置
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x48");  //重启设备
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x49");  //找设备
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x50");  //闹钟
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x56");  //脱落报警
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x57");  //同步设置数据
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x58");  //同步白名单
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x61");  //光感开关
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x66");  //修改服务器IP和端口地址
			if(Re_strx)
			{ 
				Set_IP_Port_ByServer();    //接收服务器下发的IP地址和端口号，并更新IP和端口  但并未重启EC20执行生效，是否需要立刻重启
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x67");  //恢复密码
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x80");  //手动定位
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x81");  //充电完成
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x82");  //充电连接
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x83");  //充电断开
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x86");  //超速报警
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x92");  //震动报警开
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x93");  //震动报警关
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x94");  //震动报警
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x97");  //服务器设置上传间隔
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x98");  //设备通过短信设置上传间隔，与服务器同步设置
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x99");  //SOS报警
			if(Re_strx)
			{		
				printff(UART4, "787801990D0A",strlen("787801990D0A"));     //向服务器发送求救指令
				printff(UART4, "\r\n",strlen("\r\n"));
				UART4_CLR_RecvBuf();
      }
			
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"NO CARRIER");     //服务器关闭
			if(Re_strx)
			{
				while(1)
				{
					 printff(USART3, "Server Is Closed!\r\n",strlen("Server Is Closed!\r\n"));
				}
      }
}

//服务器下发EC20初始化  
	void  EC20_TR_Init(void)
{
	  int i = 0;
	  EC20Send_ChangeMode(1);  //关闭透传模式，否则重启初始化EC20会失败
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	      printff(UART4, "AT\r\n",strlen("AT\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//返回OK
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //查询当前状态
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//返回已经上电
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //如果没上电就上电，上电就不要重复上电
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //对GNSS上电
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
		
		printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));  //关闭回显
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  //获取模块IMEI的值
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  
	  LL_mDelay(500);
		while(1)
			{
						if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//获取模块IMEI的值
													 break;
							else
					{
							UART4_CLR_RecvBuf();
							printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
							LL_mDelay(500);		 
					}
			}
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		  for(i=0;i<16;i++)
    {
       GSMdata.IMEI[i]=UART4_ReceiveBuff[i+1];   //i+1是为了剔除第一个换行符
    }	
		printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));   //打印存储的IMEI
		UART4_CLR_RecvBuf();
		
		printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n")); //检查CSQ
	  printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		printff(USART3, "ATI\r\n",strlen("ATI\r\n")); //检查模块的版本号
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
			for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //检查SIM卡是否在位
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY"); //查看是否返回ready
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
	      printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//检查SIM卡是否在位，等待卡在位，如果卡识别不到，剩余的工作就没法做了
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
	
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //查看是否注册GPRS网络
	  printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
		LL_mDelay(500);
		Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
		gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
		while(gps_strx==NULL&&gps_extstrx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //查看是否注册GPRS网络
	      printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
				LL_mDelay(500);
				Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//，这里重要，只有注册成功，才可以进行GPRS数据传输。
				gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//返回正常，漫游
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //查看注册到哪个运营商，支持移动 联通 电信 
	  printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
		LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //关闭socket连接
	  printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
	  LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  //接入APN，无用户名和密码
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
	  printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////开启成功
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //去激活
	  printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n")); //激活
	  printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));		
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//开启成功
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //获取当前卡的IP地址
	  printff(UART4, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n"));
		LL_mDelay(500);
		for(i=0;i<15;i++)
    {
			if(UART4_ReceiveBuff[i+17]== '"')
				break;
       GSMdata.Current_Card_IP[i]=UART4_ReceiveBuff[i+17];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
    //设置为透传模式
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
//	  printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
    printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN));
	  printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));	
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//检查是否登陆成功
		while(gps_strx==NULL)
		{
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//检查是否登陆成功
				LL_mDelay(100);
		}
    printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		LL_mDelay(500);
		UART4_CLR_RecvBuf();
}		

//获取GPS数据信息并发送给服务器
void EC20_GetGps_Send(void)
{
		UART4_CLR_RecvBuf();  
		printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));  //读取GPS北斗定位数据
		printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));			
		LL_mDelay(500);
		strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//返回OK
		while(strx0 == NULL)
		{
				LL_mDelay(50);
				strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//返回OK
				printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
				printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
		}			  	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		EC20Send_StrData(UART4_ReceiveBuff);//通过EC20将数据发送出去
};

void EC20_Init(void)
{	 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		LL_mDelay(200);	
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//拉高控制
		LL_mDelay(300);	
	 //	EC20_UART_PROT_INIT();//改变EC20 的串口模式
		LL_mDelay(200);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);//初始化完成
}

void EC20_Get_Data(void)
{
	  printff(USART3, "--------------------------------\r\n", strlen("--------------------------------\r\n"));
	  printff(USART3,"**************EC20**************\r\n",strlen("**************EC20**************\r\n"));
  	printff(USART3,"IMEI：",strlen("IMEI："));        //打印IMEI码
	  printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));
	  printff(USART3,"\r\n\r\nCurrent_Card_IP：\r\n",strlen("\r\n\r\nCurrent_Card_IP：\r\n"));   // 打印当前卡IP
	  printff(USART3,GSMdata.Current_Card_IP,strlen(GSMdata.Current_Card_IP));
		printff(USART3,"\r\n\r\nModule_version_number：",strlen("\r\n\r\nModule_version_number："));   // 打印模块版本号
	  printff(USART3,GSMdata.Module_version_number,strlen(GSMdata.Module_version_number));
	  printff(USART3,"\r\nOperators：",strlen("\r\nOperators："));     //打印营运商
	  printff(USART3,GSMdata.Operators,strlen(GSMdata.Operators));
	
  	/* 查询SIM卡是否在位*/
	  printff(USART3, "\r\n\r\nWhether SIM card is in place：\r\n",strlen("\r\n\r\nWhether SIM card is in place：\r\n"));  //检查SIM卡是否在位
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
	  if(strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY") != NULL)
		  printff(USART3, "SIM READY!\r\n",strlen("SIM READY!\r\n"));
		else 
			printff(USART3, "SIM Error!",strlen("SIM Error!"));  //检查SIM卡是否在位;
	//  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
	
		
    printff(USART3,"\r\n",strlen("\r\n"));
		printff(USART3, "-------------------------------\r\n\r\n", strlen("-------------------------------\r\n\r\n"));
};	

void EC20_Change_Server_IP(void) //更改传输服务器IP地址 通过串口助手更改
{
	  int i = 0, N = 0 , O = 0, Flag = 1;
	  char New_IP[30]={0};
		char Old_IP[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server IP configuration：\r\n",strlen("The original server IP configuration：\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //得到旧的IP地址
				   Old_IP[O] = AT_QIOPEN[O + 21];
	  printff(USART3, Old_IP,strlen(Old_IP));       //打印原始服务器IP
		printff(USART3,"\r\n",strlen("\r\n"));
	  //IP输入格式为：IPXXX.XXX.XXX.XXX  即在IP地址前加一个判断头IP
	  printff(USART3,"Please enter the modified IP(eg:IPXXX.XXX.XXX.XXX)：\r\n",strlen("Please enter the modified IP(eg:IPXXX.XXX.XXX.XXX)：\r\n"));
	  USART3_CLR_RecvBuf();
		while(Flag)
		{
			gps_strx=strstr((const char*)ReceiveBuff,(const char*)"IP");  
			if(gps_strx)
			{		
				for(N = 0; ReceiveBuff[N + 2] != '\0' ; N++)   //接收将要更改的IP地址
				    New_IP[N] = ReceiveBuff[N + 2];
				LL_mDelay(500);
				printff(USART3,New_IP,strlen(New_IP));     //打印出接收到的IP地址
				USART3_CLR_RecvBuf();
  			Flag = 0;               //清除标志位			
			}					 
			LL_mDelay(1000);   //延时1秒是为了方便查看打印出的数据  可以不要
		}
    //N 表示新的IP地址长度 O 表示旧的IP地址长度，因为串口助手发送会在结尾加上0x0d 0x0a 表示发送结束，所以用N与O+2比较IP长度
    if ((N - 2) == O)   //新的IP地址和旧的IP地址长度相同
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0 ; i< N-2 ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21] = New_IP[i];    //更新AT_QIOPEN 即更新IP地址
		};	
		if((N - 2) > O)    //新的IP地址长度大于旧的IP地址
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 ; i++)       //拷贝前面固定数据
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将IP地址后面的数据配置后移
			    IP_Buff [i + (N-2) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N-2 ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    IP_Buff[i + 21] = New_IP[i];    //更新AT_QIOPEN 即更新IP地址
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //将IP_Buff拷贝到AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if((N - 2) < O)     //新的IP地址长度小于旧的IP地址
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将IP地址后面的数据配置前移
			    AT_QIOPEN[i + (N-2) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N-2 ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21] = New_IP[i];    //更新AT_QIOPEN 即更新IP地址
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The IP address has been changed. Please restart...\r\n\r\n",strlen("The IP address has been changed. Please restart...\r\n\r\n"));
};

void EC20_Change_Server_IP_ByServer(char Change_IP_buff[]) //更改传输服务器IP地址 通过服务器下发指令更改
{
	  int i = 0, N = 0 , O = 0, Flag = 1;
	  char New_IP[30]={0};
		char Old_IP[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server IP configuration：\r\n",strlen("The original server IP configuration：\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //得到旧的IP地址
				   Old_IP[O] = AT_QIOPEN[O + 21];
	  printff(USART3, Old_IP,strlen(Old_IP));       //打印原始服务器IP
		printff(USART3,"\r\n",strlen("\r\n"));
		
		gps_strx=strstr((const char*)Change_IP_buff,(const char*)"IP");  
		if(gps_strx)
		{		
			for(N = 0; Change_IP_buff[N + 2] != '\0' ; N++)   //接收将要更改的IP地址
					New_IP[N] = Change_IP_buff[N + 2];
			LL_mDelay(500);
			printff(USART3,"The IP address to be changed is：\r\n",strlen("The IP address to be changed is：\r\n"));
			printff(USART3,New_IP,strlen(New_IP));     //打印出接收到的IP地址	
		}				
		printff(USART3,"\r\n",strlen("\r\n"));     //打印出接收到的IP地址	
		LL_mDelay(1000);   //延时1秒是为了方便查看打印出的数据  可以不要
    //N 表示新的IP地址长度 O 表示旧的IP地址长度，因为串口助手发送会在结尾加上0x0d 0x0a 表示发送结束，所以用N与O+2比较IP长度
    if( N == O)   //新的IP地址和旧的IP地址长度相同
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0 ; i< N ; i++)  
			    AT_QIOPEN[i + 21] = New_IP[i];    //更新AT_QIOPEN 即更新IP地址
		};	
		if( N > O)    //新的IP地址长度大于旧的IP地址
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 ; i++)       //拷贝前面固定数据
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将IP地址后面的数据配置后移
			    IP_Buff [i + (N) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    IP_Buff[i + 21] = New_IP[i];    //更新AT_QIOPEN 即更新IP地址
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //将IP_Buff拷贝到AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if( N < O)     //新的IP地址长度小于旧的IP地址
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将IP地址后面的数据配置前移
			   // AT_QIOPEN[i + (N-2) + 21] = AT_QIOPEN[i + O + 21];
			    AT_QIOPEN[i + (N) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21] = New_IP[i];    //更新AT_QIOPEN 即更新IP地址
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The IP address has been changed. Please restart...\r\n\r\n",strlen("The IP address has been changed. Please restart...\r\n\r\n"));

};
void EC20_Change_Server_Port(void) //更改传输服务器端口  通过串口助手更改
{
	  int i = 0, N = 0 , O = 0,OP = 0, NP = 0, Flag = 1;
		char Old_Port[30] = {0};
		char New_Port[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server Port configuration：\r\n",strlen("The original server Port configuration：\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //循环遍历直到IP结束
				;                                      //这里执行空语句
		for(OP = 0; AT_QIOPEN[OP + O + 21 + 2] != ',' ; OP++)
		       Old_Port[OP] = AT_QIOPEN[OP + O + 21 + 2];
	  printff(USART3, Old_Port,strlen(Old_Port));       //打印原始服务器端口
		printff(USART3,"\r\n",strlen("\r\n"));
	  //IP输入格式为：PortXXXX  即在端口号前加一个判断头Port
	  printff(USART3,"Please enter the modified Port(eg:PortXXXX)：\r\n",strlen("Please enter the modified Port(eg:PortXXXX)：\r\n"));
	  USART3_CLR_RecvBuf();
		while(Flag)
		{
			gps_strx=strstr((const char*)ReceiveBuff,(const char*)"Port");  
			if(gps_strx)
			{		
				for(NP = 0; ReceiveBuff[NP + 4] != '\0' ; NP++)   //接收将要更改的端口号
				    New_Port[NP] = ReceiveBuff[NP + 4];
				LL_mDelay(500);
				printff(USART3,New_Port,strlen(New_Port));     //打印出接收到的端口号
				USART3_CLR_RecvBuf();
  			Flag = 0;               //清除标志位			
			}					 
			LL_mDelay(1000);   //延时1秒是为了方便查看打印出的数据  可以不要
		}
    //Np 表示新的端口号长度 OP 表示旧的端口号长度，因为串口助手发送会在结尾加上0x0d 0x0a 表示发送结束，所以用N与O+2比较端口号长度
    if ((NP - 2) == OP)   //新的端口号和旧的端口号长度相同
		{
			printff(USART3,"Port is being updated...\r\n",strlen("Port is being updated...\r\n"));
			for(i = 0 ; i< (NP-2) ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //更新AT_QIOPEN 即更新端口号
		};	
		if((NP - 2) > OP)    //新的端口号长度大于旧的端口号
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 + O + 2 ; i++)       //拷贝前面固定数据
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将端口号后面的数据配置后移
			    IP_Buff [i + (NP-2) + 21 + O +2] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP-2 ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    IP_Buff[i + 21 + O + 2] = New_Port[i];    //更新AT_QIOPEN 即更新端口号
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //将IP_Buff拷贝到AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if((NP - 2) < OP)     //新的端口号长度小于旧的端口号
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将端口号后面的数据配置前移
			    AT_QIOPEN[i + (NP-2) + 21 + 2 + O] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP-2 ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //更新AT_QIOPEN 即更新端口号
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The Server Port has been changed. Please restart...\r\n\r\n",strlen("The Server Port has been changed. Please restart...\r\n\r\n"));
};
/**
  * 函数名称：void EC20_Change_Server_Port_ByServer(char Change_Port_buff[])
  * 功能及作用：用来更改服务器端口号
  * 描述：形参为即将更改的服务器端口号，
  * 问题：无
  */
void EC20_Change_Server_Port_ByServer(char Change_Port_buff[])
{
	  int i = 0, N = 0 , O = 0,OP = 0, NP = 0, Flag = 1;
		char Old_Port[30] = {0};
		char New_Port[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server Port configuration：\r\n",strlen("The original server Port configuration：\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //循环遍历直到IP结束
				;                                      //这里执行空语句
		for(OP = 0; AT_QIOPEN[OP + O + 21 + 2] != ',' ; OP++)
		       Old_Port[OP] = AT_QIOPEN[OP + O + 21 + 2];
	  printff(USART3, Old_Port,strlen(Old_Port));       //打印原始服务器端口
		printff(USART3,"\r\n",strlen("\r\n"));
	  //IP输入格式为：PortXXXX  即在端口号前加一个判断头Port
	  printff(USART3,"The Server Port to be changed is：\r\n",strlen("The Server Port to be changed is：\r\n"));

			gps_strx=strstr((const char*)Change_Port_buff,(const char*)"Port");  
			if(gps_strx)
			{		
				for(NP = 0; Change_Port_buff[NP + 4] != '\0' ; NP++)   //接收将要更改的端口号
				    New_Port[NP] = Change_Port_buff[NP + 4];
				LL_mDelay(500);
				printff(USART3,New_Port,strlen(New_Port));     //打印出接收到的端口号		
			}					 
			LL_mDelay(1000);   //延时1秒是为了方便查看打印出的数据  可以不要
    printff(USART3,"\r\n",strlen("\r\n"));
    //Np 表示新的端口号长度 OP 表示旧的端口号长度，因为串口助手发送会在结尾加上0x0d 0x0a 表示发送结束，所以用N与O+2比较端口号长度
    if (NP == OP)   //新的端口号和旧的端口号长度相同
		{
			printff(USART3,"Port is being updated...\r\n",strlen("Port is being updated...\r\n"));
			for(i = 0 ; i< NP ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //更新AT_QIOPEN 即更新端口号
		};	
		if( NP > OP)    //新的端口号长度大于旧的端口号
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 + O + 2 ; i++)       //拷贝前面固定数据
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将端口号后面的数据配置后移
			    IP_Buff [i + NP + 21 + O +2] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP ; i++)  // i<N-2 是为了剔除 最后两个换行符 即\r\n  0x0d 0x0a
			    IP_Buff[i + 21 + O + 2] = New_Port[i];    //更新AT_QIOPEN 即更新端口号
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //将IP_Buff拷贝到AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if( NP < OP)     //新的端口号长度小于旧的端口号
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //将端口号后面的数据配置前移
			    AT_QIOPEN[i + NP + 21 + 2 + O] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP ; i++)  
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //更新AT_QIOPEN 即更新端口号
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The Server Port has been changed. Please restart...\r\n\r\n",strlen("The Server Port has been changed. Please restart...\r\n\r\n"));
};

/**
  * 函数名称：void EC20Send_ChangeMode(int Mode)
  * 功能及作用：用来切换EC20数据传送模式 透传或命令模式
  * 描述：参数为非零切换为命令模式，参数为零切换为透传模式
  * 问题：无
  */
void EC20Send_ChangeMode(int Mode)   //Mode参数指定模式，1 为命令模式， 0 为透传模式
{
  if(Mode)//切换为命令模式
	{
		LL_mDelay(500);
		printff(UART4, "+++",strlen("+++"));  //切换透传
		LL_mDelay(800);
  	LL_mDelay(800);
    }
  else//切换为数据模式
    {
        UART4_CLR_RecvBuf();
   //     printf("AT+QISWTMD=0,2\r\n");//切换为透传模式
			  printff(UART4, "AT+QISWTMD=0,2\r\n",strlen("AT+QISWTMD=0,2\r\n"));
        LL_mDelay(300);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//命令切换完成
      while(gps_strx==NULL)
        {
            gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//命令切换完成
        }
        UART4_CLR_RecvBuf();
        LL_mDelay(300);
    }
}

/**
  * 函数名称：void Set_IP_Port_ByServer(void)
  * 功能及作用：用来接收服务器下发的即将更新的IP和Port,并更新全局数组变量AT_QIOPEN
  * 问题：更新全局数组变量AT_QIOPEN后并没有立即重启EC20执行更改，如果需要在更改IP及Port后立即生效，只需在函数末尾调用重启EC20函数EC20_TR_Init
  */
void Set_IP_Port_ByServer(void)
{
	char IP_Buff_Hex0[5] = {0};  //用来存放服务器下来的16进制IP  XX XX XX XX
	char IP_Buff_Hex1[5] = {0};
	char IP_Buff_Hex2[5] = {0};
	char IP_Buff_Hex3[5] = {0};
	char Port_Buff_Hex[10] = {0};
	char IP_Buff[50] = {0};    //用来存放16进制IP转换成10进制后的IP  格式：IPXXX.XXX.XXX.XXX
	char Port_Buff[10] = {0};
	char *str;
	int i = 0, n = 0;
	int IP_data =0 , IP_data1 = 0, IP_data2 = 0, IP_data3 = 0, Port_data = 0;  // 用来存放两位16进制数转换为10进制数后的值
	char data_buff[20] = {0};   //用来存放数字转换后的字符串
	UART4_CLR_RecvBuf();
	while(strstr((const char*)UART4_ReceiveBuff,(const char*)"7878") == NULL)   //等待服务器下发IP和端口号
	{
		printff(UART4, "Please enter IP and Port!\r\n",strlen("Please enter IP and Port!\r\n"));
		LL_mDelay(1000);
	};  
	for(i = 0; i < 12; i++)
	{
		if( i < 2 )
		IP_Buff_Hex0[i] = UART4_ReceiveBuff[i + 4];   //将服务器下发的IP 16进制拷贝到IP_Buff_Hex
		if( ( i >= 2 ) && ( i < 4 ))
		IP_Buff_Hex1[i-2] = UART4_ReceiveBuff[i + 4]; 
		if( ( i >= 4 ) && ( i < 6 ))
		IP_Buff_Hex2[i-4] = UART4_ReceiveBuff[i + 4]; 
		if( ( i >= 6 ) && ( i < 8 ))
		IP_Buff_Hex3[i-6] = UART4_ReceiveBuff[i + 4]; 
		if( ( i >= 8 ) && ( i < 12 ))
		Port_Buff_Hex[i-8] = 	UART4_ReceiveBuff[i + 4];
	}
	
	IP_data = strtol(IP_Buff_Hex0,&str,16);           //将两位16进制数转换为10进制数
	IP_data1 = strtol(IP_Buff_Hex1,&str,16);
	IP_data2 = strtol(IP_Buff_Hex2,&str,16);
	IP_data3 = strtol(IP_Buff_Hex3,&str,16);		
	Port_data = strtol(Port_Buff_Hex,&str,16);	       //将4位16进制数转换为10进制			
	
	sprintf( data_buff, "%d", IP_data);      //将数字转换为字符串
	IP_Buff [n++] = 'I';       //添加IP识别头
	IP_Buff [n++] = 'P';   
	for(i = 0; i < strlen(data_buff); i++)     //写入IP_Buff
		 IP_Buff[n++] = data_buff[i];
	IP_Buff[n++] = '.';
	memset(data_buff, 0, strlen(data_buff));    //清除缓存区data_buff
	sprintf( data_buff, "%d", IP_data1);    //将数字转换为字符串
	for(i = 0; i < strlen(data_buff); i++)     //写入IP_Buff
		 IP_Buff[n++] = data_buff[i];
	IP_Buff[n++] = '.';
	memset(data_buff, 0, strlen(data_buff));    //清除缓存区data_buff
	sprintf( data_buff, "%d", IP_data2);    //将数字转换为字符串
	for(i = 0; i < strlen(data_buff); i++)     //写入IP_Buff
		 IP_Buff[n++] = data_buff[i];
	IP_Buff[n++] = '.';
	memset(data_buff, 0, strlen(data_buff));    //清除缓存区data_buff
	sprintf( data_buff, "%d", IP_data3);    //将数字转换为字符串
	for(i = 0; i < strlen(data_buff); i++)     //写入IP_Buff
		 IP_Buff[n++] = data_buff[i];		
		 
	n = 0;
	Port_Buff [n++] = 'P';       //添加IP识别头
	Port_Buff [n++] = 'o';   
	Port_Buff [n++] = 'r';    
	Port_Buff [n++] = 't';  
	memset(data_buff, 0, strlen(data_buff));    //清除缓存区data_buff  
	sprintf( data_buff, "%d", Port_data);      //将数字转换为字符串
	for(i = 0; i < strlen(data_buff); i++)     //写入Port_Buff
		 Port_Buff[n++] = data_buff[i];		
		 
	printff(USART3, "The port number issued by the server:\r\n",strlen("The port number issued by the server:\r\n"));	 
	printff(USART3, Port_Buff,strlen(Port_Buff));
	printff(USART3, "\r\n",strlen("\r\n"));
	printff(USART3, "The IP adderss issued by the server:\r\n",strlen("The IP adderss issued by the server:\r\n"));	
	printff(USART3, IP_Buff,strlen(IP_Buff));
	printff(USART3, "\r\n",strlen("\r\n"));
	
	EC20_Change_Server_IP_ByServer(IP_Buff);
	EC20_Change_Server_Port_ByServer(Port_Buff);
}