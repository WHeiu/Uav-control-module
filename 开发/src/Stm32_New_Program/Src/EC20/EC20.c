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

//��ȡIMEI
void Get_IMEI(void)
{
    char i=0,*strx;
    //   printf("AT\r\n"); //ͬ��EC20����
	  //EC20_Transmit((uint8_t*)UART4_Tx_GPS_TEST0);
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT \r\n",strlen("AT\r\n"));
    LL_mDelay(300);
    strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    while(strx==NULL)
    {
  	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
       LL_mDelay(300);
        strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    }	
 		 printff(USART3, "Get_IMEI_START_1\r\n",strlen("Get_IMEI_START_1\r\n"));
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));
	   LL_mDelay(300);
  while(1)
    {
		      if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//��ȡģ��IMEI��ֵ
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
	 gspdata.senddata[i]=GSMinit.IMEI[i];//��ȡ��IMEIֵ
   printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n")); //��GPS��Դ
   LL_mDelay(300);	
	 printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //�ر���һ������
   LL_mDelay(300);	
}

/*  ��ȡGPS�ϱ������� EC20��ʼ��*/
void  EC20_GPS_Init(void)
{
	  int i;
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
			  printff(UART4, "AT\r\n",strlen("AT\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //��ѯ��ǰ״̬
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//�����Ѿ��ϵ�
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //���û�ϵ���ϵ磬�ϵ�Ͳ�Ҫ�ظ��ϵ�
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //��GNSS�ϵ�
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
 	  printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));   //�رջ���
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
    LL_mDelay(500);
	  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));  //���CSQ
   	printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
    LL_mDelay(500);
 	  printff(USART3, "ATI\r\n",strlen("ATI\r\n"));  //���ģ��İ汾��
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
		for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    /////////////////////////////////
	  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //���SIM���Ƿ���λ
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//�鿴�Ƿ񷵻�ready
    while(gps_strx==NULL)
    {
        UART4_CLR_RecvBuf();
		  	printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		  	printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//���SIM���Ƿ���λ���ȴ�����λ�������ʶ�𲻵���ʣ��Ĺ�����û������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();	
    ///////////////////////////////////
		printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //�鿴�Ƿ�ע��GSM����
		printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//��������
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//��������������
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CREG?\r\n",strlen("AT+CREG?\r\n")); //�鿴�Ƿ�ע��GSM����
			  printff(UART4, "AT+CREG?\r\n",strlen("AT+CREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,1");//��������
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CREG: 0,5");//��������������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //�鿴�Ƿ�ע��GPRS����
		printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
    gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
    while(gps_strx==NULL&&gps_extstrx==NULL)
    {
        UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //�鿴�Ƿ�ע��GPRS����
		   	printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
        gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //�鿴ע�ᵽ�ĸ���Ӫ�̣�֧���ƶ� ��ͨ ���� 
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
    LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //�ر�socket����
		printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
    //����APN�����û���������
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
  while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //ȥ����
		printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));  //����
		printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    while(gps_strx==NULL)
    {
        LL_mDelay(500);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
    }
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //��ȡ��ǰ����IP��ַ
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
    //��������Ҫ��½��IP���룬����ֱ���³�ģʽ
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
//		printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,1\r\n"));
    printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN)); //��������Ҫ��½��IP���룬����ֱ���³�ģʽ
		printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));
		LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//����Ƿ��½�ɹ�
    while(gps_strx==NULL)
    {
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QIOPEN: 0,0");//����Ƿ��½�ɹ�
        LL_mDelay(100);
       
    }
    LL_mDelay(500);
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
}		

///�����ַ������ݸ�������
void EC20Send_StrData(char *bufferdata)
{
    uint8_t untildata=0xff;
	  printff(USART3, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
		printff(UART4, "AT+QISEND=0\r\n",strlen("AT+QISEND=0\r\n"));
    LL_mDelay(100);
		printff(UART4, bufferdata,strlen(bufferdata));
    LL_mDelay(100);	
	  LL_USART_TransmitData8(UART4, (uint8_t) 0x1a); //������ɺ���
		while(LL_USART_IsActiveFlag_TC(UART4) == RESET)
		{
		}
		LL_mDelay(100);
		gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//�Ƿ���ȷ����
		while(gps_strx==NULL)
		{
				gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"SEND OK");//�Ƿ���ȷ����
				LL_mDelay(10);
		}
    LL_mDelay(100);
	  printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
    UART4_CLR_RecvBuf();
 	  printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
    LL_mDelay(200);
    gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//����ʣ���ֽ�����
		while(untildata)
    {
 				printff(USART3, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
		    printff(UART4, "AT+QISEND=0,0\r\n",strlen("AT+QISEND=0,0\r\n"));
        LL_mDelay(200);
        gps_strx=strstr((char*)UART4_ReceiveBuff,(char*)"+QISEND:");//����ʣ���ֽ�����
        gps_strx=strstr((char*)gps_strx,(char*)",");//��ȡ��һ��,
        gps_strx=strstr((char*)(gps_strx+1),(char*)",");//��ȡ�ڶ���,
        untildata=*(gps_strx+1)-0x30;
        UART4_CLR_RecvBuf();
    }
    printff(USART3, "Send Flish!!!!!\r\n",strlen("Send Flish!!!!!\r\n"));
}

///͸��ģʽ�½�������
void EC20Send_RecAccessMode(void)
{   
	    char Data_SendIMEI_Buffer[15]={0}; 
			int i;
		  Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x01");  //��¼
			if(Re_strx)
			{		
				unsigned short Str_Len = 0;
				printff(UART4, "78780A01",strlen("78780A01"));              //��������͸����������  
				for(i = 0; i < 15; i++)
				   Data_SendIMEI_Buffer[i] = GSMdata.IMEI[i+1];    //�޳���һ�����з�
				printff(UART4, Data_SendIMEI_Buffer,strlen(Data_SendIMEI_Buffer));
				printff(UART4, "010D0A",strlen("010D0A"));
				printff(UART4, "\r\n",strlen("\r\n"));
		  	UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x08");  //����������
			if(Re_strx)
			{				
				printff(UART4, "787801080D0A",strlen("787801080D0A"));     //�豸��������������������������ڴ���
				printff(UART4, "\r\n",strlen("\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x10");  //����GPS��λ���ݰ�
			if(Re_strx)
			{		
				EC20Send_ChangeMode(1);    //�л�ģʽΪ����ģʽ����Ϊ͸��ģʽ��ͻ������������δ֪
			 	EC20_GetGps_Send();  //��ȡGPS���ݰ�������
				EC20Send_ChangeMode(0);     //�л�Ϊ͸��ģʽ
				UART4_CLR_RecvBuf();
      }
		   Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x13");  //״̬��
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
	
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x14");  //�豸����ָ��
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x15");  //�ָ���������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x16");  //����������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x30");  //����ʱ��
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x40");  //Զ�̼�������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x41");  //SOS����
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x42");  //�ְֺ���
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x43");  //�������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x44");  //ֹͣ�����ϴ�
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x46");  //GPSʱ�������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x47");  //����ʱ�������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x48");  //�����豸
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x49");  //���豸
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x50");  //����
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x56");  //���䱨��
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x57");  //ͬ����������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x58");  //ͬ��������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x61");  //��п���
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x66");  //�޸ķ�����IP�Ͷ˿ڵ�ַ
			if(Re_strx)
			{ 
				Set_IP_Port_ByServer();    //���շ������·���IP��ַ�Ͷ˿ںţ�������IP�Ͷ˿�  ����δ����EC20ִ����Ч���Ƿ���Ҫ��������
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x67");  //�ָ�����
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x80");  //�ֶ���λ
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x81");  //������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x82");  //�������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x83");  //���Ͽ�
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x86");  //���ٱ���
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x92");  //�𶯱�����
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x93");  //�𶯱�����
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x94");  //�𶯱���
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x97");  //�����������ϴ����
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x98");  //�豸ͨ�����������ϴ�������������ͬ������
			if(Re_strx)
			{		
				printff(USART3, "B0 running!!!!!\r\n",strlen("B0 running!!!!!\r\n"));
				UART4_CLR_RecvBuf();
      }
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"0x99");  //SOS����
			if(Re_strx)
			{		
				printff(UART4, "787801990D0A",strlen("787801990D0A"));     //��������������ָ��
				printff(UART4, "\r\n",strlen("\r\n"));
				UART4_CLR_RecvBuf();
      }
			
			 Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"NO CARRIER");     //�������ر�
			if(Re_strx)
			{
				while(1)
				{
					 printff(USART3, "Server Is Closed!\r\n",strlen("Server Is Closed!\r\n"));
				}
      }
}

//�������·�EC20��ʼ��  
	void  EC20_TR_Init(void)
{
	  int i = 0;
	  EC20Send_ChangeMode(1);  //�ر�͸��ģʽ������������ʼ��EC20��ʧ��
	  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	  printff(UART4, "AT\r\n",strlen("AT\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();	
			  printff(USART3, "AT\r\n",strlen("AT\r\n"));
	      printff(UART4, "AT\r\n",strlen("AT\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//����OK
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(USART3, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));    //��ѯ��ǰ״̬
		printff(UART4, "AT+QGPS?\r\n",strlen("AT+QGPS?\r\n"));
    LL_mDelay(500);
    gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPS: 1");//�����Ѿ��ϵ�
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		if(gps_strx==NULL)           //���û�ϵ���ϵ磬�ϵ�Ͳ�Ҫ�ظ��ϵ�
		{
			printff(USART3, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));    //��GNSS�ϵ�
			printff(UART4, "AT+QGPS=1\r\n",strlen("AT+QGPS=1\r\n"));
		}
    LL_mDelay(500);
		
		printff(USART3, "ATE0\r\n",strlen("ATE0\r\n"));  //�رջ���
	  printff(UART4, "ATE0\r\n",strlen("ATE0\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
		
		printff(UART4, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  //��ȡģ��IMEI��ֵ
	  printff(USART3, "AT+GSN\r\n",strlen("AT+GSN\r\n"));  
	  LL_mDelay(500);
		while(1)
			{
						if(UART4_ReceiveBuff[2]>='0'&&UART4_ReceiveBuff[2]<='9')//��ȡģ��IMEI��ֵ
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
       GSMdata.IMEI[i]=UART4_ReceiveBuff[i+1];   //i+1��Ϊ���޳���һ�����з�
    }	
		printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));   //��ӡ�洢��IMEI
		UART4_CLR_RecvBuf();
		
		printff(USART3, "AT+CSQ\r\n",strlen("AT+CSQ\r\n")); //���CSQ
	  printff(UART4, "AT+CSQ\r\n",strlen("AT+CSQ\r\n"));
		LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		printff(USART3, "ATI\r\n",strlen("ATI\r\n")); //���ģ��İ汾��
	  printff(UART4, "ATI\r\n",strlen("ATI\r\n"));
		LL_mDelay(500);
			for(i=0;i<100;i++)
    {
       GSMdata.Module_version_number[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));  //���SIM���Ƿ���λ
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY"); //�鿴�Ƿ񷵻�ready
		while(gps_strx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
	      printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY");//���SIM���Ƿ���λ���ȴ�����λ�������ʶ�𲻵���ʣ��Ĺ�����û������
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();	
	
		printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n")); //�鿴�Ƿ�ע��GPRS����
	  printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
		LL_mDelay(500);
		Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
		gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
		while(gps_strx==NULL&&gps_extstrx==NULL)
		{
				UART4_CLR_RecvBuf();
			  printff(USART3, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));  //�鿴�Ƿ�ע��GPRS����
	      printff(UART4, "AT+CGREG?\r\n",strlen("AT+CGREG?\r\n"));
				LL_mDelay(500);
				Re_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,1");//��������Ҫ��ֻ��ע��ɹ����ſ��Խ���GPRS���ݴ��䡣
				gps_extstrx=strstr((const char*)UART4_ReceiveBuff,(const char*)"+CGREG: 0,5");//��������������
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+COPS?\r\n",strlen("AT+COPS?\r\n")); //�鿴ע�ᵽ�ĸ���Ӫ�̣�֧���ƶ� ��ͨ ���� 
	  printff(UART4, "AT+COPS?\r\n",strlen("AT+COPS?\r\n"));
		LL_mDelay(500);
				for(i=0;i<30;i++)
    {
         GSMdata.Operators[i]=UART4_ReceiveBuff[i];
    }	
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n")); //�ر�socket����
	  printff(UART4, "AT+QICLOSE=0\r\n",strlen("AT+QICLOSE=0\r\n"));
	  LL_mDelay(500);
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
	  //����APN�����û���������
		printff(USART3, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
	  printff(UART4, "AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n",strlen("AT+QICSGP=1,1,\042CMNET\042,\042\042,\042\042,0\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");////�����ɹ�
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n")); //ȥ����
	  printff(UART4, "AT+QIDEACT=1\r\n",strlen("AT+QIDEACT=1\r\n"));
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n")); //����
	  printff(UART4, "AT+QIACT=1\r\n",strlen("AT+QIACT=1\r\n"));		
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		while(gps_strx==NULL)
		{
				LL_mDelay(500);
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"OK");//�����ɹ�
		}
		printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		UART4_CLR_RecvBuf();
		printff(USART3, "AT+QIACT?\r\n",strlen("AT+QIACT?\r\n")); //��ȡ��ǰ����IP��ַ
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
    //����Ϊ͸��ģʽ
//		printff(USART3, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
//	  printff(UART4, "AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n",strlen("AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"));
    printff(USART3, AT_QIOPEN,strlen(AT_QIOPEN));
	  printff(UART4, AT_QIOPEN,strlen(AT_QIOPEN));	
		LL_mDelay(500);
		gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//����Ƿ��½�ɹ�
		while(gps_strx==NULL)
		{
				gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//����Ƿ��½�ɹ�
				LL_mDelay(100);
		}
    printff(USART3, UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		LL_mDelay(500);
		UART4_CLR_RecvBuf();
}		

//��ȡGPS������Ϣ�����͸�������
void EC20_GetGps_Send(void)
{
		UART4_CLR_RecvBuf();  
		printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));  //��ȡGPS������λ����
		printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));			
		LL_mDelay(500);
		strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//����OK
		while(strx0 == NULL)
		{
				LL_mDelay(50);
				strx0=strstr((const char*)UART4_ReceiveBuff,(const char*)"+QGPSGNMEA:");//����OK
				printff(USART3, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
				printff(UART4, "AT+QGPSGNMEA=\042GGA\042\r\n",strlen("AT+QGPSGNMEA=\042GGA\042\r\n"));
		}			  	
		printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
		EC20Send_StrData(UART4_ReceiveBuff);//ͨ��EC20�����ݷ��ͳ�ȥ
};

void EC20_Init(void)
{	 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		LL_mDelay(200);	
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//���߿���
		LL_mDelay(300);	
	 //	EC20_UART_PROT_INIT();//�ı�EC20 �Ĵ���ģʽ
		LL_mDelay(200);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);//��ʼ�����
}

void EC20_Get_Data(void)
{
	  printff(USART3, "--------------------------------\r\n", strlen("--------------------------------\r\n"));
	  printff(USART3,"**************EC20**************\r\n",strlen("**************EC20**************\r\n"));
  	printff(USART3,"IMEI��",strlen("IMEI��"));        //��ӡIMEI��
	  printff(USART3,GSMdata.IMEI,strlen(GSMdata.IMEI));
	  printff(USART3,"\r\n\r\nCurrent_Card_IP��\r\n",strlen("\r\n\r\nCurrent_Card_IP��\r\n"));   // ��ӡ��ǰ��IP
	  printff(USART3,GSMdata.Current_Card_IP,strlen(GSMdata.Current_Card_IP));
		printff(USART3,"\r\n\r\nModule_version_number��",strlen("\r\n\r\nModule_version_number��"));   // ��ӡģ��汾��
	  printff(USART3,GSMdata.Module_version_number,strlen(GSMdata.Module_version_number));
	  printff(USART3,"\r\nOperators��",strlen("\r\nOperators��"));     //��ӡӪ����
	  printff(USART3,GSMdata.Operators,strlen(GSMdata.Operators));
	
  	/* ��ѯSIM���Ƿ���λ*/
	  printff(USART3, "\r\n\r\nWhether SIM card is in place��\r\n",strlen("\r\n\r\nWhether SIM card is in place��\r\n"));  //���SIM���Ƿ���λ
	  printff(UART4, "AT+CPIN?\r\n",strlen("AT+CPIN?\r\n"));
    LL_mDelay(500);
	  if(strstr((const char*)UART4_ReceiveBuff,(const char*)"+CPIN: READY") != NULL)
		  printff(USART3, "SIM READY!\r\n",strlen("SIM READY!\r\n"));
		else 
			printff(USART3, "SIM Error!",strlen("SIM Error!"));  //���SIM���Ƿ���λ;
	//  printff(USART3,UART4_ReceiveBuff,strlen(UART4_ReceiveBuff));
	
		
    printff(USART3,"\r\n",strlen("\r\n"));
		printff(USART3, "-------------------------------\r\n\r\n", strlen("-------------------------------\r\n\r\n"));
};	

void EC20_Change_Server_IP(void) //���Ĵ��������IP��ַ ͨ���������ָ���
{
	  int i = 0, N = 0 , O = 0, Flag = 1;
	  char New_IP[30]={0};
		char Old_IP[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server IP configuration��\r\n",strlen("The original server IP configuration��\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //�õ��ɵ�IP��ַ
				   Old_IP[O] = AT_QIOPEN[O + 21];
	  printff(USART3, Old_IP,strlen(Old_IP));       //��ӡԭʼ������IP
		printff(USART3,"\r\n",strlen("\r\n"));
	  //IP�����ʽΪ��IPXXX.XXX.XXX.XXX  ����IP��ַǰ��һ���ж�ͷIP
	  printff(USART3,"Please enter the modified IP(eg:IPXXX.XXX.XXX.XXX)��\r\n",strlen("Please enter the modified IP(eg:IPXXX.XXX.XXX.XXX)��\r\n"));
	  USART3_CLR_RecvBuf();
		while(Flag)
		{
			gps_strx=strstr((const char*)ReceiveBuff,(const char*)"IP");  
			if(gps_strx)
			{		
				for(N = 0; ReceiveBuff[N + 2] != '\0' ; N++)   //���ս�Ҫ���ĵ�IP��ַ
				    New_IP[N] = ReceiveBuff[N + 2];
				LL_mDelay(500);
				printff(USART3,New_IP,strlen(New_IP));     //��ӡ�����յ���IP��ַ
				USART3_CLR_RecvBuf();
  			Flag = 0;               //�����־λ			
			}					 
			LL_mDelay(1000);   //��ʱ1����Ϊ�˷���鿴��ӡ��������  ���Բ�Ҫ
		}
    //N ��ʾ�µ�IP��ַ���� O ��ʾ�ɵ�IP��ַ���ȣ���Ϊ�������ַ��ͻ��ڽ�β����0x0d 0x0a ��ʾ���ͽ�����������N��O+2�Ƚ�IP����
    if ((N - 2) == O)   //�µ�IP��ַ�;ɵ�IP��ַ������ͬ
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0 ; i< N-2 ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21] = New_IP[i];    //����AT_QIOPEN ������IP��ַ
		};	
		if((N - 2) > O)    //�µ�IP��ַ���ȴ��ھɵ�IP��ַ
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 ; i++)       //����ǰ��̶�����
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //��IP��ַ������������ú���
			    IP_Buff [i + (N-2) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N-2 ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    IP_Buff[i + 21] = New_IP[i];    //����AT_QIOPEN ������IP��ַ
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //��IP_Buff������AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if((N - 2) < O)     //�µ�IP��ַ����С�ھɵ�IP��ַ
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //��IP��ַ�������������ǰ��
			    AT_QIOPEN[i + (N-2) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N-2 ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21] = New_IP[i];    //����AT_QIOPEN ������IP��ַ
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The IP address has been changed. Please restart...\r\n\r\n",strlen("The IP address has been changed. Please restart...\r\n\r\n"));
};

void EC20_Change_Server_IP_ByServer(char Change_IP_buff[]) //���Ĵ��������IP��ַ ͨ���������·�ָ�����
{
	  int i = 0, N = 0 , O = 0, Flag = 1;
	  char New_IP[30]={0};
		char Old_IP[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server IP configuration��\r\n",strlen("The original server IP configuration��\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //�õ��ɵ�IP��ַ
				   Old_IP[O] = AT_QIOPEN[O + 21];
	  printff(USART3, Old_IP,strlen(Old_IP));       //��ӡԭʼ������IP
		printff(USART3,"\r\n",strlen("\r\n"));
		
		gps_strx=strstr((const char*)Change_IP_buff,(const char*)"IP");  
		if(gps_strx)
		{		
			for(N = 0; Change_IP_buff[N + 2] != '\0' ; N++)   //���ս�Ҫ���ĵ�IP��ַ
					New_IP[N] = Change_IP_buff[N + 2];
			LL_mDelay(500);
			printff(USART3,"The IP address to be changed is��\r\n",strlen("The IP address to be changed is��\r\n"));
			printff(USART3,New_IP,strlen(New_IP));     //��ӡ�����յ���IP��ַ	
		}				
		printff(USART3,"\r\n",strlen("\r\n"));     //��ӡ�����յ���IP��ַ	
		LL_mDelay(1000);   //��ʱ1����Ϊ�˷���鿴��ӡ��������  ���Բ�Ҫ
    //N ��ʾ�µ�IP��ַ���� O ��ʾ�ɵ�IP��ַ���ȣ���Ϊ�������ַ��ͻ��ڽ�β����0x0d 0x0a ��ʾ���ͽ�����������N��O+2�Ƚ�IP����
    if( N == O)   //�µ�IP��ַ�;ɵ�IP��ַ������ͬ
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0 ; i< N ; i++)  
			    AT_QIOPEN[i + 21] = New_IP[i];    //����AT_QIOPEN ������IP��ַ
		};	
		if( N > O)    //�µ�IP��ַ���ȴ��ھɵ�IP��ַ
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 ; i++)       //����ǰ��̶�����
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //��IP��ַ������������ú���
			    IP_Buff [i + (N) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    IP_Buff[i + 21] = New_IP[i];    //����AT_QIOPEN ������IP��ַ
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //��IP_Buff������AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if( N < O)     //�µ�IP��ַ����С�ھɵ�IP��ַ
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //��IP��ַ�������������ǰ��
			   // AT_QIOPEN[i + (N-2) + 21] = AT_QIOPEN[i + O + 21];
			    AT_QIOPEN[i + (N) + 21] = AT_QIOPEN[i + O + 21];
			for(i = 0 ; i< N ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21] = New_IP[i];    //����AT_QIOPEN ������IP��ַ
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The IP address has been changed. Please restart...\r\n\r\n",strlen("The IP address has been changed. Please restart...\r\n\r\n"));

};
void EC20_Change_Server_Port(void) //���Ĵ���������˿�  ͨ���������ָ���
{
	  int i = 0, N = 0 , O = 0,OP = 0, NP = 0, Flag = 1;
		char Old_Port[30] = {0};
		char New_Port[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server Port configuration��\r\n",strlen("The original server Port configuration��\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //ѭ������ֱ��IP����
				;                                      //����ִ�п����
		for(OP = 0; AT_QIOPEN[OP + O + 21 + 2] != ',' ; OP++)
		       Old_Port[OP] = AT_QIOPEN[OP + O + 21 + 2];
	  printff(USART3, Old_Port,strlen(Old_Port));       //��ӡԭʼ�������˿�
		printff(USART3,"\r\n",strlen("\r\n"));
	  //IP�����ʽΪ��PortXXXX  ���ڶ˿ں�ǰ��һ���ж�ͷPort
	  printff(USART3,"Please enter the modified Port(eg:PortXXXX)��\r\n",strlen("Please enter the modified Port(eg:PortXXXX)��\r\n"));
	  USART3_CLR_RecvBuf();
		while(Flag)
		{
			gps_strx=strstr((const char*)ReceiveBuff,(const char*)"Port");  
			if(gps_strx)
			{		
				for(NP = 0; ReceiveBuff[NP + 4] != '\0' ; NP++)   //���ս�Ҫ���ĵĶ˿ں�
				    New_Port[NP] = ReceiveBuff[NP + 4];
				LL_mDelay(500);
				printff(USART3,New_Port,strlen(New_Port));     //��ӡ�����յ��Ķ˿ں�
				USART3_CLR_RecvBuf();
  			Flag = 0;               //�����־λ			
			}					 
			LL_mDelay(1000);   //��ʱ1����Ϊ�˷���鿴��ӡ��������  ���Բ�Ҫ
		}
    //Np ��ʾ�µĶ˿ںų��� OP ��ʾ�ɵĶ˿ںų��ȣ���Ϊ�������ַ��ͻ��ڽ�β����0x0d 0x0a ��ʾ���ͽ�����������N��O+2�Ƚ϶˿ںų���
    if ((NP - 2) == OP)   //�µĶ˿ںź;ɵĶ˿ںų�����ͬ
		{
			printff(USART3,"Port is being updated...\r\n",strlen("Port is being updated...\r\n"));
			for(i = 0 ; i< (NP-2) ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //����AT_QIOPEN �����¶˿ں�
		};	
		if((NP - 2) > OP)    //�µĶ˿ںų��ȴ��ھɵĶ˿ں�
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 + O + 2 ; i++)       //����ǰ��̶�����
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //���˿ںź�����������ú���
			    IP_Buff [i + (NP-2) + 21 + O +2] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP-2 ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    IP_Buff[i + 21 + O + 2] = New_Port[i];    //����AT_QIOPEN �����¶˿ں�
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //��IP_Buff������AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if((NP - 2) < OP)     //�µĶ˿ںų���С�ھɵĶ˿ں�
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //���˿ںź������������ǰ��
			    AT_QIOPEN[i + (NP-2) + 21 + 2 + O] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP-2 ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //����AT_QIOPEN �����¶˿ں�
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The Server Port has been changed. Please restart...\r\n\r\n",strlen("The Server Port has been changed. Please restart...\r\n\r\n"));
};
/**
  * �������ƣ�void EC20_Change_Server_Port_ByServer(char Change_Port_buff[])
  * ���ܼ����ã��������ķ������˿ں�
  * �������β�Ϊ�������ĵķ������˿ںţ�
  * ���⣺��
  */
void EC20_Change_Server_Port_ByServer(char Change_Port_buff[])
{
	  int i = 0, N = 0 , O = 0,OP = 0, NP = 0, Flag = 1;
		char Old_Port[30] = {0};
		char New_Port[30] = {0};
		char IP_Buff[100] = {0};
	  //AT_QIOPEN[100]={"AT+QIOPEN=1,0,\042TCP\042,\042132.232.49.195\042,5159,0,2\r\n"};
	  printff(USART3,"The original server Port configuration��\r\n",strlen("The original server Port configuration��\r\n"));
		for(O = 0; AT_QIOPEN[O + 21] != '"' ; O++)   //ѭ������ֱ��IP����
				;                                      //����ִ�п����
		for(OP = 0; AT_QIOPEN[OP + O + 21 + 2] != ',' ; OP++)
		       Old_Port[OP] = AT_QIOPEN[OP + O + 21 + 2];
	  printff(USART3, Old_Port,strlen(Old_Port));       //��ӡԭʼ�������˿�
		printff(USART3,"\r\n",strlen("\r\n"));
	  //IP�����ʽΪ��PortXXXX  ���ڶ˿ں�ǰ��һ���ж�ͷPort
	  printff(USART3,"The Server Port to be changed is��\r\n",strlen("The Server Port to be changed is��\r\n"));

			gps_strx=strstr((const char*)Change_Port_buff,(const char*)"Port");  
			if(gps_strx)
			{		
				for(NP = 0; Change_Port_buff[NP + 4] != '\0' ; NP++)   //���ս�Ҫ���ĵĶ˿ں�
				    New_Port[NP] = Change_Port_buff[NP + 4];
				LL_mDelay(500);
				printff(USART3,New_Port,strlen(New_Port));     //��ӡ�����յ��Ķ˿ں�		
			}					 
			LL_mDelay(1000);   //��ʱ1����Ϊ�˷���鿴��ӡ��������  ���Բ�Ҫ
    printff(USART3,"\r\n",strlen("\r\n"));
    //Np ��ʾ�µĶ˿ںų��� OP ��ʾ�ɵĶ˿ںų��ȣ���Ϊ�������ַ��ͻ��ڽ�β����0x0d 0x0a ��ʾ���ͽ�����������N��O+2�Ƚ϶˿ںų���
    if (NP == OP)   //�µĶ˿ںź;ɵĶ˿ںų�����ͬ
		{
			printff(USART3,"Port is being updated...\r\n",strlen("Port is being updated...\r\n"));
			for(i = 0 ; i< NP ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //����AT_QIOPEN �����¶˿ں�
		};	
		if( NP > OP)    //�µĶ˿ںų��ȴ��ھɵĶ˿ں�
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; i < 21 + O + 2 ; i++)       //����ǰ��̶�����
			    IP_Buff [i] = AT_QIOPEN[i ];
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //���˿ںź�����������ú���
			    IP_Buff [i + NP + 21 + O +2] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP ; i++)  // i<N-2 ��Ϊ���޳� ����������з� ��\r\n  0x0d 0x0a
			    IP_Buff[i + 21 + O + 2] = New_Port[i];    //����AT_QIOPEN �����¶˿ں�
			for(i = 0; IP_Buff[i] != '\0' ; i++)  //��IP_Buff������AT_QIOPEN
	        AT_QIOPEN[i] = IP_Buff[i];
		}
	  if( NP < OP)     //�µĶ˿ںų���С�ھɵĶ˿ں�
		{
			printff(USART3,"IP address is being updated...\r\n",strlen("IP address is being updated...\r\n"));
			for(i = 0; AT_QIOPEN[i + O + 21] != '\0' ; i++)       //���˿ںź������������ǰ��
			    AT_QIOPEN[i + NP + 21 + 2 + O] = AT_QIOPEN[i + O + 21 + 2 + OP];
			for(i = 0 ; i< NP ; i++)  
			    AT_QIOPEN[i + 21 + O + 2] = New_Port[i];    //����AT_QIOPEN �����¶˿ں�
		}	
		printff(USART3,AT_QIOPEN,strlen(AT_QIOPEN));
		printff(USART3,"The Server Port has been changed. Please restart...\r\n\r\n",strlen("The Server Port has been changed. Please restart...\r\n\r\n"));
};

/**
  * �������ƣ�void EC20Send_ChangeMode(int Mode)
  * ���ܼ����ã������л�EC20���ݴ���ģʽ ͸��������ģʽ
  * ����������Ϊ�����л�Ϊ����ģʽ������Ϊ���л�Ϊ͸��ģʽ
  * ���⣺��
  */
void EC20Send_ChangeMode(int Mode)   //Mode����ָ��ģʽ��1 Ϊ����ģʽ�� 0 Ϊ͸��ģʽ
{
  if(Mode)//�л�Ϊ����ģʽ
	{
		LL_mDelay(500);
		printff(UART4, "+++",strlen("+++"));  //�л�͸��
		LL_mDelay(800);
  	LL_mDelay(800);
    }
  else//�л�Ϊ����ģʽ
    {
        UART4_CLR_RecvBuf();
   //     printf("AT+QISWTMD=0,2\r\n");//�л�Ϊ͸��ģʽ
			  printff(UART4, "AT+QISWTMD=0,2\r\n",strlen("AT+QISWTMD=0,2\r\n"));
        LL_mDelay(300);
        gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//�����л����
      while(gps_strx==NULL)
        {
            gps_strx=strstr((const char*)UART4_ReceiveBuff,(const char*)"CONNECT");//�����л����
        }
        UART4_CLR_RecvBuf();
        LL_mDelay(300);
    }
}

/**
  * �������ƣ�void Set_IP_Port_ByServer(void)
  * ���ܼ����ã��������շ������·��ļ������µ�IP��Port,������ȫ���������AT_QIOPEN
  * ���⣺����ȫ���������AT_QIOPEN��û����������EC20ִ�и��ģ������Ҫ�ڸ���IP��Port��������Ч��ֻ���ں���ĩβ��������EC20����EC20_TR_Init
  */
void Set_IP_Port_ByServer(void)
{
	char IP_Buff_Hex0[5] = {0};  //������ŷ�����������16����IP  XX XX XX XX
	char IP_Buff_Hex1[5] = {0};
	char IP_Buff_Hex2[5] = {0};
	char IP_Buff_Hex3[5] = {0};
	char Port_Buff_Hex[10] = {0};
	char IP_Buff[50] = {0};    //�������16����IPת����10���ƺ��IP  ��ʽ��IPXXX.XXX.XXX.XXX
	char Port_Buff[10] = {0};
	char *str;
	int i = 0, n = 0;
	int IP_data =0 , IP_data1 = 0, IP_data2 = 0, IP_data3 = 0, Port_data = 0;  // ���������λ16������ת��Ϊ10���������ֵ
	char data_buff[20] = {0};   //�����������ת������ַ���
	UART4_CLR_RecvBuf();
	while(strstr((const char*)UART4_ReceiveBuff,(const char*)"7878") == NULL)   //�ȴ��������·�IP�Ͷ˿ں�
	{
		printff(UART4, "Please enter IP and Port!\r\n",strlen("Please enter IP and Port!\r\n"));
		LL_mDelay(1000);
	};  
	for(i = 0; i < 12; i++)
	{
		if( i < 2 )
		IP_Buff_Hex0[i] = UART4_ReceiveBuff[i + 4];   //���������·���IP 16���ƿ�����IP_Buff_Hex
		if( ( i >= 2 ) && ( i < 4 ))
		IP_Buff_Hex1[i-2] = UART4_ReceiveBuff[i + 4]; 
		if( ( i >= 4 ) && ( i < 6 ))
		IP_Buff_Hex2[i-4] = UART4_ReceiveBuff[i + 4]; 
		if( ( i >= 6 ) && ( i < 8 ))
		IP_Buff_Hex3[i-6] = UART4_ReceiveBuff[i + 4]; 
		if( ( i >= 8 ) && ( i < 12 ))
		Port_Buff_Hex[i-8] = 	UART4_ReceiveBuff[i + 4];
	}
	
	IP_data = strtol(IP_Buff_Hex0,&str,16);           //����λ16������ת��Ϊ10������
	IP_data1 = strtol(IP_Buff_Hex1,&str,16);
	IP_data2 = strtol(IP_Buff_Hex2,&str,16);
	IP_data3 = strtol(IP_Buff_Hex3,&str,16);		
	Port_data = strtol(Port_Buff_Hex,&str,16);	       //��4λ16������ת��Ϊ10����			
	
	sprintf( data_buff, "%d", IP_data);      //������ת��Ϊ�ַ���
	IP_Buff [n++] = 'I';       //���IPʶ��ͷ
	IP_Buff [n++] = 'P';   
	for(i = 0; i < strlen(data_buff); i++)     //д��IP_Buff
		 IP_Buff[n++] = data_buff[i];
	IP_Buff[n++] = '.';
	memset(data_buff, 0, strlen(data_buff));    //���������data_buff
	sprintf( data_buff, "%d", IP_data1);    //������ת��Ϊ�ַ���
	for(i = 0; i < strlen(data_buff); i++)     //д��IP_Buff
		 IP_Buff[n++] = data_buff[i];
	IP_Buff[n++] = '.';
	memset(data_buff, 0, strlen(data_buff));    //���������data_buff
	sprintf( data_buff, "%d", IP_data2);    //������ת��Ϊ�ַ���
	for(i = 0; i < strlen(data_buff); i++)     //д��IP_Buff
		 IP_Buff[n++] = data_buff[i];
	IP_Buff[n++] = '.';
	memset(data_buff, 0, strlen(data_buff));    //���������data_buff
	sprintf( data_buff, "%d", IP_data3);    //������ת��Ϊ�ַ���
	for(i = 0; i < strlen(data_buff); i++)     //д��IP_Buff
		 IP_Buff[n++] = data_buff[i];		
		 
	n = 0;
	Port_Buff [n++] = 'P';       //���IPʶ��ͷ
	Port_Buff [n++] = 'o';   
	Port_Buff [n++] = 'r';    
	Port_Buff [n++] = 't';  
	memset(data_buff, 0, strlen(data_buff));    //���������data_buff  
	sprintf( data_buff, "%d", Port_data);      //������ת��Ϊ�ַ���
	for(i = 0; i < strlen(data_buff); i++)     //д��Port_Buff
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