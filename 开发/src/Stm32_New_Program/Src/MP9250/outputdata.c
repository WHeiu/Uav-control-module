#include "outputdata.h"
#include  <stm32l4xx_hal.h>
#include "usart.h"
#include <stdint.h>
#include <string.h>
//#include "uart.h"
//#include "arm_math.h"
extern char data[10];
float OutData[4] = { 0 };
long  int CheckSum_OutData[4] = {0};

//void senddata(unsigned short data_buf)
//{
//while (!(IFG2&UCA0TXIFG));
//	UCA0TXBUF = data_buf;

//}

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

void convertStrToUnChar(char* str, unsigned char* UnChar)
{
	int i = strlen(str), j = 0, counter = 0;
	char c[2];
	unsigned int bytes[2];
	for (j = 0; j < i; j += 2) 
	{
		if(0 == j % 2)
		{
			c[0] = str[j];
			c[1] = str[j + 1];
			sscanf(c, "%02x" , &bytes[0]);
			UnChar[counter] = bytes[0];
			counter++;
		}
	}
	return;
}
//void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen)

//{
//	int i = 0;
//	for(i = 0; i < ucLen; i++)
//	{
//		//格式化输str,每unsigned char 转换字符占两位置%x写输%X写输
//		sprintf(str + i * 2, "%02x", UnChar[i]);
//	}

//}
//void OutPut_Data(void)
//{
//    int temp[4] = {0};
//    unsigned int temp1[4] = {0};
//    uint8_t databuf[10] = {0};
//    unsigned char i;
//    unsigned short CRC16 = 0;
//    for(i=0;i<4;i++)
//     {
//      temp[i]  = (int)OutData[i];
//      temp1[i] = (unsigned int)temp[i];  
//     }
//     
//    for(i=0;i<4;i++) 
//    {
//      databuf[i*2]   = (unsigned char)(temp1[i]%256);
//      databuf[i*2+1] = (unsigned char)(temp1[i]/256);
//    }
//    
//    CRC16 = CRC_CHECK(databuf,8);
//    databuf[8] = CRC16%256;
//    databuf[9] = CRC16/256;
//  
////  for(i=0;i<10;i++)
////		data[i] = (char)databuf[i];
//  //  uart_putchar(UART1,databuf[i]);
//  //senddata(databuf[i]);
//		convertUnCharToStr(data, databuf, 10); 

//}


//void OutPut_CheckSumData(void)
//{
//	unsigned short CRC_Tmp,i;
//        unsigned char databuf[9] = {0};
//        int temp[4] = {0};
//        unsigned int temp1[4] = {0};

//	
//        for(i=0;i<4;i++)
//        {
//            temp[i]  = (int)CheckSum_OutData[i];
//            temp1[i] = (unsigned int)temp[i];
//        }
//        for(i=0;i<4;i++) 
//        {
//            databuf[i*2]   = (unsigned char)(temp1[i]%256);
//            databuf[i*2+1] = (unsigned char)(temp1[i]/256);
//        }
//        
//	CRC_Tmp = 0;
//	for(i = 0; i < 8; i++) CRC_Tmp += databuf[i];
//	databuf[8] = CRC_Tmp ;
//        
//        for(i=0;i<9;i++)    senddata(databuf[i]);
//}
