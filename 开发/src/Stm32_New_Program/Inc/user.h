#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "bmp280.h"
#include "mp9250.h"
#include "usart.h"



void USART3_CLR_RecvBuf(void);
void UART4_CLR_RecvBuf(void);
void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen);
void USART3_Ack_Interaction(void);





