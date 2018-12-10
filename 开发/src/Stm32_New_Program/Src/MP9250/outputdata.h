#ifndef _outputdata_H
#define _outputdata_H

//#include "common.h"

extern float OutData[4];
void OutPut_Data(void);

extern long  int CheckSum_OutData[4];
void OutPut_CheckSumData(void);
void convertStrToUnChar(char* str, unsigned char* UnChar);
#endif 
