/*_________________________ Variable ________________________________________*/
/*H**************************************************************************
* NAME:    display.h
*----------------------------------------------------------------------------*/
//
#ifndef _display_h
#define _display_h
/*_____ I N C L U D E S ____________________________________________________*/
//
#include "data_collection.h"
/*_____ M A C R O S ________________________________________________________*/
//

/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Operate_7219(u8_t h,u8_t l);		//操作7219的子程序
void Display(void);
void Init_7219(void);				//初始化7219
void Init_Display_Data(void);
void Display_Stop(void);
void Send_ErrorInf_Out(void);
void Self_Test(void);
//------
void Display_MainPro(void);		//当车和地面设备均启动时调用
void Display_ErrInfo(void);		//当停车且设备关闭后，显示故障信息
void Dis_SensorVal(void);
void Dis_ControlInf(void);


//变量定义
extern u8_t m_cDisplayBuffer[4];
//
//extern u8_t m_cP377;
 //------------------------
//extern u8_t m_bNewDisData;			//刷新显示数据
//extern u8_t m_bDisFlag;
 //
//extern u8_t m_cDisErrCnt;				//显示错误信息的标示循环标记
extern u8_t m_cDisDataDelay;			//显示数据延时
extern u8_t m_cSysError;			//用不同的位表示系统故障



#endif



