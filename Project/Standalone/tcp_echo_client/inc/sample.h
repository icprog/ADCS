/***************************************************************************
* NAME:    Sample.h
*----------------------------------------------------------------------------*/
//
#ifndef _Sample_h
#define _Sample_h

/*_____ I N C L U D E S ____________________________________________________*/
//
#include "sys.h"


/*_____ M A C R O S ________________________________________________________*/
//
/*_____ D E F I N I T I O N ________________________________________________*/
     
//AD采集管脚定义     
#define ADCS0 0x7f			/*** CS_--PC.7 ***/
#define ADCS1 0x80
#define ADSCK0 0xbf			/*** SCK--PC.6 ***/
#define ADSCK1 0x40
#define ADOUT0 0xdf			/*** DOUT-PC.5 ***/
#define ADOUT1 0x20
#define ADDINBIT 0x80		/*** DIN--PB.7 ***/
//-----------------------
#define SENSOR_DISTANCE 		500	/*前后两个传感器的距离，单位cm*/
#define SENSOR_WHEEL_FRONT	300	/*前传感器距离前车轮的距离，车轮考前，传感器靠后，单位cm*/
#define SENSOR_WHEEL_BACK	40	/*后传感器距离后车轮的距离，车轮靠后，传感器靠前，单位cm*/
#define SENSOR_DIFF				5   /*前、后两个采集的数差值变化的最小差值，单位mm */
#define SENSOR_ERR				10  /*前、后两个采集的数差值变化出现突变的阈值，单位mm */
#define adval_Stack_Size  5 
//////////////////////////////////////////////////////////////////////////////
/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Sample_AD(void);		//xyw
void Sample_Data(void);
void Analyse_Data(void);					//分析数据
void Init_Sample_Data(void);
#endif
