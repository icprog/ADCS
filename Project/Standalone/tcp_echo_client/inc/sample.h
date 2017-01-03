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
     
//AD�ɼ��ܽŶ���     
#define ADCS0 0x7f			/*** CS_--PC.7 ***/
#define ADCS1 0x80
#define ADSCK0 0xbf			/*** SCK--PC.6 ***/
#define ADSCK1 0x40
#define ADOUT0 0xdf			/*** DOUT-PC.5 ***/
#define ADOUT1 0x20
#define ADDINBIT 0x80		/*** DIN--PB.7 ***/
//-----------------------
#define SENSOR_DISTANCE 		500	/*ǰ�������������ľ��룬��λcm*/
#define SENSOR_WHEEL_FRONT	300	/*ǰ����������ǰ���ֵľ��룬���ֿ�ǰ�����������󣬵�λcm*/
#define SENSOR_WHEEL_BACK	40	/*�󴫸���������ֵľ��룬���ֿ��󣬴�������ǰ����λcm*/
#define SENSOR_DIFF				5   /*ǰ���������ɼ�������ֵ�仯����С��ֵ����λmm */
#define SENSOR_ERR				10  /*ǰ���������ɼ�������ֵ�仯����ͻ�����ֵ����λmm */
#define adval_Stack_Size  5 
//////////////////////////////////////////////////////////////////////////////
/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Sample_AD(void);		//xyw
void Sample_Data(void);
void Analyse_Data(void);					//��������
void Init_Sample_Data(void);
#endif
