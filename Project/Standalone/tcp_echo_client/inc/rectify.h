/*H**************************************************************************
* NAME:    rectify.h
*----------------------------------------------------------------------------*/
//
#ifndef _rectify_h
#define _rectify_h

#include "data_collection.h"

/*_____ M A C R O S ________________________________________________________*/
//��������������ز���xyw
#define WHEEL_MID_VALUE   			240					/*�����̻��е��������ֵ*/
#define WHEEL_ERR					8					/*������λ�õ��������ֵ*/  //�����̴���������������ֵ����Ϊ1000��Ч
#define WHEEL_STEP					20					/*������λ�õĽ��ݱ仯ֵ*/
#define WHEEL_FRONT1				WHEEL_MID_VALUE+WHEEL_STEP
#define WHEEL_FRONT2				WHEEL_MID_VALUE+WHEEL_STEP*2
#define WHEEL_FRONT3				WHEEL_MID_VALUE+WHEEL_STEP*3
#define WHEEL_FRONT4				WHEEL_MID_VALUE+WHEEL_STEP*4
#define WHEEL_BACK1					WHEEL_MID_VALUE-WHEEL_STEP
#define WHEEL_BACK2					WHEEL_MID_VALUE-WHEEL_STEP*2
#define WHEEL_BACK3					WHEEL_MID_VALUE-WHEEL_STEP*3
#define WHEEL_BACK4					WHEEL_MID_VALUE-WHEEL_STEP*4						
//�Ƹ�ǰ��ļ��޾��룬�����򱨾�
#define WHEEL_MIN_BACK_VALUE		WHEEL_MID_VALUE-140	/*��������̣����������Բ�õ���С����*/
#define WHEEL_MAX_FRONT_VALUE		WHEEL_MID_VALUE+140	/*���Ҵ����̣����������Բ�õ�������ֵ */

///////////////////////////////////H������/////////////////////////////////////////////////////////
#define WHEEL_STEP_H				40					/*������λ�õĽ��ݱ仯ֵ*/ //mdf
#define WHEEL_FRONT1_H				WHEEL_MID_VALUE+WHEEL_STEP_H		 //mdf H-40
#define WHEEL_FRONT2_H				WHEEL_MID_VALUE+WHEEL_STEP_H*2	 //mdf	   H-80
#define WHEEL_FRONT3_H				WHEEL_MID_VALUE+WHEEL_STEP_H*3	 //mdf	   H-120
//#define WHEEL_FRONT4				WHEEL_MID_VALUE+WHEEL_STEP*4
#define WHEEL_BACK1_H				WHEEL_MID_VALUE-WHEEL_STEP_H		 //mdf
#define WHEEL_BACK2_H				WHEEL_MID_VALUE-WHEEL_STEP_H*2	 //mdf
#define WHEEL_BACK3_H				WHEEL_MID_VALUE-WHEEL_STEP_H*3	 //mdf
//#define WHEEL_BACK4				WHEEL_MID_VALUE-WHEEL_STEP*4	 //mdf													
//--------------------------------------------------------------------
#define SCANNER_CENTER_LINE 500
#define CENTER_ERR			20
//���²���ʹ�ú꣬��Ϊ��data_collection.c���趨__YZ_20160324
#define LEFT_MAX_VALUE0  SCANNER_CENTER_LINE-50		/* ɨ�賵������߼��޾���  */
#define RIGHT_MIN_VALUE0  SCANNER_CENTER_LINE+50	/* ɨ�賵�����ұ߼��޾���  */
#define LEFT_MAX_VALUE1  SCANNER_CENTER_LINE-50		/* ɨ�賵ǰ����߼��޾���70  */
#define RIGHT_MIN_VALUE1  SCANNER_CENTER_LINE+50	/* ɨ�賵ǰ���ұ߼��޾���70  */
//------------------------
//ɨ�賵���ҵļ��޾��룬�����򱨾�____ȡֵ������������_YZ_20160324
#define LEFT_EXTREME_VALUE 500
#define RIGHT_EXTREME_VALUE 1800
//������̬
#define STATE_1 	1  	//F1,B1
#define STATE_2 	2	//F2,B2
#define STATE_3 	3	//F3,B3
#define STATE_4 	4	//F4,B4
#define STATE_5 	5	//F5,B5
/*
//       /|\
//      / | \            O �ο���4
//        |
//				|
//	   /  |    \
//	 1/  	|     \2       O �ο���3
//	 /		|      \
//				|
//				|
//				|                O �ο���2
//				|
//	 \ 		|       /
//	 3\		|      /4       O  �ο���1 
//	   \	|     /
//				|
//				|
*/

/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Analyse_RectifyData(void);
void Init_Rectify_Data(void);
void Rectify(void);
void Drive_Judge(void);
//void Stop_Judge();
void Check_Wheel_Lock(void);

extern u8_t m_bWheelOpposite;


#endif
