/*H**************************************************************************
* NAME:    rectify.h
*----------------------------------------------------------------------------*/
//
#ifndef _rectify_h
#define _rectify_h

#include "data_collection.h"

/*_____ M A C R O S ________________________________________________________*/
//环保车传感器相关参数xyw
#define WHEEL_MID_VALUE   			240					/*方向盘回中的理想距离值*/
#define WHEEL_ERR					8					/*方向盘位置的允许误差值*/  //方向盘传感器防抖处理阈值，改为1000无效
#define WHEEL_STEP					20					/*方向盘位置的阶梯变化值*/
#define WHEEL_FRONT1				WHEEL_MID_VALUE+WHEEL_STEP
#define WHEEL_FRONT2				WHEEL_MID_VALUE+WHEEL_STEP*2
#define WHEEL_FRONT3				WHEEL_MID_VALUE+WHEEL_STEP*3
#define WHEEL_FRONT4				WHEEL_MID_VALUE+WHEEL_STEP*4
#define WHEEL_BACK1					WHEEL_MID_VALUE-WHEEL_STEP
#define WHEEL_BACK2					WHEEL_MID_VALUE-WHEEL_STEP*2
#define WHEEL_BACK3					WHEEL_MID_VALUE-WHEEL_STEP*3
#define WHEEL_BACK4					WHEEL_MID_VALUE-WHEEL_STEP*4						
//推杆前后的极限距离，超过则报警
#define WHEEL_MIN_BACK_VALUE		WHEEL_MID_VALUE-140	/*向左打方向盘，传感器可以测得的最小距离*/
#define WHEEL_MAX_FRONT_VALUE		WHEEL_MID_VALUE+140	/*向右打方向盘，传感器可以测得的最大距离值 */

///////////////////////////////////H车参数/////////////////////////////////////////////////////////
#define WHEEL_STEP_H				40					/*方向盘位置的阶梯变化值*/ //mdf
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
//以下不再使用宏，改为在data_collection.c中设定__YZ_20160324
#define LEFT_MAX_VALUE0  SCANNER_CENTER_LINE-50		/* 扫描车后轮左边极限距离  */
#define RIGHT_MIN_VALUE0  SCANNER_CENTER_LINE+50	/* 扫描车后轮右边极限距离  */
#define LEFT_MAX_VALUE1  SCANNER_CENTER_LINE-50		/* 扫描车前轮左边极限距离70  */
#define RIGHT_MIN_VALUE1  SCANNER_CENTER_LINE+50	/* 扫描车前轮右边极限距离70  */
//------------------------
//扫描车左右的极限距离，超过则报警____取值待定！！！！_YZ_20160324
#define LEFT_EXTREME_VALUE 500
#define RIGHT_EXTREME_VALUE 1800
//车身姿态
#define STATE_1 	1  	//F1,B1
#define STATE_2 	2	//F2,B2
#define STATE_3 	3	//F3,B3
#define STATE_4 	4	//F4,B4
#define STATE_5 	5	//F5,B5
/*
//       /|\
//      / | \            O 参考点4
//        |
//				|
//	   /  |    \
//	 1/  	|     \2       O 参考点3
//	 /		|      \
//				|
//				|
//				|                O 参考点2
//				|
//	 \ 		|       /
//	 3\		|      /4       O  参考点1 
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
