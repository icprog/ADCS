//==============================================
//*************  Execute.c  ***********************
//==============================================
#include "Execute.h"
#include "display.h"
#include "rectify.h"
#include "data_collection.h"
#include "stm32f4_GPIO_int.h"
#include "GLCD.h"
#include "ADC_YZ_head.h"
#include "stm324xg_eval.h"
/*_________________________ Variable ________________________________________*/
//变量定义
u8_t m_bOutPush;					//前推推杆标记，用于复位
u8_t m_bOutDraw;					//
//u8_t m_bWheelOpposite;			//反向打轮的标记位，TRUE 正在反向打轮，FALSE 没有反向打轮
u8_t m_bMoveFront;				//推推杆，右打轮标记
u8_t m_bMoveBack;				//拉推杆，左打轮标记

u8_t m_bVehicleHighSpeed;		//行车高速速度标记，TRUE是高速，FALSE是低速
//u8_t m_bTransferWorkMode;		//环保车工作模式使能--放在main.c中
//u8_t m_bRightRudder;			    //右舵模式--放在main.c中


//*****GLCD index******
#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index  6x8                    */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index 16x24                   */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif



//----------------------
//转动推杆的方向，0－推，1－拉
//----------------------------------------------------------------------
//偏移调节
u32_t m_nWheelPostion;			//推杆所处的位置
u8_t m_cVehicleAngle;			//车身的偏差程度指示
u32_t m_nWheelExtremeCount;		//推杆超限计数次数
u8_t m_cWheelMidErrCnt;		//推杆回中错误检测计数
extern u8_t m_cVehicleState;	//车所处的姿态
///////////////////////////////////////////////
extern u8_t m_bOK;			    //车没有在限内行走标志位		
extern u8_t WHEEL_CHANCE;	    //H车环保车调节力度数
extern u8_t m_cSysError;		//用不同的位表示系统故障

extern u8_t m_bSubSymbol;
extern u8_t m_bWheelLock;
extern u32_t m_nSubValue;
extern u32_t m_nDist[3];    //m_nDist[0:1]在二代纠偏储存车头车位垂直距离(这里已不再使用)，m_nDist[2]储存方向盘推杆距离__YZ__2015/11/23

extern double SENSOR_DISTANCE_FRONT;//车头距基准线距离                         ;
extern double SENSOR_DISTANCE_BACK;//车尾距基准线距离                         ;
extern u8_t m_bWheelDataOk;
extern u8_t m_bBackMidWheel;
extern u32_t m_nDiffValue;
extern u8_t m_bCenterDiff;
extern u8_t m_bNewData;

extern u32_t m_nFrontExtremeCount;		//轮超限次数
extern u32_t m_nFrontExtremeCount1;	    //初始走车轮超限次数
extern u32_t m_nBackExtremeCount;		//传感器初始化时超限次数计数

extern u8_t m_bVehicleForward;		    //行车前进方向标记，TRUE正在前进，FALSE没有前进
extern u8_t m_bVehicleBackward;			//行车后退方向标记，TRUE正在后退，FALSE没有后退
extern u32_t m_nWheelPosObj;			//推杆目标位置
extern u8_t m_bTransferWorkMode;		//环保车工作模式使能--放在main.c中
extern u8_t m_bRightRudder;			    //右舵模式--放在main.c中
extern u8_t m_bWheelOpposite;
extern u8_t LCD_Vehicle_Trail; //LCD是否显示车辆轨迹图参量
extern u8_t LCD_SENSOR_Image;  //LCD是否显示传感器扫描图像参量
extern u8_t LCD_Vehicle_Posture; //LCD是否显示车身姿态图像参量
extern void USART_Send(uint16_t Data);
/////////////////////////////////////////////////////////////////////////
/*************************************************************************
******	根据当前车辆的位置和车身状态，判断车辆处于调控的具体阶段	******
******	不同阶段需要按照不同的控制方法对车辆的姿态进行控制和调整	******
******	当车辆位置和车身姿态无限逼近基准线时，控制方向盘回正		******
**************************************************************************/
void Adjust_Deflect()
{
	m_bVehicleHighSpeed=FALSE;  //调试使用低速车
	if(m_bBackMidWheel==TRUE)//如果方向盘正在回中，暂停方向盘控制
		return;
	if(m_nSubValue>DIFF_VALUE4)	//车身偏差角度
		m_cVehicleAngle=4;
	else if(m_nSubValue>DIFF_VALUE3)
		m_cVehicleAngle=3;
	else if(m_nSubValue>DIFF_VALUE2)
		m_cVehicleAngle=2;
	else if(m_nSubValue>DIFF_VALUE1)
		m_cVehicleAngle=1;
	else
		m_cVehicleAngle=0;
	//车身偏离基准线的值
	if(m_bTransferWorkMode==TRUE)			   //环保车工作模式
	{
		switch (m_cVehicleState)
		{
		case STATE_1:
			//根据车身偏差角度，逐步调整的方式，进行正向和反向交替打轮，使车能够逐步的向基准线靠拢，不会出现车身偏差很大的情况
			if(m_bWheelOpposite==FALSE)	//没有反向打轮
			{	
				if(m_bVehicleHighSpeed==TRUE)
				{
					m_nWheelPosObj=WHEEL_FRONT1;
					if(m_cVehicleAngle>=1)
						m_bWheelOpposite=TRUE;
				}
				else
				{
					m_nWheelPosObj=WHEEL_FRONT2;
					if(m_cVehicleAngle>=2)//调节此处的偏差值，以便使车以一个较小的角度靠近state5//pl
						m_bWheelOpposite=TRUE;
				}
			}
			//
			if(m_bWheelOpposite==TRUE) //正在反向打轮
			{
				if(m_cVehicleAngle>=4)
				{
					if(m_bVehicleHighSpeed==TRUE)
						m_nWheelPosObj=WHEEL_BACK1;
					else
						m_nWheelPosObj=WHEEL_BACK2;//pl
				}
				else
					m_nWheelPosObj=WHEEL_MID_VALUE;
			}
			break;
		case STATE_2:
			//根据车身偏差角度，逐步调整的方式，进行正向和反向交替打轮，使车能够逐步的向基准线靠拢，不会出现车身偏差很大的情况
			if(m_bWheelOpposite==FALSE)	//没有反向打轮
			{	
				if(m_bVehicleHighSpeed==TRUE)
				{
					m_nWheelPosObj=WHEEL_BACK1;
					if(m_cVehicleAngle>=1)
						m_bWheelOpposite=TRUE;
				}
				else
				{
					m_nWheelPosObj=WHEEL_BACK1;
					if(m_cVehicleAngle>=2)//调节此处的偏差值，以便使车以一个较小的角度靠近state5//pl
						m_bWheelOpposite=TRUE;
				 }
			}		
			//
			if(m_bWheelOpposite==TRUE) //正在反向打轮
			{
				if(m_cVehicleAngle>=4)
				{
					if(m_bVehicleHighSpeed==TRUE)
						m_nWheelPosObj=WHEEL_FRONT1;
					else
						m_nWheelPosObj=WHEEL_FRONT2;//pl
				}
				else
					m_nWheelPosObj=WHEEL_MID_VALUE;
			}
			break;
		case STATE_3:
			if(m_bVehicleHighSpeed==TRUE)
				m_nWheelPosObj=WHEEL_FRONT2;
			else
				m_nWheelPosObj=WHEEL_FRONT3;
			break;
		case STATE_4:
			if(m_bVehicleHighSpeed==TRUE)
				m_nWheelPosObj=WHEEL_BACK2;
			else
				m_nWheelPosObj=WHEEL_BACK3;
			break;
		case STATE_5:			
			if(m_bWheelOpposite==TRUE)//从状态1、2进入5时把反向打轮标记清掉
				m_bWheelOpposite=FALSE;			
			if(m_bVehicleForward==TRUE)//前进
			{
				if(m_cVehicleAngle>=3)//如果车身偏差角度过大，先进行调直控制			
				{
					if(m_bVehicleHighSpeed==TRUE)
						m_cVehicleAngle=1;
					else
						m_cVehicleAngle=2;//pl
					if(m_bSubSymbol==TRUE)
						m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP;
					else
						m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP;
				}
				else
				{
					if(m_nDiffValue>CENTER_ERR)//================调节此处CENTER_ERR，使车轮调节幅度变小
					{
						if(m_bCenterDiff==TRUE)//靠右-远离
						{	
							if(m_bSubSymbol==TRUE)
								m_nWheelPosObj=WHEEL_MID_VALUE;	
							else
								m_nWheelPosObj=WHEEL_BACK1;
						}
						else					//靠左-靠近
						{
							if(m_bSubSymbol==TRUE)
								m_nWheelPosObj=WHEEL_FRONT1;		
							else
								m_nWheelPosObj=WHEEL_MID_VALUE;
						}
					}
					else
						m_nWheelPosObj=WHEEL_MID_VALUE;
				}
			}
			else if(m_bVehicleBackward==TRUE)//后退
			{
				if(m_cVehicleAngle>=2)
				{
					if(m_bVehicleHighSpeed==TRUE)
						m_cVehicleAngle=1;
					else
						m_cVehicleAngle=2; //pl
					if(m_bSubSymbol==TRUE)
						m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP;
					else
						m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP;				
				}
				else
				{
					if(m_nDiffValue>(CENTER_ERR/2))//================调节此处CENTER_ERR，使车轮调节幅度变小
					{	
						if(m_bCenterDiff==TRUE)//靠右-远离
						{	
							if(m_bSubSymbol==TRUE)
							{	
								if(m_bVehicleHighSpeed==TRUE)
									m_nWheelPosObj=WHEEL_BACK1;	
								else
									m_nWheelPosObj=WHEEL_BACK2;//pl
							}
							else
								m_nWheelPosObj=WHEEL_MID_VALUE;
						}
						else					//靠左-靠近
						{
							if(m_bSubSymbol==TRUE)
								m_nWheelPosObj=WHEEL_MID_VALUE;		
							else
							{
								if(m_bVehicleHighSpeed==TRUE)
									m_nWheelPosObj=WHEEL_FRONT1;
								else
									m_nWheelPosObj=WHEEL_FRONT2;
							}
						}
					}
					else
						m_nWheelPosObj=WHEEL_MID_VALUE;
				}
			}
			break;
		 default:
		 	break;
		}
	}
/////////////////////////////////////////////////////////H车//////////////////////////////////////////////////////
//////////////////////////////////////////////////////右舵车//////////////////////////////////////////////////////
	else
	{
		if(m_bRightRudder == TRUE)						   //右舵车
		{	   
		 switch (m_cVehicleState)
			{
			case STATE_1:
//									if(LCD_Interface1==FALSE && LCD_Interface2==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_1");
////#endif // __USE_LCD
//         	}
				//根据车身偏差角度，逐步调整的方式，进行正向和反向交替打轮，使车能够逐步的向基准线靠拢，不会出现车身偏差很大的情况
				if(m_bWheelOpposite==FALSE)	//没有反向打轮
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{
						m_nWheelPosObj=WHEEL_BACK3_H;		//7日更改
						if(m_cVehicleAngle>=2)			
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_BACK3_H;	
						if(m_cVehicleAngle>=2)//        mdf调节此处的偏差值，以便使车以一个较小的角度靠近state5//pl
							m_bWheelOpposite=TRUE;
					}
				}
				//
				if(m_bWheelOpposite==TRUE) //正在反向打轮
				{
					if(m_cVehicleAngle>=4)
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_nWheelPosObj=WHEEL_FRONT2_H;		  
						else
							m_nWheelPosObj=WHEEL_FRONT3_H;//pl	  
					}
					else
						m_nWheelPosObj=WHEEL_BACK2_H;		      //mdf  m_nWheelPosObj=WHEEL_MID_VALUE;	  
				}
				break;
			case STATE_2:
//					if(LCD_Interface1==FALSE && LCD_Interface2==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_2");
////#endif // __USE_LCD
//         	}
				//根据车身偏差角度，逐步调整的方式，进行正向和反向交替打轮，使车能够逐步的向基准线靠拢，不会出现车身偏差很大的情况
				if(m_bWheelOpposite==FALSE)	//没有反向打轮
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{										  
						m_nWheelPosObj=WHEEL_FRONT3_H;      //7日更改
						if(m_cVehicleAngle>=2)
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_FRONT3_H;		 
						if(m_cVehicleAngle>=2)           //  mdf 调节此处的偏差值，以便使车以一个较小的角度靠近state5//pl
							m_bWheelOpposite=TRUE;
					 }
				}		
				//
				if(m_bWheelOpposite==TRUE) //正在反向打轮
				{
					if(m_cVehicleAngle>=4)			 
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_nWheelPosObj=WHEEL_BACK2_H;	   
						else
							m_nWheelPosObj=WHEEL_BACK3_H;//pl  
					}
					else
						m_nWheelPosObj=WHEEL_FRONT2_H;	//mdf	m_nWheelPosObj=WHEEL_MID_VALUE; 	 yigai
				}
				break;
			case STATE_3:
//									if(LCD_Interface1==FALSE && LCD_Interface2==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_3");
////#endif // __USE_LCD
//         	}
				if(m_bVehicleHighSpeed==TRUE)
					m_nWheelPosObj=WHEEL_BACK3_H;
				else
					m_nWheelPosObj=WHEEL_BACK3_H;  
				break;
			case STATE_4:
//									if(LCD_Interface1==FALSE && LCD_Interface2==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_4");
////#endif // __USE_LCD
//         	}
				if(m_bVehicleHighSpeed==TRUE)
					m_nWheelPosObj=WHEEL_FRONT3_H;
				else
					m_nWheelPosObj=WHEEL_FRONT3_H;	 
				break;
			case STATE_5:	
//					if(LCD_Interface1==FALSE && LCD_Interface2==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_5");
////#endif // __USE_LCD
//         	}				
				if(m_bWheelOpposite==TRUE)//从状态1、2进入5时把反向打轮标记清掉
					m_bWheelOpposite=FALSE;			
				if(m_bVehicleForward==TRUE)//前进
				{
					if(m_cVehicleAngle>=2)//如果车身偏差角度过大，先进行调直控制			
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_cVehicleAngle=2;	 //md
						else
							m_cVehicleAngle=3;//pl
						if(m_bSubSymbol==TRUE)
							m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP_H;   
						else
							m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP_H;  
					}
					else
					{
						if(m_nDiffValue>CENTER_ERR)  //================调节此处CENTER_ERR，使车轮调节幅度变小	 mdf
						{
							if(m_bCenterDiff==TRUE)//靠右-远离
							{	
									if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_MID_VALUE;	  //mdf m_nWheelPosObj=WHEEL_MID_VALUE;
								else
									m_nWheelPosObj=WHEEL_FRONT3_H;	 //mdf
							}
							else					//靠左-靠近
							{
								if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_BACK3_H;	//mdf	
								else
									m_nWheelPosObj=WHEEL_MID_VALUE;	 //mdf m_nWheelPosObj=WHEEL_MID_VALUE;
							}
						}
						else
							m_nWheelPosObj=WHEEL_MID_VALUE;
					}
				}
				else if(m_bVehicleBackward==TRUE)//后退
				{
					if(m_cVehicleAngle>=2)
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_cVehicleAngle=3;		   //可调整
						else
							m_cVehicleAngle=3; //pl
						if(m_bSubSymbol==TRUE)
							m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP_H;  //mdf
						else
							m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP_H;   //mdf				
					}
					else
					{
						if(m_nDiffValue>(CENTER_ERR/2))//================调节此处CENTER_ERR，使车轮调节幅度变小	mdf
						{	
							if(m_bCenterDiff==TRUE)//靠右-远离
							{	
								if(m_bSubSymbol==TRUE)
								{	
									if(m_bVehicleHighSpeed==TRUE)
										m_nWheelPosObj=WHEEL_FRONT2_H;	 //调整处
									else
										m_nWheelPosObj=WHEEL_FRONT3_H;//pl  
								}
								else
									m_nWheelPosObj=WHEEL_MID_VALUE;	 //mdf	 m_nWheelPosObj=WHEEL_MID_VALUE;
							}
							else					//靠左-靠近
							{
								if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_MID_VALUE;	//mdf  m_nWheelPosObj=WHEEL_MID_VALUE;	
								else
								{
									if(m_bVehicleHighSpeed==TRUE)
										m_nWheelPosObj=WHEEL_BACK2_H;
									else
										m_nWheelPosObj=WHEEL_BACK3_H;  //mdf
								}
							}
						}
						else
							m_nWheelPosObj=WHEEL_MID_VALUE;
					}
				}
				break;
			 default:
			 	break;
			}
		}

/////////////////////////////左舵车/////////////////////////////////////////////
		else
		{
			switch (m_cVehicleState)
			{
			case STATE_1:
//													if(LCD_Interface1==FALSE && LCD_Interface2==TRUE && LCD_Vehicle_Posture==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_1");
////#endif // __USE_LCD
//         	}
		//	printf("state1");
				//根据车身偏差角度，逐步调整的方式，进行正向和反向交替打轮，使车能够逐步的向基准线靠拢，不会出现车身偏差很大的情况
				if(m_bWheelOpposite==FALSE)	//没有反向打轮
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{
						m_nWheelPosObj=WHEEL_FRONT3_H;		//7日更改
						if(m_cVehicleAngle>=2)			
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_FRONT3_H;	
						if(m_cVehicleAngle>=2)             // mdf调节此处的偏差值，以便使车以一个较小的角度靠近state5//pl
							m_bWheelOpposite=TRUE;
					}
				}
				//
				if(m_bWheelOpposite==TRUE) //正在反向打轮
				{
					if(m_cVehicleAngle>=4)
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_nWheelPosObj=WHEEL_BACK2_H;		  
						else
							m_nWheelPosObj=WHEEL_BACK3_H;//pl	  
					}
					else
						m_nWheelPosObj=WHEEL_FRONT2_H;		      //mdf  m_nWheelPosObj=WHEEL_MID_VALUE;	  
				}
				break;
			case STATE_2:
		//		printf("state2");
//													if(LCD_Interface1==FALSE && LCD_Interface2==TRUE&& LCD_Vehicle_Posture==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_2");
////#endif // __USE_LCD
//         	}
				//根据车身偏差角度，逐步调整的方式，进行正向和反向交替打轮，使车能够逐步的向基准线靠拢，不会出现车身偏差很大的情况
				if(m_bWheelOpposite==FALSE)	//没有反向打轮
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{										  
						m_nWheelPosObj=WHEEL_BACK3_H;      //7日更改
						if(m_cVehicleAngle>=2)
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_BACK3_H;		 
						if(m_cVehicleAngle>=2)           //  mdf 调节此处的偏差值，以便使车以一个较小的角度靠近state5//pl
							m_bWheelOpposite=TRUE;
					 }
				}		
				//
				if(m_bWheelOpposite==TRUE) //正在反向打轮
				{
					if(m_cVehicleAngle>=4)			 
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_nWheelPosObj=WHEEL_FRONT2_H;	   
						else
							m_nWheelPosObj=WHEEL_FRONT3_H;//pl  
					}
					else
						m_nWheelPosObj=WHEEL_BACK2_H;	//mdf	m_nWheelPosObj=WHEEL_MID_VALUE; 	 yigai
				}
				break;
			case STATE_3:
	//			printf("state3");
//													if(LCD_Interface1==FALSE && LCD_Interface2==TRUE && LCD_Vehicle_Posture==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_3");
////#endif // __USE_LCD
//         	}
				if(m_bVehicleHighSpeed==TRUE)
					m_nWheelPosObj=WHEEL_FRONT3_H;
				else
					m_nWheelPosObj=WHEEL_FRONT3_H;  
				break;
			case STATE_4:
			//	printf("state4");
//													if(LCD_Interface1==FALSE && LCD_Interface2==TRUE && LCD_Vehicle_Posture==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_4");
////#endif // __USE_LCD
//         	}
				if(m_bVehicleHighSpeed==TRUE)
					m_nWheelPosObj=WHEEL_BACK3_H;
				else
					m_nWheelPosObj=WHEEL_BACK3_H;	 
				break;
			case STATE_5:	
				printf("state5");
//									if(LCD_Interface1==FALSE && LCD_Interface2==TRUE && LCD_Vehicle_Posture==TRUE){
////#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line7, (uint8_t*)"STATE_5");
////#endif // __USE_LCD
//         	}				
				if(m_bWheelOpposite==TRUE)//从状态1、2进入5时把反向打轮标记清掉
					m_bWheelOpposite=FALSE;			
				if(m_bVehicleForward==TRUE)//前进
				{
					if(m_cVehicleAngle>=2)//如果车身偏差角度过大，先进行调直控制			
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_cVehicleAngle=2;	 //md
						else
							m_cVehicleAngle=3;//pl
						if(m_bSubSymbol==TRUE)
							m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP_H;   
						else
							m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP_H;  
					}
					else
					{
						if(m_nDiffValue>CENTER_ERR)  //================调节此处CENTER_ERR，使车轮调节幅度变小	 mdf
						{
							if(m_bCenterDiff==TRUE)//靠右-远离
							{	
									if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_MID_VALUE;	  //mdf m_nWheelPosObj=WHEEL_MID_VALUE;
								else
									m_nWheelPosObj=WHEEL_BACK3_H;	 //mdf
							}
							else					//靠左-靠近
							{
								if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_FRONT3_H;	//mdf	
								else
									m_nWheelPosObj=WHEEL_MID_VALUE;	 //mdf m_nWheelPosObj=WHEEL_MID_VALUE;
							}
						}
						else
							m_nWheelPosObj=WHEEL_MID_VALUE;
					}
				}
				else if(m_bVehicleBackward==TRUE)//后退
				{
					if(m_cVehicleAngle>=2)
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_cVehicleAngle=3;		   //可调整
						else
							m_cVehicleAngle=3; //pl
						if(m_bSubSymbol==TRUE)
							m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP_H;  //mdf
						else
							m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP_H;   //mdf				
					}
					else
					{
						if(m_nDiffValue>(CENTER_ERR/2))//================调节此处CENTER_ERR，使车轮调节幅度变小	mdf
						{	
							if(m_bCenterDiff==TRUE)//靠右-远离
							{	
								if(m_bSubSymbol==TRUE)
								{	
									if(m_bVehicleHighSpeed==TRUE)
										m_nWheelPosObj=WHEEL_BACK2_H;	 //调整处
									else
										m_nWheelPosObj=WHEEL_BACK3_H;//pl  
								}
								else
									m_nWheelPosObj=WHEEL_MID_VALUE;	 //mdf	 m_nWheelPosObj=WHEEL_MID_VALUE;
							}
							else					//靠左-靠近
							{
								if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_MID_VALUE;	//mdf  m_nWheelPosObj=WHEEL_MID_VALUE;	
								else
								{
									if(m_bVehicleHighSpeed==TRUE)
										m_nWheelPosObj=WHEEL_FRONT2_H;
									else
										m_nWheelPosObj=WHEEL_FRONT3_H;  //mdf
								}
							}
						}
						else
							m_nWheelPosObj=WHEEL_MID_VALUE;
					}
				}
				break;
			 default:
			 	break;
			}
		}	
	}
//    USART_Send(m_nWheelPosObj);
//  	USART_Send(0xff);
//	
}			  
//控制方向盘位置，并对到位与否进行实时检测
void Wheel_Pos_Control()
{
	if((m_bWheelLock==FALSE)&&(m_bWheelDataOk==TRUE))//方向盘没有被锁定且方向盘推杆数据已经准备好了，才能推动推杆
	{
		m_bWheelDataOk=FALSE;
		
		if(m_bBackMidWheel==TRUE)//如果正在回中，检测方向盘是否已经完成回中
		{
			if((m_nDist[2] >= (WHEEL_MID_VALUE-WHEEL_ERR)) && (m_nDist[2] <= (WHEEL_MID_VALUE+WHEEL_ERR)))//检测方向盘是否处于中点的位置
				m_bBackMidWheel=FALSE;
		}
		//	
		if(((m_nDist[2] <= (m_nWheelPosObj-WHEEL_ERR))&&(m_nDist[2] >=WHEEL_MIN_BACK_VALUE)))		   //mdf  否则无效数据推杆不动
		{
			Move_Wheel(FRONT_WHEEL);
			m_bMoveFront=TRUE;
			m_bMoveBack=FALSE;
			Control_Mid_Relay(FALSE);
		}
		else if((m_nDist[2] >= (m_nWheelPosObj+WHEEL_ERR) && (m_nDist[2] <=WHEEL_MAX_FRONT_VALUE)))	  //mdf	  无效数据推杆不动
		{
			Move_Wheel(BACK_WHEEL);
			m_bMoveFront=FALSE;
			m_bMoveBack=TRUE;
			Control_Mid_Relay(FALSE);
		}	
		else
		{
			Stop_Wheel();
			m_nWheelPostion=m_nWheelPosObj;
			m_bMoveFront=FALSE;
			m_bMoveBack=FALSE;
			if((m_bVehicleForward==FALSE)&&(m_bVehicleBackward==FALSE))
			{			
				if(m_nWheelPosObj==WHEEL_MID_VALUE)//推杆停止动作，且推杆当前的位置处于中点时，推杆回中继电器输出闭合
				{	
					Control_Mid_Relay(TRUE);
					m_cSysError &= ~SYSERR_MID_WHEEL;
				}
				else	//回中错误
				{
					m_cWheelMidErrCnt++;
					if(m_cWheelMidErrCnt>EXTREME_VALUE1)//PL:20130512
					{
						m_cWheelMidErrCnt=0;
						Control_Mid_Relay(FALSE);
						m_cSysError |= SYSERR_MID_WHEEL;
					}
				}
			}
		}
	}
	//推杆移动范围超限故障判断
	if(m_nDist[2] < WHEEL_MIN_BACK_VALUE)
	{
		m_nWheelExtremeCount++;
		if(m_nWheelExtremeCount>EXTREME_VALUE1)
		{
			m_nWheelExtremeCount=0;
			m_cSysError |= SYSERR_DRAW_EXTREME;			//拉超限
			m_cSysError &= ~SYSERR_PUSH_EXTREME;		
	   	}
	}
	else if(m_nDist[2] > WHEEL_MAX_FRONT_VALUE)
	{
		m_nWheelExtremeCount++;
		if(m_nWheelExtremeCount>EXTREME_VALUE1)
		{
			m_nWheelExtremeCount=0;
			m_cSysError |= SYSERR_PUSH_EXTREME;			//推超限
			m_cSysError &= ~SYSERR_DRAW_EXTREME;		
		}
	}
	else  if((m_nDist[2] > WHEEL_MIN_BACK_VALUE+10) && (m_nDist[2] < WHEEL_MAX_FRONT_VALUE-10))
	{
		m_nWheelExtremeCount=0;
		m_cSysError &= ~SYSERR_PUSH_EXTREME;		
		m_cSysError &= ~SYSERR_DRAW_EXTREME;		
	}
		
}
/////////////////////////////////////////////////////////////////////////
//行车速度和行车方向检测
/////////////////////////////////////////////////////////////////////////
void Vehicle_State_Detect()
{
	//行车方向检测
//	GPIO_ReadInputDataBit(IN_FORWARD_PORT, IN_FORWARD_PIN);//
//	IN_FORWARD = GPIO_ReadInputDataBit(IN_FORWARD_PORT, IN_FORWARD_PIN);
//	IN_BACKWARD = GPIO_ReadInputDataBit(IN_BACKWARD_PORT, IN_BACKWARD_PIN);
	if((m_bVehicleForward==TRUE)&&(m_bVehicleBackward==FALSE))//前进//PL,20130512
	{
		m_bVehicleForward=TRUE;
		m_bVehicleBackward=FALSE;
		m_bStopFlag=FALSE;
	}
	else if((m_bVehicleForward==FALSE)&&(m_bVehicleBackward==TRUE))//后退//PL20130512
	{
		m_bVehicleForward=FALSE;
		m_bVehicleBackward=TRUE;
		m_bStopFlag=FALSE;
	}
	else//停车，或是前进后退都给出来也按照停车处理
	{
		m_bVehicleForward=FALSE;
		m_bVehicleBackward=FALSE;
		m_bStopFlag=TRUE;
		m_bBackMidWheel=TRUE;			//启动方向盘回中
		m_nWheelPosObj=WHEEL_MID_VALUE;
		m_bWheelOpposite=FALSE;			//方向盘反向打轮标记失效
		m_bNewData=FALSE;				//停车时，不会有新数据了，停止相应的纠偏调整
		m_bOK=FALSE;
		m_nFrontExtremeCount=0;
		m_nFrontExtremeCount1=0;
		m_nBackExtremeCount=0;

	}
//	//行车速度检测
//	 IN_SPEED=GPIO_ReadInputDataBit(IN_SPEED_PORT, IN_SPEED_PIN);	
//	if(IN_SPEED==HIGH_SPEED_VAL)
//		m_bVehicleHighSpeed=TRUE;//高速模式
//	else
//		m_bVehicleHighSpeed=FALSE;//低速模式
//	//是否处于环保车工作模式检测
//  IN_MODE=GPIO_ReadInputDataBit(IN_MODE_PORT, IN_MODE_PIN);	
//		if(IN_MODE==TRANSFER_MODE)
//			m_bTransferWorkMode=TRUE;//环保车工作模式	
//		else
//			m_bTransferWorkMode=FALSE;//非环保车工作模式
//					
//      IN_RUDDER=GPIO_ReadInputDataBit(IN_RUDDER_PORT, IN_RUDDER_PIN);
//		if(IN_RUDDER==RIGHT_MODE)	//默认上拉电阻，读到的是1为右舵，0为左舵		
//			m_bRightRudder=TRUE;				
//		else //右舵		
//			m_bRightRudder=FALSE; 
				
}
/////////////////////////////////////////////////////////////////////////
void Push_Wheel()
{
	m_bOutDraw=PUSH_OFF;		//	
	m_bOutPush=PUSH_ON;
	OUT_DRAW=PUSH_OFF;
	OUT_PUSH=PUSH_ON;		

//	if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
	//#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line9, (uint8_t*)"WHEEL_STATE = PUSH_WHEEL");
//#endif // __USE_LCD
//	}

	
if(OUT_DRAW==0x00)
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_RESET);
//	STM_EVAL_LEDOff(LED4); //DRAW对应LED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_RESET);
	}
	else
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED4); //DRAW对应LED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);
	}
	if(OUT_PUSH==0x00)
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED2); //PUSH对应LED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);
  }
	else
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED2); //PUSH对应LED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_SET);
	}
}
/////////////////////////////////////////////////////////////////////////
///
void Draw_Wheel()
{
	m_bOutPush=PUSH_OFF;
	m_bOutDraw=PUSH_ON;
	OUT_PUSH=PUSH_OFF;
	OUT_DRAW=PUSH_ON;			//拉推杆

	//if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
	//#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line9, (uint8_t*)"WHEEL_STATE = DRAW_WHEEL");
//#endif // __USE_LCD
	//}
if(OUT_DRAW==0x00)
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED4); //DRAW对应LED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_RESET);
	}
	else
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED4); //DRAW对应LED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);
	}
	if(OUT_PUSH==0x00)
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED2); //PUSH对应LED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);
  }
	else
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_SET)	;
		STM_EVAL_LEDOn(LED2); //PUSH对应LED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_SET);
	}
}
/////////////////////////////////////////////////////////////////////////
///
void Stop_Wheel()
{
	m_bOutPush=PUSH_OFF;
	m_bOutDraw=PUSH_OFF;		//
	OUT_PUSH=m_bOutPush;
	OUT_DRAW=m_bOutDraw;
//  USART_Send(0xDD);	
//	 USART_Send(0xDD);	
//	 USART_Send(0xDD);	
//	USART_Send(OUT_DRAW);	
//	USART_Send(0xBB);	
//	USART_Send(0xBB);	
//	USART_Send(0xBB);	
//	USART_Send(OUT_PUSH);
//	if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
//#ifdef __USE_LCD
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//		LCD_DisplayStringLine(Line9, (uint8_t*)"WHEEL_STATE = STOP_WHEEL");
//#endif // __USE_LCD
//	}
	if(OUT_DRAW==0x00)
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED4); //DRAW对应LED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_RESET);
	}
	else
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED4); //DRAW对应LED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);
	}
	if(OUT_PUSH==0x00)
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED2); //PUSH对应LED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);
  }
	else
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED2); //PUSH对应LED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_SET);
	}
}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!下面内容待补充（涉及AD）+2015/08/25!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void Control_Mid_Relay(u8_t on)//报告回中错误
{
	
}
void Move_Wheel(u8_t m_bDir)
{
	if(m_bDir==FRONT_WHEEL)			//推推杆
		Push_Wheel();	
	else
		Draw_Wheel();
}


