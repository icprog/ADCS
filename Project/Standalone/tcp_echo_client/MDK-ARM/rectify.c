//==============================================
//*************  Rectify.c  ***********************
//==============================================
#include "rectify.h"
#include "data_collection.h"
#include "display.h"
#include "timer.h"
#include "execute.h"
#include "stm32f4_GPIO_int.h"
/*_________________________ Variable ________________________________________*/
extern u8_t m_bNewData;	
extern u8_t m_bOK;                 //车没有在限内行走标志位
extern u32_t m_nFrontExtremeCount;		//轮超限次数
extern u32_t m_nFrontExtremeCount1;	    //初始走车轮超限次数
extern u32_t m_nBackExtremeCount;		//传感器初始化时超限次数计数

extern u32_t m_nSubValue;				//差值(两个传感器)
extern u32_t m_nDiffValue;				//传感器和基准线之间的偏差值
extern u8_t m_bSubSymbol;				//差值的符号

extern u8_t m_bFrontLeft0;				//一级偏差基准线
extern u8_t m_bFrontRight0;				//一级偏差基准线
extern u8_t m_bBackLeft0;				//一级偏差基准线
extern u8_t m_bBackRight0;				//一级偏差基准线

extern u8_t m_bCenterDiff;				//轮子与基准线的偏差值，TRUE，靠右，FALSE，靠左
extern u8_t m_cVehicleState;			//车身姿态		
extern u8_t m_bWheelLock;				//方向盘是否锁定标记，0未锁定，1锁定


//----------------------------------------------------------------------
extern u8_t m_bDistanceSampleOk ;				//测距传感器采集数据合法
extern u8_t m_bStopFlag;
extern double SENSOR_DISTANCE_FRONT;//车头距基准线距离
extern double SENSOR_DISTANCE_BACK;//车尾距基准线距离
extern u8_t m_bVehicleBackward;  //后退标志位
extern u8_t m_bVehicleForward;  //前进标志位 这个后面要赋值给输入管脚
//------------------------------------
/////////////////////////////////////////////////////////////////////////
///初始化数据
void Init_Rectify_Data()
{
	m_bNewData=FALSE;			
	m_bFrontLeft0=TRUE;				
	m_bFrontRight0=TRUE;				
	m_bStopFlag=TRUE;
	m_bOK=FALSE;						//车没有在限内行走标志位
	m_nBackExtremeCount=0;
	m_nFrontExtremeCount=0;
	m_nFrontExtremeCount1=0;
//	WHEEL_CHANCE=1;
}
/////////////////////////////////////////////////////////////////////////
///分析传感器发来的用于纠偏判断的数据
void Analyse_RectifyData()
{
	if(m_bDistanceSampleOk ==TRUE)				//接收到纠偏的数据
	{
		m_bDistanceSampleOk =FALSE;
		//--------------------------------激光测距值
		//计算差值，SENSOR_DISTANCE_FRONT：前传感器，SENSOR_DISTANCE_BACK：后传感器
		if(SENSOR_DISTANCE_BACK > SENSOR_DISTANCE_FRONT) //车身左偏
		{
			m_nSubValue=SENSOR_DISTANCE_BACK-SENSOR_DISTANCE_FRONT;				//差值(两个传感器)
			m_bSubSymbol=TRUE;								//TRUE表示差值为正
		}
		else					//车身右偏或是平行
		{
			m_nSubValue=SENSOR_DISTANCE_FRONT-SENSOR_DISTANCE_BACK;				//差值(两个传感器)
			m_bSubSymbol=FALSE;								//FALSE表示差值为负	
		}
		//---------------------------------------------------------------
		if(m_bVehicleForward==TRUE)	//车辆相对于基准线的位置判断，注意左右互锁---前进
		{
			if(SENSOR_DISTANCE_FRONT < LEFT_MIN_VALUE_FRONT)
			{
				m_bFrontLeft0=TRUE;
				m_bFrontRight0=FALSE;				
				m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_FRONT;//calculate the difference between sensor0 and Center
			}
			else if(SENSOR_DISTANCE_FRONT > RIGHT_MAX_VALUE_FRONT)
			{

				m_bFrontLeft0=FALSE;
				m_bFrontRight0=TRUE;
				m_nDiffValue=SENSOR_DISTANCE_FRONT-SCANNER_CENTER;//calculate the difference between sensor0 and Center
			}
			else 
			{
				m_bFrontLeft0=FALSE;
				m_bFrontRight0=FALSE;				
				if(SENSOR_DISTANCE_FRONT>SCANNER_CENTER)//车身偏离中心线左侧，车头减去中心差值为正
				{
					m_bCenterDiff=TRUE; //TRUE表示差值为正(车靠右),FALSE表示差值为负(车靠左)
					m_nDiffValue=SENSOR_DISTANCE_FRONT-SCANNER_CENTER;
				}
				else							  //车身偏离中心线左侧，差值为负
				{	
					m_bCenterDiff=FALSE;
					m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_FRONT;
				}
			}
		}
		else if(m_bVehicleBackward==TRUE)	//偏移判断，注意左右互锁---后退
		{
			if(SENSOR_DISTANCE_BACK < LEFT_MIN_VALUE_BACK)
			{
				m_bBackLeft0=TRUE;
				m_bBackRight0=FALSE;
				m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_BACK;//calculate the difference between sensor1 and Center
			}    
			else if(SENSOR_DISTANCE_BACK > RIGHT_MAX_VALUE_BACK )
			{
				m_bBackLeft0=FALSE;
				m_bBackRight0=TRUE;
				m_nDiffValue=SENSOR_DISTANCE_BACK-SCANNER_CENTER;//calculate the difference between sensor1 and Center
			}
			else 
			{
				m_bBackLeft0=FALSE;
				m_bBackRight0=FALSE;
				if(SENSOR_DISTANCE_BACK>SCANNER_CENTER)//车身偏离中心线右侧，车位减去中心差值为正
				{
					m_bCenterDiff=TRUE;
					m_nDiffValue=SENSOR_DISTANCE_BACK-SCANNER_CENTER;
				}
				else							  //车身偏离中心线左侧，差值为负
				{	
					m_bCenterDiff=FALSE;
					m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_BACK;
				}
			}
		}
		//-------------------------------------------
		//轮偏移超限故障判断

		if((SENSOR_DISTANCE_FRONT > RIGHT_EXTREME_VALUE || SENSOR_DISTANCE_BACK > RIGHT_EXTREME_VALUE))
		{
			 if((SENSOR_DISTANCE_FRONT==0||SENSOR_DISTANCE_BACK==0) && (m_bOK==FALSE))
			{
			    m_nFrontExtremeCount1++;
				m_nFrontExtremeCount=0;	
				if(m_nFrontExtremeCount1>EXTREME_VALUE1)
				{
					m_nFrontExtremeCount1=0;
					m_cSysError |= SYSERR_RIGHT_EXTREME;			//扫描车车轮左偏移超限
					m_cSysError &= ~SYSERR_LEFT_EXTREME;		//扫描车车轮偏移未超限
		   		}

			}
			else 
			{
				m_nFrontExtremeCount++;
				m_nFrontExtremeCount1=0;
				if(m_nFrontExtremeCount>EXTREME_VALUE)
				{
					m_nFrontExtremeCount=0;
					m_cSysError |= SYSERR_RIGHT_EXTREME;			//扫描车车轮左偏移超限
					m_cSysError &= ~SYSERR_LEFT_EXTREME;		//扫描车车轮偏移未超限
		   		}

			}
		}
		else if((SENSOR_DISTANCE_FRONT < LEFT_EXTREME_VALUE) || (SENSOR_DISTANCE_BACK < LEFT_EXTREME_VALUE))
		{
			m_nFrontExtremeCount++;
			if(m_nFrontExtremeCount>EXTREME_VALUE)
			{
				m_nFrontExtremeCount=0;
				m_cSysError |= SYSERR_LEFT_EXTREME;		//扫描车车轮右偏移超限
				m_cSysError &= ~SYSERR_RIGHT_EXTREME;		//扫描车车轮偏移未超限
			}
		}
		else  if(((SENSOR_DISTANCE_FRONT < RIGHT_EXTREME_VALUE-10) && (SENSOR_DISTANCE_FRONT> LEFT_EXTREME_VALUE+10))&&((SENSOR_DISTANCE_BACK < RIGHT_EXTREME_VALUE-10) && (SENSOR_DISTANCE_BACK > LEFT_EXTREME_VALUE+10)))
		{
			m_nFrontExtremeCount=0;
			m_nFrontExtremeCount1=0;
			m_cSysError &= ~SYSERR_LEFT_EXTREME;		//扫描车车轮偏移未超限
			m_cSysError &= ~SYSERR_RIGHT_EXTREME;		//扫描车车轮偏移未超限
		    m_nBackExtremeCount++;
			if(m_nBackExtremeCount>EXTREME_VALUE/2)
			{
				m_bOK=TRUE;
				m_nBackExtremeCount=0;
			}
		}
		m_bNewData=TRUE;
	//	m_bNewDisData=TRUE;
	
		m_cSysError &= ~SYSERR_5S_NODATA;

	}
}
/////////////////////////////////////////////////////////////////////////
//纠偏判断
void Rectify()
{
	if(m_bNewData==TRUE) //没有新的数据不用纠偏
	{			
		m_bNewData=FALSE;
		//纠偏判断
		Drive_Judge();				
	}
}
/////////////////////////////////////////////////////////////////////////
///
void Drive_Judge()
{
	if(m_bVehicleForward==TRUE)				//前进时
	{
		if((m_bSubSymbol==FALSE)&&(m_bFrontLeft0==TRUE))//差值为负（车身右偏），车偏基准线左侧F1(对应rectify.h中Case 1)
		{//车轮应与车身方向一致，下一步适当时机向左打轮，使车轮回正，与基准线方向一致
			m_cVehicleState=STATE_1;	
		}
		else if((m_bSubSymbol==TRUE)&&(m_bFrontRight0==TRUE))//差值为正（车身左偏），车偏基准线右侧F2(对应rectify.h中Case 2)
		{//车轮应与车身方向一致，下一步适当时机向右打轮，使车轮回正，与基准线方向一致
			m_cVehicleState=STATE_2;
		}
		else if((m_bSubSymbol==TRUE)&&(m_bFrontLeft0==TRUE))//差值为正（车身左偏），车偏基准线左侧F3(对应rectify.h中Case 3)
		{//车轮应向右打，直至进入F1
			m_cVehicleState=STATE_3;
		}
		else if((m_bSubSymbol==FALSE)&&(m_bFrontRight0==TRUE))//差值为负（车身右偏），车偏基准线右侧F4(对应rectify.h中Case 4)
		{//车轮应向左打，直至进入F2
			m_cVehicleState=STATE_4;
		}
		else if((m_bFrontLeft0==FALSE)&&(m_bFrontRight0==FALSE))//前轮已经进入二级基准行车线区域内部
		{
			m_cVehicleState=STATE_5;
		}
	}
	else		//后退时
	{
		if((m_bSubSymbol==TRUE)&&(m_bBackLeft0==TRUE))//差值为正（车身左偏），车偏基准线左侧B1(对应rectify.h中Case 3)
		{//车轮应与车身方向一致，下一步适当时机向左打轮，使车轮回正，与基准线方向一致
			m_cVehicleState=STATE_1;
		}				
		else if((m_bSubSymbol==FALSE)&&(m_bBackRight0==TRUE))//差值为负（车身右偏），车偏基准线右侧B2(对应rectify.h中Case 4)
		{//车轮应与车身方向一致，下一步适当时机向右打轮，使车轮回正，与基准线方向一致
			m_cVehicleState=STATE_2;
		}
		else if((m_bSubSymbol==FALSE)&&(m_bBackLeft0==TRUE))//差值为负（车身右偏），车偏基准线左侧B3(对应rectify.h中Case 1)
		{//车轮应与向右打，直至进入B1
			m_cVehicleState=STATE_3;
		}
		else if((m_bSubSymbol==TRUE)&&(m_bBackRight0==TRUE))//差值为正（车身左偏），车偏基准线右侧B4(对应rectify.h中Case 2)
		{//车轮应向左打，直至进入B2
			m_cVehicleState=STATE_4;
		}
		//后轮已经进入基准行车线区域，车轮回正到基准线上，实际打轮角度为车身与基准线夹角，且需要根据车身姿态实施调整，直至夹角为零
		else if((m_bBackLeft0==FALSE)&&(m_bBackRight0==FALSE))
		{
			m_cVehicleState=STATE_5;
		}
	}
}
/////////////////////////////////////////////////////////////////////////
//方向盘锁定故障检测
void Check_Wheel_Lock()
{
 IN_LOCK_WHEEL=GPIO_ReadInputDataBit(IN_LOCK_WHEEL_PORT, IN_LOCK_WHEEL_PIN);
	if(IN_LOCK_WHEEL==LOCK_WHEEL_VALUE)
	{
		m_cSysError |= SYSERR_WHEEL_LCOK;
		m_bWheelLock=TRUE;
	}
	else
	{
		m_cSysError &= ~SYSERR_WHEEL_LCOK;
		m_bWheelLock=FALSE;	
	}
}

