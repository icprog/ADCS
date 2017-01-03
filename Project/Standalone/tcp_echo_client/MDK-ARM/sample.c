/***************************************************************************
* NAME:    Sample.c
*----------------------------------------------------------------------------*/
/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include "sample.h"
#include "display.h"
#include "rectify.h"
#include "Data_collection.h"
#include "stm32f4xx_adc.h"
#include "ADC_YZ_head.h"
#include "stm32f4xx_usart.h"
#include "GLCD.h"

#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index  6x8                    */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index 16x24                   */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif

/*_____ V A R I A B L E ____________________________________________________*/

u32_t m_nSBuffer[3];             //两路实时采集的值
u32_t m_nOBuffer[3];				//存放上一次的数据
u8_t m_cMutationCnt[3];		//测量数据突变计数
//
u8_t count;					//用于采样延时
u8_t m_cZdcsSample;			//（采集周期计数）
//
u8_t m_cADChannel;				//当前采集第几个通道 
//
u8_t m_cSampleDelay;				//数据采集延时，5次，50ms


u32_t adval_Stack[adval_Stack_Size]={0}; //储存ad返回值堆栈
u8_t adval_Count_Num=0; //储存ad堆栈计数参量
u32_t adval_Sum=0;//储存ad堆栈求和变量
u8_t adval_Sum_Num=0; //储存ad堆栈求和参量

///////////////////////////////////////////////////////////////////////////////
extern u32_t m_nDist[3];                //xyw
extern u8_t m_cP377;				//377标记
extern u8_t m_bBackMidWheel;
extern u8_t m_bStopFlag;
extern u8_t m_bDistanceSampleOk;
extern void USART_Send(uint16_t Data);
extern u8_t LCD_Interface1;
extern u8_t LCD_Interface2;
extern u8_t LCD_Vehicle_Trail; //LCD是否显示车辆轨迹图参量
extern u8_t LCD_SENSOR_Image;  //LCD是否显示传感器扫描图像参量
extern u8_t LCD_Vehicle_Posture; //LCD是否显示车身姿态图像参量
unsigned short AD_print = 0;

extern volatile unsigned char clock_1s; // Flag activated each second_YZ_2015/11/20

char text[40];

//***************************************************************
//采集AD通道，0-第一包；1-第二包; 2-第三包 xyw 
//对于第三代纠偏，第一二报数据对应的车头车尾信息读取放在了data_collection.c当中，只有第三包数据的读取(要用到推杆传感器)放在了sample.c当中，因此数组索引均有且只有2
void Sample_AD()  //涉及推杆传感器数据采集(AD转换)--待修改_YZ_2015/09/11
{
//	u8_t adnum,i;
	u32_t adval=0;  
//	u32_t result_l;
	//---------保存ADC规则通道返回数据_YZ_2015/09/14
//	adval=ADC_GetConversionValue(ADC3); //ADC3_channel 7--->PF9
	
	//---------借用Blinky例程：保存ADC规则通道返回数据_YZ_2015/11/18
	adval=ADC_GetCnv(); //默认使用ADC3_channel 7--->PF9
	
//  USART_Send(adval);
//	------------------------------显示-----------------------------
			                /* Read AD_last value                 */
//		LCD_SetBackColor(White);
//    LCD_SetTextColor(Blue);
    if (adval != AD_print) {
			if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
//#ifdef __USE_LCD
    // GLCD_SetTextColor(Red);
    //  GLCD_Bargraph (9 * __FONT_WIDTH, 4 * __FONT_HEIGHT, 10 * __FONT_WIDTH, __FONT_HEIGHT - 2, (adval >> 2));
//    USART_Send(0xD1);
    //  GLCD_SetTextColor(White);
//    USART_Send(0xD2);
//#endif // __USE_LCD

      AD_print = adval;              /* Get unscaled value for printout    */
//			USART_Send(0xD3);
    }
	}
		
						if(adval_Count_Num<=(adval_Stack_Size-1))
					{
					adval_Stack[adval_Count_Num]=adval;				
					adval_Count_Num=adval_Count_Num+1;
					}
					else if(adval_Count_Num>=adval_Stack_Size)
					{
					  for(adval_Sum_Num=1; adval_Sum_Num<=(adval_Stack_Size-1); adval_Sum_Num++)
						{
							adval_Stack[adval_Sum_Num-1]=adval_Stack[adval_Sum_Num];
						}
							adval_Stack[adval_Stack_Size-1]=adval;
					}
					
					for(adval_Sum_Num=1; adval_Sum_Num<=adval_Count_Num; adval_Sum_Num++)
					{
						adval_Sum=adval_Sum+adval_Stack[adval_Sum_Num-1];
					}
					
					adval=adval_Sum/adval_Count_Num;
					adval_Sum=0;
	        
	
//USART_Send(0xD4);
//    /* Printf message with AD value to serial port every 1 second */
//  //  if (clock_1s) {
//      clock_1s = 0;
//if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
//     sprintf(text, "m_nOBuffer= %6u", m_nOBuffer[2]);
// LCD_DisplayStringLine(Line1, (uint8_t*)text);
//// sprintf(text, "adval= %6u", adval);
//// LCD_DisplayStringLine(Line7, (uint8_t*)text);
//}

//if(LCD_Interface1==FALSE && LCD_Interface2==TRUE && LCD_Vehicle_Posture==TRUE){
////      sprintf(text, "adval = 0x%04X", AD_print);
////#ifdef __USE_LCD
//// LCD_DisplayStringLine(Line8, (uint8_t*)text);
////#endif // __USE_LCD
//}


////		USART_Send(0xD8);
		
  //  }
	
	/*----
	采集精度12u8_t，采集值：0~4096,对应0-3.3V，实际4-20mA，对应0.66~3.3V，对应0~400mm__YZ__2015/12/17
	(采集值*400/4096)mm
	------*/
	//m_nSBuffer[2]=(int)((double)adval*400.0/4096.0);
  m_nSBuffer[2]=(int)((double)adval*0.55264);
//	printf("adval=%lu\n",m_nSBuffer[2]);
	
}
//***************************************************************
//采集数据
void Sample_Data()
{	u32_t n;
	
  LCD_SetBackColor(Blue);
  LCD_SetTextColor(White);

//	if((m_bStopFlag==TRUE)&& (m_bBackMidWheel==FALSE))//没有移动，且方向盘没有执行回中   ????? && (m_bBackMidWheel==FALSE
//		return;	
	
	m_cSampleDelay++;
	if(m_cSampleDelay<2)
		return;
	//
	Sample_AD();	//按通道采集数据xyw_这里只采集推杆传感器返回数据_YZ_2015/09/14
//		m_bWheelDataOk=TRUE;    //准备方向盘数据分析
		if(m_nSBuffer[2] > m_nOBuffer[2])  // 数据防抖
			n=m_nSBuffer[2]-m_nOBuffer[2];	   
		else
			n=m_nOBuffer[2]-m_nSBuffer[2];
		if(n >= WHEEL_ERR)  //前后两次数据差大于抖动阈值才认为该数据返回有效
		{				     

			m_nDist[2]=m_nSBuffer[2];
			m_nOBuffer[2]= m_nSBuffer[2];
	//  USART_Send(0xD2);
			Analyse_Data();			//判断返回数据合法性
		}	
    
	//
	m_cSampleDelay=0;	
}
//------------------分析推杆传感器数据---------------------
void Analyse_Data()
{
	u32_t n;
//	u8_t i; 
//	u32_t dis_f,dis_b;//前后传感器的修正差值
	//
//下面是判断采集数据的合法性，其中测距传感器数据合法性的代码放到了data_collection.c当中，下面的代码是判断推杆采集数据合法性
//下面这部分注释掉的内容是二代纠偏防止车头车尾传感器数据抖动而设置，这里不再使用_YZ_2015/12/17	
//		if(m_nSBuffer[2] > m_nOBuffer[2])
//			n=m_nSBuffer[2]-m_nOBuffer[2];
//		else
//			n=m_nOBuffer[2]-m_nSBuffer[2];
//		if((n >= SENSOR_DIFF)&&(n<=SENSOR_ERR))//PL20130512
//			m_nOBuffer[2]=m_nSBuffer[2];
//		else if(n>SENSOR_ERR)
//		{
//			m_cMutationCnt[2]++;
//			if(m_cMutationCnt[2]>9)//连续10次数据都大于阈值，则认为数据合法
//			{
//				m_cMutationCnt[2]=0;
//				m_nOBuffer[2]=m_nSBuffer[2];  //这里8.0版程序是m_nSBuffer[i],应该修改为 m_nSBuffer[2] __YZ_2015/11/18 
//			}
//			
//		}
	
	//
	//m_bDistanceSampleOk=TRUE;		//// m_bDistanceSampleOk 置位在data_collection.c中进行
	//
	m_nDist[0]=m_nOBuffer[0];
	m_nDist[1]=m_nOBuffer[1];
	
	//增加车身斜率，从传感器直接映射到车轮的实际位置进行判断_但这部分是二代纠偏受限于传感器的安装位置缘故，三代纠偏不用进行下列修正_YZ_2015/09/11
//	if(m_nDist[0]<m_nDist[1])//车身左倾，车轮位置都减去修正值
//	{
//		if(m_nDist[0]>10)
//		{
//			n=m_nDist[1]-m_nDist[0];
//			dis_f=n*SENSOR_WHEEL_FRONT/SENSOR_DISTANCE;
//			dis_b=n*SENSOR_WHEEL_BACK/SENSOR_DISTANCE;
//			m_nDist[0]=m_nDist[0]-dis_f;
//	//			m_nDist[1]=m_nDist[1]+dis_b;
//		}
//	}
//	else //车身右倾，车轮位置都加上修正值
//	{
//		if(m_nDist[1]>10)
//		{
//			n=m_nDist[0]-m_nDist[1];
//			dis_f=n*SENSOR_WHEEL_FRONT/SENSOR_DISTANCE;
//			dis_b=n*SENSOR_WHEEL_BACK/SENSOR_DISTANCE;
//			m_nDist[0]=m_nDist[0]+dis_f;
//	//			m_nDist[1]=m_nDist[1]-dis_b;
//		}
//	}
}  
/////////////////////////////////////////////////////////////////////////
//
void Init_Sample_Data()
{
	m_bDistanceSampleOk=FALSE;
	m_bWheelDataOk=FALSE;
	//
	m_nOBuffer[0]=0;			//存放上一次的数据
	m_nOBuffer[1]=0;			//存放上一次的数据
	m_nOBuffer[2]=0;
    //
	m_nDist[0]=0;
	m_nDist[1]=0;
	m_nDist[2]=0;
	//
	m_nSBuffer[0]=0;
	m_nSBuffer[1]=0;
	m_nSBuffer[2]=0;
	//
	m_cSampleDelay=0;
}

