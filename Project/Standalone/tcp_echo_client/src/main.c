/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "Data_collection.h"
#include "lwip/tcp.h"
#include "tcp_echoclient.h"
#include "serial_debug.h"
#include "main.h"
#include "rectify.h"
#include "sample.h"
#include "sys.h"
#include "stm324xg_eval.h"
//引入ADC后需要补充的头文件_YZ_2015/11/18
#include "stm32f4xx_adc.h"
#include "ADC_YZ_head.h"
#include "GLCD.h"
//作图用_YZ_2015/11/25
#include "stm324xg_eval_lcd.h"




/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void LCD_LED_BUTTON_Init(void);
void DebugComPort_Init(void);
extern void TIM3_Int_Init(int,int);
extern void MY_NVIC_Init(u8,u8,u8,u8);
extern void USART1_init(void);
extern void data_collection(void);
extern void data_rely(void);
extern void LCD_LED_BUTTON_Init(void);
extern void Delay(__IO uint32_t nCount);
extern void DATA_int(void);   //上电初始数据赋值
extern void LCD_LED_BUTTON_Init(void);
extern void Analyse_RectifyData(void);	//分析传感器设备发来的用于纠偏和推杆的数据	
extern void	Rectify(void);
extern void Adjust_Deflect(void);
extern void Wheel_Pos_Control(void);
extern void GPIO_In_Config(void);
extern void GPIO_Out_Config(void);
extern void USART_Send(uint16_t Data);
extern void Vehicle_State_Detect(void);

extern void Draw_Wheel(void); //测试推拉是否输出正常

/*__________________data_collection.h______________________*/
u8_t m_bDistanceSampleOk ; //测距传感器采集数据合法，可以进行纠偏判断
u8_t m_bWheelDataOk;	//方向盘传感器采集数据合法，可以进行纠偏执行
u8_t m_bTransferWorkMode;		//环保车工作模式使能--放在main.c中
u8_t m_bRightRudder;			    //右舵模式--放在main.c中
double SENSOR_DISTANCE_FRONT;//车头距基准线距离
double SENSOR_DISTANCE_BACK;//车尾距基准线距离
//2015/09/11
double SENSOR_DISTANCE_TEMP[2]; //储存原始计算数据(1为车头距离基准线距离 2为车位距离基准线距离)在进行数据和发行确认后再将计算数据存入SENSOR_DISTANCE_FRONT/SENSOR_DISTANCE_BACK中
u8_t SENSOR_DISTANCE_ERR_CNT;//用于记录原始数据偏离(偏离标准参量为SENSOR_DISTANCE_ERR)之前一次存储的合法数据的次数
 //2015/08/25
u8_t m_bBackMidWheel;  //执行停车时将方向盘回到中点的动作，FALSE 不执行，TRUE 执行
u8_t m_bStopFlag=0x01;	
//static void EXTILine15_10_Config(void);
u8_t m_bVehicleForward = 0;		    //行车前进方向标记，TRUE正在前进，FALSE没有前进
u8_t m_bVehicleBackward = 0;			//行车后退方向标记，TRUE正在后退，FALSE没有后退
u8_t BD_ENABLE = 0 ; //标定允许标致位
u8_t m_bStateBefore = 0;          //前一时刻运动方向状态标志位 用来存储前一时刻的运动是正向(0x01)还是反向(0x00)的信息
u16_t Time_count=0;   //为了验证程序是否在执行Data_collection.c中卡死所设定的计数变量(从1++到10000后归零重复++)
u16_t A_count=0;  //记录采集到14A包头的数据包的个数
u16_t B_count=0;  //记录采集到14B包头的数据包的个数
u8_t LCD_Interface1=0; //LCD是否显示数据界面1参量
u8_t LCD_Interface2=0; //LCD是否显示数据界面2参量
u8_t LCD_Vehicle_Trail=0; //LCD是否显示车辆轨迹图参量
u8_t LCD_SENSOR_Image=0;  //LCD是否显示传感器扫描图像参量
u8_t LCD_Vehicle_Posture=0; //LCD是否显示车身姿态图像参量

//2015/12/10针对多个参考点所增加的接力用计数参量
u8_t jc_Record=0; //记录接力时刻该包数据的参考点数目,用于解决一次性扫描到4个参考点的情况_YZ_2015/12/10

//2015/12/13以下针对使用堆栈存储建立的参量
u8_t Stack_Sum_Num=0; //堆栈求和是计数参量
u8_t Stack_Replace_Num=0; //堆栈刷新数据(即一次向前替换)计数参量
u8_t Vehicle_Stack_Num=0; //堆栈储存计数参量
u8_t Vehicle_Stack_Full=0;//堆栈存储是否已满参量
u8_t BD_Stack_Num=0;      //标定位置堆栈储存计数参量
u8_t BD_Stack_Sum_Num=0;  //标定位置堆栈求和计数参量
u8_t BD_Stack_Replace_Num=0; //标定位置堆栈刷新数据计数参量
double Vehicle_X_Stack[Stack_Size]={0}; //储存车辆位置横坐标堆栈
double Vehicle_Y_Stack[Stack_Size]={0}; //储存车辆位置纵坐标堆栈
double Vehicle_sincx_Stack[Stack_Size]={0}; //储存车辆偏转角sin值堆栈
double Vehicle_coscx_Stack[Stack_Size]={0}; //储存车辆偏转角cos值堆栈
double Vehicle_Output_Sum_X={0}; //堆栈输出车辆横坐标值
double Vehicle_Output_Sum_Y={0}; //堆栈输出车辆纵坐标值
double Vehicle_Output_Sum_sincx=0; //堆栈输出车辆偏转角sin值
double Vehicle_Output_Sum_coscx=0; //堆栈输出车辆偏转角cos值
double Standard_Position_x1_Stack=0; //标定时，由堆栈计算得出标准位置横坐标
double Standard_Position_y1_Stack=0; //标定时，由堆栈计算得出标准位置纵坐标
double Standard_Position_x2_Stack=0; //标定时，由堆栈计算得出第二个参考点横坐标
double Standard_Position_y2_Stack=0; //标定时，由堆栈计算得出第二个参考点纵坐标
double Standard_Position_x1_Stack_Sum[Stack_Size]={0}; //标定时，由堆栈计算得出标准位置横坐标
double Standard_Position_y1_Stack_Sum[Stack_Size]={0}; //标定时，由堆栈计算得出标准位置纵坐标
double Standard_Position_x2_Stack_Sum[Stack_Size]={0}; //标定时，由堆栈计算得出第二个参考点横坐标
double Standard_Position_y2_Stack_Sum[Stack_Size]={0}; //标定时，由堆栈计算得出第二个参考点纵坐标


double SCANNER_CENTER   ;
double LEFT_MIN_VALUE_BACK   ;	/* 扫描车后轮左边极限距离  */
double RIGHT_MAX_VALUE_BACK  ;	/* 扫描车后轮右边极限距离  */
double LEFT_MIN_VALUE_FRONT  ;	/* 扫描车前轮左边极限距离70  */
double RIGHT_MAX_VALUE_FRONT ;	/* 扫描车前轮右边极限距离70  */
char text3[40];

/******************以下变量针对0.3°角分辨率交叉组合实现0.1667°角分辨率数据采集__2015/08/25***********************************/
u8_t IntersectLocate = 0x01;     // 交叉记号变量，FALSE对应初始角度为0°数据，TRUE对应初始角度为0.1667°数据(原始数据角分辨率为0.3°)
u8_t IntersectCount = 0x00;     // 储存待交叉赋值数据的计数变量 IntersectCount在程序储存str_temp1后+1 在储存str_temp2后再+1 当IntersectCount == 2时开始交叉赋值
u8_t IntersectOK = 0x00;         // 判断一组数据(14A、14B)是否已经交叉储存到str_temp3数组当中
u16_t str_temp[Size_str_temp]={0}; //储存原始数据
u16_t str_temp_inverse[Size_str_temp]={0}; //储存颠倒后数据

/*____________________display.h____________________________*/
//变量定义
u8_t m_cDisplayBuffer[4];
//
u8_t m_cP377;
//------------------------
u8_t m_bNewDisData;			//刷新显示数据
u8_t m_bDisFlag;
//
u8_t m_cDisErrCnt;				//显示错误信息的标示循环标记
u8_t m_cDisDataDelay;			//显示数据延时
u8_t m_cSysError;			//用不同的位表示系统故障

/*_____ sys.h ________________________________________________*/

s8_t IN_FORWARD;				//p1.0－前进信号(Input)
s8_t IN_BACKWARD;				//p1.1－后退信号(Input)
s8_t IN_MID_SWITCH;			//p1.2－中间点信号(Input)
s8_t OUT_PUSH;					//P3.0－前推推杆(output)，＝1,执行
s8_t OUT_DRAW;					//p3.1－后拉推杆(output)，＝1,执行
//
s8_t IN_MODE;				  //P1.6-工作模式信号输入，0为环保车模式，1为非环保车模式
//
s8_t IN_SPEED;					//p1.4－高速信号检测

s8_t IN_RUDDER;			    //P1.5 - 左舵右舵模式
//
s8_t IN_LOCK_WHEEL;			//p3.5－方向盘锁定检测
//
s8_t IN_AD;					//p1.7- AD输入信号

//下面三个变量暂时不需要
//s8_t DIN=P3^2;						//显示管脚
//s8_t CLK=P3^3;
//s8_t LOAD=P3^4;

/*______rectify.h_________________________________________________________*/
u8_t m_bNewData;	
u8_t m_bOK;                 //车没有在限内行走标志位
u32_t m_nFrontExtremeCount;		//轮超限次数
u32_t m_nFrontExtremeCount1;	    //初始走车轮超限次数
u32_t m_nBackExtremeCount;		//传感器初始化时超限次数计数

u32_t m_nSubValue;				//差值(两个传感器)
u32_t m_nDiffValue;				//传感器和基准线之间的偏差值
u8_t m_bSubSymbol;				//差值的符号

u8_t m_bFrontLeft0;				//一级偏差基准线
u8_t m_bFrontRight0;				//一级偏差基准线
u8_t m_bBackLeft0;				//一级偏差基准线
u8_t m_bBackRight0;				//一级偏差基准线

u8_t m_bCenterDiff;				//轮子与基准线的偏差值，TRUE，靠右，FALSE，靠左
u8_t m_cVehicleState;			//车身姿态		
u8_t m_bWheelLock;				//方向盘是否锁定标记，0未锁定，1锁定
u32_t m_nDist[3];         //m_nDist[0:1]在二代纠偏储存车头车位垂直距离(这里已不再使用)，m_nDist[2]储存方向盘推杆距离__YZ__2015/11/23

/*______execute.c_________________________________________________________*/
u32_t m_nWheelPosObj;			//推杆目标位置
u8_t m_bWheelOpposite;		//是否正在反向打轮

/*______timer.h_________________________________________________________*/
u8_t m_cTimerZdcs;					//定时器计时中断次数
u8_t m_cZdcs,m_cZdcsbj;

/*______adc.h_________________________________________________________*/
ADC_CommonInitTypeDef ADC_CommonInitStructure;  //ADC通用配置结构体声明
ADC_InitTypeDef ADC_InitStruct;              //ADC1配置结构体声明

/*______LCD显示模式控制变量_YZ_2015/11/26_________________________________________________________*/
u8_t Key_Button_Push = 0; //KEY按键是否按下标记参量
u8_t LCD_Change_Mode = 0; //LCD显示模式切换是否发生变量
u16_t LCD_Change_Mode_InTime = 0; //LCD显示模式切换时间条件记录参量(Key按下初始时间)
u16_t LCD_Change_Mode_FiTime = 0; //LCD显示模式切换时间条件记录参量(Key松开终止时间)
u16_t LCD_Change_Mode_dTime = 0; //LCD显示模式切换时间条件记录参量(Key按下至松开间隔时间)

u32_t Time_For_Main = 0;//测试主程序计时参数
int adval_test=0; //测试主程序adc
char text_test[40]="hello,theworld!!!!!!!!!";
unsigned short AD_print_test = 0;
u8_t GPIO_State_val=0;

#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index  6x8                    */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index 16x24                   */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
//Time_For_Main=Time_For_Main+1;
	USART1_init();
//  LCD_LED_BUTTON_Init();
  /* Configure ethernet (GPIOs, clocks, MAC, DMA) */ 
  ETH_BSP_Config();//初始化以太网硬件参数
  /* Initilaize the LwIP stack */
  LwIP_Init();
  /* Infinite loop */
	TIM3_Int_Init(4000-1,8399);//定时中断10ms（5000*8400)/84000=500ms
 // 允许标定__现在测试使用按键让标定置1而不初始化为1__YZ_20160322
//	DATA_int();  //令BD_ENABLE=0x01
	
//!!!!!!!!STM32输入通道配置标准 ADC3_channel 7--->PF9!!!!!!!!!!!!!!!!!!	
	//GPIO初始化_2015/08/31
	GPIO_In_Config();	
	GPIO_Out_Config();	
	
	Init_Rectify_Data(); //初始化纠偏参量_YZ_2015/09/01
	Init_Sample_Data(); //初始化传感器数据采集参量_YZ_2015/09/11
	
//// 使用例程ADC部分的ADC初始化_YZ_2015/11/18
  ADC_Init_YZ();
////	#ifdef __USE_LCD
//  STM324xG_LCD_Init();                          /* Initialize graphical LCD display   */

//  LCD_Clear(Black);                    /* Clear graphical LCD display        */
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
// // LCD_DisplayStringLine(Line0, (uint8_t*)"  Auto drive system  ");
// // LCD_DisplayStringLine(Line1, (uint8_t*)"      V1.0           ");
// // LCD_DisplayStringLine(Line1, (uint8_t*)" Meng && Yuan Zheng  ");

////#endif // __USE_LCD

LCD_Interface1 = TRUE;
LCD_Interface2 = FALSE;

LCD_Vehicle_Trail=FALSE; 
LCD_SENSOR_Image=FALSE;  
LCD_Vehicle_Posture=TRUE;


  while (1)
  {	

		
// Draw_Wheel();		
//printf("begin");
		//USART_Send(0x01);
		
//		GPIO_State_val=GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_7);
// if(GPIO_State_val==0x00){
///*------------下面测试板子的GPIO功能，不会使用纠偏算法部分以及纠偏执行部分程序-----------*/
//		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_SET); 
		
	   
// }
// if(GPIO_State_val==0x01){
//		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);  
//		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET); 
//	}
		
		
 

//		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);//KA2低电平
		

		
//// sprintf(text3, "A_count=0x%04u", A_count);
//// LCD_DisplayStringLine(Line6, (uint8_t*)text3);
//// sprintf(text3, "Time=%7u", Time_For_Main);
//// LCD_DisplayStringLine(Line0, (uint8_t*)text3);

//		
////		STM_EVAL_LEDOn(LED1);//test_YZ_2015/12/15
//		
//		
		if(m_bStopFlag==0x01)
		{
		//	printf("stop");
			GPIO_WriteBit(GPIOA,GPIO_Pin_12, Bit_SET);
		}
		if(m_bStopFlag==0x00)
		{
		//	printf("stop");
			GPIO_WriteBit(GPIOA,GPIO_Pin_12, Bit_RESET);

		}

		if(m_bVehicleForward==0x01)
		{
		//	printf("forward");
			GPIO_WriteBit(GPIOG,GPIO_Pin_6, Bit_SET);
		}
				if(m_bVehicleForward==0x00)
		{
		//	printf("forward");
			GPIO_WriteBit(GPIOG,GPIO_Pin_6, Bit_RESET);
		}

		
		if(m_bVehicleBackward==0x01)
		{
		//	printf("backward");
			GPIO_WriteBit(GPIOG,GPIO_Pin_7, Bit_SET);
		}
				if(m_bVehicleBackward==0x00)
		{
		//	printf("backward");
			GPIO_WriteBit(GPIOG,GPIO_Pin_7, Bit_RESET);
		}

		if(BD_ENABLE==0x01)
		{
			printf("BD");
			GPIO_WriteBit(GPIOG,GPIO_Pin_8, Bit_SET);
		}
		if(BD_ENABLE==0x00)
		{
		//	printf("BD");
			GPIO_WriteBit(GPIOG,GPIO_Pin_8, Bit_RESET);
		}



			Data_collection(); //通过测距传感器计算车身偏转情况以及汽车坐标_包含 Find_REF()和data_rely()两个函数的执行_2015/08/25
	    ADC_StartCnv (); //开始ADC转换
//      adval_test=ADC_GetCnv();
//	   AD_print_test = adval_test;  
//		 sprintf(text_test, "adval = %06u", AD_print_test);
			//sprintf(text_test, "%lu", adval_test);
			//USART_Send(adval_test);
//			printf("%s\n",text_test);
//			GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);

////		LCD_DisplayStringLine(Line1, "before sample_DATA");

		  Sample_Data();     //通过推杆传感器计算方向盘位置_2015/09/14

////		LCD_DisplayStringLine(Line7, (uint8_t*)"after sample_DATA");
////*****************整合纠正部分1_2015/08/24*********************************


////将初始各参量状态如下设置，保证程序可以返回每个时刻需要纠偏时的推杆移动状态_2015/09/01		
     Vehicle_State_Detect();
	    m_bBackMidWheel = FALSE;			
	  //  m_nWheelPosObj = WHEEL_MID_VALUE;
	    m_bWheelOpposite = FALSE;	
	    m_bNewData = TRUE;			
	    m_bOK = TRUE;
	  //   m_bDistanceSampleOk = TRUE;    // m_bDistanceSampleOk 置位在data_collection.c中进行
	    m_bWheelDataOk = TRUE;  // m_bWheelDataOk 置位在sample.c中的函数Sample_Data()中进行
		m_bTransferWorkMode=FALSE; //调试默认左舵车
		m_bRightRudder=FALSE; //调试默认左舵车
    
		if(m_bStopFlag==TRUE) //停车时方向盘回中
		{
			m_nWheelPosObj = WHEEL_MID_VALUE;
			Wheel_Pos_Control();// 测试时先不控制方向盘__YZ__20160324
		}
//		
//		
  if(m_bStopFlag==FALSE)
		{
			Analyse_RectifyData();	//分析传感器设备发来的用于纠偏和推杆的数据	
			Rectify();				//纠偏判断
			//USART_Send(0xBB);
			//USART_Send(m_cVehicleState);
			//Check_Wheel_Lock();//检查方向盘是否已锁
			Adjust_Deflect();		//调节偏移
      Wheel_Pos_Control();//移动方向盘  测试时先不控制方向盘__YZ__20160324
		}
//	
//		//sample.c --->m_bWheelDataOk 待加！！！！！！！！！！！！
//		

    //Eth_Link_ITHandler(DP83848_PHY_ADDRESS);	
    if (ETH_CheckFrameReceived()) // mdf
   // { 
      /* process received ethernet packet */
      LwIP_Pkt_Handle();
   // }		 
    /* handle periodic timers for LwIP */ 
   LwIP_Periodic_Handle(10);	
	//	printf("end");	
  } 

}

int fputc(int ch, FILE *f) //对fputc函数重新定向以实现printf向串口发送数据__20160317__YZ
{

USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成你所使用的 USARTx 

while (!(USART1->SR & USART_FLAG_TXE));

return (ch);

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}


#endif


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
