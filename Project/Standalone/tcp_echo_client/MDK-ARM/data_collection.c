	/**
		******************************************************************************
		* @file    data_collection.c
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
	#include "stm32f4x7_eth.h"
	#include "netconf.h"
	#include "arm_math.h"
	#include "lwip/tcp.h"
	#include "tcp_echoclient.h"
	#include "serial_debug.h"
	#include "data_collection.h"
	#include "rectify.h"
	#include "execute.h"
	#include "display.h"
	#include "GLCD.h"
	#include "ADC_YZ_head.h"
	//作图用_YZ_2015/11/25
#include "stm324xg_eval_lcd.h"
	/* Private typedef -----------------------------------------------------------*/
	/* Private define ------------------------------------------------------------*/
	#define SYSTEMTICK_PERIOD_MS  10   
	/* Private macro -------------------------------------------------------------*/
	 extern  __IO uint8_t EthLinkStatus;
	 u16_t IC_flag;//单片机和传感器连接标志
	 uint32_t IC_time=0;//单片机和传感器连接次数，超过三次向外发送错误标志
	 u8_t OI_flag;//基准检测标志位
	 u8_t JL_flag;//接力标志位
	 u8_t OI_normal_flag=PASS_EMPTY;//检测到基准数量小于2个
//	 u16_t Normal_data[281]={2500};
	 //#define  Normal_h  5100;
	/* 激光区域扫描仪处理数据 ---------------------------------------------------------*/
		u8_t  Data_correct_flag;
		u16_t RD_time=0;//处理完数据小于某个阈值的次数
		u8_t  Return_data_flag;//处理完数据的正确性标志
		u8_t  Data_RECE[1500];//除去包头的数据存储数组

	 //-----------------------标定时刻数据L1、L2、a1、b2-----------------------------------------------------
		u16_t BD_Length1;//标定时传感器距基准2的距离
		u16_t BD_Length2=0;//标定时传感器距基准2的距离
		u16_t BD_Angle1;//标定时传感器与基准2夹角
		u16_t BD_Angle2=0;//标定时传感器与基准2夹角
		u16_t BDY;//标定时车身与基准远点距离
	//------------------------行进过程中L11、L22、a2、b2-----------------------------------------------------
		u16_t MOVE_Length1;//行进过程中传感器距离第一个基准距离
		u16_t MOVE_Length2=0;//行进过程中传感器距离第二个基准距离
		u16_t MOVE_Angle1;//行进过程中传感器与第一个基准夹角
		u16_t MOVE_Angle2;//行进过程中传感器与第二个基准夹角

		u16_t SENSOR_DISTANCE_MID;//标定时车身距基准线距离
		u16_t Pos_num=0;//测量距离
		u16_t Tim_n;//计算时间时计数器2溢出的次数
		u16_t Last_Pos_num=0;//上次测量距离
		
		s16_t str_Lev[Size_str_temp]={0};//水平距离
		s16_t str_Ver[Size_str_temp]={0};//垂直距离
	//	s16_t Data_One_Last[649]={0};//存储上包处理过的数组
		s16_t Data_Packet[5]={0};//保存连续5包数据
		//*********************************** 新增加的变量__YZ__2015/08/11 **************************************************************************************
    u8_t BD_Flag; //标定标志参量（1为开始标定，0为不标定）	
		u16_t BD_Num=0; //标定时的计数器参量
		u8_t jc_origin=0; //储存当前包数据找到的所有突变区域的个数(并没有过滤干扰点以及找出真正的参考点)
		double gg;
		double avenum_a=0,avenum_b=0,avenum_c=0,avenum_d=0,	avenum_e=0,	avenum_f=0,		avenum_g=0,	avenum_h=0,avenum_i=0;	
		double sinave,cosave,sinaveangle,cosaveangle,sinaveave,cosaveave,sinave1,cosave1,sinaveangle1,cosaveangle,sinaveave,cosaveave;
		double a1,b1,L1,L2;
		double ANGLE_coscx=0,ANGLE_sincx=0;
		double SUM_Translation_X0=0,SUM_Translation_Y0=0;//针对第一次接力，储存坐标平移累计量(用于输出统一坐标后的传感器位置坐标信息)
		double SUM_Translation_X=0,SUM_Translation_Y=0;//针对第一次接力之后的接力，储存坐标平移累计量(用于输出统一坐标后的传感器位置坐标信息)
		struct BD_data //用来在标定的时候储存标定信息（在标BD_Flag==FALSE之后将结果取平均值）
		{
			u32_t angle;
			u32_t polar;
		} BD_Sum[2];
	//s16_t Lengh_table[];


	//*********************************** 为接力新增加的变量__YZ__2015/08/18 **************************************************************************************
    const u16_t Ref_Num =10;    //用来表示接力的次数(是累计次数，正向前进过程每接力一次Scan_Num+1，后退过程每接力一次Scan_Num-1)
    u8_t Scan_Done=0;     //用来表示接力已经完成的参数(0为)
    u16_t Cor_num=0;      //用来分辨应该将参考点信息分配给标准位置还是当前时刻位置的参量
    u16_t Cal_num=0;      //用来提示计算坐标内容的参量	
		u8_t j1=0;              // 表示在已知的真参考点中挑选出来的第一个参考点的序号
		u8_t j2=1;              // 表示在已知的真参考点中挑选出来的第二个参考点的序号
		u8_t JL_x2_Ready=0x00;   // 表示传感器已经到达接力临界区域，可以开始接力
		u16_t Scan_Num=0; 
		double a1_Ref[Ref_Num],b1_Ref[Ref_Num],L1_Ref[Ref_Num],L2_Ref[Ref_Num],cosa1_Ref[Ref_Num],
					 cosb1_Ref[Ref_Num],sina1_Ref[Ref_Num],sinb1_Ref[Ref_Num];
		double data_x1_Ref[Ref_Num],data_y1_Ref[Ref_Num],data_x2_Ref[Ref_Num],data_y2_Ref[Ref_Num];
//*********************************** 为接力新增加的变量__YZ__2015/08/18 **************************************************************************************
   
  //const u16_t Ref_Num =10; //放置参考点的数量
	//u8_t JL_x2_Ready=0x00;  //用来表示传感器已经到达临界区域，可以进行接力的参量
		double data_x1,data_x2,data_y1,data_y2;
	//double a1_Ref[Ref_Num],b1_Ref[Ref_Num],L1_Ref[Ref_Num],L2_Ref[Ref_Num],cosa1_Ref[Ref_Num],
	//				 cosb1_Ref[Ref_Num],sina1_Ref[Ref_Num],sinb1_Ref[Ref_Num];
		//用来储存前进时每次接力的标准位置(这样在后退时可以直接调用该数组)

//****************************数据合法性新增加的变量_YZ_2015/09/11************************************************************************************************************	
		u32_t n;//用于记录采集数据于储存数据差值的变量
//****************************数据合法性新增加的变量_YZ_2015/11/25************************************************************************************************************			
		unsigned short Tubian_t[Ref_Num],Tubian_c[Ref_Num];//用于储存一次扫描后得到的关于参考点的定标信息t与c，并用在之后的LCD屏幕输出
    double TC_distance; //用于计算找到标定点后t与c对应点之间的距离与参考点已知直径比较，从而滤过干扰点
//**************屏幕输出参数****************		
char text2[40];
unsigned short DATA_TO_SCREEN;
#define __FI        1                   /* Font index 16x24                   */
#if (__FI == 1)                         /* Font index  6x8                    */                         
  #define __FONT_WIDTH  16
  #define __FONT_HEIGHT 24
#else                                   /* Font index 16x24                   */
  #define __FONT_WIDTH   6
  #define __FONT_HEIGHT  8
#endif
		
		
//// Flag activated each second_YZ_2015/11/20

volatile unsigned char clock_1s;		
		
		
		int  DATA_NUM ;//除去包头的数据量
		struct Changedata
		{
			double angle;
			double polar;
		} senor[5];//用一个结构体数组去存储传感器变化的角度和其对应的极坐标长度*/
	__IO uint32_t LocalTime = 0;  /* this variable is used to create a time reference incremented by 10ms */
	uint32_t timingdelay;
	/* Private function prototypes -----------------------------------------------*/
	void LCD_LED_BUTTON_Init(void);
	void DebugComPort_Init(void);
	void USART1_init(void);
	void Find_REF(void); //找基准
	void data_rely(void) ;
	//void Charge_REF();//基准接力判断
	//void Data_Calibration();//静态标定
	void DATA_Dynamic(void);//动态走车数据
	extern void MY_NVIC_Init(u8,u8,u8,u8);  //中断初始化
	void DATA_int(void);
	/* Private functions ---------------------------------------------------------*/
	uint8_t Interrupt_flag=0;
		
/*	double fabs(double x)//求绝对值
	{
		double y;
		 if(x>=0)  y=x;
		 else y=(-x);
		 return y;
	}*/
	int Tr(char source_data)//数据转换
	{   
		int dest_data;
		//for(int i=0;i<4;i++)
	 if(source_data>=(0x41))  dest_data=(source_data-0x37);
	 else dest_data=(source_data-0x30);
	 return dest_data;
	}
	void TIM3_Int_Init(uint16_t arr,uint16_t psc)
	{
		RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
		TIM3->ARR=arr;  	//设定计数器自动重装值 
		TIM3->PSC=psc;  	//预分频器	  
		TIM3->DIER|=1<<0;   //允许更新中断	  
		TIM3->CR1|=0x01;    //使能定时器3
		MY_NVIC_Init(0,0,TIM3_IRQn,2);	//抢占0，子优先级0，组0									 
	}
	void TIM2_Init(void) //这个就是自动装载的计数值，每隔70ms计数器溢出依次数值
	{
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

			/* TIM5 clock enable */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			/* Time base configuration */
		 
			TIM_TimeBaseStructure.TIM_Period = (700 - 1);
			// 这个就是预分频系数，当由于为0时表示不分频所以要减1
			TIM_TimeBaseStructure.TIM_Prescaler = (8400 - 1);
			// 使用的采样频率之间的分频比例
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			//向上计数
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			//初始化定时器2
			TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

			/* Clear TIM2 update pending flag[清除TIM5溢出中断标志] */
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

			/* TIM IT enable */ //打开溢出中断
			TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

			/* TIM5 enable counter */
			TIM_Cmd(TIM2, ENABLE);  //计数器使能，开始工作

			/* 中断参数配置 */
		 MY_NVIC_Init(1,0,TIM2_IRQn,2);	//抢占1，子优先级0，组2				
	}
	void USART1_init()
	{
		USART_InitTypeDef USART_InitStructure;
		/* USARTx configured as follow:
					- BaudRate = 115200 baud  
					- Word Length = 8 Bits
					- One Stop Bit
					- No parity
					- Hardware flow control disabled (RTS and CTS signals)
					- Receive and transmit enabled
		*/
		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		STM_EVAL_COMInit(COM1, &USART_InitStructure);
	}
	void USART_Send(uint16_t Data)
	{
		 uint16_t Hvalue,Lvalue;
						if(Data<=255)
						{
							USART_SendData(EVAL_COM1,Data);//Data_RECE[i] SData_Second_Last[i]Data_One_Last[i]
							while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
						}
					 if(Data>255)
					 {
						 Hvalue=Data/0x100;
						 Lvalue=Data%0x100;
						 USART_SendData(EVAL_COM1,Hvalue);//Data_RECE[i] SData_Second_Last[i]Data_One_Last[i]
						 while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
						USART_SendData(EVAL_COM1,Lvalue);//Data_RECE[i] SData_Second_Last[i]Data_One_Last[i]
						while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
					 }

	}

	/*void Generate_normal_data( )//产生常规数据
	{
		int j=0;
		for(j=0;j<=281;j++)
		{
			 Normal_data[j]=750;
		}
	*/
	/**
		* @brief  Main program.
		* @param  None
		* @retval None
	*/
	void Data_collection(void)
	{ //这里检测i j k电平高低，高电平对应不导通，即对应管脚无输入信号
		u8_t i = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_15); //前进
		u8_t j = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5); //后退
		u8_t l = GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_4); //标定
		u8_t k = GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_5); //停车
//		u8_t Joystick_Up = STM_EVAL_PBGetState(BUTTON_UP);
//		u8_t Joystick_Down = STM_EVAL_PBGetState(BUTTON_DOWN);
//		u8_t Joystick_Left = STM_EVAL_PBGetState(BUTTON_LEFT);
//		u8_t Joystick_Right = STM_EVAL_PBGetState(BUTTON_RIGHT);
		extern char text3[40];
//	 STM_EVAL_PBGetState(BUTTON_KEY);
//	 STM_EVAL_PBGetState(BUTTON_TAMPER);
//	 STM_EVAL_PBGetState(BUTTON_WAKEUP);
//   STM_EVAL_PBGetState(BUTTON_RIGHT);
//   STM_EVAL_PBGetState(BUTTON_LEFT);
//   STM_EVAL_PBGetState(BUTTON_UP);
//   STM_EVAL_PBGetState(BUTTON_DOWN);		
		if(i==0 && j==1 && k==1 && l==1)   //前进 
		{
//			STM_EVAL_LEDOff(LED4);
//			STM_EVAL_LEDOn(LED1);
			m_bVehicleForward=TRUE;
		  m_bStopFlag=FALSE;
			m_bVehicleBackward=FALSE;
			BD_ENABLE=0x00;
		}
		if(j==0 && i==1 && k==1 && l==1)   //停车//现在改为后退_YZ_2015/12/14
		{
//			STM_EVAL_LEDOff(LED4);
//			STM_EVAL_LEDOff(LED1);
			m_bVehicleForward=FALSE;
			m_bStopFlag=FALSE;
			m_bVehicleBackward=TRUE;
			BD_ENABLE=0x00;
		}

		if(l==0 && i==1 && j==1 && k==1)   // 标定_注意：按下标定键同时会停车
		{
//			STM_EVAL_LEDOn(LED4);
//			STM_EVAL_LEDOff(LED1);
			BD_ENABLE = 0x01;
			m_bStopFlag=0x01;
			Scan_Num=0;
			BD_Stack_Num=0x00;
      Vehicle_Stack_Full=0x00;
			Vehicle_Stack_Num=0x00;
			m_bVehicleForward=FALSE;
			m_bVehicleBackward=FALSE;
			printf("BD111111");
		}
		
		if((BD_ENABLE==FALSE && m_bVehicleForward==FALSE && m_bVehicleBackward==FALSE) || (k==0 && i==1 && j==1 && l==1))  //假如不前进不后退则认为车停止 或者当k为低电平时认为停车_YZ_2016/03/22
		{
			BD_ENABLE=0x00;  //按下停车键会自动结束标定
			m_bStopFlag=TRUE;
			m_bVehicleForward=FALSE;
			m_bVehicleBackward=FALSE;
		}

//		if(i==0 && j==1 && k==0 && Key_Button_Push==FALSE){
//			Key_Button_Push = TRUE;
//			LCD_Change_Mode_InTime=Time_count;
//		}
//		
//		if(Key_Button_Push == TRUE && i==1 && k==0 && j==1){
//			Key_Button_Push = FALSE;
//			LCD_Change_Mode_FiTime=Time_count;
//			LCD_Change_Mode_dTime=fabs(LCD_Change_Mode_FiTime-LCD_Change_Mode_InTime);
//		}
		
//		if(LCD_Change_Mode_dTime >= dTime_Value){
//			if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
//				 LCD_Interface1=FALSE;
//  		   LCD_Interface2=TRUE;
//			}
//			else if(LCD_Interface2==TRUE && LCD_Interface1==FALSE){
//				 LCD_Interface2=FALSE;
//  		   LCD_Interface1=TRUE;
//			}

//			LCD_Change_Mode_dTime=0;
//			LCD_Change_Mode_InTime=0;
//			LCD_Change_Mode_FiTime=0;
//			LCD_Clear(Black);  
//		}

//if(LCD_Interface2==FALSE && LCD_Interface1==TRUE){	
//		LCD_SetBackColor(Blue);
//		LCD_SetTextColor(White);
//// sprintf(text3, "BD_ENABLE=%06u", BD_ENABLE);
////LCD_DisplayStringLine(Line4, (uint8_t*)text3);
//// sprintf(text3, "InTime=0x%04u", LCD_Change_Mode_InTime);
////LCD_DisplayStringLine(Line1, (uint8_t*)text3);
//// sprintf(text3, "time=0x%04X", Time_count);
////LCD_DisplayStringLine(Line2, (uint8_t*)text3);	
//}	


//		 sprintf(text3, "Wakeup=0x%04u", k);
// LCD_DisplayStringLine(Line0, (uint8_t*)text3);
//		 sprintf(text3, "Tamper=0x%04u", j);
// LCD_DisplayStringLine(Line1, (uint8_t*)text3);
//		 sprintf(text3, "Key=0x%04u", i);
// LCD_DisplayStringLine(Line3, (uint8_t*)text3);
		
//		if((Joystick_Right==1 && Joystick_Left==0 && Joystick_Up==0 && Joystick_Down==0) || (Joystick_Right==0 && Joystick_Left==0 && Joystick_Up==0 && Joystick_Down==0) )   // LCD显示数据界面1
//		{
//    LCD_Interface1=TRUE;
//		LCD_Interface2=FALSE;
//		}
		
//		if((Joystick_Right==0 && Joystick_Left==1 && Joystick_Up==0 && Joystick_Down==0))   // LCD显示数据界面1
//		{
//    LCD_Interface1=FALSE;
//		LCD_Interface2=TRUE;
//		}
		
/*
		if(k==1 && i==0 && j==1)
		{
			STM_EVAL_LEDOn(LED4);
			STM_EVAL_LEDOn(LED1);
	    BD_ENABLE = 0x01;
			m_bStopFlag=0x01;
			m_bVehicleForward=FALSE;
			m_bVehicleBackward=FALSE;
		}
*/	
		 Find_REF(); //找基准(提取基准点信息并且判断接力)

//*********************只有完成了数据的交叉存储后才执行下面的部分_2015/08/25*********//取消_YZ_2015/11/25		
 
	   data_rely() ; 

	}
	void Find_REF()
	{
		int b;//找到空格的标志位
		int x,y,n;//存储去掉空格的临时数组变量
		int i=0;
		int c,t;
	//	int c,t,j,e,g;
		//*********************************** 新增加的变量__YZ_2015/08/10 **************************************************************************************
		u16_t h; //用来计算拟合和角度平均值的参量
		u16_t w;             // 用来记录一包数据里传感器条数的序号(例如对于角分辨率为0.166667°的数据包，w取值为[1,649])
//		u16_t w_change;      // 用来记录发生突变时的第一条线段的序号   
//		u16_t q = 0;         // 用来记录扫描到一个参考点后，落在参考点上的线段条数
//		u16_t w1 = 0;       // 用来表示扫描的开始条数
//		u16_t w2 = 281;     // 用来表示扫描的截至条数
//		int data[281];      // 初始的扫描数据（整型）
		double Ave_Num;        //用来计算拟合和角度平均值的参量
//		double hh;
//		int data_temp[281][4]={0}; //临时储存所有参考点信息的变量
		double Angle_HD,DATA_ADD;    //夹角弧度
		//*********************************** 新增加的变量__YZ_2015/12/10 **************************************************************************************
 //		u8_t BD_Button;     //记录按键信号的变量
	u16_t Scan_Filter_Num; //针对第一次扫描寻找数值异常点的循环计数参量
		//********************************************************************************************************************************************
	//s16_t DA;//Detection angle物体进入时，检测的进入的角度
	//	s16_t DA1;//突变发生横坐标
  //	s16_t DA2;//突变结束横坐标
	//s16_t DA3;//基准纵坐标
		u8_t  Tubian=0; //找到突变标志位
	//	double J_Angle;   //检测到基准的2束光夹角
	//	double JZ_Dia;   //检测到基准的2束光长度差
    u8_t jc;     //记录基准个数
		//static u16_t Pos_num[209]={0};//测量距离
	  //static int a;	
		/*Initialize LCD and Leds */ 
		//TIM3_Int_Init(3000-1,8399);//定时中断10ms（5000*8400)/84000=500ms
		/* check if any packet rece0ived */
			 
		
			if(Interrupt_flag==1)//定时中断打开客服端，开始接收数据
			 {	 
					tcp_echoclient_connect();
				 if((IC_flag==IC_FALSE)&&(IC_time<=3) ) 
				 {
						IC_time++;
				 }
				 else if((IC_flag==IC_FALSE)&&(IC_time>3) ) 
				 {
					 USART_Send(0x00);
					 IC_time=0;
				 }
				 Interrupt_flag=0;
			 }

			 if(Data_correct_flag==1)//检验包尾成功后，开始应用处理数据
			 {    
						x=0;
						y=1;
			//			j=0;
						i=0;
						n=0;
						t=0;
		//				e=0;
						h=0;
			//			g=0;
						Ave_Num=0;
					  RD_time=0;

			for(y=0;y<(DATA_NUM-13);y++)//得到最终处理数据，查找数据空格，然后将其转换为整型数据
			{		
			if (Data_RECE[y]==' ')  
					 {   
									b=y-x-1;
									x=y+1;
								if(b==0) 
								 {

										 str_temp[n]=Tr(Data_RECE[y-1])&(0xFFFF);  //Size_str_temp-1-n 目的是对传感器发出数据进行逆存储，从而满足传感器扫描方向与一开始假定方向相反的情况_YZ_2015/12/14
									 
										n++;
										b=0;
								 }
								 if(b==1)
								 {


										 str_temp[n]=(Tr(Data_RECE[y-1])&(0xFFFF))|((Tr(Data_RECE[y-2])&(0xFFFF))<<4);
									
										 
										 n++;
									 b=0;
								 }
								if(b==2)
								{

								str_temp[n]=(Tr(Data_RECE[y-1])&(0xFFFF))|((Tr(Data_RECE[y-2])&(0xFFFF))<<4)
														 |((Tr(Data_RECE[y-3])&(0xFFFF))<<8);
									 
									 n++;
									 b=0;
								}		
								if(b==3)		
							 {	
				 
								str_temp[n]=(Tr(Data_RECE[y-1])&(0xFFFF))|((Tr(Data_RECE[y-2])&(0xFFFF))<<4)|
														((Tr(Data_RECE[y-3])&(0xFFFF))<<8)|((Tr(Data_RECE[y-4])&(0xFFFF))<<12);

									 n++;
									 b=0;
							 }
					 } 
			}	 

			
			
	//*******************************************************************************************************************************
	//*******************************************************************************************************************************
	//*******************************************************************************************************************************
	//*************************************直接引入Charge_Ref()进行参考点数据提取以及接力分析****************************************
	//*******************************************************************************************************************************
	//*******************************************************************************************************************************
	//*******************************************************************************************************************************
    
    //BD_Flag=Debounce(BD_Button);//通过采集标定按钮的信号，并经过去抖处理来判断是否处于标定状态
		
		
	//****************************************修改后的突变条件 _2015/08/12****************************************************	
				//if((fabs(str_temp[j]-str_temp[j+1])>JUDGE_DATA_THRESHOLD)&&(fabs(str_Ver[j+1]-str_Ver[j+2])<LESS_DATA_DES)&&(NORMAL_DATA_MIN<str_Ver[j+1]<NORMAL_DATA_MAX)) //基准范围内有跳变
				//L小于20米，垂直距离小于10m;突变处距离大于200mm，突变后，前后两线距离小于5mm
	
	//*********************只有完成了数据的交叉存储后才执行下面的部分_2015/08/25*************************************		

	t = 0;
	c = 0;
	jc = 0; 
	jc_origin=0;
//if((LCD_Interface2==TRUE)&&(LCD_Interface1==FALSE)&&((Time_count%1)==0)&&(LCD_SENSOR_Image==TRUE)){		
//	LCD_Clear(Black);                    /* Clear graphical LCD display        */
//}

	if(1){
		
//将原始数据储存在inverse数组里，方便之后刷新扫描方向与数组序号的关系	__YZ_20160324
	for (Scan_Filter_Num=0; Scan_Filter_Num<=Size_str_temp; Scan_Filter_Num++)
	{		
		str_temp_inverse[Scan_Filter_Num]=str_temp[Scan_Filter_Num];  
	}	
	
	//这里由于0.5角分辨率数据长度为真实值1/2，所以【首先】应该对采集数据进行*2处理	__YZ_20160324
	for (Scan_Filter_Num=0; Scan_Filter_Num<=Size_str_temp; Scan_Filter_Num++)
	{
		if(Scan_Angle_Resolution == 0.5){
		str_temp[Size_str_temp-Scan_Filter_Num]=2*str_temp_inverse[Scan_Filter_Num];
		}
		else{
		str_temp[Size_str_temp-Scan_Filter_Num]=str_temp_inverse[Scan_Filter_Num];
		}
	}	
		
		
	for (Scan_Filter_Num=0; Scan_Filter_Num<=Size_str_temp; Scan_Filter_Num++)  //第一次扫描本包数据，找出纵向距离异常点(超出阈值范围),并用阈值对应数据进行刷新覆盖_YZ_2015/12/10
	{
		//printf("%u \n",str_temp[Scan_Filter_Num]);
		Angle_HD = (Scan_In_Angle + (Scan_Filter_Num)*Scan_Angle_Resolution) / 180.0 * 3.1415926;
		str_Ver[Scan_Filter_Num] = str_temp[Scan_Filter_Num] * sin(Angle_HD);//计算每包数据的数据点的纵向距离储存到str_Ver当中
			
			if(str_Ver[Scan_Filter_Num]>=STR_DATA_MAX_Vertical || str_Ver[Scan_Filter_Num] == 0)
			{
				str_Ver[Scan_Filter_Num]=STR_DATA_MAX_Vertical; 
				str_temp[Scan_Filter_Num]=STR_DATA_MAX_Vertical/sin(Angle_HD);
			}
			
			//	printf("str_temp[%u]=%u\n",Scan_Filter_Num,str_temp[Scan_Filter_Num]);
		
	} //经过刷新后数据均处于正常范围，因此宏STR_DATA_MAX可不再使用_YZ_2015/12/10
	
	

		
	
	for (w = 2; w <= Size_str_temp; w++)
	{ 
//		if((LCD_Interface2==TRUE)&&(LCD_Interface1==FALSE)&&((Time_count%1)==0)&&(LCD_SENSOR_Image==TRUE)){
//    LCD_DrawLine(0,w,str_Ver[w] /10,Vertical);
//		}
		
//		if ((str_temp[w] == 0x00)||(str_temp[w]>=4000)) //这里不再使用这种覆盖数据异常点的方式，而改用纵坐标过滤法__YZ_2015/12/10
//		{
//			str_temp[w] = str_temp[w - 1]; //调试阶段对0值处理，待修改__YZ__2015/11/23
//		}
//		USART_Send(str_temp[w]);
		
		if(Tubian==1 && t==0 && c==0) //这里将Tubian置零的目的是防止在找到t之后并未找到c然后影响到之后其他参考点的扫描等情况_YZ_2015/12/10
		{
			Tubian=0;
		}
		if(Tubian==1 && c==0 && w>=(t+ct_MAX)) //如果在找到t后经过ct_MAX条线还未找到c则自动重新开始寻找t
		{
			Tubian=0;
			t=0;
		}
		
		
		
       // USART_Send(Tubian);
//		if ((Tubian == 0 && str_temp[w - 1]<4000 && str_temp[w - 2]<4000 && str_temp[w]<4000 && str_temp[w + 1]<4000 && 
//			 (str_temp[w - 1] - str_temp[w])>JUDGE_DATA_THRESHOLD) && ((str_temp[w] - str_temp[w + 1]) < LESS_DATA_DES) && 
//		   ((str_temp[w + 1] - str_temp[w + 2]) < LESS_DATA_DES))


//			printf("str_temp[%u]=%u\n",w,str_temp[w]);
		
		
		
	  	if (
	  	(Tubian == 0                                                  //--->这是判断体1-1
	  	&& (str_temp[w - 1] - str_temp[w])>JUDGE_DATA_THRESHOLD)      //--->这是判断体1-2
	  	&& (fabs(str_temp[w] - str_temp[w + 1]) < LESS_DATA_DES)      //--->这是一个判断体2   //这里还是要加上绝对值__YZ_20160322
	  	&& (fabs(str_temp[w + 1] - str_temp[w + 2]) < LESS_DATA_DES) //--->这是一个判断体3   //这里还是要加上绝对值__YZ_20160322
			&& str_Ver[w] > NORMAL_DATA_MIN                              //--->这是一个判断体4 
			&& str_Ver[w] < NORMAL_DATA_MAX)                            //--->这是一个判断体5  
	   {
//				printf("w=%u\n",w);
//			printf("str_temp[w]=%u ",str_temp[w]/2);
//			printf("str_temp[w + 1]=%u ",str_temp[w + 1]/2);
//			printf("str_temp[w + 2]=%u ",str_temp[w + 2]/2);
//			printf("str_temp[w - 1]=%u \n",str_temp[w - 1]/2);
			DATA_ADD = 0;
		 //STM_EVAL_LEDOn(LED2);
			for (i = w; i <= w + 3; i++)
			{
				Angle_HD = (Scan_In_Angle + (i-1)*Scan_Angle_Resolution) / 180.0 * 3.1415926;
				str_Ver[i] = str_temp[i] * sin(Angle_HD);//计算对应数据点的纵向距离储存到str_Ver当中
//				str_Lev[i] = str_temp[i] * cos(Angle_HD);//计算对应数据点的横向距离储存到str_Lev当中_这里没有用到故不进行计算_YZ_2015/12/10
				DATA_ADD = DATA_ADD + str_Ver[i];
	
			}
			DATA_ADD = DATA_ADD / 4;

			if ((NORMAL_DATA_MIN < DATA_ADD) &&( DATA_ADD < NORMAL_DATA_MAX))   //2.5m~3m
			{
				//DA1=str_Lev[w];//?????????,???.
				t = w;
//				printf("t=%u\n",t);
					  DATA_TO_SCREEN=y;
			//	USART_Send(t);
//				USART_Send(0xAA);
				Tubian = 1;
			}
			else
				Tubian = 0;
		}
//		if ((Tubian == 1 && str_temp[w - 1]<4000 && str_temp[w - 2]<4000 && str_temp[w]<4000 && str_temp[w+1]<4000 &&
//			 (str_temp[w + 1] - str_temp[w])>JUDGE_DATA_THRESHOLD) && ((str_temp[w] - str_temp[w - 1])< LESS_DATA_DES) && 
//		   ((str_temp[w - 1] - str_temp[w - 2]) < LESS_DATA_DES))
			 if (
			 (Tubian == 1                                                  //--->这是判断体1-1
			 && (str_temp[w + 1] - str_temp[w])>JUDGE_DATA_THRESHOLD)      //--->这是判断体1-2
			 && (fabs(str_temp[w] - str_temp[w - 1])< LESS_DATA_DES)       //--->这是一个判断体2   //这里还是要加上绝对值__YZ_20160322
			 && (fabs(str_temp[w - 1] - str_temp[w - 2]) < LESS_DATA_DES) //--->这是一个判断体3   //这里还是要加上绝对值__YZ_20160322
			 && str_Ver[w] > NORMAL_DATA_MIN                              //--->这是一个判断体4 
			 && str_Ver[w] < NORMAL_DATA_MAX)                             //--->这是一个判断体5  
		{

			DATA_ADD = 0;
			for (i = w-3; i <= w; i++)
			{
				Angle_HD = (Scan_In_Angle + (i - 1)*Scan_Angle_Resolution) / 180.0 * 3.1415926;
				str_Ver[i] = str_temp[i] * sin(Angle_HD);//????,???
				str_Lev[i] = str_temp[i] * cos(Angle_HD);//????
				DATA_ADD = DATA_ADD + str_Ver[i];
				
			}
			DATA_ADD = DATA_ADD / 4;
			if ((NORMAL_DATA_MIN < DATA_ADD) &&( DATA_ADD < NORMAL_DATA_MAX))   //2.5m~3m
			{
				c = w;
//				printf("c=%u\n",c);
				//USART_Send(c);
			//	USART_Send(0xBB);
				Tubian = 0;	
        jc_origin++; //这里只要得出一组c,t后，原始突变点计数参量jc_origin就要加一_YZ_2015/12/10				
				Tubian_t[jc_origin]=t;
				Tubian_c[jc_origin]=c;
				
				if(jc_origin>1 && (Tubian_t[jc_origin]-Tubian_c[jc_origin-1])<5 && Tubian_t[jc_origin] != 0 && Tubian_c[jc_origin-1] != 0)
				{
					t=Tubian_t[jc_origin-1]; //这里是针对参考点处反光的情况，对于参考点处的反光会将一个参考点一分为2，但得到的前者c与后者t会十分接近，这里可以将这种情况得到的两组c,t合二为一，整理成一个正常参考点的c,t.
				}
				//cout << "t= " << t << endl;
				//cout << "c= " << c << endl;
		/*		cout << "str_temp[13]=  " << str_temp[10] << endl;*/
				
			}
					
				//  USART_Send(0xAA);
			//***************************************2015/08/12************************************
		//	if (t != 0 && c != 0 && 10 <= c - t && c - t <= 40 )
		//***************************************考虑了参考点半径后对参考点信息的过滤_YZ_2015/11/25************************************
			if(t != 0 && c != 0)  //这里只有在得到一组c,t后才会进行计算，减小程序的计算量_YZ_2015/12/10
			{
				TC_distance=pow(pow(str_temp[t],2)+pow(str_temp[c],2)-2*str_temp[t]*str_temp[c]*cos((c-t)*Scan_Angle_Resolution/180*3.1415926),0.5);
				
//					printf("t=%u\n",t);
//					printf("c=%u\n",c);
//				  printf("TC_distance=%f",TC_distance);
				
			}
			if (t != 0 && c != 0 && ct_MIN <= (c - t) && (c - t) <= ct_MAX &&((R_length)<TC_distance) && (TC_distance<(4*R_length))) 
			 //这里是当放宽TC_distance的过滤条件(因为经过验证发现相同半径参考点在不同角度扫描后得到的计算直径波动较大)_YZ_2015/12/10
			{
	//			J_Angle = (fabs((c - t) / 2.0) / 180 * 3.141592653);
			//	JZ_Dia = str_temp[c] * sin(J_Angle);
				//cout << "J_Angle= " << J_Angle << endl;
				//cout << "JZ_Dia= " << JZ_Dia << endl;
//				if ( 3 <= c - t&&c - t <= 25 && (fabs(JZ_Dia - 2 * R_length)) < LESS_DATA_DES)
//				{	
				Ave_Num = 0;
					h = 0;
					senor[jc].angle = (t-1 + c-1) / 2.0 * Scan_Angle_Resolution + Scan_In_Angle;
					senor[jc].angle = senor[jc].angle / 360.0 * 2 * 3.1415926;
					for (i = t; i <= c; i++)             //????????
					{		
				//		STM_EVAL_LEDOn(LED2);
						Ave_Num = Ave_Num + cos(fabs(((i - 1)*Scan_Angle_Resolution + Scan_In_Angle) / 360.0 * 2 * 3.1415926 - senor[jc].angle))*str_temp[i] + sqrt(pow(R_length, 2) - pow(fabs(str_temp[i] * sin(fabs(((i - 1)*Scan_Angle_Resolution + Scan_In_Angle) / 360.0 * 2 * 3.1415926 - senor[jc].angle))), 2));
							h = h + 1;				
					}
					senor[jc].polar = Ave_Num / h;	

					//USART_Send(t);
//					  printf("jc=%u",jc);
//          	printf("t=%u\n",t);
//          	printf("c=%u\n",c);
					//USART_Send(senor[jc].polar);
					jc++;
//					c = 0;   //这里的c与t归零应放在判断外部_YZ_2015/11/25
//					t = 0;			
			}			
					c = 0;
					t = 0;
		}
	}
}
//	        USART_Send(jc);  //输出最终扫描到的参考点数目
    printf("jc=%u\n",jc);
//					USART_Send(0xff);
	
//if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){
//	  DATA_TO_SCREEN=jc;
//		sprintf(text2, "NUM_J=0x%04X", DATA_TO_SCREEN);
////#ifdef __USE_LCD
//    LCD_SetTextColor(White);
//    LCD_DisplayStringLine(Line8, (uint8_t*)text2);
////#endif // __USE_LCD
//}
//	  DATA_TO_SCREEN=Time_count;
//		sprintf(text2, "Time_count=0x%04X", DATA_TO_SCREEN);
//	  LCD_DisplayStringLine(Line5, (uint8_t*)text2);
////#ifdef __USE_LCD
//    GLCD_SetTextColor(Blue);
//		GLCD_DisplayString(9, 1, __FI,  (unsigned char *)text2);
////#endif // __USE_LCD
////

//	 Time_count=Time_count+1;
//	  if(Time_count==65530){
//			Time_count=0;
//		}


//		sprintf(text2, "t1=0x%04u,c1=0x%04u",Tubian_t[1],Tubian_c[1]);
////#ifdef __USE_LCD
//    GLCD_SetTextColor(Blue);
//		GLCD_DisplayString(0, 1, __FI,  (unsigned char *)text2);
////#endif // __USE_LCD
//Tubian_t[1]=0;
//Tubian_c[1]=0;

//			sprintf(text2, "t2=0x%04u,c2=0x%04u",Tubian_t[2],Tubian_c[2]);
////#ifdef __USE_LCD
//    GLCD_SetTextColor(Blue);
//		GLCD_DisplayString(1, 1, __FI,  (unsigned char *)text2);
////#endif // __USE_LCD
//Tubian_t[2]=0;
//Tubian_c[2]=0;

//		sprintf(text2, "t3=0x%04u,c3=0x%04u",Tubian_t[3],Tubian_c[3]);
////#ifdef __USE_LCD
//    GLCD_SetTextColor(Blue);
//		GLCD_DisplayString(2, 1, __FI,  (unsigned char *)text2);
////#endif // __USE_LCD
//Tubian_t[3]=0;
//Tubian_c[3]=0;
//	

	//上面已经将扫描区域数据分析完毕，已经存储了所有过滤后的参考点信息，下面进行接力的判断
	//***************************************3个参考点新增接力条件2015/08/18************************************
	
					if((jc==3 || jc==4) && Scan_Done==0 && JL_x2_Ready==0x01 && m_bVehicleForward==TRUE) //正向接力
					{
						j1=1; //如果接力条件满足，且区域内有三个参考点，那么将需要挑选的参考点序号（此时区域内有三个参考点序号为0 1 2）为1 2
						j2=2;
						jc_Record=jc;
						Scan_Done=0x01;
						Scan_Num=Scan_Num+1;
						JL_x2_Ready=0x00; //此时接力计数变量Scan_Num+1，之后JL_x2_Ready = 0x00，意味着接力准备阶段结束，这就保证了循环内容只在接力后的第一次运算中执行。
					  Cal_num=0x01; //说明这次的位置信息应当作为标准位置信息代入之后的坐标计算当中去。
//					  STM_EVAL_LEDOn(LED2);
						//USART_Send(0xAA);
						//USART_Send(Scan_Done);

					}
					
					if(jc_Record==3 && jc==4 &&JL_x2_Ready == 0)
					{
						jc_Record=4;
					}
					
					if(jc==3 && jc_Record==2 && m_bVehicleBackward==TRUE && JL_x2_Ready==0x00)
					{
						j1=1;
						j2=2;
						jc_Record=3;
					}
					
					
					if(j2==2 && j1==1 &&jc==3 && m_bVehicleBackward==TRUE && JL_x2_Ready == 0x01 )
					{
            j1=0;
            j2=1;
						Scan_Done =0x01; //此时说明返回过程接力完成
						Scan_Num=Scan_Num-1;
						JL_x2_Ready = 0x00;
					}			
				//	if(j==2 && Scan_Done ==0x01 && JL_x2_Ready == 0x00)
					if((jc==2 || (jc>=3 && jc_Record ==0) || (jc_Record==4 && jc==3) ) && JL_x2_Ready == 0x00) //又增加了两个情况(第二个和第三个)，第二个针对初始时刻就扫描到了3个或3个以上参考点的情况，第三个针对由四个参考点变为3个参考点的情况_YZ_2015/12/10
					{
						j1=0; //如果在Scan_Done == 1的情况下，扫描到的参考点数目为2，说明此时接力阶段已经完成。如果Scan_Done == 0，未经接力，
						j2=1; //因此，此时区域内的参考点就是我们现在需要使用的参考点。
						Scan_Done = 0x00;
						jc_Record=2;
					//	USART_Send(0xBB);
					//	USART_Send(Scan_Done);
					}
//if(LCD_Interface1==TRUE && LCD_Interface2==FALSE  ){					
//sprintf(text2, "Scan_Num = %04u", Scan_Num); 
////#ifdef __USE_LCD
//    
//		LCD_DisplayStringLine(Line0, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//sprintf(text2, "Scan_Done = %04u", Scan_Done); 
////#ifdef __USE_LCD
//    
//		LCD_DisplayStringLine(Line2, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//	sprintf(text2, "data_x2 = %6f", data_x2);
////#ifdef __USE_LCD
//		LCD_DisplayStringLine(Line3, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//	

//}
	//************************针对传感器未完成一个完整接力过程时有正向变为反向运动所出现的接力情况(测试)_2015/08/21**************************************************************************
			if(Scan_Done==0x01 &&( jc==3 || jc==4)  && JL_x2_Ready ==0x00 && m_bStateBefore ==0x01 && m_bVehicleForward==FALSE && m_bVehicleBackward==TRUE )
			{
				Scan_Done = 0x00;  //假如当前时刻运动方向与之前一个时刻方向相反，并且此时Scan_Done==0x01(说明完整接力阶段未结束)，那么自动认为此时传感器已经进入提前反向接力阶段
			}
			
  //************************针对传感器未完成一个完整接力过程时有反向变为正向运动所出现的接力情况(测试)_2015/08/21**************************************************************************
			if(Scan_Done==0x01 &&( jc==3 || jc==4) && JL_x2_Ready ==0x00 && m_bStateBefore ==0x00 && m_bVehicleForward==TRUE && m_bVehicleBackward==FALSE )
			{
				Scan_Done = 0x00;  //假如当前时刻运动方向与之前一个时刻方向相反，并且此时Scan_Done==0x01(说明完整接力阶段未结束)，那么自动认为此时传感器已经进入提前反向接力阶段
			}

				//	USART_Send(Scan_Done);
				//	USART_Send(0xBB);
				
	}
}
					
//上面已经将扫描区域数据分析完毕，已经存储了所有过滤后的参考点信息，下面进行接力的判断
	//***************************************多个参考点新增接力条件2015/08/20************************************
				/*	if(jc==3 && Scan_Done==0 && JL_x2_Ready==0x01 && m_bVehicleForward==TRUE) //正向接力
					{
						j1=1; //如果接力条件满足，且区域内有三个参考点，那么将需要挑选的参考点序号（此时区域内有三个参考点序号为0 1 2）为1 2
						j2=2;
						Scan_Done=0x01;
						Scan_Num=Scan_Num+1;
						JL_x2_Ready=0x00; //此时接力计数变量Scan_Num+1，之后JL_x2_Ready = 0x00，意味着接力准备阶段结束，这就保证了循环内容只在接力后的第一次运算中执行。
					  Cal_num=0x01; //说明这次的位置信息应当作为标准位置信息代入之后的坐标计算当中去。
					}
					if(jc==3 && m_bVehicleBackward==TRUE && JL_x2_Ready == 0x01 )
					{
            j1=0;
            j2=1;
						Scan_Done =0x01; //此时说明返回过程接力完成
						Scan_Num=Scan_Num-1;
						JL_x2_Ready = 0x00;
					}			
				//	if(j==2 && Scan_Done ==0x01 && JL_x2_Ready == 0x00)
					if(j==2 && JL_x2_Ready == 0x00)
					{
						j1=0; //如果在Scan_Done == 1的情况下，扫描到的参考点数目为2，说明此时接力阶段已经完成。如筍can_Done == 0，未经接力，
						j2=1; //因此，此时区域内的参考点就是我们现在需要使用的参考点。
						Scan_Done = 0x00; 
					}				
}
			 }
*/
			 
		//**************************************************************************************************
			 
	void data_rely(void)     //数据计算
	{  double a2,ANGLE_cosr,ANGLE_sinr,ANGLE_cosb,ANGLE_sinb,sina1,sinb1,cosa1,cosb1;//夹角bx和r的正弦余弦值
		 double L12,L11,L22,SENSON_X,SENSON_Y,temp1,temp2,temp3,temp4,temp5,temp6,temp7;
     double SENSOR_X_OUTPUT,SENSOR_Y_OUTPUT;
		 u8_t Translation_Num=0; //坐标平移求和时的计数参量
		if(m_bStopFlag==0x01 && BD_ENABLE == 0x01) //车静止，且标定状态为0x01 标定
		  // if( BD_ENABLE == 0x01)
				 {  //STM_EVAL_LEDOn(LED4);
           //USART_Send(BD_ENABLE);					 
						a1=senor[j1].angle;   
						b1=3.1415926-senor[j2].angle;   
						L1=senor[j1].polar;
						L2=senor[j2].polar;
						
						a1_Ref[Scan_Num]=a1; //存储每次接力后的标准位置，返回接力时用
						b1_Ref[Scan_Num]=b1;
						L1_Ref[Scan_Num]=L1;
						L2_Ref[Scan_Num]=L2;
					data_y1 = L1*sin(a1);		
					data_x1 = L2*cos(a1);	
					data_x2 = L2*cos(b1) + L1*cos(a1);	
					data_y2 = L1*sin(a1) - L2*sin(b1);
					 
					if(BD_Stack_Num<=(Stack_Size-1))
					{
					Standard_Position_y1_Stack_Sum[BD_Stack_Num]=data_y1;
					Standard_Position_x2_Stack_Sum[BD_Stack_Num]=data_x2;
					Standard_Position_y2_Stack_Sum[BD_Stack_Num]=data_y2;						
					BD_Stack_Num=BD_Stack_Num+1;
					}
					else if(BD_Stack_Num>=Stack_Size)
					{
					  for(BD_Stack_Replace_Num=1; BD_Stack_Replace_Num<=(Stack_Size-1); BD_Stack_Replace_Num++)
						{
							Standard_Position_y1_Stack_Sum[BD_Stack_Replace_Num-1]=Standard_Position_y1_Stack_Sum[BD_Stack_Replace_Num];
							Standard_Position_x2_Stack_Sum[BD_Stack_Replace_Num-1]=Standard_Position_x2_Stack_Sum[BD_Stack_Replace_Num];
						  Standard_Position_y2_Stack_Sum[BD_Stack_Replace_Num-1]=Standard_Position_y2_Stack_Sum[BD_Stack_Replace_Num];
						}
							Standard_Position_y1_Stack_Sum[Stack_Size-1]=data_y1;
							Standard_Position_x2_Stack_Sum[Stack_Size-1]=data_x2;
						  Standard_Position_y2_Stack_Sum[Stack_Size-1]=data_y2;
					}
					
					for(BD_Stack_Sum_Num=1; BD_Stack_Sum_Num<=BD_Stack_Num; BD_Stack_Sum_Num++)
					{
						Standard_Position_y1_Stack=Standard_Position_y1_Stack+Standard_Position_y1_Stack_Sum[BD_Stack_Sum_Num-1];
						Standard_Position_x2_Stack=Standard_Position_x2_Stack+Standard_Position_x2_Stack_Sum[BD_Stack_Sum_Num-1];
						Standard_Position_y2_Stack=Standard_Position_y2_Stack+Standard_Position_y2_Stack_Sum[BD_Stack_Sum_Num-1];
					}
					
					Standard_Position_y1_Stack=Standard_Position_y1_Stack/BD_Stack_Num;
					Standard_Position_x2_Stack=Standard_Position_x2_Stack/BD_Stack_Num;
					Standard_Position_y2_Stack=Standard_Position_y2_Stack/BD_Stack_Num;
					data_y1_Ref[Scan_Num]= Standard_Position_y1_Stack;
					data_x2_Ref[Scan_Num]= Standard_Position_x2_Stack;
					data_y2_Ref[Scan_Num]= Standard_Position_y2_Stack;
					Standard_Position_y1_Stack=0;
					Standard_Position_x2_Stack=0;
					Standard_Position_y2_Stack=0;
					 
					 
					 SCANNER_CENTER =data_y1;
					 LEFT_MIN_VALUE_BACK    = SCANNER_CENTER-50	 ;
					 RIGHT_MAX_VALUE_BACK   =  SCANNER_CENTER+50  ;
					 LEFT_MIN_VALUE_FRONT   = SCANNER_CENTER-50	 ;
					 RIGHT_MAX_VALUE_FRONT  =  SCANNER_CENTER+50  ;
					 
					 //将标定后的数据储存到SENSOR_DISTANCE_FRONT和SENSOR_DISTANCE_BACK当中
					 SENSOR_DISTANCE_FRONT=SCANNER_CENTER;
					 SENSOR_DISTANCE_BACK=SCANNER_CENTER;
		 
					 
					 // i++;
//					  if(L1!=0)
//					  BD_ENABLE=0x00; //在加入了接力之后，取消了原先的m_bStopFlag=0x00归零
					
				 }
				 
				 if	(m_bStopFlag==0x00 && BD_ENABLE == 0x00) 				 
				{
             
					if(m_bVehicleForward==TRUE && m_bVehicleBackward==FALSE) //传感器正向运动，所有接力点坐标信息需要计算
					{ 
						if(Scan_Num!=0&&Cal_num==1) //Cal_num==1说明之前标准位置信息已经储存，因此这里应当首先计算标准位置坐标
						{
							//sina1=sin(senor[j1].angle);
							//cosa1=cos(senor[j1].angle);
							//sinb1=sin(3.1415926-senor[j2].angle);
							//cosb1=cos(3.1415926-senor[j2].angle);
							
							sina1=sin(senor[j1].angle)*ANGLE_coscx-ANGLE_sincx*cos(senor[j1].angle);//由于已经进行过接力，因此标准位置角度需要修正(cx为前一时刻计算该位置的偏转角)
							cosa1=cos(senor[j1].angle)*ANGLE_coscx+ANGLE_sincx*sin(senor[j1].angle); //由于反三角函数会累加误差，这里对于需要修正的情况直接计算出其三角函数值
							sinb1=sin(3.1415926-senor[j2].angle)*ANGLE_coscx-ANGLE_sincx*cos(3.1415926-senor[j2].angle);
							cosb1=cos(3.1415926-senor[j2].angle)*ANGLE_coscx+ANGLE_sincx*sin(3.1415926-senor[j2].angle);
							cosa1_Ref[Scan_Num]=cosa1; //将标准位置信息储存，之后反向运动接力时直接提取。
							cosb1_Ref[Scan_Num]=cosb1;
							sina1_Ref[Scan_Num]=sina1;
							sinb1_Ref[Scan_Num]=sinb1;
					  	L1=senor[j1].polar;
						  L2=senor[j2].polar;
						  L1_Ref[Scan_Num]=L1;
						  L2_Ref[Scan_Num]=L2;	
							data_y1 = L1*sina1;		
							data_x1 = L2*cosa1;	
							data_x2 = L2*cosb1 + L1*cosa1;	
							data_y2 = L1*sina1 - L2*sinb1;
							data_x1_Ref[Scan_Num]=data_x1;
							data_y1_Ref[Scan_Num]=data_y1;
							data_x2_Ref[Scan_Num]=data_x2;
							data_y2_Ref[Scan_Num]=data_y2;
										//USART_Send(Cal_num);
							Cal_num =0x00; //归零的目的是只把接力后的第一个个位置作为接力后的标准位置，之后的每一个位置均作为下一时刻位置
						}
				
					}
					else if(m_bVehicleForward==FALSE && m_bVehicleBackward==TRUE) //对于反向运动，可以直接在每一时刻都调用之前储存的标准位置信息(Scan_Num来决定调用的标准位置信息序号)
					{
						if(Scan_Num==0) 
					{
					a1=  a1_Ref[Scan_Num];
					b1=  b1_Ref[Scan_Num];
					L1=  L1_Ref[Scan_Num];
					L2=  L2_Ref[Scan_Num];		
					data_y1 = L1*sin(a1);		
					data_x1 = L2*cos(a1);	
					data_x2 = L2*cos(b1) + L1*cos(a1);	
					data_y2 = L1*sin(a1) - L2*sin(b1);	
					}
					else
					{
					cosa1=cosa1_Ref[Scan_Num];
					cosb1=cosb1_Ref[Scan_Num];
					sina1=sina1_Ref[Scan_Num];
					sinb1=sinb1_Ref[Scan_Num];
					L1=  L1_Ref[Scan_Num];
					L2=  L2_Ref[Scan_Num];		
					data_y1 = L1*sina1;		
					data_x1 = L2*cosa1;	
					data_x2 = L2*cosb1 + L1*cosa1;	
					data_y2 = L1*sina1 - L2*sinb1;	
					}		
					}
					
				a2 = senor[j1].angle ;
				L11 = senor[j1].polar;//correct  346
				L22 = senor[j2].polar;//correct
			
			temp1 = pow(data_y2, 2) + pow(data_x2, 2);
			L12 = sqrt(temp1);
			temp2 = pow(L11, 2) + pow(L12, 2) - pow(L22, 2);
			temp3 = 2 * L11*L12;
			ANGLE_cosb = temp2 / temp3;//b为传感器带参考点一的连线和参考点一到参考点二的连线的夹角
			ANGLE_sinb = sqrt(1 - pow(ANGLE_cosb, 2));
			ANGLE_cosr = data_x2 / L12;//r为传感器一二连线与x轴的夹角
			ANGLE_sinr = data_y2 / L12;
			//设此时传感器于坐标原点连线和x轴夹角为celta,则有celta=b+r
			temp4 = (ANGLE_cosb*ANGLE_cosr) - (ANGLE_sinb*ANGLE_sinr);//cos(celta);
			temp5 = (ANGLE_sinb*ANGLE_cosr) + (ANGLE_cosb*ANGLE_sinr);//sin(celta)
			SENSON_X = L11*temp4;
			SENSON_Y = L11*temp5;                     //1750mm   1734mm   1750mm
					

			//设偏转角为cx 则应有cx=a2-celta
			
			
			ANGLE_sincx = sin(a2)*temp4 - cos(a2)*temp5;
			ANGLE_coscx = cos(a2)*temp4 + sin(a2)*temp5;
			if(Scan_Num==0){
      SENSOR_X_OUTPUT=SENSON_X;
			SENSOR_Y_OUTPUT=SENSON_Y;	
			}
			else if(Scan_Num==1){
			SUM_Translation_X0=data_x2_Ref[0];
			SUM_Translation_Y0=data_y2_Ref[0];
			SENSOR_X_OUTPUT=SENSON_X+SUM_Translation_X0;
			SENSOR_Y_OUTPUT=SENSON_Y+SUM_Translation_Y0;
			}
			else if(Scan_Num>1){
				for (Translation_Num=1; Translation_Num<Scan_Num;Translation_Num++){
					SUM_Translation_X=SUM_Translation_X+data_x2_Ref[Translation_Num];
					SUM_Translation_Y=SUM_Translation_Y+data_y2_Ref[Translation_Num];
				}
			SENSOR_X_OUTPUT=SENSON_X+SUM_Translation_X+SUM_Translation_X0;
			SENSOR_Y_OUTPUT=SENSON_Y+SUM_Translation_Y+SUM_Translation_Y0;	
      SUM_Translation_X=0;
			SUM_Translation_Y=0;	
			}
			
			if(Vehicle_Stack_Full==0)
			{
				
				Vehicle_Y_Stack[Vehicle_Stack_Num]=SENSOR_Y_OUTPUT;
				Vehicle_sincx_Stack[Vehicle_Stack_Num]=ANGLE_sincx;
				Vehicle_coscx_Stack[Vehicle_Stack_Num]=ANGLE_coscx;
				Vehicle_Stack_Num=Vehicle_Stack_Num+1;
				
				for(Stack_Sum_Num=0; Stack_Sum_Num<=(Vehicle_Stack_Num-1); Stack_Sum_Num++)
				{
					Vehicle_Output_Sum_Y=Vehicle_Output_Sum_Y+Vehicle_Y_Stack[Stack_Sum_Num];
					Vehicle_Output_Sum_sincx=Vehicle_Output_Sum_sincx+Vehicle_sincx_Stack[Stack_Sum_Num];
				  Vehicle_Output_Sum_coscx=Vehicle_Output_Sum_coscx+Vehicle_coscx_Stack[Stack_Sum_Num];
				}
				
				SENSOR_Y_OUTPUT=Vehicle_Output_Sum_Y/Vehicle_Stack_Num;
        ANGLE_sincx=Vehicle_Output_Sum_sincx/Vehicle_Stack_Num;
        ANGLE_coscx=Vehicle_Output_Sum_coscx/Vehicle_Stack_Num; 
        Vehicle_Output_Sum_Y=0;
        Vehicle_Output_Sum_sincx=0;
        Vehicle_Output_Sum_coscx=0;	

        if(Vehicle_Stack_Num==Stack_Size)
				{
					Vehicle_Stack_Full=TRUE;
				}					
				
			}
			
			if(Vehicle_Stack_Full==1)
			{
				for(Stack_Replace_Num=1;Stack_Replace_Num<=(Stack_Size-1);Stack_Replace_Num++)
				{
					Vehicle_Y_Stack[Stack_Replace_Num-1]=Vehicle_Y_Stack[Stack_Replace_Num];
					Vehicle_sincx_Stack[Stack_Replace_Num-1]=Vehicle_sincx_Stack[Stack_Replace_Num];
					Vehicle_coscx_Stack[Stack_Replace_Num-1]=Vehicle_coscx_Stack[Stack_Replace_Num];
				}
				Vehicle_Y_Stack[Stack_Size-1]=SENSOR_Y_OUTPUT;
				Vehicle_sincx_Stack[Stack_Size-1]=ANGLE_sincx;
				Vehicle_coscx_Stack[Stack_Size-1]=ANGLE_coscx;
				
				for(Stack_Sum_Num=0; Stack_Sum_Num<=(Stack_Size-1);Stack_Sum_Num++)
				{
					Vehicle_Output_Sum_Y=Vehicle_Output_Sum_Y+Vehicle_Y_Stack[Stack_Sum_Num];
					Vehicle_Output_Sum_sincx=Vehicle_Output_Sum_sincx+Vehicle_sincx_Stack[Stack_Sum_Num];
				  Vehicle_Output_Sum_coscx=Vehicle_Output_Sum_coscx+Vehicle_coscx_Stack[Stack_Sum_Num];
				}
				
				SENSOR_Y_OUTPUT=Vehicle_Output_Sum_Y/Stack_Size;
				ANGLE_sincx=Vehicle_Output_Sum_sincx/Stack_Size;
				ANGLE_coscx=Vehicle_Output_Sum_coscx/Stack_Size;
        Vehicle_Output_Sum_Y=0;
        Vehicle_Output_Sum_sincx=0;
        Vehicle_Output_Sum_coscx=0;
			}				
			 
			
			SENSOR_DISTANCE_TEMP[0] = SENSOR_Y_OUTPUT + 5000 * ANGLE_sincx ;     
			SENSOR_DISTANCE_TEMP[1] = SENSOR_Y_OUTPUT - 5000 * ANGLE_sincx ;    
			
		//下面是判断测距传感器采集数据的合法行�
		if(SENSOR_DISTANCE_TEMP[0] > SENSOR_DISTANCE_FRONT)
			n=SENSOR_DISTANCE_TEMP[0]-SENSOR_DISTANCE_FRONT;
		else
			n=SENSOR_DISTANCE_FRONT-SENSOR_DISTANCE_TEMP[0];
		if(n<=SENSOR_DISTANCE_ERR)//采集数据合法，存入SENSOR_DISTANCE_FRONT/BACK当中
		{//这里暂时不引入差值合法范围的下限，合理性待验证_YZ_2015/09/11
			SENSOR_DISTANCE_FRONT=SENSOR_DISTANCE_TEMP[0];  
		  SENSOR_DISTANCE_BACK=SENSOR_DISTANCE_TEMP[1];
		}
		else if(n>SENSOR_DISTANCE_ERR)
		{
			SENSOR_DISTANCE_ERR_CNT++;
			if(SENSOR_DISTANCE_ERR_CNT>9)//连续10次数据都大于阈值，则认为数据合法
			{
				SENSOR_DISTANCE_ERR_CNT=0;
				SENSOR_DISTANCE_FRONT=SENSOR_DISTANCE_TEMP[0];
				SENSOR_DISTANCE_BACK=SENSOR_DISTANCE_TEMP[1];
				
			}
			
		}
//				 printf("FRONT=%f\n",SENSOR_DISTANCE_FRONT);
//		     printf("BACK=%f\n",SENSOR_DISTANCE_BACK);
//		     printf("CENTER=%f\n",SCANNER_CENTER);
//		     printf("celta=%7f\n", asin(ANGLE_sincx)/3.1415926*180);
			
//      USART_Send(0xff);
//  		USART_Send(SENSOR_DISTANCE_FRONT);
//			USART_Send(0xee);
//   		USART_Send(SENSON_X);
//      USART_Send(0xff);
//			USART_Send(0xff);
//			USART_Send(0xff);
//  		USART_Send(SENSOR_DISTANCE_FRONT);

//		printf("X=%f\n",SENSON_X);
//		printf("Y=%f\n",SENSON_Y);
//		printf("celta=%7f\n", asin(ANGLE_sincx)/3.1415926*180);
		
//if(LCD_Interface1==TRUE && LCD_Interface2==FALSE){		
//		DATA_TO_SCREEN=SENSOR_DISTANCE_FRONT;
////		sprintf(text2, "celta=%7f", asin(ANGLE_sincx)/3.1415926*180);
//			sprintf(text2, "BACK_Y=%7f", SENSOR_DISTANCE_BACK);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line6, (uint8_t*)text2);
////#endif // __USE_LCD
//		
//		DATA_TO_SCREEN=SENSOR_Y_OUTPUT;
//		sprintf(text2, "SENSOR_Y=%7f", SENSOR_Y_OUTPUT);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line5, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//			sprintf(text2, "SENSON_X = %6f", SENSON_X);
////#ifdef __USE_LCD
//		LCD_DisplayStringLine(Line7, (uint8_t*)text2);
////#endif // __USE_LCD

//		DATA_TO_SCREEN=ANGLE_sincx;
//		sprintf(text2, "sincx=%7f",ANGLE_sincx);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line4, (uint8_t*)text2);
////#endif // __USE_LCD
//} 

//if(LCD_Interface1==FALSE && LCD_Interface2==TRUE && LCD_Vehicle_Posture==TRUE){		
//		DATA_TO_SCREEN=SENSOR_DISTANCE_FRONT;
////		sprintf(text2, "celta=%7f", asin(ANGLE_sincx)/3.1415926*180);
//			sprintf(text2, "BACK_Y=%7f", SENSOR_DISTANCE_BACK);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line6, (uint8_t*)text2);
////#endif // __USE_LCD

//		DATA_TO_SCREEN=data_y1;
//		sprintf(text2, "FRONT_Y=%7f",SENSOR_DISTANCE_FRONT);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line5, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//			DATA_TO_SCREEN=data_y1;
//		sprintf(text2, "FRONT_Y=%7f",SENSOR_DISTANCE_FRONT);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line5, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//} 


//if(LCD_Interface2==TRUE && LCD_Interface1==FALSE && LCD_Vehicle_Trail==TRUE ){
//  
//	LCD_DrawLine(SENSOR_Y_OUTPUT/10,SENSOR_X_OUTPUT/10,3,Horizontal);
//		sprintf(text2, "SENSOR_Y=%7f", SENSOR_Y_OUTPUT);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line9, (uint8_t*)text2);
////#endif // __USE_LCD
//	
//	sprintf(text2, "SENSOR_X= %6f", SENSOR_X_OUTPUT);
////#ifdef __USE_LCD
//		LCD_DisplayStringLine(Line8, (uint8_t*)text2);
////#endif // __USE_LCD
//}


//			USART_Send(0xee);
//			USART_Send(0xee);
//			USART_Send(0xee);
//   		USART_Send(SENSOR_DISTANCE_BACK);


//if(LCD_Interface2==TRUE && LCD_Interface1==FALSE && LCD_Vehicle_Posture==TRUE){
//  LCD_Clear(Black); 
//	LCD_DrawLine(SCANNER_CENTER/10,0,330,Horizontal);
//	LCD_DrawUniLine(SENSOR_DISTANCE_FRONT/10, 50, SENSOR_DISTANCE_BACK/10, 250);
//	
//		sprintf(text2, "SENSOR_CENTER=%7f", SCANNER_CENTER);
////#ifdef __USE_LCD
//  LCD_SetBackColor(Blue);
//  LCD_SetTextColor(White);
//LCD_DisplayStringLine(Line8, (uint8_t*)text2);
////#endif // __USE_LCD
//}



				}

//				a2 = senor[j1].angle ;
//				L11 = senor[j1].polar;//correct  346
//				L22 = senor[j2].polar;//correct
//			
//			temp1 = pow(data_y2, 2) + pow(data_x2, 2);
//			L12 = sqrt(temp1);
//			temp2 = pow(L11, 2) + pow(L12, 2) - pow(L22, 2);
//			temp3 = 2 * L11*L12;
//			ANGLE_cosb = temp2 / temp3;
//			ANGLE_sinb = sqrt(1 - pow(ANGLE_cosb, 2));
//			ANGLE_cosr = data_x2 / L12;
//			ANGLE_sinr = data_y2 / L12;
//			temp4 = (ANGLE_cosb*ANGLE_cosr) - (ANGLE_sinb*ANGLE_sinr);//cosr;
//			temp5 = (ANGLE_sinb*ANGLE_cosr) + (ANGLE_cosb*ANGLE_sinr);
//			SENSON_X = L11*temp4;
//			SENSON_Y = L11*temp5;                     //1750mm   1734mm   1750mm
//			temp6 = SENSON_Y / sqrt(pow(SENSON_X, 2) + pow(SENSON_Y, 2));
//			temp7 = SENSON_X / sqrt(pow(SENSON_X, 2) + pow(SENSON_Y, 2));
//			ANGLE_sincx = cos(a2)*temp6 - sin(a2)*temp7;
//			ANGLE_coscx = cos(a2)*temp7 + sin(a2)*temp6;

//			SENSOR_DISTANCE_FRONT = SENSON_Y + 0x1388 * ANGLE_sincx ;     
//			SENSOR_DISTANCE_BACK = SENSON_Y - 0x1388 * ANGLE_sincx ;    
////USART_Send(0xff);
//  //			USART_Send(SENSOR_DISTANCE_FRONT);
//		//		USART_Send(0xee);
//   		//	USART_Send(SENSOR_DISTANCE_BACK);

//	
			
//*********************************** 连接Rectify部分__YZ__2015/08/18 **************************************************************************************
		m_bDistanceSampleOk =TRUE; //说明数据采集与分析正常，可以进行接下来方向调整(纠偏)的部分
				
				
			//**************************储存当前时刻运动方向_2015/08/21************************************************************************	
			if(m_bVehicleForward==TRUE && m_bStopFlag == FALSE &&BD_ENABLE == FALSE)
			{
				m_bStateBefore=0x01;
			}
			if(m_bVehicleForward==FALSE && m_bStopFlag == FALSE &&BD_ENABLE == FALSE)
			{
				m_bStateBefore=0x00;
			}
			
			if(fabs(SENSON_X-data_x2)<50 && Scan_Done == 0x00 &&m_bVehicleForward==TRUE && m_bVehicleBackward==FALSE) //正向接力ready条件:这里的Scan_Done == 0x00 是保证只有在第一次满足坐标条件(SENSON_X-data_x2)>50 后，才会有JL_x2_Ready=0x01，防止程序将每次满足该条件的位置都当作接力后的第一个位置
			{ 
				if(data_x2!=0)
				JL_x2_Ready=0x01;

			}
		//***************************该条件针对反向接力额外情况（测试）_2015/08/21***********************************************************************	
			if(fabs(SENSON_X)<50 && Scan_Done == 0x00 &&m_bVehicleForward==FALSE && m_bVehicleBackward==TRUE) //反向接力ready条件:这里的接力ready条件是针对传感器正向运动过程中突然开始反向运动而没有经过jc由3-->2-->3的过程
			{ 
				if(data_x2!=0)
				JL_x2_Ready=0x01;

			}
	
        
} 

		void DATA_Dynamic()     //动态走车
	{
		Find_REF();
		//Charge_REF ();      //接力分析
	}

		
	void DATA_int()
	{
	  BD_ENABLE=0x01;
//	m_bVehicleForward =0x01;	
	}
		

	/**
		* @brief  Inserts a delay time.
		* @param  nCount: number of 10ms periods to wait for.
		* @retval None
		*/

	void Delay(__IO uint32_t nCount)
	{
		for(; nCount != 0; nCount--);
	}
	/*void Delay(uint32_t nCount)
	{
		timingdelay = LocalTime + nCount;  
		while(timingdelay > LocalTime)
		{     
		}
	}*/
	//定时器3中断服务程序	 
	void TIM3_IRQHandler(void)
	{ 		  
		 
		if(TIM3->SR&0X0001)//溢出中断
		{
			Interrupt_flag=1;
		}
		TIM3->SR&=~(1<<0);//清除中断标志位 	
		//Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
	}
	void TIM2_IRQHandler(void)//计数器2中断服务程序
	{
			if(TIM2->SR&0X0001)//溢出中断
			{
					 Tim_n++;
			}
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	/**
		* @brief  Updates the system local time
		* @param  None
		* @retval None
		*/
	void Time_Update(void)
	{
		LocalTime += SYSTEMTICK_PERIOD_MS;
		if(LocalTime>=100){
			LocalTime=0;
			clock_1s=1;
		}
			
	}

	/**
		* @brief  Initializes the STM324xG-EVAL's LCD, LEDs and push-buttons resources.
		* @param  None
		* @retval None
		*/
	void LCD_LED_BUTTON_Init(void)
	{
	#ifdef USE_LCD
		/* Initialize the STM324xG-EVAL's LCD */
		STM324xG_LCD_Init();
	#endif

		/* Initialize STM324xG-EVAL's LEDs */
//		STM_EVAL_LEDInit(LED1);
//		STM_EVAL_LEDInit(LED2);
//		STM_EVAL_LEDInit(LED3);
//		STM_EVAL_LEDInit(LED4);

		/* Leds on */
		//STM_EVAL_LEDOn(LED1);
//		STM_EVAL_LEDOff(LED2);
//		STM_EVAL_LEDOff(LED3);
//		STM_EVAL_LEDOff(LED4);
	 //STM_EVAL_LEDOff(LED4);
	//#ifdef USE_LCD
		/* Clear the LCD */
//		LCD_Clear(Black);

		/* Set the LCD Back Color */
//		LCD_SetBackColor(White);

		/* Set the LCD Text Color */
//		LCD_SetTextColor(Black);

		/* Display message on the LCD*/
	//#endif
//		STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_GPIO);
//		STM_EVAL_PBInit(BUTTON_TAMPER, BUTTON_MODE_GPIO);
//		STM_EVAL_PBInit(BUTTON_WAKEUP, BUTTON_MODE_GPIO);
	}




	/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
