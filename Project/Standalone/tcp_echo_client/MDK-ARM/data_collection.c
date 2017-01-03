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
	//��ͼ��_YZ_2015/11/25
#include "stm324xg_eval_lcd.h"
	/* Private typedef -----------------------------------------------------------*/
	/* Private define ------------------------------------------------------------*/
	#define SYSTEMTICK_PERIOD_MS  10   
	/* Private macro -------------------------------------------------------------*/
	 extern  __IO uint8_t EthLinkStatus;
	 u16_t IC_flag;//��Ƭ���ʹ��������ӱ�־
	 uint32_t IC_time=0;//��Ƭ���ʹ��������Ӵ����������������ⷢ�ʹ����־
	 u8_t OI_flag;//��׼����־λ
	 u8_t JL_flag;//������־λ
	 u8_t OI_normal_flag=PASS_EMPTY;//��⵽��׼����С��2��
//	 u16_t Normal_data[281]={2500};
	 //#define  Normal_h  5100;
	/* ��������ɨ���Ǵ������� ---------------------------------------------------------*/
		u8_t  Data_correct_flag;
		u16_t RD_time=0;//����������С��ĳ����ֵ�Ĵ���
		u8_t  Return_data_flag;//���������ݵ���ȷ�Ա�־
		u8_t  Data_RECE[1500];//��ȥ��ͷ�����ݴ洢����

	 //-----------------------�궨ʱ������L1��L2��a1��b2-----------------------------------------------------
		u16_t BD_Length1;//�궨ʱ���������׼2�ľ���
		u16_t BD_Length2=0;//�궨ʱ���������׼2�ľ���
		u16_t BD_Angle1;//�궨ʱ���������׼2�н�
		u16_t BD_Angle2=0;//�궨ʱ���������׼2�н�
		u16_t BDY;//�궨ʱ�������׼Զ�����
	//------------------------�н�������L11��L22��a2��b2-----------------------------------------------------
		u16_t MOVE_Length1;//�н������д����������һ����׼����
		u16_t MOVE_Length2=0;//�н������д���������ڶ�����׼����
		u16_t MOVE_Angle1;//�н������д��������һ����׼�н�
		u16_t MOVE_Angle2;//�н������д�������ڶ�����׼�н�

		u16_t SENSOR_DISTANCE_MID;//�궨ʱ������׼�߾���
		u16_t Pos_num=0;//��������
		u16_t Tim_n;//����ʱ��ʱ������2����Ĵ���
		u16_t Last_Pos_num=0;//�ϴβ�������
		
		s16_t str_Lev[Size_str_temp]={0};//ˮƽ����
		s16_t str_Ver[Size_str_temp]={0};//��ֱ����
	//	s16_t Data_One_Last[649]={0};//�洢�ϰ������������
		s16_t Data_Packet[5]={0};//��������5������
		//*********************************** �����ӵı���__YZ__2015/08/11 **************************************************************************************
    u8_t BD_Flag; //�궨��־������1Ϊ��ʼ�궨��0Ϊ���궨��	
		u16_t BD_Num=0; //�궨ʱ�ļ���������
		u8_t jc_origin=0; //���浱ǰ�������ҵ�������ͻ������ĸ���(��û�й��˸��ŵ��Լ��ҳ������Ĳο���)
		double gg;
		double avenum_a=0,avenum_b=0,avenum_c=0,avenum_d=0,	avenum_e=0,	avenum_f=0,		avenum_g=0,	avenum_h=0,avenum_i=0;	
		double sinave,cosave,sinaveangle,cosaveangle,sinaveave,cosaveave,sinave1,cosave1,sinaveangle1,cosaveangle,sinaveave,cosaveave;
		double a1,b1,L1,L2;
		double ANGLE_coscx=0,ANGLE_sincx=0;
		double SUM_Translation_X0=0,SUM_Translation_Y0=0;//��Ե�һ�ν�������������ƽ���ۼ���(�������ͳһ�����Ĵ�����λ��������Ϣ)
		double SUM_Translation_X=0,SUM_Translation_Y=0;//��Ե�һ�ν���֮��Ľ�������������ƽ���ۼ���(�������ͳһ�����Ĵ�����λ��������Ϣ)
		struct BD_data //�����ڱ궨��ʱ�򴢴�궨��Ϣ���ڱ�BD_Flag==FALSE֮�󽫽��ȡƽ��ֵ��
		{
			u32_t angle;
			u32_t polar;
		} BD_Sum[2];
	//s16_t Lengh_table[];


	//*********************************** Ϊ���������ӵı���__YZ__2015/08/18 **************************************************************************************
    const u16_t Ref_Num =10;    //������ʾ�����Ĵ���(���ۼƴ���������ǰ������ÿ����һ��Scan_Num+1�����˹���ÿ����һ��Scan_Num-1)
    u8_t Scan_Done=0;     //������ʾ�����Ѿ���ɵĲ���(0Ϊ)
    u16_t Cor_num=0;      //�����ֱ�Ӧ�ý��ο�����Ϣ�������׼λ�û��ǵ�ǰʱ��λ�õĲ���
    u16_t Cal_num=0;      //������ʾ�����������ݵĲ���	
		u8_t j1=0;              // ��ʾ����֪����ο�������ѡ�����ĵ�һ���ο�������
		u8_t j2=1;              // ��ʾ����֪����ο�������ѡ�����ĵڶ����ο�������
		u8_t JL_x2_Ready=0x00;   // ��ʾ�������Ѿ���������ٽ����򣬿��Կ�ʼ����
		u16_t Scan_Num=0; 
		double a1_Ref[Ref_Num],b1_Ref[Ref_Num],L1_Ref[Ref_Num],L2_Ref[Ref_Num],cosa1_Ref[Ref_Num],
					 cosb1_Ref[Ref_Num],sina1_Ref[Ref_Num],sinb1_Ref[Ref_Num];
		double data_x1_Ref[Ref_Num],data_y1_Ref[Ref_Num],data_x2_Ref[Ref_Num],data_y2_Ref[Ref_Num];
//*********************************** Ϊ���������ӵı���__YZ__2015/08/18 **************************************************************************************
   
  //const u16_t Ref_Num =10; //���òο��������
	//u8_t JL_x2_Ready=0x00;  //������ʾ�������Ѿ������ٽ����򣬿��Խ��н����Ĳ���
		double data_x1,data_x2,data_y1,data_y2;
	//double a1_Ref[Ref_Num],b1_Ref[Ref_Num],L1_Ref[Ref_Num],L2_Ref[Ref_Num],cosa1_Ref[Ref_Num],
	//				 cosb1_Ref[Ref_Num],sina1_Ref[Ref_Num],sinb1_Ref[Ref_Num];
		//��������ǰ��ʱÿ�ν����ı�׼λ��(�����ں���ʱ����ֱ�ӵ��ø�����)

//****************************���ݺϷ��������ӵı���_YZ_2015/09/11************************************************************************************************************	
		u32_t n;//���ڼ�¼�ɼ������ڴ������ݲ�ֵ�ı���
//****************************���ݺϷ��������ӵı���_YZ_2015/11/25************************************************************************************************************			
		unsigned short Tubian_t[Ref_Num],Tubian_c[Ref_Num];//���ڴ���һ��ɨ���õ��Ĺ��ڲο���Ķ�����Ϣt��c��������֮���LCD��Ļ���
    double TC_distance; //���ڼ����ҵ��궨���t��c��Ӧ��֮��ľ�����ο�����ֱ֪���Ƚϣ��Ӷ��˹����ŵ�
//**************��Ļ�������****************		
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
		
		
		int  DATA_NUM ;//��ȥ��ͷ��������
		struct Changedata
		{
			double angle;
			double polar;
		} senor[5];//��һ���ṹ������ȥ�洢�������仯�ĽǶȺ����Ӧ�ļ����곤��*/
	__IO uint32_t LocalTime = 0;  /* this variable is used to create a time reference incremented by 10ms */
	uint32_t timingdelay;
	/* Private function prototypes -----------------------------------------------*/
	void LCD_LED_BUTTON_Init(void);
	void DebugComPort_Init(void);
	void USART1_init(void);
	void Find_REF(void); //�һ�׼
	void data_rely(void) ;
	//void Charge_REF();//��׼�����ж�
	//void Data_Calibration();//��̬�궨
	void DATA_Dynamic(void);//��̬�߳�����
	extern void MY_NVIC_Init(u8,u8,u8,u8);  //�жϳ�ʼ��
	void DATA_int(void);
	/* Private functions ---------------------------------------------------------*/
	uint8_t Interrupt_flag=0;
		
/*	double fabs(double x)//�����ֵ
	{
		double y;
		 if(x>=0)  y=x;
		 else y=(-x);
		 return y;
	}*/
	int Tr(char source_data)//����ת��
	{   
		int dest_data;
		//for(int i=0;i<4;i++)
	 if(source_data>=(0x41))  dest_data=(source_data-0x37);
	 else dest_data=(source_data-0x30);
	 return dest_data;
	}
	void TIM3_Int_Init(uint16_t arr,uint16_t psc)
	{
		RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
		TIM3->ARR=arr;  	//�趨�������Զ���װֵ 
		TIM3->PSC=psc;  	//Ԥ��Ƶ��	  
		TIM3->DIER|=1<<0;   //��������ж�	  
		TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
		MY_NVIC_Init(0,0,TIM3_IRQn,2);	//��ռ0�������ȼ�0����0									 
	}
	void TIM2_Init(void) //��������Զ�װ�صļ���ֵ��ÿ��70ms���������������ֵ
	{
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

			/* TIM5 clock enable */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			/* Time base configuration */
		 
			TIM_TimeBaseStructure.TIM_Period = (700 - 1);
			// �������Ԥ��Ƶϵ����������Ϊ0ʱ��ʾ����Ƶ����Ҫ��1
			TIM_TimeBaseStructure.TIM_Prescaler = (8400 - 1);
			// ʹ�õĲ���Ƶ��֮��ķ�Ƶ����
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			//���ϼ���
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			//��ʼ����ʱ��2
			TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

			/* Clear TIM2 update pending flag[���TIM5����жϱ�־] */
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

			/* TIM IT enable */ //������ж�
			TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

			/* TIM5 enable counter */
			TIM_Cmd(TIM2, ENABLE);  //������ʹ�ܣ���ʼ����

			/* �жϲ������� */
		 MY_NVIC_Init(1,0,TIM2_IRQn,2);	//��ռ1�������ȼ�0����2				
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

	/*void Generate_normal_data( )//������������
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
	{ //������i j k��ƽ�ߵͣ��ߵ�ƽ��Ӧ����ͨ������Ӧ�ܽ��������ź�
		u8_t i = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_15); //ǰ��
		u8_t j = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5); //����
		u8_t l = GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_4); //�궨
		u8_t k = GPIO_ReadInputDataBit(GPIOI,GPIO_Pin_5); //ͣ��
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
		if(i==0 && j==1 && k==1 && l==1)   //ǰ�� 
		{
//			STM_EVAL_LEDOff(LED4);
//			STM_EVAL_LEDOn(LED1);
			m_bVehicleForward=TRUE;
		  m_bStopFlag=FALSE;
			m_bVehicleBackward=FALSE;
			BD_ENABLE=0x00;
		}
		if(j==0 && i==1 && k==1 && l==1)   //ͣ��//���ڸ�Ϊ����_YZ_2015/12/14
		{
//			STM_EVAL_LEDOff(LED4);
//			STM_EVAL_LEDOff(LED1);
			m_bVehicleForward=FALSE;
			m_bStopFlag=FALSE;
			m_bVehicleBackward=TRUE;
			BD_ENABLE=0x00;
		}

		if(l==0 && i==1 && j==1 && k==1)   // �궨_ע�⣺���±궨��ͬʱ��ͣ��
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
		
		if((BD_ENABLE==FALSE && m_bVehicleForward==FALSE && m_bVehicleBackward==FALSE) || (k==0 && i==1 && j==1 && l==1))  //���粻ǰ������������Ϊ��ֹͣ ���ߵ�kΪ�͵�ƽʱ��Ϊͣ��_YZ_2016/03/22
		{
			BD_ENABLE=0x00;  //����ͣ�������Զ������궨
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
		
//		if((Joystick_Right==1 && Joystick_Left==0 && Joystick_Up==0 && Joystick_Down==0) || (Joystick_Right==0 && Joystick_Left==0 && Joystick_Up==0 && Joystick_Down==0) )   // LCD��ʾ���ݽ���1
//		{
//    LCD_Interface1=TRUE;
//		LCD_Interface2=FALSE;
//		}
		
//		if((Joystick_Right==0 && Joystick_Left==1 && Joystick_Up==0 && Joystick_Down==0))   // LCD��ʾ���ݽ���1
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
		 Find_REF(); //�һ�׼(��ȡ��׼����Ϣ�����жϽ���)

//*********************ֻ����������ݵĽ���洢���ִ������Ĳ���_2015/08/25*********//ȡ��_YZ_2015/11/25		
 
	   data_rely() ; 

	}
	void Find_REF()
	{
		int b;//�ҵ��ո�ı�־λ
		int x,y,n;//�洢ȥ���ո����ʱ�������
		int i=0;
		int c,t;
	//	int c,t,j,e,g;
		//*********************************** �����ӵı���__YZ_2015/08/10 **************************************************************************************
		u16_t h; //����������ϺͽǶ�ƽ��ֵ�Ĳ���
		u16_t w;             // ������¼һ�������ﴫ�������������(������ڽǷֱ���Ϊ0.166667������ݰ���wȡֵΪ[1,649])
//		u16_t w_change;      // ������¼����ͻ��ʱ�ĵ�һ���߶ε����   
//		u16_t q = 0;         // ������¼ɨ�赽һ���ο�������ڲο����ϵ��߶�����
//		u16_t w1 = 0;       // ������ʾɨ��Ŀ�ʼ����
//		u16_t w2 = 281;     // ������ʾɨ��Ľ�������
//		int data[281];      // ��ʼ��ɨ�����ݣ����ͣ�
		double Ave_Num;        //����������ϺͽǶ�ƽ��ֵ�Ĳ���
//		double hh;
//		int data_temp[281][4]={0}; //��ʱ�������вο�����Ϣ�ı���
		double Angle_HD,DATA_ADD;    //�нǻ���
		//*********************************** �����ӵı���__YZ_2015/12/10 **************************************************************************************
 //		u8_t BD_Button;     //��¼�����źŵı���
	u16_t Scan_Filter_Num; //��Ե�һ��ɨ��Ѱ����ֵ�쳣���ѭ����������
		//********************************************************************************************************************************************
	//s16_t DA;//Detection angle�������ʱ�����Ľ���ĽǶ�
	//	s16_t DA1;//ͻ�䷢��������
  //	s16_t DA2;//ͻ�����������
	//s16_t DA3;//��׼������
		u8_t  Tubian=0; //�ҵ�ͻ���־λ
	//	double J_Angle;   //��⵽��׼��2����н�
	//	double JZ_Dia;   //��⵽��׼��2���ⳤ�Ȳ�
    u8_t jc;     //��¼��׼����
		//static u16_t Pos_num[209]={0};//��������
	  //static int a;	
		/*Initialize LCD and Leds */ 
		//TIM3_Int_Init(3000-1,8399);//��ʱ�ж�10ms��5000*8400)/84000=500ms
		/* check if any packet rece0ived */
			 
		
			if(Interrupt_flag==1)//��ʱ�жϴ򿪿ͷ��ˣ���ʼ��������
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

			 if(Data_correct_flag==1)//�����β�ɹ��󣬿�ʼӦ�ô�������
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

			for(y=0;y<(DATA_NUM-13);y++)//�õ����մ������ݣ��������ݿո�Ȼ����ת��Ϊ��������
			{		
			if (Data_RECE[y]==' ')  
					 {   
									b=y-x-1;
									x=y+1;
								if(b==0) 
								 {

										 str_temp[n]=Tr(Data_RECE[y-1])&(0xFFFF);  //Size_str_temp-1-n Ŀ���ǶԴ������������ݽ�����洢���Ӷ����㴫����ɨ�跽����һ��ʼ�ٶ������෴�����_YZ_2015/12/14
									 
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
	//*************************************ֱ������Charge_Ref()���вο���������ȡ�Լ���������****************************************
	//*******************************************************************************************************************************
	//*******************************************************************************************************************************
	//*******************************************************************************************************************************
    
    //BD_Flag=Debounce(BD_Button);//ͨ���ɼ��궨��ť���źţ�������ȥ���������ж��Ƿ��ڱ궨״̬
		
		
	//****************************************�޸ĺ��ͻ������ _2015/08/12****************************************************	
				//if((fabs(str_temp[j]-str_temp[j+1])>JUDGE_DATA_THRESHOLD)&&(fabs(str_Ver[j+1]-str_Ver[j+2])<LESS_DATA_DES)&&(NORMAL_DATA_MIN<str_Ver[j+1]<NORMAL_DATA_MAX)) //��׼��Χ��������
				//LС��20�ף���ֱ����С��10m;ͻ�䴦�������200mm��ͻ���ǰ�����߾���С��5mm
	
	//*********************ֻ����������ݵĽ���洢���ִ������Ĳ���_2015/08/25*************************************		

	t = 0;
	c = 0;
	jc = 0; 
	jc_origin=0;
//if((LCD_Interface2==TRUE)&&(LCD_Interface1==FALSE)&&((Time_count%1)==0)&&(LCD_SENSOR_Image==TRUE)){		
//	LCD_Clear(Black);                    /* Clear graphical LCD display        */
//}

	if(1){
		
//��ԭʼ���ݴ�����inverse���������֮��ˢ��ɨ�跽����������ŵĹ�ϵ	__YZ_20160324
	for (Scan_Filter_Num=0; Scan_Filter_Num<=Size_str_temp; Scan_Filter_Num++)
	{		
		str_temp_inverse[Scan_Filter_Num]=str_temp[Scan_Filter_Num];  
	}	
	
	//��������0.5�Ƿֱ������ݳ���Ϊ��ʵֵ1/2�����ԡ����ȡ�Ӧ�öԲɼ����ݽ���*2����	__YZ_20160324
	for (Scan_Filter_Num=0; Scan_Filter_Num<=Size_str_temp; Scan_Filter_Num++)
	{
		if(Scan_Angle_Resolution == 0.5){
		str_temp[Size_str_temp-Scan_Filter_Num]=2*str_temp_inverse[Scan_Filter_Num];
		}
		else{
		str_temp[Size_str_temp-Scan_Filter_Num]=str_temp_inverse[Scan_Filter_Num];
		}
	}	
		
		
	for (Scan_Filter_Num=0; Scan_Filter_Num<=Size_str_temp; Scan_Filter_Num++)  //��һ��ɨ�豾�����ݣ��ҳ���������쳣��(������ֵ��Χ),������ֵ��Ӧ���ݽ���ˢ�¸���_YZ_2015/12/10
	{
		//printf("%u \n",str_temp[Scan_Filter_Num]);
		Angle_HD = (Scan_In_Angle + (Scan_Filter_Num)*Scan_Angle_Resolution) / 180.0 * 3.1415926;
		str_Ver[Scan_Filter_Num] = str_temp[Scan_Filter_Num] * sin(Angle_HD);//����ÿ�����ݵ����ݵ��������봢�浽str_Ver����
			
			if(str_Ver[Scan_Filter_Num]>=STR_DATA_MAX_Vertical || str_Ver[Scan_Filter_Num] == 0)
			{
				str_Ver[Scan_Filter_Num]=STR_DATA_MAX_Vertical; 
				str_temp[Scan_Filter_Num]=STR_DATA_MAX_Vertical/sin(Angle_HD);
			}
			
			//	printf("str_temp[%u]=%u\n",Scan_Filter_Num,str_temp[Scan_Filter_Num]);
		
	} //����ˢ�º����ݾ�����������Χ����˺�STR_DATA_MAX�ɲ���ʹ��_YZ_2015/12/10
	
	

		
	
	for (w = 2; w <= Size_str_temp; w++)
	{ 
//		if((LCD_Interface2==TRUE)&&(LCD_Interface1==FALSE)&&((Time_count%1)==0)&&(LCD_SENSOR_Image==TRUE)){
//    LCD_DrawLine(0,w,str_Ver[w] /10,Vertical);
//		}
		
//		if ((str_temp[w] == 0x00)||(str_temp[w]>=4000)) //���ﲻ��ʹ�����ָ��������쳣��ķ�ʽ����������������˷�__YZ_2015/12/10
//		{
//			str_temp[w] = str_temp[w - 1]; //���Խ׶ζ�0ֵ�������޸�__YZ__2015/11/23
//		}
//		USART_Send(str_temp[w]);
		
		if(Tubian==1 && t==0 && c==0) //���ｫTubian�����Ŀ���Ƿ�ֹ���ҵ�t֮��δ�ҵ�cȻ��Ӱ�쵽֮�������ο����ɨ������_YZ_2015/12/10
		{
			Tubian=0;
		}
		if(Tubian==1 && c==0 && w>=(t+ct_MAX)) //������ҵ�t�󾭹�ct_MAX���߻�δ�ҵ�c���Զ����¿�ʼѰ��t
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
	  	(Tubian == 0                                                  //--->�����ж���1-1
	  	&& (str_temp[w - 1] - str_temp[w])>JUDGE_DATA_THRESHOLD)      //--->�����ж���1-2
	  	&& (fabs(str_temp[w] - str_temp[w + 1]) < LESS_DATA_DES)      //--->����һ���ж���2   //���ﻹ��Ҫ���Ͼ���ֵ__YZ_20160322
	  	&& (fabs(str_temp[w + 1] - str_temp[w + 2]) < LESS_DATA_DES) //--->����һ���ж���3   //���ﻹ��Ҫ���Ͼ���ֵ__YZ_20160322
			&& str_Ver[w] > NORMAL_DATA_MIN                              //--->����һ���ж���4 
			&& str_Ver[w] < NORMAL_DATA_MAX)                            //--->����һ���ж���5  
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
				str_Ver[i] = str_temp[i] * sin(Angle_HD);//�����Ӧ���ݵ��������봢�浽str_Ver����
//				str_Lev[i] = str_temp[i] * cos(Angle_HD);//�����Ӧ���ݵ�ĺ�����봢�浽str_Lev����_����û���õ��ʲ����м���_YZ_2015/12/10
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
			 (Tubian == 1                                                  //--->�����ж���1-1
			 && (str_temp[w + 1] - str_temp[w])>JUDGE_DATA_THRESHOLD)      //--->�����ж���1-2
			 && (fabs(str_temp[w] - str_temp[w - 1])< LESS_DATA_DES)       //--->����һ���ж���2   //���ﻹ��Ҫ���Ͼ���ֵ__YZ_20160322
			 && (fabs(str_temp[w - 1] - str_temp[w - 2]) < LESS_DATA_DES) //--->����һ���ж���3   //���ﻹ��Ҫ���Ͼ���ֵ__YZ_20160322
			 && str_Ver[w] > NORMAL_DATA_MIN                              //--->����һ���ж���4 
			 && str_Ver[w] < NORMAL_DATA_MAX)                             //--->����һ���ж���5  
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
        jc_origin++; //����ֻҪ�ó�һ��c,t��ԭʼͻ����������jc_origin��Ҫ��һ_YZ_2015/12/10				
				Tubian_t[jc_origin]=t;
				Tubian_c[jc_origin]=c;
				
				if(jc_origin>1 && (Tubian_t[jc_origin]-Tubian_c[jc_origin-1])<5 && Tubian_t[jc_origin] != 0 && Tubian_c[jc_origin-1] != 0)
				{
					t=Tubian_t[jc_origin-1]; //��������Բο��㴦�������������ڲο��㴦�ķ���Ὣһ���ο���һ��Ϊ2�����õ���ǰ��c�����t��ʮ�ֽӽ���������Խ���������õ�������c,t�϶�Ϊһ�������һ�������ο����c,t.
				}
				//cout << "t= " << t << endl;
				//cout << "c= " << c << endl;
		/*		cout << "str_temp[13]=  " << str_temp[10] << endl;*/
				
			}
					
				//  USART_Send(0xAA);
			//***************************************2015/08/12************************************
		//	if (t != 0 && c != 0 && 10 <= c - t && c - t <= 40 )
		//***************************************�����˲ο���뾶��Բο�����Ϣ�Ĺ���_YZ_2015/11/25************************************
			if(t != 0 && c != 0)  //����ֻ���ڵõ�һ��c,t��Ż���м��㣬��С����ļ�����_YZ_2015/12/10
			{
				TC_distance=pow(pow(str_temp[t],2)+pow(str_temp[c],2)-2*str_temp[t]*str_temp[c]*cos((c-t)*Scan_Angle_Resolution/180*3.1415926),0.5);
				
//					printf("t=%u\n",t);
//					printf("c=%u\n",c);
//				  printf("TC_distance=%f",TC_distance);
				
			}
			if (t != 0 && c != 0 && ct_MIN <= (c - t) && (c - t) <= ct_MAX &&((R_length)<TC_distance) && (TC_distance<(4*R_length))) 
			 //�����ǵ��ſ�TC_distance�Ĺ�������(��Ϊ������֤������ͬ�뾶�ο����ڲ�ͬ�Ƕ�ɨ���õ��ļ���ֱ�������ϴ�)_YZ_2015/12/10
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
//					c = 0;   //�����c��t����Ӧ�����ж��ⲿ_YZ_2015/11/25
//					t = 0;			
			}			
					c = 0;
					t = 0;
		}
	}
}
//	        USART_Send(jc);  //�������ɨ�赽�Ĳο�����Ŀ
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

	//�����Ѿ���ɨ���������ݷ�����ϣ��Ѿ��洢�����й��˺�Ĳο�����Ϣ��������н������ж�
	//***************************************3���ο���������������2015/08/18************************************
	
					if((jc==3 || jc==4) && Scan_Done==0 && JL_x2_Ready==0x01 && m_bVehicleForward==TRUE) //�������
					{
						j1=1; //��������������㣬���������������ο��㣬��ô����Ҫ��ѡ�Ĳο�����ţ���ʱ�������������ο������Ϊ0 1 2��Ϊ1 2
						j2=2;
						jc_Record=jc;
						Scan_Done=0x01;
						Scan_Num=Scan_Num+1;
						JL_x2_Ready=0x00; //��ʱ������������Scan_Num+1��֮��JL_x2_Ready = 0x00����ζ�Ž���׼���׶ν�������ͱ�֤��ѭ������ֻ�ڽ�����ĵ�һ��������ִ�С�
					  Cal_num=0x01; //˵����ε�λ����ϢӦ����Ϊ��׼λ����Ϣ����֮���������㵱��ȥ��
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
						Scan_Done =0x01; //��ʱ˵�����ع��̽������
						Scan_Num=Scan_Num-1;
						JL_x2_Ready = 0x00;
					}			
				//	if(j==2 && Scan_Done ==0x01 && JL_x2_Ready == 0x00)
					if((jc==2 || (jc>=3 && jc_Record ==0) || (jc_Record==4 && jc==3) ) && JL_x2_Ready == 0x00) //���������������(�ڶ����͵�����)���ڶ�����Գ�ʼʱ�̾�ɨ�赽��3����3�����ϲο���������������������ĸ��ο����Ϊ3���ο�������_YZ_2015/12/10
					{
						j1=0; //�����Scan_Done == 1������£�ɨ�赽�Ĳο�����ĿΪ2��˵����ʱ�����׶��Ѿ���ɡ����Scan_Done == 0��δ��������
						j2=1; //��ˣ���ʱ�����ڵĲο����������������Ҫʹ�õĲο��㡣
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
	//************************��Դ�����δ���һ��������������ʱ�������Ϊ�����˶������ֵĽ������(����)_2015/08/21**************************************************************************
			if(Scan_Done==0x01 &&( jc==3 || jc==4)  && JL_x2_Ready ==0x00 && m_bStateBefore ==0x01 && m_bVehicleForward==FALSE && m_bVehicleBackward==TRUE )
			{
				Scan_Done = 0x00;  //���統ǰʱ���˶�������֮ǰһ��ʱ�̷����෴�����Ҵ�ʱScan_Done==0x01(˵�����������׶�δ����)����ô�Զ���Ϊ��ʱ�������Ѿ�������ǰ��������׶�
			}
			
  //************************��Դ�����δ���һ��������������ʱ�з����Ϊ�����˶������ֵĽ������(����)_2015/08/21**************************************************************************
			if(Scan_Done==0x01 &&( jc==3 || jc==4) && JL_x2_Ready ==0x00 && m_bStateBefore ==0x00 && m_bVehicleForward==TRUE && m_bVehicleBackward==FALSE )
			{
				Scan_Done = 0x00;  //���統ǰʱ���˶�������֮ǰһ��ʱ�̷����෴�����Ҵ�ʱScan_Done==0x01(˵�����������׶�δ����)����ô�Զ���Ϊ��ʱ�������Ѿ�������ǰ��������׶�
			}

				//	USART_Send(Scan_Done);
				//	USART_Send(0xBB);
				
	}
}
					
//�����Ѿ���ɨ���������ݷ�����ϣ��Ѿ��洢�����й��˺�Ĳο�����Ϣ��������н������ж�
	//***************************************����ο���������������2015/08/20************************************
				/*	if(jc==3 && Scan_Done==0 && JL_x2_Ready==0x01 && m_bVehicleForward==TRUE) //�������
					{
						j1=1; //��������������㣬���������������ο��㣬��ô����Ҫ��ѡ�Ĳο�����ţ���ʱ�������������ο������Ϊ0 1 2��Ϊ1 2
						j2=2;
						Scan_Done=0x01;
						Scan_Num=Scan_Num+1;
						JL_x2_Ready=0x00; //��ʱ������������Scan_Num+1��֮��JL_x2_Ready = 0x00����ζ�Ž���׼���׶ν�������ͱ�֤��ѭ������ֻ�ڽ�����ĵ�һ��������ִ�С�
					  Cal_num=0x01; //˵����ε�λ����ϢӦ����Ϊ��׼λ����Ϣ����֮���������㵱��ȥ��
					}
					if(jc==3 && m_bVehicleBackward==TRUE && JL_x2_Ready == 0x01 )
					{
            j1=0;
            j2=1;
						Scan_Done =0x01; //��ʱ˵�����ع��̽������
						Scan_Num=Scan_Num-1;
						JL_x2_Ready = 0x00;
					}			
				//	if(j==2 && Scan_Done ==0x01 && JL_x2_Ready == 0x00)
					if(j==2 && JL_x2_Ready == 0x00)
					{
						j1=0; //�����Scan_Done == 1������£�ɨ�赽�Ĳο�����ĿΪ2��˵����ʱ�����׶��Ѿ���ɡ���Scan_Done == 0��δ��������
						j2=1; //��ˣ���ʱ�����ڵĲο����������������Ҫʹ�õĲο��㡣
						Scan_Done = 0x00; 
					}				
}
			 }
*/
			 
		//**************************************************************************************************
			 
	void data_rely(void)     //���ݼ���
	{  double a2,ANGLE_cosr,ANGLE_sinr,ANGLE_cosb,ANGLE_sinb,sina1,sinb1,cosa1,cosb1;//�н�bx��r����������ֵ
		 double L12,L11,L22,SENSON_X,SENSON_Y,temp1,temp2,temp3,temp4,temp5,temp6,temp7;
     double SENSOR_X_OUTPUT,SENSOR_Y_OUTPUT;
		 u8_t Translation_Num=0; //����ƽ�����ʱ�ļ�������
		if(m_bStopFlag==0x01 && BD_ENABLE == 0x01) //����ֹ���ұ궨״̬Ϊ0x01 �궨
		  // if( BD_ENABLE == 0x01)
				 {  //STM_EVAL_LEDOn(LED4);
           //USART_Send(BD_ENABLE);					 
						a1=senor[j1].angle;   
						b1=3.1415926-senor[j2].angle;   
						L1=senor[j1].polar;
						L2=senor[j2].polar;
						
						a1_Ref[Scan_Num]=a1; //�洢ÿ�ν�����ı�׼λ�ã����ؽ���ʱ��
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
					 
					 //���궨������ݴ��浽SENSOR_DISTANCE_FRONT��SENSOR_DISTANCE_BACK����
					 SENSOR_DISTANCE_FRONT=SCANNER_CENTER;
					 SENSOR_DISTANCE_BACK=SCANNER_CENTER;
		 
					 
					 // i++;
//					  if(L1!=0)
//					  BD_ENABLE=0x00; //�ڼ����˽���֮��ȡ����ԭ�ȵ�m_bStopFlag=0x00����
					
				 }
				 
				 if	(m_bStopFlag==0x00 && BD_ENABLE == 0x00) 				 
				{
             
					if(m_bVehicleForward==TRUE && m_bVehicleBackward==FALSE) //�����������˶������н�����������Ϣ��Ҫ����
					{ 
						if(Scan_Num!=0&&Cal_num==1) //Cal_num==1˵��֮ǰ��׼λ����Ϣ�Ѿ����棬�������Ӧ�����ȼ����׼λ������
						{
							//sina1=sin(senor[j1].angle);
							//cosa1=cos(senor[j1].angle);
							//sinb1=sin(3.1415926-senor[j2].angle);
							//cosb1=cos(3.1415926-senor[j2].angle);
							
							sina1=sin(senor[j1].angle)*ANGLE_coscx-ANGLE_sincx*cos(senor[j1].angle);//�����Ѿ����й���������˱�׼λ�ýǶ���Ҫ����(cxΪǰһʱ�̼����λ�õ�ƫת��)
							cosa1=cos(senor[j1].angle)*ANGLE_coscx+ANGLE_sincx*sin(senor[j1].angle); //���ڷ����Ǻ������ۼ������������Ҫ���������ֱ�Ӽ���������Ǻ���ֵ
							sinb1=sin(3.1415926-senor[j2].angle)*ANGLE_coscx-ANGLE_sincx*cos(3.1415926-senor[j2].angle);
							cosb1=cos(3.1415926-senor[j2].angle)*ANGLE_coscx+ANGLE_sincx*sin(3.1415926-senor[j2].angle);
							cosa1_Ref[Scan_Num]=cosa1; //����׼λ����Ϣ���棬֮�����˶�����ʱֱ����ȡ��
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
							Cal_num =0x00; //�����Ŀ����ֻ�ѽ�����ĵ�һ����λ����Ϊ������ı�׼λ�ã�֮���ÿһ��λ�þ���Ϊ��һʱ��λ��
						}
				
					}
					else if(m_bVehicleForward==FALSE && m_bVehicleBackward==TRUE) //���ڷ����˶�������ֱ����ÿһʱ�̶�����֮ǰ����ı�׼λ����Ϣ(Scan_Num���������õı�׼λ����Ϣ���)
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
			ANGLE_cosb = temp2 / temp3;//bΪ���������ο���һ�����ߺͲο���һ���ο���������ߵļн�
			ANGLE_sinb = sqrt(1 - pow(ANGLE_cosb, 2));
			ANGLE_cosr = data_x2 / L12;//rΪ������һ��������x��ļн�
			ANGLE_sinr = data_y2 / L12;
			//���ʱ������������ԭ�����ߺ�x��н�Ϊcelta,����celta=b+r
			temp4 = (ANGLE_cosb*ANGLE_cosr) - (ANGLE_sinb*ANGLE_sinr);//cos(celta);
			temp5 = (ANGLE_sinb*ANGLE_cosr) + (ANGLE_cosb*ANGLE_sinr);//sin(celta)
			SENSON_X = L11*temp4;
			SENSON_Y = L11*temp5;                     //1750mm   1734mm   1750mm
					

			//��ƫת��Ϊcx ��Ӧ��cx=a2-celta
			
			
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
			
		//�������жϲ�ഫ�����ɼ����ݵĺϷ����
		if(SENSOR_DISTANCE_TEMP[0] > SENSOR_DISTANCE_FRONT)
			n=SENSOR_DISTANCE_TEMP[0]-SENSOR_DISTANCE_FRONT;
		else
			n=SENSOR_DISTANCE_FRONT-SENSOR_DISTANCE_TEMP[0];
		if(n<=SENSOR_DISTANCE_ERR)//�ɼ����ݺϷ�������SENSOR_DISTANCE_FRONT/BACK����
		{//������ʱ�������ֵ�Ϸ���Χ�����ޣ������Դ���֤_YZ_2015/09/11
			SENSOR_DISTANCE_FRONT=SENSOR_DISTANCE_TEMP[0];  
		  SENSOR_DISTANCE_BACK=SENSOR_DISTANCE_TEMP[1];
		}
		else if(n>SENSOR_DISTANCE_ERR)
		{
			SENSOR_DISTANCE_ERR_CNT++;
			if(SENSOR_DISTANCE_ERR_CNT>9)//����10�����ݶ�������ֵ������Ϊ���ݺϷ�
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
			
//*********************************** ����Rectify����__YZ__2015/08/18 **************************************************************************************
		m_bDistanceSampleOk =TRUE; //˵�����ݲɼ���������������Խ��н������������(��ƫ)�Ĳ���
				
				
			//**************************���浱ǰʱ���˶�����_2015/08/21************************************************************************	
			if(m_bVehicleForward==TRUE && m_bStopFlag == FALSE &&BD_ENABLE == FALSE)
			{
				m_bStateBefore=0x01;
			}
			if(m_bVehicleForward==FALSE && m_bStopFlag == FALSE &&BD_ENABLE == FALSE)
			{
				m_bStateBefore=0x00;
			}
			
			if(fabs(SENSON_X-data_x2)<50 && Scan_Done == 0x00 &&m_bVehicleForward==TRUE && m_bVehicleBackward==FALSE) //�������ready����:�����Scan_Done == 0x00 �Ǳ�ֻ֤���ڵ�һ��������������(SENSON_X-data_x2)>50 �󣬲Ż���JL_x2_Ready=0x01����ֹ����ÿ�������������λ�ö�����������ĵ�һ��λ��
			{ 
				if(data_x2!=0)
				JL_x2_Ready=0x01;

			}
		//***************************��������Է������������������ԣ�_2015/08/21***********************************************************************	
			if(fabs(SENSON_X)<50 && Scan_Done == 0x00 &&m_bVehicleForward==FALSE && m_bVehicleBackward==TRUE) //�������ready����:����Ľ���ready��������Դ����������˶�������ͻȻ��ʼ�����˶���û�о���jc��3-->2-->3�Ĺ���
			{ 
				if(data_x2!=0)
				JL_x2_Ready=0x01;

			}
	
        
} 

		void DATA_Dynamic()     //��̬�߳�
	{
		Find_REF();
		//Charge_REF ();      //��������
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
	//��ʱ��3�жϷ������	 
	void TIM3_IRQHandler(void)
	{ 		  
		 
		if(TIM3->SR&0X0001)//����ж�
		{
			Interrupt_flag=1;
		}
		TIM3->SR&=~(1<<0);//����жϱ�־λ 	
		//Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
	}
	void TIM2_IRQHandler(void)//������2�жϷ������
	{
			if(TIM2->SR&0X0001)//����ж�
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
