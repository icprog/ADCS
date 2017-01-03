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
//����ADC����Ҫ�����ͷ�ļ�_YZ_2015/11/18
#include "stm32f4xx_adc.h"
#include "ADC_YZ_head.h"
#include "GLCD.h"
//��ͼ��_YZ_2015/11/25
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
extern void DATA_int(void);   //�ϵ��ʼ���ݸ�ֵ
extern void LCD_LED_BUTTON_Init(void);
extern void Analyse_RectifyData(void);	//�����������豸���������ھ�ƫ���Ƹ˵�����	
extern void	Rectify(void);
extern void Adjust_Deflect(void);
extern void Wheel_Pos_Control(void);
extern void GPIO_In_Config(void);
extern void GPIO_Out_Config(void);
extern void USART_Send(uint16_t Data);
extern void Vehicle_State_Detect(void);

extern void Draw_Wheel(void); //���������Ƿ��������

/*__________________data_collection.h______________________*/
u8_t m_bDistanceSampleOk ; //��ഫ�����ɼ����ݺϷ������Խ��о�ƫ�ж�
u8_t m_bWheelDataOk;	//�����̴������ɼ����ݺϷ������Խ��о�ƫִ��
u8_t m_bTransferWorkMode;		//����������ģʽʹ��--����main.c��
u8_t m_bRightRudder;			    //�Ҷ�ģʽ--����main.c��
double SENSOR_DISTANCE_FRONT;//��ͷ���׼�߾���
double SENSOR_DISTANCE_BACK;//��β���׼�߾���
//2015/09/11
double SENSOR_DISTANCE_TEMP[2]; //����ԭʼ��������(1Ϊ��ͷ�����׼�߾��� 2Ϊ��λ�����׼�߾���)�ڽ������ݺͷ���ȷ�Ϻ��ٽ��������ݴ���SENSOR_DISTANCE_FRONT/SENSOR_DISTANCE_BACK��
u8_t SENSOR_DISTANCE_ERR_CNT;//���ڼ�¼ԭʼ����ƫ��(ƫ���׼����ΪSENSOR_DISTANCE_ERR)֮ǰһ�δ洢�ĺϷ����ݵĴ���
 //2015/08/25
u8_t m_bBackMidWheel;  //ִ��ͣ��ʱ�������̻ص��е�Ķ�����FALSE ��ִ�У�TRUE ִ��
u8_t m_bStopFlag=0x01;	
//static void EXTILine15_10_Config(void);
u8_t m_bVehicleForward = 0;		    //�г�ǰ�������ǣ�TRUE����ǰ����FALSEû��ǰ��
u8_t m_bVehicleBackward = 0;			//�г����˷����ǣ�TRUE���ں��ˣ�FALSEû�к���
u8_t BD_ENABLE = 0 ; //�궨�������λ
u8_t m_bStateBefore = 0;          //ǰһʱ���˶�����״̬��־λ �����洢ǰһʱ�̵��˶�������(0x01)���Ƿ���(0x00)����Ϣ
u16_t Time_count=0;   //Ϊ����֤�����Ƿ���ִ��Data_collection.c�п������趨�ļ�������(��1++��10000������ظ�++)
u16_t A_count=0;  //��¼�ɼ���14A��ͷ�����ݰ��ĸ���
u16_t B_count=0;  //��¼�ɼ���14B��ͷ�����ݰ��ĸ���
u8_t LCD_Interface1=0; //LCD�Ƿ���ʾ���ݽ���1����
u8_t LCD_Interface2=0; //LCD�Ƿ���ʾ���ݽ���2����
u8_t LCD_Vehicle_Trail=0; //LCD�Ƿ���ʾ�����켣ͼ����
u8_t LCD_SENSOR_Image=0;  //LCD�Ƿ���ʾ������ɨ��ͼ�����
u8_t LCD_Vehicle_Posture=0; //LCD�Ƿ���ʾ������̬ͼ�����

//2015/12/10��Զ���ο��������ӵĽ����ü�������
u8_t jc_Record=0; //��¼����ʱ�̸ð����ݵĲο�����Ŀ,���ڽ��һ����ɨ�赽4���ο�������_YZ_2015/12/10

//2015/12/13�������ʹ�ö�ջ�洢�����Ĳ���
u8_t Stack_Sum_Num=0; //��ջ����Ǽ�������
u8_t Stack_Replace_Num=0; //��ջˢ������(��һ����ǰ�滻)��������
u8_t Vehicle_Stack_Num=0; //��ջ�����������
u8_t Vehicle_Stack_Full=0;//��ջ�洢�Ƿ���������
u8_t BD_Stack_Num=0;      //�궨λ�ö�ջ�����������
u8_t BD_Stack_Sum_Num=0;  //�궨λ�ö�ջ��ͼ�������
u8_t BD_Stack_Replace_Num=0; //�궨λ�ö�ջˢ�����ݼ�������
double Vehicle_X_Stack[Stack_Size]={0}; //���泵��λ�ú������ջ
double Vehicle_Y_Stack[Stack_Size]={0}; //���泵��λ���������ջ
double Vehicle_sincx_Stack[Stack_Size]={0}; //���泵��ƫת��sinֵ��ջ
double Vehicle_coscx_Stack[Stack_Size]={0}; //���泵��ƫת��cosֵ��ջ
double Vehicle_Output_Sum_X={0}; //��ջ�������������ֵ
double Vehicle_Output_Sum_Y={0}; //��ջ�������������ֵ
double Vehicle_Output_Sum_sincx=0; //��ջ�������ƫת��sinֵ
double Vehicle_Output_Sum_coscx=0; //��ջ�������ƫת��cosֵ
double Standard_Position_x1_Stack=0; //�궨ʱ���ɶ�ջ����ó���׼λ�ú�����
double Standard_Position_y1_Stack=0; //�궨ʱ���ɶ�ջ����ó���׼λ��������
double Standard_Position_x2_Stack=0; //�궨ʱ���ɶ�ջ����ó��ڶ����ο��������
double Standard_Position_y2_Stack=0; //�궨ʱ���ɶ�ջ����ó��ڶ����ο���������
double Standard_Position_x1_Stack_Sum[Stack_Size]={0}; //�궨ʱ���ɶ�ջ����ó���׼λ�ú�����
double Standard_Position_y1_Stack_Sum[Stack_Size]={0}; //�궨ʱ���ɶ�ջ����ó���׼λ��������
double Standard_Position_x2_Stack_Sum[Stack_Size]={0}; //�궨ʱ���ɶ�ջ����ó��ڶ����ο��������
double Standard_Position_y2_Stack_Sum[Stack_Size]={0}; //�궨ʱ���ɶ�ջ����ó��ڶ����ο���������


double SCANNER_CENTER   ;
double LEFT_MIN_VALUE_BACK   ;	/* ɨ�賵������߼��޾���  */
double RIGHT_MAX_VALUE_BACK  ;	/* ɨ�賵�����ұ߼��޾���  */
double LEFT_MIN_VALUE_FRONT  ;	/* ɨ�賵ǰ����߼��޾���70  */
double RIGHT_MAX_VALUE_FRONT ;	/* ɨ�賵ǰ���ұ߼��޾���70  */
char text3[40];

/******************���±������0.3��Ƿֱ��ʽ������ʵ��0.1667��Ƿֱ������ݲɼ�__2015/08/25***********************************/
u8_t IntersectLocate = 0x01;     // ����Ǻű�����FALSE��Ӧ��ʼ�Ƕ�Ϊ0�����ݣ�TRUE��Ӧ��ʼ�Ƕ�Ϊ0.1667������(ԭʼ���ݽǷֱ���Ϊ0.3��)
u8_t IntersectCount = 0x00;     // ��������渳ֵ���ݵļ������� IntersectCount�ڳ��򴢴�str_temp1��+1 �ڴ���str_temp2����+1 ��IntersectCount == 2ʱ��ʼ���渳ֵ
u8_t IntersectOK = 0x00;         // �ж�һ������(14A��14B)�Ƿ��Ѿ����洢�浽str_temp3���鵱��
u16_t str_temp[Size_str_temp]={0}; //����ԭʼ����
u16_t str_temp_inverse[Size_str_temp]={0}; //����ߵ�������

/*____________________display.h____________________________*/
//��������
u8_t m_cDisplayBuffer[4];
//
u8_t m_cP377;
//------------------------
u8_t m_bNewDisData;			//ˢ����ʾ����
u8_t m_bDisFlag;
//
u8_t m_cDisErrCnt;				//��ʾ������Ϣ�ı�ʾѭ�����
u8_t m_cDisDataDelay;			//��ʾ������ʱ
u8_t m_cSysError;			//�ò�ͬ��λ��ʾϵͳ����

/*_____ sys.h ________________________________________________*/

s8_t IN_FORWARD;				//p1.0��ǰ���ź�(Input)
s8_t IN_BACKWARD;				//p1.1�������ź�(Input)
s8_t IN_MID_SWITCH;			//p1.2���м���ź�(Input)
s8_t OUT_PUSH;					//P3.0��ǰ���Ƹ�(output)����1,ִ��
s8_t OUT_DRAW;					//p3.1�������Ƹ�(output)����1,ִ��
//
s8_t IN_MODE;				  //P1.6-����ģʽ�ź����룬0Ϊ������ģʽ��1Ϊ�ǻ�����ģʽ
//
s8_t IN_SPEED;					//p1.4�������źż��

s8_t IN_RUDDER;			    //P1.5 - ����Ҷ�ģʽ
//
s8_t IN_LOCK_WHEEL;			//p3.5���������������
//
s8_t IN_AD;					//p1.7- AD�����ź�

//��������������ʱ����Ҫ
//s8_t DIN=P3^2;						//��ʾ�ܽ�
//s8_t CLK=P3^3;
//s8_t LOAD=P3^4;

/*______rectify.h_________________________________________________________*/
u8_t m_bNewData;	
u8_t m_bOK;                 //��û�����������߱�־λ
u32_t m_nFrontExtremeCount;		//�ֳ��޴���
u32_t m_nFrontExtremeCount1;	    //��ʼ�߳��ֳ��޴���
u32_t m_nBackExtremeCount;		//��������ʼ��ʱ���޴�������

u32_t m_nSubValue;				//��ֵ(����������)
u32_t m_nDiffValue;				//�������ͻ�׼��֮���ƫ��ֵ
u8_t m_bSubSymbol;				//��ֵ�ķ���

u8_t m_bFrontLeft0;				//һ��ƫ���׼��
u8_t m_bFrontRight0;				//һ��ƫ���׼��
u8_t m_bBackLeft0;				//һ��ƫ���׼��
u8_t m_bBackRight0;				//һ��ƫ���׼��

u8_t m_bCenterDiff;				//�������׼�ߵ�ƫ��ֵ��TRUE�����ң�FALSE������
u8_t m_cVehicleState;			//������̬		
u8_t m_bWheelLock;				//�������Ƿ�������ǣ�0δ������1����
u32_t m_nDist[3];         //m_nDist[0:1]�ڶ�����ƫ���泵ͷ��λ��ֱ����(�����Ѳ���ʹ��)��m_nDist[2]���淽�����Ƹ˾���__YZ__2015/11/23

/*______execute.c_________________________________________________________*/
u32_t m_nWheelPosObj;			//�Ƹ�Ŀ��λ��
u8_t m_bWheelOpposite;		//�Ƿ����ڷ������

/*______timer.h_________________________________________________________*/
u8_t m_cTimerZdcs;					//��ʱ����ʱ�жϴ���
u8_t m_cZdcs,m_cZdcsbj;

/*______adc.h_________________________________________________________*/
ADC_CommonInitTypeDef ADC_CommonInitStructure;  //ADCͨ�����ýṹ������
ADC_InitTypeDef ADC_InitStruct;              //ADC1���ýṹ������

/*______LCD��ʾģʽ���Ʊ���_YZ_2015/11/26_________________________________________________________*/
u8_t Key_Button_Push = 0; //KEY�����Ƿ��±�ǲ���
u8_t LCD_Change_Mode = 0; //LCD��ʾģʽ�л��Ƿ�������
u16_t LCD_Change_Mode_InTime = 0; //LCD��ʾģʽ�л�ʱ��������¼����(Key���³�ʼʱ��)
u16_t LCD_Change_Mode_FiTime = 0; //LCD��ʾģʽ�л�ʱ��������¼����(Key�ɿ���ֹʱ��)
u16_t LCD_Change_Mode_dTime = 0; //LCD��ʾģʽ�л�ʱ��������¼����(Key�������ɿ����ʱ��)

u32_t Time_For_Main = 0;//�����������ʱ����
int adval_test=0; //����������adc
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
  ETH_BSP_Config();//��ʼ����̫��Ӳ������
  /* Initilaize the LwIP stack */
  LwIP_Init();
  /* Infinite loop */
	TIM3_Int_Init(4000-1,8399);//��ʱ�ж�10ms��5000*8400)/84000=500ms
 // ����궨__���ڲ���ʹ�ð����ñ궨��1������ʼ��Ϊ1__YZ_20160322
//	DATA_int();  //��BD_ENABLE=0x01
	
//!!!!!!!!STM32����ͨ�����ñ�׼ ADC3_channel 7--->PF9!!!!!!!!!!!!!!!!!!	
	//GPIO��ʼ��_2015/08/31
	GPIO_In_Config();	
	GPIO_Out_Config();	
	
	Init_Rectify_Data(); //��ʼ����ƫ����_YZ_2015/09/01
	Init_Sample_Data(); //��ʼ�����������ݲɼ�����_YZ_2015/09/11
	
//// ʹ������ADC���ֵ�ADC��ʼ��_YZ_2015/11/18
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
///*------------������԰��ӵ�GPIO���ܣ�����ʹ�þ�ƫ�㷨�����Լ���ƫִ�в��ֳ���-----------*/
//		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_SET); 
		
	   
// }
// if(GPIO_State_val==0x01){
//		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);  
//		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET); 
//	}
		
		
 

//		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);//KA2�͵�ƽ
		

		
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



			Data_collection(); //ͨ����ഫ�������㳵��ƫת����Լ���������_���� Find_REF()��data_rely()����������ִ��_2015/08/25
	    ADC_StartCnv (); //��ʼADCת��
//      adval_test=ADC_GetCnv();
//	   AD_print_test = adval_test;  
//		 sprintf(text_test, "adval = %06u", AD_print_test);
			//sprintf(text_test, "%lu", adval_test);
			//USART_Send(adval_test);
//			printf("%s\n",text_test);
//			GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);

////		LCD_DisplayStringLine(Line1, "before sample_DATA");

		  Sample_Data();     //ͨ���Ƹ˴��������㷽����λ��_2015/09/14

////		LCD_DisplayStringLine(Line7, (uint8_t*)"after sample_DATA");
////*****************���Ͼ�������1_2015/08/24*********************************


////����ʼ������״̬�������ã���֤������Է���ÿ��ʱ����Ҫ��ƫʱ���Ƹ��ƶ�״̬_2015/09/01		
     Vehicle_State_Detect();
	    m_bBackMidWheel = FALSE;			
	  //  m_nWheelPosObj = WHEEL_MID_VALUE;
	    m_bWheelOpposite = FALSE;	
	    m_bNewData = TRUE;			
	    m_bOK = TRUE;
	  //   m_bDistanceSampleOk = TRUE;    // m_bDistanceSampleOk ��λ��data_collection.c�н���
	    m_bWheelDataOk = TRUE;  // m_bWheelDataOk ��λ��sample.c�еĺ���Sample_Data()�н���
		m_bTransferWorkMode=FALSE; //����Ĭ����泵
		m_bRightRudder=FALSE; //����Ĭ����泵
    
		if(m_bStopFlag==TRUE) //ͣ��ʱ�����̻���
		{
			m_nWheelPosObj = WHEEL_MID_VALUE;
			Wheel_Pos_Control();// ����ʱ�Ȳ����Ʒ�����__YZ__20160324
		}
//		
//		
  if(m_bStopFlag==FALSE)
		{
			Analyse_RectifyData();	//�����������豸���������ھ�ƫ���Ƹ˵�����	
			Rectify();				//��ƫ�ж�
			//USART_Send(0xBB);
			//USART_Send(m_cVehicleState);
			//Check_Wheel_Lock();//��鷽�����Ƿ�����
			Adjust_Deflect();		//����ƫ��
      Wheel_Pos_Control();//�ƶ�������  ����ʱ�Ȳ����Ʒ�����__YZ__20160324
		}
//	
//		//sample.c --->m_bWheelDataOk ���ӣ�����������������������
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

int fputc(int ch, FILE *f) //��fputc�������¶�����ʵ��printf�򴮿ڷ�������__20160317__YZ
{

USART_SendData(USART1, (unsigned char) ch);// USART1 ���Ի�������ʹ�õ� USARTx 

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
