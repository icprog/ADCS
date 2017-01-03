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
//��������
u8_t m_bOutPush;					//ǰ���Ƹ˱�ǣ����ڸ�λ
u8_t m_bOutDraw;					//
//u8_t m_bWheelOpposite;			//������ֵı��λ��TRUE ���ڷ�����֣�FALSE û�з������
u8_t m_bMoveFront;				//���Ƹˣ��Ҵ��ֱ��
u8_t m_bMoveBack;				//���Ƹˣ�����ֱ��

u8_t m_bVehicleHighSpeed;		//�г������ٶȱ�ǣ�TRUE�Ǹ��٣�FALSE�ǵ���
//u8_t m_bTransferWorkMode;		//����������ģʽʹ��--����main.c��
//u8_t m_bRightRudder;			    //�Ҷ�ģʽ--����main.c��


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
//ת���Ƹ˵ķ���0���ƣ�1����
//----------------------------------------------------------------------
//ƫ�Ƶ���
u32_t m_nWheelPostion;			//�Ƹ�������λ��
u8_t m_cVehicleAngle;			//�����ƫ��̶�ָʾ
u32_t m_nWheelExtremeCount;		//�Ƹ˳��޼�������
u8_t m_cWheelMidErrCnt;		//�Ƹ˻��д��������
extern u8_t m_cVehicleState;	//����������̬
///////////////////////////////////////////////
extern u8_t m_bOK;			    //��û�����������߱�־λ		
extern u8_t WHEEL_CHANCE;	    //H������������������
extern u8_t m_cSysError;		//�ò�ͬ��λ��ʾϵͳ����

extern u8_t m_bSubSymbol;
extern u8_t m_bWheelLock;
extern u32_t m_nSubValue;
extern u32_t m_nDist[3];    //m_nDist[0:1]�ڶ�����ƫ���泵ͷ��λ��ֱ����(�����Ѳ���ʹ��)��m_nDist[2]���淽�����Ƹ˾���__YZ__2015/11/23

extern double SENSOR_DISTANCE_FRONT;//��ͷ���׼�߾���                         ;
extern double SENSOR_DISTANCE_BACK;//��β���׼�߾���                         ;
extern u8_t m_bWheelDataOk;
extern u8_t m_bBackMidWheel;
extern u32_t m_nDiffValue;
extern u8_t m_bCenterDiff;
extern u8_t m_bNewData;

extern u32_t m_nFrontExtremeCount;		//�ֳ��޴���
extern u32_t m_nFrontExtremeCount1;	    //��ʼ�߳��ֳ��޴���
extern u32_t m_nBackExtremeCount;		//��������ʼ��ʱ���޴�������

extern u8_t m_bVehicleForward;		    //�г�ǰ�������ǣ�TRUE����ǰ����FALSEû��ǰ��
extern u8_t m_bVehicleBackward;			//�г����˷����ǣ�TRUE���ں��ˣ�FALSEû�к���
extern u32_t m_nWheelPosObj;			//�Ƹ�Ŀ��λ��
extern u8_t m_bTransferWorkMode;		//����������ģʽʹ��--����main.c��
extern u8_t m_bRightRudder;			    //�Ҷ�ģʽ--����main.c��
extern u8_t m_bWheelOpposite;
extern u8_t LCD_Vehicle_Trail; //LCD�Ƿ���ʾ�����켣ͼ����
extern u8_t LCD_SENSOR_Image;  //LCD�Ƿ���ʾ������ɨ��ͼ�����
extern u8_t LCD_Vehicle_Posture; //LCD�Ƿ���ʾ������̬ͼ�����
extern void USART_Send(uint16_t Data);
/////////////////////////////////////////////////////////////////////////
/*************************************************************************
******	���ݵ�ǰ������λ�úͳ���״̬���жϳ������ڵ��صľ���׶�	******
******	��ͬ�׶���Ҫ���ղ�ͬ�Ŀ��Ʒ����Գ�������̬���п��ƺ͵���	******
******	������λ�úͳ�����̬���ޱƽ���׼��ʱ�����Ʒ����̻���		******
**************************************************************************/
void Adjust_Deflect()
{
	m_bVehicleHighSpeed=FALSE;  //����ʹ�õ��ٳ�
	if(m_bBackMidWheel==TRUE)//������������ڻ��У���ͣ�����̿���
		return;
	if(m_nSubValue>DIFF_VALUE4)	//����ƫ��Ƕ�
		m_cVehicleAngle=4;
	else if(m_nSubValue>DIFF_VALUE3)
		m_cVehicleAngle=3;
	else if(m_nSubValue>DIFF_VALUE2)
		m_cVehicleAngle=2;
	else if(m_nSubValue>DIFF_VALUE1)
		m_cVehicleAngle=1;
	else
		m_cVehicleAngle=0;
	//����ƫ���׼�ߵ�ֵ
	if(m_bTransferWorkMode==TRUE)			   //����������ģʽ
	{
		switch (m_cVehicleState)
		{
		case STATE_1:
			//���ݳ���ƫ��Ƕȣ��𲽵����ķ�ʽ����������ͷ�������֣�ʹ���ܹ��𲽵����׼�߿�£��������ֳ���ƫ��ܴ�����
			if(m_bWheelOpposite==FALSE)	//û�з������
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
					if(m_cVehicleAngle>=2)//���ڴ˴���ƫ��ֵ���Ա�ʹ����һ����С�ĽǶȿ���state5//pl
						m_bWheelOpposite=TRUE;
				}
			}
			//
			if(m_bWheelOpposite==TRUE) //���ڷ������
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
			//���ݳ���ƫ��Ƕȣ��𲽵����ķ�ʽ����������ͷ�������֣�ʹ���ܹ��𲽵����׼�߿�£��������ֳ���ƫ��ܴ�����
			if(m_bWheelOpposite==FALSE)	//û�з������
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
					if(m_cVehicleAngle>=2)//���ڴ˴���ƫ��ֵ���Ա�ʹ����һ����С�ĽǶȿ���state5//pl
						m_bWheelOpposite=TRUE;
				 }
			}		
			//
			if(m_bWheelOpposite==TRUE) //���ڷ������
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
			if(m_bWheelOpposite==TRUE)//��״̬1��2����5ʱ�ѷ�����ֱ�����
				m_bWheelOpposite=FALSE;			
			if(m_bVehicleForward==TRUE)//ǰ��
			{
				if(m_cVehicleAngle>=3)//�������ƫ��Ƕȹ����Ƚ��е�ֱ����			
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
					if(m_nDiffValue>CENTER_ERR)//================���ڴ˴�CENTER_ERR��ʹ���ֵ��ڷ��ȱ�С
					{
						if(m_bCenterDiff==TRUE)//����-Զ��
						{	
							if(m_bSubSymbol==TRUE)
								m_nWheelPosObj=WHEEL_MID_VALUE;	
							else
								m_nWheelPosObj=WHEEL_BACK1;
						}
						else					//����-����
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
			else if(m_bVehicleBackward==TRUE)//����
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
					if(m_nDiffValue>(CENTER_ERR/2))//================���ڴ˴�CENTER_ERR��ʹ���ֵ��ڷ��ȱ�С
					{	
						if(m_bCenterDiff==TRUE)//����-Զ��
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
						else					//����-����
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
/////////////////////////////////////////////////////////H��//////////////////////////////////////////////////////
//////////////////////////////////////////////////////�Ҷ泵//////////////////////////////////////////////////////
	else
	{
		if(m_bRightRudder == TRUE)						   //�Ҷ泵
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
				//���ݳ���ƫ��Ƕȣ��𲽵����ķ�ʽ����������ͷ�������֣�ʹ���ܹ��𲽵����׼�߿�£��������ֳ���ƫ��ܴ�����
				if(m_bWheelOpposite==FALSE)	//û�з������
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{
						m_nWheelPosObj=WHEEL_BACK3_H;		//7�ո���
						if(m_cVehicleAngle>=2)			
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_BACK3_H;	
						if(m_cVehicleAngle>=2)//        mdf���ڴ˴���ƫ��ֵ���Ա�ʹ����һ����С�ĽǶȿ���state5//pl
							m_bWheelOpposite=TRUE;
					}
				}
				//
				if(m_bWheelOpposite==TRUE) //���ڷ������
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
				//���ݳ���ƫ��Ƕȣ��𲽵����ķ�ʽ����������ͷ�������֣�ʹ���ܹ��𲽵����׼�߿�£��������ֳ���ƫ��ܴ�����
				if(m_bWheelOpposite==FALSE)	//û�з������
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{										  
						m_nWheelPosObj=WHEEL_FRONT3_H;      //7�ո���
						if(m_cVehicleAngle>=2)
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_FRONT3_H;		 
						if(m_cVehicleAngle>=2)           //  mdf ���ڴ˴���ƫ��ֵ���Ա�ʹ����һ����С�ĽǶȿ���state5//pl
							m_bWheelOpposite=TRUE;
					 }
				}		
				//
				if(m_bWheelOpposite==TRUE) //���ڷ������
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
				if(m_bWheelOpposite==TRUE)//��״̬1��2����5ʱ�ѷ�����ֱ�����
					m_bWheelOpposite=FALSE;			
				if(m_bVehicleForward==TRUE)//ǰ��
				{
					if(m_cVehicleAngle>=2)//�������ƫ��Ƕȹ����Ƚ��е�ֱ����			
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
						if(m_nDiffValue>CENTER_ERR)  //================���ڴ˴�CENTER_ERR��ʹ���ֵ��ڷ��ȱ�С	 mdf
						{
							if(m_bCenterDiff==TRUE)//����-Զ��
							{	
									if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_MID_VALUE;	  //mdf m_nWheelPosObj=WHEEL_MID_VALUE;
								else
									m_nWheelPosObj=WHEEL_FRONT3_H;	 //mdf
							}
							else					//����-����
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
				else if(m_bVehicleBackward==TRUE)//����
				{
					if(m_cVehicleAngle>=2)
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_cVehicleAngle=3;		   //�ɵ���
						else
							m_cVehicleAngle=3; //pl
						if(m_bSubSymbol==TRUE)
							m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP_H;  //mdf
						else
							m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP_H;   //mdf				
					}
					else
					{
						if(m_nDiffValue>(CENTER_ERR/2))//================���ڴ˴�CENTER_ERR��ʹ���ֵ��ڷ��ȱ�С	mdf
						{	
							if(m_bCenterDiff==TRUE)//����-Զ��
							{	
								if(m_bSubSymbol==TRUE)
								{	
									if(m_bVehicleHighSpeed==TRUE)
										m_nWheelPosObj=WHEEL_FRONT2_H;	 //������
									else
										m_nWheelPosObj=WHEEL_FRONT3_H;//pl  
								}
								else
									m_nWheelPosObj=WHEEL_MID_VALUE;	 //mdf	 m_nWheelPosObj=WHEEL_MID_VALUE;
							}
							else					//����-����
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

/////////////////////////////��泵/////////////////////////////////////////////
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
				//���ݳ���ƫ��Ƕȣ��𲽵����ķ�ʽ����������ͷ�������֣�ʹ���ܹ��𲽵����׼�߿�£��������ֳ���ƫ��ܴ�����
				if(m_bWheelOpposite==FALSE)	//û�з������
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{
						m_nWheelPosObj=WHEEL_FRONT3_H;		//7�ո���
						if(m_cVehicleAngle>=2)			
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_FRONT3_H;	
						if(m_cVehicleAngle>=2)             // mdf���ڴ˴���ƫ��ֵ���Ա�ʹ����һ����С�ĽǶȿ���state5//pl
							m_bWheelOpposite=TRUE;
					}
				}
				//
				if(m_bWheelOpposite==TRUE) //���ڷ������
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
				//���ݳ���ƫ��Ƕȣ��𲽵����ķ�ʽ����������ͷ�������֣�ʹ���ܹ��𲽵����׼�߿�£��������ֳ���ƫ��ܴ�����
				if(m_bWheelOpposite==FALSE)	//û�з������
				{	
					if(m_bVehicleHighSpeed==TRUE)
					{										  
						m_nWheelPosObj=WHEEL_BACK3_H;      //7�ո���
						if(m_cVehicleAngle>=2)
							m_bWheelOpposite=TRUE;
					}
					else
					{
						m_nWheelPosObj=WHEEL_BACK3_H;		 
						if(m_cVehicleAngle>=2)           //  mdf ���ڴ˴���ƫ��ֵ���Ա�ʹ����һ����С�ĽǶȿ���state5//pl
							m_bWheelOpposite=TRUE;
					 }
				}		
				//
				if(m_bWheelOpposite==TRUE) //���ڷ������
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
				if(m_bWheelOpposite==TRUE)//��״̬1��2����5ʱ�ѷ�����ֱ�����
					m_bWheelOpposite=FALSE;			
				if(m_bVehicleForward==TRUE)//ǰ��
				{
					if(m_cVehicleAngle>=2)//�������ƫ��Ƕȹ����Ƚ��е�ֱ����			
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
						if(m_nDiffValue>CENTER_ERR)  //================���ڴ˴�CENTER_ERR��ʹ���ֵ��ڷ��ȱ�С	 mdf
						{
							if(m_bCenterDiff==TRUE)//����-Զ��
							{	
									if(m_bSubSymbol==TRUE)
									m_nWheelPosObj=WHEEL_MID_VALUE;	  //mdf m_nWheelPosObj=WHEEL_MID_VALUE;
								else
									m_nWheelPosObj=WHEEL_BACK3_H;	 //mdf
							}
							else					//����-����
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
				else if(m_bVehicleBackward==TRUE)//����
				{
					if(m_cVehicleAngle>=2)
					{
						if(m_bVehicleHighSpeed==TRUE)
							m_cVehicleAngle=3;		   //�ɵ���
						else
							m_cVehicleAngle=3; //pl
						if(m_bSubSymbol==TRUE)
							m_nWheelPosObj=WHEEL_MID_VALUE - m_cVehicleAngle*WHEEL_STEP_H;  //mdf
						else
							m_nWheelPosObj=WHEEL_MID_VALUE + m_cVehicleAngle*WHEEL_STEP_H;   //mdf				
					}
					else
					{
						if(m_nDiffValue>(CENTER_ERR/2))//================���ڴ˴�CENTER_ERR��ʹ���ֵ��ڷ��ȱ�С	mdf
						{	
							if(m_bCenterDiff==TRUE)//����-Զ��
							{	
								if(m_bSubSymbol==TRUE)
								{	
									if(m_bVehicleHighSpeed==TRUE)
										m_nWheelPosObj=WHEEL_BACK2_H;	 //������
									else
										m_nWheelPosObj=WHEEL_BACK3_H;//pl  
								}
								else
									m_nWheelPosObj=WHEEL_MID_VALUE;	 //mdf	 m_nWheelPosObj=WHEEL_MID_VALUE;
							}
							else					//����-����
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
//���Ʒ�����λ�ã����Ե�λ������ʵʱ���
void Wheel_Pos_Control()
{
	if((m_bWheelLock==FALSE)&&(m_bWheelDataOk==TRUE))//������û�б������ҷ������Ƹ������Ѿ�׼�����ˣ������ƶ��Ƹ�
	{
		m_bWheelDataOk=FALSE;
		
		if(m_bBackMidWheel==TRUE)//������ڻ��У���ⷽ�����Ƿ��Ѿ���ɻ���
		{
			if((m_nDist[2] >= (WHEEL_MID_VALUE-WHEEL_ERR)) && (m_nDist[2] <= (WHEEL_MID_VALUE+WHEEL_ERR)))//��ⷽ�����Ƿ����е��λ��
				m_bBackMidWheel=FALSE;
		}
		//	
		if(((m_nDist[2] <= (m_nWheelPosObj-WHEEL_ERR))&&(m_nDist[2] >=WHEEL_MIN_BACK_VALUE)))		   //mdf  ������Ч�����Ƹ˲���
		{
			Move_Wheel(FRONT_WHEEL);
			m_bMoveFront=TRUE;
			m_bMoveBack=FALSE;
			Control_Mid_Relay(FALSE);
		}
		else if((m_nDist[2] >= (m_nWheelPosObj+WHEEL_ERR) && (m_nDist[2] <=WHEEL_MAX_FRONT_VALUE)))	  //mdf	  ��Ч�����Ƹ˲���
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
				if(m_nWheelPosObj==WHEEL_MID_VALUE)//�Ƹ�ֹͣ���������Ƹ˵�ǰ��λ�ô����е�ʱ���Ƹ˻��м̵�������պ�
				{	
					Control_Mid_Relay(TRUE);
					m_cSysError &= ~SYSERR_MID_WHEEL;
				}
				else	//���д���
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
	//�Ƹ��ƶ���Χ���޹����ж�
	if(m_nDist[2] < WHEEL_MIN_BACK_VALUE)
	{
		m_nWheelExtremeCount++;
		if(m_nWheelExtremeCount>EXTREME_VALUE1)
		{
			m_nWheelExtremeCount=0;
			m_cSysError |= SYSERR_DRAW_EXTREME;			//������
			m_cSysError &= ~SYSERR_PUSH_EXTREME;		
	   	}
	}
	else if(m_nDist[2] > WHEEL_MAX_FRONT_VALUE)
	{
		m_nWheelExtremeCount++;
		if(m_nWheelExtremeCount>EXTREME_VALUE1)
		{
			m_nWheelExtremeCount=0;
			m_cSysError |= SYSERR_PUSH_EXTREME;			//�Ƴ���
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
//�г��ٶȺ��г�������
/////////////////////////////////////////////////////////////////////////
void Vehicle_State_Detect()
{
	//�г�������
//	GPIO_ReadInputDataBit(IN_FORWARD_PORT, IN_FORWARD_PIN);//
//	IN_FORWARD = GPIO_ReadInputDataBit(IN_FORWARD_PORT, IN_FORWARD_PIN);
//	IN_BACKWARD = GPIO_ReadInputDataBit(IN_BACKWARD_PORT, IN_BACKWARD_PIN);
	if((m_bVehicleForward==TRUE)&&(m_bVehicleBackward==FALSE))//ǰ��//PL,20130512
	{
		m_bVehicleForward=TRUE;
		m_bVehicleBackward=FALSE;
		m_bStopFlag=FALSE;
	}
	else if((m_bVehicleForward==FALSE)&&(m_bVehicleBackward==TRUE))//����//PL20130512
	{
		m_bVehicleForward=FALSE;
		m_bVehicleBackward=TRUE;
		m_bStopFlag=FALSE;
	}
	else//ͣ��������ǰ�����˶�������Ҳ����ͣ������
	{
		m_bVehicleForward=FALSE;
		m_bVehicleBackward=FALSE;
		m_bStopFlag=TRUE;
		m_bBackMidWheel=TRUE;			//���������̻���
		m_nWheelPosObj=WHEEL_MID_VALUE;
		m_bWheelOpposite=FALSE;			//�����̷�����ֱ��ʧЧ
		m_bNewData=FALSE;				//ͣ��ʱ���������������ˣ�ֹͣ��Ӧ�ľ�ƫ����
		m_bOK=FALSE;
		m_nFrontExtremeCount=0;
		m_nFrontExtremeCount1=0;
		m_nBackExtremeCount=0;

	}
//	//�г��ٶȼ��
//	 IN_SPEED=GPIO_ReadInputDataBit(IN_SPEED_PORT, IN_SPEED_PIN);	
//	if(IN_SPEED==HIGH_SPEED_VAL)
//		m_bVehicleHighSpeed=TRUE;//����ģʽ
//	else
//		m_bVehicleHighSpeed=FALSE;//����ģʽ
//	//�Ƿ��ڻ���������ģʽ���
//  IN_MODE=GPIO_ReadInputDataBit(IN_MODE_PORT, IN_MODE_PIN);	
//		if(IN_MODE==TRANSFER_MODE)
//			m_bTransferWorkMode=TRUE;//����������ģʽ	
//		else
//			m_bTransferWorkMode=FALSE;//�ǻ���������ģʽ
//					
//      IN_RUDDER=GPIO_ReadInputDataBit(IN_RUDDER_PORT, IN_RUDDER_PIN);
//		if(IN_RUDDER==RIGHT_MODE)	//Ĭ���������裬��������1Ϊ�Ҷ棬0Ϊ���		
//			m_bRightRudder=TRUE;				
//		else //�Ҷ�		
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
//	STM_EVAL_LEDOff(LED4); //DRAW��ӦLED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_RESET);
	}
	else
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED4); //DRAW��ӦLED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);
	}
	if(OUT_PUSH==0x00)
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED2); //PUSH��ӦLED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);
  }
	else
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED2); //PUSH��ӦLED2
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
	OUT_DRAW=PUSH_ON;			//���Ƹ�

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
//		STM_EVAL_LEDOff(LED4); //DRAW��ӦLED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_RESET);
	}
	else
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED4); //DRAW��ӦLED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);
	}
	if(OUT_PUSH==0x00)
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED2); //PUSH��ӦLED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);
  }
	else
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_SET)	;
		STM_EVAL_LEDOn(LED2); //PUSH��ӦLED2
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
//		STM_EVAL_LEDOff(LED4); //DRAW��ӦLED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_RESET);
	}
	else
	{
//	GPIO_WriteBit(OUT_DRAW_PORT, OUT_DRAW_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED4); //DRAW��ӦLED4
		GPIO_WriteBit(GPIOH,GPIO_Pin_15, Bit_SET);
	}
	if(OUT_PUSH==0x00)
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_RESET);
//		STM_EVAL_LEDOff(LED2); //PUSH��ӦLED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_RESET);
  }
	else
	{
//	GPIO_WriteBit(OUT_PUSH_PORT, OUT_PUSH_PIN, Bit_SET)	;
//		STM_EVAL_LEDOn(LED2); //PUSH��ӦLED2
		GPIO_WriteBit(GPIOH,GPIO_Pin_14, Bit_SET);
	}
}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!�������ݴ����䣨�漰AD��+2015/08/25!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void Control_Mid_Relay(u8_t on)//������д���
{
	
}
void Move_Wheel(u8_t m_bDir)
{
	if(m_bDir==FRONT_WHEEL)			//���Ƹ�
		Push_Wheel();	
	else
		Draw_Wheel();
}


