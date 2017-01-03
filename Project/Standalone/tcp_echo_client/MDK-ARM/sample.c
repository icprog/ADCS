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

u32_t m_nSBuffer[3];             //��·ʵʱ�ɼ���ֵ
u32_t m_nOBuffer[3];				//�����һ�ε�����
u8_t m_cMutationCnt[3];		//��������ͻ�����
//
u8_t count;					//���ڲ�����ʱ
u8_t m_cZdcsSample;			//���ɼ����ڼ�����
//
u8_t m_cADChannel;				//��ǰ�ɼ��ڼ���ͨ�� 
//
u8_t m_cSampleDelay;				//���ݲɼ���ʱ��5�Σ�50ms


u32_t adval_Stack[adval_Stack_Size]={0}; //����ad����ֵ��ջ
u8_t adval_Count_Num=0; //����ad��ջ��������
u32_t adval_Sum=0;//����ad��ջ��ͱ���
u8_t adval_Sum_Num=0; //����ad��ջ��Ͳ���

///////////////////////////////////////////////////////////////////////////////
extern u32_t m_nDist[3];                //xyw
extern u8_t m_cP377;				//377���
extern u8_t m_bBackMidWheel;
extern u8_t m_bStopFlag;
extern u8_t m_bDistanceSampleOk;
extern void USART_Send(uint16_t Data);
extern u8_t LCD_Interface1;
extern u8_t LCD_Interface2;
extern u8_t LCD_Vehicle_Trail; //LCD�Ƿ���ʾ�����켣ͼ����
extern u8_t LCD_SENSOR_Image;  //LCD�Ƿ���ʾ������ɨ��ͼ�����
extern u8_t LCD_Vehicle_Posture; //LCD�Ƿ���ʾ������̬ͼ�����
unsigned short AD_print = 0;

extern volatile unsigned char clock_1s; // Flag activated each second_YZ_2015/11/20

char text[40];

//***************************************************************
//�ɼ�ADͨ����0-��һ����1-�ڶ���; 2-������ xyw 
//���ڵ�������ƫ����һ�������ݶ�Ӧ�ĳ�ͷ��β��Ϣ��ȡ������data_collection.c���У�ֻ�е��������ݵĶ�ȡ(Ҫ�õ��Ƹ˴�����)������sample.c���У������������������ֻ��2
void Sample_AD()  //�漰�Ƹ˴��������ݲɼ�(ADת��)--���޸�_YZ_2015/09/11
{
//	u8_t adnum,i;
	u32_t adval=0;  
//	u32_t result_l;
	//---------����ADC����ͨ����������_YZ_2015/09/14
//	adval=ADC_GetConversionValue(ADC3); //ADC3_channel 7--->PF9
	
	//---------����Blinky���̣�����ADC����ͨ����������_YZ_2015/11/18
	adval=ADC_GetCnv(); //Ĭ��ʹ��ADC3_channel 7--->PF9
	
//  USART_Send(adval);
//	------------------------------��ʾ-----------------------------
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
	�ɼ�����12u8_t���ɼ�ֵ��0~4096,��Ӧ0-3.3V��ʵ��4-20mA����Ӧ0.66~3.3V����Ӧ0~400mm__YZ__2015/12/17
	(�ɼ�ֵ*400/4096)mm
	------*/
	//m_nSBuffer[2]=(int)((double)adval*400.0/4096.0);
  m_nSBuffer[2]=(int)((double)adval*0.55264);
//	printf("adval=%lu\n",m_nSBuffer[2]);
	
}
//***************************************************************
//�ɼ�����
void Sample_Data()
{	u32_t n;
	
  LCD_SetBackColor(Blue);
  LCD_SetTextColor(White);

//	if((m_bStopFlag==TRUE)&& (m_bBackMidWheel==FALSE))//û���ƶ����ҷ�����û��ִ�л���   ????? && (m_bBackMidWheel==FALSE
//		return;	
	
	m_cSampleDelay++;
	if(m_cSampleDelay<2)
		return;
	//
	Sample_AD();	//��ͨ���ɼ�����xyw_����ֻ�ɼ��Ƹ˴�������������_YZ_2015/09/14
//		m_bWheelDataOk=TRUE;    //׼�����������ݷ���
		if(m_nSBuffer[2] > m_nOBuffer[2])  // ���ݷ���
			n=m_nSBuffer[2]-m_nOBuffer[2];	   
		else
			n=m_nOBuffer[2]-m_nSBuffer[2];
		if(n >= WHEEL_ERR)  //ǰ���������ݲ���ڶ�����ֵ����Ϊ�����ݷ�����Ч
		{				     

			m_nDist[2]=m_nSBuffer[2];
			m_nOBuffer[2]= m_nSBuffer[2];
	//  USART_Send(0xD2);
			Analyse_Data();			//�жϷ������ݺϷ���
		}	
    
	//
	m_cSampleDelay=0;	
}
//------------------�����Ƹ˴���������---------------------
void Analyse_Data()
{
	u32_t n;
//	u8_t i; 
//	u32_t dis_f,dis_b;//ǰ�󴫸�����������ֵ
	//
//�������жϲɼ����ݵĺϷ��ԣ����в�ഫ�������ݺϷ��ԵĴ���ŵ���data_collection.c���У�����Ĵ������ж��Ƹ˲ɼ����ݺϷ���
//�����ⲿ��ע�͵��������Ƕ�����ƫ��ֹ��ͷ��β���������ݶ��������ã����ﲻ��ʹ��_YZ_2015/12/17	
//		if(m_nSBuffer[2] > m_nOBuffer[2])
//			n=m_nSBuffer[2]-m_nOBuffer[2];
//		else
//			n=m_nOBuffer[2]-m_nSBuffer[2];
//		if((n >= SENSOR_DIFF)&&(n<=SENSOR_ERR))//PL20130512
//			m_nOBuffer[2]=m_nSBuffer[2];
//		else if(n>SENSOR_ERR)
//		{
//			m_cMutationCnt[2]++;
//			if(m_cMutationCnt[2]>9)//����10�����ݶ�������ֵ������Ϊ���ݺϷ�
//			{
//				m_cMutationCnt[2]=0;
//				m_nOBuffer[2]=m_nSBuffer[2];  //����8.0�������m_nSBuffer[i],Ӧ���޸�Ϊ m_nSBuffer[2] __YZ_2015/11/18 
//			}
//			
//		}
	
	//
	//m_bDistanceSampleOk=TRUE;		//// m_bDistanceSampleOk ��λ��data_collection.c�н���
	//
	m_nDist[0]=m_nOBuffer[0];
	m_nDist[1]=m_nOBuffer[1];
	
	//���ӳ���б�ʣ��Ӵ�����ֱ��ӳ�䵽���ֵ�ʵ��λ�ý����ж�_���ⲿ���Ƕ�����ƫ�����ڴ������İ�װλ��Ե�ʣ�������ƫ���ý�����������_YZ_2015/09/11
//	if(m_nDist[0]<m_nDist[1])//�������㣬����λ�ö���ȥ����ֵ
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
//	else //�������㣬����λ�ö���������ֵ
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
	m_nOBuffer[0]=0;			//�����һ�ε�����
	m_nOBuffer[1]=0;			//�����һ�ε�����
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

