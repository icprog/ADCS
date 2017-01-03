/**
  ******************************************************************************
  * @file    data_collection.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011 
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
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

/* Define to prevent recursive inclusion -------------------------------------*/


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4x7_eth_bsp.h"
#include "lwip/tcp.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define USE_LCD        /* enable LCD  */  
//#define USE_DHCP       /* enable DHCP, if disabled static address is used */
   
/* Uncomment SERIAL_DEBUG to enables retarget of printf to  serial port (COM1 on STM32 evalboard) 
   for debug purpose */   
//#define SERIAL_DEBUG




/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */  
void Time_Update(void);
void Delay(uint32_t nCount);
void Data_collection(void);
/* ��Ƭ����������еı�־λ ------------------------------------------------------------*/
#define IC_FALSE 0x00//��Ƭ���ʹ��������ӱ�־��00��ʾʧ��
#define IC_TRUE  0x01//��Ƭ���ʹ��������ӱ�־��01��ʾ�ɹ�  
#define N_OBJECT 0x100//�������ͨ���ڵı�־λ��100��ʾ���������
#define Y_OBJECT 0x200//�������ͨ���ڵı�־λ��200��ʾ������������
#define ERR_OBJECT 0x300//�������ͨ���ڵı�־λ��300��ʾ������
#define PASS_EMPTY 0x00//ͨ��Ϊ��
#define PASS_EXIT  0x01//�ҵ�����2����׼
/* ��������------------------------------------------------------------*/
//#define NORMAL_DATA_MAX 2000//��׼�봫������ֱ�������ֵ
//#define NORMAL_DATA_MIN 600//��׼�봫������ֱ������Сֵ
#define STR_DATA_MAX 10000  
#define ct_MIN 10 //���ڻ�׼���Ϲ�����������С��ֵ
#define ct_MAX 35 //���ڻ�׼���Ϲ��������������ֵ
#define NORMAL_DATA_MAX  900*2//��׼�봫������ֱ�������ֵ(Ϊ��ʵֵ����Ҫ����50HZ 0.5���2����)
#define NORMAL_DATA_MIN  400*2//��׼�봫������ֱ������Сֵ
#define STR_DATA_MAX_Vertical NORMAL_DATA_MAX //��ʼ���ݹ�������(���������ֵ)
#define LESS_DATA_DES 100//���ڻ�׼��������������ֱ����
#define LESS_DATA_TIME 5//С�ڳ�����ֵ��ֵ�Ĵ���
#define Scan_In_Angle 40.0 //ɨ����ʼ�Ƕ�
#define Scan_Fi_Angle 140.0 //ɨ����ֹ�Ƕ�
#define Scan_Angle_Resolution 0.5//ɨ��Ƿֱ���
#define Size_str_temp (int)((Scan_Fi_Angle-Scan_In_Angle)/Scan_Angle_Resolution+1.0) //str_temp�����С
#define Relay_distance 50 //������������ο�������뷶Χ��ֵ��

/*______LCD��ʾģʽ���Ʊ���_YZ_2015/11/26_________________________________________________________*/
#define dTime_Value 50 //�л�LCD��Ļ��ʾ������������������ʱ����ֵ(�������¼����ڴ���ֵʱ����ʾͼ������л�)

/* ��⵽��׼�ı�־------------------------------------------------------------*/
#define JUDGE_DATA_THRESHOLD 100//�жϻ�׼�뱳����࣬�ҵ�ͻ���
#define JUDGE_ANGLE_THRESHOLD1 0//�ж��������ĽǶ�
#define JUDGE_ANGLE_THRESHOLD2 30//�ж��������ĽǶ�
#define POS_DATA_THRESHOLD 100//�жϳ�ͷ�˶�λ�õ���ֵ
#define R_length  100.0  //��׼�뾶10.0cm ��������׼ʱ�뾶Ϊ0
//#define R_length  50.0  //��׼ֱ��20cm ��Ӧ�뾶10.0cm (0.50�Ƿֱ����´������������ݾ�������Ϊ1/2 �������Ϊ50)

//2015/09/11
#define SENSOR_DISTANCE_ERR 300//ԭʼ��������ƫ���жϱ�׼����

/* ��λ�Ͳ��ٱ�־-----------------------------------------------------------*/
//#define SENOR_DATA_SIZE 100//	�����������仯�������С
#define TRUE 0x01//	�����������仯�������С
#define FALSE 0x00//�����������仯�������С

/* ��ջ������-----------------------------------------------------------*/
#define Stack_Size 10 //��ջ�Ĵ�С(Ҳ�����궨ʱ����궨��������Ĵ�С)


/**************************���ϳ���ʱ�����ӵı�������_2015/08/24******************************/ 
extern  u8_t m_bDistanceSampleOk  ;
extern double SENSOR_DISTANCE_FRONT;//��ͷ���׼�߾���
extern double SENSOR_DISTANCE_BACK;//��β���׼�߾���
extern double SENSOR_DISTANCE_TEMP[2]; //����ԭʼ��������(1Ϊ��ͷ�����׼�߾��� 2Ϊ��λ�����׼�߾���)�ڽ������ݺͷ���ȷ�Ϻ��ٽ��������ݴ���SENSOR_DISTANCE_FRONT/SENSOR_DISTANCE_BACK��
extern u8_t SENSOR_DISTANCE_ERR_CNT;//���ڼ�¼ԭʼ����ƫ��(ƫ���׼����ΪSENSOR_DISTANCE_ERR)֮ǰһ�δ洢�ĺϷ����ݵĴ���
//2015/08/25
extern u8_t m_bBackMidWheel;
extern u8_t m_bWheelDataOk;
extern u8_t m_bStopFlag;	
extern u8_t m_bVehicleForward;		    //�г�ǰ�������ǣ�TRUE����ǰ����FALSEû��ǰ��
extern u8_t m_bVehicleBackward;			//�г����˷����ǣ�TRUE���ں��ˣ�FALSEû�к���
extern u8_t  BD_ENABLE ; //�궨�������λ
extern u8_t IntersectLocate;  // ����Ǻű�����FALSE��Ӧ��ʼ�Ƕ�Ϊ35.0�����ݣ�TRUE��Ӧ��ʼ�Ƕ�Ϊ35.1667������(ԭʼ���ݽǷֱ���Ϊ0.3��)
extern u8_t m_bStateBefore; //ǰһʱ���˶�����״̬��־λ �����洢ǰһʱ�̵��˶�������(0x01)���Ƿ���(0x00)����Ϣ
extern u8_t IntersectOK;         // �ж�һ������(14A��14B)�Ƿ��Ѿ����洢�浽str_temp3���鵱��
extern u8_t IntersectCount;     // ��������渳ֵ���ݵļ������� IntersectCount�ڳ��򴢴�str_temp1��+1 �ڴ���str_temp2����+1 ��IntersectCount == 2ʱ��ʼ���渳ֵ
extern u16_t str_temp[Size_str_temp]; //�� str_temp1 �� str_temp2 ���洢�浽����str_temp3���У�����֮������ݴ���
extern u16_t str_temp_inverse[Size_str_temp]; //����ߵ�������
extern u16_t Time_count;   //Ϊ����֤�����Ƿ���ִ��Data_collection.c�п������趨�ļ�������(��1++��10000������ظ�++)
extern u8_t LCD_Interface1; //LCD�Ƿ���ʾ���ݽ���1����
extern u8_t LCD_Interface2; //LCD�Ƿ���ʾ���ݽ���2����
extern u8_t jc_Record; //��¼����ʱ�̸ð����ݵĲο�����Ŀ,���ڽ��һ����ɨ�赽4���ο�������_YZ_2015/12/10
/*______LCD��ʾģʽ���Ʊ���_YZ_2015/11/26_________________________________________________________*/
extern u8_t Key_Button_Push; //KEY�����Ƿ��±�ǲ���
extern u8_t LCD_Change_Mode; //LCD��ʾģʽ�л��Ƿ�������
extern u16_t LCD_Change_Mode_InTime; //LCD��ʾģʽ�л�ʱ��������¼����(Key���³�ʼʱ��)
extern u16_t LCD_Change_Mode_FiTime; //LCD��ʾģʽ�л�ʱ��������¼����(Key�ɿ���ֹʱ��)
extern u16_t LCD_Change_Mode_dTime; //LCD��ʾģʽ�л�ʱ��������¼����(Key�������ɿ����ʱ��)
extern u8_t LCD_Vehicle_Trail; //LCD�Ƿ���ʾ�����켣ͼ����
extern u8_t LCD_SENSOR_Image;  //LCD�Ƿ���ʾ������ɨ��ͼ�����
extern u8_t LCD_Vehicle_Posture; //LCD�Ƿ���ʾ������̬ͼ�����
extern double SCANNER_CENTER ;
extern double LEFT_MIN_VALUE_BACK  ;	
extern double RIGHT_MAX_VALUE_BACK ;	
extern double LEFT_MIN_VALUE_FRONT  ;	
extern double RIGHT_MAX_VALUE_FRONT ;	

/*______�������ʹ�ö�ջ�洢�����Ĳ���_YZ_2015/12/13_________________________________________________________*/
extern double Vehicle_X_Stack[Stack_Size]; //���泵��λ�ú������ջ
extern double Vehicle_Y_Stack[Stack_Size]; //���泵��λ���������ջ
extern double Vehicle_sincx_Stack[Stack_Size]; //���泵��ƫת��sinֵ��ջ
extern double Vehicle_coscx_Stack[Stack_Size]; //���泵��ƫת��cosֵ��ջ
extern double Vehicle_Output_Sum_X; //��ջ�������������ֵ
extern double Vehicle_Output_Sum_Y; //��ջ�������������ֵ
extern double Vehicle_Output_Sum_sincx; //��ջ�������ƫת��sinֵ
extern double Vehicle_Output_Sum_coscx; //��ջ�������ƫת��cosֵ
extern double Standard_Position_x1_Stack; //�궨ʱ���ɶ�ջ����ó���׼λ�ú�����
extern double Standard_Position_y1_Stack; //�궨ʱ���ɶ�ջ����ó���׼λ��������
extern double Standard_Position_x2_Stack; //�궨ʱ���ɶ�ջ����ó��ڶ����ο��������
extern double Standard_Position_y2_Stack; //�궨ʱ���ɶ�ջ����ó��ڶ����ο���������
extern double Standard_Position_x1_Stack_Sum[Stack_Size]; //���泵���궨ʱ�̱�׼λ�ú�����
extern double Standard_Position_y1_Stack_Sum[Stack_Size]; //���泵���궨ʱ�̱�׼λ��������
extern double Standard_Position_x2_Stack_Sum[Stack_Size]; //���泵���궨ʱ�̵ڶ����ο��������
extern double Standard_Position_y2_Stack_Sum[Stack_Size]; //���泵���궨ʱ�̵ڶ����ο���������
extern u8_t Stack_Sum_Num; //��ջ����Ǽ�������
extern u8_t Stack_Replace_Num; //��ջˢ������(��һ����ǰ�滻)��������
extern u8_t Vehicle_Stack_Num; //��ջ�����������
extern u8_t Vehicle_Stack_Full;//��ջ�洢�Ƿ���������
extern u8_t BD_Stack_Num;      //�궨λ�ö�ջ�����������
extern u8_t BD_Stack_Sum_Num;  //�궨λ�ö�ջ��ͼ�������
extern u8_t BD_Stack_Replace_Num; //�궨λ�ö�ջˢ�����ݼ�������
extern u16_t A_count;  //��¼�ɼ������ݰ��ĸ���								
							
#ifdef __cplusplus
}
#endif




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

