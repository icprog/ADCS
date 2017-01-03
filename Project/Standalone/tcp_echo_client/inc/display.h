/*_________________________ Variable ________________________________________*/
/*H**************************************************************************
* NAME:    display.h
*----------------------------------------------------------------------------*/
//
#ifndef _display_h
#define _display_h
/*_____ I N C L U D E S ____________________________________________________*/
//
#include "data_collection.h"
/*_____ M A C R O S ________________________________________________________*/
//

/*_____ D E F I N I T I O N ________________________________________________*/
//
#define DIS_DIN_PORT 0x01		/* U377.0 */
#define DIS_CLK_PORT 0x02		/* U377.1 */
#define DIS_LOAD_PORT 0x04		/* U377.2 */
/////////////////////////////////////////////////////////
//B������0~9����
//#define CODE_ 0x0a
//#define CODE_E 0x0b
//#define CODE_H 0x0c
//#define CODE_L 0x0d
//#define CODE_P 0x0e
//#define CODE_BLANK 0x0f
//������0~f�Ժ�
#define NOCODE_P 0xE7
#define NOCODE_F_H 0x47      //F ����ǰ��
#define NOCODE_F 0xC7		 //F.����ǰ�� 
#define NOCODE_B 0xFF		 //B.���ٺ���
#define NOCODE_B_H 0x7F		 //B ���ٺ���
#define NOCODE_L 0x0E	  
//#define NOCODE_T 0x46
//#define NOCODE_S 0x5B
#define NOCODE_  0x01
#define NOCODE_E 0x4f
#define NOCODE_R 0x05
//#define NOCODE_A 0x77

#define NOCODE_BLANK 0x0
//LEDָʾ����Ϣ
#define LED_GROUP0	0x05		/* 7219.5 */
#define LED_GROUP1	0x06		/* 7219.6 */
//
#define	FAST_LED_GROUP0		0x40
#define	SLOW_LED_GROUP0		0x20
#define	FWRD_LED_GROUP0		0x10
#define	BWD_LED_GROUP0		0x08
#define	PUSH_LED_GROUP0		0x04
#define	DRAW_LED_GROUP0		0x02
//
#define	RX_LED_GROUP1		0x40
#define	TX_LED_GROUP1		0x20
#define	GPWR_LED_GROUP1		0x10
#define	ERROR_LED_GROUP1	0x08
/////////////////////////////////////////////////////////
#define Select_Decode_7219() Operate_7219(0x09,0xff)
#define Select_Nodecode_7219() Operate_7219(0x09,0)
/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Operate_7219(u8_t h,u8_t l);		//����7219���ӳ���
void Display();
void Init_7219();				//��ʼ��7219
void Init_Display_Data();
void Display_Stop();
void Send_ErrorInf_Out();
void Self_Test();
//------
void Display_MainPro();		//�����͵����豸������ʱ����
void Display_ErrInfo();		//��ͣ�����豸�رպ���ʾ������Ϣ
void Dis_SensorVal();
void Dis_ControlInf();
#endif



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