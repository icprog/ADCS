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

/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Operate_7219(u8_t h,u8_t l);		//����7219���ӳ���
void Display(void);
void Init_7219(void);				//��ʼ��7219
void Init_Display_Data(void);
void Display_Stop(void);
void Send_ErrorInf_Out(void);
void Self_Test(void);
//------
void Display_MainPro(void);		//�����͵����豸������ʱ����
void Display_ErrInfo(void);		//��ͣ�����豸�رպ���ʾ������Ϣ
void Dis_SensorVal(void);
void Dis_ControlInf(void);


//��������
extern u8_t m_cDisplayBuffer[4];
//
//extern u8_t m_cP377;
 //------------------------
//extern u8_t m_bNewDisData;			//ˢ����ʾ����
//extern u8_t m_bDisFlag;
 //
//extern u8_t m_cDisErrCnt;				//��ʾ������Ϣ�ı�ʾѭ�����
extern u8_t m_cDisDataDelay;			//��ʾ������ʱ
extern u8_t m_cSysError;			//�ò�ͬ��λ��ʾϵͳ����



#endif



