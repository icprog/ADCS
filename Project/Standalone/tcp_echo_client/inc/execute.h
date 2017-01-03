/***************************************************************************
* NAME:    Execute.h
*----------------------------------------------------------------------------*/
//
#ifndef _Execute_h
#define _Execute_h

/*_____ I N C L U D E S ____________________________________________________*/
//
#include "data_collection.h"

/*_____ M A C R O S ________________________________________________________*/
//
#define U377_MID_PORT 0x01			/* u377.0 */
#define U377_ERROR_PORT 0x02		/* u377.1 */
#define U377_ERRLED_PORT 0x04		/* u377.2 */
/*_____ D E F I N I T I O N ________________________________________________*/
//
#define PUSH_ON	1
#define PUSH_OFF 0
//
#define FRONT_WHEEL 0
#define BACK_WHEEL 1
#define EXTREME_VALUE1	50
#define EXTREME_VALUE	20	   //50
//ǰ���������ֵĲ�ֵ����ʾ�������б�Ƕ�
#define DIFF_VALUE4		100
#define DIFF_VALUE3		80
#define DIFF_VALUE2		40
#define DIFF_VALUE1		20	

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define PushRod 0x01 //���Ƹ˱��� m_nDist[2]�����޸�
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//-------------------------------------------
////////////////////////////////////////////////////////////////////////////
//
/*_____ D E C L A R A T I O N ______________________________________________*/
//
void Push_Wheel();
void Draw_Wheel();
void Stop_Wheel();
//
void Init_Execute_Data();
void Move_Wheel(u8_t m_bDir);
void Adjust_Deflect();
void Adjust_Parallel();
//
void Control_Mid_Relay(u8_t on);
void Control_Error_Relay(u8_t on);
void Control_ErrLed_Relay(u8_t on);
//
void Init_IoPort();			//��ʼ��I/O��
//
void Wheel_Pos_Control();//�Ƹ�λ�ü��
//
void Vehicle_State_Detect();//ģʽ�������ٶȼ��
#endif
