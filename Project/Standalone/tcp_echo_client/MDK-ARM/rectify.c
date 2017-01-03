//==============================================
//*************  Rectify.c  ***********************
//==============================================
#include "rectify.h"
#include "data_collection.h"
#include "display.h"
#include "timer.h"
#include "execute.h"
#include "stm32f4_GPIO_int.h"
/*_________________________ Variable ________________________________________*/
extern u8_t m_bNewData;	
extern u8_t m_bOK;                 //��û�����������߱�־λ
extern u32_t m_nFrontExtremeCount;		//�ֳ��޴���
extern u32_t m_nFrontExtremeCount1;	    //��ʼ�߳��ֳ��޴���
extern u32_t m_nBackExtremeCount;		//��������ʼ��ʱ���޴�������

extern u32_t m_nSubValue;				//��ֵ(����������)
extern u32_t m_nDiffValue;				//�������ͻ�׼��֮���ƫ��ֵ
extern u8_t m_bSubSymbol;				//��ֵ�ķ���

extern u8_t m_bFrontLeft0;				//һ��ƫ���׼��
extern u8_t m_bFrontRight0;				//һ��ƫ���׼��
extern u8_t m_bBackLeft0;				//һ��ƫ���׼��
extern u8_t m_bBackRight0;				//һ��ƫ���׼��

extern u8_t m_bCenterDiff;				//�������׼�ߵ�ƫ��ֵ��TRUE�����ң�FALSE������
extern u8_t m_cVehicleState;			//������̬		
extern u8_t m_bWheelLock;				//�������Ƿ�������ǣ�0δ������1����


//----------------------------------------------------------------------
extern u8_t m_bDistanceSampleOk ;				//��ഫ�����ɼ����ݺϷ�
extern u8_t m_bStopFlag;
extern double SENSOR_DISTANCE_FRONT;//��ͷ���׼�߾���
extern double SENSOR_DISTANCE_BACK;//��β���׼�߾���
extern u8_t m_bVehicleBackward;  //���˱�־λ
extern u8_t m_bVehicleForward;  //ǰ����־λ �������Ҫ��ֵ������ܽ�
//------------------------------------
/////////////////////////////////////////////////////////////////////////
///��ʼ������
void Init_Rectify_Data()
{
	m_bNewData=FALSE;			
	m_bFrontLeft0=TRUE;				
	m_bFrontRight0=TRUE;				
	m_bStopFlag=TRUE;
	m_bOK=FALSE;						//��û�����������߱�־λ
	m_nBackExtremeCount=0;
	m_nFrontExtremeCount=0;
	m_nFrontExtremeCount1=0;
//	WHEEL_CHANCE=1;
}
/////////////////////////////////////////////////////////////////////////
///�������������������ھ�ƫ�жϵ�����
void Analyse_RectifyData()
{
	if(m_bDistanceSampleOk ==TRUE)				//���յ���ƫ������
	{
		m_bDistanceSampleOk =FALSE;
		//--------------------------------������ֵ
		//�����ֵ��SENSOR_DISTANCE_FRONT��ǰ��������SENSOR_DISTANCE_BACK���󴫸���
		if(SENSOR_DISTANCE_BACK > SENSOR_DISTANCE_FRONT) //������ƫ
		{
			m_nSubValue=SENSOR_DISTANCE_BACK-SENSOR_DISTANCE_FRONT;				//��ֵ(����������)
			m_bSubSymbol=TRUE;								//TRUE��ʾ��ֵΪ��
		}
		else					//������ƫ����ƽ��
		{
			m_nSubValue=SENSOR_DISTANCE_FRONT-SENSOR_DISTANCE_BACK;				//��ֵ(����������)
			m_bSubSymbol=FALSE;								//FALSE��ʾ��ֵΪ��	
		}
		//---------------------------------------------------------------
		if(m_bVehicleForward==TRUE)	//��������ڻ�׼�ߵ�λ���жϣ�ע�����һ���---ǰ��
		{
			if(SENSOR_DISTANCE_FRONT < LEFT_MIN_VALUE_FRONT)
			{
				m_bFrontLeft0=TRUE;
				m_bFrontRight0=FALSE;				
				m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_FRONT;//calculate the difference between sensor0 and Center
			}
			else if(SENSOR_DISTANCE_FRONT > RIGHT_MAX_VALUE_FRONT)
			{

				m_bFrontLeft0=FALSE;
				m_bFrontRight0=TRUE;
				m_nDiffValue=SENSOR_DISTANCE_FRONT-SCANNER_CENTER;//calculate the difference between sensor0 and Center
			}
			else 
			{
				m_bFrontLeft0=FALSE;
				m_bFrontRight0=FALSE;				
				if(SENSOR_DISTANCE_FRONT>SCANNER_CENTER)//����ƫ����������࣬��ͷ��ȥ���Ĳ�ֵΪ��
				{
					m_bCenterDiff=TRUE; //TRUE��ʾ��ֵΪ��(������),FALSE��ʾ��ֵΪ��(������)
					m_nDiffValue=SENSOR_DISTANCE_FRONT-SCANNER_CENTER;
				}
				else							  //����ƫ����������࣬��ֵΪ��
				{	
					m_bCenterDiff=FALSE;
					m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_FRONT;
				}
			}
		}
		else if(m_bVehicleBackward==TRUE)	//ƫ���жϣ�ע�����һ���---����
		{
			if(SENSOR_DISTANCE_BACK < LEFT_MIN_VALUE_BACK)
			{
				m_bBackLeft0=TRUE;
				m_bBackRight0=FALSE;
				m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_BACK;//calculate the difference between sensor1 and Center
			}    
			else if(SENSOR_DISTANCE_BACK > RIGHT_MAX_VALUE_BACK )
			{
				m_bBackLeft0=FALSE;
				m_bBackRight0=TRUE;
				m_nDiffValue=SENSOR_DISTANCE_BACK-SCANNER_CENTER;//calculate the difference between sensor1 and Center
			}
			else 
			{
				m_bBackLeft0=FALSE;
				m_bBackRight0=FALSE;
				if(SENSOR_DISTANCE_BACK>SCANNER_CENTER)//����ƫ���������Ҳ࣬��λ��ȥ���Ĳ�ֵΪ��
				{
					m_bCenterDiff=TRUE;
					m_nDiffValue=SENSOR_DISTANCE_BACK-SCANNER_CENTER;
				}
				else							  //����ƫ����������࣬��ֵΪ��
				{	
					m_bCenterDiff=FALSE;
					m_nDiffValue=SCANNER_CENTER-SENSOR_DISTANCE_BACK;
				}
			}
		}
		//-------------------------------------------
		//��ƫ�Ƴ��޹����ж�

		if((SENSOR_DISTANCE_FRONT > RIGHT_EXTREME_VALUE || SENSOR_DISTANCE_BACK > RIGHT_EXTREME_VALUE))
		{
			 if((SENSOR_DISTANCE_FRONT==0||SENSOR_DISTANCE_BACK==0) && (m_bOK==FALSE))
			{
			    m_nFrontExtremeCount1++;
				m_nFrontExtremeCount=0;	
				if(m_nFrontExtremeCount1>EXTREME_VALUE1)
				{
					m_nFrontExtremeCount1=0;
					m_cSysError |= SYSERR_RIGHT_EXTREME;			//ɨ�賵������ƫ�Ƴ���
					m_cSysError &= ~SYSERR_LEFT_EXTREME;		//ɨ�賵����ƫ��δ����
		   		}

			}
			else 
			{
				m_nFrontExtremeCount++;
				m_nFrontExtremeCount1=0;
				if(m_nFrontExtremeCount>EXTREME_VALUE)
				{
					m_nFrontExtremeCount=0;
					m_cSysError |= SYSERR_RIGHT_EXTREME;			//ɨ�賵������ƫ�Ƴ���
					m_cSysError &= ~SYSERR_LEFT_EXTREME;		//ɨ�賵����ƫ��δ����
		   		}

			}
		}
		else if((SENSOR_DISTANCE_FRONT < LEFT_EXTREME_VALUE) || (SENSOR_DISTANCE_BACK < LEFT_EXTREME_VALUE))
		{
			m_nFrontExtremeCount++;
			if(m_nFrontExtremeCount>EXTREME_VALUE)
			{
				m_nFrontExtremeCount=0;
				m_cSysError |= SYSERR_LEFT_EXTREME;		//ɨ�賵������ƫ�Ƴ���
				m_cSysError &= ~SYSERR_RIGHT_EXTREME;		//ɨ�賵����ƫ��δ����
			}
		}
		else  if(((SENSOR_DISTANCE_FRONT < RIGHT_EXTREME_VALUE-10) && (SENSOR_DISTANCE_FRONT> LEFT_EXTREME_VALUE+10))&&((SENSOR_DISTANCE_BACK < RIGHT_EXTREME_VALUE-10) && (SENSOR_DISTANCE_BACK > LEFT_EXTREME_VALUE+10)))
		{
			m_nFrontExtremeCount=0;
			m_nFrontExtremeCount1=0;
			m_cSysError &= ~SYSERR_LEFT_EXTREME;		//ɨ�賵����ƫ��δ����
			m_cSysError &= ~SYSERR_RIGHT_EXTREME;		//ɨ�賵����ƫ��δ����
		    m_nBackExtremeCount++;
			if(m_nBackExtremeCount>EXTREME_VALUE/2)
			{
				m_bOK=TRUE;
				m_nBackExtremeCount=0;
			}
		}
		m_bNewData=TRUE;
	//	m_bNewDisData=TRUE;
	
		m_cSysError &= ~SYSERR_5S_NODATA;

	}
}
/////////////////////////////////////////////////////////////////////////
//��ƫ�ж�
void Rectify()
{
	if(m_bNewData==TRUE) //û���µ����ݲ��þ�ƫ
	{			
		m_bNewData=FALSE;
		//��ƫ�ж�
		Drive_Judge();				
	}
}
/////////////////////////////////////////////////////////////////////////
///
void Drive_Judge()
{
	if(m_bVehicleForward==TRUE)				//ǰ��ʱ
	{
		if((m_bSubSymbol==FALSE)&&(m_bFrontLeft0==TRUE))//��ֵΪ����������ƫ������ƫ��׼�����F1(��Ӧrectify.h��Case 1)
		{//����Ӧ�복����һ�£���һ���ʵ�ʱ��������֣�ʹ���ֻ��������׼�߷���һ��
			m_cVehicleState=STATE_1;	
		}
		else if((m_bSubSymbol==TRUE)&&(m_bFrontRight0==TRUE))//��ֵΪ����������ƫ������ƫ��׼���Ҳ�F2(��Ӧrectify.h��Case 2)
		{//����Ӧ�복����һ�£���һ���ʵ�ʱ�����Ҵ��֣�ʹ���ֻ��������׼�߷���һ��
			m_cVehicleState=STATE_2;
		}
		else if((m_bSubSymbol==TRUE)&&(m_bFrontLeft0==TRUE))//��ֵΪ����������ƫ������ƫ��׼�����F3(��Ӧrectify.h��Case 3)
		{//����Ӧ���Ҵ�ֱ������F1
			m_cVehicleState=STATE_3;
		}
		else if((m_bSubSymbol==FALSE)&&(m_bFrontRight0==TRUE))//��ֵΪ����������ƫ������ƫ��׼���Ҳ�F4(��Ӧrectify.h��Case 4)
		{//����Ӧ�����ֱ������F2
			m_cVehicleState=STATE_4;
		}
		else if((m_bFrontLeft0==FALSE)&&(m_bFrontRight0==FALSE))//ǰ���Ѿ����������׼�г��������ڲ�
		{
			m_cVehicleState=STATE_5;
		}
	}
	else		//����ʱ
	{
		if((m_bSubSymbol==TRUE)&&(m_bBackLeft0==TRUE))//��ֵΪ����������ƫ������ƫ��׼�����B1(��Ӧrectify.h��Case 3)
		{//����Ӧ�복����һ�£���һ���ʵ�ʱ��������֣�ʹ���ֻ��������׼�߷���һ��
			m_cVehicleState=STATE_1;
		}				
		else if((m_bSubSymbol==FALSE)&&(m_bBackRight0==TRUE))//��ֵΪ����������ƫ������ƫ��׼���Ҳ�B2(��Ӧrectify.h��Case 4)
		{//����Ӧ�복����һ�£���һ���ʵ�ʱ�����Ҵ��֣�ʹ���ֻ��������׼�߷���һ��
			m_cVehicleState=STATE_2;
		}
		else if((m_bSubSymbol==FALSE)&&(m_bBackLeft0==TRUE))//��ֵΪ����������ƫ������ƫ��׼�����B3(��Ӧrectify.h��Case 1)
		{//����Ӧ�����Ҵ�ֱ������B1
			m_cVehicleState=STATE_3;
		}
		else if((m_bSubSymbol==TRUE)&&(m_bBackRight0==TRUE))//��ֵΪ����������ƫ������ƫ��׼���Ҳ�B4(��Ӧrectify.h��Case 2)
		{//����Ӧ�����ֱ������B2
			m_cVehicleState=STATE_4;
		}
		//�����Ѿ������׼�г������򣬳��ֻ�������׼���ϣ�ʵ�ʴ��ֽǶ�Ϊ�������׼�߼нǣ�����Ҫ���ݳ�����̬ʵʩ������ֱ���н�Ϊ��
		else if((m_bBackLeft0==FALSE)&&(m_bBackRight0==FALSE))
		{
			m_cVehicleState=STATE_5;
		}
	}
}
/////////////////////////////////////////////////////////////////////////
//�������������ϼ��
void Check_Wheel_Lock()
{
 IN_LOCK_WHEEL=GPIO_ReadInputDataBit(IN_LOCK_WHEEL_PORT, IN_LOCK_WHEEL_PIN);
	if(IN_LOCK_WHEEL==LOCK_WHEEL_VALUE)
	{
		m_cSysError |= SYSERR_WHEEL_LCOK;
		m_bWheelLock=TRUE;
	}
	else
	{
		m_cSysError &= ~SYSERR_WHEEL_LCOK;
		m_bWheelLock=FALSE;	
	}
}

