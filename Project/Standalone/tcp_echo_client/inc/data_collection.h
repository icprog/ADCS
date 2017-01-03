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
/* 单片机处理过程中的标志位 ------------------------------------------------------------*/
#define IC_FALSE 0x00//单片机和传感器连接标志，00表示失败
#define IC_TRUE  0x01//单片机和传感器连接标志，01表示成功  
#define N_OBJECT 0x100//物体进入通道内的标志位，100表示无物体进入
#define Y_OBJECT 0x200//物体进入通道内的标志位，200表示物体正常进入
#define ERR_OBJECT 0x300//物体进入通道内的标志位，300表示物体误闯
#define PASS_EMPTY 0x00//通道为空
#define PASS_EXIT  0x01//找到多余2个基准
/* 常规数组------------------------------------------------------------*/
//#define NORMAL_DATA_MAX 2000//基准与传感器垂直距离最大值
//#define NORMAL_DATA_MIN 600//基准与传感器垂直距离最小值
#define STR_DATA_MAX 10000  
#define ct_MIN 10 //落在基准点上光束个数的最小阈值
#define ct_MAX 35 //落在基准点上光束个数的最大阈值
#define NORMAL_DATA_MAX  900*2//基准与传感器垂直距离最大值(为真实值，需要考虑50HZ 0.5°乘2问题)
#define NORMAL_DATA_MIN  400*2//基准与传感器垂直距离最小值
#define STR_DATA_MAX_Vertical NORMAL_DATA_MAX //初始数据过滤条件(纵向距离阈值)
#define LESS_DATA_DES 100//打在基准上相邻两光束垂直距离
#define LESS_DATA_TIME 5//小于常规数值阈值的次数
#define Scan_In_Angle 40.0 //扫描起始角度
#define Scan_Fi_Angle 140.0 //扫描终止角度
#define Scan_Angle_Resolution 0.5//扫描角分辨率
#define Size_str_temp (int)((Scan_Fi_Angle-Scan_In_Angle)/Scan_Angle_Resolution+1.0) //str_temp数组大小
#define Relay_distance 50 //接力条件（与参考点二距离范围阈值）

/*______LCD显示模式控制变量_YZ_2015/11/26_________________________________________________________*/
#define dTime_Value 50 //切换LCD屏幕显示内容是条件按键按下时间阈值(当按下事件大于此阈值时，显示图像进行切换)

/* 检测到基准的标志------------------------------------------------------------*/
#define JUDGE_DATA_THRESHOLD 100//判断基准与背景间距，找到突变点
#define JUDGE_ANGLE_THRESHOLD1 0//判断物体进入的角度
#define JUDGE_ANGLE_THRESHOLD2 30//判断物体进入的角度
#define POS_DATA_THRESHOLD 100//判断车头运动位置的阈值
#define R_length  100.0  //基准半径10.0cm 板子作基准时半径为0
//#define R_length  50.0  //基准直径20cm 对应半径10.0cm (0.50角分辨率下传感器返回数据距离缩短为1/2 故这里改为50)

//2015/09/11
#define SENSOR_DISTANCE_ERR 300//原始计算数据偏离判断标准参量

/* 定位和测速标志-----------------------------------------------------------*/
//#define SENOR_DATA_SIZE 100//	传感器发生变化的数组大小
#define TRUE 0x01//	传感器发生变化的数组大小
#define FALSE 0x00//传感器发生变化的数组大小

/* 堆栈储存用-----------------------------------------------------------*/
#define Stack_Size 10 //堆栈的大小(也包含标定时储存标定数据数组的大小)


/**************************整合程序时新增加的变量声明_2015/08/24******************************/ 
extern  u8_t m_bDistanceSampleOk  ;
extern double SENSOR_DISTANCE_FRONT;//车头距基准线距离
extern double SENSOR_DISTANCE_BACK;//车尾距基准线距离
extern double SENSOR_DISTANCE_TEMP[2]; //储存原始计算数据(1为车头距离基准线距离 2为车位距离基准线距离)在进行数据和发行确认后再将计算数据存入SENSOR_DISTANCE_FRONT/SENSOR_DISTANCE_BACK中
extern u8_t SENSOR_DISTANCE_ERR_CNT;//用于记录原始数据偏离(偏离标准参量为SENSOR_DISTANCE_ERR)之前一次存储的合法数据的次数
//2015/08/25
extern u8_t m_bBackMidWheel;
extern u8_t m_bWheelDataOk;
extern u8_t m_bStopFlag;	
extern u8_t m_bVehicleForward;		    //行车前进方向标记，TRUE正在前进，FALSE没有前进
extern u8_t m_bVehicleBackward;			//行车后退方向标记，TRUE正在后退，FALSE没有后退
extern u8_t  BD_ENABLE ; //标定允许标致位
extern u8_t IntersectLocate;  // 交叉记号变量，FALSE对应初始角度为35.0°数据，TRUE对应初始角度为35.1667°数据(原始数据角分辨率为0.3°)
extern u8_t m_bStateBefore; //前一时刻运动方向状态标志位 用来存储前一时刻的运动是正向(0x01)还是反向(0x00)的信息
extern u8_t IntersectOK;         // 判断一组数据(14A、14B)是否已经交叉储存到str_temp3数组当中
extern u8_t IntersectCount;     // 储存待交叉赋值数据的计数变量 IntersectCount在程序储存str_temp1后+1 在储存str_temp2后再+1 当IntersectCount == 2时开始交叉赋值
extern u16_t str_temp[Size_str_temp]; //将 str_temp1 与 str_temp2 交叉储存到数组str_temp3当中，进行之后的数据处理
extern u16_t str_temp_inverse[Size_str_temp]; //储存颠倒后数据
extern u16_t Time_count;   //为了验证程序是否在执行Data_collection.c中卡死所设定的计数变量(从1++到10000后归零重复++)
extern u8_t LCD_Interface1; //LCD是否显示数据界面1参量
extern u8_t LCD_Interface2; //LCD是否显示数据界面2参量
extern u8_t jc_Record; //记录接力时刻该包数据的参考点数目,用于解决一次性扫描到4个参考点的情况_YZ_2015/12/10
/*______LCD显示模式控制变量_YZ_2015/11/26_________________________________________________________*/
extern u8_t Key_Button_Push; //KEY按键是否按下标记参量
extern u8_t LCD_Change_Mode; //LCD显示模式切换是否发生变量
extern u16_t LCD_Change_Mode_InTime; //LCD显示模式切换时间条件记录参量(Key按下初始时间)
extern u16_t LCD_Change_Mode_FiTime; //LCD显示模式切换时间条件记录参量(Key松开终止时间)
extern u16_t LCD_Change_Mode_dTime; //LCD显示模式切换时间条件记录参量(Key按下至松开间隔时间)
extern u8_t LCD_Vehicle_Trail; //LCD是否显示车辆轨迹图参量
extern u8_t LCD_SENSOR_Image;  //LCD是否显示传感器扫描图像参量
extern u8_t LCD_Vehicle_Posture; //LCD是否显示车身姿态图像参量
extern double SCANNER_CENTER ;
extern double LEFT_MIN_VALUE_BACK  ;	
extern double RIGHT_MAX_VALUE_BACK ;	
extern double LEFT_MIN_VALUE_FRONT  ;	
extern double RIGHT_MAX_VALUE_FRONT ;	

/*______以下针对使用堆栈存储建立的参量_YZ_2015/12/13_________________________________________________________*/
extern double Vehicle_X_Stack[Stack_Size]; //储存车辆位置横坐标堆栈
extern double Vehicle_Y_Stack[Stack_Size]; //储存车辆位置纵坐标堆栈
extern double Vehicle_sincx_Stack[Stack_Size]; //储存车辆偏转角sin值堆栈
extern double Vehicle_coscx_Stack[Stack_Size]; //储存车辆偏转角cos值堆栈
extern double Vehicle_Output_Sum_X; //堆栈输出车辆横坐标值
extern double Vehicle_Output_Sum_Y; //堆栈输出车辆纵坐标值
extern double Vehicle_Output_Sum_sincx; //堆栈输出车辆偏转角sin值
extern double Vehicle_Output_Sum_coscx; //堆栈输出车辆偏转角cos值
extern double Standard_Position_x1_Stack; //标定时，由堆栈计算得出标准位置横坐标
extern double Standard_Position_y1_Stack; //标定时，由堆栈计算得出标准位置纵坐标
extern double Standard_Position_x2_Stack; //标定时，由堆栈计算得出第二个参考点横坐标
extern double Standard_Position_y2_Stack; //标定时，由堆栈计算得出第二个参考点纵坐标
extern double Standard_Position_x1_Stack_Sum[Stack_Size]; //储存车辆标定时刻标准位置横坐标
extern double Standard_Position_y1_Stack_Sum[Stack_Size]; //储存车辆标定时刻标准位置纵坐标
extern double Standard_Position_x2_Stack_Sum[Stack_Size]; //储存车辆标定时刻第二个参考点横坐标
extern double Standard_Position_y2_Stack_Sum[Stack_Size]; //储存车辆标定时刻第二个参考点纵坐标
extern u8_t Stack_Sum_Num; //堆栈求和是计数参量
extern u8_t Stack_Replace_Num; //堆栈刷新数据(即一次向前替换)计数参量
extern u8_t Vehicle_Stack_Num; //堆栈储存计数参量
extern u8_t Vehicle_Stack_Full;//堆栈存储是否已满参量
extern u8_t BD_Stack_Num;      //标定位置堆栈储存计数参量
extern u8_t BD_Stack_Sum_Num;  //标定位置堆栈求和计数参量
extern u8_t BD_Stack_Replace_Num; //标定位置堆栈刷新数据计数参量
extern u16_t A_count;  //记录采集到数据包的个数								
							
#ifdef __cplusplus
}
#endif




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

