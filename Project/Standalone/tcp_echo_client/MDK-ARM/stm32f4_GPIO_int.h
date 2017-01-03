#include "stm32f4xx.h"
#include "stm32_eval_legacy.h"

//对管脚进行定义 PORT为系列 PIN为系列下的具体管脚序号

#define IN_FORWARD_PIN                   GPIO_Pin_11
#define IN_FORWARD_PORT                   GPIOI
  
#define IN_BACKWARD_PIN                  GPIO_Pin_12
#define IN_BACKWARD_PORT                  GPIOI
  
#define IN_MID_SWITCH_PIN                GPIO_Pin_13
#define IN_MID_SWITCH_PORT                GPIOI
  
#define OUT_PUSH_PIN                     GPIO_Pin_0
#define OUT_PUSH_PORT                     GPIOI

#define OUT_DRAW_PIN                     GPIO_Pin_1
#define OUT_DRAW_PORT                     GPIOI

#define IN_MODE_PIN                     GPIO_Pin_9
#define IN_MODE_PORT                     GPIOI

#define IN_SPEED_PIN                     GPIO_Pin_14
#define IN_SPEED_PORT                     GPIOI

#define IN_RUDDER_PIN                     GPIO_Pin_10
#define IN_RUDDER_PORT                     GPIOI

#define IN_LOCK_WHEEL_PIN                 GPIO_Pin_8
#define IN_LOCK_WHEEL_PORT                 GPIOI

#define IN_AD_PIN                         GPIO_Pin_15
#define IN_AD_PORT                         GPIOI

#define OUT_STOP_FLAG_PIN                 GPIO_Pin_4
#define OUT_STOP_FLAG_PORT                 GPIOI
 
