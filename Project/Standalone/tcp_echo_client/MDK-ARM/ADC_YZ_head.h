/*----------------------------------------------------------------------------
 * Name:    ADC.h
 * Purpose: low level ADC definitions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2004-2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "stm32f4xx.h"
#include "stm32f4x7_eth_bsp.h"
#include "lwip/tcp.h"


#ifndef __ADC_H
#define __ADC_H

#define ADC_VALUE_MAX      (0xFFF)

extern void     ADC_Init_YZ    (void);
extern void     ADC_StartCnv(void);
extern u32_t ADC_DoneCnv (void);
extern u16_t ADC_GetCnv  (void);

#endif
