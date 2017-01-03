/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 **/
#ifndef __LWIP_SYS_H__
#define __LWIP_SYS_H__

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C" {
#endif

#if NO_SYS

/* For a totally minimal and standalone system, we provide null
   definitions of the sys_ functions. */
typedef u8_t sys_sem_t;
typedef u8_t sys_mbox_t;
struct sys_timeo {u8_t dummy;};

#define sys_init()
#define sys_timeout(m,h,a)
#define sys_untimeout(m,a)
#define sys_sem_new(c) c
#define sys_sem_signal(s)
#define sys_sem_wait(s)
#define sys_sem_wait_timeout(s,t)
#define sys_arch_sem_wait(s,t)
#define sys_sem_free(s)
#define sys_mbox_new(s) 0
#define sys_mbox_fetch(m,d)
#define sys_mbox_tryfetch(m,d)
#define sys_mbox_post(m,d)
#define sys_mbox_trypost(m,d)
#define sys_mbox_free(m)

#define sys_thread_new(n,t,a,s,p)

#define IO_DOG_PORT 0x08
//
#define MIDDLE_POSITION 0			/* 推杆到中点的测量值 */
#define LOCK_WHEEL_VALUE 1			/* 方向盘被锁定时测量值是1 */
//
#define RELOADVALH 0xdc
#define RELOADVALL 0x00				/*11.0592----10ms*/
//
#define ADR_377 0x0000		/* Adress of 74LS377 */
#define ADR_1225 0x2000		/* Adress of DS1225 */
//
#define HIGH_SPEED_VAL	0	/* 高速信号 */
//--------------
#define CAR_MOVE_FORWORD 0	/* 扫描车前进信号 */
#define CAR_MOVE_BACKWORD 0	/* 扫描车后退信号 */
//--------------------
#define TRANSFER_MODE		0 /*环保车模式*/
#define RIGHT_MODE	    	1 /*右舵模式*/
//-------------------------------------


//系统错误信息
#define SYSERR_LEFT_EXTREME 0x01		/*车体左平移超限*/	
#define SYSERR_RIGHT_EXTREME 0x02		/* 车体右平移超限 */
#define SYSERR_5S_NODATA 0x04			/* 连续5秒没有采集到正确数据 */
#define SYSERR_WHEEL_LCOK 0x08			/* H车方向盘锁定 */
#define SYSERR_MID_WHEEL 0x10			/* 推杆回中错误 */
#define SYSERR_PUSH_EXTREME 0x20		/*推杆推超限*/
#define SYSERR_DRAW_EXTREME 0x40		/*推杆拉超限*/
#define SYSERR_SUM 0x07					/* 系统故障数目 */

/*_____ D E F I N I T I O N ________________________________________________*/
#define P3 0x01 //管脚值待修改

extern s8_t IN_FORWARD;				//p1.0－前进信号(Input)
extern s8_t IN_BACKWARD;				//p1.1－后退信号(Input)
extern s8_t IN_MID_SWITCH;			//p1.2－中间点信号(Input)
extern s8_t OUT_PUSH;					//P3.0－前推推杆(output)，＝1,执行
extern s8_t OUT_DRAW;					//p3.1－后拉推杆(output)，＝1,执行
//
extern s8_t IN_MODE;				  //P1.6-工作模式信号输入，0为环保车模式，1为非环保车模式
//
extern s8_t IN_SPEED;					//p1.4－高速信号检测
 
extern s8_t IN_RUDDER;			    //P1.5 - 左舵右舵模式
 //
extern s8_t IN_LOCK_WHEEL;			//p3.5－方向盘锁定检测
 //
extern s8_t IN_AD;					//p1.7- AD输入信号
extern s8_t DIN;						//显示管脚
extern s8_t CLK;
extern s8_t LOAD;


#else /* NO_SYS */

/** Return code for timeouts from sys_arch_mbox_fetch and sys_arch_sem_wait */
#define SYS_ARCH_TIMEOUT 0xffffffffUL

/* sys_mbox_tryfetch returns SYS_MBOX_EMPTY if appropriate.
 * For now we use the same magic value, but we allow this to change in future.
 */
#define SYS_MBOX_EMPTY SYS_ARCH_TIMEOUT 

#include "lwip/err.h"
#include "arch/sys_arch.h"

typedef void (* sys_timeout_handler)(void *arg);

struct sys_timeo {
  struct sys_timeo *next;
  u32_t time;
  sys_timeout_handler h;
  void *arg;
};

struct sys_timeouts {
  struct sys_timeo *next;
};

/* sys_init() must be called before anthing else. */
void sys_init(void);

/*
 * sys_timeout():
 *
 * Schedule a timeout a specified amount of milliseconds in the
 * future. When the timeout occurs, the specified timeout handler will
 * be called. The handler will be passed the "arg" argument when
 * called.
 *
 */
void sys_timeout(u32_t msecs, sys_timeout_handler h, void *arg);
void sys_untimeout(sys_timeout_handler h, void *arg);
struct sys_timeouts *sys_arch_timeouts(void);

/* Semaphore functions. */
sys_sem_t sys_sem_new(u8_t count);
void sys_sem_signal(sys_sem_t sem);
u32_t sys_arch_sem_wait(sys_sem_t sem, u32_t timeout);
void sys_sem_free(sys_sem_t sem);
void sys_sem_wait(sys_sem_t sem);
int sys_sem_wait_timeout(sys_sem_t sem, u32_t timeout);

/* Time functions. */
#ifndef sys_msleep
void sys_msleep(u32_t ms); /* only has a (close to) 1 jiffy resolution. */
#endif
#ifndef sys_jiffies
u32_t sys_jiffies(void); /* since power up. */
#endif

/* Mailbox functions. */
sys_mbox_t sys_mbox_new(int size);
void sys_mbox_post(sys_mbox_t mbox, void *msg);
err_t sys_mbox_trypost(sys_mbox_t mbox, void *msg);
u32_t sys_arch_mbox_fetch(sys_mbox_t mbox, void **msg, u32_t timeout);
#ifndef sys_arch_mbox_tryfetch /* Allow port to override with a macro */
u32_t sys_arch_mbox_tryfetch(sys_mbox_t mbox, void **msg);
#endif
/* For now, we map straight to sys_arch implementation. */
#define sys_mbox_tryfetch(mbox, msg) sys_arch_mbox_tryfetch(mbox, msg)
void sys_mbox_free(sys_mbox_t mbox);
void sys_mbox_fetch(sys_mbox_t mbox, void **msg);

/* Thread functions. */
sys_thread_t sys_thread_new(char *name, void (* thread)(void *arg), void *arg, int stacksize, int prio);

#endif /* NO_SYS */

/** Returns the current time in milliseconds. */
u32_t sys_now(void);

/* Critical Region Protection */
/* These functions must be implemented in the sys_arch.c file.
   In some implementations they can provide a more light-weight protection
   mechanism than using semaphores. Otherwise semaphores can be used for
   implementation */
#ifndef SYS_ARCH_PROTECT
/** SYS_LIGHTWEIGHT_PROT
 * define SYS_LIGHTWEIGHT_PROT in lwipopts.h if you want inter-task protection
 * for certain critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#if SYS_LIGHTWEIGHT_PROT

/** SYS_ARCH_DECL_PROTECT
 * declare a protection variable. This macro will default to defining a variable of
 * type sys_prot_t. If a particular port needs a different implementation, then
 * this macro may be defined in sys_arch.h.
 */
#define SYS_ARCH_DECL_PROTECT(lev) sys_prot_t lev
/** SYS_ARCH_PROTECT
 * Perform a "fast" protect. This could be implemented by
 * disabling interrupts for an embedded system or by using a semaphore or
 * mutex. The implementation should allow calling SYS_ARCH_PROTECT when
 * already protected. The old protection level is returned in the variable
 * "lev". This macro will default to calling the sys_arch_protect() function
 * which should be implemented in sys_arch.c. If a particular port needs a
 * different implementation, then this macro may be defined in sys_arch.h
 */
#define SYS_ARCH_PROTECT(lev) lev = sys_arch_protect()
/** SYS_ARCH_UNPROTECT
 * Perform a "fast" set of the protection level to "lev". This could be
 * implemented by setting the interrupt level to "lev" within the MACRO or by
 * using a semaphore or mutex.  This macro will default to calling the
 * sys_arch_unprotect() function which should be implemented in
 * sys_arch.c. If a particular port needs a different implementation, then
 * this macro may be defined in sys_arch.h
 */
#define SYS_ARCH_UNPROTECT(lev) sys_arch_unprotect(lev)
sys_prot_t sys_arch_protect(void);
void sys_arch_unprotect(sys_prot_t pval);

#else

#define SYS_ARCH_DECL_PROTECT(lev)
#define SYS_ARCH_PROTECT(lev)
#define SYS_ARCH_UNPROTECT(lev)

#endif /* SYS_LIGHTWEIGHT_PROT */

#endif /* SYS_ARCH_PROTECT */

/*
 * Macros to set/get and increase/decrease variables in a thread-safe way.
 * Use these for accessing variable that are used from more than one thread.
 */

#ifndef SYS_ARCH_INC
#define SYS_ARCH_INC(var, val) do { \
                                SYS_ARCH_DECL_PROTECT(old_level); \
                                SYS_ARCH_PROTECT(old_level); \
                                var += val; \
                                SYS_ARCH_UNPROTECT(old_level); \
                              } while(0)
#endif /* SYS_ARCH_INC */

#ifndef SYS_ARCH_DEC
#define SYS_ARCH_DEC(var, val) do { \
                                SYS_ARCH_DECL_PROTECT(old_level); \
                                SYS_ARCH_PROTECT(old_level); \
                                var -= val; \
                                SYS_ARCH_UNPROTECT(old_level); \
                              } while(0)
#endif /* SYS_ARCH_DEC */

#ifndef SYS_ARCH_GET
#define SYS_ARCH_GET(var, ret) do { \
                                SYS_ARCH_DECL_PROTECT(old_level); \
                                SYS_ARCH_PROTECT(old_level); \
                                ret = var; \
                                SYS_ARCH_UNPROTECT(old_level); \
                              } while(0)
#endif /* SYS_ARCH_GET */

#ifndef SYS_ARCH_SET
#define SYS_ARCH_SET(var, val) do { \
                                SYS_ARCH_DECL_PROTECT(old_level); \
                                SYS_ARCH_PROTECT(old_level); \
                                var = val; \
                                SYS_ARCH_UNPROTECT(old_level); \
                              } while(0)
#endif /* SYS_ARCH_SET */


#ifdef __cplusplus
}
#endif

#endif /* __LWIP_SYS_H__ */
