/**
  ******************************************************************************
  * @file    tcp_echoclient.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011 
  * @brief   tcp echoclient application using LwIP RAW API
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

/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "main.h"
#include "data_collection.h"
//#include "memp.h"
#include <stdio.h>
#include <string.h>
#if LWIP_TCP
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define TCP_CLIENT_RX_BUFSIZE	10000
__IO uint32_t message_count=0;
u8_t data_flag;	//包头校验标志位
extern u8_t Data_correct_flag;//包尾校验标志位
extern u8_t Data_RECE[1500];//除去包头的数据存储数组
extern int  DATA_NUM;
extern u16_t IC_flag;//单片机和传感器连接标志
extern u16_t A_count;  //记录采集到14A包头的数据包的个数
extern u16_t B_count;  //记录采集到14B包头的数据包的个数
u8_t tcp_client_flag=0;	 
/* data查询实际输出角度，data1 单一输出数据*/
u8_t   data[20]={0x02,0x73,0x52,0x4E,0x20,0x4c,0x4D,0x50,0x6F,0x75,0x74,0x70,0x75,0x74,0x52,0x61,0x6E,0x67,0x65,0x03};
u8_t   data1[20]={0x02,0x73,0x52,0x4E,0x20,0x4c,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74,0x61,0x03};
u8_t  tcp_client_recvbuf[TCP_CLIENT_RX_BUFSIZE];	
struct tcp_pcb *echoclient_pcb;
/* ECHO protocol states */
enum echoclient_states
{
  ES_NOT_CONNECTED = 0,
  ES_CONNECTED,
  ES_RECEIVED,
  ES_CLOSING,
};

/* structure to be passed as argument to the tcp callbacks */
struct echoclient
{
  enum echoclient_states state; /* connection status */
  struct tcp_pcb *pcb;          /* pointer on the current tcp_pcb */
  struct pbuf *p_tx;            /* pointer on pbuf to be transmitted */
};


/* Private function prototypes -----------------------------------------------*/
static err_t tcp_echoclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_echoclient_connection_close(struct tcp_pcb *tpcb, struct echoclient * es);
static err_t tcp_echoclient_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_echoclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_echoclient_send(struct tcp_pcb *tpcb, struct echoclient * es);
static err_t tcp_echoclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Connects to the TCP echo server
  * @param  None
  * @retval None
  */

void tcp_echoclient_connect(void)
{
  struct ip_addr DestIPaddr;
  /* create new tcp pcb */
  echoclient_pcb = tcp_new();   
//	  STM_EVAL_LEDOn(LED3);
  if (echoclient_pcb != NULL)
  {
     IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 ); 
    /* connect to destination address/port */
    tcp_connect(echoclient_pcb,&DestIPaddr,DEST_PORT,tcp_echoclient_connected);
  }
	
}

/**
  * @brief Function called when TCP connection established
  * @param tpcb: pointer on the connection contol block
  * @param err: when connection correctly established err should be ERR_OK 
  * @retval err_t: returned error 
  */
static err_t tcp_echoclient_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  struct echoclient *es = NULL;
  
  if (err == ERR_OK)   
  {
		 IC_flag=IC_TRUE ;
    /* allocate structure es to maintain tcp connection informations */
    es = (struct echoclient *)mem_malloc(sizeof(struct echoclient));
    if (es != NULL)
    {
      es->state = ES_CONNECTED;
      es->pcb = tpcb;	  
      es->p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char*)data1) , PBUF_POOL);   
      if (es->p_tx)
      {       
        /* copy data to pbuf */
        pbuf_take(es->p_tx, (char*)data1, strlen((char*)data1));
        
        /* pass newly allocated es structure as argument to tpcb */
        tcp_arg(tpcb, es);
        /* initialize LwIP tcp_recv callback function */ 
        tcp_recv(tpcb, tcp_echoclient_recv);
        
        /* initialize LwIP tcp_sent callback function */
        tcp_sent(tpcb, tcp_echoclient_sent);
  
        /* initialize LwIP tcp_poll callback function */
        tcp_poll(tpcb, tcp_echoclient_poll, 1);
        /* send data */
        tcp_echoclient_send(tpcb,es);
				
        return ERR_OK;
      }
    }
    else
    {
      /* close connection */
      tcp_echoclient_connection_close(tpcb, es);
       //tcp_client_flag=0;
      /* return memory allocation error */
      return ERR_MEM;  
    }
  }
  else
  {
    /* close connection */
    tcp_echoclient_connection_close(tpcb, es);
  }
  return err;
}
    
/**
  * @brief tcp_receiv callback
  * @param arg: argument to be passed to receive callback 
  * @param tpcb: tcp connection control block 
  * @param err: receive error code 
  * @retval err_t: retuned error  
  */
static err_t tcp_echoclient_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{ 
	
	int i,n,a;
	int m=0;
  struct echoclient *es;
  err_t ret_err;
  struct pbuf *q;
  u32 data_len=0; 
  static int count;//数组发送	
  LWIP_ASSERT("arg != NULL",arg != NULL);
  es = (struct echoclient *)arg;
  
  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p_tx == NULL)
    {
       /* we're done sending, close connection */
       tcp_echoclient_connection_close(tpcb, es);
    }
    else
    {    
      /* send remaining data*/
      tcp_echoclient_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from echo server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_CONNECTED)
  {
    /* increment message count */
   message_count++;
		if(p!= NULL)
    {   
			 //STM_EVAL_LEDOff(LED1);
       memset(tcp_client_recvbuf,0,TCP_CLIENT_RX_BUFSIZE);  //数据接收缓冲区清零
			for(q =p;q!=NULL;q=q->next)  //遍历完整个pbuf链表
			{
				//判断要拷贝到TCP_CLIENT_RX_BUFSIZE中的数据是否大于TCP_CLIENT_RX_BUFSIZE的剩余空间，如果大于
				//的话就只拷贝TCP_CLIENT_RX_BUFSIZE中剩余长度的数据，否则的话就拷贝所有的数据
				if(q->len > (TCP_CLIENT_RX_BUFSIZE-data_len))
					memcpy(tcp_client_recvbuf+data_len,q->payload,(TCP_CLIENT_RX_BUFSIZE-data_len));//拷贝数据
				else 
					memcpy(tcp_client_recvbuf+data_len,q->payload,q->len);
				data_len += q->len; 
				//USART_SendData(EVAL_COM1,data_len);				
				if(data_len > TCP_CLIENT_RX_BUFSIZE) break; //超出TCP客户端接收数组,跳出	
			} 
		for(i=0;i<data_len;i++) //寻找报文头
	  	{   
					//LMS151 包头
					//if((tcp_client_recvbuf[i]==0x38)&&(tcp_client_recvbuf[i+1]==0x38)&&(tcp_client_recvbuf[i+2]==0x20)&&(tcp_client_recvbuf[i+3]==0x42)&&(tcp_client_recvbuf[i+4]==0x35))
					//if((tcp_client_recvbuf[i]==0x38)&&(tcp_client_recvbuf[i+1]==0x20)&&(tcp_client_recvbuf[i+2]==0x31)&&(tcp_client_recvbuf[i+3]==0x31)&&(tcp_client_recvbuf[i+4]==0x39))  //LMS511包头 50HZ 0.50 20~160
					//  if((tcp_client_recvbuf[i]==0x33)&&(tcp_client_recvbuf[i+1]==0x20)&&(tcp_client_recvbuf[i+2]==0x33)&&(tcp_client_recvbuf[i+3]==0x34)&&(tcp_client_recvbuf[i+4]==0x39))  //LMS511包头 25HZ 0.1667 20~160	数据包出界了
				   //  if((tcp_client_recvbuf[i]==0x30)&&(tcp_client_recvbuf[i+1]==0x35)&&(tcp_client_recvbuf[i+2]==0x20)&&(tcp_client_recvbuf[i+3]==0x31)&&(tcp_client_recvbuf[i+4]==0x34))  //LMS511包头 50HZ 0.1667间隔采集 35~145		
				 //    if((tcp_client_recvbuf[i]==0x30)&&(tcp_client_recvbuf[i+1]==0x35)&&(tcp_client_recvbuf[i+2]==0x20)&&(tcp_client_recvbuf[i+3]==0x31)&&(tcp_client_recvbuf[i+4]==0x34))  //LMS511包头 50HZ 0.33333 非间隔采集 36~144		
				//		 if((tcp_client_recvbuf[i]==0x30)&&(tcp_client_recvbuf[i+1]==0x35)&&(tcp_client_recvbuf[i+2]==0x20)&&(tcp_client_recvbuf[i+3]==0x31)&&(tcp_client_recvbuf[i+4]==0x30))  //LMS511包头 50HZ 0.1667 间隔采集 40~130	
					if((tcp_client_recvbuf[i]==0x33)&&(tcp_client_recvbuf[i+1]==0x38)&&(tcp_client_recvbuf[i+2]==0x38)&&(tcp_client_recvbuf[i+3]==0x20)&&(tcp_client_recvbuf[i+4]==0x43))  //LMS511包头 50HZ 0.33333 非间隔采集 40~140		
 
//										if((tcp_client_recvbuf[i]==0x20))  //LMS511包头 50HZ 0.33333 非间隔采集 40~140		

				    { 
//								if(tcp_client_recvbuf[i+5]==0x39)       //包头为，对应初始角度为40.0°
//								{
//									IntersectLocate=FALSE;
//									IntersectCount=0x01;
//									A_count=A_count+1;
//								}
//								else if(tcp_client_recvbuf[i+5]==0x39)  // 包头为14A，对应初始角度为35.16667°
//								{
//									IntersectLocate=TRUE;
//									IntersectCount=0x02;
//									B_count=B_count+1;
//								}
								m=i+7;
								data_flag=1;
//							  A_count=A_count+1;
//							if(A_count >= 50000)
//							{
//								A_count=0;
//							}
//							  STM_EVAL_LEDOn(LED1);
								break;
							}
						else data_flag=0;
	    }		
	if (data_flag==1)
	 {       
		      DATA_NUM=0;
		      count++;
		for(n=m;n<data_len;n++)
		 {      
				   Data_RECE[DATA_NUM]=tcp_client_recvbuf[n]; //将含报尾的数据存入Data_RECE[j]中
		       DATA_NUM++; 
	   }	
       data_flag=0;		 
   }
	for (a=data_len;a>10;a--)
	{  
		
	  //LMS151 包尾
		if((Data_RECE[a-4]==0x20)&&(Data_RECE[a-3]==0x30)&&(Data_RECE[a-2]==0x20)&&(Data_RECE[a-1]==0x30)&&(Data_RECE[a]==0x03))  
	 { 
	//	 STM_EVAL_LEDOn(LED2);
		 Data_correct_flag=1;
		 break;
	 }
	}
	}
	/* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);  
    pbuf_free(p);
    tcp_echoclient_connection_close(tpcb, es);
    ret_err = ERR_OK;
  }
  /* data received when connection already closed */
  else
  {
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);
    
    /* free pbuf and do nothing */
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief function used to send data
  * @param  tpcb: tcp control block
  * @param  es: pointer on structure of type echoclient containing info on data 
  *             to be sent
  * @retval None 
  */
static void tcp_echoclient_send(struct tcp_pcb *tpcb, struct echoclient * es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
  while ((wr_err == ERR_OK) &&
         (es->p_tx != NULL) && 
         (es->p_tx->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p_tx;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
   
		 
    if (wr_err == ERR_OK)
    { 
      /* continue with next pbuf in chain (if any) */
      es->p_tx = ptr->next;
      
      if(es->p_tx != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p_tx);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      pbuf_free(ptr);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later, defer to poll */
     es->p_tx = ptr;
   }
	 
   else
   {
		 tcp_output(tpcb);		//将发送缓冲队列中的数据立即发送出去
     /* other problem ?? */
   }
  }
}

/**
  * @brief  This function implements the tcp_poll callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: tcp connection control block
  * @retval err_t: error code
  */
static err_t tcp_echoclient_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct echoclient *es;
  es = (struct echoclient*)arg;
  if (es != NULL)
  {
    if (es->p_tx != NULL)
    {
      /* there is a remaining pbuf (chain) , try to send data */
      tcp_echoclient_send(tpcb, es);	
		if(es->p_tx) pbuf_free(es->p_tx);	//释放内存
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /* close tcp connection */
        tcp_echoclient_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  arg: pointer on argument passed to callback
  * @param  tcp_pcb: tcp connection control block
  * @param  len: length of data sent 
  * @retval err_t: returned error code
  */
static err_t tcp_echoclient_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct echoclient *es;

  LWIP_UNUSED_ARG(len);

  es = (struct echoclient *)arg;
  
  if(es->p_tx != NULL)
  {
    /* still got pbufs to send */
    tcp_echoclient_send(tpcb, es);
  }

  return ERR_OK;
}

/**
  * @brief This function is used to close the tcp connection with server
  * @param tpcb: tcp connection control block
  * @param es: pointer on echoclient structure
  * @retval None
  */
static void tcp_echoclient_connection_close(struct tcp_pcb *tpcb, struct echoclient * es )
{
  /* remove callbacks */
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_poll(tpcb, NULL,0);

  if (es != NULL)
  {
    mem_free(es);
  }
  /* close tcp connection */
  tcp_close(tpcb);
  
}

#endif /* LWIP_TCP */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
