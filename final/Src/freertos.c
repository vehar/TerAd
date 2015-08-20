/**
  ******************************************************************************
  * File Name          : freertos.c
  * Date               : 01/02/2015 19:28:06
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"


/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId U1RXHandle;
osThreadId U2RXHandle;
osThreadId USBTXHandle;
osThreadId U3TXHandle;
osMessageQId RecQHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void Usart1Rx(void const * argument);
void Usart2Rx(void const * argument);
void UsbSend(void const * argument);
void Usart3Tx(void const * argument);

/* USER CODE BEGIN FunctionPrototypes */

uint8_t b1;
uint8_t b2;

typedef struct {
	uint8_t usart;
	uint8_t byte;
} myMes;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{ 
	myMes m;
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
	switch((uint32_t)UartHandle->Instance)
	{
		case (uint32_t)USART1:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
		  m.usart=1;
		  m.byte=b1;
		  xQueueSend( RecQHandle, &m, portMAX_DELAY  );
		  // Do this need? 
      //HAL_UART_Receive_IT(&huart1, &b1,1);
		  break;
		case (uint32_t)USART2:
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		  m.usart=2;
		  m.byte=b2;
		  xQueueSend( RecQHandle, &m, portMAX_DELAY  );  
		  HAL_UART_Receive_IT(&huart2, &b2,1); 
			break;			
	}
}

/* USER CODE END FunctionPrototypes */
/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init() {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of U1RX */
  osThreadDef(U1RX, Usart1Rx, osPriorityBelowNormal, 0, 256);
  U1RXHandle = osThreadCreate(osThread(U1RX), NULL);

  /* definition and creation of U2RX */
  osThreadDef(U2RX, Usart2Rx, osPriorityBelowNormal, 0, 256);
  U2RXHandle = osThreadCreate(osThread(U2RX), NULL);

  /* definition and creation of USBTX */
  osThreadDef(USBTX, UsbSend, osPriorityBelowNormal, 0, 256);
  USBTXHandle = osThreadCreate(osThread(USBTX), NULL);

  /* definition and creation of U3TX */
  osThreadDef(U3TX, Usart3Tx, osPriorityAboveNormal, 0, 128);
  U3TXHandle = osThreadCreate(osThread(U3TX), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of RecQ */
  osMessageQDef(RecQ, 32, 2);
  RecQHandle = osMessageCreate(osMessageQ(RecQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* Usart1Rx function */
void Usart1Rx(void const * argument)
{
  /* USER CODE BEGIN Usart1Rx */
  /* Infinite loop */
  for(;;)
  {
		if(HAL_UART_Receive_IT(&huart1, &b1,1)==HAL_OK)
		{
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
		}
	osDelay(1); // which one is better? check youself!
  //taskYIELD(); 
	
  }
  /* USER CODE END Usart1Rx */
}

/* Usart2Rx function */
void Usart2Rx(void const * argument)
{
  /* USER CODE BEGIN Usart2Rx */
  /* Infinite loop */
	
  for(;;)
  {
		if(HAL_UART_Receive_IT(&huart2, &b2,1)==HAL_OK)
		{
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
		}
	 osDelay(1); // the same question
	//taskYIELD();	
	//or may be set wait for semafore here? try it!
		
  }
	
  /* USER CODE END Usart2Rx */
}

/* UsbSend function */

void UsbSend(void const * argument)
{
  /* USER CODE BEGIN UsbSend */
  /* Infinite loop */
	myMes w;
	uint8_t scr[82]; // screen buffer
	uint8_t buf[18];
	uint8_t t1[18]; // temp buffer
	uint8_t t2[18*3];
	uint8_t oldusart=0;
	uint8_t char_count=0;  
	uint8_t newline=0;  
	
  for(;;)
  {
		xQueueReceive( RecQHandle, &w, portMAX_DELAY );
		
		if(oldusart!=w.usart || char_count>16)
			{
				oldusart=w.usart;
				newline=1;
				char_count=0;
			}
		buf[char_count++]=w.byte;
			
		if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED)
		{
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
			t1[0]=0;
			t2[0]=0;
			scr[0]=0;
			
			if(newline==1)
			{
				newline=0;
				sprintf((char *) &scr[0],"\n\rU%d ",w.usart);
			}
			else
			{
				sprintf((char *) &scr[0],"\rU%d ",w.usart);
			}
					
			// now print 16 bytes 
			
			for(int i=0;i<char_count;i++)
				{
					if(buf[i]>0x1f && buf[i]<0x7f)
					{
						sprintf((char *) &t2[0],"%c",buf[i]);
					}
					else
					{
						sprintf((char *) &t2[0],".");
					}
					strcat((char *)&t1[0],(char *)&t2[0]);					
				}
			//padding	
			for(int i=char_count-1;i<16;i++)
				{
					sprintf((char *) &t2[0]," ");
					strcat((char *)&t1[0],(char *)&t2[0]);					
				}
			strcat((char *) &scr[0],(char *)&t1[0]);
				
			// and hex
			t2[0]=0;
			for(int i=0;i<char_count;i++)
				{
					sprintf((char *) &t1[0]," %02X",buf[i]);
					strcat((char *)&t2[0],(char *)&t1[0]);					
				}	
			strcat((char *) &scr[0],(char *)&t2[0]);
// Do you want some debug output?			
//			sprintf((char *) &t1[0]," %d",char_count);
//			strcat((char *) &scr[0],(char *)&t1[0]);	
			CDC_Transmit_FS(&scr[0],strlen((char *)&scr[0]));	
			osDelay(5);
		}
  }
  /* USER CODE END UsbSend */
}

/* Usart3Tx function */
void Usart3Tx(void const * argument)
{
  /* USER CODE BEGIN Usart3Tx */
  /* Infinite loop */
  for(;;)
  {
		// Wall-e: Eeeee... va? 
		uint8_t walle[]="Eeeee... va?\r\n";
		// Short Circuit: Johny Five is Alive!
		uint8_t johny[]="Johny Five is Alive! \r\n";
		// StarWars 
		uint8_t c3po[]="Sir, the possibility of successfully navigating an asteroid field is approximately 3,720 to 1 \r\n";
		HAL_UART_Transmit(&huart3,(uint8_t *)&walle,15,16); //PB10
		HAL_UART_Transmit(&huart2,(uint8_t *)&johny,23,100); //PA2
		HAL_UART_Transmit(&huart1,(uint8_t *)&c3po,97,100);  //PC4
    osDelay(1000);
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
  }
  /* USER CODE END Usart3Tx */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
