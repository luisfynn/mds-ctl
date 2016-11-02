/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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

/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "shell.h"
#include "adc.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osMessageQId uartRxQueueHandle;
osMutexId i2cMutexHandle;

/* USER CODE BEGIN Variables */
uint8_t usartRxBuff;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void alarmOutTask(void const * argument);
void stm32LiveTask(void const * argument);
void stm32Usart1Task(void const * argument);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of i2cMutex */
  osMutexDef(i2cMutex);
  i2cMutexHandle = osMutexCreate(osMutex(i2cMutex));

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE - 96 );
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(stm32AlarmOut, alarmOutTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE - 96 );
  defaultTaskHandle = osThreadCreate(osThread(stm32AlarmOut), NULL);

  osThreadDef(stm32LiveCheck, stm32LiveTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE -32 );
  defaultTaskHandle = osThreadCreate(osThread(stm32LiveCheck), NULL);

  osThreadDef(stm32Usart, stm32Usart1Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 128 );
  defaultTaskHandle = osThreadCreate(osThread(stm32Usart), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of uartRxQueue */
  osMessageQDef(uartRxQueue, CONFIG_SYS_CBSIZE, uint8_t);
  uartRxQueueHandle = osMessageCreate(osMessageQ(uartRxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
#ifdef	DEBUG_PARSER
  	  printf("FreeRTOS init complete\n");
#endif
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
__weak void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
/* stm32LiveTask function */
void stm32LiveTask(void const * argument)
{
	uint8_t count = 0;

	for(;;)
	{
		if(count % 2)	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		else			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

		if(count < 100) 	count++;
		else				count = 0;

		adcTest();

		osDelay(1000);
	}
}

/* alarmOutTask function */
void alarmOutTask(void const * argument)
{
	for(;;)
	{
		osDelay(1000);
	}
}

void stm32Usart1Task(void const * argument)
{
	HAL_UART_Receive_IT(&huart1, &usartRxBuff, USART_RX_BUFF_SIZE);

	for(;;)
	{
		stm32ShellCommand();
		osDelay(10);
	}
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,signed char *pcTaskName )
{
	printf("%s stack overlow \n", (char*)pcTaskName);
}

void vApplicationMallocFailedHook( void )
{
	printf("FreeRTOS memory allocation failed\n");
	printf("Check all of Task Stack Size or data + bss Size\n");
}

     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
