/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t ecd;           
    int16_t speed_rpm;      
    int16_t given_current;  
    uint8_t temperature;    
    uint16_t last_ecd; 
} motor_measure_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 #define get_motor_measure(ptr, data) \
{ \
    (ptr)->last_ecd = (ptr)->ecd; \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]); \
    (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperature = (data)[6]; \
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ================= JustFloat 发送相关定义 ================= */
#define VOFA_CH_COUNT 4      // 发送 4 个通道（4个电机转速）
uint8_t vofa_tx_buf[(VOFA_CH_COUNT * 4) + 4]; // 4个float(16字节) + 4字节帧尾 = 20字节
volatile uint8_t vofa_dma_busy = 0;           // DMA 忙标志位 (0:空闲, 1:发送中)

// 帧尾定义：JustFloat 协议固定以 00 00 80 7F 结尾 (小端模式的 float 无穷大)
const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7F};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

motor_measure_t motor[4];
//volatile uint32_t can_rx_test_count = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 
 CAN_RxHeaderTypeDef rx_header;
 uint8_t rx_data[8];
 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
 uint8_t motor_idx = rx_header.StdId - 0x201;
 if (motor_idx < 4)
 {
   //can_rx_test_count++;
   get_motor_measure(&motor[motor_idx], rx_data);
 }
  
}

void can_filter_init(void)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank =14;
  can_filter_st.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  can_filter_st.SlaveStartFilterBank = 0;
}




/* ================= JustFloat 发送函数 ================= */
void JustFloat_SendDMA(void)
{
    // 1. 如果 DMA 正在发送，直接退出，防止数据覆盖
    if (vofa_dma_busy) return;

    // 2. 准备数据：将4个电机的转速转成 float，并打包进缓冲区
    float speeds[4];
    speeds[0] = (float)motor[0].speed_rpm;
    speeds[1] = (float)motor[1].speed_rpm;
    speeds[2] = (float)motor[2].speed_rpm;
    speeds[3] = (float)motor[3].speed_rpm;

    // 把 float 数组拷贝到发送缓冲区
    memcpy(vofa_tx_buf, speeds, sizeof(speeds));
    // 把 4 字节帧尾拷贝到缓冲区末尾
    memcpy(vofa_tx_buf + sizeof(speeds), justfloat_tail, 4);

    // 3. 上锁：标记 DMA 忙碌
    vofa_dma_busy = 1;

    // 4. 启动串口 7 的 DMA 发送 (注意：这里传的是长度 20)
    HAL_UART_Transmit_DMA(&huart7, vofa_tx_buf, sizeof(vofa_tx_buf));
}

/* ================= DMA 发送完成回调 ================= */
// 当 DMA 把 20 个字节全部搬运到串口硬件后，会自动调用这个函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart7) // 判断是否是串口 7 发送完成
    {
        vofa_dma_busy = 0; // 解锁：允许下一帧发送
    }
}

/* ================= 串口/DMA 错误回调 ================= */
// 如果发生串口错误或 DMA 传输出错，强制解锁，防止程序死锁卡在 busy=1
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart==&huart7)
    {
        vofa_dma_busy = 0; 
        // 必要时可以在这里重新启动 DMA 或清理错误标志
    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  int16_t current[4] = {500, 500, 500, 500};
  uint8_t send_data[8] = {0};
  CAN_TxHeaderTypeDef Tx_message;
  uint32_t tx_Mailbox;
  can_filter_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Tx_message.DLC = 0x08;
    Tx_message.IDE = CAN_ID_STD;
    Tx_message.RTR = CAN_RTR_DATA;
    Tx_message.StdId = 0x200;

    send_data[0] = (current[0] >> 8) ;
    send_data[1] =  current[0];
    send_data[2] = (current[1] >> 8) ;
    send_data[3] =  current[1];
    send_data[4] = (current[2] >> 8) ;
    send_data[5] =  current[2];
    send_data[6] = (current[3] >> 8) ;
    send_data[7] =  current[3];
    HAL_CAN_AddTxMessage(&hcan1, &Tx_message, send_data, &tx_Mailbox);

    JustFloat_SendDMA();
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
