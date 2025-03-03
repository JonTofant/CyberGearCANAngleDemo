/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P_MIN  -12.5f     // min angle (or -4π or some doc example)
#define P_MAX  12.5f      // max angle
#define V_MIN  -30.0f
#define V_MAX   30.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN  -12.0f
#define T_MAX   12.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t pTxMailbox;

uint8_t received_data[8];
// UART
char RxUARTBuffer[256]="";
uint8_t RxUARTLength=0;
uint8_t RxSingleByte;
uint8_t onFlag = 0;
float angle = 0.0f;
int toggle = 0;
uint16_t angle_u;





// initial target angle. Value should be 6.28... for a full rotation
volatile float targetAngle = 1.0f;


// We'll define prototypes for param writes, enabling motor, etc.
// We'll place the actual code in USER CODE BEGIN 4, but these must be declared first.
HAL_StatusTypeDef writeParameter(uint16_t paramIndex, const volatile void* paramValue,
                                 uint8_t hostID, uint8_t motorID);

HAL_StatusTypeDef getMotorDeviceID(uint8_t hostID, uint8_t motorID);
HAL_StatusTypeDef clearMotorFault(uint8_t hostID, uint8_t motorID);

HAL_StatusTypeDef motorEnable(uint8_t hostID, uint8_t motorID);
HAL_StatusTypeDef motorStop(uint8_t hostID, uint8_t motorID);
void testMotor(void);

// We'll do a simple function that sets position mode and moves the motor

HAL_StatusTypeDef motorControlMode(uint8_t hostID, uint8_t motorID,
                                   float torque, float position,
                                   float velocity, float kp, float kd);

HAL_StatusTypeDef motorControlFrame(
    uint8_t  motorID,        // bits 7..0 in extended ID
    float    torque,         // –12..+12 Nm
    float    angle,          // e.g. –4..+4 rad
    float    velocity,       // e.g. –30..+30 rad/s
    float    kp,             // 0..500
    float    kd              // 0..5
);
/* USER CODE END PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void serialWrite(char data[]);
void serialProcessRxData();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
static uint16_t float_to_uint(float x, float x_min, float x_max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  // UART
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_UART_Receive_IT(&huart2, &RxSingleByte, 1);
  char buffer[11] = "CyberGear Serial CAN Receiver\n";
  HAL_UART_Transmit_IT(&huart2,(uint8_t *) buffer, 11);
  // CAN
  // 1) Configure the CAN filter to accept *all* extended frames
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow  = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow  = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);



  // 2) Start CAN
  HAL_CAN_Start(&hcan1);



  // 3) Optionally enable Rx interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  getMotorDeviceID(0xFE, 0x7F);   // type=0 request




	  // 1) Clear fault
	  //clearMotorFault(0xFE, 0x7F);
	  //HAL_Delay(20);

	  //motorEnable(0xFE, 0x7F);  // type=3
	  //HAL_Delay(10);

	  testMotor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_DIR_Pin|LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DIR_Pin LD1_Pin LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_Pin|LD1_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


#include <string.h>  // for memcpy

/**
 * Write a 4-byte parameter at index paramIndex (e.g. 0x7005 for run_mode).
 *   paramValue must point to 4 bytes (e.g. float).
 *   type=18 (0x12) in bits28..24
 */
HAL_StatusTypeDef writeParameter(uint16_t paramIndex, const volatile void* paramValue,
                                 uint8_t hostID, uint8_t motorID)

{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // Build extended ID => (type=18)
    uint32_t extId = ((uint32_t)0x12 << 24)
                   | ((uint32_t)hostID << 8)
                   | (uint32_t)motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // Byte0..1 = paramIndex (little-endian)
    txData[0] = (uint8_t)(paramIndex & 0xFF);
    txData[1] = (uint8_t)(paramIndex >> 8);
    // Byte2..3 = 0
    // Byte4..7 = paramValue
    memcpy(&txData[4], paramValue, 4);

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/**
 * Enable motor => type=3
 */
HAL_StatusTypeDef motorEnable(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    uint32_t extId = ((uint32_t)3 << 24) |
                     ((uint32_t)hostID << 8) |
                     motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/**
 * Stop motor => type=4
 */
HAL_StatusTypeDef motorStop(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8] = {0};
    uint32_t txMailbox;

    uint32_t extId = ((uint32_t)4 << 24) |
                     ((uint32_t)hostID << 8) |
                     motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // data[0] = 1 => if you want to clear faults, else 0
    // txData[0] = 1;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}





/**
 * Ask the motor for its device ID => type=0
 * The motor (if it hears this) should respond with type=0,
 * data=64-bit unique MCU ID, or an info frame.
 */
HAL_StatusTypeDef getMotorDeviceID(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // 0 in bits28..24 => get device ID
    uint32_t extId = ((uint32_t)0 << 24)
                   | ((uint32_t)hostID << 8)
                   | (uint32_t)motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;     // extended frame
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // Typically data can be all 0
    memset(txData, 0, 8);

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}


HAL_StatusTypeDef clearMotorFault(uint8_t hostID, uint8_t motorID)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t txData[8] = {0};

    // 4 in bits28..24 => Stop command
    uint32_t extId = ((uint32_t)4 << 24)
                   | ((uint32_t)hostID << 8)
                   | (uint32_t)motorID;

    txHeader.ExtId = extId;
    txHeader.IDE   = CAN_ID_EXT;     // extended frame
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    // data[0] = 1 => clear fault
    txData[0] = 1;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status;
    status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, received_data);
    if (status != HAL_OK)
    {
        // Error reading the message
        return;
    }

    uint32_t extId = pRxHeader.ExtId;
    uint8_t type   = (extId >> 24) & 0x1F;   // bits28..24
    uint8_t motorID= extId & 0xFF;          // bits7..0

    // For debug, print the entire extended ID
    char dbg[128];
    sprintf(dbg, "RX: ExtId=0x%08lX (type=%u, motorID=%u), DLC=%u\n",
            (unsigned long)extId, (unsigned)type, (unsigned)motorID, pRxHeader.DLC);
    serialWrite(dbg);

    // Check what type we received
    if (type == 2)
    {
        // This is typically the motor feedback frame
        // Byte0..1 might be current angle, Byte2..3 velocity, Byte4..5 torque, Byte6..7 temperature, etc.
        sprintf(dbg, "Received type=2 (feedback) from motorID=%u\n", (unsigned)motorID);
        serialWrite(dbg);

        // Optionally parse the data
        // e.g. parse angle:
        angle_u = (received_data[0] << 8) | received_data[1];
        // etc. Then print or store
    }
    else if (type == 21)
    {
        // This is typically the fault/error frame
        // Byte0..3 might be a fault code
        sprintf(dbg, "Received type=21 (fault) from motorID=%u\n", (unsigned)motorID);
        serialWrite(dbg);

        // Optionally parse the fault code in received_data[0..3]
        // For example:
        uint32_t fault = (received_data[0])
                       | (received_data[1] << 8)
                       | (received_data[2] << 16)
                       | (received_data[3] << 24);
        sprintf(dbg, "Fault code = 0x%08lX\n", (unsigned long)fault);
        serialWrite(dbg);
    }
    else if (type == 0)
    {
        // Possibly a "Get Device ID" response
        sprintf(dbg, "Received type=0 (device info) from motorID=%u\n", (unsigned)motorID);
        serialWrite(dbg);
        // The data bytes may contain the 64-bit unique ID
    }
    else
    {
        // Some other type
        sprintf(dbg, "Received unknown type=%u from motorID=%u\n",
                (unsigned)type, (unsigned)motorID);
        serialWrite(dbg);
    }
}

void testMotor(void)
{
    // 1) Clear fault
    clearMotorFault(/*hostID=*/0x00, /*motorID=*/0x7F);
    HAL_Delay(20);

    // 2) Enable motor
    motorEnable(/*hostID=*/0x00, /*motorID=*/0x7F);
    HAL_Delay(20);


    // 2) Put motor in position mode
    uint8_t runMode = 1; // 1 => position mode
    writeParameter(0x7005, &runMode, /*hostID=*/0xFE, /*motorID=*/0x7F);


    float speedLimit = 1.0f; // e.g. 1 rad/s
    writeParameter(0x7017, &speedLimit, 0xFE, 0x7F);
    HAL_Delay(10);

    // 3) Send repeated frames in a loop
    while(1)
    {

    	writeParameter(0x7016, &targetAngle, 0xFE, 0x7F);

        HAL_Delay(10); // ~100 Hz
    }
}



void serialWrite(char data[]){
	HAL_UART_Transmit(&huart2, (uint8_t *) data, strlen(data), 10);
	HAL_UART_Transmit(&huart2,(uint8_t *)"\n",1,10);
}

void serialProcessRxData(){

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){

	}
}

/*
 * Function used for MIT control.
 */
static uint16_t float_to_uint(float x, float x_min, float x_max)
{
    // Clamp x into [x_min, x_max]
    if(x < x_min) x = x_min;
    if(x > x_max) x = x_max;

    float span = (x_max - x_min);
    float offset = (x - x_min);

    // Scale into [0..65535]
    // The Xiaomi snippet does:
    //   (x - offset)*(65535.0f)/span
    // but we can do something like:
    return (uint16_t)((offset * 65535.0f) / span + 0.5f);
}


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

#ifdef  USE_FULL_ASSERT
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
