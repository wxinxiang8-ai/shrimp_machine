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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rm3508.h"
#include "dc_motor.h"
#include "emm_motor.h"
#include "Screen.h"
#include "machine_workflow.h"
#include "dump.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_MODE_RM3508         0
#define TEST_MODE_DC_MOTOR       1
#define TEST_MODE_BOTH_MOTORS    2
#define TEST_MODE_EMM_DUAL       3
#define TEST_MODE_ALL_MOTORS     4
#define TEST_MODE_SCREEN         5

#define ACTIVE_TEST_MODE         TEST_MODE_SCREEN

#define RM3508_TEST_MOTOR_ID     0
#define RM3508_TEST_SPEED_RPM    2000
#define RM3508_TEST_DIRECTION    1
#define RM3508_RUN_CURRENT       8000
#define DC_TEST_TARGET_RPM       1200
#define DC_TEST_ACCEL_STEP_RPM   100
#define DC_TEST_STEP_DELAY_MS    200
#define DC_TEST_RUN_TIME_MS      20000
#define DC_TEST_STOP_TIME_MS     2000

#define EMM_TEST_MOTOR_ID_A       4
#define EMM_TEST_MOTOR_ID_B       1
#define EMM_TEST_DIR_A            1
#define EMM_TEST_DIR_B            1
#define EMM_TEST_SPEED_RPM_A      50
#define EMM_TEST_SPEED_RPM_B      50
#define EMM_TEST_ACCEL            10
#define EMM_TEST_RUN_TIME_MS      5000
#define EMM_TEST_STOP_TIME_MS     2000
#define EMM_TEST_CMD_GAP_MS       50
#define EMM_TEST_ENABLE_DELAY_MS  100
#define EMM_TEST_STARTUP_DELAY_MS 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t g_dc_test_speed_rpm = 0;
MachineContext_t g_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void RM3508_RawCanInit(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &filter);
  HAL_CAN_Start(&hcan1);
}

static void RM3508_RawSendCurrent(int16_t current)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8] = {0};
  uint32_t tx_mailbox = 0;

  tx_header.StdId = 0x200;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  tx_data[0] = (uint8_t)(current >> 8);
  tx_data[1] = (uint8_t)(current & 0xFF);

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
  {
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
#if (ACTIVE_TEST_MODE == TEST_MODE_DC_MOTOR)
  DC_Motor_Init();
  DC_Motor_SetDirection(2);  /* stopped */
  DC_Motor_SetSpeed(0);
#elif (ACTIVE_TEST_MODE == TEST_MODE_RM3508)
  RM3508_RawCanInit();
  HAL_Delay(200);
#elif (ACTIVE_TEST_MODE == TEST_MODE_BOTH_MOTORS)
  Dump_Init();
  Dump_Stop();
  HAL_Delay(200);
#elif (ACTIVE_TEST_MODE == TEST_MODE_EMM_DUAL)
  EMM_MOTOR_Init();
  HAL_Delay(EMM_TEST_STARTUP_DELAY_MS);
  Emm_V5_Reset_Clog_Pro(EMM_TEST_MOTOR_ID_A);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_Modify_Ctrl_Mode(EMM_TEST_MOTOR_ID_A, false, 2);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_En_Control(EMM_TEST_MOTOR_ID_A, true, false);
  HAL_Delay(EMM_TEST_ENABLE_DELAY_MS);
  Emm_V5_Stop_Now(EMM_TEST_MOTOR_ID_A, false);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_Reset_Clog_Pro(EMM_TEST_MOTOR_ID_B);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_Modify_Ctrl_Mode(EMM_TEST_MOTOR_ID_B, false, 2);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_En_Control(EMM_TEST_MOTOR_ID_B, true, false);
  HAL_Delay(EMM_TEST_ENABLE_DELAY_MS);
  Emm_V5_Stop_Now(EMM_TEST_MOTOR_ID_B, false);
  HAL_Delay(200);
#elif (ACTIVE_TEST_MODE == TEST_MODE_ALL_MOTORS)
  DC_Motor_Init();
  DC_Motor_SetDirection(0);
  DC_Motor_SetSpeed(0);
  EMM_MOTOR_Init();
  HAL_Delay(EMM_TEST_STARTUP_DELAY_MS);
  Emm_V5_Reset_Clog_Pro(EMM_TEST_MOTOR_ID_A);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_Modify_Ctrl_Mode(EMM_TEST_MOTOR_ID_A, false, 2);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_En_Control(EMM_TEST_MOTOR_ID_A, true, false);
  HAL_Delay(EMM_TEST_ENABLE_DELAY_MS);
  Emm_V5_Stop_Now(EMM_TEST_MOTOR_ID_A, false);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_Reset_Clog_Pro(EMM_TEST_MOTOR_ID_B);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_Modify_Ctrl_Mode(EMM_TEST_MOTOR_ID_B, false, 2);
  HAL_Delay(EMM_TEST_CMD_GAP_MS);
  Emm_V5_En_Control(EMM_TEST_MOTOR_ID_B, true, false);
  HAL_Delay(EMM_TEST_ENABLE_DELAY_MS);
  Emm_V5_Stop_Now(EMM_TEST_MOTOR_ID_B, false);
  HAL_Delay(200);
#elif (ACTIVE_TEST_MODE == TEST_MODE_SCREEN)
  /* Screen workflow mode: hardware init is handled by machine_workflow */
#endif

#if (ACTIVE_TEST_MODE == TEST_MODE_SCREEN)
  Machine_Workflow_Init(&g_ctx);
  Screen_Init();

  HAL_Delay(300);
  /* Draw full UI via GUI commands */
  Screen_DrawFullUI(&g_ctx);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if (ACTIVE_TEST_MODE == TEST_MODE_DC_MOTOR)
    DC_Motor_SetDirection(0);

    for (g_dc_test_speed_rpm = 0; g_dc_test_speed_rpm <= DC_TEST_TARGET_RPM; g_dc_test_speed_rpm += DC_TEST_ACCEL_STEP_RPM)
    {
      DC_Motor_SetSpeed(g_dc_test_speed_rpm);
      HAL_Delay(DC_TEST_STEP_DELAY_MS);
    }

    DC_Motor_SetSpeed(DC_TEST_TARGET_RPM);
    HAL_Delay(DC_TEST_RUN_TIME_MS);

    for (g_dc_test_speed_rpm = DC_TEST_TARGET_RPM; g_dc_test_speed_rpm >= 0; g_dc_test_speed_rpm -= DC_TEST_ACCEL_STEP_RPM)
    {
      DC_Motor_SetSpeed(g_dc_test_speed_rpm);
      HAL_Delay(DC_TEST_STEP_DELAY_MS);
    }

    DC_Motor_SetDirection(2);
    DC_Motor_SetSpeed(0);
    HAL_Delay(DC_TEST_STOP_TIME_MS);
#elif (ACTIVE_TEST_MODE == TEST_MODE_RM3508)
    RM3508_RawSendCurrent(RM3508_RUN_CURRENT);
    HAL_Delay(10);
#elif (ACTIVE_TEST_MODE == TEST_MODE_BOTH_MOTORS)
    Dump_Start();
    HAL_Delay(100);
#elif (ACTIVE_TEST_MODE == TEST_MODE_EMM_DUAL)
    EMM_Vel_control(EMM_TEST_MOTOR_ID_A, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_A, EMM_TEST_ACCEL, false);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    EMM_Vel_control(EMM_TEST_MOTOR_ID_B, EMM_TEST_DIR_B, EMM_TEST_SPEED_RPM_B, EMM_TEST_ACCEL, false);
    HAL_Delay(100);
#elif (ACTIVE_TEST_MODE == TEST_MODE_ALL_MOTORS)
    DC_Motor_SetDirection(0);

    Emm_V5_Reset_Clog_Pro(EMM_TEST_MOTOR_ID_A);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    Emm_V5_Modify_Ctrl_Mode(EMM_TEST_MOTOR_ID_A, false, 2);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    Emm_V5_En_Control(EMM_TEST_MOTOR_ID_A, true, false);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    Emm_V5_Reset_Clog_Pro(EMM_TEST_MOTOR_ID_B);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    Emm_V5_Modify_Ctrl_Mode(EMM_TEST_MOTOR_ID_B, false, 2);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    Emm_V5_En_Control(EMM_TEST_MOTOR_ID_B, true, false);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);

    for (g_dc_test_speed_rpm = 0; g_dc_test_speed_rpm <= DC_TEST_TARGET_RPM; g_dc_test_speed_rpm += DC_TEST_ACCEL_STEP_RPM)
    {
      DC_Motor_SetSpeed(g_dc_test_speed_rpm);
      HAL_Delay(EMM_TEST_CMD_GAP_MS);
      EMM_Vel_control(EMM_TEST_MOTOR_ID_A, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_A, EMM_TEST_ACCEL, false);
      HAL_Delay(EMM_TEST_CMD_GAP_MS);
      EMM_Vel_control(EMM_TEST_MOTOR_ID_B, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_B, EMM_TEST_ACCEL, false);
      HAL_Delay(DC_TEST_STEP_DELAY_MS);
    }

    DC_Motor_SetSpeed(DC_TEST_TARGET_RPM);
    for (uint32_t i = 0; i < (DC_TEST_RUN_TIME_MS / 10U); i++)
    {
      if ((i % 10U) == 0U)
      {
        EMM_Vel_control(EMM_TEST_MOTOR_ID_A, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_A, EMM_TEST_ACCEL, false);
        HAL_Delay(EMM_TEST_CMD_GAP_MS);
        EMM_Vel_control(EMM_TEST_MOTOR_ID_B, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_B, EMM_TEST_ACCEL, false);
      }
      HAL_Delay(10);
    }

    for (g_dc_test_speed_rpm = DC_TEST_TARGET_RPM; g_dc_test_speed_rpm >= 0; g_dc_test_speed_rpm -= DC_TEST_ACCEL_STEP_RPM)
    {
      DC_Motor_SetSpeed(g_dc_test_speed_rpm);
      HAL_Delay(EMM_TEST_CMD_GAP_MS);
      EMM_Vel_control(EMM_TEST_MOTOR_ID_A, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_A, EMM_TEST_ACCEL, false);
      HAL_Delay(EMM_TEST_CMD_GAP_MS);
      EMM_Vel_control(EMM_TEST_MOTOR_ID_B, EMM_TEST_DIR_A, EMM_TEST_SPEED_RPM_B, EMM_TEST_ACCEL, false);
      HAL_Delay(DC_TEST_STEP_DELAY_MS);
    }

    DC_Motor_SetDirection(2);
    DC_Motor_SetSpeed(0);
    Emm_V5_Stop_Now(EMM_TEST_MOTOR_ID_A, false);
    HAL_Delay(EMM_TEST_CMD_GAP_MS);
    Emm_V5_Stop_Now(EMM_TEST_MOTOR_ID_B, false);
    HAL_Delay(DC_TEST_STOP_TIME_MS);
#elif (ACTIVE_TEST_MODE == TEST_MODE_SCREEN)
    /* Screen test mode: hardware is driven by UI events below */
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if (ACTIVE_TEST_MODE == TEST_MODE_SCREEN)
    Screen_ProcessRx();
    App_ProcessEvents(&g_ctx);
    App_ApplyOutputs(&g_ctx);
    App_UpdateRuntime(&g_ctx);
    Screen_RefreshDirty(&g_ctx);
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
