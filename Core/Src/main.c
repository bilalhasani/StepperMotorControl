/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STEPS_PER_REV 200 // Number of steps per revolution for the stepper motor (NEMA17)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char rxBuffer[1]; // Buffer for single character received via UART
char commandBuffer[50]; // Buffer to accumulate command
uint8_t commandIndex = 0; // Index for command buffer
bool motorRunning = false; // Motor state
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void StartMotor(float _rpm, bool _direction);
void StopMotor(void);
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
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    
    // Send startup message over UART1
    char startupMessage[] = "Stepper Motor Control Initialized.\r\nSend \"START <Speed> <Direction>\" or \"STOP\" commands.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)startupMessage, strlen(startupMessage), HAL_MAX_DELAY);
    
    // Start UART reception in interrupt mode
    HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);
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
    
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
* @brief TIM2 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM2_Init(void)
{
    
    /* USER CODE BEGIN TIM2_Init 0 */
    
    /* USER CODE END TIM2_Init 0 */
    
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* USER CODE BEGIN TIM2_Init 1 */
    
    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */
    
    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
    
}

/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART1_UART_Init(void)
{
    
    /* USER CODE BEGIN USART1_Init 0 */
    
    /* USER CODE END USART1_Init 0 */
    
    /* USER CODE BEGIN USART1_Init 1 */
    
    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */
    
    /* USER CODE END USART1_Init 2 */
    
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
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    
    /*Configure GPIO pin : PA2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* USER CODE BEGIN MX_GPIO_Init_2 */
    
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // Check if the interrupt is from UART1
    {
        // Echo the received character back to the sender to make a console style interface
        HAL_UART_Transmit(&huart1, (uint8_t *)rxBuffer, 1, HAL_MAX_DELAY);
        
        // Check for line feed character
        if (rxBuffer[0] == '\r')
        {
            commandBuffer[commandIndex] = '\0'; // Null-terminate the command string
            commandIndex = 0; // Reset the index for the next command
            
            // Process the command
            int speed = 10;      // Default speed in steps/second
            bool direction = true; // Default direction
            
            // Parse the command manually using strtok
            char *token = strtok(commandBuffer, " "); // Split by space
            if (token != NULL && strcmp(token, "START") == 0) 
            {
                token = strtok(NULL, " "); // Get the speed
                if (token != NULL) 
                {
                    speed = atoi(token); // Convert speed to integer
                    token = strtok(NULL, " "); // Get the direction
                    if (token != NULL) 
                    {
                        direction = atoi(token); // Convert direction to integer
                        StartMotor(speed, direction);

                        char startMessage[100];
                        sprintf(startMessage, "Command: [%s], Decoded: Speed: [%d], Direction [%d]\r\n", commandBuffer, speed, direction);
                        HAL_UART_Transmit(&huart1, (uint8_t *)startMessage, strlen(startMessage), HAL_MAX_DELAY);
                    }                    
                }
                if (token == NULL) 
                {
                    char invalidCommand[] = "Invalid command format. Use START <Speed> <Direction>\r\n";
                    HAL_UART_Transmit(&huart1, (uint8_t *)invalidCommand, strlen(invalidCommand), HAL_MAX_DELAY);
                }
            }
            else if (strncmp(commandBuffer, "STOP", 4) == 0) 
            {
                StopMotor();
                // Send warning message for invalid command
                char stopMessage[] = "Motor Stopped\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t *)stopMessage, strlen(stopMessage), HAL_MAX_DELAY);
            }
            else 
            {
                // Send warning message for invalid command
                char warningMessage[] = "Invalid command received. Use START or STOP.\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t *)warningMessage, strlen(warningMessage), HAL_MAX_DELAY);
            }
        } 
        else 
        {
            // Accumulate characters in the command buffer
            if (commandIndex < sizeof(commandBuffer) - 1) 
            {
                commandBuffer[commandIndex++] = rxBuffer[0];
            }
        }
        
        // Restart UART reception in interrupt mode
        HAL_UART_Receive_IT(&huart1, (uint8_t *)rxBuffer, 1);
    }
}

void StartMotor(float _rpm, bool _direction) {
    
    /*
    * 1. Convert RPM to STEP frequency
    * 2. Get actual TIM2 input clock from HAL (from APB1)
    * 3. Find prescaler and period (ARR) for desired frequency
    * 4. Stop the timer before reconfiguring
    * 5. Apply new settings
    * 6. Start PWM
    */
    
    // Frequency = SYSCLOCK / ((prescaler + 1) * (period + 1))
    // slowest frequency is 1.86Hz = 8MHz / (65536 * 65536) => 0.558 RPM
    // so we assume any thing less than 0.6 RPM is motor off 
    if (_rpm <= 0.6f) 
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        motorRunning = false;
        return;
    }
    
    // Set direction
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, _direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // 1. Convert RPM to STEP frequency
    // NEMA17 has 200 steps/rev, so:
    float step_freq = (_rpm * STEPS_PER_REV) / 60.0f;  // in Hz
    
    // 2. Get actual TIM2 input clock from HAL (from APB1)
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    
    // If APB1 prescaler > 1, timer clock gets doubled (STM32F1 rule)
    RCC_ClkInitTypeDef clk_config;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clk_config, &latency);
    if (clk_config.APB1CLKDivider != RCC_HCLK_DIV1) {
        timer_clock *= 2;
    }
    
    // 3. Find prescaler and period (ARR) for desired frequency
    uint32_t prescaler = 0;
    uint32_t period = 0;
    
    for (prescaler = 0; prescaler <= 0xFFFF; prescaler++) {
        period = (uint32_t)(timer_clock / ((prescaler + 1) * step_freq));
        if (period <= 0xFFFF) {
            break;
        }
    }
    
    if (prescaler > 0xFFFF || period == 0) {
        // Can't configure for this RPM
        return;
    }
    
    // 4. Stop the timer before reconfiguring
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    
    // 5. Apply new settings
    htim2.Instance->PSC = prescaler;
    htim2.Instance->ARR = period - 1;
    htim2.Instance->CCR2 = (period - 1) / 2;  // 50% duty
    
    // 6. Start PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    
    motorRunning = true;
}

void StopMotor(void) {
    // Stop PWM
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    
    motorRunning = false;
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
