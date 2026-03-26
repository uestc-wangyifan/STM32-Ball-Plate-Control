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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h> // strtof 所在库
#include <math.h>   // fabsf
#include "oled.h"   // 引入真实的 OLED 驱动
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 1. 物理意义清晰的 PID 结构体
// 1. 定义 PID 结构体
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float last_derivative;  // D项低通滤波记忆值
} PID_TypeDef;

// 声明两个 PID 变量
PID_TypeDef pid_x, pid_y;

volatile float target_x = 150.0f; 
volatile float target_y = 150.0f; 
volatile float current_x = 0.0f;
volatile float current_y = 0.0f;

uint8_t rx_buffer[50];                 
volatile uint8_t parse_buffer[50];     
volatile uint8_t rx_flag = 0;          
volatile uint16_t rx_len = 0;          

volatile uint8_t data_ready_flag = 0;  
volatile uint8_t servo_center_flag = 0;
volatile uint8_t is_lost_state = 1;    
int lost_frame_count = 0;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
// 初始化PID
pid_x.Kp = 1.2f; pid_x.Ki = 0.05f; pid_x.Kd = 0.3f; pid_x.integral = 0; pid_x.last_error = 0; pid_x.last_derivative = 0;
pid_y.Kp = 1.2f; pid_y.Ki = 0.05f; pid_y.Kd = 0.3f; pid_y.integral = 0; pid_y.last_error = 0; pid_y.last_derivative = 0;

// OLED 初始化与开机动画 (注意这里函数名改成了 OLED_ShowStr，并且加了 unsigned char* 强转消除警告)
OLED_Init();
OLED_Clear();
OLED_ShowStr(0, 0, (unsigned char*)"Ball Plate V1.0", 1);
OLED_ShowStr(0, 2, (unsigned char*)"System Booting...", 1);
HAL_Delay(500); 
OLED_Clear();

HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer)); 
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                 

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); 
// Fix2: 上电安全归中 — 强制舵机到中位(1500us)，防止上电瞬间满偏
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
HAL_Delay(500);  // 让舵机有时间稳定到中位
HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 uint32_t last_oled_tick = 0; 

while (1)
{
    // === 阶段 A: 解析串口数据 ===
    if (rx_flag == 1) 
    {
        rx_flag = 0; 
        char local_buf[50];
        
        __disable_irq(); 
        memcpy(local_buf, (void*)parse_buffer, sizeof(local_buf));
        __enable_irq();  
        
        char *p = local_buf;
        
        if (p[0] == 'X' && p[1] == ':') {
            char *endptr;
            float temp_x = strtof(p + 2, &endptr);
            char *comma = strchr(endptr, ','); 
            if (comma && *(comma + 1) == 'Y' && *(comma + 2) == ':') {
                current_x = temp_x;
                current_y = strtof(comma + 3, NULL);
                data_ready_flag = 1; 
                lost_frame_count = 0; 
                is_lost_state = 0; 
            }
        } 
        else if (p[0] == 'T' && p[1] == ':') {
            char *endptr;
            float temp_x = strtof(p + 2, &endptr);
            char *comma = strchr(endptr, ',');
            if (comma && *(comma + 1) == 'Y' && *(comma + 2) == ':') {
                target_x = temp_x;
                target_y = strtof(comma + 3, NULL);
            }
        }
        else if (strncmp(local_buf, "LOST", 4) == 0) {
            lost_frame_count++;
            if (lost_frame_count >= 5) {
                data_ready_flag = 0;     
                servo_center_flag = 1;   
                is_lost_state = 2;       
            }
        }
    }

    // === 阶段 B: OLED 异步显示 ===
    if (HAL_GetTick() - last_oled_tick >= 100) 
    {
        last_oled_tick = HAL_GetTick();
        char disp_buf[20]; 
        
        sprintf(disp_buf, "Cur:X%03d Y%03d", (int)current_x, (int)current_y);
        OLED_ShowStr(0, 0, (unsigned char*)disp_buf, 1);
        
        sprintf(disp_buf, "Tar:X%03d Y%03d", (int)target_x, (int)target_y);
        OLED_ShowStr(0, 2, (unsigned char*)disp_buf, 1);
        
        if (is_lost_state == 0)      OLED_ShowStr(0, 4, (unsigned char*)"Stat: TRACKING ", 1);
        else if (is_lost_state == 2) OLED_ShowStr(0, 4, (unsigned char*)"Stat: LOST/CENT", 1);
        else                         OLED_ShowStr(0, 4, (unsigned char*)"Stat: WAIT VIS ", 1);
        
        sprintf(disp_buf, "L-Cnt:%d   ", lost_frame_count);
        OLED_ShowStr(0, 6, (unsigned char*)disp_buf, 1);
    }
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
#define LIMIT(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define INTEGRAL_MAX 200.0f 

// 带有 dt 物理意义的抗饱和 PID + D项一阶低通滤波
float PID_Calculate(PID_TypeDef *pid, float target, float current, float dt) 
{
    float error = target - current;
    pid->integral += (error * dt);
    
    if(pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
    else if(pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;
    
    // Fix1: D项一阶低通滤波 — 0.3*新值+0.7*旧值，滤除像素噪声导致的高频震荡
    float raw_derivative = (error - pid->last_error) / dt;
    pid->last_derivative = 0.3f * raw_derivative + 0.7f * pid->last_derivative;
    pid->last_error = error;
    
    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * pid->last_derivative);
}

// TIM2 控制心跳：全系统唯一允许操作 PWM 硬件的地方，彻底消灭数据竞争
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) 
    {
        // 优先级1：失控归中 (一触即发并立刻清零，防止阻塞 PID)
        if(servo_center_flag) {
            servo_center_flag = 0;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1500);
            return; 
        }

        // 优先级2：无有效数据不动作
        if(data_ready_flag == 0) return; 

        // Fix3: 误差死区 — 极小误差时不动作，防止舵机在目标点附近高频微颤
        float err_x = target_x - current_x;
        float err_y = target_y - current_y;
        if(fabsf(err_x) < 2.0f && fabsf(err_y) < 2.0f) return;

        // 优先级3：闭环计算 (dt = 0.02s)
        float pid_out_x = PID_Calculate(&pid_x, target_x, current_x, 0.02f);
        float pid_out_y = PID_Calculate(&pid_y, target_y, current_y, 0.02f);
        
        // 【注意】如果上机发现系统为正反馈(球越跑越远)，把下面的 + pid_out 改为 - pid_out
        uint32_t ccr_x = (uint32_t)LIMIT(1500.0f + pid_out_x, 1200.0f, 1800.0f);
        uint32_t ccr_y = (uint32_t)LIMIT(1500.0f + pid_out_y, 1200.0f, 1800.0f);
        
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_x);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr_y);
    }
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
