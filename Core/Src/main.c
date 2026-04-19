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
#include <stdio.h>
#include <math.h>   // fabsf
#include "oled.h"   // 引入真实的 OLED 驱动
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_DT                0.02f
#define DERIV_TAU                 0.03f
#define CONTROL_OUTPUT_LIMIT      300.0f
#define INTEGRAL_OUTPUT_LIMIT      80.0f   /* [C2] I项输出限幅 */
#define ERROR_DEADBAND_MM           2.0f   /* [C2] 死区 */

#define SERVO_CENTER_US           1500.0f
#define SERVO_MIN_US              1200.0f
#define SERVO_MAX_US              1800.0f

#define SERVO_X_DIR               1
#define SERVO_Y_DIR              -1

#define POS_MIN_MM                0.0f
#define POS_MAX_MM                300.0f
#define POS_FILTER_ALPHA          0.5f   /* [C1] 加快低通响应 */

#define LOST_FRAME_THRESHOLD      5
#define RX_TIMEOUT_MS             120U

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
typedef enum {
  STATE_INIT = 0,
  STATE_RUN,
  STATE_LOST
} ControlState;

typedef struct {
    float Kp, Ki, Kd;
    float integral;
  float last_measurement;
  float last_derivative;
} PID_TypeDef;

volatile PID_TypeDef pid_x, pid_y;

volatile ControlState control_state = STATE_INIT;

volatile float target_x = 150.0f; 
volatile float target_y = 150.0f; 
volatile float current_x = 0.0f;
volatile float current_y = 0.0f;
float last_pos_x = 0.0f;
float last_pos_y = 0.0f;
volatile uint8_t pos_filter_seeded = 0;  /* [C1] 首帧seed标志 */

uint8_t rx_buffer[50];                 
volatile uint8_t parse_buffer[50];     
volatile uint8_t rx_flag = 0;          
volatile uint16_t rx_len = 0;          

volatile uint8_t data_ready_flag = 0;  
volatile uint8_t servo_center_flag = 0;
volatile uint32_t last_rx_tick = 0;
int lost_frame_count = 0;
int invalid_frame_count = 0;

/* [C1] 重置PID内部状态 */
static void PID_Reset(volatile PID_TypeDef *pid, float measurement)
{
  pid->integral = 0.0f;
  pid->last_measurement = measurement;
  pid->last_derivative = 0.0f;
}

/* [C1] 进入LOST状态，同时清滤波器和PID历史 */
static void Control_EnterLostState(void)
{
  data_ready_flag = 0;
  servo_center_flag = 1;
  lost_frame_count = LOST_FRAME_THRESHOLD;
  control_state = STATE_LOST;
  pos_filter_seeded = 0;
  PID_Reset(&pid_x, current_x);
  PID_Reset(&pid_y, current_y);
}
/* [T1] 判断字符串是否是纯空白（不计入 invalid_frame_count） */
static uint8_t IsBlankLine(const char *p)
{
  while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t') {
    p++;
  }
  return (*p == '\0');
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
pid_x.Kp = 1.2f; pid_x.Ki = 0.05f; pid_x.Kd = 0.3f; pid_x.integral = 0.0f; pid_x.last_measurement = 0.0f; pid_x.last_derivative = 0.0f;
pid_y = pid_x;

// OLED 初始化与开机动画 (注意这里函数名改成了 OLED_ShowStr，并且加了 unsigned char* 强转消除警告)
OLED_Init();
OLED_Clear();
OLED_ShowStr(0, 0, (unsigned char*)"Ball Plate V1.0", 1);
OLED_ShowStr(0, 2, (unsigned char*)"System Booting...", 1);
HAL_Delay(500); 
OLED_Clear();

/* [S1] UART DMA 加返回值检查 */
if (HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer)) != HAL_OK) {
    Error_Handler();
}
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
last_rx_tick = HAL_GetTick();

/* [S1] 先写中位再开PWM，用 SERVO_CENTER_US 替代硬编码 */
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)SERVO_CENTER_US);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)SERVO_CENTER_US);

if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
}
if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
}

HAL_Delay(500);  /* [S1] PWM启动后等待舵机机械归中 */

if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 uint32_t last_oled_tick = 0; 

while (1)
{
    // === 阶段 A: 解析串口数据 ===
    if (rx_flag == 1) 
    {
        char local_buf[50];
        
        __disable_irq(); 
        rx_flag = 0;
        memcpy(local_buf, (void*)parse_buffer, sizeof(local_buf));
        __enable_irq();  
        
        char *p = local_buf;
        float raw_x = 0.0f;
        float raw_y = 0.0f;
        uint8_t valid_pos_frame = 0;

        while (*p == ' ' || *p == '\r' || *p == '\n' || *p == '\t') {
          p++;
        }
        
        if (p[0] == 'X' && p[1] == ':') {
            char *endptr;
          raw_x = strtof(p + 2, &endptr);
          char *comma = strchr(endptr, ','); 
            if (comma && *(comma + 1) == 'Y' && *(comma + 2) == ':') {
              char *y_start = comma + 3;
              char *y_end;
              raw_y = strtof(y_start, &y_end);
              if (y_end != y_start &&
                  (*y_end == '\0' || *y_end == '\r' || *y_end == '\n' ||
                   *y_end == ' ' || *y_end == '\t')) {
                valid_pos_frame = 1;
              }
            }
        }

        if (valid_pos_frame) {
          raw_x = fminf(POS_MAX_MM, fmaxf(POS_MIN_MM, raw_x));
          raw_y = fminf(POS_MAX_MM, fmaxf(POS_MIN_MM, raw_y));

          /* [C1] 首帧/恢复用原始值seed，避免从0低通 */
          if (!pos_filter_seeded || control_state != STATE_RUN) {
            last_pos_x = raw_x;
            last_pos_y = raw_y;
            pos_filter_seeded = 1U;
          } else {
            last_pos_x = POS_FILTER_ALPHA * raw_x + (1.0f - POS_FILTER_ALPHA) * last_pos_x;
            last_pos_y = POS_FILTER_ALPHA * raw_y + (1.0f - POS_FILTER_ALPHA) * last_pos_y;
          }

          current_x = last_pos_x;
          current_y = last_pos_y;

          /* [C1] 状态恢复时重置PID历史 */
          if (control_state != STATE_RUN) {
            PID_Reset(&pid_x, current_x);
            PID_Reset(&pid_y, current_y);
          }

          last_rx_tick = HAL_GetTick();
          data_ready_flag = 1U;
          lost_frame_count = 0;
          invalid_frame_count = 0;
          control_state = STATE_RUN;
        }
        else if (p[0] == 'T' && p[1] == ':') {
            char *endptr;
            float temp_x = strtof(p + 2, &endptr);
            char *comma = strchr(endptr, ',');
            if (comma && *(comma + 1) == 'Y' && *(comma + 2) == ':') {
              char *y_start = comma + 3;
              char *y_end;
              float temp_y = strtof(y_start, &y_end);
              if (y_end != y_start &&
                  (*y_end == '\0' || *y_end == '\r' || *y_end == '\n' ||
                   *y_end == ' ' || *y_end == '\t')) {
                target_x = fminf(POS_MAX_MM, fmaxf(POS_MIN_MM, temp_x));
                target_y = fminf(POS_MAX_MM, fmaxf(POS_MIN_MM, temp_y));
              }
            }
        }
        else if (strncmp(p, "PID:", 4) == 0) {
          char *cursor = p + 4;
          char *endptr;
          float p_val = strtof(cursor, &endptr);
          if (endptr != cursor && *endptr == ',') {
            cursor = endptr + 1;
            float i_val = strtof(cursor, &endptr);
            if (endptr != cursor && *endptr == ',') {
              cursor = endptr + 1;
              float d_val = strtof(cursor, &endptr);

              /* [C3] 跳过尾部空白 */
              while (*endptr == ' ' || *endptr == '\r' || *endptr == '\n' || *endptr == '\t') {
                endptr++;
              }

              /* [C3] 帧尾干净 + 数值有效 + 范围检查，全满足才写入 */
              if (endptr != cursor &&
                  *endptr == '\0' &&
                  isfinite(p_val) && isfinite(i_val) && isfinite(d_val) &&
                  p_val > 0.0f && p_val < 10.0f &&
                  i_val >= 0.0f && i_val < 5.0f &&
                  d_val >= 0.0f && d_val < 5.0f) {

                __disable_irq();
                /* [C3] 逐字段赋值，避免 pid_y=pid_x 污染 last_measurement */
                pid_x.Kp = p_val;
                pid_x.Ki = i_val;
                pid_x.Kd = d_val;
                pid_y.Kp = p_val;
                pid_y.Ki = i_val;
                pid_y.Kd = d_val;
                PID_Reset(&pid_x, current_x);
                PID_Reset(&pid_y, current_y);
                __enable_irq();
              }
            }
          }
        }
        else if (strncmp(p, "LOST", 4) == 0 &&
                 (p[4] == '\0' || p[4] == '\r' || p[4] == '\n' ||
                  p[4] == ' ' || p[4] == '\t')) {
          Control_EnterLostState();  /* [C1] 清PID/滤波状态 */
        }
            else if (!IsBlankLine(p)) {  /* [T1] 空白帧不计入 invalid */
              invalid_frame_count++;
              data_ready_flag = 0;
              if (invalid_frame_count >= LOST_FRAME_THRESHOLD) {
                Control_EnterLostState();
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
        
        if (control_state == STATE_RUN)       OLED_ShowStr(0, 4, (unsigned char*)"Stat: TRACKING ", 1);
        else if (control_state == STATE_LOST) OLED_ShowStr(0, 4, (unsigned char*)"Stat: LOST/CENT", 1);
        else                                  OLED_ShowStr(0, 4, (unsigned char*)"Stat: WAIT VIS ", 1);
        
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

float PID_Calculate(volatile PID_TypeDef *pid, float target, float current, float dt) 
{
  if (dt <= 0.0f) {
    dt = CONTROL_DT;
  }

    float error = target - current;
    /* [C2] 死区：目标附近不产生控制量 */
    if (fabsf(error) < ERROR_DEADBAND_MM) {
      error = 0.0f;
    }

    pid->integral += (error * dt);

    /* [C2] 积分限幅按I项输出量级 */
    if (pid->Ki > 0.0f) {
      float integral_limit = INTEGRAL_OUTPUT_LIMIT / pid->Ki;
      pid->integral = LIMIT(pid->integral, -integral_limit, integral_limit);
    } else {
      pid->integral = 0.0f;
    }
    
  float raw_derivative = -(current - pid->last_measurement) / dt;
  float alpha = dt / (DERIV_TAU + dt);
  pid->last_derivative = alpha * raw_derivative + (1.0f - alpha) * pid->last_derivative;
  pid->last_measurement = current;

  float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * pid->last_derivative);
  output = LIMIT(output, -CONTROL_OUTPUT_LIMIT, CONTROL_OUTPUT_LIMIT);
    
  return output;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) 
    {
    if ((HAL_GetTick() - last_rx_tick) > RX_TIMEOUT_MS) {
      Control_EnterLostState();  /* [C1] 超时清PID/滤波状态 */
    }

        if(servo_center_flag) {
            servo_center_flag = 0;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)SERVO_CENTER_US);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)SERVO_CENTER_US);
            return; 
        }

    if(data_ready_flag == 0 || control_state != STATE_RUN) return; 

    float pid_out_x = PID_Calculate(&pid_x, target_x, current_x, CONTROL_DT);
    float pid_out_y = PID_Calculate(&pid_y, target_y, current_y, CONTROL_DT);
        
    float pwm_x = SERVO_CENTER_US + ((float)SERVO_X_DIR * pid_out_x);
    float pwm_y = SERVO_CENTER_US + ((float)SERVO_Y_DIR * pid_out_y);

    pwm_x = LIMIT(pwm_x, SERVO_MIN_US, SERVO_MAX_US);
    pwm_y = LIMIT(pwm_y, SERVO_MIN_US, SERVO_MAX_US);

    uint32_t ccr_x = (uint32_t)pwm_x;
    uint32_t ccr_y = (uint32_t)pwm_y;
        
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
