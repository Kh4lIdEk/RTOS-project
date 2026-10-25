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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "retarget.h"
#include "sensirion_gas_index_algorithm.h"
#include "../../Drivers/VL53L1_Nucleo/core/inc/vl53l1_api.h"
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- Serial console / BLE ---- */
#define COM_HANDLE              hlpuart1
#define COM_INSTANCE            LPUART1

/* ---- Ambient light hysteresis (%) ---- */
#define LUM_DARK_THRESHOLD      30      /* below  -> dark mode   */
#define LUM_BRIGHT_THRESHOLD    40      /* above  -> normal mode */

/* ---- Alarm ---- */
#define SNOOZE_MS               5000U
#define N_SOUNDS                4U

/* ---- ToF gestures ---- */
#define HAND_THRESHOLD_MM   200U    /* a hand is "seen" below this distance */
#define SAMPLE_PERIOD_MS    50U     /* sampling period of both sensors */
#define SWIPE_WINDOW_MS     500U    /* max delay ToF1 -> ToF2 for a swipe */
#define SWIPE_LOCKOUT_MS    1000U   /* ignore new swipes right after one */
#define HOLD_STOP_MS        3000U   /* hold time to stop the alarm */
#define HOLD_GRACE_MS       200U    /* tolerated dropout while holding */

/* ---- WS2812 (TIM3 ARR = 99 @ 80 MHz -> 1.25 us bit) ---- */
#define WS2812_BIT_0            32
#define WS2812_BIT_1            64
#define WS2812_RESET_SLOTS      50
#define N_LEDS                  4U
#define WS2812_FRAME_LEN        (N_LEDS * 24U + WS2812_RESET_SLOTS)

/* ---- SGP40 ---- */
#define SGP40_I2C_ADDR          (0x59 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim3_ch4_up;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/* ===== Types ============================================================== */
struct temps { uint8_t heure, min, sec; };

typedef struct {
    uint8_t hour;
    uint8_t minute;
} AlarmTime;

typedef enum { ALARM_IDLE = 0, ALARM_RINGING, ALARM_SNOOZE } AlarmState;

typedef struct { uint16_t freq; uint16_t duration; } Note;      /* Hz, ms */

typedef struct { uint16_t movementCount; uint8_t sleepQuality; } CAN_SleepData_t;
typedef struct { uint8_t  startFlag;     uint16_t delayTimeSec; } CAN_AlarmMsg_t;

/* ===== Clock & alarm ====================================================== */
struct temps horloge = {0, 0, 0};
AlarmTime    alarm_time = {0, 0};
bool         alarm_enabled = false;

volatile AlarmState alarm_state = ALARM_IDLE;
volatile TickType_t snoozeStart = 0;

/* ===== Sensors ============================================================ */
volatile float    fTemperature    = 0.0f;
volatile int32_t  voc_index_value = 0;
volatile uint16_t lum_percent     = 0;
volatile uint16_t lum_reel        = 0;

uint32_t adc_raw[2];                    /* Rank1: CH13 (photoresistor), Rank2: CH11 */

/* SPI2 temperature sensor */
uint8_t control_T[2]  = {0x80, 0x15};   /* one-shot + shutdown */
uint8_t Temp_Address  = 0x02;
uint8_t Temp[3];

/* I2C1 SGP40 */
uint8_t cmd_measure[] = {0x26, 0x0F, 0x80, 0x00, 0xA2, 0x66, 0x66, 0x93};
uint8_t voc_raw_data[3];
GasIndexAlgorithmParams voc_params;

/* ===== Fan ================================================================ */
volatile uint8_t fanForced = 0;         /* toggled by SW2 */

/* ===== RTOS objects ======================================================= */
SemaphoreHandle_t semaph;               /* binary : LPUART ISR  -> uart_task   */
SemaphoreHandle_t semaphM;              /* mutex  : horloge + alarm_time       */
SemaphoreHandle_t semaphI2c;            /* mutex  : I2C1 bus                   */
SemaphoreHandle_t semaphAlarm;          /* binary : AlarmTask   -> MelodyTask  */
SemaphoreHandle_t fanSemaph;            /* binary : airQ / SW2  -> MotorTask   */
QueueHandle_t     rxQueue;              /* CAN RX ISR -> CAN_ReceiverTask      */
QueueHandle_t     canQueue;             /* SW1 ISR    -> CAN_SendTask          */

/* ===== BLE / console ====================================================== */
uint8_t           RxChar;
char              rxBuffer[32];
volatile uint8_t  input_index  = 0;
volatile uint8_t  command_mode = 0;     /* 0 idle | 1 time | 2 busy | 3 alarm | 4 sound */
volatile uint8_t  flag         = 0;

/* ===== LEDs =============================================================== */
uint16_t PwmRC_Blue[WS2812_FRAME_LEN];  /* fade-in  (sunrise) */
uint16_t PwmRC_Red [WS2812_FRAME_LEN];  /* fade-out (sunset)  */

TaskHandle_t fadeInBlueTaskHandle  = NULL;
TaskHandle_t fadeOutRedTaskHandle  = NULL;

volatile bool stopFadeIn      = false;
volatile bool startFadeOutRed = false;

/* ===== ToF ================================================================ */
VL53L1_Dev_t tof[2];
const uint8_t tofAddr[2] = { 0x54, 0x56 };

/* ===== CAN ================================================================ */
volatile uint16_t   movement = 0;
volatile uint8_t    quality  = 0;
CAN_TxHeaderTypeDef TxHeader;
uint8_t             TxData[3];
uint32_t            TxMailbox;

/* ===== Buzzer ============================================================= */
/* Note frequencies (Hz) */
#define G3   196
#define GS3  208
#define A3   220
#define B3   247
#define C4   262
#define CS4  277
#define D4   294
#define DS4  311
#define E4   330
#define F4   349
#define FS4  370
#define G4   392
#define GS4  415
#define A4   440
#define AS4  466
#define B4   494
#define C5   523
#define CS5  554
#define D5   587
#define DS5  622
#define E5   659
#define F5   698
#define FS5  740
#define G5   784
#define GS5  831
#define A5   880
#define B5   988
#define C6   1047
#define R    0                          /* rest */

const Note melody_star_wars[] = {       /* 0: Imperial March */
    {A4, 500}, {A4, 500}, {A4, 500}, {F4, 350}, {C5, 150},
    {A4, 500}, {F4, 350}, {C5, 150}, {A4, 650}, {R,  200},
    {E5, 500}, {E5, 500}, {E5, 500}, {F5, 350}, {C5, 150},
    {GS4,500}, {F4, 350}, {C5, 150}, {A4, 650}, {R,  300}
};

const Note melody_tetris[] = {          /* 1: Korobeiniki */
    {E5, 400}, {B4, 200}, {C5, 200}, {D5, 400}, {C5, 200}, {B4, 200},
    {A4, 400}, {A4, 200}, {C5, 200}, {E5, 400}, {D5, 200}, {C5, 200},
    {B4, 400}, {B4, 200}, {C5, 200}, {D5, 400}, {E5, 400},
    {C5, 400}, {A4, 400}, {A4, 400}, {R,  300}
};

const Note melody_nokia[] = {           /* 2: Nokia */
    {E5, 160}, {D5, 160}, {FS4,320}, {GS4,320},
    {CS5,160}, {B4, 160}, {D4, 320}, {E4, 320},
    {B4, 160}, {A4, 160}, {CS4,320}, {E4, 320},
    {A4, 600}, {R,  300}
};

const Note melody_zelda[] = {           /* 3: Zelda secret */
    {G4, 130}, {FS4,130}, {DS4,130}, {A3, 130},
    {GS3,130}, {E4, 130}, {GS4,130}, {C5, 400},
    {R,  300}
};

const Note *const sounds[N_SOUNDS] = {
    melody_star_wars, melody_tetris, melody_nokia, melody_zelda
};

const uint8_t sound_lengths[N_SOUNDS] = {
    sizeof(melody_star_wars) / sizeof(melody_star_wars[0]),
    sizeof(melody_tetris)    / sizeof(melody_tetris[0]),
    sizeof(melody_nokia)     / sizeof(melody_nokia[0]),
    sizeof(melody_zelda)     / sizeof(melody_zelda[0])
};

uint8_t sound = 0;                      /* selected melody */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM15_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void chrono           (void *pvParameters);
void uart_task        (void *pvParameters);
void temp_task        (void *pvParameters);
void airQ_task        (void *pvParameters);
void AlarmTask        (void *pvParameters);
void MelodyTask       (void *pvParameters);
void GestureDetectTask(void *pvParameters);
void MotorTask        (void *pvParameters);
void FadeInBlueTask   (void *pvParameters);
void FadeOutRedTask   (void *pvParameters);
void CAN_SendTask     (void *pvParameters);
void CAN_ReceiverTask (void *pvParameters);
void CAN_SetupFiltersAndStart(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&COM_HANDLE);
  SSD1306_Init();
  GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, adc_raw, 2);
  HAL_TIM_Base_Start(&htim15);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  semaphM   = xSemaphoreCreateMutex();
  semaphI2c = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  semaph      = xSemaphoreCreateBinary();
  semaphAlarm = xSemaphoreCreateBinary();
  fanSemaph   = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  rxQueue  = xQueueCreate(5,  sizeof(CAN_SleepData_t));
  canQueue = xQueueCreate(10, sizeof(CAN_AlarmMsg_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  	  xTaskCreate(chrono,            "CHRONO",     512, NULL, tskIDLE_PRIORITY + 4, NULL);
      xTaskCreate(uart_task,         "UART",       512, NULL, tskIDLE_PRIORITY + 4, NULL);
      xTaskCreate(temp_task,         "TEMP",       256, NULL, tskIDLE_PRIORITY + 1, NULL);
      xTaskCreate(airQ_task,         "AIR_Q",      512, NULL, tskIDLE_PRIORITY + 1, NULL);
      xTaskCreate(AlarmTask,         "Alarm",      256, NULL, tskIDLE_PRIORITY + 2, NULL);
      xTaskCreate(MelodyTask,        "Melody",     256, NULL, tskIDLE_PRIORITY + 1, NULL);
      xTaskCreate(GestureDetectTask, "Gesture",   1024, NULL, tskIDLE_PRIORITY + 2, NULL);
      xTaskCreate(MotorTask,         "Motor",      256, NULL, tskIDLE_PRIORITY + 2, NULL);
      xTaskCreate(FadeInBlueTask,    "FadeInBlue", 512, NULL, tskIDLE_PRIORITY + 2, &fadeInBlueTaskHandle);
      xTaskCreate(FadeOutRedTask,    "FadeOutRed", 512, NULL, tskIDLE_PRIORITY + 2, &fadeOutRedTaskHandle);
      xTaskCreate(CAN_SendTask,      "CAN_TX",     256, NULL, tskIDLE_PRIORITY,     NULL);
      xTaskCreate(CAN_ReceiverTask,  "CAN_RX",     256, NULL, tskIDLE_PRIORITY,     NULL);

      CAN_SetupFiltersAndStart();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 79;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 20000-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 20-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOH, SHUT1_Pin|SHUT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIS_Pin|CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUT1_Pin SHUT2_Pin */
  GPIO_InitStruct.Pin = SHUT1_Pin|SHUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIS_Pin CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = DIS_Pin|CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ============================================================================
 *  1. DISPLAY / CLOCK
 * ========================================================================== */
void chrono(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint8_t retro = 1;                       /* OLED colour mode */
    char line1[20], line2[22], line3[20], line4[20];

    for (;;)
    {
        uint8_t cur_h, cur_m, cur_s, alm_h, alm_m;

        /* -- clock tick -- */
        xSemaphoreTake(semaphM, portMAX_DELAY);
        if (++horloge.sec >= 60) {
            horloge.sec = 0;
            if (++horloge.min >= 60) {
                horloge.min = 0;
                if (++horloge.heure >= 24) horloge.heure = 0;
            }
        }
        cur_h = horloge.heure;  cur_m = horloge.min;  cur_s = horloge.sec;
        alm_h = alarm_time.hour; alm_m = alarm_time.minute;
        xSemaphoreGive(semaphM);

        /* -- ambient light with hysteresis -- */
        lum_percent = (uint16_t)((adc_raw[0] * 100U) / 4095U);
        lum_reel    = 100U - lum_percent;

        if      (lum_percent < LUM_DARK_THRESHOLD)   retro = 1;
        else if (lum_percent > LUM_BRIGHT_THRESHOLD) retro = 0;

        /* -- format -- */
        snprintf(line1, sizeof(line1), "%02d:%02d:%02d      ", cur_h, cur_m, cur_s);
        snprintf(line2, sizeof(line2), "A:%02d:%02d  LUM:%2d%% ", alm_h, alm_m, lum_reel);
        snprintf(line3, sizeof(line3), "TEMP: %.1f C     ", fTemperature);   /* needs -u _printf_float */
        snprintf(line4, sizeof(line4), "AIR : VOC %-4ld  ", (long)voc_index_value);

        /* -- render -- */
        xSemaphoreTake(semaphI2c, portMAX_DELAY);
        SSD1306_GotoXY(0,  0); SSD1306_Puts(line1, &Font_11x18, retro);
        SSD1306_GotoXY(0, 21); SSD1306_Puts(line2, &Font_7x10,  retro);
        SSD1306_GotoXY(0, 38); SSD1306_Puts(line3, &Font_7x10,  retro);
        SSD1306_GotoXY(0, 52); SSD1306_Puts(line4, &Font_7x10,  retro);
        SSD1306_UpdateScreen();
        xSemaphoreGive(semaphI2c);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}


/* ============================================================================
 *  2. BLE / UART CONSOLE
 *     t = set time | a = set alarm | s = set sound | q = sleep quality
 * ========================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t woken = pdFALSE;

    if (huart->Instance == COM_INSTANCE)
    {
        if (command_mode == 0)                              /* waiting for a command letter */
        {
            switch (RxChar) {
            case 't': case 'T': flag = 1; input_index = 0; xSemaphoreGiveFromISR(semaph, &woken); break;
            case 'a': case 'A': flag = 3; input_index = 0; xSemaphoreGiveFromISR(semaph, &woken); break;
            case 's': case 'S': flag = 5; input_index = 0; xSemaphoreGiveFromISR(semaph, &woken); break;
            case 'q': case 'Q': flag = 7;                  xSemaphoreGiveFromISR(semaph, &woken); break;
            default: break;
            }
        }
        else if (command_mode == 1 || command_mode == 3 || command_mode == 4)   /* typing a value */
        {
            if (RxChar == '\r' || RxChar == '\n')
            {
                if (input_index > 0) {
                    flag = (command_mode == 1) ? 2 : (command_mode == 3) ? 4 : 6;
                    command_mode = 2;                       /* ignore the trailing CR/LF */
                    xSemaphoreGiveFromISR(semaph, &woken);
                }
            }
            else if (input_index < sizeof(rxBuffer) - 1)
            {
                rxBuffer[input_index++] = RxChar;
                HAL_UART_Transmit(&COM_HANDLE, &RxChar, 1, 10);     /* echo */
            }
        }

        HAL_UART_Receive_IT(&COM_HANDLE, &RxChar, 1);       /* always re-arm */
        portYIELD_FROM_ISR(woken);
    }
}

void uart_task(void *pvParameters)
{
    HAL_UART_Receive_IT(&COM_HANDLE, &RxChar, 1);
    printf("\r\nReveil pret. t = heure, a = alarme, s = son, q = sommeil\r\n");

    for (;;)
    {
        xSemaphoreTake(semaph, portMAX_DELAY);

        switch (flag)
        {
        /* ---- set current time ---- */
        case 1:
            input_index = 0;
            command_mode = 1;
            printf("\r\nHeure (HH MM SS) :\r\n");
            break;

        case 2: {
            int h, m, s;
            rxBuffer[input_index] = '\0';
            if (sscanf(rxBuffer, "%d %d %d", &h, &m, &s) == 3 &&
                h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60)
            {
                xSemaphoreTake(semaphM, portMAX_DELAY);
                horloge.heure = h;  horloge.min = m;  horloge.sec = s;
                xSemaphoreGive(semaphM);
                printf("ACK %02d:%02d:%02d\r\n", h, m, s);
            }
            else printf("NACK\r\n");
            input_index = 0;  command_mode = 0;
            break;
        }

        /* ---- set alarm ---- */
        case 3:
            input_index = 0;
            command_mode = 3;
            printf("\r\nAlarme (HH MM) :\r\n");
            break;

        case 4: {
            int h, m;
            rxBuffer[input_index] = '\0';
            if (sscanf(rxBuffer, "%d %d", &h, &m) == 2 &&
                h >= 0 && h < 24 && m >= 0 && m < 60)
            {
                xSemaphoreTake(semaphM, portMAX_DELAY);
                alarm_time.hour   = (uint8_t)h;
                alarm_time.minute = (uint8_t)m;
                alarm_enabled     = true;
                xSemaphoreGive(semaphM);
                printf("ACK ALARME %02d:%02d\r\n", h, m);
            }
            else printf("NACK ALARME\r\n");
            input_index = 0;  command_mode = 0;
            break;
        }

        /* ---- set sound ---- */
        case 5:
            input_index = 0;
            command_mode = 4;
            printf("\r\nSon: 0 Imperial March, 1 Tetris, 2 Nokia, 3 Zelda\r\nNumero :\r\n");
            break;

        case 6: {
            int s;
            rxBuffer[input_index] = '\0';
            if (sscanf(rxBuffer, "%d", &s) == 1 && s >= 0 && s < (int)N_SOUNDS) {
                sound = (uint8_t)s;
                printf("ACK SON %d\r\n", s);
            }
            else printf("NACK SON\r\n");
            input_index = 0;  command_mode = 0;
            break;
        }

        /* ---- sleep quality ---- */
        case 7:
            printf("Mouvements = %u\r\nSommeil = %s\r\n", movement, quality == 1 ? "Bon" : "Mauvais");
            break;

        default: break;
        }
    }
}


/* ============================================================================
 *  3. SENSORS
 * ========================================================================== */
void temp_task(void *pvParameters)                          /* SPI2, every 1 s */
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* start one-shot conversion */
        HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CE_GPIO_Port,  CE_Pin,  GPIO_PIN_SET);
        HAL_SPI_Transmit(&hspi2, control_T, sizeof(control_T), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(CE_GPIO_Port,  CE_Pin,  GPIO_PIN_RESET);

        vTaskDelay(pdMS_TO_TICKS(150));                     /* conversion time */

        /* read result */
        HAL_GPIO_WritePin(CE_GPIO_Port,  CE_Pin,  GPIO_PIN_SET);
        HAL_SPI_Transmit(&hspi2, &Temp_Address, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive (&hspi2, Temp, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(CE_GPIO_Port,  CE_Pin,  GPIO_PIN_RESET);

        uint8_t msb = Temp[0], lsb = Temp[1];
        if      (lsb & 0x80) fTemperature = (float)msb + 0.50f;
        else if (lsb & 0x40) fTemperature = (float)msb + 0.25f;
        else                 fTemperature = (float)msb;

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void airQ_task(void *pvParameters)                          /* I2C1 SGP40, every 1 s */
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        xSemaphoreTake(semaphI2c, portMAX_DELAY);
        HAL_I2C_Master_Transmit(&hi2c1, SGP40_I2C_ADDR, cmd_measure, sizeof(cmd_measure), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(35));
        HAL_I2C_Master_Receive (&hi2c1, SGP40_I2C_ADDR, voc_raw_data, sizeof(voc_raw_data), HAL_MAX_DELAY);
        xSemaphoreGive(semaphI2c);

        /* NOTE: SGP40 words are MSB-first -> (raw[0] << 8) | raw[1]. Your code is LSB-first; verify. */
        int32_t sraw    = (int32_t)(voc_raw_data[0] | (voc_raw_data[1] << 8));
        int32_t voc_val = 0;
        GasIndexAlgorithm_process(&voc_params, sraw, &voc_val);
        voc_index_value = voc_val;

        xSemaphoreGive(fanSemaph);                          /* wake MotorTask */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}


/* ============================================================================
 *  4. ALARM STATE MACHINE        IDLE -> RINGING <-> SNOOZE -> IDLE
 * ========================================================================== */
void AlarmTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    bool armed = true;                                      /* one trigger per matching minute */

    for (;;)
    {
        xSemaphoreTake(semaphM, portMAX_DELAY);
        bool match = alarm_enabled &&
                     horloge.heure == alarm_time.hour &&
                     horloge.min   == alarm_time.minute;
        xSemaphoreGive(semaphM);

        if (!match) armed = true;

        switch (alarm_state)
        {
        case ALARM_IDLE:
            if (match && armed)
            {
                armed = false;
                stopFadeIn      = false;
                startFadeOutRed = false;
                alarm_state     = ALARM_RINGING;
                xSemaphoreGive(semaphAlarm);                /* start melody */
                if (fadeInBlueTaskHandle) xTaskNotifyGive(fadeInBlueTaskHandle);
            }
            break;

        case ALARM_SNOOZE:
            if ((xTaskGetTickCount() - snoozeStart) >= pdMS_TO_TICKS(SNOOZE_MS))
            {
                alarm_state = ALARM_RINGING;
                xSemaphoreGive(semaphAlarm);
            }
            break;

        default: break;
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(250));
    }
}


/* ============================================================================
 *  5. BUZZER (TIM8 CH4, timer clock = 80 MHz / (PSC+1) = 1 MHz)
 * ========================================================================== */
static void Buzzer_Off(void) { HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4); }

/* Interruptible delay: false as soon as the alarm leaves RINGING. */
static bool Buzzer_Wait(uint32_t ms)
{
    while (ms)
    {
        uint32_t slice = (ms > 20) ? 20 : ms;
        vTaskDelay(pdMS_TO_TICKS(slice));
        ms -= slice;
        if (alarm_state != ALARM_RINGING) return false;
    }
    return true;
}

static bool Buzzer_PlayNote(uint32_t freq, uint32_t duration_ms)
{
    if (freq == 0) { Buzzer_Off(); return Buzzer_Wait(duration_ms); }

    uint32_t tim_clk = 80000000UL / (htim8.Init.Prescaler + 1);
    uint32_t period  = (tim_clk / freq) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim8, period);
    __HAL_TIM_SET_COMPARE   (&htim8, TIM_CHANNEL_4, period / 2);
    __HAL_TIM_SET_COUNTER   (&htim8, 0);                    /* [FIX] avoid a 65 ms glitch if CNT > new ARR */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    bool ok = Buzzer_Wait(duration_ms);
    Buzzer_Off();
    return ok;
}

static void Buzzer_PlayMelody(void)
{
    uint8_t s = (sound < N_SOUNDS) ? sound : 0;

    for (uint8_t i = 0; i < sound_lengths[s]; i++)
    {
        if (!Buzzer_PlayNote(sounds[s][i].freq, sounds[s][i].duration)) return;
        if (!Buzzer_Wait(50)) return;                       /* gap between notes */
    }
}

void MelodyTask(void *pvParameters)
{
    for (;;)
    {
        xSemaphoreTake(semaphAlarm, portMAX_DELAY);
        while (alarm_state == ALARM_RINGING) Buzzer_PlayMelody();
        Buzzer_Off();
    }
}


/* ============================================================================
 *  6. ToF GESTURES (VL53L1, shared I2C1)
 *     wave once  : snooze   | wave twice within 2 s : stop
 * ========================================================================== */
static void TOF_InitBoth(void)
{
    GPIO_TypeDef *port[2] = { SHUT1_GPIO_Port, SHUT2_GPIO_Port };
    uint16_t      pin [2] = { SHUT1_Pin,       SHUT2_Pin       };

    /* 1. both in reset */
    for (int i = 0; i < 2; i++) HAL_GPIO_WritePin(port[i], pin[i], GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 2. wake one sensor at a time, then move it off 0x52 */
    for (int i = 0; i < 2; i++)
    {
        HAL_GPIO_WritePin(port[i], pin[i], GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(5));

        tof[i].comms_speed_khz = 400;
        tof[i].I2cHandle       = &hi2c1;
        tof[i].comms_type      = 1;
        tof[i].I2cDevAddr      = 0x52;

        xSemaphoreTake(semaphI2c, portMAX_DELAY);
        VL53L1_SetDeviceAddress(&tof[i], tofAddr[i]);
        tof[i].I2cDevAddr = tofAddr[i];
        VL53L1_WaitDeviceBooted(&tof[i]);
        VL53L1_DataInit(&tof[i]);
        VL53L1_StaticInit(&tof[i]);
        VL53L1_SetDistanceMode(&tof[i], VL53L1_DISTANCEMODE_LONG);
        VL53L1_SetMeasurementTimingBudgetMicroSeconds(&tof[i], 40000);
        VL53L1_SetInterMeasurementPeriodMilliSeconds(&tof[i], 50);
        VL53L1_ClearInterruptAndStartMeasurement(&tof[i]);
        xSemaphoreGive(semaphI2c);
    }
}

/* ---- Is a hand in front of sensor i? ---- */
static bool TOF_HandPresent(uint8_t i)
{
    VL53L1_RangingMeasurementData_t m;

    xSemaphoreTake(semaphI2c, portMAX_DELAY);
    uint8_t st = VL53L1_GetRangingMeasurementData(&tof[i], &m);
    if (st == 0) VL53L1_ClearInterruptAndStartMeasurement(&tof[i]);
    xSemaphoreGive(semaphI2c);

    return (st == 0) && (m.RangeMilliMeter < HAND_THRESHOLD_MM);
}

/* ---- Gestures:  swipe ToF1 -> ToF2 = snooze | hold 3 s on either = stop ---- */
void GestureDetectTask(void *pvParameters)
{
    TickType_t xLastWake = xTaskGetTickCount();
    TickType_t longAgo   = xLastWake - pdMS_TO_TICKS(60000);

    TickType_t lastSeen[2]  = { longAgo, longAgo };   /* last sample with a hand */
    TickType_t holdStart[2] = { longAgo, longAgo };
    TickType_t lastSwipe    = longAgo;
    bool       prev[2]      = { false, false };

    TOF_InitBoth();
    vTaskDelay(pdMS_TO_TICKS(200));

    for (;;)
    {
        bool hand[2] = { TOF_HandPresent(0), TOF_HandPresent(1) };
        TickType_t now = xTaskGetTickCount();

        /* ---- swipe: ToF2 sees a hand just after ToF1 did ---- */
        if (hand[1] && !prev[1] &&
            (now - lastSeen[0])  <= pdMS_TO_TICKS(SWIPE_WINDOW_MS) &&
            (now - lastSwipe)    >= pdMS_TO_TICKS(SWIPE_LOCKOUT_MS))
        {
            lastSwipe = now;
            if (alarm_state == ALARM_RINGING)
            {
                alarm_state = ALARM_SNOOZE;
                snoozeStart = now;
                Buzzer_Off();
            }
        }

        /* ---- hold: continuous presence on one sensor ---- */
        for (int i = 0; i < 2; i++)
        {
            if (!hand[i]) continue;

            if ((now - lastSeen[i]) > pdMS_TO_TICKS(HOLD_GRACE_MS))
                holdStart[i] = now;                          /* new hold begins */

            if ((now - holdStart[i]) >= pdMS_TO_TICKS(HOLD_STOP_MS) &&
                alarm_state != ALARM_IDLE)
            {
                alarm_state     = ALARM_IDLE;
                stopFadeIn      = true;
                startFadeOutRed = false;
                Buzzer_Off();
            }
        }

        /* ---- bookkeeping (after the checks, so they see the previous sample) ---- */
        for (int i = 0; i < 2; i++)
        {
            if (hand[i]) lastSeen[i] = now;
            prev[i] = hand[i];
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}


/* ============================================================================
 *  7. FAN + BUTTONS
 *     SW2 : toggle forced fan | SW1 : night mode (red fade-out + CAN message)
 * ========================================================================== */
void MotorTask(void *pvParameters)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

    for (;;)
    {
        xSemaphoreTake(fanSemaph, portMAX_DELAY);
        bool on = (voc_index_value > 200) || fanForced;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, on ? 200 : 0);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static TickType_t lastTick = 0;
    BaseType_t woken = pdFALSE;

    TickType_t now = xTaskGetTickCountFromISR();
    if ((now - lastTick) < pdMS_TO_TICKS(200)) return;     /* debounce */
    lastTick = now;

    if (GPIO_Pin == SW2_Pin)
    {
        fanForced ^= 1;
        xSemaphoreGiveFromISR(fanSemaph, &woken);
    }
    else if (GPIO_Pin == SW1_Pin)
    {
        startFadeOutRed = true;

        uint32_t nowS = horloge.heure * 3600UL + horloge.min * 60UL + horloge.sec;
        uint32_t almS = alarm_time.hour * 3600UL + alarm_time.minute * 60UL;
        uint32_t d    = (almS > nowS) ? (almS - nowS) : (almS + 86400UL - nowS);   /* midnight wrap */
        if (d > 0xFFFF) d = 0xFFFF;                         /* 16-bit field: clamp (~18 h) */

        CAN_AlarmMsg_t msg = { .startFlag = 1, .delayTimeSec = (uint16_t)d };
        xQueueSendFromISR(canQueue, &msg, &woken);

        if (fadeOutRedTaskHandle) vTaskNotifyGiveFromISR(fadeOutRedTaskHandle, &woken);
    }
    portYIELD_FROM_ISR(woken);
}


/* ============================================================================
 *  8. CAN   (TX 0x11: [flag][delay_lo][delay_hi]   RX: [mov_hi][mov_lo][quality])
 * ========================================================================== */
void CAN_SetupFiltersAndStart(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank           = 0;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;                            /* accept everything */

    HAL_CAN_ConfigFilter(&hcan1, &f);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_SendTask(void *pvParameters)
{
    CAN_AlarmMsg_t msg;

    TxHeader.DLC   = 3;
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.StdId = 0x11;
    TxHeader.TransmitGlobalTime = DISABLE;

    for (;;)
    {
        if (xQueueReceive(canQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            TxData[0] = msg.startFlag;
            TxData[1] = msg.delayTimeSec & 0xFF;
            TxData[2] = msg.delayTimeSec >> 8;
            HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t    d[8];                                        /* full frame size: no overflow */
    BaseType_t woken = pdFALSE;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) == HAL_OK && hdr.DLC >= 3)
    {
        CAN_SleepData_t r = { (uint16_t)(((uint16_t)d[0] << 8) | d[1]), d[2] };
        xQueueSendFromISR(rxQueue, &r, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void CAN_ReceiverTask(void *pvParameters)
{
    CAN_SleepData_t r;

    for (;;)
    {
        if (xQueueReceive(rxQueue, &r, portMAX_DELAY) == pdPASS)
        {
            movement = r.movementCount;
            quality  = r.sleepQuality;
        }
    }
}


/* ============================================================================
 *  9. WS2812 LEDs (TIM3 CH4 + DMA)  - frame order per LED is G, R, B
 * ========================================================================== */
static void WS2812_Fill(uint16_t *buf, uint8_t g, uint8_t r, uint8_t b)
{
    const uint8_t grb[3] = { g, r, b };
    uint32_t k = 0;

    for (uint32_t led = 0; led < N_LEDS; led++)
        for (int c = 0; c < 3; c++)
            for (int bit = 7; bit >= 0; bit--)
                buf[k++] = ((grb[c] >> bit) & 1) ? WS2812_BIT_1 : WS2812_BIT_0;

    while (k < WS2812_FRAME_LEN) buf[k++] = 0;              /* reset slots */
}

static void WS2812_Show(uint16_t *frame, uint32_t hold_ms)
{
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)frame, WS2812_FRAME_LEN);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
}

/* Sunrise: 0 -> 255 blue over 60 s, started by AlarmTask */
void FadeInBlueTask(void *pvParameters)
{
    const int steps     = 255;
    const int stepDelay = 60000 / steps;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        stopFadeIn = false;

        for (int level = 0; level <= steps; level++)
        {
            if (alarm_state == ALARM_IDLE || stopFadeIn) break;
            WS2812_Fill(PwmRC_Blue, 0, 0, (uint8_t)level);
            WS2812_Show(PwmRC_Blue, stepDelay);
        }

        /* [FIX] ALWAYS switch the LEDs off. Your guard `if (!stopFadeIn && ...)`
         *       left the strip lit after a gesture-stop (WS2812 keeps last colour). */
        WS2812_Fill(PwmRC_Blue, 0, 0, 0);
        WS2812_Show(PwmRC_Blue, 1000);
    }
}

/* Sunset: 255 -> 0 red over 60 s, started by SW1 */
void FadeOutRedTask(void *pvParameters)
{
    const int steps     = 255;
    const int stepDelay = 60000 / steps;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (int level = steps; level >= 0; level--)
        {
            if (!startFadeOutRed) break;
            WS2812_Fill(PwmRC_Red, 0, (uint8_t)level, 0);
            WS2812_Show(PwmRC_Red, stepDelay);
        }
        startFadeOutRed = false;
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
