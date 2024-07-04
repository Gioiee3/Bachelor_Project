/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "common.h"
#include "sx1278.h"
#include "string.h"
#include "modbus_crc.h"
#include "flash.h"
#include "aht20.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct LoRa_Link_Struct
{
  uint16_t Node_ID;
  uint16_t Node_Status;
  uint16_t Node_Battery_Voltage;
  uint16_t Node_Period;
} Link_Struct_t;

typedef struct LoRa_Data_Struct
{
  Link_Struct_t Link;
  uint16_t Node_LM35_Temperature;
  uint16_t Node_AHT_Temperature;
  uint16_t Node_AHT_Humidity;
  uint16_t Node_wind_speed;
  uint16_t Node_wind_direction;
  uint16_t Node_wind_level;
} Data_Struct_t;

typedef struct LoRa_Packet_ID_0
{
  uint16_t Packet_ID;
  Link_Struct_t Payload;
} Link_Packet_t;

typedef struct LoRa_Packet_ID_1
{
  uint16_t Packet_ID;
  Data_Struct_t Payload;
} Data_Packet_t;

typedef struct LoRa_Packet_ID_2
{
  uint16_t Packet_ID;
  uint16_t Target_Node_ID;
  uint16_t Target_Node_Status;
  uint16_t Target_Node_Period;
  uint16_t Target_Node_Response;
} ResponsePacket_t;

typedef struct WindData
{
  uint16_t rawTemperature;
  float Temperature;

  uint16_t RawSpeed;
	float Speed;
  uint16_t Direction;

  uint16_t windLevel;
} WindData_t;

typedef struct DHT22
{
  float Temperature;
  float Humidity;
  float Fahrenheit;
} DHT22_t;

typedef struct AHT20
{
  float Temperature;
  float Humidity;
} AHT20_t;

typedef enum NodeStatus
{
  WAKEUP_MODE = 1U,
  LINK_MODE = 2U,
  NORMAL_MODE = 4U,
  RETRY_MODE = 8U,
  SHUTDOWN_MODE = 16U
} NodeStatus_t;

// typedef enum NodeStatusMesh
// {
//   LINK_MODE_MESH = 3U,
//   NORMAL_MODE_MESH = 5U,
//   RETRY_MODE_MESH = 7U
// } NodeStatusMesh_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// #define NODE_ID     (0xCAAFU)
// #define NODE_ID     (0xCAAEU)
// #define NODE_ID     (0xCAADU)
#define NODE_ID     (0xCAACU)

#define TIMEOUT_MS                  (50U)
#define TIMEOUT_NODE                (12000)
#define PERIOD_RX_MS                (5000)
#define LINK_PACKET_ID              (0xBBBBU)
#define DATA_PACKET_ID              (0x4A4AU)
#define RESPONSE_PACKET_ID          (0x4444U)

#define PACKET_ID_0_MESH   (0xBBBBU)
#define PACKET_ID_1_MESH   (0x4A4AU)
#define PACKET_ID_2_MESH   (0x4444U)

#define SLEEPTIME_NORMAL_MODE_MIN   (5U)
#define SLEEPTIME_LINK_MODE_MIN     (1U)
#define SLEEPTIME_RETRY_MODE_SEC    (15U)

#define READ_SENSORS_DURATION_MS    (4500U)
#define DEFAULT_PERIOD_SEC          (300U)

#define WAIT_RESPONSE_DURATION_MS   (2000U)

#define NODE_WAKEUP_BIT_SHIFT       (0U)
#define NODE_WAKEUP_BIT             (1U << NODE_WAKEUP_BIT_SHIFT)
#define NODE_LINK_MODE_BIT_SHIFT    (1U)
#define NODE_LINK_MODE_BIT          (1U << NODE_LINK_MODE_BIT_SHIFT)
#define NODE_NORMAL_MODE_BIT_SHIFT  (2U)
#define NODE_NORMAL_MODE_BIT        (1U << NODE_NORMAL_MODE_BIT_SHIFT)
#define NODE_RETRY_MODE_BIT_SHIFT   (3U)
#define NODE_RETRY_MODE_BIT         (1U << NODE_RETRY_MODE_BIT_SHIFT)

#define NODE_SHUTDOWN_MODE_BIT_SHIFT   (4U)
#define NODE_SHUTDOWN_MODE_BIT         (1U << NODE_RETRY_MODE_BIT_SHIFT)

#define LINK_ACCEPT   (0xAAU)
#define LINK_REJECT   (0x55U)

#define LINK_CARRYON  (0x00U)
#define LINK_DISMISS  (0xFFU)

#define LINK_ACK      (0xABU)
#define LINK_NACK     (0xBAU)

#define AHT10_ADDRESS_1 (0x38 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]
#define AHT10_ADDRESS_2 (0x39 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]

#define SAVE_ADDR     (0x08013030U)

#define SPEED_ADDRESS 		0x01
#define DIRECTION_ADDRESS 0x02

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

void read_ADC(void);
void read_wind_sensor(void);
void read_DHT22(void);
void read_AHT20(void);

void goingDark(void);

void Link_Mode_Handle(void);
void Normal_Mode_Handle(void);
void Retry_Mode_Handle(void);

void Response_Handle(void);
void LoRa_Packet_Parser_Mesh(const uint8_t *data, uint32_t len);
void LoRa_Recv_Node(void);
bool Node_ID_inNetwork(uint16_t Node_ID);
void Normal_Mode_Mesh_Handle(Data_Packet_t Node);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const char *TAG = "Main";
uint16_t bat = 0, temp35 = 0, test_bat;
double test_bat_f, test_lm;
uint16_t adc[2];

static WindData_t myWind;
static DHT22_t myDHT;
static AHT20_t myAHT;
Data_Packet_t Node_Mesh[3] = {0};

static NodeStatus_t myStatus;
static uint32_t backupStorage = 0U;
static uint8_t payload[100] = {0};
static uint8_t payload_node[100] = {0};
static volatile bool is_LoRa_EXTI = false;
static volatile bool is_All_Power_OFF = true;
static uint8_t array2store[32] = {0};
static volatile uint8_t nTry = 0;
static volatile Flag_Node_E = 0, Flag_Node_D = 0, Flag_Node_C = 0; 

char Log[100];
uint8_t uart1_tx_data[8];
uint8_t uart1_tx_set_add[8];
uint8_t uart1_rx_data[15];
uint16_t recv_data[2];
uint8_t a[20];
uint16_t test_speed, test_level, test_di, test_dih;
float test_s;

uint8_t hum1, hum2, tempC1, tempC2, SUM, CHECK;
uint32_t pMillis, cMillis;
float temp_Celsius = 0;
float temp_Fahrenheit = 0;
float humidity = 0;
uint8_t hum_integral, hum_decimal, tempC_integral, tempC_decimal, tempF_integral, tempF_decimal;
char string[15];
uint8_t ver1;
float t, h;

uint32_t pre_time = 0, cur_time = 0;

void modbus_tx_data(uint8_t address)
{
	uart1_tx_data[0] = address; // slave address
	uart1_tx_data[1] = 0x03; // Function code for Read Input Registers (0x03)

	// where we want to start reading
	// The Register address will be 00000000 00000000 (40001)
	uart1_tx_data[2] = 0x00;
	uart1_tx_data[3] = 0x00;

	// this 2 byte indicate how many 16-bit register we want to read.
	// we will read 2, 16-bit data register from_
	// 00000000 00000000 to 00000000 00000001 (from 40001 to 40002 according to salve datasheet)
	uart1_tx_data[4] = 0x00;
	uart1_tx_data[5] = 0x02; //Sensor wind speed is 0x01, direction is 0x02

	// CRC Check function
	uint16_t crc = crc16(uart1_tx_data, 6);
	uart1_tx_data[6] = crc & 0xFF; //CRC LOW
	uart1_tx_data[7] = (crc >> 8) & 0xFF; //CRC HIGH

	// set DE,RE pin HIGH to set max485 as transmitter mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	// sending the uart1_tx_data array
	HAL_UART_Transmit(&huart1, uart1_tx_data, 8, 1000);
	// set DE,RE pin LOW to set max485 as receiver mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
/*
void modbus_tx_data_direction(){
	uart1_tx_data[0] = 0x02; // slave address
	uart1_tx_data[1] = 0x03; // Function code for Read Input Registers (0x03)

	// this 2 byte indicate the first 16-bit register.
	// where we want to start reading
	//The Register address will be 00000000 00000000 (40001)
	uart1_tx_data[2] = 0x00;
	uart1_tx_data[3] = 0x00;

	// this 2 byte indicate how many 16-bit register we want to read.
	// we will read 2, 16-bit data register from_
	// 00000000 00000000 to 00000000 00000001 (from 40001 to 40002 according to salve datasheet)
	uart1_tx_data[4] = 0x00;
	uart1_tx_data[5] = 0x02; //Sensor wind speed is 0x01, direction is 0x02

	// CRC Check function
	uint16_t crc = crc16(uart1_tx_data, 6);
	uart1_tx_data[6] = crc & 0xFF; //CRC LOW
	uart1_tx_data[7] = (crc >> 8) & 0xFF; //CRC HIGH

	// set DE,RE pin HIGH to set max485 as transmitter mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	// sending the uart1_tx_data array
	HAL_UART_Transmit(&huart1, uart1_tx_data, 8, 1000);
	// set DE,RE pin LOW to set max485 as receiver mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
*/
void modbus_tx_set_address()
{
	uart1_tx_set_add[0] = 0x01; // dia chi hien tai cua wind sensor
	uart1_tx_set_add[1] = 0x06; // Function code for set address Registers (0x03)

	// this 2 byte indicate the first 16-bit register.
	//We want set add, register add will be 07D0
	uart1_tx_set_add[2] = 0x07;
	uart1_tx_set_add[3] = 0xD0;

	// this 2 byte indicate how many 16-bit register we want to read.
	uart1_tx_set_add[4] = 0x00;
	uart1_tx_set_add[5] = 0x02;// set add = 0x02

	// CRC Check function
	uint16_t crc = crc16(uart1_tx_set_add, 6);
	uart1_tx_set_add[6] = crc & 0xFF; //CRC LOW
	uart1_tx_set_add[7] = (crc >> 8) & 0xFF; //CRC HIGH

	// set DE,RE pin HIGH to set max485 as transmitter mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	// sending the uart1_tx_set_add
	HAL_UART_Transmit(&huart1, uart1_tx_set_add, 8, 1000);
	// set DE,RE pin LOW to set max485 as receiver mode
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
//  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	sx1278_init();
	HAL_Delay(100);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_data, 15); // delay 3.5s sau khi on moi read duoc
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // xanh

while(1)
{
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  myStatus = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);

	  if (myStatus == 0U)
	  {
	    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, SLEEPTIME_NORMAL_MODE_MIN * 60);
	    myStatus = LINK_MODE;
	    is_All_Power_OFF = false;
	  }

	  if ((myStatus & LINK_MODE) != 0U)
	  {
	    Link_Mode_Handle();
	  }
	  else if ((myStatus & NORMAL_MODE) != 0U)
	      {
			  Normal_Mode_Handle();
			  HAL_Delay(50);
			  if (Flag_Node_C)
			  {
				Normal_Mode_Mesh_Handle(Node_Mesh[0]);
				Flag_Node_C = 0;
			  }
			  if (Flag_Node_D)
			  {
				Normal_Mode_Mesh_Handle(Node_Mesh[1]);
				Flag_Node_D = 0;
			  }
			  if (Flag_Node_E)
			  {
				Normal_Mode_Mesh_Handle(Node_Mesh[2]);
				Flag_Node_E = 0;
			  }
	      }
	      else if ((myStatus & RETRY_MODE) != 0U)
	        {
//	          Retry_Mode_Handle();
		        Normal_Mode_Handle();
				HAL_Delay(50);
				if (Flag_Node_C)
				{
				  Normal_Mode_Mesh_Handle(Node_Mesh[0]);
				  Flag_Node_C = 0;
				}
				if (Flag_Node_D)
				{
				  Normal_Mode_Mesh_Handle(Node_Mesh[1]);
				  Flag_Node_D = 0;
				}
				if (Flag_Node_E)
				{
				  Normal_Mode_Mesh_Handle(Node_Mesh[2]);
				  Flag_Node_E = 0;
				}
	        }

	  if ((myStatus & SHUTDOWN_MODE) != 0U)
	  {
	    // MX_GPIO_Init();
	    MX_SPI1_Init();
	    MX_RTC_Init();
	    myStatus = WAKEUP_MODE;
	    while (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) != myStatus)  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, myStatus);
	    while (1);
	  }

	  while (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) != myStatus)  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, myStatus);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    LoRa_Recv_Node();
}
// goingDark();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  /*
	  	      if (sx1278_recv_data((uint8_t *)rx, &nByteRx, &rssi, &snr, false) == SX1278_OK)
	  	      {
	  	    	  HAL_UART_Transmit(&huart1,(uint8_t*) "rx:", sizeof("rx:"), 100);
	  	    	  HAL_UART_Transmit(&huart1, rx, sizeof(rx), 100);
	  	    	  printf("data: %d\n", nByteRx);
	  	    	  printf("rssi: %d\n", rssi);
	  	      }
	  */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	recv_data[0] = uart1_rx_data[3]<<8 | uart1_rx_data[4];
	recv_data[1] = uart1_rx_data[5]<<8 | uart1_rx_data[6];

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart1_rx_data, 15);
}

void read_ADC(void)
{
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc, 2);
		HAL_Delay(30);
		for(uint8_t i =0; i<15; i++)
		{
			bat += adc[0];
			temp35 += adc[1];
			HAL_Delay(1);
		}

		bat = bat/15;
		test_bat_f =(double)((double)bat * 3.3 * 4.9 / 4095.0) - 0.22656;
		temp35 = temp35 / 15;
    myWind.rawTemperature = temp35;
    myWind.Temperature = (float)((float)temp35 * 11.0 * 17.0 /4095.0);
	  test_lm = (float) myWind.Temperature;
		HAL_ADC_Stop_DMA(&hadc1);
}

void read_wind_sensor (void)
{
	HAL_GPIO_WritePin(PWR_SEN_GPIO_Port, PWR_SEN_Pin, 1);
  HAL_Delay(3500);

	modbus_tx_data(SPEED_ADDRESS);
  HAL_Delay(50);
  myWind.RawSpeed = recv_data[0];
//	test_speed = recv_data[0];
//	test_s  = (float)((float)test_speed/10.0);
  myWind.Speed = (float)((float)myWind.RawSpeed / 10.0);
  myWind.windLevel = recv_data[1];
//	test_level = recv_data[1];

  modbus_tx_data(DIRECTION_ADDRESS);
  HAL_Delay(50);
  myWind.Direction = recv_data[0];
//	test_di = recv_data[1];
//	test_dih = recv_data[0];

  HAL_GPIO_WritePin(PWR_SEN_GPIO_Port, PWR_SEN_Pin, 0);
}

void read_AHT20(void)
{
		MX_I2C1_Init();
    HAL_Delay(100);
		AHT20_init();
		AHT20_read(&t, &h);
    myAHT.Temperature = t;
    myAHT.Humidity = h;
		HAL_I2C_DeInit(&hi2c1);
}

void goingDark(void)
{
  /* Enable WUPIN PA0: HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1) */
  SET_BIT(PWR->CSR, PWR_CSR_EWUP);

  /* Set RTC Alarm for 300s */
  RTC_TimeTypeDef timeNow;
  RTC_AlarmTypeDef timeAlarm;
  HAL_RTC_GetTime(&hrtc, &timeNow, RTC_FORMAT_BCD);
  timeAlarm.Alarm = RTC_ALARM_A;
  timeAlarm.AlarmTime = timeNow; // luu thoi gian hien tai
  if (myStatus == NORMAL_MODE)
  {
    // timeAlarm.AlarmTime.Seconds += 5;
    timeAlarm.AlarmTime.Minutes += SLEEPTIME_NORMAL_MODE_MIN; // + thoi gian cua chu ky
	}
  else if (myStatus == LINK_MODE)
      {
        // timeAlarm.AlarmTime.Seconds += 5;
        timeAlarm.AlarmTime.Minutes += SLEEPTIME_LINK_MODE_MIN;
      }
      else if (myStatus == RETRY_MODE)
          {
            timeAlarm.AlarmTime.Seconds += SLEEPTIME_RETRY_MODE_SEC;
          }


  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
  HAL_RTC_SetAlarm_IT(&hrtc, &timeAlarm, RTC_FORMAT_BCD); // enable interrupt

  //STANDBY mode
  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Select Standby mode */
  SET_BIT(PWR->CR, PWR_CR_PDDS);

  /* Clear WUF bit in Power Control/Status register */
  SET_BIT(PWR->CR, PWR_CR_CWUF); /*!< Clear Wakeup Flag */

  /* Clear SBF bit in Power Control/Status register */
  SET_BIT(PWR->CR, PWR_CR_CSBF); /*!< Clear Standby Flag */

  (void)PWR->CR;
  (void)PWR->CSR;

  // Disable debug, trace and IWDG in low-power modes
  DBGMCU->CR = (uint32_t)0x00;

// #if DEBUG == 1
//   logPC("Bravo 6\nGoing Dark\n");
// #endif /* DEBUG == 1 */

  for (;;)
  {
    __DSB();
    __WFI(); // Wait for interrupt
  }
}
//kiem tra xem truyen thong duoc khong
bool is_OK_2_Talk(void)
{
  sx1278_standby();
  uint32_t checkTimeout = HAL_GetTick();
  uint8_t irq = sx1278_read_reg(REG_IRQ_FLAGS);
  sx1278_write_reg(REG_IRQ_FLAGS, irq);     //reset irq
  is_LoRa_EXTI = false;
  sx1278_cad();
  while ((is_LoRa_EXTI == false) && ((HAL_GetTick() - checkTimeout) < TIMEOUT_MS));
  if (is_LoRa_EXTI == false) return false;
  irq = sx1278_read_reg(REG_IRQ_FLAGS);
  sx1278_write_reg(REG_IRQ_FLAGS, irq);     //reset irq
  sx1278_standby();
  if (((irq & 0x01U) != 0U) && ((irq & 0x04U) != 0U))
  {
    return false;
  }
  else if (((irq & 0x01U) == 0U) && ((irq & 0x04U) != 0U))
  {
    return true;
  }
  return false;
}

void Link_Mode_Handle(void)
{
  read_ADC();
	HAL_Delay(10);
	read_ADC();
	HAL_Delay(10);
	read_ADC();

  MX_SPI1_Init();
  HAL_Delay(100);
  sx1278_init();

  Link_Packet_t LinkReq;
  LinkReq.Packet_ID = LINK_PACKET_ID;
  LinkReq.Payload.Node_ID = NODE_ID;
  LinkReq.Payload.Node_Status = myStatus;
  LinkReq.Payload.Node_Battery_Voltage = (uint16_t)(test_bat_f * 100.0); // 100 x Bat_voltage
  LinkReq.Payload.Node_Period = (uint16_t) (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3));
  memcpy((uint8_t*)&payload, (uint8_t*)&LinkReq, sizeof(Link_Packet_t));

  nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(110);
    // HAL_Delay(get_random_value(backupStorage, 50, 200));
  }

  if (nTry == 10)
  {
    myStatus = LINK_MODE;
    HAL_SPI_DeInit(&hspi1);
    return;
  }

  sx1278_send_data((uint8_t *)&payload, sizeof(Link_Packet_t));

  Response_Handle();

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);
}

void Normal_Mode_Handle(void)
{
  read_ADC();
	HAL_Delay(10);
	read_ADC();
	HAL_Delay(10);
	read_ADC();

  read_wind_sensor();
  read_AHT20();

  MX_SPI1_Init();
  HAL_Delay(100);
  sx1278_init();

  Data_Packet_t data_packet;
  data_packet.Packet_ID = DATA_PACKET_ID;
  data_packet.Payload.Link.Node_ID = NODE_ID;
  data_packet.Payload.Link.Node_Status = myStatus;
  data_packet.Payload.Link.Node_Battery_Voltage = (uint16_t)(test_bat_f * 100.0); // 100 x Bat_voltage
  data_packet.Payload.Link.Node_Period      = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3));

  data_packet.Payload.Node_LM35_Temperature = (uint16_t)(myWind.Temperature * 10); // 10 x temp_lm35
  data_packet.Payload.Node_AHT_Temperature  = (uint16_t)(myAHT.Temperature * 10);  // 10 x temp_aht
  data_packet.Payload.Node_AHT_Humidity     = (uint16_t)(myAHT.Humidity * 10);     // 10 x humid_aht
  data_packet.Payload.Node_wind_speed       = (uint16_t)(myWind.Speed * 10);       // 10 x wind speed
  data_packet.Payload.Node_wind_direction   = myWind.Direction;                    // direction type uint16
  data_packet.Payload.Node_wind_level       = myWind.windLevel;                    // direction type uint16
  memset(payload, '\0', sizeof(payload));
  memcpy((uint8_t *)payload, (uint8_t *)&data_packet, sizeof(Data_Packet_t));

  nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(150);
    // HAL_Delay(get_random_value(backupStorage, 50, 200));
  }
  if (nTry == 10)
  {
    myStatus = RETRY_MODE;
    HAL_SPI_DeInit(&hspi1);
    // HAL_GPIO_WritePin(PWR_SUB_GPIO_Port, PWR_SUB_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    return;
  }

  sx1278_send_data(payload, sizeof(Data_Packet_t));
  Response_Handle();

  if (myStatus == RETRY_MODE)
  {
    memset(array2store, '\0', sizeof(array2store));
    memcpy((uint8_t *)&array2store, (uint8_t *)&data_packet, sizeof(Data_Packet_t)); // save value measured
    Flash_Write_Data(SAVE_ADDR, (uint32_t*)&array2store, 8U);
    HAL_Delay(300);
  }

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);
}

void Retry_Mode_Handle(void)
{
  Flash_Read_Data(SAVE_ADDR, (uint32_t *)&array2store, sizeof(array2store));
  array2store[4] = RETRY_MODE;
  memcpy((uint8_t *)payload, (uint8_t *)&array2store, sizeof(Data_Packet_t));

  MX_SPI1_Init();
  HAL_Delay(100);
  sx1278_init();

  nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(150);
    // HAL_Delay(get_random_value(backupStorage, 50, 200));
  }
  if (nTry == 10)
  {
    myStatus = RETRY_MODE;
    HAL_SPI_DeInit(&hspi1);
    return;
  }

  sx1278_send_data(payload, sizeof(Data_Packet_t));

  Response_Handle();

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);
}

void Response_Handle(void)
{
  static ResponsePacket_t resp;
  static uint8_t data[100] = {0};
  static uint32_t nByteRx = 0;
  static int rssi = -1;
  static float snr = -1;

  is_LoRa_EXTI = false;
  sx1278_start_recv_data();

  uint32_t timeOut = HAL_GetTick();
  while ((HAL_GetTick() - timeOut) < 5000U)
  {
    if (is_LoRa_EXTI == true)
    {
      if (sx1278_recv_data((uint8_t *)data, &nByteRx, &rssi, &snr, false) == SX1278_OK)
      {
        if (nByteRx == sizeof(ResponsePacket_t))  memcpy(&resp, &data, sizeof(ResponsePacket_t));
        else
        {
          is_LoRa_EXTI = false;
          sx1278_set_irq(0x00);
          sx1278_write_reg(REG_IRQ_FLAGS, sx1278_read_reg(REG_IRQ_FLAGS));
          continue;
        }

        if ((resp.Packet_ID != RESPONSE_PACKET_ID) || (resp.Target_Node_ID != NODE_ID))
        {
          // this is not what I want OR this is not for me
          is_LoRa_EXTI = false;
          sx1278_start_recv_data();
          continue;
        }
        else
        {
          // Yep ok this is my packet and ID
          if (myStatus == LINK_MODE)
          {
            if ((resp.Target_Node_Response & 0x00FF) != LINK_ACK) {myStatus = LINK_MODE; return;}
            if      (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_ACCEPT)    myStatus = NORMAL_MODE;
            else if (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_REJECT)    myStatus = (uint8_t)resp.Target_Node_Status;
          }
          else if (myStatus == NORMAL_MODE)
              {
                if ((resp.Target_Node_Response & 0x00FF) != LINK_ACK) {myStatus = RETRY_MODE; return;}
                if      (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_CARRYON)   myStatus = (uint8_t)resp.Target_Node_Status;
                else if (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_DISMISS)   myStatus = LINK_MODE;
              }
              else if (myStatus == RETRY_MODE)
              {
                if      ((resp.Target_Node_Response & 0x00FF) != LINK_ACK) {myStatus = RETRY_MODE;  return;}
                else if ((resp.Target_Node_Response & 0x00FF) == LINK_ACK) {myStatus = NORMAL_MODE;}
                if      (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_CARRYON)   myStatus = (uint8_t)resp.Target_Node_Status;
                else if (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_DISMISS)   myStatus = LINK_MODE;
              }
          HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, resp.Target_Node_Period);
          return;
        }
      }
      else
      {
        is_LoRa_EXTI = false;
        sx1278_set_irq(0x00);
        sx1278_write_reg(REG_IRQ_FLAGS, sx1278_read_reg(REG_IRQ_FLAGS));
        continue;
      }
    }
  }

  if (myStatus == LINK_MODE)
  {
    myStatus = LINK_MODE;
  }
  else if (myStatus == NORMAL_MODE)
      {
        myStatus = RETRY_MODE;
      }
      else if (myStatus == RETRY_MODE)
          {
            myStatus = RETRY_MODE;
          }
}

bool Node_ID_inNetwork(uint16_t Node_ID)
{
	uint16_t N_ID[MAX_NODE];
  N_ID[0] = 0xCAACU;
  N_ID[1] = 0xCAADU;
  N_ID[2] = 0xCAAEU;
  for (int i = 0; i < MAX_NODE; i++)
  {
      if (N_ID[i] == NODE_ID) continue;
      if (N_ID[i] == Node_ID) return true;

  }
  return false;
}

void LoRa_Packet_Parser_Mesh(const uint8_t *data, uint32_t len)
{
    double val_sen = 0;
    switch ((uint16_t)((data[0] << 8) & 0xFF00) | (uint16_t)(data[1] & 0x00FF))
    {
        case PACKET_ID_0_MESH:
        {
            if (len != sizeof(Link_Packet_t)) LOG(TAG, "error");

            Link_Struct_t link_payload;
            memcpy((uint8_t *)&link_payload, (uint8_t *)&data[2], sizeof(Link_Struct_t)); // copy all data link for link_payload
            sprintf(Log,"Node ID: %02X", link_payload.Node_ID);
            LOG(TAG, Log);
            if (Node_ID_inNetwork(link_payload.Node_ID) == true)
            {
                ResponsePacket_t resp = {PACKET_ID_2_MESH, link_payload.Node_ID, NORMAL_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK)};

                nTry = 0;
                while ((is_OK_2_Talk() == false) && (nTry < 10))
                {
                  nTry++;
                  HAL_Delay(110);
                }

                if (nTry == 10)
                {
                  myStatus = LINK_MODE;
                  HAL_SPI_DeInit(&hspi1);
                  return;
                }                

                sx1278_send_data(&resp, sizeof(ResponsePacket_t));
                LOG(TAG, "Done!\n");
            }
            // sx1278_start_recv_data();
            break;
        }

        case PACKET_ID_1_MESH:
        {
            if (len != sizeof(Data_Packet_t)) break;
            LOG(TAG, "Data Packet\t");

            Data_Struct_t data_recv;
            memcpy((uint8_t *)&data_recv, (uint8_t *) &data[2], sizeof(Data_Struct_t));
            if (data_recv.Link.Node_ID == NODE_ID) break;
            if (data_recv.Link.Node_ID == 0xCAAC) 
            {
              memcpy((uint8_t *)&Node_Mesh[0], (uint8_t *) &data[0], sizeof(Data_Packet_t));
              Flag_Node_C = 1;
            }
            else if (data_recv.Link.Node_ID == 0xCAAD) 
                 {
                    memcpy((uint8_t *)&Node_Mesh[1], (uint8_t *) &data[0], sizeof(Data_Packet_t));
                    Flag_Node_D = 1;
                 }
                 else if (data_recv.Link.Node_ID == 0xCAAE) 
                      {
                        memcpy((uint8_t *)&Node_Mesh[2], (uint8_t *) &data[0], sizeof(Data_Packet_t));
                        Flag_Node_E = 1;
                      }

            ResponsePacket_t resp = {PACKET_ID_2_MESH, data_recv.Link.Node_ID, NORMAL_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_CARRYON << 8) & 0xFF00) | LINK_ACK)};
            LOG(TAG, "Response Lora Mesh...\t");

            nTry = 0;
            while ((is_OK_2_Talk() == false) && (nTry < 10))
            {
              nTry++;
              HAL_Delay(150);
            }
            if (nTry == 10)
            {
              myStatus = LINK_MODE;
              HAL_SPI_DeInit(&hspi1);
              return;
            }

            LOG(TAG, "Sending packet response to NODE!\t");
            sx1278_send_data(&resp, sizeof(ResponsePacket_t));
            if (data_recv.Link.Node_ID == 0xCAACU) sprintf(Log,"\nNode_1 ID: %02X",data_recv.Link.Node_ID);
			else if (data_recv.Link.Node_ID == 0xCAADU) sprintf(Log,"\nNode_2 ID: %02X",data_recv.Link.Node_ID);
				  else if (data_recv.Link.Node_ID == 0xCAAEU) sprintf(Log,"\nNode_3 ID: %02X",data_recv.Link.Node_ID);
			LOG(TAG, Log);

			sprintf(Log,"Period: %d",data_recv.Link.Node_Period);
			LOG(TAG, Log);
			if (data_recv.Link.Node_Status == 1) LOG(TAG,"Status: WAKEUP MODE!");
			else if (data_recv.Link.Node_Status == 2) LOG(TAG,"Status: LINK MODE!");
				  else if (data_recv.Link.Node_Status == 4) LOG(TAG,"Status: NORMAL MODE!");
					  else if (data_recv.Link.Node_Status == 8) LOG(TAG,"Status: RETRY MODE!");
							else if (data_recv.Link.Node_Status == 16) LOG(TAG,"Status: SHUTDOWN MODE!");

			val_sen = (double)(((double)(data_recv.Link.Node_Battery_Voltage))/100.0);
			sprintf(Log,"Battery: %.1f", val_sen);
			LOG(TAG, Log);

			val_sen = (double)(((double)(data_recv.Node_AHT_Humidity))/10.0);
			sprintf(Log,"AHT Humidity: %.1f%c", val_sen, 0x25);
			LOG(TAG, Log);

			val_sen = (double)(((double)(data_recv.Node_AHT_Temperature))/10.0);
			sprintf(Log,"AHT Temperature: %.1f", val_sen);
			LOG(TAG, Log);

			val_sen = (double)(((double)(data_recv.Node_LM35_Temperature))/10.0);
			sprintf(Log,"LM35: %.1f", val_sen);
			LOG(TAG, Log);

			val_sen = (double)(((double)(data_recv.Node_wind_speed))/10.0 * 3.6);
			sprintf(Log,"Wind Speed: %.1f", val_sen);
			LOG(TAG, Log);

			sprintf(Log,"Wind Level: %d", data_recv.Node_wind_level);
			LOG(TAG, Log);

			sprintf(Log,"Wind Direction: %d", data_recv.Node_wind_direction);
			LOG(TAG, Log);

            if (Node_ID_inNetwork(data_recv.Link.Node_ID) == false)
            {
                LOG(TAG, "NOT IN NETWORK");
                ResponsePacket_t resp = {PACKET_ID_2_MESH, data_recv.Link.Node_ID, LINK_MODE, DEFAULT_PERIOD_SEC, (uint16_t)(((LINK_DISMISS << 8) & 0xFF00) | LINK_ACK)};

                // nTry = 0;
                // while ((is_OK_2_Talk() == false) && (nTry < 10))
                // {
                //   nTry++;
                //   HAL_Delay(150);
                // }
                // if (nTry == 10)
                // {
                //   myStatus = RETRY_MODE;
                //   HAL_SPI_DeInit(&hspi1);
                //   return;
                // }

                LOG(TAG, "Sending packet to NODE...\t");
                sx1278_send_data(&resp, sizeof(ResponsePacket_t));
                
                LOG(TAG, "Dismiss NODE!!\n");
                // sx1278_start_recv_data();
            }
            break;
        }
        default: break;
    }
}

void Normal_Mode_Mesh_Handle(Data_Packet_t Node)
{
  MX_SPI1_Init();
  HAL_Delay(100);
  sx1278_init();

  memset(payload, '\0', sizeof(payload));
  memcpy((uint8_t *)payload, (uint8_t *)&Node, sizeof(Data_Packet_t));

  nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(150);
  }
  if (nTry == 10)
  {
    myStatus = LINK_MODE;
    HAL_SPI_DeInit(&hspi1);
    return;
  }

  sx1278_send_data(payload, sizeof(Data_Packet_t));
  Response_Handle();

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);
}

void LoRa_Recv_Node(void)
{
  // static ResponsePacket_t resp_mesh;
  static uint8_t data_mesh[100] = {0};
  static uint32_t nByteRx_mesh = 0;
  static int rssi_mesh = -1;
  static float snr_mesh = -1;
  uint32_t Tick_Time = HAL_GetTick();
  MX_SPI1_Init();
  HAL_Delay(100);
  sx1278_init();
  is_LoRa_EXTI = false;
  sx1278_start_recv_data();
  while ((HAL_GetTick() - Tick_Time) < TIMEOUT_NODE)
  {
    if (is_LoRa_EXTI == true)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
      if (sx1278_recv_data((uint8_t *)data_mesh, &nByteRx_mesh, &rssi_mesh, &snr_mesh, false) == SX1278_OK)
      {
    	  is_LoRa_EXTI = false;
    	  sprintf(Log, "\nLoRa Received %lu byte(s) - rssi: %d - snr: %2.1f", nByteRx_mesh, rssi_mesh, snr_mesh);
        LOG(TAG, Log);
//          if (nByteRx_mesh == 10)
//          {
//            sx1278_start_recv_data();
//            continue;
            // memcpy(&resp_mesh, &data_mesh, sizeof(ResponsePacket_t)); // this is response
        		// sprintf(Log,"Pack: %02x", resp_mesh.Packet_ID);
        		// LOG( TAG, Log);
            // sprintf(Log, "ID: %02x",   resp_mesh.Target_Node_ID);
            // LOG(TAG, Log);
            // for (int i =0; i< nByteRx_mesh; i++)
            // {
            // 	sprintf(Log, "%02x", data_mesh[i]);
            // 	LOG(TAG, Log);
            // }
//          }
          LoRa_Packet_Parser_Mesh((const uint8_t *)data_mesh, nByteRx_mesh);
          sx1278_start_recv_data();
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
      }  
      else
      {
    	  is_LoRa_EXTI = false;
    	  sx1278_set_irq(0x00);
    	  sx1278_write_reg(REG_IRQ_FLAGS, sx1278_read_reg(REG_IRQ_FLAGS));
      }
      is_LoRa_EXTI = false; 
    } 
  }
  
  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == LoRa_EXT0_Pin) && (is_LoRa_EXTI == false))
  {
    is_LoRa_EXTI = true;
  }
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
  HAL_NVIC_SystemReset();
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
