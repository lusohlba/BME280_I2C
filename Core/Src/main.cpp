/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <string.h>
#include <stdio.h>
using namespace std;
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

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

//Adresses of the different BME280 registers.
enum {
//Calibration data registers.
  REGISTER_DIG_T1 = 0x88,
  REGISTER_DIG_T2 = 0x8A,
  REGISTER_DIG_T3 = 0x8C,

  REGISTER_DIG_P1 = 0x8E,
  REGISTER_DIG_P2 = 0x90,
  REGISTER_DIG_P3 = 0x92,
  REGISTER_DIG_P4 = 0x94,
  REGISTER_DIG_P5 = 0x96,
  REGISTER_DIG_P6 = 0x98,
  REGISTER_DIG_P7 = 0x9A,
  REGISTER_DIG_P8 = 0x9C,
  REGISTER_DIG_P9 = 0x9E,

  REGISTER_DIG_H1 = 0xA1,
  REGISTER_DIG_H2 = 0xE1,
  REGISTER_DIG_H3 = 0xE3,
  REGISTER_DIG_H4 = 0xE4,
  REGISTER_DIG_H5 = 0xE5,
  REGISTER_DIG_H6 = 0xE7,

 //Controll and stetting registers.
  REGISTER_CHIPID = 0xD0,
  REGISTER_VERSION = 0xD1,
  REGISTER_SOFTRESET = 0xE0,

  REGISTER_CAL26 = 0xE1,

  REGISTER_CONTROLHUMID = 0xF2,
  REGISTER_STATUS = 0XF3,
  REGISTER_CONTROL = 0xF4,
  REGISTER_CONFIG = 0xF5,

 //Measurement data registers.
  REGISTER_PRESSUREDATA = 0xF7,
  REGISTER_TEMPDATA = 0xFA,
  REGISTER_HUMIDDATA = 0xFD
};

//Structure to store the calibration data.
typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;

  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;

  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} calibration_data;


//BME280 class
class BME280{
private:
	uint8_t read8(uint8_t add);
	uint16_t read16(uint8_t add);
	uint16_t read16_LE(uint8_t add);
	int16_t readS16(uint8_t add);
	int16_t readS16_LE(uint8_t add);
	uint32_t read24(uint8_t add);
	bool write(uint8_t add, uint8_t data);
	void get_coefficients(void); //Reading the calibration data
	void set_profile(uint8_t profile); //Choosing sensor setting

protected:
	calibration_data calib;
	int32_t temp_fine;

	//Structure containing the config registers bits.
	 struct config {
	    unsigned int t_sb : 3;
	    unsigned int filter : 3;
	    // unused - don't set
	    unsigned int none : 1;     // unused - don't set
	    unsigned int spi3w_en : 1; // unused - don't set


	    unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
	  };
	  config configReg;

	  //Structure containing the ctrl_meas registers bits.
	  struct ctrl_meas {
	     unsigned int osrs_t : 3;
	     unsigned int osrs_p : 3;
	     unsigned int mode : 2;


	     unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
	   };
	   ctrl_meas measReg;

	   //Structure containing the ctrl_hum registers bits.
	   struct ctrl_hum {
	       /// unused - don't set
	       unsigned int none : 5;
	       unsigned int osrs_h : 3; // pressure oversampling


	       unsigned int get() { return (osrs_h); }
	     };
	     ctrl_hum humReg;
public:

	uint8_t SLAVE_READ_ADDRESS; //Adress of the BME280 in READ mode
	uint8_t SLAVE_WRITE_ADDRESS; //Adress of the BME280 in WRITE mode


	//Different possibities for sensor setting
	enum sensor_sampling {
	    SAMPLING_NONE = 0b000,
	    SAMPLING_X1 = 0b001,
	    SAMPLING_X2 = 0b010,
	    SAMPLING_X4 = 0b011,
	    SAMPLING_X8 = 0b100,
	    SAMPLING_X16 = 0b101
	  };

	enum sensor_mode {
	    MODE_SLEEP = 0b00,
	    MODE_FORCED = 0b01,
	    MODE_NORMAL = 0b11
	  };

	enum sensor_filter {
	    FILTER_OFF = 0b000,
	    FILTER_X2 = 0b001,
	    FILTER_X4 = 0b010,
	    FILTER_X8 = 0b011,
	    FILTER_X16 = 0b100
	  };

	enum standby_duration {
	    STANDBY_MS_0_5 = 0b000,
	    STANDBY_MS_10 = 0b110,
	    STANDBY_MS_20 = 0b111,
	    STANDBY_MS_62_5 = 0b001,
	    STANDBY_MS_125 = 0b010,
	    STANDBY_MS_250 = 0b011,
	    STANDBY_MS_500 = 0b100,
	    STANDBY_MS_1000 = 0b101
	  };

	BME280(uint8_t SLAVE_READ_ADDRESS = 0xED, uint8_t SLAVE_WRITE_ADDRESS = 0xEC); //Constructor with a default read/wirte adress.
	bool init(uint8_t profile);
	float get_temperature(void);
	float get_pressure(void);
	float get_altitude(void);
	float get_humidity(void);


};

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
	uint8_t buf[12];
	float data;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  BME280 bme280;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  bme280.init(1);
	  data = bme280.get_humidity();
	  data *= 100;
	          sprintf((char*)buf,
	                "%u.%u pro\r\n",
	                ((unsigned int)data / 100),
	                ((unsigned int)data % 100));
	 HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
	 HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
    
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x20303E5D;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

BME280::BME280(uint8_t SLAVE_READ_ADDRESS, uint8_t SLAVE_WRITE_ADDRESS){
		this->SLAVE_READ_ADDRESS = SLAVE_READ_ADDRESS;
		this->SLAVE_WRITE_ADDRESS = SLAVE_WRITE_ADDRESS;
	}


uint8_t  BME280::read8(uint8_t add){
	uint8_t buffer[] = {add};
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_WRITE_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_READ_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Receive(&hi2c1,SLAVE_READ_ADDRESS, buffer, 1, HAL_MAX_DELAY); // Receiving the data of the register and storing it into the buffer.
	return buffer[0];
}

uint16_t  BME280::read16(uint8_t add){
	uint8_t buffer[] = {add};
	uint16_t data;
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_WRITE_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_READ_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Receive(&hi2c1,SLAVE_READ_ADDRESS, buffer, 2, HAL_MAX_DELAY); // Receiving the data of the register and storing it into the buffer.
	data = buffer[0] << 8 | buffer[1];
	return data;
}

uint16_t  BME280::read16_LE(uint8_t add){
	uint8_t buffer[] = {add};
	uint16_t data;
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_WRITE_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_READ_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Receive(&hi2c1,SLAVE_READ_ADDRESS, buffer, 2, HAL_MAX_DELAY); // Receiving the data of the register and storing it into the buffer.
	data = buffer[1] << 8 | buffer[0];
	return data;
}

int16_t  BME280::readS16(uint8_t add){
	return (int16_t)read16(add);
}

int16_t  BME280::readS16_LE(uint8_t add){
	return (int16_t)read16_LE(add);
}

uint32_t  BME280::read24(uint8_t add){
	uint8_t buffer[] = {add};
	uint32_t data;
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_WRITE_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_READ_ADDRESS, buffer, 1, HAL_MAX_DELAY ); // Sending in WRITE mode the address of the register which should be read.
	HAL_I2C_Master_Receive(&hi2c1,SLAVE_READ_ADDRESS, buffer, 3, HAL_MAX_DELAY); // Receiving the data of the register and storing it into the buffer.
	data = buffer[0] << 16 | buffer[1] << 8 | buffer[2];
	return data;
}



bool BME280::write(uint8_t add, uint8_t data){
	HAL_StatusTypeDef ret;
	bool status = true;

	uint8_t buffer[]={add, data};

	ret =  HAL_I2C_Master_Transmit(&hi2c1, SLAVE_WRITE_ADDRESS, buffer, 2, HAL_MAX_DELAY ); //Sending in WIRTE mode the address of the register and the data that should be written into the register.
	if ( ret != HAL_OK ) {
		status = false;
	}
	return status;
}


bool BME280::init(uint8_t profile){
	bool status = true;

	//Checking if the Chip ID is correct.
	uint8_t chip_id = read8(REGISTER_CHIPID);
	if (chip_id != 0x60){
		status = false;
	}

	//Doing a reset.
	write(REGISTER_SOFTRESET,0xB6);
	HAL_Delay(400);

	get_coefficients();

	set_profile(profile);

	HAL_Delay(100);

	return status;
}

void BME280::get_coefficients(void){
	calib.dig_T1 = read16_LE(REGISTER_DIG_T1);
	calib.dig_T2 = readS16_LE(REGISTER_DIG_T2);
	calib.dig_T3 = readS16_LE(REGISTER_DIG_T3);

	calib.dig_P1 = read16_LE(REGISTER_DIG_P1);
	calib.dig_P2 = readS16_LE(REGISTER_DIG_P2);
	calib.dig_P3 = readS16_LE(REGISTER_DIG_P3);
	calib.dig_P4 = readS16_LE(REGISTER_DIG_P4);
	calib.dig_P5 = readS16_LE(REGISTER_DIG_P5);
	calib.dig_P6 = readS16_LE(REGISTER_DIG_P6);
	calib.dig_P7 = readS16_LE(REGISTER_DIG_P7);
	calib.dig_P8 = readS16_LE(REGISTER_DIG_P8);
	calib.dig_P9 = readS16_LE(REGISTER_DIG_P9);

	calib.dig_H1 = read8(REGISTER_DIG_H1);
	calib.dig_H2 = readS16_LE(REGISTER_DIG_H2);
	calib.dig_H3 = read8(REGISTER_DIG_H3);
	calib.dig_H4 = (read8(REGISTER_DIG_H4) << 4) |
	                         (read8(REGISTER_DIG_H4 + 1) & 0xF);
	calib.dig_H5 = (read8(REGISTER_DIG_H5 + 1) << 4) |
	                         (read8(REGISTER_DIG_H5) >> 4);
	calib.dig_H6 = (int8_t)read8(REGISTER_DIG_H6);
}

void BME280::set_profile(uint8_t profile){
	// 0: Default
	// 1: Weather monitoring
	// 2: Humidity sensing
	// 3: Indoor navigation
	// 4: Gaming
	switch (profile){
	case 0:
		break;
	case 1:
		measReg.mode = MODE_FORCED;
		measReg.osrs_t = SAMPLING_X1;
		measReg.osrs_p = SAMPLING_X1;
		humReg.osrs_h = SAMPLING_X1;
		configReg.filter = FILTER_OFF;
		configReg.t_sb = STANDBY_MS_500;
		break;
	case 2:
		measReg.mode = MODE_FORCED;
		measReg.osrs_t = SAMPLING_X1;
		measReg.osrs_p = SAMPLING_NONE;
		humReg.osrs_h = SAMPLING_X1;
		configReg.filter = FILTER_OFF;
		configReg.t_sb = STANDBY_MS_1000;
		break;
	case 3:
		measReg.mode = MODE_NORMAL;
		measReg.osrs_t = SAMPLING_X2;
		measReg.osrs_p = SAMPLING_X16;
		humReg.osrs_h = SAMPLING_X1;
		configReg.filter = FILTER_X16;
		configReg.t_sb = STANDBY_MS_0_5;
		break;
	case 4:
		measReg.mode = MODE_NORMAL;
		measReg.osrs_t = SAMPLING_X1;
		measReg.osrs_p = SAMPLING_X4;
		humReg.osrs_h = SAMPLING_NONE;
		configReg.filter = FILTER_X16;
		configReg.t_sb = STANDBY_MS_0_5;
	}

	write(REGISTER_CONTROL, MODE_SLEEP);
	write(REGISTER_CONTROLHUMID, humReg.get());
	write(REGISTER_CONFIG, configReg.get());
	write(REGISTER_CONTROL, measReg.get());
}

float BME280::get_temperature(void){
	int32_t var1, var2;
	int32_t digial_temp = read24(REGISTER_TEMPDATA);

	if (digial_temp == 0x800000)
	    return NAN;
	digial_temp >>=4;

	var1 = ((((digial_temp >> 3) - ((int32_t)calib.dig_T1 << 1))) *
	          ((int32_t)calib.dig_T2)) >>
	         11;

	  var2 = (((((digial_temp >> 4) - ((int32_t)calib.dig_T1)) *
	            ((digial_temp >> 4) - ((int32_t)calib.dig_T1))) >>
	           12) *
	          ((int32_t)calib.dig_T3)) >>
	         14;

	 temp_fine = var1 + var2;

	 float temp = (temp_fine * 5 + 128) >> 8;
	 return temp / 100;
}

float BME280::get_pressure(void){
	int64_t var1, var2, press;
	get_temperature();
	int32_t digital_press = read24(REGISTER_PRESSUREDATA);

	if (digital_press == 0x800000)
	    return NAN;
	digital_press >>= 4;

	var1 = ((int64_t)temp_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
	         ((var1 * (int64_t)calib.dig_P2) << 12);
	var1 =
	      (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

	 if (var1 == 0) {
	    return 0;
	  }
	 press = 1048576 - digital_press;
	 press = (((press << 31) - var2) * 3125) / var1;
	 var1 = (((int64_t)calib.dig_P9) * (press >> 13) * (press >> 13)) >> 25;
	 var2 = (((int64_t)calib.dig_P8) * press) >> 19;
	 press = ((press + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

	 return (float)press / 256;

}

float BME280::get_altitude(void){
	float sea_level =  101325; //Pa
	float current_press = get_pressure(); //Pressure in Pa
	float altitude = 44330*(1-pow(current_press/sea_level,(1/5.255)));
	return altitude;
}

float BME280::get_humidity(void){
	get_temperature();
	int32_t digital_hum = read16(REGISTER_HUMIDDATA);
	if (digital_hum == 0x8000)
	    return NAN;

	int32_t v_x1_u32r;
	v_x1_u32r = (temp_fine - ((int32_t)76800));
	v_x1_u32r = (((((digital_hum << 14) - (((int32_t)calib.dig_H4) << 20) -
	                  (((int32_t)calib.dig_H5) * v_x1_u32r)) +
	                 ((int32_t)16384)) >>
	                15) *
	               (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
	                    (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) +
	                     ((int32_t)32768))) >>
	                   10) +
	                  ((int32_t)2097152)) *
	                     ((int32_t)calib.dig_H2) +
	                 8192) >>
	                14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
	                             ((int32_t)calib.dig_H1)) >>
	                            4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	float humidity = (v_x1_u32r >> 12);
	 return humidity / 1024.0;

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
