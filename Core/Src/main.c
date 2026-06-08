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
#include "sensors.h"
#include "stopwatch.h"
#include "time_state.h"
#include "ui.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// Hard-iron calibration parameters
static const float magnetometer_bias[3] = {
	132.30558640063234,
	33.271282971992065,
	-525.2472235060491
};

// Soft-iron calibration parameters
static const float magnetometer_softiron[3][3] = {
    {  19.03280489242586, -1.5731734544724312, 0.1032820245011633 },
    {-1.5731734544724343,  20.443466630136957, 0.6245395764881244},
    { 0.10328202450116475, 0.6245395764881235, 19.334706279055993 }
};

volatile uint32_t last_activity_ms = 0;

static volatile uint32_t last_pa0_ms = 0;
static volatile uint32_t last_pa10_ms = 0;
static volatile uint32_t last_pb9_ms = 0;
static volatile uint32_t last_pb8_ms = 0;
static volatile uint32_t last_pb3_ms = 0;

LPS22HH_Object_t lps22hh;
STTS22H_Object_t stts22h;
bool display_is_on;

volatile bool rtc_tick_flag;
volatile bool power_button_flag;

Interface_State_t previous_state = SET_TIME;
Interface_State_t interface_state = OFF;

float_t edit_temperature;
volatile bool ui_dirty = true;
volatile bool blink = false;

extern uint8_t displayBuffer[1024];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void update_last_activity_time(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Record the most recent user activity time for timeout tracking
static void update_last_activity_time(void) {
	last_activity_ms = HAL_GetTick();
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Bring up the IMU before any display mode tries to read compass/incline data
  imu_init();


  // Configure the pressure sensor for pressure-only sampling at a slow UI rate
  const float_t lps22hh_output_rate_hz = 1.0f;
  int32_t lps22hh_status = LPS22HH_OK;

  do {
	  if (LPS22HH_BusIO_Register_SPI(&lps22hh) != LPS22HH_OK) break;
	  if ((lps22hh_status = LPS22HH_Init(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_TEMP_Disable(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_PRESS_Enable(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_PRESS_SetOutputDataRate(&lps22hh, lps22hh_output_rate_hz)) != LPS22HH_OK) break;
  } while (0);

  // Configure the temperature sensor for one sample per second
  const float_t stts22h_output_rate_hz = 1.0f;
  int32_t stts22h_status = STTS22H_OK;

  do {
      if ((stts22h_status = STTS22H_BusIO_Register_I2C(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_Init(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_TEMP_Enable(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_TEMP_SetOutputDataRate(&stts22h, stts22h_output_rate_hz)) != STTS22H_OK) break;
  } while (0);

  // Initialize the LCD, clear it, then start with the display off
  ST7565_init();
  ST7565_command(CMD_DISPLAY_OFF);
  ST7565_clear();
  updateDisplay();
  display_is_on = false;

  // Shake detection is polled in the main loop and re-armed after motion settles
  static uint32_t last_shake_poll_ms = 0;
  static uint32_t last_shake_motion_ms = 0;
  static bool shake_armed = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (power_button_flag) {
	      power_button_flag = false;

	      if (interface_state == OFF) {
	          interface_state = TIME;
	    	  ST7565_on();
	    	  display_is_on = true;


	          ui_dirty = true;
	      } else {
	    	  ST7565_off();
	    	  display_is_on = false;

	          previous_state = interface_state;
	          interface_state = OFF;
	          ui_dirty = false;
	      }
	  }

      // RTC wakeup ticks drive blinking and low-rate screen refreshes
      if (rtc_tick_flag) {
          rtc_tick_flag = 0;

          blink = !blink;

          if (interface_state == TIME) {
              ui_dirty = true;
          } else if (interface_state == SET_TIME || interface_state == CALIBRATION) {
              ui_dirty = true;
          }
          else if (interface_state == PRESSURE || interface_state == TEMPERATURE) ui_dirty = true;
      }

      // Compass and incline update more often than the one-second RTC tick
      static uint32_t next_compass_ms = 0;
      static uint32_t next_incline_ms = 0;


      uint32_t current_tick_ms = HAL_GetTick();
      if (interface_state == COMPASS)
      {
          if (next_compass_ms == 0) next_compass_ms = current_tick_ms;

          if ((int32_t)(current_tick_ms - next_compass_ms) >= 0)
          {
              next_compass_ms += COMPASS_PERIOD_MS;
              ui_dirty = true;
          }
      } else if (interface_state == INCLINE) {
    	    if (next_incline_ms == 0) next_incline_ms = current_tick_ms;

    	    if ((int32_t)(current_tick_ms - next_incline_ms) >= 0)
    	    {
    	        next_incline_ms += INCLINE_PERIOD_MS;
    	        ui_dirty = true;
    	    }
      }
      else if (interface_state != OFF)
      {
          next_compass_ms = 0;
          next_incline_ms = 0;
      }

      // Turn the display off after a period with no button or shake activity
      if (interface_state != OFF) {
          uint32_t current_time_ms = HAL_GetTick();
          if ((current_time_ms - last_activity_ms) >= AUTO_TIMEOUT_MS) {
              ST7565_off();
              display_is_on = false;

              previous_state = interface_state;
              interface_state = OFF;
              ui_dirty = false;
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
          }
      }

	  // Read sensors every loop; successful samples feed display values and histories
	  float temperature;
	  float pressure;
	  int32_t temperature_status = STTS22H_TEMP_GetTemperature(&stts22h, &temperature);
	  int32_t pressure_status = LPS22HH_PRESS_GetPressure(&lps22hh, &pressure);

	  if (temperature_status == STTS22H_OK) {
		  temperature = celsius_to_fahrenheit(temperature) + temperature_offset;

		  if (temperature_history_count == 0) {
			  spark_fill(temperature_history, &temperature_history_head, &temperature_history_count, temperature);
		  }

	      spark_push(temperature_history, &temperature_history_head, &temperature_history_count, temperature);
	  }

	  if (pressure_status == LPS22HH_OK) {
		  if (pressure_history_count == 0) {
			  spark_fill(pressure_history, &pressure_history_head, &pressure_history_count, pressure);
		  }

		  spark_push(pressure_history, &pressure_history_head, &pressure_history_count, pressure);
	  }

      // Redraw only when an event marks the UI dirty
      if (ui_dirty) {
          ui_dirty = false;

          switch (interface_state) {

          case SET_TIME:
              // Date/time edit screen is redrawn for blink or field/value changes
              display_edit_date_time();
              updateDisplay();
              break;

          case TIME: {
              // Normal clock screen always reads the RTC immediately before drawing
              DateTime_t current_datetime;
              get_date_time(&current_datetime);

              display_date_time(&current_datetime);
              updateDisplay();
              break;
          }

          case STOPWATCH:
              // Stopwatch display is updated by timer ticks and user controls
              stopwatch_display_time();
              updateDisplay();
              break;

          case PRESSURE: {
        	  // Show current pressure, derived altitude, and recent pressure history
        	  char pressure_display_string[37];
        	  char altitude_display_string[37];

        	  if (pressure_status == LPS22HH_OK) {
        	      char pressure_string[20];
        	      char altitude_string[20];

        	      float altitude_ft = calculate_altitude(pressure);

                  ftoa(pressure_string, pressure, 2);
        	      ftoa(altitude_string, altitude_ft, 2);

        	      snprintf(pressure_display_string, sizeof(pressure_display_string),
        	               "%s HPa", pressure_string);
        	      snprintf(altitude_display_string, sizeof(altitude_display_string),
        	               "%s ft", altitude_string);
        	  } else {
        	      snprintf(pressure_display_string, sizeof(pressure_display_string),
        	               "Pressure Failure");
        	  }

              memset(displayBuffer, 0, sizeof(displayBuffer));
              ST7565_drawstring_anywhere(
                  (LCD_WIDTH - strlen(pressure_display_string) * 7) / 2,
                  LCD_HEIGHT - 8,
                  pressure_display_string
              );
              ST7565_drawstring_anywhere(
                  (LCD_WIDTH - strlen(altitude_display_string) * 7) / 2,
                  LCD_HEIGHT - 20,
                  altitude_display_string
              );

              spark_draw_line(0, 0, SPARK_W, SPARK_H - 10, PRESSURE, pressure_history, pressure_history_head, pressure_history_count, true, 8);

              updateDisplay();
              break;
          }

          case CALIBRATION: {
        	  // Calibration screen shares the date/time blink cadence for selection
        	  display_calibrate();
        	  updateDisplay();
        	  break;
          }

          case TEMPERATURE: {
              // Show current temperature and recent temperature history
        	  	  char temperature_display_string[38];

        	      char temperature_string[20];
            	  if (temperature_status == STTS22H_OK) {
					  ftoa(temperature_string, temperature, 2);
					  snprintf(temperature_display_string, sizeof(temperature_display_string),
							   "%s%cF", temperature_string, DEGREE_CHAR);
            	  } else {
					  snprintf(temperature_display_string, sizeof(temperature_display_string),
							   "Temperature Failure");
            	  }

              memset(displayBuffer, 0, sizeof(displayBuffer));
              ST7565_drawstring_anywhere_7x12(
                  (LCD_WIDTH - strlen(temperature_display_string) * 7) / 2,
                  LCD_HEIGHT - 16,
                  temperature_display_string
              );

              spark_draw_line(0, 0, SPARK_W, SPARK_H, TEMPERATURE, temperature_history, temperature_history_head, temperature_history_count, true, 8);

              updateDisplay();
              break;
          }

          case COMPASS: {
              // Apply hard-iron bias and soft-iron matrix before calculating heading
              float accel_x, accel_y, accel_z;
              float mag_x, mag_y, mag_z;

              memset(displayBuffer, 0, sizeof(displayBuffer));

              if (C6DOFIMU13_Accel_GetXYZ(&h6dof, &accel_x, &accel_y, &accel_z) == HAL_OK &&
                  C6DOFIMU13_Mag_GetXYZ(&h6dof, &mag_x, &mag_y, &mag_z) == HAL_OK)
              {
                  float mag_delta_x = mag_x - magnetometer_bias[0];
                  float mag_delta_y = mag_y - magnetometer_bias[1];
                  float mag_delta_z = mag_z - magnetometer_bias[2];

                  float corrected_mag_x =
                      magnetometer_softiron[0][0] * mag_delta_x +
                      magnetometer_softiron[0][1] * mag_delta_y +
                      magnetometer_softiron[0][2] * mag_delta_z;

                  float corrected_mag_y =
                      magnetometer_softiron[1][0] * mag_delta_x +
                      magnetometer_softiron[1][1] * mag_delta_y +
                      magnetometer_softiron[1][2] * mag_delta_z;

                  float heading_rad = atan2f(corrected_mag_y, corrected_mag_x);
                  float heading_degrees = heading_rad * (180.0f / 3.14159265f) + magnetometer_offset + 90.0;

                  if (heading_degrees < 0.0f) heading_degrees += 360.0f;
                  if (heading_degrees >= 360.0f) heading_degrees -= 360.0f;

                  draw_compass(heading_degrees);
              }
              else
              {
                  const char *imu_error = "IMU Failure";
                  ST7565_drawstring_anywhere(
                      (LCD_WIDTH / 2) - ((strlen(imu_error) / 2) * 6),
                      27,
                      imu_error
                  );
              }

              updateDisplay();
              break;
          }

          case INCLINE: {
        	  // Use accelerometer tilt around Y against the X/Z gravity vector
        	  float accel_x, accel_y, accel_z;

        	  memset(displayBuffer, 0, sizeof(displayBuffer));

        	  if (C6DOFIMU13_Accel_GetXYZ(&h6dof, &accel_x, &accel_y, &accel_z) == HAL_OK) {
        		  float incline_rad = fabsf(atan2f(accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)));
        		  float incline_degrees = fabsf(incline_rad * (180.0f / 3.14159265f) + accelerometer_offset);
        		  draw_incline(incline_degrees);
        	  } else {
                  const char *imu_error = "IMU Failure";
                  ST7565_drawstring_anywhere(
                      (LCD_WIDTH / 2) - ((strlen(imu_error) / 2) * 6),
                      27,
                      imu_error
                  );
              }

        	  updateDisplay();
        	  break;
          }

          default:
              break;
          }
      }

      // Shake toggles the display only once per motion burst, then waits to re-arm
      if ((current_tick_ms - last_shake_poll_ms) > SHAKE_POLL_MS)
         {
    	  	 last_shake_poll_ms = current_tick_ms;
             float accel_x, accel_y, accel_z;

             if (C6DOFIMU13_Accel_GetXYZ(&h6dof, &accel_x, &accel_y, &accel_z) == HAL_OK)
             {
                 float motion_delta = compute_motion_delta(accel_x, accel_y, accel_z);

                 if (motion_delta > SHAKE_THRESHOLD) {
                     last_shake_motion_ms = current_tick_ms;

                     if (shake_armed) {
                         shake_armed = false;

                         if (interface_state == OFF) {
                             ST7565_on();
                             display_is_on = true;
                             interface_state = TIME;
                             update_last_activity_time();
                             ui_dirty = true;
                         } else {
                             ST7565_off();
                             display_is_on = false;
                             interface_state = OFF;
                             update_last_activity_time();
                             ui_dirty = false;
                             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                         }

                         sensors_reset_motion_delta();
                     }
                 } else if (!shake_armed &&
                            (current_tick_ms - last_shake_motion_ms) >= SHAKE_REARM_MS) {
                     shake_armed = true;
                 }
             }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV16;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00000103;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000103;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x20;
  sTime.Minutes = 0x13;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 0x13;
  sDate.Year = 0x26;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *rtc_handle) {
	rtc_tick_flag = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer_handle)
{
    stopwatch_handle_timer_elapsed(timer_handle);
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    uint32_t current_tick_ms = HAL_GetTick();

    if (interface_state == OFF && gpio_pin != GPIO_PIN_0) {
        return;
    }

    if (gpio_pin == GPIO_PIN_0) {
        if ((uint32_t)(current_tick_ms - last_pa0_ms) < BTN_DEBOUNCE_MS) return;
        last_pa0_ms = current_tick_ms;

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_RESET) return;

        update_last_activity_time();

        switch (interface_state) {
            case SET_TIME:
                commit_date_time(&edit_time);
                edit_time_dirty = false;
                interface_state = TIME;
                ui_dirty = true;
            	break;
            case CALIBRATION:  interface_state = TIME; break;
            case STOPWATCH:
                power_button_flag = true;
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
                break;
            default: power_button_flag = true; HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); break;
        }
    }
    else if (gpio_pin == GPIO_PIN_9) {
        if ((uint32_t)(current_tick_ms - last_pb9_ms) < BTN_DEBOUNCE_MS) return;
        last_pb9_ms = current_tick_ms;

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) != GPIO_PIN_RESET) return;

        update_last_activity_time();

        switch (interface_state) {
            case SET_TIME:     increment_time(); break;
            case CALIBRATION:  adjust_offset(0.1); break;
            case TIME: interface_state = PRESSURE; break;
            case PRESSURE: interface_state = INCLINE; break;
            case INCLINE:  interface_state = COMPASS; break;
            case COMPASS:  interface_state = TEMPERATURE; break;
            case TEMPERATURE: interface_state = PRESSURE; break;
            case STOPWATCH: stopwatch_toggle_running(); break;
            default: break;
        }
    }
    else if (gpio_pin == GPIO_PIN_8) {
        if ((uint32_t)(current_tick_ms - last_pb8_ms) < BTN_DEBOUNCE_MS) return;
        last_pb8_ms = current_tick_ms;

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != GPIO_PIN_RESET) return;

        update_last_activity_time();

        switch (interface_state) {
            case SET_TIME:     next_time_field(); break;
            case CALIBRATION:  next_calibration_field(); break;
            case STOPWATCH:    stopwatch_clear(); break;
            default:
                interface_state = SET_TIME;
                enter_set_time_mode();
                ui_dirty = true;
            	break;
        }
    }
    else if (gpio_pin == GPIO_PIN_3) {
        if ((uint32_t)(current_tick_ms - last_pb3_ms) < BTN_DEBOUNCE_MS) return;
        last_pb3_ms = current_tick_ms;

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) != GPIO_PIN_RESET) return;

        update_last_activity_time();

        switch (interface_state) {
            case SET_TIME:     decrement_time(); break;
            case CALIBRATION:  adjust_offset(-0.1); break;
            case STOPWATCH: interface_state = TIME; ui_dirty = true; break;
            default:
                interface_state = STOPWATCH;
                ui_dirty = true;
            	break;
        }
    }
    else if (gpio_pin == GPIO_PIN_10) {
        if ((uint32_t)(current_tick_ms - last_pa10_ms) < BTN_DEBOUNCE_MS) return;
        last_pa10_ms = current_tick_ms;

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) != GPIO_PIN_RESET) return;

        switch (interface_state) {
            case SET_TIME:
				commit_date_time(&edit_time);
				edit_time_dirty = false;
				ui_dirty = true;
				interface_state = CALIBRATION;
				break;
            case CALIBRATION:  interface_state = SET_TIME; enter_set_time_mode(); ui_dirty = true; break;
            case TIME: HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); break;
            case STOPWATCH: HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); break;
            default: interface_state = TIME; break;
        }
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
