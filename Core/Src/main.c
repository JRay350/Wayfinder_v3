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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	uint8_t day;
	uint8_t month;
	uint16_t year;

	uint8_t weekday;
} DateTime_t;

typedef enum {
	OFF,
	SET_TIME,
	CALIBRATION,
	TIME,
	COMPASS,
	INCLINE,
	PRESSURE,
	TEMPERATURE,
	STOPWATCH,
} Interface_State_t;

typedef enum {
	EDIT_MONTH,
	EDIT_DAY,
	EDIT_YEAR,
    EDIT_HOUR,
    EDIT_MINUTE,
    EDIT_SECOND,
} TimeEditField_t;

typedef enum {
	TEMPERATURE_FIELD,
	MAGNETOMETER_FIELD,
	ACCELEROMETER_FIELD,
	PRESSURE_FIELD,
} CalibrationEditField_t;

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
static const float magnetometer_bias[3] = {
	132.30558640063234,
	33.271282971992065,
	-525.2472235060491
};

static const float magnetometer_softiron[3][3] = {
    {  19.03280489242586, -1.5731734544724312, 0.1032820245011633 },
    {-1.5731734544724343,  20.443466630136957, 0.6245395764881244},
    { 0.10328202450116475, 0.6245395764881235, 19.334706279055993 }
};

static float previous_accel_x = 0, previous_accel_y = 0, previous_accel_z = 0;
static bool previous_accel_valid = false;

volatile uint32_t last_activity_ms = 0;

static volatile uint32_t last_pa0_ms = 0;
static volatile uint32_t last_pa10_ms = 0;
static volatile uint32_t last_pb9_ms = 0;
static volatile uint32_t last_pb8_ms = 0;
static volatile uint32_t last_pb3_ms = 0;

C6DOFIMU13_HandleTypeDef h6dof;

LPS22HH_Object_t lps22hh;
STTS22H_Object_t stts22h;
bool display_is_on;

volatile bool rtc_tick_flag;
volatile bool power_button_flag;

Interface_State_t previous_state = SET_TIME;
Interface_State_t interface_state = OFF;

volatile TimeEditField_t time_edit_field = EDIT_MONTH;
DateTime_t edit_time;
float_t edit_temperature;
volatile bool ui_dirty = true;
volatile bool edit_time_dirty = false;
volatile bool blink = false;

static volatile bool stopwatch_running = false;
static volatile uint32_t stopwatch_elapsed_seconds = 0;

volatile CalibrationEditField_t calibration_field = TEMPERATURE_FIELD;

float_t temperature_offset = 0.0;
float_t magnetometer_offset = 0.0;
float_t accelerometer_offset = 0.0;
float_t pressure_offset = 0.0;

static float_t pressure_history[SPARK_W];
static uint8_t pressure_history_head = 0;
static uint8_t pressure_history_count = 0;

static float_t temperature_history[SPARK_W];
static uint8_t temperature_history_head = 0;
static uint8_t temperature_history_count = 0;

static float_t incline_history[SPARK_W];
static uint8_t incline_history_head = 0;
static uint8_t incline_history_count = 0;


static int16_t pressure_scale_min = 260;
static int16_t pressure_scale_max = 1260;

static int16_t temperature_scale_min = 24;
static int16_t temperature_scale_max = 100;

static int16_t incline_scale_min = 0;
static int16_t incline_scale_max = 360;

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
static bool is_leap_year(uint16_t year);
static uint8_t days_in_month(uint8_t month, uint16_t year);
static void clamp_day_to_month(DateTime_t *date_time);
static void imu_init(void);
static void spark_fill(float *history, uint8_t *head_index, uint8_t *sample_count, float sample);
static void spark_push(float *history, uint8_t *head_index, uint8_t *sample_count, float sample);
static void spark_draw_line(uint8_t origin_x, uint8_t origin_y, uint8_t width, uint8_t height, Interface_State_t display_state, const float *history, uint8_t head_index, uint8_t sample_count, bool draw_border, uint8_t x_step);
static uint8_t calculate_weekday(uint8_t day, uint8_t month, uint16_t year);
void enter_set_time_mode(void);
void rtc_get_date_time(DateTime_t *date_time);
HAL_StatusTypeDef rtc_commit_date_time(const DateTime_t *date_time);
void rtc_display_date_time(DateTime_t *date_time);
static void stopwatch_display_time(void);
static void stopwatch_toggle_running(void);
static void stopwatch_clear(void);
void rtc_display_edit_date_time(void);
void rtc_display_calibrate(void);
void next_time_field(void);
void increment_time(void);
void decrement_time(void);
void adjust_offset(float_t offset_delta);
void next_calibration_field(void);
void draw_compass(float heading_degrees);
void draw_incline(float incline_degrees);
void float_to_string(char *buffer, float value, int decimals);
float calculate_altitude(float pressure_hpa);
float celsius_to_fahrenheit(float celsius_temperature);
static void update_last_activity_time(void);
float compute_motion_delta(float accel_x, float accel_y, float accel_z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool is_leap_year(uint16_t year)
{
    return ((year % 4u) == 0u && (year % 100u) != 0u) || ((year % 400u) == 0u);
}

static uint8_t days_in_month(uint8_t month, uint16_t year)
{
    static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1 || month > 12) return 31;

    if (month == 2 && is_leap_year(year)) return 29;
    return dim[month - 1];
}

static void clamp_day_to_month(DateTime_t *date_time)
{
    uint8_t max_day = days_in_month(date_time->month, date_time->year);
    if (date_time->day < 1) date_time->day = 1;
    if (date_time->day > max_day) date_time->day = max_day;
}

static void imu_init(void) {
	C6DOFIMU13_Init(&h6dof, &hi2c1, C6DOFIMU13_DEV_ADDRESS_ACCEL_GND, C6DOFIMU13_DEV_ADDRESS_MAG);

	C6DOFIMU13_Accel_Init(&h6dof,
	                          C6DOFIMU13_ACCEL_SRTFR_RATE_32,
	                          C6DOFIMU13_ACCEL_OUTCFG_RANGE_16,
	                          C6DOFIMU13_ACCEL_OUTCFG_RES_14);

	C6DOFIMU13_Mag_Init(&h6dof,
	                        C6DOFIMU13_MAG_RES_15_BIT,
	                        C6DOFIMU13_MAG_OP_MODE_CONT,
	                        C6DOFIMU13_MAG_TEMP_MEAS_ON);
}

static void spark_fill(float *history, uint8_t *head_index, uint8_t *sample_count, float sample)
{
    for (uint8_t sample_index = 0; sample_index < SPARK_W; sample_index++) history[sample_index] = sample;
    *head_index = 0;
    *sample_count = SPARK_W;
}

static void spark_push(float *history, uint8_t *head_index, uint8_t *sample_count, float sample)
{
    history[*head_index] = sample;
    *head_index = (uint8_t)((*head_index + 1u) % SPARK_W);
    if (*sample_count < SPARK_W) (*sample_count)++;
}

static void spark_draw_line(uint8_t origin_x, uint8_t origin_y, uint8_t width, uint8_t height, Interface_State_t display_state, const float *history, uint8_t head_index, uint8_t sample_count, bool draw_border, uint8_t x_step)
{
    if (width == 0 || height == 0) return;

    ST7565_fillrect(origin_x, origin_y, width, height, WHITE);
    if (draw_border) ST7565_drawrect(origin_x, origin_y, width, height, BLACK);
    if (sample_count < 2) return;

    int16_t default_scale_min, default_scale_max;
    int16_t *scale_min, *scale_max;

    int16_t scale_relax_step = 0;

    switch (display_state) {
        case PRESSURE:
            default_scale_min = 260;
            default_scale_max = 1260;
            scale_min = &pressure_scale_min;
            scale_max = &pressure_scale_max;

            scale_relax_step = 0;
            break;

        case TEMPERATURE:
            default_scale_min = -1;
            default_scale_max = 140;
            scale_min = &temperature_scale_min;
            scale_max = &temperature_scale_max;

            scale_relax_step = 0;
            break;

        default:
            default_scale_min = 0;
            default_scale_max = 360;
            scale_min = &incline_scale_min;
            scale_max = &incline_scale_max;

            scale_relax_step = 0;
            break;
    }

    if (scale_relax_step > 0) {
        if (*scale_min < default_scale_min) {
            int16_t next_scale_min = *scale_min + scale_relax_step;
            *scale_min = (next_scale_min > default_scale_min) ? default_scale_min : next_scale_min;
        } else if (*scale_min > default_scale_min) {
            int16_t next_scale_min = *scale_min - scale_relax_step;
            *scale_min = (next_scale_min < default_scale_min) ? default_scale_min : next_scale_min;
        }

        if (*scale_max < default_scale_max) {
            int16_t next_scale_max = *scale_max + scale_relax_step;
            *scale_max = (next_scale_max > default_scale_max) ? default_scale_max : next_scale_max;
        } else if (*scale_max > default_scale_max) {
            int16_t next_scale_max = *scale_max - scale_relax_step;
            *scale_max = (next_scale_max < default_scale_max) ? default_scale_max : next_scale_max;
        }
    }

    float scale_min_value = *scale_min;
    float scale_max_value = *scale_max;

    float scale_span = scale_max_value - scale_min_value;
    if (scale_span < 1e-6f) scale_span = 1e-6f;

    uint8_t previous_pixel_x = origin_x;
    uint8_t previous_pixel_y = (uint8_t)(origin_y + (height - 1));

    if (x_step == 0) x_step = 1;

    uint8_t max_points = (uint8_t)(((width - 1) / x_step) + 1);
    if (max_points < 2) return;

    uint8_t points_to_draw = sample_count;
    if (points_to_draw > max_points) points_to_draw = max_points;
    if (points_to_draw < 2) return;

    int16_t pixel_span = (int16_t)(points_to_draw - 1) * (int16_t)x_step;
    int16_t start_x_candidate = (int16_t)origin_x + ((int16_t)(width - 1) - pixel_span);

    if (start_x_candidate < (int16_t)origin_x) start_x_candidate = (int16_t)origin_x;
    uint8_t start_x = (uint8_t)start_x_candidate;

    for (uint8_t point_index = 0; point_index < points_to_draw; point_index++) {
        uint8_t source_offset = (uint8_t)(sample_count - points_to_draw + point_index);
        uint8_t history_index = (uint8_t)((head_index + SPARK_W - sample_count + source_offset) % SPARK_W);
        float sample_value = history[history_index];

        if (sample_value < scale_min_value) sample_value = scale_min_value;
        if (sample_value > scale_max_value) sample_value = scale_max_value;

        float normalized_value = (sample_value - scale_min_value) / scale_span;

        int16_t pixel_y_candidate = (int16_t)(origin_y + normalized_value * (height - 1));

        uint8_t pixel_x = (uint8_t)(start_x + (uint8_t)(point_index * x_step));
        uint8_t pixel_y = (pixel_y_candidate < origin_y) ? origin_y : (pixel_y_candidate >= (origin_y + height) ? (uint8_t)(origin_y + height - 1) : (uint8_t)pixel_y_candidate);

        if (pixel_x < origin_x) pixel_x = origin_x;
        uint8_t right_edge = (uint8_t)(origin_x + width - 1);
        if (pixel_x > right_edge) pixel_x = right_edge;

        if (point_index > 0) ST7565_drawline(previous_pixel_x, previous_pixel_y, pixel_x, pixel_y, BLACK, 1);

        previous_pixel_x = pixel_x;
        previous_pixel_y = pixel_y;
    }
}

static uint8_t calculate_weekday(uint8_t day, uint8_t month, uint16_t year) {
	if (month < 3) {
		month += 12;
		year -= 1;
	}

	uint16_t year_of_century = year % 100;
	uint16_t zero_based_century = year / 100;

	uint8_t zeller_weekday = (day + (13 * (month + 1)) / 5 + year_of_century + year_of_century / 4 + zero_based_century / 4 + 5 * zero_based_century) % 7;

	uint8_t weekday = ((zeller_weekday + 5) % 7) + 1;
	return weekday;
}

void enter_set_time_mode(void)
{
    rtc_get_date_time(&edit_time);
    time_edit_field = EDIT_MONTH;
    ui_dirty = true;
}

void rtc_get_date_time(DateTime_t *date_time) {
	RTC_TimeTypeDef rtcTime;
	RTC_DateTypeDef rtcDate;

	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

    date_time->hours   = rtcTime.Hours;
    date_time->minutes = rtcTime.Minutes;
    date_time->seconds = rtcTime.Seconds;

    date_time->day   = rtcDate.Date;
    date_time->month = rtcDate.Month;
    date_time->year  = 2000 + rtcDate.Year;

    date_time->weekday = calculate_weekday(date_time->day, date_time->month, date_time->year);
}

HAL_StatusTypeDef rtc_commit_date_time(const DateTime_t *date_time) {
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};

    time.Hours = date_time->hours;
    time.Minutes = date_time->minutes;
    time.Seconds = date_time->seconds;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;

    uint16_t year = date_time->year;

    if (year < YEAR_MIN) year = YEAR_MIN;
    if (year > YEAR_MAX) year = YEAR_MAX;

    date.Year = (uint8_t)(year - YEAR_MIN);
    date.Month = date_time->month;
    date.Date  = date_time->day;
    date.WeekDay = calculate_weekday(date_time->day, date_time->month, date_time->year);

    if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
        return HAL_ERROR;

    if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

void rtc_display_date_time(DateTime_t *date_time)
{
    char weekday_str[5];
    char time_str[16];
    char date_str[16];

    static const char *weekday_names[] = {
        "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"
    };

    uint8_t weekday_index = date_time->weekday;
    if (weekday_index < 1 || weekday_index > 7) weekday_index = 1;

    snprintf(weekday_str, sizeof(weekday_str), "%s", weekday_names[weekday_index - 1]);

    snprintf(time_str, sizeof(time_str), "%02u:%02u",
             (unsigned)(date_time->hours % 24),
             (unsigned)(date_time->minutes % 60));

    snprintf(date_str, sizeof(date_str), "%02u/%02u",
             (unsigned)(date_time->month),
             (unsigned)(date_time->day));

    memset(displayBuffer, 0, sizeof(displayBuffer));

    const uint8_t TIME_FONT_H    = 28;
    const uint8_t TIME_FONT_STEP = 12;

    const uint8_t SIDE_FONT_H    = FONT8X13_H;
    const uint8_t SIDE_FONT_STEP = FONT8X13_STEP;

    uint16_t time_w    = (uint16_t)strlen(time_str)    * TIME_FONT_STEP;
    uint16_t weekday_w = (uint16_t)strlen(weekday_str) * SIDE_FONT_STEP;
    uint16_t date_w    = (uint16_t)strlen(date_str)    * SIDE_FONT_STEP;

    uint16_t side_width = (weekday_w > date_w) ? weekday_w : date_w;

    uint8_t left_margin = 4;
    uint8_t right_margin = 8;
    uint8_t col_gap = 16;
    uint8_t row_gap = 4;

    uint8_t time_x = left_margin;
    uint8_t time_y = (LCD_HEIGHT > TIME_FONT_H)
                   ? (uint8_t)((LCD_HEIGHT - TIME_FONT_H) / 2)
                   : 0;

    uint8_t right_x = (LCD_WIDTH > (right_margin + side_width))
                    ? (uint8_t)(LCD_WIDTH - right_margin - side_width)
                    : 0;

    uint8_t right_height = (uint8_t)(SIDE_FONT_H + row_gap + SIDE_FONT_H);
    uint8_t right_y = (LCD_HEIGHT > right_height)
                    ? (uint8_t)((LCD_HEIGHT - right_height) / 2)
                    : 0;

    if ((uint16_t)(time_x + time_w + col_gap) > right_x) {
        right_x = (uint8_t)(time_x + time_w + col_gap);
    }

    uint8_t weekday_x = (uint8_t)(right_x + (side_width - weekday_w) / 2);
    uint8_t date_x    = (uint8_t)(right_x + (side_width - date_w) / 2);

    uint8_t weekday_y = right_y;
    uint8_t date_y    = (uint8_t)(right_y + SIDE_FONT_H + row_gap);

    ST7565_drawstring_anywhere_8x40(time_x, time_y, time_str);
    ST7565_drawstring_anywhere_8x13(weekday_x, weekday_y, weekday_str);
    ST7565_drawstring_anywhere_8x13(date_x, date_y, date_str);
}

static void stopwatch_display_time(void)
{
    char time_str[16];
    uint32_t elapsed = stopwatch_elapsed_seconds;
    uint32_t hours = elapsed / 3600u;
    uint32_t minutes = (elapsed / 60u) % 60u;
    uint32_t seconds = elapsed % 60u;

    snprintf(time_str, sizeof(time_str), "%02lu:%02lu:%02lu",
             (unsigned long)hours,
             (unsigned long)minutes,
             (unsigned long)seconds);

    memset(displayBuffer, 0, sizeof(displayBuffer));

    uint16_t time_w = (uint16_t)strlen(time_str) * FONT8X40_STEP;
    uint8_t time_x = (time_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - time_w) / 2u) : 0u;
    uint8_t time_y = (LCD_HEIGHT > FONT8X40_H)
                   ? (uint8_t)((LCD_HEIGHT - FONT8X40_H) / 2u)
                   : 0u;

    ST7565_drawstring_anywhere_8x40(time_x, time_y, time_str);
}

static void stopwatch_toggle_running(void)
{
    if (stopwatch_running) {
        if (HAL_TIM_Base_Stop_IT(&htim2) == HAL_OK) {
            stopwatch_running = false;
        }
    } else {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        if (HAL_TIM_Base_Start_IT(&htim2) == HAL_OK) {
            stopwatch_running = true;
        }
    }

    ui_dirty = true;
}

static void stopwatch_clear(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    stopwatch_elapsed_seconds = 0u;
    ui_dirty = true;
}

void rtc_display_edit_date_time(void)
{
    char time_str[16];
    char date_str[16];

    snprintf(time_str, sizeof(time_str), "%02u:%02u:%02u",
             (unsigned)(edit_time.hours   % 24),
             (unsigned)(edit_time.minutes % 60),
             (unsigned)(edit_time.seconds % 60));

    snprintf(date_str, sizeof(date_str), "%02u/%02u/%04u",
             (unsigned)edit_time.month,
             (unsigned)edit_time.day,
             (unsigned)edit_time.year);

    const char *prompt = "Set Date/Time";

    memset(displayBuffer, 0, sizeof(displayBuffer));

    uint16_t prompt_w = (uint16_t)strlen(prompt)   * FONT7X12_STEP;
    uint16_t time_w   = (uint16_t)strlen(time_str) * FONT7X12_STEP;
    uint16_t date_w   = (uint16_t)strlen(date_str) * FONT7X12_STEP;

    uint8_t prompt_x = (prompt_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - prompt_w) / 2) : 0;
    uint8_t time_x   = (time_w   < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - time_w)   / 2) : 0;
    uint8_t date_x   = (date_w   < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - date_w)   / 2) : 0;

    uint8_t prompt_gap = 10;
    uint8_t line_gap   = 6;

    uint8_t block_h = (uint8_t)(
        FONT7X12_H + prompt_gap +
        FONT7X12_H + line_gap +
        FONT7X12_H
    );

    uint8_t bottom_y = (LCD_HEIGHT > block_h)
                         ? (uint8_t)((LCD_HEIGHT - block_h) / 2)
                         : 0;

    uint8_t time_y   = bottom_y;
    uint8_t date_y   = (uint8_t)(time_y + FONT7X12_H + line_gap);
    uint8_t prompt_y = (uint8_t)(date_y + FONT7X12_H + prompt_gap);

    ST7565_drawstring_anywhere_7x12(prompt_x, prompt_y, prompt);
    ST7565_drawstring_anywhere_7x12(time_x,   time_y,   time_str);
    ST7565_drawstring_anywhere_7x12(date_x,   date_y,   date_str);

    if (blink) {
        uint8_t underline_x = 0, underline_y = 0, underline_width = 0;

        const uint8_t UL_BELOW_BASELINE = 1;

        uint8_t underline_y_time =
            (time_y > UL_BELOW_BASELINE) ? (uint8_t)(time_y - UL_BELOW_BASELINE) : time_y;

        uint8_t underline_y_date =
            (date_y > UL_BELOW_BASELINE) ? (uint8_t)(date_y - UL_BELOW_BASELINE) : date_y;

        switch (time_edit_field) {
        case EDIT_MONTH:
            underline_x = (uint8_t)(date_x + 0 * FONT7X12_STEP);
            underline_y = underline_y_date;
            underline_width = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_DAY:
            underline_x = (uint8_t)(date_x + 3 * FONT7X12_STEP);
            underline_y = underline_y_date;
            underline_width = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_YEAR:
            underline_x = (uint8_t)(date_x + 6 * FONT7X12_STEP);
            underline_y = underline_y_date;
            underline_width = (uint8_t)(4 * FONT7X12_STEP);
            break;

        case EDIT_HOUR:
            underline_x = (uint8_t)(time_x + 0 * FONT7X12_STEP);
            underline_y = underline_y_time;
            underline_width = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_MINUTE:
            underline_x = (uint8_t)(time_x + 3 * FONT7X12_STEP);
            underline_y = underline_y_time;
            underline_width = (uint8_t)(2 * FONT7X12_STEP);
            break;

        case EDIT_SECOND:
            underline_x = (uint8_t)(time_x + 6 * FONT7X12_STEP);
            underline_y = underline_y_time;
            underline_width = (uint8_t)(2 * FONT7X12_STEP);
            break;

        default:
            break;
        }

        if (underline_width > 0) {
            ST7565_drawline(
                underline_x,
                underline_y,
                (uint8_t)(underline_x + underline_width - 1),
                underline_y,
                BLACK,
                1
            );
        }
    }
}



void rtc_display_calibrate(void)
{
    const char *title = "Calibration";

    const char *labels[4] = {
        "Temp.",
        "Compass",
        "Incline",
        "Press."
    };

    char value_strings[4][30];

    {
        char offset_string[20];

        float_to_string(offset_string, temperature_offset, 1);
        snprintf(value_strings[0], sizeof(value_strings[0]), "%s%cF", offset_string, DEGREE_CHAR);

        float_to_string(offset_string, magnetometer_offset, 1);
        snprintf(value_strings[1], sizeof(value_strings[1]), "%s%c ", offset_string, DEGREE_CHAR);

        float_to_string(offset_string, accelerometer_offset, 1);
        snprintf(value_strings[2], sizeof(value_strings[2]), "%s%c ", offset_string, DEGREE_CHAR);

        float_to_string(offset_string, pressure_offset, 1);
        snprintf(value_strings[3], sizeof(value_strings[3]), "%shPa", offset_string);
    }

    memset(displayBuffer, 0, sizeof(displayBuffer));

    const uint8_t TITLE_H   = FONT7X12_H;
    const uint8_t TITLE_STEP= FONT7X12_STEP;

    const uint8_t ROW_H     = FONT7X12_H;
    const uint8_t ROW_STEP  = FONT7X12_STEP;

    const uint8_t LEFT_MARGIN  = 0;
    const uint8_t RIGHT_MARGIN = 30;

    const uint8_t split_x = 72;

    uint16_t title_w = (uint16_t)strlen(title) * (uint16_t)TITLE_STEP;
    uint8_t  title_x = (title_w < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - title_w) / 2) : 0;

    const uint8_t top_gap = 2;

    uint8_t row_gap = 2;
    {
        uint8_t needed_height = (uint8_t)(TITLE_H + top_gap + 4 * ROW_H);
        if (LCD_HEIGHT > needed_height) {
            uint8_t extra_height = (uint8_t)(LCD_HEIGHT - needed_height);
            row_gap = (uint8_t)(extra_height / 3);
            if (row_gap > 10) row_gap = 10;
        }
    }

    int16_t title_y_candidate = (int16_t)LCD_HEIGHT - (int16_t)TITLE_H;
    if (title_y_candidate < 0) title_y_candidate = 0;
    uint8_t title_y = (uint8_t)title_y_candidate;

    int16_t first_row_y_candidate = (int16_t)title_y - (int16_t)top_gap - (int16_t)ROW_H;
    if (first_row_y_candidate < 0) first_row_y_candidate = 0;

    uint8_t row_y_positions[4];
    for (uint8_t row_index = 0; row_index < 4; row_index++) {
        int16_t row_y_candidate = first_row_y_candidate - (int16_t)row_index * (int16_t)(ROW_H + row_gap);
        if (row_y_candidate < 0) row_y_candidate = 0;
        row_y_positions[row_index] = (uint8_t)row_y_candidate;
    }

    ST7565_drawstring_anywhere_7x12(title_x, title_y, title);

    for (uint8_t row_index = 0; row_index < 4; row_index++) {
        ST7565_drawstring_anywhere_7x12(LEFT_MARGIN, row_y_positions[row_index], labels[row_index]);

        uint16_t value_width = (uint16_t)strlen(value_strings[row_index]) * (uint16_t)ROW_STEP;
        int16_t value_x_candidate  = (int16_t)LCD_WIDTH - (int16_t)value_width - RIGHT_MARGIN;
        if (value_x_candidate < split_x) value_x_candidate = split_x;

        ST7565_drawstring_anywhere_7x12((uint8_t)value_x_candidate, row_y_positions[row_index], value_strings[row_index]);
    }

    if (blink) {
        uint8_t selected_index = (calibration_field < 4) ? calibration_field : 0;

        const uint8_t UL_BELOW_BASELINE = 1;
        uint8_t underline_y = (row_y_positions[selected_index] > UL_BELOW_BASELINE)
                         ? (uint8_t)(row_y_positions[selected_index] - UL_BELOW_BASELINE)
                         : row_y_positions[selected_index];

        uint16_t value_width = (uint16_t)strlen(value_strings[selected_index]) * (uint16_t)ROW_STEP;
        int16_t value_x_candidate  = (int16_t)LCD_WIDTH - (int16_t)value_width - RIGHT_MARGIN;
        if (value_x_candidate < split_x) value_x_candidate = split_x;

        if (value_width > 0 && value_width < 255) {
            ST7565_drawline(
                (uint8_t)value_x_candidate,
                underline_y,
                (uint8_t)((uint8_t)value_x_candidate + (uint8_t)value_width - 1),
                underline_y,
                BLACK,
                1
            );
        }
    }
}



void next_time_field(void) {
    if (time_edit_field == EDIT_SECOND)
        time_edit_field = EDIT_MONTH;
    else
        time_edit_field++;

    ui_dirty = true;
}

void increment_time(void) {
    switch (time_edit_field) {

    case EDIT_MONTH:
        edit_time.month++;
        if (edit_time.month > 12)
            edit_time.month = 1;

        clamp_day_to_month(&edit_time);
        break;

    case EDIT_DAY:
    {
        uint8_t maxd = days_in_month(edit_time.month, edit_time.year);
        edit_time.day++;
        if (edit_time.day > maxd)
            edit_time.day = 1;
    }
    break;

    case EDIT_YEAR:
        edit_time.year = (edit_time.year >= YEAR_MAX) ? YEAR_MIN : (edit_time.year + 1);
        clamp_day_to_month(&edit_time);
        break;

    case EDIT_HOUR:
        edit_time.hours = (edit_time.hours + 1) % 24;
        break;

    case EDIT_MINUTE:
        edit_time.minutes = (edit_time.minutes + 1) % 60;
        break;

    case EDIT_SECOND:
        edit_time.seconds = (edit_time.seconds + 1) % 60;
        break;
    }
    edit_time_dirty = true;
    ui_dirty = true;
}

void decrement_time(void) {
	switch (time_edit_field) {

	case EDIT_MONTH:
	    if (edit_time.month <= 1)
	        edit_time.month = 12;
	    else
	        edit_time.month--;

	    clamp_day_to_month(&edit_time);
	    break;

    case EDIT_DAY: {
        uint8_t maxd = days_in_month(edit_time.month, edit_time.year);
        edit_time.day = (edit_time.day <= 1) ? maxd : (edit_time.day - 1);
        break;
    }

    case EDIT_YEAR:
        edit_time.year = (edit_time.year <= YEAR_MIN) ? YEAR_MAX : (edit_time.year - 1);
        clamp_day_to_month(&edit_time);
        break;

	case EDIT_HOUR:
	    edit_time.hours = (edit_time.hours == 0) ? 23 : edit_time.hours - 1;
	    break;

	case EDIT_MINUTE:
	    edit_time.minutes = (edit_time.minutes == 0) ? 59 : edit_time.minutes - 1;
	    break;

	case EDIT_SECOND:
	    edit_time.seconds = (edit_time.seconds == 0) ? 59 : edit_time.seconds - 1;
	    break;
    }

    edit_time_dirty = true;
    ui_dirty = true;
}

void adjust_offset(float_t offset_delta) {
	switch (calibration_field) {

	case TEMPERATURE_FIELD:
		temperature_offset += offset_delta;
		break;

	case MAGNETOMETER_FIELD:
		magnetometer_offset += offset_delta;
		break;

	case ACCELEROMETER_FIELD:
		accelerometer_offset += offset_delta;
		break;

	case PRESSURE_FIELD:
		pressure_offset += offset_delta;
		break;
	}
}

void next_calibration_field(void) {
	calibration_field++;
	if (calibration_field >= 4) calibration_field = TEMPERATURE_FIELD;
}

void draw_compass(float heading_degrees)
{
    const uint8_t text_height   = 14;
    const uint8_t drawing_area_height = (LCD_HEIGHT > text_height) ? (LCD_HEIGHT - text_height) : LCD_HEIGHT;

    const uint8_t center_x = (uint8_t)(LCD_WIDTH / 2);
    const uint8_t center_y = (uint8_t)(drawing_area_height / 2);

    uint8_t radius = 15;
    if (center_y > 1 && radius > (uint8_t)(center_y - 1)) radius = (uint8_t)(center_y - 1);
    if (center_x > 1 && radius > (uint8_t)(center_x - 1)) radius = (uint8_t)(center_x - 1);

    ST7565_drawcircle(center_x, center_y, radius, BLACK);

    ST7565_drawchar_anywhere(center_x - 2,             center_y + radius + 1,  'N');
    ST7565_drawchar_anywhere(center_x + radius + 3,    center_y - 3,           'E');
    ST7565_drawchar_anywhere(center_x - 2,             center_y - radius - 9,  'S');
    ST7565_drawchar_anywhere(center_x - radius - 9,    center_y - 3,           'W');

    float angle_rad = heading_degrees * (3.14159265f / 180.0f);

    float needle_x_float = center_x + radius * sinf(angle_rad);
    float needle_y_float = center_y + radius * cosf(angle_rad);

    uint8_t needle_x = (uint8_t)(needle_x_float + 0.5f);
    uint8_t needle_y = (uint8_t)(needle_y_float + 0.5f);

    ST7565_drawline(center_x, center_y, needle_x, needle_y, BLACK, 2);

    char degree_string[8];
    float_to_string(degree_string, heading_degrees, 1);

    char display_string[16];
    snprintf(display_string, sizeof(display_string), "%s%c", degree_string, DEGREE_CHAR);

    uint8_t text_width = (uint8_t)(strlen(display_string) * 7);
    uint8_t text_x = (text_width < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - text_width) / 2) : 0;
    uint8_t text_y = (uint8_t)(LCD_HEIGHT - 12);

    ST7565_drawstring_anywhere_7x12(text_x, text_y, display_string);
}

void draw_incline(float incline_degrees)
{
    const uint8_t text_height   = 14;
    const uint8_t drawing_area_height = (LCD_HEIGHT > text_height) ? (LCD_HEIGHT - text_height) : LCD_HEIGHT;

    const uint8_t center_x = (uint8_t)(LCD_WIDTH / 2);
    const uint8_t center_y = (uint8_t)(drawing_area_height / 2);

    uint8_t radius = 15;
    if (center_y > 1 && radius > (uint8_t)(center_y - 1)) radius = (uint8_t)(center_y - 1);
    if (center_x > 1 && radius > (uint8_t)(center_x - 1)) radius = (uint8_t)(center_x - 1);

    ST7565_drawcircle(center_x, center_y, radius, BLACK);

    ST7565_drawline(center_x - radius, center_y, center_x + radius, center_y, BLACK, 1);

    if (incline_degrees < 0.0f)  incline_degrees = 0.0f;
    if (incline_degrees > 90.0f) incline_degrees = 90.0f;

    float angle_rad = incline_degrees * (3.14159265f / 180.0f);

    float needle_x_float = center_x + radius * cosf(angle_rad);
    float needle_y_float = center_y + radius * sinf(angle_rad);

    uint8_t needle_x = (uint8_t)(needle_x_float + 0.5f);
    uint8_t needle_y = (uint8_t)(needle_y_float + 0.5f);

    ST7565_drawline(center_x, center_y, needle_x, needle_y, BLACK, 2);

    char degree_string[8];
    float_to_string(degree_string, incline_degrees, 1);

    char display_string[16];
    snprintf(display_string, sizeof(display_string), "%s%c", degree_string, DEGREE_CHAR);

    uint8_t text_width = (uint8_t)(strlen(display_string) * 7);
    uint8_t text_x = (text_width < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - text_width) / 2) : 0;
    uint8_t text_y = (uint8_t)(LCD_HEIGHT - 12);

    ST7565_drawstring_anywhere_7x12(text_x, text_y, display_string);
}

void float_to_string(char *buffer, float value, int decimals)
{
    if (decimals < 0) {
        decimals = 0;
    }

    if (value < 0) {
        *buffer++ = '-';
        value = -value;
    }

    int scale = 1;
    for (int decimal_index = 0; decimal_index < decimals; decimal_index++)
        scale *= 10;

    int integer_part = (int)value;
    float remainder = value - integer_part;
    int fractional_part = (int)(remainder * scale + 0.5f);

    if (fractional_part >= scale) {
        integer_part++;
        fractional_part -= scale;
    }

    sprintf(buffer, "%d", integer_part);

    while (*buffer != '\0') buffer++;

    if (decimals > 0) {
        *buffer++ = '.';

        int pad = scale / 10;
        while (pad > 1 && fractional_part < pad) {
            *buffer++ = '0';
            pad /= 10;
        }

        sprintf(buffer, "%d", fractional_part);
    }
}

float calculate_altitude(float pressure_hpa) {
	float sea_level_pressure = 1013.25 + pressure_offset;
	float base = pressure_hpa / sea_level_pressure;
	float exponent = 0.190284;
	return (1 - pow(base, exponent)) * 145366.45;
}

float celsius_to_fahrenheit(float celsius_temperature) {
	return celsius_temperature * 1.8 + 32;
}

static void update_last_activity_time(void) {
	last_activity_ms = HAL_GetTick();
}

float compute_motion_delta(float accel_x, float accel_y, float accel_z)
{
    if (!previous_accel_valid) {
        previous_accel_x = accel_x;
        previous_accel_y = accel_y;
        previous_accel_z = accel_z;
        previous_accel_valid = true;
        return 0.0;
    }

    float delta =
        fabsf(accel_x - previous_accel_x) +
        fabsf(accel_y - previous_accel_y) +
        fabsf(accel_z - previous_accel_z);

    previous_accel_x = accel_x;
    previous_accel_y = accel_y;
    previous_accel_z = accel_z;

    return delta;
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

  imu_init();


  const float_t lps22hh_output_rate_hz = 1.0f;
  int32_t lps22hh_status = LPS22HH_OK;

  do {
	  if (LPS22HH_BusIO_Register_SPI(&lps22hh) != LPS22HH_OK) break;
	  if ((lps22hh_status = LPS22HH_Init(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_TEMP_Disable(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_PRESS_Enable(&lps22hh)) != LPS22HH_OK) break;
      if ((lps22hh_status = LPS22HH_PRESS_SetOutputDataRate(&lps22hh, lps22hh_output_rate_hz)) != LPS22HH_OK) break;
  } while (0);

  const float_t stts22h_output_rate_hz = 1.0f;
  int32_t stts22h_status = STTS22H_OK;

  do {
      if ((stts22h_status = STTS22H_BusIO_Register_I2C(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_Init(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_TEMP_Enable(&stts22h)) != STTS22H_OK) break;
      if ((stts22h_status = STTS22H_TEMP_SetOutputDataRate(&stts22h, stts22h_output_rate_hz)) != STTS22H_OK) break;
  } while (0);

  ST7565_init();
  ST7565_command(CMD_DISPLAY_OFF);
  ST7565_clear();
  updateDisplay();
  display_is_on = false;

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

      if (ui_dirty) {
          ui_dirty = false;

          switch (interface_state) {

          case SET_TIME:
              rtc_display_edit_date_time();
              updateDisplay();
              break;

          case TIME: {
              DateTime_t current_datetime;
              rtc_get_date_time(&current_datetime);

              rtc_display_date_time(&current_datetime);
              updateDisplay();
              break;
          }

          case STOPWATCH:
              stopwatch_display_time();
              updateDisplay();
              break;

          case PRESSURE: {
        	  char pressure_display_string[37];
        	  char altitude_display_string[37];

        	  if (pressure_status == LPS22HH_OK) {
        	      char pressure_string[20];
        	      char altitude_string[20];

        	      float altitude_ft = calculate_altitude(pressure);

                  float_to_string(pressure_string, pressure, 2);
        	      float_to_string(altitude_string, altitude_ft, 2);

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
        	  rtc_display_calibrate();
        	  updateDisplay();
        	  break;
          }

          case TEMPERATURE: {
        	  	  char temperature_display_string[38];

        	      char temperature_string[20];
            	  if (temperature_status == STTS22H_OK) {
					  float_to_string(temperature_string, temperature, 2);
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

                         previous_accel_valid = false;
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
    if (timer_handle->Instance == TIM2 && stopwatch_running) {
        stopwatch_elapsed_seconds++;

        if (interface_state == STOPWATCH) {
            ui_dirty = true;
        }
    }
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
                rtc_commit_date_time(&edit_time);
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
				rtc_commit_date_time(&edit_time);
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
