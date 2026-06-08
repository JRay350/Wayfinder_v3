#include "stopwatch.h"

#include <string.h>

extern TIM_HandleTypeDef htim2;
extern volatile bool ui_dirty;
extern Interface_State_t interface_state;
extern uint8_t displayBuffer[1024];

static volatile bool stopwatch_running = false;
static volatile uint32_t stopwatch_elapsed_seconds = 0;

// Render the stopwatch value in the form HH:MM:SS
void stopwatch_display_time(void)
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

// Start or stop the timer
void stopwatch_toggle_running(void)
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

// Reset the stopwatch
void stopwatch_clear(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    stopwatch_elapsed_seconds = 0u;
    ui_dirty = true;
}

void stopwatch_handle_timer_elapsed(TIM_HandleTypeDef *timer_handle)
{
    if (timer_handle->Instance == TIM2 && stopwatch_running) {
        stopwatch_elapsed_seconds++;

        if (interface_state == STOPWATCH) {
            ui_dirty = true;
        }
    }
}
