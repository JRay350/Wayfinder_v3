#ifndef STOPWATCH_H
#define STOPWATCH_H

#include "main.h"

void stopwatch_display_time(void);
void stopwatch_toggle_running(void);
void stopwatch_clear(void);
void stopwatch_handle_timer_elapsed(TIM_HandleTypeDef *timer_handle);

#endif
