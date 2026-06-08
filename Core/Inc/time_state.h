#ifndef TIME_STATE_H
#define TIME_STATE_H

#include "main.h"

extern volatile TimeEditField_t time_edit_field;
extern DateTime_t edit_time;
extern volatile bool edit_time_dirty;

void enter_set_time_mode(void);
void get_date_time(DateTime_t *date_time);
HAL_StatusTypeDef commit_date_time(const DateTime_t *date_time);
void next_time_field(void);
void increment_time(void);
void decrement_time(void);

#endif
