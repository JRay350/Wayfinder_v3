#ifndef UI_H
#define UI_H

#include "main.h"

void display_date_time(DateTime_t *date_time);
void display_edit_date_time(void);
void display_calibrate(void);
void draw_compass(float heading_degrees);
void draw_incline(float incline_degrees);

#endif
