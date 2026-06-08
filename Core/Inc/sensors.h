#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"

extern C6DOFIMU13_HandleTypeDef h6dof;

extern volatile CalibrationEditField_t calibration_field;
extern float_t temperature_offset;
extern float_t magnetometer_offset;
extern float_t accelerometer_offset;
extern float_t pressure_offset;

extern float_t pressure_history[SPARK_W];
extern uint8_t pressure_history_head;
extern uint8_t pressure_history_count;
extern float_t temperature_history[SPARK_W];
extern uint8_t temperature_history_head;
extern uint8_t temperature_history_count;
extern float_t incline_history[SPARK_W];
extern uint8_t incline_history_head;
extern uint8_t incline_history_count;

void imu_init(void);
void spark_fill(float *history, uint8_t *head_index, uint8_t *sample_count, float sample);
void spark_push(float *history, uint8_t *head_index, uint8_t *sample_count, float sample);
void spark_draw_line(uint8_t origin_x, uint8_t origin_y, uint8_t width, uint8_t height, Interface_State_t display_state, const float *history, uint8_t head_index, uint8_t sample_count, bool draw_border, uint8_t x_step);
void adjust_offset(float_t offset_delta);
void next_calibration_field(void);
float calculate_altitude(float pressure_hpa);
float celsius_to_fahrenheit(float celsius_temperature);
float compute_motion_delta(float accel_x, float accel_y, float accel_z);
void sensors_reset_motion_delta(void);

#endif
