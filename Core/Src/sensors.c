#include "sensors.h"

extern I2C_HandleTypeDef hi2c1;

static float previous_accel_x = 0, previous_accel_y = 0, previous_accel_z = 0;
static bool previous_accel_valid = false;

C6DOFIMU13_HandleTypeDef h6dof;

volatile CalibrationEditField_t calibration_field = TEMPERATURE_FIELD;

float_t temperature_offset = 0.0;
float_t magnetometer_offset = 0.0;
float_t accelerometer_offset = 0.0;
float_t pressure_offset = 0.0;

float_t pressure_history[SPARK_W];
uint8_t pressure_history_head = 0;
uint8_t pressure_history_count = 0;

float_t temperature_history[SPARK_W];
uint8_t temperature_history_head = 0;
uint8_t temperature_history_count = 0;

float_t incline_history[SPARK_W];
uint8_t incline_history_head = 0;
uint8_t incline_history_count = 0;

static int16_t pressure_scale_min = 260;
static int16_t pressure_scale_max = 1260;

static int16_t temperature_scale_min = 24;
static int16_t temperature_scale_max = 100;

static int16_t incline_scale_min = 0;
static int16_t incline_scale_max = 360;

// Configure the combined accelerometer/magnetometer sensor
void imu_init(void) {
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

// Seed a sparkline history buffer so the first graph starts as a flat line
void spark_fill(float *history, uint8_t *head_index, uint8_t *sample_count, float sample)
{
    for (uint8_t sample_index = 0; sample_index < SPARK_W; sample_index++) history[sample_index] = sample;
    *head_index = 0;
    *sample_count = SPARK_W;
}

// Push one sample into the circular sparkline history buffer
void spark_push(float *history, uint8_t *head_index, uint8_t *sample_count, float sample)
{
    history[*head_index] = sample;
    *head_index = (uint8_t)((*head_index + 1u) % SPARK_W);
    if (*sample_count < SPARK_W) (*sample_count)++;
}

// Draw a sparkline from the circular history buffer
void spark_draw_line(uint8_t origin_x, uint8_t origin_y, uint8_t width, uint8_t height, Interface_State_t display_state, const float *history, uint8_t head_index, uint8_t sample_count, bool draw_border, uint8_t x_step)
{
    if (width == 0 || height == 0) return;

    ST7565_fillrect(origin_x, origin_y, width, height, WHITE);
    if (draw_border) ST7565_drawrect(origin_x, origin_y, width, height, BLACK);
    if (sample_count < 2) return;

    // Pick the y-axis range for the current sparkline
    const int16_t *scale_min;
    const int16_t *scale_max;
    switch (display_state) {
        case PRESSURE:
            scale_min = &pressure_scale_min;
            scale_max = &pressure_scale_max;
            break;

        case TEMPERATURE:
            scale_min = &temperature_scale_min;
            scale_max = &temperature_scale_max;
            break;

        default:
            scale_min = &incline_scale_min;
            scale_max = &incline_scale_max;
            break;
    }

    float min_value = *scale_min;
    float max_value = *scale_max;
    float scale_span = max_value - min_value;
    if (scale_span < 1e-6f) scale_span = 1e-6f;

    if (x_step == 0) x_step = 1;

    // Draw only the newest samples that fit in the requested width
    uint8_t max_visible_points = (uint8_t)(((width - 1) / x_step) + 1);
    uint8_t visible_points = sample_count;
    if (visible_points > max_visible_points) visible_points = max_visible_points;
    if (visible_points < 2) return;

    uint8_t right_edge = (uint8_t)(origin_x + width - 1);
    uint8_t bottom_edge = (uint8_t)(origin_y + height - 1);
    uint8_t first_history_offset = (uint8_t)(sample_count - visible_points);

    uint8_t graph_width = (uint8_t)((visible_points - 1) * x_step);
    uint8_t start_x = (graph_width < width) ? (uint8_t)(right_edge - graph_width) : origin_x;

    uint8_t previous_x = start_x;
    uint8_t previous_y = bottom_edge;

    for (uint8_t point_index = 0; point_index < visible_points; point_index++) {
        uint8_t sample_offset = (uint8_t)(first_history_offset + point_index);
        uint8_t history_index = (uint8_t)((head_index + SPARK_W - sample_count + sample_offset) % SPARK_W);
        float sample = history[history_index];

        // Clamp the sample before mapping it into the graph rectangle
        if (sample < min_value) sample = min_value;
        if (sample > max_value) sample = max_value;

        float normalized = (sample - min_value) / scale_span;
        uint8_t x = (uint8_t)(start_x + (uint8_t)(point_index * x_step));
        uint8_t y = (uint8_t)(origin_y + normalized * (height - 1));

        if (x > right_edge) x = right_edge;
        if (y > bottom_edge) y = bottom_edge;

        if (point_index > 0) ST7565_drawline(previous_x, previous_y, x, y, BLACK, 1);

        previous_x = x;
        previous_y = y;
    }
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

// Advance to the next editable calibration field
void next_calibration_field(void) {
	calibration_field++;
	if (calibration_field >= 4) calibration_field = TEMPERATURE_FIELD;
}

// Convert pressure in hPa to altitude in feet using the barometric formula
float calculate_altitude(float pressure_hpa) {
	float sea_level_pressure = 1013.25 + pressure_offset;
	float base = pressure_hpa / sea_level_pressure;
	float exponent = 0.190284;
	return (1 - pow(base, exponent)) * 145366.45;
}

// Convert Celsius sensor data to Fahrenheit display units
float celsius_to_fahrenheit(float celsius_temperature) {
	return celsius_temperature * 1.8 + 32;
}

// Add up the the per-axis accelerometer change from the previous sample
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

void sensors_reset_motion_delta(void)
{
    previous_accel_valid = false;
}
