#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>

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

#endif
