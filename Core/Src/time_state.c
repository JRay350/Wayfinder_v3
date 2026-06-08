#include "time_state.h"

extern RTC_HandleTypeDef hrtc;
extern volatile bool ui_dirty;

volatile TimeEditField_t time_edit_field = EDIT_MONTH;
DateTime_t edit_time;
volatile bool edit_time_dirty = false;

// Return if the supplied year has a February 29
static bool is_leap_year(uint16_t year)
{
    return ((year % 4u) == 0u && (year % 100u) != 0u) || ((year % 400u) == 0u);
}

// Return the number of days in month
static uint8_t days_in_month(uint8_t month, uint16_t year)
{
    static const uint8_t dim[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (month < 1 || month > 12) return 31;

    if (month == 2 && is_leap_year(year)) return 29;
    return dim[month - 1];
}

// Keep the edited day valid after month/year changes
static void clamp_day_to_month(DateTime_t *date_time)
{
    uint8_t max_day = days_in_month(date_time->month, date_time->year);
    if (date_time->day < 1) date_time->day = 1;
    if (date_time->day > max_day) date_time->day = max_day;
}

// Calculate weekday using Zeller's congruence - return 1=>Mon through 7=>Sun
static uint8_t calculate_weekday(uint8_t day, uint8_t month, uint16_t year) {
	if (month < 3) {
		month += 12;
		year -= 1;
	}

	uint16_t K = year % 100;
	uint16_t J = year / 100;

	uint8_t h = (day + (13 * (month + 1)) / 5 + K + K/4 + J/4 + 5*J) % 7;

	uint8_t weekday = ((h + 5) % 7) + 1;
	return weekday;
}

// Start date/time editing from the RTC's current value
void enter_set_time_mode(void)
{
    get_date_time(&edit_time);
    time_edit_field = EDIT_MONTH;
    ui_dirty = true;
}

// Read the RTC into the app's DateTime_t format
void get_date_time(DateTime_t *date_time) {
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

// Write the edited date/time back to the RTC after clamping the supported year
HAL_StatusTypeDef commit_date_time(const DateTime_t *date_time) {
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

// Select the next date/time field
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
