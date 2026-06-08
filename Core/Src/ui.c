#include "ui.h"

#include <string.h>

#include "sensors.h"
#include "time_state.h"

extern volatile bool blink;
extern uint8_t displayBuffer[1024];

// Render the time screen
void display_date_time(DateTime_t *date_time)
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

    // Build all text positions before drawing so the layout stays centered
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

// Render the date/time edit screen (underline the selected field)
void display_edit_date_time(void)
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
        // Underline location logic based on the selected date/time field
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

// Render the calibration screen
void display_calibrate(void)
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
        // Format offsets once, then reuse them for both drawing and underline width
        char offset_string[20];

        ftoa(offset_string, temperature_offset, 1);
        snprintf(value_strings[0], sizeof(value_strings[0]), "%s%cF", offset_string, DEGREE_CHAR);

        ftoa(offset_string, magnetometer_offset, 1);
        snprintf(value_strings[1], sizeof(value_strings[1]), "%s%c ", offset_string, DEGREE_CHAR);

        ftoa(offset_string, accelerometer_offset, 1);
        snprintf(value_strings[2], sizeof(value_strings[2]), "%s%c ", offset_string, DEGREE_CHAR);

        ftoa(offset_string, pressure_offset, 1);
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
        // Left-align labels and right-align values, with split_x as a safety floor
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

// Draw the compass face, needle, and write the heading
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

    // Cardinal labels are positioned for the ST7565 coordinate orientation
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
    ftoa(degree_string, heading_degrees, 1);

    char display_string[16];
    snprintf(display_string, sizeof(display_string), "%s%c", degree_string, DEGREE_CHAR);

    uint8_t text_width = (uint8_t)(strlen(display_string) * 7);
    uint8_t text_x = (text_width < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - text_width) / 2) : 0;
    uint8_t text_y = (uint8_t)(LCD_HEIGHT - 12);

    ST7565_drawstring_anywhere_7x12(text_x, text_y, display_string);
}

// Draw the incline graphic
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

    // Graphic always displays incline in range of 0-90
    if (incline_degrees < 0.0f)  incline_degrees = 0.0f;
    if (incline_degrees > 90.0f) incline_degrees = 90.0f;

    float angle_rad = incline_degrees * (3.14159265f / 180.0f);

    float needle_x_float = center_x + radius * cosf(angle_rad);
    float needle_y_float = center_y + radius * sinf(angle_rad);

    uint8_t needle_x = (uint8_t)(needle_x_float + 0.5f);
    uint8_t needle_y = (uint8_t)(needle_y_float + 0.5f);

    ST7565_drawline(center_x, center_y, needle_x, needle_y, BLACK, 2);

    char degree_string[8];
    ftoa(degree_string, incline_degrees, 1);

    char display_string[16];
    snprintf(display_string, sizeof(display_string), "%s%c", degree_string, DEGREE_CHAR);

    uint8_t text_width = (uint8_t)(strlen(display_string) * 7);
    uint8_t text_x = (text_width < LCD_WIDTH) ? (uint8_t)((LCD_WIDTH - text_width) / 2) : 0;
    uint8_t text_y = (uint8_t)(LCD_HEIGHT - 12);

    ST7565_drawstring_anywhere_7x12(text_x, text_y, display_string);
}
