#include "motor_tune.h"
#include "motor_publicdata.h"
#include <stdlib.h>
#include <string.h>

#define MOTOR_TUNE_RX_SIZE 128U
#define MOTOR_TUNE_LINE_SIZE 64U

static volatile u8 rx_buffer[MOTOR_TUNE_RX_SIZE];
static volatile u16 rx_head = 0;
static volatile u16 rx_tail = 0;
static char line_buffer[MOTOR_TUNE_LINE_SIZE];
static u16 line_length = 0;

static float display_alpha = 0.12f;
static float ref_min = 0.10f;
static float ref_max = 0.20f;
static u16 ramp_half_ticks = 10000U;
static u8 rotate_enable = 0U;
static float rotate_hz = 0.50f;
static float rotate_id = 0.20f;
static u8 speed_enable = 0U;
static float speed_rpm = 0.05f;
static float speed_ff = 0.45f;
static float speed_min_current = 0.40f;

static u8 key_equals(const char *key, const char *expected)
{
    return (u8)(strcmp(key, expected) == 0);
}

static void process_command(void)
{
    char *separator;
    char *key;
    char *value;
    float number;

    line_buffer[line_length] = '\0';
    separator = strchr(line_buffer, '=');
    if(separator == NULL)
    {
        return;
    }

    *separator = '\0';
    key = line_buffer;
    value = separator + 1;
    number = (float)atof(value);

    if(key_equals(key, "ID_KP") && number >= 0.0f)
    {
        mc.id_pid.kp = number;
    }
    else if(key_equals(key, "ID_KI") && number >= 0.0f)
    {
        mc.id_pid.ki = number;
    }
    else if(key_equals(key, "IQ_KP") && number >= 0.0f)
    {
        mc.iq_pid.kp = number;
    }
    else if(key_equals(key, "IQ_KI") && number >= 0.0f)
    {
        mc.iq_pid.ki = number;
    }
    else if(key_equals(key, "OUT_MAX") && number > 0.0f)
    {
        mc.id_pid.out_max = number;
        mc.id_pid.out_min = -number;
        mc.iq_pid.out_max = number;
        mc.iq_pid.out_min = -number;
    }
    else if(key_equals(key, "REF_MIN") && number >= 0.0f)
    {
        ref_min = number;
    }
    else if(key_equals(key, "REF_MAX") && number >= 0.0f)
    {
        ref_max = number;
    }
    else if(key_equals(key, "RAMP_HALF_MS") && number > 0.0f)
    {
        ramp_half_ticks = (u16)(number * 20.0f);
        if(ramp_half_ticks < 100U)
        {
            ramp_half_ticks = 100U;
        }
    }
    else if(key_equals(key, "DISPLAY_ALPHA") && number > 0.01f)
    {
        if(number > 1.0f)
        {
            number = 1.0f;
        }
        display_alpha = number;
    }
    else if(key_equals(key, "ROTATE"))
    {
        rotate_enable = (number > 0.5f) ? 1U : 0U;
        if(rotate_enable != 0U)
        {
            mc.e_angle.electrical_angle_set_pu =
                mc.e_angle.electrical_angle_pu;
        }
    }
    else if(key_equals(key, "ROTATE_HZ") && number > 0.0f)
    {
        if(number > 5.0f)
        {
            number = 5.0f;
        }
        rotate_hz = number;
    }
    else if(key_equals(key, "ROTATE_ID") && number >= 0.0f)
    {
        if(number > 0.5f)
        {
            number = 0.5f;
        }
        rotate_id = number;
    }
    else if(key_equals(key, "SPEED_MODE"))
    {
        speed_enable = (number > 0.5f) ? 1U : 0U;
        if(speed_enable != 0U)
        {
            rotate_enable = 0U;
            pid_clear(&mc.spd_pid);
        }
        else
        {
            mc.iq_pid.ref = 0.0f;
        }
    }
    else if(key_equals(key, "SPEED_RPM"))
    {
        if(number > 5.0f)
        {
            number = 5.0f;
        }
        if(number < -5.0f)
        {
            number = -5.0f;
        }
        speed_rpm = number;
    }
    else if(key_equals(key, "SPD_KP") && number >= 0.0f)
    {
        mc.spd_pid.kp = number;
    }
    else if(key_equals(key, "SPD_KI") && number >= 0.0f)
    {
        mc.spd_pid.ki = number;
    }
    else if(key_equals(key, "SPD_OUT_MAX") && number > 0.0f)
    {
        mc.spd_pid.out_max = number;
        mc.spd_pid.out_min = -number;
    }
    else if(key_equals(key, "SPD_FF") && number >= 0.0f)
    {
        if(number > 1.0f)
        {
            number = 1.0f;
        }
        speed_ff = number;
    }
    else if(key_equals(key, "SPD_MIN") && number >= 0.0f)
    {
        if(number > 1.0f)
        {
            number = 1.0f;
        }
        speed_min_current = number;
    }

    if(ref_max <= ref_min)
    {
        ref_max = ref_min + 0.05f;
    }
}

void motor_tune_init(void)
{
    rx_head = 0;
    rx_tail = 0;
    line_length = 0;
    display_alpha = 0.12f;
    ref_min = 0.10f;
    ref_max = 0.20f;
    ramp_half_ticks = 10000U;
    rotate_enable = 0U;
    rotate_hz = 0.50f;
    rotate_id = 0.20f;
    speed_enable = 0U;
    speed_rpm = 0.05f;
    speed_ff = 0.45f;
    speed_min_current = 0.40f;
}

void motor_tune_receive(const u8 *data, u32 length)
{
    u32 i;
    for(i = 0; i < length; i++)
    {
        u16 next = (rx_head + 1U) % MOTOR_TUNE_RX_SIZE;
        if(next != rx_tail)
        {
            rx_buffer[rx_head] = data[i];
            rx_head = next;
        }
    }
}

void motor_tune_task(void)
{
    while(rx_tail != rx_head)
    {
        char character = (char)rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1U) % MOTOR_TUNE_RX_SIZE;

        if(character == '\r')
        {
            continue;
        }
        if(character == '\n')
        {
            if(line_length > 0U)
            {
                process_command();
                line_length = 0U;
            }
        }
        else if(line_length < (MOTOR_TUNE_LINE_SIZE - 1U))
        {
            line_buffer[line_length++] = character;
        }
        else
        {
            line_length = 0U;
        }
    }
}

float motor_tune_get_display_alpha(void)
{
    return display_alpha;
}

float motor_tune_get_ref_min(void)
{
    return ref_min;
}

float motor_tune_get_ref_max(void)
{
    return ref_max;
}

u16 motor_tune_get_ramp_half_ticks(void)
{
    return ramp_half_ticks;
}

u8 motor_tune_get_rotate_enable(void)
{
    return rotate_enable;
}

float motor_tune_get_rotate_hz(void)
{
    return rotate_hz;
}

float motor_tune_get_rotate_id(void)
{
    return rotate_id;
}

u8 motor_tune_get_speed_enable(void)
{
    return speed_enable;
}

float motor_tune_get_speed_rpm(void)
{
    return speed_rpm;
}

float motor_tune_get_speed_ff(void)
{
    return speed_ff;
}

float motor_tune_get_speed_min_current(void)
{
    return speed_min_current;
}
