#include "motor_tune.h"
#include "motor_publicdata.h"
#include <stdlib.h>
#include <string.h>

#define MOTOR_TUNE_RX_SIZE 128U
#define MOTOR_TUNE_LINE_SIZE 64U
#define RAMP_HALF_DEFAULT_TICKS ((u16)(500.0f * CONTROL_TICKS_PER_MS))
#define OPEN_SWEEP_DEFAULT_TICKS ((u32)(2000.0f * CONTROL_TICKS_PER_MS))
#define OPEN_RAMP_DEFAULT_TICKS ((u32)(100.0f * CONTROL_TICKS_PER_MS))

static volatile u8 rx_buffer[MOTOR_TUNE_RX_SIZE];
static volatile u16 rx_head = 0;
static volatile u16 rx_tail = 0;
static char line_buffer[MOTOR_TUNE_LINE_SIZE];
static u16 line_length = 0;

static float display_alpha = 0.12f;
static float ref_min = 0.10f;
static float ref_max = 0.20f;
static u16 ramp_half_ticks = RAMP_HALF_DEFAULT_TICKS;
static u8 rotate_enable = 0U;
static float rotate_hz = 0.50f;
static float rotate_id = 0.20f;
static float track_angle_lpf = 0.0002f;
static float current_output_lpf = 1.0f;
static float current_deadband = 0.12f;
static u8 current_angle_hold = 1U;
static u8 speed_enable = 0U;
static float speed_rpm = 0.05f;
static float speed_ff = 0.45f;
static float speed_min_current = 0.40f;
static u8 open_enable = 0U;
static u8 open_angle_hold = 0U;
static float open_ud = 0.30f;
static u8 open_sweep = 0U;
static float open_ud_low = 0.15f;
static float open_ud_high = 0.30f;
static u32 open_sweep_ticks = OPEN_SWEEP_DEFAULT_TICKS;
static u32 open_ramp_ticks = OPEN_RAMP_DEFAULT_TICKS;
static float open_dither_hz = 0.0f;
static float open_dither_ud = 0.0f;

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
        ramp_half_ticks = (u16)(number * CONTROL_TICKS_PER_MS);
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
            speed_enable = 0U;
            open_enable = 0U;
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
    else if(key_equals(key, "TRACK_ANGLE_LPF") && number > 0.0f)
    {
        if(number > 1.0f)
        {
            number = 1.0f;
        }
        if(number < 0.00005f)
        {
            number = 0.00005f;
        }
        track_angle_lpf = number;
    }
    else if(key_equals(key, "CURRENT_OUT_LPF") && number > 0.0f)
    {
        if(number > 1.0f)
        {
            number = 1.0f;
        }
        if(number < 0.00005f)
        {
            number = 0.00005f;
        }
        current_output_lpf = number;
    }
    else if(key_equals(key, "CURRENT_DEADBAND") && number >= 0.0f)
    {
        if(number > 0.20f)
        {
            number = 0.20f;
        }
        current_deadband = number;
    }
    else if(key_equals(key, "CURRENT_ANGLE_HOLD"))
    {
        current_angle_hold = (number > 0.5f) ? 1U : 0U;
    }
    else if(key_equals(key, "SPEED_MODE"))
    {
        speed_enable = (number > 0.5f) ? 1U : 0U;
        if(speed_enable != 0U)
        {
            rotate_enable = 0U;
            open_enable = 0U;
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
    else if(key_equals(key, "OPEN_MODE"))
    {
        open_enable = (number > 0.5f) ? 1U : 0U;
        if(open_enable != 0U)
        {
            speed_enable = 0U;
            rotate_enable = 0U;
        }
    }
    else if(key_equals(key, "OPEN_UD") && number >= 0.0f)
    {
        if(number > 0.5f)
        {
            number = 0.5f;
        }
        open_ud = number;
    }
    else if(key_equals(key, "OPEN_ANGLE_HOLD"))
    {
        open_angle_hold = (number > 0.5f) ? 1U : 0U;
        if(open_angle_hold != 0U)
        {
            mc.e_angle.electrical_angle_set_pu =
                mc.e_angle.electrical_angle_pu;
        }
    }
    else if(key_equals(key, "OPEN_SWEEP"))
    {
        open_sweep = (number > 0.5f) ? 1U : 0U;
    }
    else if(key_equals(key, "OPEN_UD_LOW") && number >= 0.0f)
    {
        open_ud_low = number;
    }
    else if(key_equals(key, "OPEN_UD_HIGH") && number >= 0.0f)
    {
        open_ud_high = number;
    }
    else if(key_equals(key, "OPEN_SWEEP_MS") && number > 0.0f)
    {
        open_sweep_ticks = (u32)(number * CONTROL_TICKS_PER_MS);
        if(open_sweep_ticks < 100U)
        {
            open_sweep_ticks = 100U;
        }
    }
    else if(key_equals(key, "OPEN_RAMP_MS") && number >= 0.0f)
    {
        open_ramp_ticks = (u32)(number * CONTROL_TICKS_PER_MS);
    }
    else if(key_equals(key, "OPEN_DITHER_HZ") && number >= 0.0f)
    {
        if(number > 500.0f)
        {
            number = 500.0f;
        }
        open_dither_hz = number;
    }
    else if(key_equals(key, "OPEN_DITHER_UD") && number >= 0.0f)
    {
        if(number > 0.20f)
        {
            number = 0.20f;
        }
        open_dither_ud = number;
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
    ramp_half_ticks = RAMP_HALF_DEFAULT_TICKS;
    rotate_enable = 0U;
    rotate_hz = 0.50f;
    rotate_id = 0.20f;
    track_angle_lpf = 0.0002f;
    current_output_lpf = 1.0f;
    current_deadband = 0.12f;
    current_angle_hold = 1U;
    speed_enable = 0U;
    speed_rpm = 0.05f;
    speed_ff = 0.45f;
    speed_min_current = 0.40f;
    open_enable = 0U;
    open_angle_hold = 0U;
    open_ud = 0.30f;
    open_sweep = 0U;
    open_ud_low = 0.15f;
    open_ud_high = 0.30f;
    open_sweep_ticks = OPEN_SWEEP_DEFAULT_TICKS;
    open_ramp_ticks = OPEN_RAMP_DEFAULT_TICKS;
    open_dither_hz = 0.0f;
    open_dither_ud = 0.0f;
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

float motor_tune_get_track_angle_lpf(void)
{
    return track_angle_lpf;
}

float motor_tune_get_current_output_lpf(void)
{
    return current_output_lpf;
}

float motor_tune_get_current_deadband(void)
{
    return current_deadband;
}

u8 motor_tune_get_current_angle_hold(void)
{
    return current_angle_hold;
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

u8 motor_tune_get_open_enable(void)
{
    return open_enable;
}

u8 motor_tune_get_open_angle_hold(void)
{
    return open_angle_hold;
}

float motor_tune_get_open_ud(void)
{
    return open_ud;
}

u8 motor_tune_get_open_sweep(void)
{
    return open_sweep;
}

float motor_tune_get_open_ud_low(void)
{
    return open_ud_low;
}

float motor_tune_get_open_ud_high(void)
{
    return open_ud_high;
}

u32 motor_tune_get_open_sweep_ticks(void)
{
    return open_sweep_ticks;
}

u32 motor_tune_get_open_ramp_ticks(void)
{
    return open_ramp_ticks;
}

float motor_tune_get_open_dither_hz(void)
{
    return open_dither_hz;
}

float motor_tune_get_open_dither_ud(void)
{
    return open_dither_ud;
}
