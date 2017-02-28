/**
 * @section License
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 *
 * This file is part of the Simba project.
 */

#include "simba.h"

struct pwm_soft_driver_t pwm_soft[2];

static int test_init(struct harness_t *harness_p)
{
    BTASSERT(pwm_soft_module_init(1000) == 0);

    BTASSERT(pwm_soft_init(&pwm_soft[0],
                           &pin_d2_dev,
                           pwm_soft_duty_cycle(10)) == 0);

    BTASSERT(pwm_soft_init(&pwm_soft[1],
                           &pin_d5_dev,
                           pwm_soft_duty_cycle(95)) == 0);

    return (0);
}

static int test_duty_cycles(struct harness_t *harness_p)
{
    int percentage;
    long duty_cycle;

    BTASSERT(pwm_soft_start(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_start(&pwm_soft[1]) == 0);
    thrd_sleep_ms(100);

    /* Test various duty cycles. */
    for (percentage = 0; percentage <= 100; percentage += 10) {
        duty_cycle = pwm_soft_duty_cycle(percentage);

        BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[0], duty_cycle) == 0);
        BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[1], duty_cycle) == 0);
        
        std_printf(FSTR("Duty cycle: %ld\r\n"), duty_cycle);
        thrd_sleep_ms(100);
    }
    
    BTASSERT(pwm_soft_stop(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_stop(&pwm_soft[1]) == 0);

    return (0);
}

static int test_duty_cycle_min(struct harness_t *harness_p)
{
    long duty_cycle;

    BTASSERT(pwm_soft_start(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_start(&pwm_soft[1]) == 0);

    /* Always low for min duty cycle. */
    duty_cycle = pwm_soft_duty_cycle(0);
    
    BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[0], duty_cycle) == 0);
    BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[1], duty_cycle) == 0);

    thrd_sleep_ms(150);

    BTASSERT(pwm_soft_stop(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_stop(&pwm_soft[1]) == 0);

    return (0);
}

static int test_duty_cycle_max(struct harness_t *harness_p)
{
    long duty_cycle;
    
    BTASSERT(pwm_soft_start(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_start(&pwm_soft[1]) == 0);

    /* Always high for max duty cycle. */
    duty_cycle = pwm_soft_duty_cycle(100);
    BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[0], duty_cycle) == 0);
    BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[1], duty_cycle) == 0);
    
    thrd_sleep_ms(50);

    BTASSERT(pwm_soft_stop(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_stop(&pwm_soft[1]) == 0);

    return (0);
}

static int test_frequency(struct harness_t *harness_p)
{
    long frequency;

    for (frequency = 100; frequency < 10000; frequency += 1000) {
        /* Set the new frequency. */
        BTASSERT(pwm_soft_set_frequency(frequency) == 0);

        /* Run the PWM. */
        BTASSERT(pwm_soft_start(&pwm_soft[0]) == 0);
        BTASSERT(pwm_soft_set_duty_cycle(&pwm_soft[0],
                                         pwm_soft_duty_cycle(30)) == 0);
        thrd_sleep_ms(50);
        BTASSERT(pwm_soft_stop(&pwm_soft[0]) == 0);
    }

    /* May not change frequency when a PWM is started. */
    BTASSERT(pwm_soft_start(&pwm_soft[0]) == 0);
    BTASSERT(pwm_soft_set_frequency(frequency) == -1);
    BTASSERT(pwm_soft_stop(&pwm_soft[0]) == 0);
    
    return (0);
}

static int test_duty_cycle_convert(struct harness_t *harness_p)
{
    BTASSERT(pwm_soft_duty_cycle_as_percent(pwm_soft_duty_cycle(0)) == 0);
    BTASSERT(pwm_soft_duty_cycle_as_percent(pwm_soft_duty_cycle(100)) == 100);

    return (0);
}

int main()
{
    struct harness_t harness;
    struct harness_testcase_t harness_testcases[] = {
        { test_init, "test_init" },
        { test_duty_cycles, "test_duty_cycles" },
        { test_duty_cycle_min, "test_duty_cycle_min" },
        { test_duty_cycle_max, "test_duty_cycle_max" },
        { test_frequency, "test_frequency" },
        { test_duty_cycle_convert, "test_duty_cycle_convert" },
        { NULL, NULL }
    };

    sys_start();

    harness_init(&harness);
    harness_run(&harness, harness_testcases);

    return (0);
}
