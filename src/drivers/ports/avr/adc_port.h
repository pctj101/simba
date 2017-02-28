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

#ifndef __DRIVERS_ADC_PORT_H__
#define __DRIVERS_ADC_PORT_H__

#include <avr/io.h>

#define ADC_PORT_REFERENCE_VCC _BV(REFS0)

struct adc_driver_t;

struct adc_device_t {
    struct {
        struct adc_driver_t *head_p;
        struct adc_driver_t *tail_p;
    } jobs;
};

struct adc_driver_t {
    struct adc_device_t *dev_p;
    struct pin_driver_t pin_drv;
    uint8_t admux;
#if defined(MUX5)
    uint8_t adcsrb;
#endif
    long interrupt_count;
    long interrupt_max;
    size_t pos;
    size_t length;
    uint16_t *samples_p;
    struct thrd_t *thrd_p;
    struct adc_driver_t *next_p;
};

#endif
