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

#ifndef __DRIVERS_ANALOG_OUTPUT_PIN_H__
#define __DRIVERS_ANALOG_OUTPUT_PIN_H__

#include "simba.h"

struct analog_output_pin_t {
    struct pwm_driver_t pwm;
};

/**
 * Initialize the analog output pin module. This function must be
 * called before calling any other function in this module.
 *
 * The module will only be initialized once even if this function is
 * called multiple times.
 *
 * @return zero(0) or negative error code.
 */
int analog_output_pin_module_init(void);

/**
 * Initialize given driver object with given device and mode.
 *
 * @param[out] self_p Driver object to be initialized.
 * @param[in] dev_p Device to use.
 *
 * @return zero(0) or negative error code.
 */
int analog_output_pin_init(struct analog_output_pin_t *self_p,
                           struct pin_device_t *dev_p);

/**
 * Write given value to the analog pin.
 *
 * @param[in] self_p Driver object.
 * @param[in] value The value to write to the pin. A number in the
 *                  range 0 to 1023, where 0 is lowest output and 1023
 *                  is highest output.
 *
 * @return zero(0) or negative error code.
 */
int analog_output_pin_write(struct analog_output_pin_t *self_p,
                            int value);

/**
 * Read the value that is currently written to given analog output
 * pin.
 *
 * @param[in] self_p Driver object.
 *
 * @return Value in the range 0 to 1023, or negative error code.
 */
int analog_output_pin_read(struct analog_output_pin_t *self_p);

#endif
