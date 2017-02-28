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

#include "driver/adc.h"

static int adc_port_module_init(void)
{
    esp_adc1_config_width(ADC_WIDTH_12Bit);

    return (0);
}

static int adc_port_init(struct adc_driver_t *self_p,
                         struct adc_device_t *dev_p,
                         struct pin_device_t *pin_dev_p,
                         int reference,
                         long sampling_rate)
{
    if (pin_dev_p == &pin_device[32]) {
        /* GPIO36, ADC1, CH0. */
        self_p->channel = 0;
    } else if (pin_dev_p == &pin_device[33]) {
        /* GPIO37, ADC1, CH1. */
        self_p->channel = 1;
    } else if (pin_dev_p == &pin_device[34]) {
        /* GPIO38, ADC1, CH2. */
        self_p->channel = 2;
    } else if (pin_dev_p == &pin_device[35]) {
        /* GPIO39, ADC1, CH3. */
        self_p->channel = 3;
    } else if (pin_dev_p == &pin_device[28]) {
        /* GPIO32, ADC1, CH4. */
        self_p->channel = 4;
    } else if (pin_dev_p == &pin_device[29]) {
        /* GPIO33, ADC1, CH5. */
        self_p->channel = 5;
    } else if (pin_dev_p == &pin_device[30]) {
        /* GPIO34, ADC1, CH6. */
        self_p->channel = 6;
    } else if (pin_dev_p == &pin_device[31]) {
        /* GPIO35, ADC1, CH7. */
        self_p->channel = 7;
    } else {
        return (-1);
    }

    esp_adc1_config_channel_atten(self_p->channel, ADC_ATTEN_11db);
    self_p->dev_p = dev_p;

    return (0);
}

static int adc_port_async_convert(struct adc_driver_t *self_p,
                                  uint16_t *samples_p,
                                  size_t length)
{
    *samples_p = esp_adc1_get_voltage(self_p->channel);

    return (0);
}

static int adc_port_async_wait(struct adc_driver_t *self_p)
{
    return (0);
}

static int adc_port_convert_isr(struct adc_driver_t *self_p,
                                uint16_t *sample_p)
{
    return (-1);
}
