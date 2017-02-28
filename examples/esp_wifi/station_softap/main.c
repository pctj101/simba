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

int main()
{
    sys_start();

    /* Configure and start the the WiFi module as a Station and
       SoftAP. */
    esp_wifi_set_op_mode(esp_wifi_op_mode_station_softap_t);

    if (esp_wifi_softap_init("Simba", NULL) != 0) {
        std_printf(FSTR("Failed to configure the Soft AP.\r\n"));
    }

    if (esp_wifi_station_init("Qvist2", "maxierik", NULL) != 0) {
        std_printf(FSTR("Failed to configure the Station.\r\n"));
    }

    while (1) {
        esp_wifi_print(sys_get_stdout());
        thrd_sleep(2);
    }

    return (0);
}
