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

#ifndef __DRIVERS_WATCHDOG_H__
#define __DRIVERS_WATCHDOG_H__

#include "simba.h"

#include "watchdog_port.h"

/**
 * Initialize the watchdog driver module. This function must be called
 * before calling any other function in this module.
 *
 * The module will only be initialized once even if this function is
 * called multiple times.
 *
 * @return zero(0) or negative error code.
 */
int watchdog_module_init(void);

/**
 * Start the watchdog with given timeout. Use `watchdog_kick()` to
 * periodically restart the timer.
 *
 * @param[in] timeout Watchdog timeout in milliseconds.
 *
 * @return zero(0) or negative error code.
 */
int watchdog_start_ms(int timeout);

/**
 * Stop the watchdog.
 *
 * @return zero(0) or negative error code.
 */
int watchdog_stop(void);

/**
 * Kick the watchdog. Restarts the watchdog timer with its original
 * timeout given to `watchdog_start_ms()`. The board will be reset if
 * this function is not called before the watchdog timer expires.
 *
 * @return zero(0) or negative error code.
 */
int watchdog_kick(void);

#endif
