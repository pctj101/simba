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

struct pin_device_t pin_device[PIN_DEVICE_MAX] = {
    /* PA0-15. */
    { .id = 0 },
    { .id = 1 },
    { .id = 2 },
    { .id = 3 },
    { .id = 4 },
    { .id = 5 },
    { .id = 6 },
    { .id = 7 },
    { .id = 8 },
    { .id = 9 },
    { .id = 10 },
    { .id = 11 },
    { .id = 12 },
    { .id = 13 },
    { .id = 14 },
    { .id = 15 },

    /* PB0-15. */
    { .id = 16 },
    { .id = 17 },
    { .id = 18 },
    { .id = 19 },
    { .id = 20 },
    { .id = 21 },
    { .id = 22 },
    { .id = 23 },
    { .id = 24 },
    { .id = 25 },
    { .id = 26 },
    { .id = 27 },
    { .id = 28 },
    { .id = 29 },
    { .id = 30 },
    { .id = 31 },

    /* PC0-10. */
    { .id = 32 },
    { .id = 33 },
    { .id = 34 },
    { .id = 35 },
    { .id = 36 },
    { .id = 37 },
    { .id = 38 },
    { .id = 39 },
    { .id = 40 },
    { .id = 41 },
    { .id = 42 }
};

struct uart_device_t uart_device[UART_DEVICE_MAX] = {
    { .drv_p = NULL, .regs_p = SPC5_LINFLEX_0 },
    { .drv_p = NULL, .regs_p = SPC5_LINFLEX_1 },
    { .drv_p = NULL, .regs_p = SPC5_LINFLEX_2 }
};

static int32_t cflash_sector_sizes[] = {
    0x8000,
    0x4000,
    0x4000,
    0x8000,
    0x8000,
    0x20000,
    -1
};

static int32_t dflash_sector_sizes[] = {
    0x4000,
    0x4000,
    0x4000,
    0x4000,
    -1
};

struct flash_device_t flash_device[FLASH_DEVICE_MAX] = {
    {
        .regs_p = SPC5_CFLASH,
        .address = SPC5_CFLASH_ADDRESS,
        .size = SPC5_CFLASH_SIZE,
        .sector_sizes_p = cflash_sector_sizes,
        .program_size = 2
    },
    {
        .regs_p = SPC5_DFLASH,
        .address = SPC5_DFLASH_ADDRESS,
        .size = SPC5_DFLASH_SIZE,
        .sector_sizes_p = dflash_sector_sizes,
        .program_size = 1
    }
};

struct can_device_t can_device[CAN_DEVICE_MAX] = {
    { .regs_p = SPC5_FLEXCAN_0 }
};
