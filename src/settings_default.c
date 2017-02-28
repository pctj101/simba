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

/**
 * This file was generated by settings.py 1.0 2016-08-08 14:50 CEST.
 */

#include "simba.h"

const FAR struct setting_t settings[]
__attribute__ ((weak)) = {
    { NULL, 0, 0, 0 }
};

#if defined(ARCH_AVR)

uint8_t settings_area[CONFIG_SETTINGS_AREA_SIZE]
__attribute__ ((section (".eeprom"), weak)) = {
    0xff,
};

const FAR uint8_t settings_default_area[CONFIG_SETTINGS_AREA_SIZE]
__attribute__ ((weak)) = {
    0xff,
};

#elif defined(ARCH_ARM)

uint8_t settings_area[2][CONFIG_SETTINGS_AREA_SIZE]
__attribute__ ((section (".settings"), weak)) = {
    {
        0xff,
    },
    {
        0xff,
    }
};

uint8_t settings_default_area[CONFIG_SETTINGS_AREA_SIZE]
__attribute__ ((section (".settings"), weak)) = {
    0xff,
};

#else

const uint8_t settings_default_area[CONFIG_SETTINGS_AREA_SIZE]
__attribute__ ((weak)) = {
    0xff,
};

#endif
