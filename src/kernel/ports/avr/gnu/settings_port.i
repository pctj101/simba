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

#include <avr/eeprom.h>

/* The array is generated by settings.py. */
extern uint8_t settings_area[CONFIG_SETTINGS_AREA_SIZE]
__attribute__ ((section (".eeprom")));

extern const FAR uint8_t settings_default_area[CONFIG_SETTINGS_AREA_SIZE];

static int settings_port_module_init(void)
{
    return (0);
}

static int settings_port_read(void *dst_p, size_t src, size_t size)
{
    eeprom_read_block(dst_p, &settings_area[src], size);

    return (size);
}

static int settings_port_write(size_t dst, const void *src_p, size_t size)
{
    eeprom_update_block(src_p, &settings_area[dst], size);

    return (size);
}

static ssize_t settings_port_reset()
{
    int i;

    for (i = 0; i < sizeof(settings_default_area); i++) {
        eeprom_update_byte(&settings_area[i], settings_default_area[i]);
    }

    return (0);
}
