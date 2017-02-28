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

static int test_bootloader(struct harness_t *self_p)
{
    BTASSERT(upgrade_bootloader_enter() == -1);
    BTASSERT(upgrade_bootloader_stay_set() == 0);
    BTASSERT(upgrade_bootloader_stay_get() == 1);
    BTASSERT(upgrade_bootloader_stay_clear() == 0);
    BTASSERT(upgrade_bootloader_stay_get() == 0);

    return (0);
}

static int test_binary_upload(struct harness_t *self_p)
{
    uint8_t header_data_size_2[64] = {
        /* Version. */
        0, 0, 0, 1,
        /* Header size. */
        0, 0, 0, 40,
        /* Data size. */
        0, 0, 0, 2,
        /* Data SHA1. */
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 0, 1, 2, 3, 4, 5, 6,
        7, 8, 9, 0,
        /* Data description. */
        'f', 'o', 'o', '\0',
        /* Header CRC. */
        0x91, 0x9e, 0x50, 0x12,
        /* Data. */
        'a', 'b'
    };

    BTASSERT(upgrade_binary_upload_begin() == 0);
    BTASSERT(upgrade_binary_upload(&header_data_size_2[0], 42) == 0);
    BTASSERT(upgrade_binary_upload_end() == 0);

    return (0);
}

static int test_binary_upload_bad_version(struct harness_t *self_p)
{
    uint8_t buf[42] = {
        /* Version. */
        0, 0, 0, 2,
        /* Header size. */
        0, 0, 0, 40,
        /* Data size. */
        0, 0, 0, 2,
        /* Data SHA1. */
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 0, 1, 2, 3, 4, 5, 6,
        7, 8, 9, 0,
        /* Data description. */
        'f', 'o', 'o', '\0',
        /* Header CRC. */
        0x91, 0x9e, 0x50, 0x12,
        /* Data. */
        'a', 'b'
    };

    BTASSERT(upgrade_binary_upload_begin() == 0);
    BTASSERT(upgrade_binary_upload(&buf[0], 42) == -1);
    BTASSERT(upgrade_binary_upload_end() == 0);

    return (0);
}

static int test_binary_upload_bad_crc(struct harness_t *self_p)
{
    uint8_t buf[42] = {
        /* Version. */
        0, 0, 0, 1,
        /* Header size. */
        0, 0, 0, 40,
        /* Data size. */
        0, 0, 0, 2,
        /* Data SHA1. */
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 0, 1, 2, 3, 4, 5, 6,
        7, 8, 9, 0,
        /* Data description. */
        'f', 'o', 'o', '\0',
        /* Header CRC. */
        0x91, 0x9e, 0x50, 0x11,
        /* Data. */
        'a', 'b'
    };

    BTASSERT(upgrade_binary_upload_begin() == 0);
    BTASSERT(upgrade_binary_upload(&buf[0], 42) == -1);
    BTASSERT(upgrade_binary_upload_end() == 0);

    return (0);
}

static int test_binary_upload_short_header(struct harness_t *self_p)
{
    uint8_t buf[39] = {
        /* Version. */
        0, 0, 0, 1,
        /* Header size. */
        0, 0, 0, 39,
        /* Data size. */
        0, 0, 0, 2,
        /* Data SHA1. */
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 0, 1, 2, 3, 4, 5, 6,
        7, 8, 9, 0,
        /* Data description. */
        '\0',
        /* Header CRC. */
        0x56, 0x45, 0x27, 0x1f
    };

    BTASSERT(upgrade_binary_upload_begin() == 0);
    BTASSERT(upgrade_binary_upload(&buf[0], 39) == -1);
    BTASSERT(upgrade_binary_upload_end() == 0);

    return (0);
}

static int test_binary_upload_long_header(struct harness_t *self_p)
{
    uint8_t buf[40] = {
        /* Version. */
        0, 0, 0, 1,
        /* Header size too big. */
        0, 1, 0, 0,
        /* Data size. */
        0, 0, 0, 2,
        /* Data SHA1. */
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 0, 1, 2, 3, 4, 5, 6,
        7, 8, 9, 0,
        /* Data description. */
        'f', 'o', 'o', '\0',
        /* Header CRC. */
        0x56, 0x45, 0x27, 0x1f
    };

    BTASSERT(upgrade_binary_upload_begin() == 0);
    BTASSERT(upgrade_binary_upload(&buf[0], 40) == -1);
    BTASSERT(upgrade_binary_upload_end() == 0);

    return (0);
}

int main()
{
    struct harness_t harness;
    struct harness_testcase_t harness_testcases[] = {
        { test_bootloader, "test_bootloader" },
        { test_binary_upload, "test_binary_upload" },
        { test_binary_upload_bad_version, "test_binary_upload_bad_version" },
        { test_binary_upload_bad_crc, "test_binary_upload_bad_crc" },
        { test_binary_upload_short_header, "test_binary_upload_short_header" },
        { test_binary_upload_long_header, "test_binary_upload_long_header" },
        { NULL, NULL }
    };

    sys_start();

    harness_init(&harness);
    harness_run(&harness, harness_testcases);

    return (0);
}
