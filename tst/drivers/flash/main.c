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

#if defined(BOARD_ARDUINO_DUE)

static int test_read_write(struct harness_t *harness_p)
{
    struct flash_driver_t drv;
    char name[] = "Kalle kula";
    char buf[16];
    uint32_t address;

    BTASSERT(flash_init(&drv, &flash_0_dev) == 0);

    /* Write and read over a page boundary. */
    address = (FLASH1_BEGIN + FLASH1_PAGE_SIZE - 2);

    BTASSERT(flash_write(&drv, address, name, sizeof(name)) == sizeof(name));

    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv, buf, address, sizeof(buf)) == sizeof(buf));

    BTASSERT(strcmp(name, buf) == 0);

    /* Write and read over a bank boundary. */
    address = (FLASH1_BEGIN - 2);

    BTASSERT(flash_write(&drv, address, name, sizeof(name)) == sizeof(name));

    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv, buf, address, sizeof(buf)) == sizeof(buf));

    BTASSERT(strcmp(name, buf) == 0);

    return (0);
}

#elif defined(ARCH_ESP) || defined(ARCH_ESP32)

extern char __rom_size;

static int test_read_write(struct harness_t *harness_p)
{
    struct flash_driver_t drv;
    char buf[32];
    uintptr_t flash_address;
    uintptr_t ram_address;

    /* Aligned start address*/
    flash_address = ((uintptr_t)&__rom_size - 0x10000);
    ram_address = (uintptr_t)&buf[0];
    ram_address &= 0xfffffffc;
    
    BTASSERT(flash_init(&drv, &flash_0_dev) == 0);
    BTASSERT(flash_erase(&drv, flash_address, 0x10000) == 0);

    /* Read and write aligned in ROM and RAM. */
    strcpy((void *)ram_address, "1234");
    BTASSERT(flash_write(&drv, flash_address, (void *)ram_address, 5) == 5);
    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv, buf, flash_address, 5) == 5);
    BTASSERT(strcmp(buf, "1234") == 0);

    /* Read and write non-aligned in ROM and aligned RAM. One is added
       to the ram address to align the chunk reads. */
    strcpy((void *)ram_address, "67890123");
    flash_address += 5;
    BTASSERT(flash_write(&drv, flash_address, (void *)ram_address, 9) == 9);
    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv,
                        (uint32_t *)(ram_address + 1),
                        flash_address, 9) == 9);
    BTASSERT(strcmp((void *)(ram_address + 1), "67890123") == 0);

    /* Read and write aligned in ROM and non-aligned RAM. */
    ram_address++;
    strcpy((void *)ram_address, "abcdefghij");
    flash_address += 19; /* 5 + 19 = 24. */
    BTASSERT(flash_write(&drv, flash_address, (void *)ram_address, 11) == 11);
    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv,
                        (uint32_t *)ram_address,
                        flash_address, 11) == 11);
    BTASSERT(strcmp((void *)ram_address, "abcdefghij") == 0);

    /* Read and write 1 byte at non-aligned ROM address. */
    ram_address = (uintptr_t)&buf[0];
    strcpy((void *)ram_address, "");
    flash_address++;
    BTASSERT(flash_write(&drv, flash_address, (void *)ram_address, 1) == 1);
    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv,
                        (uint32_t *)ram_address,
                        flash_address, 1) == 1);
    BTASSERT(strcmp((void *)ram_address, "") == 0);

    return (0);
}

#elif defined(BOARD_SPC56DDISCOVERY)

static int test_read_write(struct harness_t *harness_p)
{
    struct flash_driver_t drv;
    char name[] = "Kalle kula";
    char buf[16];
    uint32_t address;

    BTASSERT(flash_init(&drv, &flash_device[1]) == 0);

    /* Write and read over a 8 bytes boundary. */
    address = (SPC5_DFLASH_ADDRESS + 7);

    BTASSERT(flash_erase(&drv, address, sizeof(name)) == 0);
    BTASSERT(flash_write(&drv, address, name, sizeof(name)) == sizeof(name));

    memset(buf, 0, sizeof(buf));
    BTASSERT(flash_read(&drv, buf, address, sizeof(buf)) == sizeof(buf));

    BTASSERT(strcmp(name, buf) == 0);

    return (0);
}

#else

static int test_read_write(struct harness_t *harness_p)
{
    return (1);
}

#endif

int main()
{
    struct harness_t harness;
    struct harness_testcase_t harness_testcases[] = {
        { test_read_write, "test_read_write" },
        { NULL, NULL }
    };

    sys_start();
    flash_module_init();

    harness_init(&harness);
    harness_run(&harness, harness_testcases);

    return (0);
}
