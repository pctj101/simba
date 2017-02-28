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

#if CONFIG_I2C == 1

struct module_t {
    int initialized;
#if CONFIG_FS_CMD_I2C_READ == 1
    struct fs_command_t cmd_read;
#endif
#if CONFIG_FS_CMD_I2C_WRITE == 1
    struct fs_command_t cmd_write;
#endif
};

#include "i2c_port.i"

static struct module_t module;

#if CONFIG_FS_CMD_I2C_READ == 1

static int cmd_read_cb(int argc,
                       const char *argv[],
                       void *chout_p,
                       void *chin_p,
                       void *arg_p,
                       void *call_arg_p)
{
    struct i2c_driver_t i2c;
    long value;
    int slave_address;
    uint8_t data;

    if (argc != 2) {
        std_printf(OSTR("Usage: %s <slave address>\r\n"), argv[0]);

        return (-EINVAL);
    }

    if (std_strtol(argv[1], &value) == NULL) {
        std_printf(OSTR("Bad slave address.\r\n"));

        return (-EINVAL);
    }

    slave_address = value;

    i2c_init(&i2c, &i2c_0_dev, I2C_BAUDRATE_100KBPS, -1);
    i2c_start(&i2c);

    if (i2c_read(&i2c, slave_address, &data, 1) != 1) {
        std_printf(OSTR("Failed to read from slave device 0x%02x.\r\n"),
                   slave_address);
        goto out;
    }

    std_printf(OSTR("0x%02x\r\n"), data);

 out:
    i2c_stop(&i2c);

    return (0);
}

#endif

#if CONFIG_FS_CMD_I2C_WRITE == 1

static int cmd_write_cb(int argc,
                        const char *argv[],
                        void *chout_p,
                        void *chin_p,
                        void *arg_p,
                        void *call_arg_p)
{
    struct i2c_driver_t i2c;
    long value;
    int slave_address;
    uint8_t data;

    if (argc != 3) {
        std_printf(OSTR("Usage: %s <slave address> <data byte>\r\n"),
                   argv[0]);

        return (-EINVAL);
    }

    if (std_strtol(argv[1], &value) == NULL) {
        std_printf(OSTR("Bad slave address.\r\n"));

        return (-EINVAL);
    }

    slave_address = value;

    if (std_strtol(argv[2], &value) == NULL) {
        std_printf(OSTR("Bad data value.\r\n"));

        return (-EINVAL);
    }

    if ((value > 255) || (value < 0)) {
        std_printf(OSTR("Data byte value out of range.\r\n"));

        return (-EINVAL);
    }

    data = value;

    i2c_init(&i2c, &i2c_0_dev, I2C_BAUDRATE_100KBPS, -1);
    i2c_start(&i2c);

    if (i2c_write(&i2c, slave_address, &data, 1) != 1) {
        std_printf(OSTR("Failed to write data 0x%02x to slave"
                         " device 0x%02x.\r\n"),
                   data,
                   slave_address);
    }

    i2c_stop(&i2c);

    return (0);
}

#endif

int i2c_module_init()
{
    /* Return immediately if the module is already initialized. */
    if (module.initialized == 1) {
        return (0);
    }

    module.initialized = 1;

#if CONFIG_FS_CMD_I2C_READ == 1

    fs_command_init(&module.cmd_read,
                    CSTR("/drivers/i2c/read"),
                    cmd_read_cb,
                    NULL);
    fs_command_register(&module.cmd_read);

#endif

#if CONFIG_FS_CMD_I2C_WRITE == 1

    fs_command_init(&module.cmd_write,
                    CSTR("/drivers/i2c/write"),
                    cmd_write_cb,
                    NULL);
    fs_command_register(&module.cmd_write);

#endif

    return (i2c_port_module_init());
}

int i2c_init(struct i2c_driver_t *self_p,
             struct i2c_device_t *dev_p,
             int baudrate,
             int address)
{
    ASSERTN(self_p != NULL, EINVAL);
    ASSERTN(dev_p != NULL, EINVAL);

    return (i2c_port_init(self_p, dev_p, baudrate, address));
}

int i2c_start(struct i2c_driver_t *self_p)
{
    ASSERTN(self_p != NULL, EINVAL);

    return (i2c_port_start(self_p));
}

int i2c_stop(struct i2c_driver_t *self_p)
{
    ASSERTN(self_p != NULL, EINVAL);

    return (i2c_port_stop(self_p));
}

ssize_t i2c_read(struct i2c_driver_t *self_p,
                 int address,
                 void *buf_p,
                 size_t size)
{
    ASSERTN(self_p != NULL, EINVAL);
    ASSERTN(address < 128, EINVAL);
    ASSERTN(buf_p != NULL, EINVAL);
    ASSERTN(size > 0, EINVAL);

    return (i2c_port_read(self_p, address, buf_p, size));
}

ssize_t i2c_write(struct i2c_driver_t *self_p,
                  int address,
                  const void *buf_p,
                  size_t size)
{
    ASSERTN(self_p != NULL, EINVAL);
    ASSERTN(address < 128, EINVAL);
    ASSERTN(buf_p != NULL, EINVAL);
    ASSERTN(size > 0, EINVAL);

    return (i2c_port_write(self_p, address, buf_p, size));
}

int i2c_scan(struct i2c_driver_t *self_p, int address)
{
    ASSERTN(self_p != NULL, EINVAL);
    ASSERTN(address < 128, EINVAL);

    return (i2c_port_scan(self_p, address));
}

int i2c_slave_start(struct i2c_driver_t *self_p)
{
    ASSERTN(self_p != NULL, EINVAL);

    return (i2c_port_slave_start(self_p));
}

int i2c_slave_stop(struct i2c_driver_t *self_p)
{
    ASSERTN(self_p != NULL, EINVAL);

    return (i2c_port_slave_stop(self_p));
}

ssize_t i2c_slave_read(struct i2c_driver_t *self_p,
                       void *buf_p,
                       size_t size)
{
    ASSERTN(self_p != NULL, EINVAL);
    ASSERTN(buf_p != NULL, EINVAL);
    ASSERTN(size > 0, EINVAL);

    return (i2c_port_slave_read(self_p, buf_p, size));
}

ssize_t i2c_slave_write(struct i2c_driver_t *self_p,
                        const void *buf_p,
                        size_t size)
{
    ASSERTN(self_p != NULL, EINVAL);
    ASSERTN(buf_p != NULL, EINVAL);
    ASSERTN(size > 0, EINVAL);

    return (i2c_port_slave_write(self_p, buf_p, size));
}

#endif
