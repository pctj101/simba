/**
 * @file drivers/linux/spi_port.i
 * @version 1.0
 *
 * @section License
 * Copyright (C) 2014-2015, Erik Moqvist
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * This file is part of the Simba project.
 */

static int spi_port_start(struct spi_driver_t *drv_p)
{
    return (0);
}

static int spi_port_stop(struct spi_driver_t *drv_p)
{
    return (0);
}

static ssize_t spi_port_transfer(struct spi_driver_t *drv,
                                void *rxbuf,
                                const void *txbuf,
                                size_t n)
{
    return (-1);
}