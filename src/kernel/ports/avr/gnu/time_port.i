/**
 * @file time_port.i
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

#include <util/delay_basic.h>

#define I_CPU (F_CPU / 1000000L)

static int time_port_get(struct time_t *now)
{
    now->seconds = sys.tick;

    return (0);
}

static int time_port_set(struct time_t *now)
{
    return (-1);
}

static void time_port_sleep(int us)
{
    _delay_loop_2((us * I_CPU) / 4);
}
