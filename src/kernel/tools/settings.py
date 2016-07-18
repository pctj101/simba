#!/usr/bin/env python

import sys
import time
import struct
import re
import zlib
import argparse
import os

from ConfigParser import ConfigParser
from collections import OrderedDict


MAJOR=1
MINOR=0

SETTINGS_H_FILENAME = "settings.h"
SETTINGS_C_FILENAME = "settings.c"
SETTINGS_BIN_FILENAME = "settings.bin"

HEADER_FMT = """/**
 * @file {filename}
 * @version 2.0.0
 *
 * @section License
 * Copyright (C) 2014-2016, Erik Moqvist
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

/**
 * This file was generated by setting.py {major}.{minor} {date}.
 */

#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#define SETTING_AREA_OFFSET     {setting_offset}
#define SETTING_AREA_SIZE       {setting_size}
#define SETTING_AREA_CRC_OFFSET (SETTING_AREA_SIZE - 4)

{addresses}

{sizes}

{types}

{values}

#endif
"""

SOURCE_FMT = """/**
 * @file {filename}
 * @version 2.0.0
 *
 * @section License
 * Copyright (C) 2014-2016, Erik Moqvist
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

/**
 * This file was generated by setting.py {major}.{minor} {date}.
 */

#include "simba.h"

#if defined(SETTING_MEMORY_EEPROM)

uint8_t setting_area[SETTING_AREA_OFFSET + SETTING_AREA_SIZE] __attribute__ ((section (".eeprom"))) = {{
    {content}
}};

#else

uint8_t setting_area[2][SETTING_AREA_OFFSET + SETTING_AREA_SIZE] __attribute__ ((section (".setting"))) = {{
    {{
        {content}
    }},
    {{
        {content}
    }}
}};

#endif

"""

re_integer = re.compile(r"(?P<sign>[u]?)int(?P<bits>\d+)_t")


def parse_setting_file(filename):
    setting_parser = ConfigParser()
    setting_parser.read(filename)

    addresses = []
    sizes = []
    values = []
    types = []

    for item in setting_parser.items("addresses"):
        addresses.append((item[0], int(item[1], 0)))

    for item in setting_parser.items("sizes"):
        sizes.append((item[0], int(item[1], 0)))

    for item in setting_parser.items("types"):
        types.append((item[0], item[1]))

    for item in setting_parser.items("values"):
        values.append((item[0], item[1]))

    addresses.sort(key=lambda item: item[1])

    return addresses, sizes, types, values


def create_setting_dict(addresses, sizes, types, values):
    setting = OrderedDict()

    for name, address in addresses:
        setting[name] = {"address": address}

    for name, size in sizes:
        if name not in setting:
            sys.stderr.write("{}: no address for setting\n".format(name))
            sys.exit(1)
        setting[name]["size"] = size

    for name, type in types:
        if name not in setting:
            sys.stderr.write("{}: no address for setting\n".format(name))
            sys.exit(1)
        setting[name]["type"] = type

    for name, value in values:
        if name not in setting:
            sys.stderr.write("{}: no address for setting\n".format(name))
            sys.exit(1)
        setting[name]["value"] = value

    return setting


def create_binary_content(setting, setting_memory, setting_size, endianess):
    endianess_prefix = ">" if endianess == "big" else "<"

    # create the setting file content
    content = ""

    for name, item in setting.items():
        # add padding between previous setting and this one
        content += "\xff" * (item["address"] - len(content))
        # add the value
        if item["type"] == "string":
            if item["size"] <= len(item["value"]):
                sys.stderr.write("{}: value does not fit in size {}\n".format(item["value"],
                                                                              item["size"]))
                sys.exit(1)
            content += item["value"]
            # null termination
            content += "\x00"
        elif re_integer.match(item["type"]):
            bits_to_fmt = {
                8: "b",
                16: "h",
                32: "i",
                64: "q"
            }
            mo = re_integer.match(item["type"])
            sign = mo.group("sign")
            bits = int(mo.group("bits"))
            if bits not in [8, 16, 32, 64]:
                sys.stderr.write("{}: bad type\n".format(item["type"]))
                sys.exit(1)
            if bits / 8 != item["size"]:
                sys.stderr.write("{}: bad length of {}\n".format(item["size"],
                                                                 item["type"]))
                sys.exit(1)
            fmt = bits_to_fmt[bits]
            if sign == "u":
                fmt.upper()
            content += struct.pack(endianess_prefix + fmt, int(item["value"], 0))
        else:
            sys.stderr.write("{}: bad type\n".format(item["type"]))
            sys.exit(1)

    if len(content) > setting_size:
        fmt = "Settings area of size {} does not fit in memory of size {}.\n"
        sys.stderr.write(fmt.format(len(content), setting_size))
        sys.exit(1)

    if setting_memory == "eeprom":
        content += '\xff' * (setting_size - len(content))
    else:
        # pad the rest of the area and calculate a crc32
        content += '\xff' * (setting_size - 4 - len(content))
        crc = (zlib.crc32(content) & 0xffffffff)
        content += struct.pack(endianess_prefix + 'I', crc)

    return content


def create_header_file(outdir,
                       setting_offset,
                       setting_size,
                       setting):

    addresses = []
    sizes = []
    types = []
    values = []

    for name, item in setting.items():
        addresses.append("#define SETTING_{name}_ADDR (SETTING_AREA_OFFSET + {value})"
                         .format(name=name.upper(), value=item["address"]))
        sizes.append("#define SETTING_{name}_SIZE {value}"
                     .format(name=name.upper(), value=item["size"]))
        types.append("#define SETTING_{name}_TYPE {value}"
                     .format(name=name.upper(), value=item["type"]))
        values.append("#define SETTING_{name}_VALUE {value}"
                      .format(name=name.upper(), value=item["value"]))

    now = time.strftime("%Y-%m-%d %H:%M %Z")

    # write to setting header file
    with open(os.path.join(outdir, SETTINGS_H_FILENAME), "w") as fout:
        fout.write(HEADER_FMT.format(filename=SETTINGS_H_FILENAME,
                                     date=now,
                                     major=MAJOR,
                                     minor=MINOR,
                                     setting_offset=setting_offset,
                                     setting_size=setting_size,
                                     addresses="\n".join(addresses),
                                     sizes="\n".join(sizes),
                                     types="\n".join(types),
                                     values="\n".join(values)))


def create_binary_file(outdir, setting_offset, content):
    # write the content to the setting file
    with open(os.path.join(outdir, SETTINGS_BIN_FILENAME), "wb") as fout:
        content = ("\xff" * setting_offset + content)
        fout.write(content)


def create_source_file(outdir,
                       setting_memory,
                       setting_offset,
                       setting_size,
                       content):
    now = time.strftime("%Y-%m-%d %H:%M %Z")

    content = ("\xff" * setting_offset + content)
    content_bytes = ['{:#04x}'.format(ord(byte)) for byte in content]

    # write to setting source file
    with open(os.path.join(outdir, SETTINGS_C_FILENAME), "w") as fout:
        section = "eeprom"
        if setting_memory == "flash":
            section = "setting"
        fout.write(SOURCE_FMT.format(filename=SETTINGS_C_FILENAME,
                                     date=now,
                                     major=MAJOR,
                                     minor=MINOR,
                                     section=section,
                                     setting_size=(setting_offset + setting_size),
                                     content=', '.join(content_bytes)))
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--header", action="store_true")
    parser.add_argument("--binary", action="store_true")
    parser.add_argument("--source", action="store_true")
    parser.add_argument("--output-directory", default=".")
    parser.add_argument("--setting-memory", default="eeprom")
    parser.add_argument("--setting-offset", default=0)
    parser.add_argument("--setting-size", default=512)
    parser.add_argument("settings")
    parser.add_argument("endianess")

    args = parser.parse_args()

    endianess = args.endianess
    items = parse_setting_file(args.settings)
    setting = create_setting_dict(*items)

    setting_offset = int(args.setting_offset)
    setting_size = int(args.setting_size)

    if args.header:
        create_header_file(args.output_directory,
                           setting_offset,
                           setting_size,
                           setting)

    content = create_binary_content(setting,
                                    args.setting_memory,
                                    setting_size,
                                    endianess)

    if args.binary:
        create_binary_file(args.output_directory,
                           setting_offset,
                           content)

    if args.source:
        create_source_file(args.output_directory,
                           args.setting_memory,
                           setting_offset,
                           setting_size,
                           content)
