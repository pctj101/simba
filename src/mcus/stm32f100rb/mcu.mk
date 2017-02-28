#
# @section License
#
# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. We make this dedication for the benefit
# of the public at large and to the detriment of our heirs and
# successors. We intend this dedication to be an overt act of
# relinquishment in perpetuity of all present and future rights to this
# software under copyright law.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# For more information, please refer to <http://unlicense.org/>
#
# This file is part of the Simba project.
#

INC += $(SIMBA_ROOT)/src/mcus/stm32f100rb
SRC += $(SIMBA_ROOT)/src/mcus/stm32f100rb/mcu.c \
       $(SIMBA_ROOT)/src/mcus/stm32f100rb/stm32.c

MCPU = cortex-m3
F_CPU = 24000000

ARCH = arm
FAMILY = stm32f1

MCU_HOMEPAGE = ""
MCU_NAME = "ST STM32F100RB ARM Cortex-M3"
MCU_DESC = "ST STM32F100RB ARM Cortex-M3 @ 24MHz, 8k sram, 128k flash"

LIBPATH += "$(SIMBA_ROOT)/src/mcus/$(MCU)"
LINKER_SCRIPT ?= script.ld

include $(SIMBA_ROOT)/make/$(TOOLCHAIN)/arm.mk
