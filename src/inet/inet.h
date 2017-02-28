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

#ifndef __INET_INET_H__
#define __INET_INET_H__

#include "simba.h"

struct inet_ip_addr_t {
    /** IPv4 address. */
    uint32_t number;
};

struct inet_addr_t {
    /** IPv4 address. */
    struct inet_ip_addr_t ip;
    /** Port. */
    uint16_t port;
};

/**
 * Interface IP information.
 */
struct inet_if_ip_info_t {
    struct inet_ip_addr_t address;
    struct inet_ip_addr_t netmask;
    struct inet_ip_addr_t gateway;
};

/**
 * Initialize the inet module. This function must be called before
 * calling any other function in this module.
 *
 * The module will only be initialized once even if this function is
 * called multiple times.
 *
 * @return zero(0) or negative error code.
 */
int inet_module_init(void);

/**
 * Convert the Internet host address src_p from the IPv4
 * numbers-and-dots notation into binary form (in network byte order)
 * and stores it in the structure that dst_p points to.
 *
 * The address supplied in src_p can have one of the following forms:
 *
 * - a.b.c.d Each of the four numeric parts specifies a byte of the
 *           address; the bytes are assigned in left-to-right order to
 *           produce the binary address.
 *
 * @param[in] src_p Address a.b.c.d to convert into a number.
 * @param[out] dst_p Converted address.
 *
 * @return zero(0) or negative error code.
 */
int inet_aton(const char *src_p,
              struct inet_ip_addr_t *dst_p);

/**
 * Convert the Internet host src_p from the IPv4 binary form (in
 * network byte order) to numbers-and-dots notation and stores it in
 * the structure that dst_p points to.
 *
 * @param[in] src_p Address to convert into a string.
 * @param[out] dst_p Converted address as a string.
 *
 * @return Converted address pointer or NULL on failure.
 */
char *inet_ntoa(const struct inet_ip_addr_t *src_p,
                char *dst_p);

/**
 * Calculate the internet checksum of given buffer.
 *
 * @param[in] buf_p Buffer to calculate the chechsum of.
 * @param[in] size Size of the buffer.
 *
 * @return Calculated checksum.
 */
uint16_t inet_checksum(void *buf_p, size_t size);

#endif
