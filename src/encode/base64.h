/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2016, Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * This file is part of the Simba project.
 */

#ifndef __ENCODE_BASE64_H__
#define __ENCODE_BASE64_H__

#include "simba.h"

/**
 * Encode given buffer. The encoded data will be ~33.3% larger than
 * the source data. Choose the destination buffer size accordingly.
 *
 * @param[out] dst_p Encoded output data.
 * @param[in] src_p Input data.
 * @param[in] size Number of bytes in the input data.
 *
 * @return zero(0) or negative error code.
 */
int base64_encode(char *dst_p, const void *src_p, size_t size);

/**
 * Decode given base64 encoded buffer. The decoded data will be ~25%
 * smaller than the destination data. Choose the destination buffer
 * size accordingly.
 *
 * @param[out] dst_p Output data.
 * @param[in] src_p Encoded input data.
 * @param[in] size Number of bytes in the encoded input data.
 *
 * @return zero(0) or negative error code.
 */
int base64_decode(void *dst_p, const char *src_p, size_t size);

#endif
