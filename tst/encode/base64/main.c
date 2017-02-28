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

static char encoded_text[] =
    "TWFuIGlzIGRpc3Rpbmd1aXNoZWQsIG5vdCBvbmx5IGJ5IGhpcyByZWFzb24sIGJ1dCBieSB0aGlz"
    "IHNpbmd1bGFyIHBhc3Npb24gZnJvbSBvdGhlciBhbmltYWxzLCB3aGljaCBpcyBhIGx1c3Qgb2Yg"
    "dGhlIG1pbmQsIHRoYXQgYnkgYSBwZXJzZXZlcmFuY2Ugb2YgZGVsaWdodCBpbiB0aGUgY29udGlu"
    "dWVkIGFuZCBpbmRlZmF0aWdhYmxlIGdlbmVyYXRpb24gb2Yga25vd2xlZGdlLCBleGNlZWRzIHRo"
    "ZSBzaG9ydCB2ZWhlbWVuY2Ugb2YgYW55IGNhcm5hbCBwbGVhc3VyZS4=";
static char decoded_text[] =
    "Man is distinguished, not only by his reason, but by this singular passion from "
    "other animals, which is a lust of the mind, that by a perseverance of delight "
    "in the continued and indefatigable generation of knowledge, exceeds the short "
    "vehemence of any carnal pleasure.";

static char encoded_m[] = "TQ==";
static char decoded_m[] = "M";

static char encoded_ma[] = "TWE=";
static char decoded_ma[] = "Ma";

static char encoded_man[] = "TWFu";
static char decoded_man[] = "Man";

static char *encoded[] = {
    "",
    "Zg==",
    "Zm8=",
    "Zm9v",
    "Zm9vYg==",
    "Zm9vYmE=",
    "Zm9vYmFy"
};

static char *decoded[] = {
    "",
    "f",
    "fo",
    "foo",
    "foob",
    "fooba",
    "foobar"
};

static int test_encode(struct harness_t *harness_p)
{
    int i;
    char buf[512];

    BTASSERT(base64_encode(buf, decoded_text, strlen(decoded_text)) == 0);
    buf[strlen(encoded_text)] = '\0';
    BTASSERT(strcmp(buf, encoded_text) == 0);

    BTASSERT(base64_encode(buf, decoded_m, strlen(decoded_m)) == 0);
    buf[strlen(encoded_m)] = '\0';
    BTASSERT(strcmp(buf, encoded_m) == 0);

    BTASSERT(base64_encode(buf, decoded_ma, strlen(decoded_ma)) == 0);
    buf[strlen(encoded_ma)] = '\0';
    BTASSERT(strcmp(buf, encoded_ma) == 0);

    BTASSERT(base64_encode(buf, decoded_man, strlen(decoded_man)) == 0);
    buf[strlen(encoded_man)] = '\0';
    BTASSERT(strcmp(buf, encoded_man) == 0);

    BTASSERT(base64_encode(buf, "\xfb\xff\xbf", 3) == 0);
    BTASSERT(buf[0] == '+');
    BTASSERT(buf[1] == '/');
    BTASSERT(buf[2] == '+');
    BTASSERT(buf[3] == '/');

    for (i = 0; i < membersof(decoded); i++) {
        BTASSERT(base64_encode(buf, decoded[i], strlen(decoded[i])) == 0);
        buf[strlen(encoded[i])] = '\0';
        BTASSERT(strcmp(buf, encoded[i]) == 0);
    }

    return (0);
}

static int test_decode(struct harness_t *harness_p)
{
    int i;
    char buf[512];

    BTASSERT(base64_decode(buf, encoded_text, strlen(encoded_text)) == 0);
    buf[strlen(decoded_text)] = '\0';
    BTASSERT(strcmp(buf, decoded_text) == 0);

    BTASSERT(base64_decode(buf, encoded_m, strlen(encoded_m)) == 0);
    buf[strlen(decoded_m)] = '\0';
    BTASSERT(strcmp(buf, decoded_m) == 0);

    BTASSERT(base64_decode(buf, encoded_ma, strlen(encoded_ma)) == 0);
    buf[strlen(decoded_ma)] = '\0';
    BTASSERT(strcmp(buf, decoded_ma) == 0);

    BTASSERT(base64_decode(buf, encoded_man, strlen(encoded_man)) == 0);
    buf[strlen(decoded_man)] = '\0';
    BTASSERT(strcmp(buf, decoded_man) == 0);

    BTASSERT(base64_decode(buf, encoded_man, 3) == -EINVAL);
    BTASSERT(base64_decode(buf, "\x01\x02\x03\x04", 4) == -1);

    BTASSERT(base64_decode(buf, "+/+/", 4) == 0);
    BTASSERT(buf[0] == '\xfb');
    BTASSERT(buf[1] == '\xff');
    BTASSERT(buf[2] == '\xbf');

    for (i = 0; i < membersof(encoded); i++) {
        BTASSERT(base64_decode(buf, encoded[i], strlen(encoded[i])) == 0);
        buf[strlen(decoded[i])] = '\0';
        BTASSERT(strcmp(buf, decoded[i]) == 0);
    }

    return (0);
}

int main()
{
    struct harness_t harness;
    struct harness_testcase_t harness_testcases[] = {
        { test_encode, "test_encode" },
        { test_decode, "test_decode" },
        { NULL, NULL }
    };

    sys_start();

    harness_init(&harness);
    harness_run(&harness, harness_testcases);

    return (0);
}
