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

int harness_init(struct harness_t *self_p)
{
    return (0);
}

int harness_run(struct harness_t *self_p,
                struct harness_testcase_t *testcases_p)
{
    int err;
    struct harness_testcase_t *testcase_p;
    int total, passed, failed, skipped;

    total = 0;
    passed = 0;
    failed = 0;
    skipped = 0;
    testcase_p = testcases_p;

#if !defined(ARCH_LINUX)
    thrd_sleep_us(200000);
#endif
    
    /* Print a header. */
    std_printf(OSTR("\r\n"));

    std_printf(OSTR("================================== TEST BEGIN ==================================\r\n\r\n"));
    std_printf(sys_get_info());
    std_printf(OSTR("\r\n"));

    while (testcase_p->callback != NULL) {
        if (testcase_p->name_p != NULL) {
            std_printf(OSTR("enter: %s\r\n"), testcase_p->name_p);
        }

        err = testcase_p->callback(self_p);

        if (err == 0) {
            passed++;
            std_printf(OSTR("exit: %s: PASSED\r\n\r\n"),
                       testcase_p->name_p);
        } else if (err < 0) {
            failed++;
            std_printf(OSTR("exit: %s: FAILED\r\n\r\n"),
                       testcase_p->name_p);
        } else {
            skipped++;
            std_printf(OSTR("exit: %s: SKIPPED\r\n\r\n"),
                       testcase_p->name_p);
        }

        total++;
        testcase_p++;
    }

#if CONFIG_FS_CMD_THRD_LIST == 1

    char buf[18];

    std_strcpy(buf, FSTR("/kernel/thrd/list"));
    fs_call(buf, NULL, sys_get_stdout(), NULL);

    std_printf(OSTR("\r\n"));

#endif

    std_printf(OSTR("harness report: total(%d), passed(%d), "
                    "failed(%d), skipped(%d)\r\n\r\n"),
               total, passed, failed, skipped);

    std_printf(OSTR("=============================== TEST END (%s) ==============================\r\n\r\n"),
               ((passed + skipped) == total ? "PASSED" : "FAILED"));

    sys_stop(failed);

    return (0);
}
