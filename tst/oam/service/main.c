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

static struct service_t foo_service;
static struct service_t bar_service;
static struct event_t foo_event;
static struct event_t bar_event;
static enum service_status_t foo_status = service_status_stopped_t;
static enum service_status_t bar_status = service_status_running_t;

enum service_status_t foo_status_cb(struct service_t *self_p)
{
    return (foo_status);
}

enum service_status_t bar_status_cb(struct service_t *self_p)
{
    return (bar_status);
}

static int test_init(struct harness_t *harness_p)
{
    BTASSERT(event_init(&foo_event) == 0);
    BTASSERT(event_init(&bar_event) == 0);

    BTASSERT(service_module_init() == 0);

    BTASSERT(service_init(&foo_service,
                          "foo",
                          foo_status_cb) == 0);

    BTASSERT(service_init(&bar_service,
                          "bar",
                          bar_status_cb) == 0);

    return (0);
}

static int test_register(struct harness_t *harness_p)
{
    BTASSERT(service_register(&foo_service) == 0);
    BTASSERT(service_register(&bar_service) == 0);

    return (0);
}

static int test_control(struct harness_t *harness_p)
{
    uint32_t mask;

    /* Start the foo service. */
    BTASSERT(service_start(&foo_service) == 0);

    mask = 0xffffffff;
    event_read(&foo_service.control, &mask, sizeof(mask));
    BTASSERT(mask == SERVICE_CONTROL_EVENT_START);
    foo_status = service_status_running_t;

    /* Stop the bar service. */
    BTASSERT(service_stop(&bar_service) == 0);

    mask = 0xffffffff;
    event_read(&bar_service.control, &mask, sizeof(mask));
    BTASSERT(mask == SERVICE_CONTROL_EVENT_STOP);
    bar_status = service_status_stopped_t;

    return (0);
}

static int test_fs_commands(struct harness_t *harness_p)
{
    uint32_t mask;
    char buf[64];

    strcpy(buf, "/oam/service/list");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == 0);

    /* Stop the foo service. */
    strcpy(buf, "/oam/service/stop foo");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == 0);

    mask = 0xffffffff;
    event_read(&foo_service.control, &mask, sizeof(mask));
    BTASSERT(mask == SERVICE_CONTROL_EVENT_STOP);
    foo_status = service_status_stopped_t;

    /* Start the bar service. */
    strcpy(buf, "/oam/service/start bar");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == 0);

    mask = 0xffffffff;
    event_read(&bar_service.control, &mask, sizeof(mask));
    BTASSERT(mask == SERVICE_CONTROL_EVENT_START);
    bar_status = service_status_running_t;

    strcpy(buf, "/oam/service/list");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == 0);

    /* Bad service name. */
    strcpy(buf, "/oam/service/start fum");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == -1);
    strcpy(buf, "/oam/service/stop fum");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == -1);

    /* Bad number of arguments. */
    strcpy(buf, "/oam/service/start");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == -EINVAL);
    strcpy(buf, "/oam/service/stop");
    BTASSERT(fs_call(buf, NULL, sys_get_stdout(), NULL) == -EINVAL);

    return (0);
}

static int test_unregister(struct harness_t *harness_p)
{
    BTASSERT(service_deregister(&foo_service) == -ENOSYS);
    BTASSERT(service_deregister(&bar_service) == -ENOSYS);

    return (0);
}

int main()
{
    struct harness_t harness;
    struct harness_testcase_t harness_testcases[] = {
        { test_init, "test_init" },
        { test_register, "test_register" },
        { test_control, "test_control" },
        { test_fs_commands, "test_fs_commands" },
        { test_unregister, "test_unregister" },
        { NULL, NULL }
    };

    sys_start();

    harness_init(&harness);
    harness_run(&harness, harness_testcases);

    return (0);
}
