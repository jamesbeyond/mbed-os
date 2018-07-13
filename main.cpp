/* mbed Microcontroller Library
 * Copyright (c) 2017 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#if !DEVICE_WATCHDOG
#error [NOT_SUPPORTED] Watchdog not supported for this target
#endif

#ifndef MBED_HAL_WATCHDOG_RESET_TESTS_H
#define MBED_HAL_WATCHDOG_RESET_TESTS_H
#endif

#include "greentea-client/test_env.h"
#include "utest/utest.h"
#include "unity/unity.h"
#include "hal/watchdog_api.h"
#include "mbed.h"

#define TIMEOUT_MS 500UL
#define TIMEOUT_DELTA_MS 1000UL


using namespace utest::v1;

void test_simple_reset()
{
    // Init the watchdog and wait for a device reset.
    watchdog_config_t config = { TIMEOUT_MS };
    utest_printf("Setting Watchdog to { %d } \n", TIMEOUT_MS);

    TEST_ASSERT_EQUAL(WATCHDOG_STATUS_OK, hal_watchdog_init(&config));
    wait_ms(TIMEOUT_MS + TIMEOUT_DELTA_MS); // Device reset expected.

    // Watchdog reset should have occurred during wait_ms() above;
    TEST_ASSERT_MESSAGE(0, "Watchdog did not reset the device as expected.");
}

utest::v1::status_t case_setup(const size_t number_of_cases)
{
    GREENTEA_SETUP(90, "watchdog_reset");
    utest_printf("Starting Testing %d \n", number_of_cases);
    return greentea_test_setup_handler(number_of_cases);
}

Case cases[] = {
    Case("Watchdog reset", test_simple_reset),
};

Specification specification(case_setup, cases);

int main()
{
    // Harness will start with a test case index provided by host script.
    return !Harness::run(specification);
}
