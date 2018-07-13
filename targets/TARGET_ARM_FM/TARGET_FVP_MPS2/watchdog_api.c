/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
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

#ifdef DEVICE_WATCHDOG
 
#include "watchdog_api.h"
#include "PeripheralNames.h"
#define WATCHDOG              CMSDK_WATCHDOG
#define WATCHDOG_NonMaskableInt_IRQn  NonMaskableInt_IRQn

// Platform specific watchdog definitions
#define LPO_CLOCK_FREQUENCY 1000
#define MAX_PRESCALER       16
#define MAX_TIMEOUT         0xFFFFFFFFUL
#define WCT_IN_BUS_CYCLES   256U // Watchdog configuration time (WCT) in bus clock cycles.

// Number of decrements in the timeout register per millisecond
#define TICKS_PER_MS ((LPO_CLOCK_FREQUENCY) / 1000)

// Maximum timeout that can be specified in milliseconds
#define MAX_TIMEOUT_MS_UINT64 (1ULL * ((MAX_TIMEOUT) / (TICKS_PER_MS)) * (MAX_PRESCALER))
#if (MAX_TIMEOUT_MS_UINT64 > UINT32_MAX)
#define MAX_TIMEOUT_MS UINT32_MAX
#else
#define MAX_TIMEOUT_MS (MAX_TIMEOUT_MS_UINT64 & 0xFFFFFFFFUL)
#endif
 

static void unlock_watchdog(void){
    WATCHDOG->LOCK = 0x1ACCE551;
}

static void lock_watchdog(void){
    WATCHDOG->LOCK = 0x0;
}

watchdog_status_t hal_watchdog_init(const watchdog_config_t *config)
{
    if (WATCHDOG->LOCK)
    {
        unlock_watchdog();
    }

    WATCHDOG->LOAD = (config->timeout_ms*25);
    WATCHDOG->CTRL |= CMSDK_Watchdog_CTRL_INTEN_Msk;
    WATCHDOG->CTRL |= CMSDK_Watchdog_CTRL_RESEN_Msk;

    lock_watchdog();
    return WATCHDOG_STATUS_OK;
}

void hal_watchdog_kick(void)
{
    if (WATCHDOG->LOCK)
    {
        unlock_watchdog();
    }
    WATCHDOG->INTCLR = 0x1u;
    lock_watchdog();

}
watchdog_status_t hal_watchdog_stop(void)
{
    if (WATCHDOG->LOCK)
    {
        unlock_watchdog();
    }
    WATCHDOG->CTRL &= ~CMSDK_Watchdog_CTRL_RESEN_Msk;
    WATCHDOG->CTRL &= ~CMSDK_Watchdog_CTRL_INTEN_Msk;
    lock_watchdog();
    return WATCHDOG_STATUS_OK;
    
}


uint32_t hal_watchdog_get_reload_value(void)
{
    return WATCHDOG->LOAD;
}


watchdog_features_t hal_watchdog_get_platform_features(void)
{
  watchdog_features_t features;
  features.max_timeout = MAX_TIMEOUT_MS;
  features.update_config = true;
  features.disable_watchdog = true;
  return features;
}

void NMI_Handler(void)
{
    //do nothing
}

#endif // DEVICE_WATCHDOG
