/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _HARDWARE_POWER_H
#define _HARDWARE_POWER_H

#include <stdint.h>

#if __cplusplus
extern "C" {
#endif

enum {
    PARTIAL_WAKE_LOCK = 1,  // the cpu stays on, but the screen is off
    FULL_WAKE_LOCK = 2      // the screen is also on
};

// while you have a lock held, the device will stay on at least at the
// level you request.
int acquire_wake_lock(int lock, const char* id);
int release_wake_lock(const char* id);

// true if you want the screen on, false if you want it off
int set_screen_state(int on);

// set how long to stay awake after the last user activity in seconds
int set_last_user_activity_timeout(int64_t delay);
//#ifdef SLSI_S5P6442
// turn if you want to use DVFS(Dynamic Voltage Frequency Scaling) on, false if you want it off
int set_cpufreq_state(int on);
//#endif


#if __cplusplus
} // extern "C"
#endif

#endif // _HARDWARE_POWER_H
