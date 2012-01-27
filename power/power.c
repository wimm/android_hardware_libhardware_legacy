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
#include <hardware_legacy/power.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h>

#define LOG_TAG "power"
#include <utils/Log.h>

#include "qemu.h"
#ifdef QEMU_POWER
#include "power_qemu.h"
#endif

#ifdef SLSI_S5P6442
#define SAMSUNG_BACKLIGHT_HACK
#endif

enum {
    ACQUIRE_PARTIAL_WAKE_LOCK = 0,
    RELEASE_WAKE_LOCK,
    REQUEST_STATE,
    OUR_FD_COUNT
};

const char * const OLD_PATHS[] = {
    "/sys/android_power/acquire_partial_wake_lock",
    "/sys/android_power/release_wake_lock",
    "/sys/android_power/request_state"
};

const char * const NEW_PATHS[] = {
    "/sys/power/wake_lock",
    "/sys/power/wake_unlock",
    "/sys/power/state"
};

const char * const AUTO_OFF_TIMEOUT_DEV = "/sys/android_power/auto_off_timeout";
#ifdef SLSI_S5P6442
#ifdef SAMSUNG_BACKLIGHT_HACK
const char * const LCD_BACKLIGHT_BRIGHTNESS_DEV = "/sys/class/leds/backlight/brightness";
#endif

// for ctrl cpu freq by hskang
const char * const CPUFREQ_GOVERNOR_PATHS = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor";
static const char *cpufreq_on = "conservative";
static const char *cpufreq_off = "performance";

#endif /*#ifdef SLSI_S5P6442*/
//XXX static pthread_once_t g_initialized = THREAD_ONCE_INIT;
static int g_initialized = 0;
static int g_fds[OUR_FD_COUNT];
static int g_error = 1;

static const char *off_state = "mem";
static const char *on_state = "on";
#ifdef SLSI_S5P6442
#ifdef SAMSUNG_BACKLIGHT_HACK
static int
write_int(char const* path, int value)
{
    int fd;
    static int already_warned = 0;

    fd = open(path, O_RDWR);
    if (fd >= 0) {
        char buffer[20];
        int bytes = sprintf(buffer, "%d\n", value);
        int amt = write(fd, buffer, bytes);
        close(fd);
        return amt == -1 ? -errno : 0;
    } else {
        if (already_warned == 0) {
            LOGE("write_int failed to open %s\n", path);
//            already_warned = 1;
        }
        return -errno;
    }
}

static int
read_int(const char* path, int* value)
{
    int fd;
    static int already_warned = 0;

    fd = open(path, O_RDONLY);
    if (fd >= 0) {
        char buffer[20];
        int amt = read(fd, buffer, sizeof(buffer)-1);
        if (amt < 0)
        {
           close(fd);
           return -errno;
        }
        else
        {
           buffer[amt] = 0;
           if (sscanf(buffer,"%d",value) != 1)
           {
              close(fd);
              return -1;
           }
        }
        close(fd);
        return 0;
    } else {
        if (already_warned == 0) {
            LOGE("read_int failed to open %s\n", path);
//            already_warned = 1;
        }
        return -errno;
    }
}
#endif
#endif/*#ifdef SLSI_S5P6442*/

static int64_t systemTime()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec*1000000000LL + t.tv_nsec;
}

static int
open_file_descriptors(const char * const paths[])
{
    int i;
    for (i=0; i<OUR_FD_COUNT; i++) {
        int fd = open(paths[i], O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "fatal error opening \"%s\"\n", paths[i]);
            g_error = errno;
            return -1;
        }
        g_fds[i] = fd;
    }

    g_error = 0;
    return 0;
}

static inline void
initialize_fds(void)
{
    // XXX: should be this:
    //pthread_once(&g_initialized, open_file_descriptors);
    // XXX: not this:
    if (g_initialized == 0) {
        if(open_file_descriptors(NEW_PATHS) < 0) {
            open_file_descriptors(OLD_PATHS);
            on_state = "wake";
            off_state = "standby";
        }
        g_initialized = 1;
    }
}

int
acquire_wake_lock(int lock, const char* id)
{
    initialize_fds();

//    LOGI("acquire_wake_lock lock=%d id='%s'\n", lock, id);

    if (g_error) return g_error;

    int fd;

    if (lock == PARTIAL_WAKE_LOCK) {
        fd = g_fds[ACQUIRE_PARTIAL_WAKE_LOCK];
    }
    else {
        return EINVAL;
    }

    return write(fd, id, strlen(id));
}

int
release_wake_lock(const char* id)
{
    initialize_fds();

//    LOGI("release_wake_lock id='%s'\n", id);

    if (g_error) return g_error;

    ssize_t len = write(g_fds[RELEASE_WAKE_LOCK], id, strlen(id));
    return len >= 0;
}

int
set_last_user_activity_timeout(int64_t delay)
{
//    LOGI("set_last_user_activity_timeout delay=%d\n", ((int)(delay)));

    int fd = open(AUTO_OFF_TIMEOUT_DEV, O_RDWR);
    if (fd >= 0) {
        char buf[32];
        ssize_t len;
        len = sprintf(buf, "%d", ((int)(delay)));
        len = write(fd, buf, len);
        close(fd);
        return 0;
    } else {
        return errno;
    }
}
#ifdef SLSI_S5P6442
int
set_screen_state(int on)
{
#ifdef SAMSUNG_BACKLIGHT_HACK
int brightness;
int restore_brightness = 0;
 static pthread_mutex_t pwrmutex = PTHREAD_MUTEX_INITIALIZER;
#endif
    QEMU_FALLBACK(set_screen_state(on));

    //LOGI("*** set_screen_state %d", on);

    initialize_fds();

    //LOGI("go_to_sleep eventTime=%lld now=%lld g_error=%s\n", eventTime,
      //      systemTime(), strerror(g_error));

    if (g_error) return g_error;

    char buf[32];
    int len;
#ifdef SAMSUNG_BACKLIGHT_HACK
    pthread_mutex_lock(&pwrmutex);
    lseek(g_fds[REQUEST_STATE],0,SEEK_SET);
    len = read(g_fds[REQUEST_STATE], buf, sizeof(buf)-1);
    if (len <= 0)
    {
        LOGE("Failed to read current state: %d %d\n",len,errno);
    }
    else
    {
       buf[len]=0;
       if (strncmp(buf,on_state,strlen(on_state)) == 0 && !on)
       {
          /* about to turn LCD off */
          if (read_int(LCD_BACKLIGHT_BRIGHTNESS_DEV,&brightness) == 0)
          {
             if (brightness > 0)
             {
                LOGD("Turning off LCD, but brightness is still %d, setting it to 0",brightness);
                write_int(LCD_BACKLIGHT_BRIGHTNESS_DEV,0);
                restore_brightness = 1;
             }
          }
       }
       else if (strncmp(buf,off_state,strlen(off_state)) == 0 && on)
       {
          /* about to turn LCD on */
          /* if we have backlight on, first turn it off */
          if (read_int(LCD_BACKLIGHT_BRIGHTNESS_DEV,&brightness) == 0)
          {
             if (brightness > 0)
             {
                LOGD("Turning off LCD, but brightness is already %d, setting it to 0",brightness);
                write_int(LCD_BACKLIGHT_BRIGHTNESS_DEV,0);
                restore_brightness = 1;
             }
          }
       }
    }
#endif
    if(on)
    {
        len = sprintf(buf, on_state);
    }
    else
    {
        len = sprintf(buf, off_state);
    }
    LOGD("Setting LCD to %s",buf);
    len = write(g_fds[REQUEST_STATE], buf, len);
    if(len < 0) {
        LOGE("Failed setting last user activity: g_error=%d\n", g_error);
    }
#ifdef SAMSUNG_BACKLIGHT_HACK
    if (restore_brightness)
    {
       LOGD("Restoring brightness to %d",brightness);
       write_int(LCD_BACKLIGHT_BRIGHTNESS_DEV,brightness);
    }
    pthread_mutex_unlock(&pwrmutex);
#endif
    return 0;
}

// for ctrl cpu freq by hskang
int
set_cpufreq_state(int on)
{
    int fd = open(CPUFREQ_GOVERNOR_PATHS, O_RDWR);
    
    LOGI("*** set_cpufreq_state %d", on);
    
    if (fd >= 0) {
        char buf[32];
        ssize_t len;
	    if (on)
	        len = sprintf(buf, cpufreq_on);
	    else
	        len = sprintf(buf, cpufreq_off);
        len = write(fd, buf, len);
        close(fd);
        return 0;
    } else {
        return errno;
    }
}
#else /*#ifdef SLSI_S5P6442*/
int
set_screen_state(int on)
{
    QEMU_FALLBACK(set_screen_state(on));

    LOGI("*** set_screen_state %d", on);

    initialize_fds();

    //LOGI("go_to_sleep eventTime=%lld now=%lld g_error=%s\n", eventTime,
      //      systemTime(), strerror(g_error));

    if (g_error) return g_error;

    char buf[32];
    int len;
    if(on)
        len = sprintf(buf, on_state);
    else
        len = sprintf(buf, off_state);
    len = write(g_fds[REQUEST_STATE], buf, len);
    if(len < 0) {
        LOGE("Failed setting last user activity: g_error=%d\n", g_error);
    }
    return 0;
}

#endif SLSI_S5P6442
