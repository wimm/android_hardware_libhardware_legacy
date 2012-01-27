#include <hardware_legacy/gps.h>
#include <cutils/properties.h>

#define LOG_TAG "libhardware_legacy"
#include <utils/Log.h>
#include "qemu.h"

#define  GPS_DEBUG  0

#if GPS_DEBUG
#  define  D(...)   LOGE(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif
static const GpsInterface*  sGpsInterface = NULL;

static void
gps_find_hardware( void )
{
    D("gps_find_hardware IN");
#ifdef HAVE_QEMU_GPS_HARDWARE
    D("using QEMU GPS Hardware");
    if (qemu_check()) {
        sGpsInterface = gps_get_qemu_interface();
        if (sGpsInterface) {
            LOGD("using QEMU GPS Hardware emulation\n");
            return;
        }
    }
#endif

/* guanxiaowei 20100729 begin: add this function to get interface */
#ifdef HAVE_GPS_HARDWARE
    D("=============using gps_hardware.c");
    sGpsInterface = gps_get_hardware_interface();
#endif
    if (!sGpsInterface)
        LOGD("no GPS hardware on this device\n");
    D("gps_find_hardware out");
}
/* guanxiaowei 20100729 end: add this function to get interface */

const GpsInterface*
gps_get_interface()
{
    D("gps_get_interface IN");
    if (sGpsInterface == NULL)
         gps_find_hardware();
    D("gps_get_interface out");
    return sGpsInterface;
}
