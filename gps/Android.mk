# Use hardware GPS implementation if available.
#
ifneq ($(BOARD_GPS_LIBRARIES),)
  LOCAL_CFLAGS           += -DHAVE_GPS_HARDWARE
  LOCAL_SHARED_LIBRARIES += $(BOARD_GPS_LIBRARIES)
endif

# Use emulator GPS implementation if QEMU_HARDWARE is set.
#
USE_QEMU_GPS_HARDWARE := $(QEMU_HARDWARE)

ifeq ($(USE_QEMU_GPS_HARDWARE),true)
    LOCAL_CFLAGS    += -DHAVE_QEMU_GPS_HARDWARE
    LOCAL_SRC_FILES += gps/gps_qemu.c
endif

# guanxiaowei 20100729 begin: add this function to find gps_hardware.c
ifeq ($(USE_FOXCONN_GPS_HARDWARE),true)
    LOCAL_CFLAGS    += -DHAVE_GPS_HARDWARE
    LOCAL_SRC_FILES += gps/gps_hardware.c
endif
#  guanxiaowei 20100729 end: add this function to find gps_hardware.c


LOCAL_SRC_FILES += gps/gps.cpp

