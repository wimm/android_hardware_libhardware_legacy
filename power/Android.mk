# Copyright 2006 The Android Open Source Project

LOCAL_SRC_FILES += power/power.c

ifeq ($(TARGET_BOARD_PLATFORM),s5p6442)
LOCAL_CFLAGS  += -DSLSI_S5P6442
endif

ifeq ($(QEMU_HARDWARE),true)
  LOCAL_SRC_FILES += power/power_qemu.c
  LOCAL_CFLAGS    += -DQEMU_POWER=1
endif
