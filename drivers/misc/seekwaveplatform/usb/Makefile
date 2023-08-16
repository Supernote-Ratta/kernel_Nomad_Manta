# Seekwavetech: GPL-2.0
#
# Copyright (C) 2021-2021 Seekwavetech
#

#
# Makefile for Seekwavetech on-board usb driver
#
ccflags-y :=-Idrivers/misc/seekwaveplatform/skwutil
ifneq ($(skw_extra_flags),)
	ccflags-y := $(skw_extra_flags)	-DSKW_EXT_INC
endif
ifeq ($(CONFIG_SKW_BT),m)
	ccflags-y += -DCONFIG_BT_SEEKWAVE
endif
obj-$(CONFIG_SKW_USB) += skw_usb.o
skw_usb-y += skw_usb_io.o
skw_usb-y += skw_usb_debugfs.o
skw_usb-y += skw_usb_log.o
skw_usb-y += skw_usb_trace.o

CFLAGS_trace.o := -I$(src)
CFLAGS_skw_usb_trace.o := -I$(src)
