# Makefile for the focaltech touchscreen drivers.



obj-$(CONFIG_SLIDER_FTS) += focaltech_slider.o
focaltech_slider-y := focaltech_core.o \
		focaltech_ex_fun.o \
		focaltech_ex_mode.o \
		focaltech_gesture.o \
		focaltech_esdcheck.o \
		focaltech_point_report_check.o


focaltech_slider-y += focaltech_i2c.o





focaltech_slider-y += focaltech_flash.o
focaltech_slider-y += focaltech_flash/focaltech_upgrade_ft3268.o