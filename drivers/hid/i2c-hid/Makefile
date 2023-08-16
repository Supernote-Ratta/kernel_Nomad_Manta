#
# Makefile for the I2C input drivers
#

obj-$(CONFIG_I2C_HID) += i2c-hid.o
obj-$(CONFIG_I2C_HID_WACOM_9013) += wacom_w9013.o
obj-$(CONFIG_I2C_HID_WACOM_10S12MI) += wacom_10s12mi.o
obj-$(CONFIG_I2C_HID_WACOM_HT) += wacom_hid.o
i2c-hid-objs =  i2c-hid-core.o
i2c-hid-$(CONFIG_DMI) += i2c-hid-dmi-quirks.o
