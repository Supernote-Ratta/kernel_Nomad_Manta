/*
 * cyttsp5_listener.h
 * Copyright (C) 2023 Htfyun.
 *
 */

#ifndef _LINUX_CYTTSP5_LISTENER_H
#define _LINUX_CYTTSP5_LISTENER_H

// 2030721,define the listener for SN-X2.dev: the device whitch gen the touch.
typedef void (*touch_listener)(struct device *dev,int t, int x, int y, int tip);

void cyttsp5_register_listener(touch_listener listener);

#endif /* _LINUX_CYTTSP5_LISTENER_H */
