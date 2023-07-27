/*
 * pt_slider_listener.c
 * Copyright (C) 2023-2025 Htfyun Technologies
 *
 */

#include <linux/ktime.h>
#include <linux/mutex.h>

#include "pt_regs.h"

#include "../cyttsp5_slider/cyttsp5_listener.h"
#define SLIDER_DRV_VER "230727"
// max time from slider up to tp down for swap.
#define MAX_TP_SLIDER_DELAYMS_FOR_SWAP      80 // ms

// max tp move x distance(from downx to upx) for swap.
#define MAX_TP_XDIS_FOR_SWAP                350 // pixel

// min tp move x distance for swap.
#define MIN_TP_XDIS_FOR_SWAP                84 // pixel

// max tp move y distance for swap: OY < 1/3 OX
//#define MAX_TP_YDIS_FOR_SWAP                80 // pixel.

// max time from slider up to tp up for swap.
#define MAX_TP_DELAY_FOR_SWAP               550  // ms

// 2030724: define the tp-swap invalid area.
#define MAX_TPY_AB                  120
#define MIN_TPY_CD                  1752

// first pt_y valid <= 30 || >= 1374.
#define MAX_FIRST_TPX_AD            1374
#define MIN_FIRST_TPX_BC            30

#define MIN_TPX_AD                  (MAX_FIRST_TPX_AD-MAX_TP_XDIS_FOR_SWAP)
#define MAX_TPX_BC                  (MIN_FIRST_TPX_BC+MAX_TP_XDIS_FOR_SWAP)

struct pt_slider_area {
    char    id;
    int     slider_minx;
    int     slider_maxx;
    int     slider_y;
    int     slider_keycode;

    int     tp_min_y;
    int     tp_max_y;
    int     tp_first_x;
};

// 20230724: define the special up_time.
#define UPTIME_IDLE             0
#define UPTIME_TOUCHING         1
#define UPTIME_SPECIAL_MAX      5

struct pt_slider_touch {
    int         t;
    int         x;
    int         y;
    ktime_t     up_time;

    // the current swap area.
    const struct pt_slider_area *area;
    int         tp_x; // tp first x/y.
    int         tp_y;
    int         tp_t; // tp tracking.

    int         tp_lx; // tp last x/y.
    int         tp_ly;
};

#define MAX_SLIDER_TRACK        4
static struct pt_slider_touch  slider_touchs[MAX_SLIDER_TRACK];
static int                     slider_track_mask = 0;
//static int                     tp_track_mask = 0;
static struct pt_slider_touch   *tp_track_slider = NULL;
static struct mutex            slider_mutex;

// define the TP touch area for slider-swap.
const struct pt_slider_area   slider_areas[MAX_SLIDER_TRACK] = {
    // AREA-A: RIGHT-TOP
	{'A', 1, 110, 30/*slider y*/, 292, 1, MAX_TPY_AB, MAX_FIRST_TPX_AD/*tp_first_y*/},
	
	// AREA-B: LEFT-TOP
	{'B', 1, 110, 70/*slider y*/, 293, 1, MAX_TPY_AB, MIN_FIRST_TPX_BC/*tp_first_y*/},
	
	// AREA-C: LEFT-BOTTOM
	{'C', 140, 230, 50/*slider y*/, 294, MIN_TPY_CD, 1872, MIN_FIRST_TPX_BC/*tp_first_y*/},
	
	// AREA-D: RIGHT-BOTTOM
	{'D', 140, 230, 10/*slider y*/, KEY_BACK, MIN_TPY_CD, 1872, MAX_FIRST_TPX_AD/*tp_first_y*/},

};

// 20230724: dump for debug.
static void pt_dump_slider_touch(struct device *dev)
{
    int         i;
    s64 	    total_us;
    struct pt_slider_touch* p;
    for(i = 0; i < MAX_SLIDER_TRACK; i++) {
        p = &slider_touchs[i];
        if(p->up_time < UPTIME_SPECIAL_MAX) {
            total_us = p->up_time;
        } else {
            total_us = ktime_to_us(ktime_sub(ktime_get(), p->up_time));
        }
        pt_debug(dev, DL_INFO,"slider_track_mask=0x%02x [%d/%d]:t=%d,x=%d,y=%d,up_time=%lld\n", 
         	slider_track_mask, i, MAX_SLIDER_TRACK, p->t, p->x, p->y, total_us);
    }
}


static void pt_dump_slider_area(struct device *dev)
{
    int         i;
    const struct pt_slider_area *p;
    for(i = 0; i < MAX_SLIDER_TRACK; i++) {
        p = &slider_areas[i];
        pt_debug(dev, DL_LISTEN,"[%d/%d]%c:sx=[%d-%d],sy=%d,tpy=[%d,%d],tfx=%d,kc=%d\n",
            i, MAX_SLIDER_TRACK, p->id,
            p->slider_minx, p->slider_maxx,
            p->slider_y, p->tp_min_y, p->tp_max_y,
            p->tp_first_x, p->slider_keycode);
    }
}

static void pt_reset_slide_touch(struct pt_slider_touch* p)
{
    mutex_lock(&slider_mutex);
    slider_track_mask &= ~(1<<p->t);
    p->up_time = UPTIME_IDLE;
    p->t = -1;
    p->area = NULL;
    mutex_unlock(&slider_mutex);
}

// 20230724: update the slider state by slider-touch event.
static void pt_update_slider_touch(struct device *dev,int t, int x, int y, int tip)
{
    struct pt_slider_touch *p;
    //int i;
    if(t >= MAX_SLIDER_TRACK) {
        pt_debug(dev, DL_LISTEN,"%s: Invalid t=%d,max=%d!\n", __func__, t, MAX_SLIDER_TRACK);
        return;
    }

    p = &slider_touchs[t];
    mutex_lock(&slider_mutex);

    // already got touch.or maybe slide touch-up and then down.
    if(slider_track_mask & (1<<t)) {
        if(!tip) {
            p->up_time = ktime_get();

            // 20230724: clear delay some time.but we will clear at touch.
            //slider_track_mask &= ~(1<<t);
            //p->t = -1;
            pt_debug(dev, DL_LISTEN,"%s:off slider: t=%d,x=%d,y=%d,up_time=%lld!\n", __func__,
                t, x, y, p->up_time);
        } else {
            p->x = x;
            p->y = y;
            p->up_time = UPTIME_TOUCHING;
        }
        pt_debug(dev, DL_LISTEN,"%s:mask=0x%x,t=%d,but No touch found\n", __func__, slider_track_mask, t);
    } else {
        if(tip) {
            slider_track_mask |= (1<<t);
            pt_debug(dev, DL_LISTEN,"%s:set slider: t=%d,x=%d,y=%d!\n", __func__, t, x, y);
            p->t = t;
            p->x = x;
            p->y = y;
            p->up_time = UPTIME_TOUCHING;
        } else {
            pt_debug(dev, DL_LISTEN,"%s:mask=0x%x,t=%d,but No Mask for track up!\n", __func__, slider_track_mask, t);
            //p->up_time = ktime_get();
        }
    }

    mutex_unlock(&slider_mutex);
    //return &slider_touchs[t];
}


// return 0: not valid tp-swap touching. 1: maybe valid-tp-swap touching.
static bool pt_filter_by_fix_slider_area(int pt_t, int pt_x, int pt_y)
{
    if(pt_y > MAX_TPY_AB && pt_y < MIN_TPY_CD) {
        /*if(tp_track_slider != NULL && tp_track_slider->area != NULL
            && tp_track_slider->area->tp_mask_bit == (1<<pt_t)) {
            tp_track_slider = NULL;
        }*/
        return 0;
    }

    if(pt_x > MAX_TPX_BC && pt_x < MIN_TPX_AD) {
        return 0;
    }

    return 1;
}

// return true: if multi slider is touching. false: only single slide touch.
static int pt_filter_check_multi_slider()
{
    int         i;
    int         c = 0;
    for(i = 0; i < MAX_SLIDER_TRACK; i++) {
        if(slider_track_mask & (1<<i)){
            c++;
        }
    }
    return c;
}

// 20230724: found the slider area by tp x/y.
static const struct pt_slider_area* pt_get_slider_area(struct device *dev,int x, int y)
{
    int i;
    const struct pt_slider_area *p;
    bool    found = false;
    for(i = 0; i < MAX_SLIDER_TRACK; i++){
        p = &slider_areas[i];

        // first check x.
        if(y > p->tp_max_y || y < p->tp_min_y) {
            continue;
        }

        // then check y.
        if(p->tp_first_x <= MAX_TPX_BC) {
            // AREA B/C
            if(x < p->tp_first_x) {
                found = true;
                break;
            }
        } else {
            // AREA A/D
            if(x > p->tp_first_x) {
                found = true;
                break;
            }
        }
    }

    pt_debug(dev, DL_LISTEN,"%s: found=%d,area[%c]:miny=%d,maxy=%d,fx=%d\n",
		    __func__, found, p->id,
		    p->tp_min_y, p->tp_max_y,
		    p->tp_first_x);

    if(found) return p;
    return NULL;
}

static struct pt_slider_touch* pt_get_slider_touch(struct device *dev,int x, int y, int t,
    const struct pt_slider_area* pa)
{
    int         i;
    struct pt_slider_touch *p;
    for(i = 0; i < MAX_SLIDER_TRACK; i++) {
        if(slider_track_mask & (1<<i)){
            p = &slider_touchs[i];
            pt_debug(dev, DL_LISTEN,"%s: slider y=%d,up_time=%lld, tx=%d, ty=%d,pa=%c\n",
    		    __func__, p->y, p->up_time, x, y, pa->id);
            if(p->y == pa->slider_y) {
                ktime_t nowx = ktime_get();
                if(p->up_time == UPTIME_TOUCHING
                    || ktime_to_ms(ktime_sub(nowx, p->up_time)) < MAX_TP_SLIDER_DELAYMS_FOR_SWAP){
                    p->area = pa;
                    p->tp_x = p->tp_lx = x;
                    p->tp_y = p->tp_ly = y;
                    p->tp_t = t;
                    if(p->up_time == UPTIME_TOUCHING) {
                        p->up_time = nowx;
                    }
                    return p;
                }
            }
        }
    }
    return NULL;
}

static void pt_filter_clear_slider_by_time(struct device *dev)
{
    int         i;
    struct pt_slider_touch *p;
    ktime_t nowx = ktime_get();

    for(i = 0; i < MAX_SLIDER_TRACK; i++) {
        if(slider_track_mask & (1<<i)){
            p = &slider_touchs[i];
            if(p == tp_track_slider) {
                continue;
            }
            if(p->up_time > UPTIME_SPECIAL_MAX &&
                ktime_to_ms(ktime_sub(nowx, p->up_time)) > MAX_TP_SLIDER_DELAYMS_FOR_SWAP){
                pt_reset_slide_touch(p);
				 pt_debug(dev, DL_LISTEN,"%s: t=%d, i=%d pt_reset_slide_touch\n", __func__, p->t, i);
            }
        }
    }

}


// 20230724: tp touch up with slide tracking. return 0: don't need report
// key, other: the keycode when needs report.
static int pt_slider_touch_up(struct device *dev)
{
    //int         i;
    struct pt_slider_touch *p = tp_track_slider;
    int         kc = 0;
    int         ox = p->tp_lx - p->tp_x;
    int         oy = p->tp_ly - p->tp_y;
    s64         os = ktime_to_ms(ktime_sub(ktime_get(), p->up_time));

    if(ox < 0) ox = -ox;
    if(oy < 0) oy = -oy;

    // 20230724: pt_slider_touch_up: x=254,y=53, dx=5,dy=46,ox=249,oy=7,os=119ms
    pt_debug(dev, DL_LISTEN,"%s: x=%d,y=%d, dx=%d,dy=%d,ox=%d,oy=%d,os=%lldms,pa=%p\n",
        __func__, p->tp_lx, p->tp_ly, p->tp_x, p->tp_y, ox, oy, os, p->area);
    if(p->area && ox > MIN_TP_XDIS_FOR_SWAP && ox < MAX_TP_XDIS_FOR_SWAP
        && oy < ox/3 //MAX_TP_YDIS_FOR_SWAP
        && os < MAX_TP_DELAY_FOR_SWAP) {
        // 20230724: maybe area is NULL here. don't know why.
        kc = p->area->slider_keycode;
    }

    tp_track_slider = NULL;
    pt_reset_slide_touch(p);
    return kc;
}


// 20230721: return 1: filter the tch, don't report. 0: report normal.
static int pt_filter_by_slider(struct pt_mt_data *md,
    struct pt_touch *tch, int num_cur_tch)
{
	struct device *dev = md->dev;
	//struct pt_sysinfo *si = md->si;
	//int max_tch = si->sensing_conf_data.max_tch;
	//int rc = 0;
    int     pt_t = tch->abs[PT_TCH_T] - md->t_min;
    //bool    pt_off = tch->abs[PT_TCH_E] == PT_EV_LIFTOFF;
    int     x = tch->abs[PT_TCH_X];
    int     y = tch->abs[PT_TCH_Y];
    int     tip = tch->abs[PT_TCH_TIP];
    int     sc;
    const struct pt_slider_area* pa;
    int     tracking = 0;

    if(slider_track_mask == 0) {
        return 0;  // 0: don't filter the tp-event,report to user.
    }

    pt_debug(dev, DL_LISTEN,"%s: num_tch=%d,t=%d,x=%d,y=%d,tip=%d,smask=0x%02x,tracker=%p\n",
		__func__, num_cur_tch, pt_t, x, y, tip, slider_track_mask, tp_track_slider);

    if(pt_filter_by_fix_slider_area(pt_t, x, y) == 0) {
        pt_debug(dev, DL_LISTEN,"%s: filter_by_fix,x=%d,y=%d,smask=0x%x\n",
		    __func__, x, y, slider_track_mask, sc);
        if(slider_track_mask) {
            pt_dump_slider_touch(dev);
            pt_filter_clear_slider_by_time(dev);
            pt_dump_slider_touch(dev);
        }
        return 0;
    }

    // for debug.
    //pt_dump_slider_touch();

    sc = pt_filter_check_multi_slider();
    if(sc != 1) {
        pt_debug(dev, DL_LISTEN,"%s: smask=0x%02x,sc=%d,abort cause multi-slider touching\n",
		    __func__, slider_track_mask, sc);
		return 0;
    }

    // 20230724: maybe a swap.TP maybu multi touching.
    if(!tp_track_slider) {
        if(!tip) {
            return 0;
        }
        pa = pt_get_slider_area(dev,x, y);
        if(!pa) {
            // clear the slider-state.
            pt_filter_clear_slider_by_time(dev);
            pt_dump_slider_touch(dev);
        } else {
            // set the tp-tracking.
            tp_track_slider = pt_get_slider_touch(dev,x, y, pt_t, pa);
            if(!tp_track_slider){
                pt_debug(dev, DL_LISTEN,"%s: No slider touch Found: x=%d,y=%d,pa=%c\n",
                    __func__, x, y, pa->id);

                pt_dump_slider_touch(dev);
                pt_dump_slider_area(dev);
            } else {
                tracking = 1;
            }
        }
    } else {
        // only tracking one swap at the same time.
        if(tp_track_slider->tp_t == pt_t) {
            if(!tip) {
                // 20230724: check x/y distance and uptime to deside weather report key event.
                int key_code = pt_slider_touch_up(dev);
                pt_debug(dev, DL_ERROR,"%s: tp-touchup: x=%d,y=%d, t=%d,keycode=%d\n",
                    __func__, x, y, pt_t, key_code);
                if(key_code != 0) {
                    // report the keycode.
                    //md->input->repeat_key();
					input_report_key(md->input_key, key_code, 1);
					input_report_key(md->input_key, key_code, 0);
					input_sync(md->input_key);
                }
            } else {
                tp_track_slider->tp_lx = x;
                tp_track_slider->tp_ly = y;
            }
            tracking = 1; // 1: filter the tp-event,don't report to user.
        }
    }
	return tracking;
}



// 20230724: monitor the slider touch event. tip:1  touch-down, 0: touch-up.
// when tip = 0,the x/y is not correct.
static void pt_slider_listener(struct device *dev,int t, int x, int y, int tip)
{
    pt_debug(dev, DL_LISTEN,"%s: Got slider touch[%d]: x=%d,y=%d,tip=%d\n", __func__,
		t, x, y, tip);
	pt_update_slider_touch(dev,t, x, y, tip);
	if(!tip) pt_dump_slider_touch(dev);
}
static int pt_setup_input_device(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	int rc = 0, i;

	md->input_key = input_allocate_device();
	if (!md->input_key) {
		pt_debug(dev, DL_ERROR,
			"%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENODEV;
	}
	md->input_key->name = "slider";
	md->input_key->phys = "input/slider";
	md->input_key->id.bustype = BUS_I2C;
	md->input_key->id.version = 0727;
	__set_bit(EV_KEY, md->input_key->evbit);
	__set_bit(292, md->input_key->keybit);
	__set_bit(293, md->input_key->keybit);
	__set_bit(294, md->input_key->keybit);
	__set_bit(KEY_BACK, md->input_key->keybit);
	rc = input_register_device(md->input_key);
	if (rc < 0)
		pt_debug(dev, DL_ERROR,
			"%s: Error, failed register input device r=%d\n",
			__func__, rc);

	return rc;

}

int pt_slider_probe(struct device *dev)
{
	struct pt_core_data *cd = dev_get_drvdata(dev);
	struct pt_mt_data *md = &cd->md;
	int rc = 0, i;

	pt_debug(dev, DL_ERROR,
		"%s: >>>>>> Register slider listerner:%s <<<<<<\n", __func__,SLIDER_DRV_VER);

	mutex_init(&slider_mutex);

    for(i = 0; i < MAX_SLIDER_TRACK; i++) {
        slider_touchs[i].t = -1;
        slider_touchs[i].up_time = UPTIME_IDLE; //ktime_set(0, 0);
    }

    cyttsp5_register_listener(pt_slider_listener);
    md->input_filter = pt_filter_by_slider;

    // 20230724: debug.
    pt_dump_slider_area(dev);
	pt_setup_input_device(dev);

	return rc;
}


