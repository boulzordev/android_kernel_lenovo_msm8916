/*
 * drivers/input/touchscreen/wake_gestures.c
 *
 *
 * Copyright (c) 2013, Dennis Rassmann <showp1984@gmail.com>
 * Copyright (c) 2013-16 Aaron Segaert <asegaert@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/input/wake_gestures.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>
/*
#include <linux/wakelock.h>
*/
#ifdef CONFIG_POCKET_MOD
#include <linux/input/pocket_mod.h>
#endif

/* Tuneables */
#define WG_DEBUG		0
#define WG_DEFAULT		0
#define DT2W_DEFAULT		0
#define S2W_DEFAULT		0
#define S2S_DEFAULT		0
#define WG_PWRKEY_DUR           60
//#define EXP_KEYPAD_S2W

/* Lenovo A6020a46 */
#ifdef EXP_KEYPAD_S2W
#define SWEEP_Y_MAX             2100
#else
#define SWEEP_Y_MAX             1920
#endif
#define SWEEP_X_MAX             1080
#define SWEEP_EDGE		90
#define SWEEP_Y_LIMIT           SWEEP_Y_MAX-SWEEP_EDGE
#define SWEEP_X_LIMIT           SWEEP_X_MAX-SWEEP_EDGE
#define SWEEP_X_B1              299
#define SWEEP_X_B2              620
#define SWEEP_Y_START		800
#define SWEEP_X_START		540
#define SWEEP_X_FINAL           270
#define SWEEP_Y_NEXT            135
#define DT2W_FEATHER		150
#define DT2W_TIME 		500

#define S2W_KEY_LEFT			180
#define S2W_KEY_CENTER			540
#define S2W_KEY_RIGHT			900

/* Wake Gestures */
#define SWEEP_TIMEOUT		300
#define TRIGGER_TIMEOUT		500
#define WAKE_GESTURE		0x0b
#define SWEEP_RIGHT		0x01
#define SWEEP_LEFT		0x02
#define SWEEP_UP		0x04
#define SWEEP_DOWN		0x08
#define VIB_ENABLE 		1
#define VIB_STRENGTH 		50

#define WAKE_GESTURES_ENABLED	1

#define LOGTAG			"WG"

#if (WAKE_GESTURES_ENABLED)
int gestures_switch = WG_DEFAULT;
static struct input_dev *gesture_dev;
#endif

/* Resources */
int s2w_switch = S2W_DEFAULT;
int s2w_switch_temp;
int s2w_switch_bin;
int s2w_switch_right = S2W_DEFAULT;
int s2w_switch_right_temp;
int s2w_switch_left = S2W_DEFAULT;
int s2w_switch_left_temp;
int s2w_switch_up = S2W_DEFAULT;
int s2w_switch_up_temp;
int s2w_switch_down = S2W_DEFAULT;
int s2w_switch_down_temp;
bool s2w_switch_changed = false;
int dt2w_switch = DT2W_DEFAULT;
int dt2w_switch_temp; 
bool dt2w_switch_changed = false;
static int s2s_switch = S2S_DEFAULT;
static int touch_x = 0, touch_y = 0;
static bool touch_x_called = false, touch_y_called = false;
static bool exec_countx = true, exec_county = true, exec_count = true;
static bool barrierx[2] = {false, false}, barriery[2] = {false, false};
static int firstx = 0, firsty = 0;
static int sweep_y_limit = SWEEP_Y_LIMIT;
static int sweep_x_limit = SWEEP_X_LIMIT;
static unsigned long firstx_time = 0, firsty_time = 0;
static unsigned long pwrtrigger_time[2] = {0, 0};
static unsigned long long tap_time_pre = 0;
static int touch_nr = 0, x_pre = 0, y_pre = 0;
#ifdef EXP_KEYPAD_S2W
int s2w_keypad_swipe_length = 3;
#endif
static bool touch_cnt = true;
static int vib_strength = VIB_STRENGTH;
static int vib_enable = VIB_ENABLE;

static struct input_dev * wake_dev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *s2w_input_wq;
static struct workqueue_struct *dt2w_input_wq;
static struct work_struct s2w_input_work;
static struct work_struct dt2w_input_work;

extern void qpnp_kernel_vib_enable(int value);

static bool is_suspended(void)
{
		return scr_suspended_ft();
}

/* Wake Gestures */
#if (WAKE_GESTURES_ENABLED)
static void report_gesture(int gest)
{
	pwrtrigger_time[1] = pwrtrigger_time[0];
	pwrtrigger_time[0] = ktime_to_ms(ktime_get());	

	if (pwrtrigger_time[0] - pwrtrigger_time[1] < TRIGGER_TIMEOUT)
		return;

	input_report_rel(gesture_dev, WAKE_GESTURE, gest);
	input_sync(gesture_dev);
}
#endif

/* PowerKey work func */
static void wake_presspwr(struct work_struct * wake_presspwr_work) {
	if (!mutex_trylock(&pwrkeyworklock))
		return;

	input_event(wake_dev, EV_KEY, KEY_POWER, 1);
	input_event(wake_dev, EV_SYN, 0, 0);
	msleep(WG_PWRKEY_DUR);
	input_event(wake_dev, EV_KEY, KEY_POWER, 0);
	input_event(wake_dev, EV_SYN, 0, 0);
	msleep(WG_PWRKEY_DUR);
	mutex_unlock(&pwrkeyworklock);

	if (vib_enable == 1) {
		qpnp_kernel_vib_enable(vib_strength);
	}

	return;
}
static DECLARE_WORK(wake_presspwr_work, wake_presspwr);

/* PowerKey trigger */
static void wake_pwrtrigger(void) {
	pwrtrigger_time[1] = pwrtrigger_time[0];
	pwrtrigger_time[0] = ktime_to_ms(ktime_get());

#ifdef CONFIG_POCKET_MOD
	if ((pwrtrigger_time[0] - pwrtrigger_time[1] < TRIGGER_TIMEOUT) || (device_is_pocketed()))
#else
	if (pwrtrigger_time[0] - pwrtrigger_time[1] < TRIGGER_TIMEOUT)
#endif
		return;

	schedule_work(&wake_presspwr_work);

        return;
}


/* Doubletap2wake */

static void doubletap2wake_reset(void) {
	exec_count = true;
	touch_nr = 0;
	tap_time_pre = 0;
	x_pre = 0;
	y_pre = 0;
}

static unsigned int calc_feather(int coord, int prev_coord) {
	int calc_coord = 0;
	calc_coord = coord-prev_coord;
	if (calc_coord < 0)
		calc_coord = calc_coord * (-1);
	return calc_coord;
}

/* init a new touch */
static void new_touch(int x, int y) {
	tap_time_pre = ktime_to_ms(ktime_get());
	x_pre = x;
	y_pre = y;
	touch_nr++;
}

/* Doubletap2wake main function */
static void detect_doubletap2wake(int x, int y, bool st)
{
        bool single_touch = st;
#if WG_DEBUG
        pr_info(LOGTAG"x,y(%4d,%4d) tap_time_pre:%llu\n",
                x, y, tap_time_pre);
#endif
	if (x < SWEEP_EDGE || x > sweep_x_limit)
		return;
	if (y < SWEEP_EDGE || y > sweep_y_limit)
		return;

	if ((single_touch) && (dt2w_switch) && (exec_count) && (touch_cnt)) {
		touch_cnt = false;
		if (touch_nr == 0) {
			new_touch(x, y);
		} else if (touch_nr == 1) {
			if ((calc_feather(x, x_pre) < DT2W_FEATHER) &&
			    (calc_feather(y, y_pre) < DT2W_FEATHER) &&
			    ((ktime_to_ms(ktime_get())-tap_time_pre) < DT2W_TIME))
				touch_nr++;
			else {
				doubletap2wake_reset();
				new_touch(x, y);
			}
		} else {
			doubletap2wake_reset();
			new_touch(x, y);
		}
		if ((touch_nr > 1)) {
			exec_count = false;
#if (WAKE_GESTURES_ENABLED)
			if (gestures_switch) {
				report_gesture(5);
			} else {
#endif
				wake_pwrtrigger();
#if (WAKE_GESTURES_ENABLED)
			}
#endif
			doubletap2wake_reset();
		}
	}
}


/* Sweep2wake/Sweep2sleep */
static void sweep2wake_reset(void) {

	exec_countx = true;
	barrierx[0] = false;
	barrierx[1] = false;
	firstx = 0;
	firstx_time = 0;

	exec_county = true;
	barriery[0] = false;
	barriery[1] = false;
	firsty = 0;
	firsty_time = 0;
}

/* Sweep2wake main functions*/
static void detect_sweep2wake_v(int x, int y, bool st)
{
	int prevy = 0, nexty = 0;
        bool single_touch = st;

	if (firsty == 0) {
		firsty = y;
		firsty_time = ktime_to_ms(ktime_get());
	}

#if WG_DEBUG
        pr_info(LOGTAG"s2w vert  x,y(%4d,%4d) single:%s\n",
                x, y, (single_touch) ? "true" : "false");
#endif

	//sweep up
	if (firsty > SWEEP_Y_START && single_touch && s2w_switch & SWEEP_UP) {
		prevy = firsty;
		nexty = prevy - SWEEP_Y_NEXT;
		if (barriery[0] == true || (y < prevy && y > nexty)) {
			prevy = nexty;
			nexty -= SWEEP_Y_NEXT;
			barriery[0] = true;
			if (barriery[1] == true || (y < prevy && y > nexty)) {
				prevy = nexty;
				barriery[1] = true;
				if (y < prevy) {
					if (y < (nexty - SWEEP_Y_NEXT)) {
						if (exec_county && (ktime_to_ms(ktime_get()) - firsty_time < SWEEP_TIMEOUT)) {
#if (WAKE_GESTURES_ENABLED)
							if (gestures_switch) {
								report_gesture(3);
							} else {
#endif
								wake_pwrtrigger();
#if (WAKE_GESTURES_ENABLED)
							}		
#endif								
							exec_county = false;
						}
					}
				}
			}
		}
	//sweep down
	} else if (firsty <= SWEEP_Y_START && single_touch && s2w_switch & SWEEP_DOWN) {
		prevy = firsty;
		nexty = prevy + SWEEP_Y_NEXT;
		if (barriery[0] == true || (y > prevy && y < nexty)) {
			prevy = nexty;
			nexty += SWEEP_Y_NEXT;
			barriery[0] = true;
			if (barriery[1] == true || (y > prevy && y < nexty)) {
				prevy = nexty;
				barriery[1] = true;
				if (y > prevy) {
					if (y > (nexty + SWEEP_Y_NEXT)) {
						if (exec_county && (ktime_to_ms(ktime_get()) - firsty_time < SWEEP_TIMEOUT)) {
#if (WAKE_GESTURES_ENABLED)
							if (gestures_switch) {
								report_gesture(4);
							} else {
#endif
								wake_pwrtrigger();
#if (WAKE_GESTURES_ENABLED)
							}								
#endif
							exec_county = false;
						}
					}
				}
			}
		}
	}
	
}

static void detect_sweep2wake_h(int x, int y, bool st, bool scr_suspended)
{
        int prevx = 0, nextx = 0;
        bool single_touch = st;

	if (!scr_suspended && y < sweep_y_limit) {
		sweep2wake_reset();
		return;
	}

	if (firstx == 0) {
		firstx = x;
		firstx_time = ktime_to_ms(ktime_get());
	}

#if WG_DEBUG
        pr_info(LOGTAG"s2w Horz x,y(%4d,%4d) wake:%s\n",
                x, y, (scr_suspended) ? "true" : "false");
#endif
#ifdef EXP_KEYPAD_S2W
	if (x_pre) {
		if (s2w_keypad_swipe_length == 2) {
			if (x == S2W_KEY_CENTER) {
				if (x_pre == S2W_KEY_LEFT) {
					if (scr_suspended) {
#if WG_DEBUG
						pr_info(LOGTAG"LTR: keypad: ON\n");
#endif
						wake_pwrtrigger();
					}
				} else if (x_pre == S2W_KEY_RIGHT) {
					if (!scr_suspended) {
#if WG_DEBUG
						pr_info(LOGTAG"RTL: keypad: OFF\n");
#endif
						wake_pwrtrigger();
					}
				}
			}
		} else if (s2w_keypad_swipe_length == 3) {
			if (x_pre == S2W_KEY_CENTER) {
				if (x == S2W_KEY_LEFT) {
					if (!scr_suspended) {
#if WG_DEBUG
						pr_info(LOGTAG"RTL: keypad: OFF\n");
#endif
						wake_pwrtrigger();
					}
				} else if (x == S2W_KEY_RIGHT) {
					if (scr_suspended) {
#if WG_DEBUG
						pr_info(LOGTAG"LTR: keypad: ON\n");
#endif
						wake_pwrtrigger();
					}
				}
			}
		}
		return;
	}
#endif // EXP_KEYPAD_S2W
	//left->right
	if (firstx < SWEEP_X_START && single_touch &&
			((scr_suspended && (s2w_switch & SWEEP_RIGHT)) ||
			(!scr_suspended && (s2s_switch & SWEEP_RIGHT)))) {
		prevx = 0;
		nextx = SWEEP_X_B1;
		if ((barrierx[0] == true) ||
		   ((x > prevx) && (x < nextx))) {
			prevx = nextx;
			nextx = SWEEP_X_B2;
			barrierx[0] = true;
			if ((barrierx[1] == true) ||
			   ((x > prevx) && (x < nextx))) {
				prevx = nextx;
				barrierx[1] = true;
				if (x > prevx) {
					if (x > (SWEEP_X_MAX - SWEEP_X_FINAL)) {
						if (exec_countx && (ktime_to_ms(ktime_get()) - firstx_time < SWEEP_TIMEOUT)) {
#if (WAKE_GESTURES_ENABLED)
							if (gestures_switch && scr_suspended) {
								report_gesture(1);
							} else {
#endif
								wake_pwrtrigger();
#if (WAKE_GESTURES_ENABLED)
							}
#endif							
							exec_countx = false;
						}
					}
				}
			}
		}
	//right->left
	} else if (firstx >= SWEEP_X_START && single_touch &&
			((scr_suspended && (s2w_switch & SWEEP_LEFT)) ||
			(!scr_suspended && (s2s_switch & SWEEP_LEFT)))) {
		prevx = (SWEEP_X_MAX - SWEEP_X_FINAL);
		nextx = SWEEP_X_B2;
		if ((barrierx[0] == true) ||
		   ((x < prevx) && (x > nextx))) {
			prevx = nextx;
			nextx = SWEEP_X_B1;
			barrierx[0] = true;
			if ((barrierx[1] == true) ||
			   ((x < prevx) && (x > nextx))) {
				prevx = nextx;
				barrierx[1] = true;
				if (x < prevx) {
					if (x < SWEEP_X_FINAL) {
						if (exec_countx) {
#if (WAKE_GESTURES_ENABLED)
							if (gestures_switch && scr_suspended) {
								report_gesture(2);
							} else {
#endif
								wake_pwrtrigger();
#if (WAKE_GESTURES_ENABLED)
							}		
#endif							
							exec_countx = false;
						}
					}
				}
			}
		}
	}
}

int in_phone_call(void) {
	return var_in_phone_call;
}

static void s2w_input_callback(struct work_struct *unused)
{
	if (in_phone_call()) {
#if WG_DEBUG
		pr_info("Sweep2Wake: in phone call! return!\n");
#endif
		return;
	}

	detect_sweep2wake_h(touch_x, touch_y, true, is_suspended());
	if (is_suspended())
		detect_sweep2wake_v(touch_x, touch_y, true);

	return;
}

static void dt2w_input_callback(struct work_struct *unused)
{
	if (in_phone_call()) {
#if WG_DEBUG
		pr_info("DoubleTap2Wake: in phone call! return!\n");
#endif
		return;
	}

	if (is_suspended() && dt2w_switch)
		detect_doubletap2wake(touch_x, touch_y, true);
	return;
}

static void wg_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value)
{
	if (is_suspended() && code == ABS_MT_POSITION_X) {
		value -= 5000;
	}
	
#if WG_DEBUG
	pr_info("wg: code: %s|%u, val: %i\n",
		((code==ABS_MT_POSITION_X) ? "X" :
		(code==ABS_MT_POSITION_Y) ? "Y" :
		(code==ABS_MT_TRACKING_ID) ? "ID" :
		"undef"), code, value);
#endif

	if (code == ABS_MT_SLOT) {
		sweep2wake_reset();
		doubletap2wake_reset();
		return;
	}

	if (code == ABS_MT_TRACKING_ID && value == -1) {
#ifdef EXP_KEYPAD_S2W
		if (x_pre == 0)
#endif
		sweep2wake_reset();
		touch_cnt = true;
		queue_work_on(0, dt2w_input_wq, &dt2w_input_work);
		return;
	}

	if (code == ABS_MT_POSITION_X) {
#ifdef EXP_KEYPAD_S2W
		if ((value == S2W_KEY_LEFT) || (value == S2W_KEY_CENTER) || (value == S2W_KEY_RIGHT)) {
			if (x_pre == 0) {
				if ((value == S2W_KEY_LEFT) || (value == S2W_KEY_RIGHT)) {
					if (scr_suspended) {
						if (value == S2W_KEY_LEFT) {
							x_pre = value;
						}
					} else {
						if (value == S2W_KEY_RIGHT) {
							x_pre = value;
						}
					}
				}
			} else {
				if (s2w_keypad_swipe_length == 3) {
					if (value == S2W_KEY_CENTER)
						x_pre = value;

					if (x_pre == S2W_KEY_CENTER) {
						if (touch_x == S2W_KEY_LEFT)
							if (value == S2W_KEY_RIGHT)
								if (scr_suspended)
									sweep2wake_reset();

						if (touch_x == S2W_KEY_RIGHT)
							if (value == S2W_KEY_LEFT)
								if (!scr_suspended)
									sweep2wake_reset();
					}
				}
			}
		}
#endif // EXP_KEYPAD_S2W
		touch_x = value;
		touch_x_called = true;
	}

	if (code == ABS_MT_POSITION_Y) {
		touch_y = value;
#ifdef EXP_KEYPAD_S2W
		if (x_pre) {
			if (value < SWEEP_Y_MAX) {
				x_pre = 0;
			}
		}
#endif
		touch_y_called = true;
	}

#ifdef EXP_KEYPAD_S2W
	if (touch_x_called && (x_pre ? true : (touch_y_called ? true : false))) {
#else
	if (touch_x_called && touch_y_called) {
#endif // EXP_KEYPAD_S2W
		touch_x_called = false;
		touch_y_called = false;
		queue_work_on(0, s2w_input_wq, &s2w_input_work);
	} else if (!is_suspended() && touch_x_called && !touch_y_called) {
		touch_x_called = false;
		touch_y_called = false;
		queue_work_on(0, s2w_input_wq, &s2w_input_work);
	}
}

static int input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "ft5x06_ts")) {
		return 0;
	} else {
		return 1;
	}
	return 0;
}

static int wg_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "wg";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void wg_input_disconnect(struct input_handle *handle) {
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id wg_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler wg_input_handler = {
	.event		= wg_input_event,
	.connect	= wg_input_connect,
	.disconnect	= wg_input_disconnect,
	.name		= "wg_inputreq",
	.id_table	= wg_ids,
};

int s2w_switch_set(int s2w_switch_right, int s2w_switch_left, int s2w_switch_up, int s2w_switch_down) {
	if ((s2w_switch_right == 0) && (s2w_switch_left == 0) && (s2w_switch_up == 0) && (s2w_switch_down == 0)) {
		s2w_switch = 0;
		s2w_switch_bin = 0;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 0) && (s2w_switch_up == 0) && (s2w_switch_down == 0)) {
		s2w_switch = 1;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 1) && (s2w_switch_up == 0) && (s2w_switch_down == 0)) {
		s2w_switch = 2;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 1) && (s2w_switch_up == 0) && (s2w_switch_down == 0)) {
		s2w_switch = 3;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 0) && (s2w_switch_up == 1) && (s2w_switch_down == 0)) {
		s2w_switch = 4;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 0) && (s2w_switch_up == 1) && (s2w_switch_down == 0)) {
		s2w_switch = 5;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 1) && (s2w_switch_up == 1) && (s2w_switch_down == 0)) {
		s2w_switch = 6;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 1) && (s2w_switch_up == 1) && (s2w_switch_down == 0)) {
		s2w_switch = 7;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 0) && (s2w_switch_up == 0) && (s2w_switch_down == 1)) {
		s2w_switch = 8;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 0) && (s2w_switch_up == 0) && (s2w_switch_down == 1)) {
		s2w_switch = 9;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 1) && (s2w_switch_up == 0) && (s2w_switch_down == 1)) {
		s2w_switch = 10;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 1) && (s2w_switch_up == 0) && (s2w_switch_down == 1)) {
		s2w_switch = 11;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 0) && (s2w_switch_up == 1) && (s2w_switch_down == 1)) {
		s2w_switch = 12;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 0) && (s2w_switch_up == 1) && (s2w_switch_down == 1)) {
		s2w_switch = 13;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 0) && (s2w_switch_left == 1) && (s2w_switch_up == 1) && (s2w_switch_down == 1)) {
		s2w_switch = 14;
		s2w_switch_bin = 1;
	} else if ((s2w_switch_right == 1) && (s2w_switch_left == 1) && (s2w_switch_up == 1) && (s2w_switch_down == 1)) {
		s2w_switch = 15;
		s2w_switch_bin = 1;
	}
	return s2w_switch;
}

/*
 * SYSFS stuff below here
 */
#ifdef EXP_KEYPAD_S2W
static ssize_t s2w_sweep2wake_touchkey_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_keypad_swipe_length);

	return count;
}

static ssize_t s2w_sweep2wake_touchkey_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (buf[0] >= '1' && buf[0] <= '4' && buf[1] == '\n')
		if (s2w_keypad_swipe_length != buf[0] - '0')
			s2w_keypad_swipe_length = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(sweep2wake_touchkey, 0755,
	s2w_sweep2wake_touchkey_show, s2w_sweep2wake_touchkey_dump);
#endif // EXP_KEYPAD_S2W

/*****************************************************************/

static ssize_t sweep2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch_bin);

	return count;
}

static ssize_t sweep2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_temp);
	if (s2w_switch_temp < 0 || s2w_switch_temp > 1)
		s2w_switch_temp = 0;
		
	if (!is_suspended())
	    if (s2w_switch_temp == 1) {
		    s2w_switch_right = 1;
		    s2w_switch_left = 1;
		    s2w_switch_up = 1;
		    s2w_switch_down = 1;
		    s2w_switch = s2w_switch_set(s2w_switch_right, s2w_switch_left, s2w_switch_up, s2w_switch_down);
	    } else {
		    s2w_switch_right = 0;
		    s2w_switch_left = 0;
		    s2w_switch_up = 0;
		    s2w_switch_down = 0;
		    s2w_switch = s2w_switch_set(s2w_switch_right, s2w_switch_left, s2w_switch_up, s2w_switch_down);
		}
	else
		s2w_switch_changed = true;

	return count;
}

static DEVICE_ATTR(sweep2wake, 0755,
	sweep2wake_show, sweep2wake_dump);

/*****************************************************************/

static ssize_t sweep2wake_all_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch);

	return count;
}

static ssize_t sweep2wake_all_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_temp);
	if (s2w_switch_temp < 0 || s2w_switch_temp > 15)
		s2w_switch_temp = 0;
		
	if (!is_suspended())
		s2w_switch = s2w_switch_temp;
	else
		s2w_switch_changed = true;

	return count;
}

static DEVICE_ATTR(sweep2wake_all, 0755,
	sweep2wake_all_show, sweep2wake_all_dump);

/*****************************************************************/

static ssize_t sweep2wake_right_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch_right);

	return count;
}

static ssize_t sweep2wake_right_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_right_temp);
	if (s2w_switch_right_temp < 0 || s2w_switch_right_temp > 1)
		s2w_switch_right_temp = 0;
		
	if (!is_suspended()) {
		s2w_switch_right = s2w_switch_right_temp;
		s2w_switch = s2w_switch_set(s2w_switch_right, s2w_switch_left, s2w_switch_up, s2w_switch_down);
	} else {
		s2w_switch_changed = true;
	}

	return count;
}

static DEVICE_ATTR(sweep2wake_right, 0755,
	sweep2wake_right_show, sweep2wake_right_dump);

/*****************************************************************/

static ssize_t sweep2wake_left_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch_left);

	return count;
}

static ssize_t sweep2wake_left_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_left_temp);
	if (s2w_switch_left_temp < 0 || s2w_switch_left_temp > 1)
		s2w_switch_left_temp = 0;
		
	if (!is_suspended()) {
		s2w_switch_left = s2w_switch_left_temp;
		s2w_switch = s2w_switch_set(s2w_switch_right, s2w_switch_left, s2w_switch_up, s2w_switch_down);
	} else {
		s2w_switch_changed = true;
	}

	return count;
}

static DEVICE_ATTR(sweep2wake_left, 0755,
	sweep2wake_left_show, sweep2wake_left_dump);

/*****************************************************************/

static ssize_t sweep2wake_up_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch_up);

	return count;
}

static ssize_t sweep2wake_up_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_up_temp);
	if (s2w_switch_up_temp < 0 || s2w_switch_up_temp > 1)
		s2w_switch_up_temp = 0;
		
	if (!is_suspended()) {
		s2w_switch_up = s2w_switch_up_temp;
		s2w_switch = s2w_switch_set(s2w_switch_right, s2w_switch_left, s2w_switch_up, s2w_switch_down);
	} else {
		s2w_switch_changed = true;
	}

	return count;
}

static DEVICE_ATTR(sweep2wake_up, 0755,
	sweep2wake_up_show, sweep2wake_up_dump);

/*****************************************************************/

static ssize_t sweep2wake_down_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", s2w_switch_down);

	return count;
}

static ssize_t sweep2wake_down_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2w_switch_down_temp);
	if (s2w_switch_down_temp < 0 || s2w_switch_down_temp > 1)
		s2w_switch_down_temp = 0;
		
	if (!is_suspended()) {
		s2w_switch_down = s2w_switch_down_temp;
		s2w_switch = s2w_switch_set(s2w_switch_right, s2w_switch_left, s2w_switch_up, s2w_switch_down);
	} else {
		s2w_switch_changed = true;
	}

	return count;
}

static DEVICE_ATTR(sweep2wake_down, 0755,
	sweep2wake_down_show, sweep2wake_down_dump);

/*****************************************************************/
/*
static ssize_t sweep2sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", s2s_switch);
	return count;
}

static ssize_t sweep2sleep_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &s2s_switch);
	if (s2s_switch < 0 || s2s_switch > 3)
		s2s_switch = 0;				
				
	return count;
}

static DEVICE_ATTR(sweep2sleep, 0755,
	sweep2sleep_show, sweep2sleep_dump);
*/
/*****************************************************************/

static ssize_t doubletap2wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", dt2w_switch);

	return count;
}

static ssize_t doubletap2wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &dt2w_switch_temp);
	if (dt2w_switch_temp < 0 || dt2w_switch_temp > 2)
		dt2w_switch_temp = 0;

	if (!is_suspended())
	    if (dt2w_switch_temp == 1 || dt2w_switch_temp == 2)
	        dt2w_switch = 1;
	    else
		    dt2w_switch = dt2w_switch_temp;
	else
		dt2w_switch_changed = true;

	return count;
}

static DEVICE_ATTR(doubletap2wake, 0755,
	doubletap2wake_show, doubletap2wake_dump);

/*****************************************************************/

#if (WAKE_GESTURES_ENABLED)
static ssize_t wake_gestures_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", gestures_switch);
	return count;
}
static ssize_t wake_gestures_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ", &gestures_switch);
	if (gestures_switch < 0 || gestures_switch > 1)
		gestures_switch = 0;	
	return count;
}

static DEVICE_ATTR(wake_gestures, 0755,
	wake_gestures_show, wake_gestures_dump);
#endif	

/*****************************************************************/

static ssize_t vib_strength_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", vib_strength);
	return count;
}

static ssize_t vib_strength_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ",&vib_strength);
	if (vib_strength < 0 || vib_strength > 90)
		vib_strength = 50;

	return count;
}

static DEVICE_ATTR(vib_strength, 0755,
	vib_strength_show, vib_strength_dump);

/*****************************************************************/

static ssize_t vib_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	count += sprintf(buf, "%d\n", vib_enable);
	return count;
}

static ssize_t vib_enable_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d ",&vib_enable);
	if (vib_enable < 0 || vib_enable > 1)
		vib_enable = 1;

	return count;
}

static DEVICE_ATTR(vib_enable, 0755,
	vib_enable_show, vib_enable_dump);

/*
 * INIT / EXIT stuff below here
 */

struct kobject *android_touch_kobj;
EXPORT_SYMBOL_GPL(android_touch_kobj);

static int __init wake_gestures_init(void)
{
	int rc = 0;

	wake_dev = input_allocate_device();
	if (!wake_dev) {
		pr_err("Failed to allocate wake_dev\n");
		goto err_alloc_dev;
	}

	input_set_capability(wake_dev, EV_KEY, KEY_POWER);
	wake_dev->name = "wg_pwrkey";
	wake_dev->phys = "wg_pwrkey/input0";

	rc = input_register_device(wake_dev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	rc = input_register_handler(&wg_input_handler);
	if (rc)
		pr_err("%s: Failed to register wg_input_handler\n", __func__);

	s2w_input_wq = create_workqueue("s2wiwq");
	if (!s2w_input_wq) {
		pr_err("%s: Failed to create s2wiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&s2w_input_work, s2w_input_callback);
		
	dt2w_input_wq = create_workqueue("dt2wiwq");
	if (!dt2w_input_wq) {
		pr_err("%s: Failed to create dt2wiwq workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&dt2w_input_work, dt2w_input_callback);
		
#if (WAKE_GESTURES_ENABLED)
	gesture_dev = input_allocate_device();
	if (!gesture_dev) {
		pr_err("Failed to allocate gesture_dev\n");
		goto err_alloc_dev;
	}
	
	gesture_dev->name = "wake_gesture";
	gesture_dev->phys = "wake_gesture/input0";
	input_set_capability(gesture_dev, EV_REL, WAKE_GESTURE);

	rc = input_register_device(gesture_dev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_gesture_dev;
	}
#endif

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		pr_warn("%s: android_touch_kobj create_and_add failed\n", __func__);
	}
#ifdef EXP_KEYPAD_S2W
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_touchkey.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake_touchkey\n", __func__);
	}
#endif // EXP_KEYPAD_S2W
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_all.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake_all\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_right.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake_right\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_left.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake_left\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_up.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake_up\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2wake_down.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2wake_down\n", __func__);
	}
/*
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_sweep2sleep.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for sweep2sleep\n", __func__);
	}
*/
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_doubletap2wake.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for doubletap2wake\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_vib_strength.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for vib_strength\n", __func__);
	}
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_vib_enable.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for vib_enable\n", __func__);
	}
#if (WAKE_GESTURES_ENABLED)
	rc = sysfs_create_file(android_touch_kobj, &dev_attr_wake_gestures.attr);
	if (rc) {
		pr_warn("%s: sysfs_create_file failed for wake_gestures\n", __func__);
	}

	return 0;

err_gesture_dev:
	input_free_device(gesture_dev);
err_input_dev:
	input_free_device(wake_dev);
err_alloc_dev:
#endif

	return 0;
}

static void __exit wake_gestures_exit(void)
{
	kobject_del(android_touch_kobj);
	input_unregister_handler(&wg_input_handler);
	destroy_workqueue(s2w_input_wq);
	destroy_workqueue(dt2w_input_wq);
	input_unregister_device(wake_dev);
	input_free_device(wake_dev);
#if (WAKE_GESTURES_ENABLED)	
	input_unregister_device(gesture_dev);
	input_free_device(gesture_dev);
#endif

	return;
}

module_init(wake_gestures_init);
module_exit(wake_gestures_exit);
