/*
 * drivers/input/misc/ps_detection_check.c
 *
 * Copyright (c) 2016, Vasilii Kovalev <vgdn1942@gmail.com>
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
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <asm-generic/cputime.h>
#include <linux/input/pocket_mod.h>

/********************* SYSFS INTERFACE ***********************/
static ssize_t ps_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", epl259x_pocket_detection_check());
}

static ssize_t ps_state_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return 0;
}

static DEVICE_ATTR(ps_state_enable, 0755,
		ps_state_show, ps_state_set);

/********************************************************************************************/
static struct attribute *ps_detection_check_attributes[] =
{
	&dev_attr_ps_state_enable.attr,
	NULL
};

static struct attribute_group ps_detection_check_group =
{
	.attrs  = ps_detection_check_attributes,
};

extern struct kobject *ps_detection_check_kobj;

static int ps_detection_check_init_sysfs(void) {

	int rc = 0;

	struct kobject *ps_detection_check_kobj;
	ps_detection_check_kobj = kobject_create_and_add("ps_debug", NULL);

	dev_attr_ps_state_enable.attr.name = "ps_state";

	rc = sysfs_create_group(ps_detection_check_kobj,
			&ps_detection_check_group);

	if (unlikely(rc < 0))
		pr_err("ps_detection_check: sysfs_create_group failed: %d\n", rc);

	return rc;

}

module_init(ps_detection_check_init_sysfs);
