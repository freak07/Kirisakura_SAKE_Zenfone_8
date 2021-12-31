// SPDX-License-Identifier: GPL-2.0-only
/*
 *  user_space.c - A simple user space Thermal events notifier
 *
 *  Copyright (C) 2012 Intel Corp
 *  Copyright (C) 2012 Durgadoss R <durgadoss.r@intel.com>
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/thermal.h>
#include <linux/slab.h>

#include "thermal_core.h"

/**
 * notify_user_space - Notifies user space about thermal events
 * @tz - thermal_zone_device
 * @trip - trip point index
 *
 * This function notifies the user space through UEvents.
 */
 #if defined ASUS_SAKE_PROJECT
 int get_virtual_therm(void);
 #endif
static int notify_user_space(struct thermal_zone_device *tz, int trip)
{
	char *thermal_prop[5];
	int i;
 #if defined ASUS_SAKE_PROJECT
	int temp;
 #endif

	mutex_lock(&tz->lock);
	thermal_prop[0] = kasprintf(GFP_KERNEL, "NAME=%s", tz->type);
#if defined ASUS_SAKE_PROJECT
	if(strcmp(tz->type, "virtual-therm") == 0)
	{	
		temp = get_virtual_therm();		
		thermal_prop[1] = kasprintf(GFP_KERNEL, "TEMP=%d", temp);
	}
	else
	{
		thermal_prop[1] = kasprintf(GFP_KERNEL, "TEMP=%d", tz->temperature);
	}
#else
	thermal_prop[1] = kasprintf(GFP_KERNEL, "TEMP=%d", tz->temperature);
#endif
	thermal_prop[2] = kasprintf(GFP_KERNEL, "TRIP=%d", trip);
	thermal_prop[3] = kasprintf(GFP_KERNEL, "EVENT=%d", tz->notify_event);
	thermal_prop[4] = NULL;
	kobject_uevent_env(&tz->device.kobj, KOBJ_CHANGE, thermal_prop);
	for (i = 0; i < 4; ++i)
		kfree(thermal_prop[i]);
	mutex_unlock(&tz->lock);
	return 0;
}

static struct thermal_governor thermal_gov_user_space = {
	.name		= "user_space",
	.throttle	= notify_user_space,
};
THERMAL_GOVERNOR_DECLARE(thermal_gov_user_space);
