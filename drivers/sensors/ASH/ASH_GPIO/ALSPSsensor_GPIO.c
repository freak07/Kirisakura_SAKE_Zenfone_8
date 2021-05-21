/* 
 * Copyright (C) 2015 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /*************************************/
/* ALSPS Sensor GPIO Module */
/************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input/ASH.h>

#define ALSPS_INTEL_NAME 	"ALSPS_INT#"
#define ALSPS_QCOM_NAME 	"qcom,alsps-gpio"
#define ALSPS_IRQ_NAME		"ALSPS_SENSOR_IRQ"
#define ALSPS_INT_NAME		"ALSPS_SENSOR_INT"
 
static int ALSPS_SENSOR_GPIO;
static ALSPSsensor_GPIO * mALSPSsensor_GPIO;

/******************************/
/* Debug and Log System */
/*****************************/
#define MODULE_NAME			"ASH_GPIO"
#define SENSOR_TYPE_NAME		"ALSPS"
static struct i2c_client * g_i2c_client = NULL;

#undef dbg
#ifdef ASH_GPIO_DEBUG
	#define dbg(fmt, args...) printk(KERN_DEBUG "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif
#define log(fmt, args...) printk(KERN_INFO "[%s][%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,__func__,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s][%s]"fmt,MODULE_NAME,SENSOR_TYPE_NAME,##args)

static irqreturn_t ALSPSsensor_irq_handler(int irq, void *dev_id);

#ifdef GPIO_INTEL
#include <asm/intel-mid.h>
#endif

#ifdef GPIO_QCOM
#ifndef CONFIG_TMD2755_FLAG
#include <linux/of_gpio.h>
#ifdef ALSP_GPIO_NO_PULL
#define GPIO_LOOKUP_STATE	"alsps_gpio_no_pull"
#else
#define GPIO_LOOKUP_STATE	"alsps_gpio_high"
#endif

static void set_pinctrl(struct i2c_client *client)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(&client->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, GPIO_LOOKUP_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0)
		err("%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
}
#endif
#endif
static int init_irq (void)
{
	int ret = 0;
	int irq = 0;
#ifdef CONFIG_TMD2755_FLAG
	unsigned long default_irq_trigger = 0;
#endif

	/* GPIO to IRQ */
	irq = gpio_to_irq(ALSPS_SENSOR_GPIO);
	if (irq < 0) {
		err("%s: gpio_to_irq ERROR(%d). \n", __FUNCTION__, irq);
		return irq;
	}else {
		log("gpio_to_irq IRQ %d successed on GPIO:%d\n", irq, ALSPS_SENSOR_GPIO);
	}

	/*Request IRQ*/	
	#ifdef GPIO_INTEL
	ret = request_irq(irq,ALSPSsensor_irq_handler, IRQF_TRIGGER_LOW, ALSPS_INT_NAME, NULL);
	#endif
	#ifdef GPIO_QCOM
#ifdef CONFIG_TMD2755_FLAG
	default_irq_trigger = irqd_get_trigger_type(irq_get_irq_data(g_i2c_client->irq));
	ret = request_threaded_irq(irq, NULL, ALSPSsensor_irq_handler,
				default_irq_trigger | IRQF_SHARED | IRQF_ONESHOT, ALSPS_INT_NAME, NULL);
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
	}
	ret = request_threaded_irq(irq, NULL, ALSPSsensor_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, ALSPS_INT_NAME, NULL);
	if (ret < 0) {
		err("%s: 1request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
	}
	
	ret = devm_request_threaded_irq(&g_i2c_client->dev, g_i2c_client->irq, NULL, &ALSPSsensor_irq_handler, default_irq_trigger | IRQF_SHARED | IRQF_ONESHOT,
					dev_name(&g_i2c_client->dev), NULL);
	if (ret) {
		err("%s: 2request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
	}
#else
	ret = request_threaded_irq(irq, NULL, ALSPSsensor_irq_handler,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, ALSPS_INT_NAME, NULL);
#endif
	#endif
	if (ret < 0) {
		err("%s: request_irq/request_threaded_irq ERROR(%d).\n", __FUNCTION__, ret);
		return ret;
	}else {		
		dbg("Disable irq !! \n");
		disable_irq(irq);
	}

	return irq;
}

irqreturn_t ALSPSsensor_irq_handler(int irq, void *dev_id)
{
	dbg("ALSPSensor isr");
	mALSPSsensor_GPIO->ALSPSsensor_isr();
	return IRQ_HANDLED;
}

int ALSPSsensor_gpio_register(struct i2c_client *client, ALSPSsensor_GPIO *gpio_ist)
{
	int ret = 0;
	int irq = 0;
	g_i2c_client = client;

	mALSPSsensor_GPIO = gpio_ist;
	
	/* GPIO */
	#ifdef GPIO_INTEL
	log("Intel GPIO \n");
	ALSPS_SENSOR_GPIO = get_gpio_by_name(ALSPS_INTEL_NAME);
	#endif
	
	#ifdef GPIO_QCOM
#ifdef CONFIG_TMD2755_FLAG
#else
	log("Qcom GPIO \n");

#ifdef ALSP_GPIO_NO_PULL
	log("ALSP_GPIO_NO_PULL applied");
#else
	log("ALSP_GPIO_HIGH applied");
#endif

	set_pinctrl(client);
//	ALSPS_SENSOR_GPIO = of_get_named_gpio_flags(client->dev.of_node, ALSPS_QCOM_NAME, 0, NULL);
	ALSPS_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, ALSPS_QCOM_NAME, 0);
	#endif
		
	log("[GPIO] GPIO =%d(%d)\n", ALSPS_SENSOR_GPIO, gpio_get_value(ALSPS_SENSOR_GPIO));	
	/* GPIO Request */
	ret = gpio_request(ALSPS_SENSOR_GPIO, ALSPS_IRQ_NAME);
	if (ret) {
		err("%s: gpio_request ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}

	/* GPIO Direction */
	ret = gpio_direction_input(ALSPS_SENSOR_GPIO);

	if (ret < 0) {
		err("%s: gpio_direction_input ERROR(%d). \n", __FUNCTION__, ret);
		return ret;
	}
	/*IRQ*/
	irq = init_irq();
#endif
	return irq;

}
EXPORT_SYMBOL(ALSPSsensor_gpio_register);


int ALSPSsensor_gpio_unregister(int irq)
{
	free_irq(irq, NULL);
	gpio_free(ALSPS_SENSOR_GPIO);
	return 0;
}
EXPORT_SYMBOL(ALSPSsensor_gpio_unregister);
