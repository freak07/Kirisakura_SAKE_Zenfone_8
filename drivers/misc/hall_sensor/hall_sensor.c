#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>

/*****************************/
/* Hall Sensor Configuration */
/****************************/
#define DRIVER_NAME 		"hall_sensor"
#define IRQ_Name			"ASUS_Hall_Sensor-irq"
#define INT_NAME			"HallSensor_INT"
#define REPORT_WAKE_LOCK_TIMEOUT (1 * HZ)
#define DELAYED_WORK_TIME   500

/**************************/
/* Driver Data Structure */
/*************************/
static struct hall_sensor_str {
	int status;
	int enable;
	struct mutex hall_mutex;
	struct input_dev *hall_indev;
	struct wakeup_source *wake_src;
 	struct delayed_work hall_sensor_work;
	struct regulator		*vdd_supply;
}* hall_sensor_dev;

/*******************************/
/* Hall Sensor Global Variables */
/******************************/

static int ASUS_HALL_SENSOR_GPIO;
static int ASUS_HALL_SENSOR_IRQ;
static struct workqueue_struct 	*hall_sensor_wq;
static struct platform_device *g_pdev;
static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	pr_info("%s hall sensor irq +++\n", __func__);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	__pm_stay_awake(hall_sensor_dev->wake_src);

	return IRQ_HANDLED;
}

static void hall_sensor_report_function(struct work_struct *dat)
{
//	int counter, counter_trigger = 0, initial_status;
//	__pm_wakeup_event(hall_sensor_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT);
	char * envp[2];
	if(!hall_sensor_dev->enable){
		pr_info("%s hall sensor is disable!\n", __func__);
		__pm_relax(hall_sensor_dev->wake_src);
		return;
	}
//	initial_status =hall_sensor_dev->status;
	mutex_lock(&hall_sensor_dev->hall_mutex);
//	for (counter = 0;counter < 2;counter++) {
//		msleep(50);
//		pr_info("%s hall sensor counter:%d \n", __func__, counter);
//		if (gpio_get_value(ASUS_HALL_SENSOR_GPIO) == 1) {
//			hall_sensor_dev->status = 1;
//			counter_trigger++;
//			pr_info("%s gpio_get_value 1 \n", __func__);
//		} else {
//			hall_sensor_dev->status = 0;
//			pr_info("%s gpio_get_value 0 \n", __func__);
//		}
//	}
//	if( (counter_trigger > 0) && (counter_trigger < 2)){
//		pr_info("%s SW_LID do not report to framework.\n", __func__);
//		hall_sensor_dev->status = initial_status;
//		__pm_relax(hall_sensor_dev->wake_src);
//		return;
//	}
	hall_sensor_dev->status = gpio_get_value(ASUS_HALL_SENSOR_GPIO);
	pr_info("%s Hall sensor status = %d", __func__, hall_sensor_dev->status);
	mutex_unlock(&hall_sensor_dev->hall_mutex);
    input_report_switch(hall_sensor_dev->hall_indev, SW_LID, !hall_sensor_dev->status);
    input_sync(hall_sensor_dev->hall_indev);
	if(hall_sensor_dev->status==0)
	{
		envp[0] = "STATUS=OPEN";
		envp[1] = NULL;
		kobject_uevent_env(&g_pdev->dev.kobj, KOBJ_CHANGE, envp);
	}else{
		envp[0] = "STATUS=OTHER";
		envp[1] = NULL;
		kobject_uevent_env(&g_pdev->dev.kobj, KOBJ_CHANGE, envp);
	}
	__pm_relax(hall_sensor_dev->wake_src);
}

int report_hall_status(void)
{
	if(!hall_sensor_dev) {
		pr_err("%s Hall sensor does not exist!\n", __func__);
		return -1; //hall sensor is not
	}
	return hall_sensor_dev->status;
}
EXPORT_SYMBOL(report_hall_status);

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
    sscanf(buf, "%du", &request);
	mutex_lock(&hall_sensor_dev->hall_mutex);
    if (!request)
    	hall_sensor_dev->status = 0;
	else
		hall_sensor_dev->status = 1;
	mutex_unlock(&hall_sensor_dev->hall_mutex);

	pr_info("%s status rewite value = %d\n", __func__, !hall_sensor_dev->status);
	return count;
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
    	return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;

	if(!hall_sensor_dev) {
		pr_err("%s Hall sensor does not exist!\n", __func__);
		return 0;
	}
	sscanf(buf, "%du", &request);

	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		if(0 == request) {
			/* Turn off */
			pr_info("%s Turn off.\n", __func__);
			mutex_lock(&hall_sensor_dev->hall_mutex);
			hall_sensor_dev->enable=request;
			mutex_unlock(&hall_sensor_dev->hall_mutex);
		}else if(1 == request){
			/* Turn on */
			pr_info("%s Turn on. \n", __func__);
			mutex_lock(&hall_sensor_dev->hall_mutex);
			hall_sensor_dev->enable=request;
			mutex_unlock(&hall_sensor_dev->hall_mutex);
		}else{
			pr_err("%s Enable/Disable Error, can not recognize (%d)", __func__, request);
		}
	}
	return count;
}

static DEVICE_ATTR(status, 0664, show_action_status, store_action_status);
static DEVICE_ATTR(switch, 0664,show_hall_sensor_enable, store_hall_sensor_enable);

static struct attribute *hall_sensor_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_switch.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};


static int init_input_event(void)
{
	int ret = 0;

	hall_sensor_dev->hall_indev = input_allocate_device();
	if(!hall_sensor_dev->hall_indev){
		pr_err("%s Failed to allocate input event device\n", __func__);
		return -ENOMEM;
	}

	hall_sensor_dev->hall_indev->name = "hall_input";
	hall_sensor_dev->hall_indev->phys= "/dev/input/hall_indev";
	hall_sensor_dev->hall_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->hall_indev, EV_SW, SW_LID);

	ret = input_register_device(hall_sensor_dev->hall_indev);
	if (ret) {
		pr_err("%s Failed to register input event device\n", __func__);
		return -1;
	}

	pr_info("%s Input Event registration Success!\n", __func__);
	return 0;
}

static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "hall_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	pr_info("%s: pinctrl_select_state = %d\n", __func__, ret);
}

static int init_irq (void)
{
	int ret = 0;

	/* GPIO to IRQ */
	ASUS_HALL_SENSOR_IRQ = gpio_to_irq(ASUS_HALL_SENSOR_GPIO);

	if (ASUS_HALL_SENSOR_IRQ < 0) {
		pr_err("%s gpio_to_irq ERROR, irq=%d.\n", __func__, ASUS_HALL_SENSOR_IRQ);
	}else {
		pr_info("%s gpio_to_irq IRQ %d successed on GPIO:%d\n", __func__, ASUS_HALL_SENSOR_IRQ, ASUS_HALL_SENSOR_GPIO);
	}

	ret = request_threaded_irq(ASUS_HALL_SENSOR_IRQ, NULL, hall_sensor_interrupt_handler,
				IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				INT_NAME, hall_sensor_dev);

	if (ret < 0)
		pr_err("%s request_irq() ERROR %d.\n", __func__, ret);
	else {
		pr_debug("%s Enable irq !! \n", __func__);
		enable_irq_wake(ASUS_HALL_SENSOR_IRQ);
	}
	return 0;
}

static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s Hall sensor probe  +++\n", __func__);

	/* Initialization Data */
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		pr_err("%s Memory allocation fails for hall sensor\n", __func__);
		return -ENOMEM;
	}
	mutex_init(&hall_sensor_dev->hall_mutex);

	/* GPIO */
	ASUS_HALL_SENSOR_GPIO = of_get_named_gpio(pdev->dev.of_node, "qcom,hall-gpio", 0);
    pr_info("%s Hall sensor [GPIO] GPIO =%d(%d)\n", __func__, ASUS_HALL_SENSOR_GPIO, gpio_get_value(ASUS_HALL_SENSOR_GPIO));
	gpio_free(ASUS_HALL_SENSOR_GPIO);

	/* GPIO Request */
	set_pinctrl(&pdev->dev);
	ret = gpio_request(ASUS_HALL_SENSOR_GPIO, IRQ_Name);
	if (ret) {
		pr_err("%s Hall sensor [GPIO] Unable to request gpio %s(%d)\n", __func__, IRQ_Name, ASUS_HALL_SENSOR_GPIO);
		goto probe_err;
	}

	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_HALL_SENSOR_GPIO);
	if (ret < 0) {
		pr_err("%s Hall sensor [GPIO] Unable to set the direction of gpio %d\n", __func__, ASUS_HALL_SENSOR_GPIO);
		goto probe_err;
	}

	hall_sensor_dev->vdd_supply = regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(hall_sensor_dev->vdd_supply)) {
        pr_err("%s Hall sensor ret vdd regulator failed,ret=%d", __func__, ret);
        goto probe_err;
    }
	ret = regulator_enable(hall_sensor_dev->vdd_supply);
	if (ret) {
		pr_err("%s Hall sensor enable vcc_i2c regulator failed,ret=%d", __func__, ret);
    }
	msleep(10);

	ret = init_input_event();
	if (ret < 0)
		goto probe_err;

	ret = sysfs_create_group(&pdev->dev.kobj, &hall_sensor_group);
	if (ret) {
		pr_err("%s Hall sensor sysfs_create_group ERROR.\n", __func__);
		goto probe_err;
	}

	/* Work Queue init */
    hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
    INIT_DEFERRABLE_WORK(&hall_sensor_dev->hall_sensor_work, hall_sensor_report_function);
    ret = init_irq();
    if (ret < 0)
         goto probe_err;
    queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	hall_sensor_dev->enable = 1;
	g_pdev=pdev;//add for uevent
    pr_info("%s Hall sensor probe  ---\n", __func__);
	return 0;

probe_err:
	pr_err("%s Hall sensor probe error\n", __func__);
	return ret;

}

static const struct platform_device_id hall_id_table[] = {
        {DRIVER_NAME, 1},
};

static struct of_device_id hallsensor_match_table[] = {
	{ .compatible = "qcom,hall",},
	{},
};

static struct platform_driver hall_sensor_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = hallsensor_match_table,
	},
	.probe          = hall_sensor_probe,
	.id_table	= hall_id_table,
};

static int __init hall_sensor_init(void)
{
	int err = 0;

	pr_info("%s Hall sensor driver init +++\n", __func__);
	err = platform_driver_register(&hall_sensor_driver);
	if (err != 0) {
		pr_err("%s Hall sensor platform_driver_register fail, error=%d\n", __func__, err);
		return err;
    }
	pr_info("%s Hall sensor driver init ---\n", __func__);
	return err;
}

static void __exit hall_sensor_exit(void)
{
	int ret = 0;

	pr_info("%s Hall sensor driver exit +++\n", __func__);
	ret = regulator_disable(hall_sensor_dev->vdd_supply);
    if (ret)
		pr_err("%s disable ibb regulator failed,ret=%d\n", __func__, ret);

    mutex_destroy(&hall_sensor_dev->hall_mutex);
	free_irq(ASUS_HALL_SENSOR_IRQ, hall_sensor_dev);
	wakeup_source_remove(hall_sensor_dev->wake_src);
	wakeup_source_destroy(hall_sensor_dev->wake_src);
	platform_driver_unregister(&hall_sensor_driver);
	gpio_free(ASUS_HALL_SENSOR_GPIO);
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	pr_info("%s Hall sensor driver exit ---\n", __func__);
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_DESCRIPTION("Hall Sensor");
MODULE_LICENSE("GPL v2");

