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

#include <linux/i2c.h>
#include <linux/fs.h>

#include <linux/kernel.h>

#include <linux/regulator/consumer.h>

#include <linux/proc_fs.h>


#include <linux/of_gpio.h>

/*************************/
/* Debug Switch System */
/************************/
#undef dbg
#ifdef LATCH_DEBUG
	#define dbg(fmt, args...) printk("[%s] "fmt,DRIVER_NAME,##args)
#else
	#define dbg(fmt, args...)
#endif

#define log(fmt, args...) printk(KERN_INFO"[%s] "fmt,DRIVER_NAME,##args)
#define err(fmt, args...) printk(KERN_ERR "[%s] "fmt,DRIVER_NAME,##args)

/*****************************/
/* Latch Sensor Configuration */
/****************************/
#define DRIVER_NAME 		"latch_sensor"
#define GPIO_NAME 		"latch_det#"
#
#define INT_NAME			"LatchSensor_INT"
#define KERNEL_OBJECT	"latch_sensor_kobject"
#define LATCHSEN_ID        924

#define REPORT_WAKE_LOCK_TIMEOUT (1 * HZ)
#define DELAYED_WORK_TIME   500  //500 ms



#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
#define DEBUG_LOG 1
#else
#define DEBUG_LOG 0
#endif

/**************************/
/* Driver Data Structure */
/*************************/
static struct latch_sensor_str {
	int status;
	int enable;
	spinlock_t mLatchSensorLock;
	struct wakeup_source *wake_src;
	struct delayed_work latch_sensor_work;
    struct i2c_client *client;
	struct input_dev *latch_indev;
	struct regulator		*vdd_supply;
	struct mutex latch_mutex;
	int hardcode;
    int debounce;
    int sleep;
}* latch_sensor_dev;

static int major;
static int 							ASUS_LATCH_SENSOR_GPIO;
static int ASUS_LATCH_SENSOR_IRQ;
static int e = 0;
static struct workqueue_struct 	*latch_sensor_wq;
static struct i2c_client* report_client;
struct device *dev = NULL;
static int g_state=-1; //0-none;1-open;2-close
static int g_threshold1X=-550;
static int g_threshold2X=550;
static int g_threshold1Y=400;
static int is_suspend=0;

static int latch_sensor_suspend(struct device *dev)
{
	log("latch_sensor SUSPEND +++\n");
	is_suspend = 1;
	log("latch_sensor SUSPEND ---\n");
	return 0;
}

static int latch_sensor_resume(struct device *dev)
{
	log("latch_sensor RESUME +++\n");
	is_suspend = 0;
	log("latch_sensor RESUME ---\n");
	return 0;
}

static int i2c_write_bytes(struct i2c_client *client, char *write_buf, int writelen)
{
	struct i2c_msg msg;
	int ret=-1;

	msg.flags = !I2C_M_RD;		//write
	msg.addr = client->addr;
	msg.len = writelen;
	msg.buf = write_buf;

	ret = i2c_transfer(client->adapter,&msg, 1);
	if(ret <= 0)
	{
		err("[latch_sensor] %s error %d,will retry\n",__func__,ret);
		ret = i2c_transfer(client->adapter,&msg, 1);
		if(ret <= 0)
		{
			err("[latch_sensor] %s error %d\n",__func__,ret);
		}
	}
	return ret;
}

static int i2c_read_bytes(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};
	struct i2c_msg msgs;
	buf[0] = addr & 0xFF;

	err = i2c_write_bytes(client, buf, 1);
	if (err !=1)
		printk("[latch_sensor] i2c_write_bytes:err %d\n", err);

	msleep(1);//wait for ic

	msgs.flags = I2C_M_RD;		//read
	msgs.addr = client->addr;
	msgs.len = 8;
	msgs.buf = data;

	err = i2c_transfer(client->adapter,&msgs, 1);
	if(err <= 0)
	{
		err("[latch_sensor] %s error %d,will retry\n",__func__,err);
		err = i2c_transfer(client->adapter,&msgs, 1);
		if(err <= 0)
		{
			err("[latch_sensor] %s error %d\n",__func__,err);
		}
	}
	return err;
}

static irqreturn_t latch_sensor_reenable_irq(int irq, void *dev_id)
{

	unsigned char data[8]={0};
	int err = 0;
	int SWX1=-1;
	int SWX2=-1;
	int SWY1=-1;

	if(is_suspend == 1){
		log("%s,is_suspend=1\n",__func__);
		__pm_wakeup_event(latch_sensor_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT+(4 * HZ));
		queue_delayed_work(latch_sensor_wq, &latch_sensor_dev->latch_sensor_work, msecs_to_jiffies(DELAYED_WORK_TIME*2));
        return IRQ_HANDLED;
    }

	dbg("[ISR] %s latch_sensor_interrupt = %d\n",__func__,ASUS_LATCH_SENSOR_IRQ);
	__pm_wakeup_event(latch_sensor_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT+(1 * HZ));
	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(report_client, 0x17, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] %s:err %d\n",__func__,err);
		return IRQ_HANDLED;
	}

	SWX1 = (data[1]&0x2)>>1;  //ST-D1
	SWX2 = (data[1]&0x4)>>2;  //ST-D2
	SWY1 = (data[1]&0x8)>>3;  //ST-D3
	err = cancel_delayed_work(&latch_sensor_dev->latch_sensor_work);
	if(err == 1 ){
		log("cancel pending work  SWX1=%d  SWX2=%d SWY1=%d\n",SWX1,SWX2,SWY1);
	}else{
		log("no pending work  SWX1=%d  SWX2=%d SWY1=%d\n",SWX1,SWX2,SWY1);
	}

	queue_delayed_work(latch_sensor_wq, &latch_sensor_dev->latch_sensor_work, msecs_to_jiffies(DELAYED_WORK_TIME));

	return IRQ_HANDLED;
}

static void debounce_latch_sensor_report_function(struct work_struct *dat)
{
//	struct i2c_client *client = to_i2c_client(&client->dev);
	unsigned char data[8]={0};
	int err = 0;
	char * envp[2];
	int X_value=0;
	int Y_value=0;
	int SWX1=-1;
	int SWX2=-1;
	int SWY1=-1;

	__pm_wakeup_event(latch_sensor_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT);
	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(report_client, 0x17, data);
	if (err != 1)
		err("[latch_sensor] show mode:err %d\n", err);

	mutex_unlock(&latch_sensor_dev->latch_mutex);

	SWX1 = (data[1]&0x2)>>1;  //ST-D1
	SWX2 = (data[1]&0x4)>>2;  //ST-D1
	SWY1 = (data[1]&0x8)>>3;  //ST-D3
	if(data[6] >= 128)
		X_value = ~((data[6]^255)* 256 + (data[7]^255));
	else
	        X_value = data[6]*256 + data[7];

	if(data[4] >= 128)
		Y_value = ~((data[4]^255)* 256 + (data[5]^255));
	else
		Y_value = data[4]*256 + data[5];

	log("interrupt  X_value=%d SWX1=%d SWX2=%d Y_value=%d SWY1=%d\n",X_value,SWX1,SWX2,Y_value,SWY1);

	if( ((SWX1==0) && (SWY1==0)) || ((SWX2==1) && (SWY1==0)))
	{
		envp[0] = "STATUS=CLOSE";
		envp[1] = NULL;
		kobject_uevent_env(&report_client->dev.kobj, KOBJ_CHANGE, envp);
		g_state=2;
		if((SWX1==0) && (SWY1==0))
			log("old latch g_threshold1X=%d\n",g_threshold1X);
		else
			log("new latch g_threshold2X=%d\n",g_threshold2X);
	}else if( ((SWX1==0) && (SWY1==1)) || ((SWX2==0) && (SWY1==0)))
	{
		envp[0] = "STATUS=OPEN";
		envp[1] = NULL;
		kobject_uevent_env(&report_client->dev.kobj, KOBJ_CHANGE, envp);
		g_state=1;
		if((SWX1==0) && (SWY1==1))
			log("old latch g_threshold1X=%d\n",g_threshold1X);
		else
			log("new latch g_threshold2X=%d\n",g_threshold2X);
	}else if((SWX1==1) || (SWY1==1))
	{
		envp[0] = "STATUS=NONE";
		envp[1] = NULL;
		kobject_uevent_env(&report_client->dev.kobj, KOBJ_CHANGE, envp);
		g_state=0;
	}

}

static ssize_t show_action_mode(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8]={0};
	int err = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] show mode:err %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}
	log("[latch_sensor] %s  mode=%d\n",__func__,data[0]);
	return snprintf(buf,  PAGE_SIZE,"mode %x\n", data[0]);
}

static int latch_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr & 0xFF;
	buf[1] = value;

	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		err("[latch_sensor] i2c_write_bytes:err %d\n", err);
	return err;
}

static int latch_write_threshold(struct i2c_client *client, short addr, int bop1x,int brp1x)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr & 0xFF; //0x22 set 1x threshole;  0x24,set 1y threshold
	buf[1] = (bop1x & 0xFF00)>>8; //bop1x high byte
	buf[2] = bop1x & 0xFF; //bop1x low byte
	buf[3] = (brp1x & 0xFF00)>>8;  //brp1x high byte
	buf[4] = brp1x & 0xFF;  //brp1x low byte

	err = i2c_write_bytes(client, buf, 5);
	if (err !=1)
		err("[latch_sensor] i2c_write_bytes:err %d\n", err);

	return err;
}


static ssize_t store_1X_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int request=0;
	struct i2c_client *client = to_i2c_client(dev);

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_sensor_dev->latch_mutex);
	ret=latch_write_threshold(client,0x22,request+32,request-32);
	mutex_unlock(&latch_sensor_dev->latch_mutex);

	if(ret<0){
		log("[latch_sensor]%s: latch_write_threshold write X1threshold %d fail\n",__func__,request);
		return count;
	}

	g_threshold1X=request;
	log("[latch_sensor]%s: X1threshold=%d\n",__func__,request);
	return count;
}

static ssize_t store_2X_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int request=0;
	struct i2c_client *client = to_i2c_client(dev);

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_sensor_dev->latch_mutex);
	ret=latch_write_threshold(client,0x23,request+32,request-32);
	mutex_unlock(&latch_sensor_dev->latch_mutex);

	if(ret<0){
		log("[latch_sensor]%s: latch_write_threshold write X2threshold %d fail\n",__func__,request);
		return count;
	}

	g_threshold2X=request;
	log("[latch_sensor]%s: X2threshold=%d\n",__func__,request);
	return count;
}

static ssize_t show_1X_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//unsigned char data[8]={0};
	//int err = 0;
	log("[latch_sensor]%s: g_threshold1X=%d\n",__func__,g_threshold1X);
	return snprintf(buf, PAGE_SIZE,"g_threshold1X=%d\n", g_threshold1X);
}

static ssize_t show_2X_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//unsigned char data[8]={0};
	//int err = 0;
	log("[latch_sensor]%s: g_threshold2X=%d\n",__func__,g_threshold2X);
	return snprintf(buf, PAGE_SIZE,"g_threshold2X=%d\n", g_threshold2X);
}

static ssize_t store_1Y_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int request=0;
	struct i2c_client *client = to_i2c_client(dev);

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_sensor_dev->latch_mutex);
	ret=latch_write_threshold(client,0x24,request+32,request-32);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if(ret<0){
		log("[latch_sensor]%s: latch_write_threshold write threshold %d fail\n",__func__,request);
		return count;
	}
	g_threshold1Y=request;
	log("[latch_sensor]%s: threshold=%d\n",__func__,request);
	return count;
}


static ssize_t show_1Y_threshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//unsigned char data[8]={0};
	//int err = 0;
	log("[latch_sensor]%s: g_threshold1Y=%d\n",__func__,g_threshold1Y);
	return snprintf(buf, PAGE_SIZE,"g_threshold1Y=%d\n", g_threshold1Y);
}

static ssize_t store_trigger_update(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	log("[latch_sensor]%s: entry \n",__func__);
	queue_delayed_work(latch_sensor_wq, &latch_sensor_dev->latch_sensor_work, 0);
	return count;
}

static ssize_t store_action_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int request=0 ;

	struct i2c_client *client = to_i2c_client(dev);
   	sscanf(buf, "%x", &request);

	mutex_lock(&latch_sensor_dev->latch_mutex);
	ret = latch_write_bytes(client, 0x21, request);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if(ret < 0)
	{
		err("[latch_sensor] store mode error\n");
		return count;
	}
	log("[latch_sensor]write %d to mode\n",request);
	return count;
}

static ssize_t show_latch_sensor_interrupt_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8]={0};
	int err = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x20, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] interrupt show:err %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x20 error");
	}
	log("[latch_sensor] %s  data[0]=%d data[1]=%d\n",__func__,data[0],data[1]);
	return snprintf(buf,  PAGE_SIZE,"int %d-%d\n", data[0],data[1]);
}

static int latch_write_interrupt(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr & 0xFF;
	buf[1] = 6;
	buf[2] = value;

	err = i2c_write_bytes(client, buf, 3);
	if (err !=1)
		err("[latch_sensor] i2c_write_bytes:err %d\n", err);

	return err;
}

static ssize_t store_latch_sensor_interrupt_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int  request=0;

	struct i2c_client *client = to_i2c_client(dev);
   	sscanf(buf, "%x", &request);

	mutex_lock(&latch_sensor_dev->latch_mutex);
	ret = latch_write_interrupt(client, 0x20, request);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if(ret < 0)
	{
		err("[latch_sensor]write interrupt send error\n");
		return count;
	}
	log("[latch_sensor]write %d to interrupr\n",request);
	return count;
}

static ssize_t show_st_xyz(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 2: sleeptime = 4000;
				  break;
		case 4: sleeptime = 2000;
				  break;
		case 6: sleeptime = 1000;
				  break;
		case 8: sleeptime = 100;
				  break;
		case 10: sleeptime = 50;
				  break;
		case 12: sleeptime = 20;
				  break;
		case 14: sleeptime = 10;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}


	if(data[1] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_sensor_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_sensor_dev->latch_mutex);
		if (err != 1){
			err("[latch_sensor] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[6] * 256 + data[7];
	p = data[4] * 256 + data[5];
	o = data[3] * 256 + data[2];
	log("[latch_sensor] %s  status:%d-%d z:%ld y:%ld x:%ld\n",__func__,data[0],data[1],o,p,q);
	return snprintf(buf,  PAGE_SIZE,"status:%d-%d z:%ld y:%ld x:%ld\n", data[0],data[1],o,p,q);
}

static ssize_t show_x(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 2: sleeptime = 4000;
				  break;
		case 4: sleeptime = 2000;
				  break;
		case 6: sleeptime = 1000;
				  break;
		case 8: sleeptime = 100;
				  break;
		case 10: sleeptime = 50;
				  break;
		case 12: sleeptime = 20;
				  break;
		case 14: sleeptime = 10;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[1] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_sensor_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_sensor_dev->latch_mutex);
		if (err != 1){
			err("[latch_sensor] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[6] * 256 + data[7];
	p = data[4] * 256 + data[5];
	o = data[2] * 256 + data[3];

	if(data[6] >= 128)
		q = ~((data[6]^255)* 256 + (data[7]^255));

	if(data[4] >= 128)
		p = ~((data[4]^255)* 256 + (data[5]^255));

	if(data[2] >= 128)
		o = ~((data[2]^255)* 256 + (data[3]^255));
	log("[latch_sensor] %s  x=%d\n",__func__,q);
	return snprintf(buf,  PAGE_SIZE,"%d\n",q);
}

static ssize_t show_y(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}


	switch(modedata[0])
	{
		case 2: sleeptime = 4000;
				  break;
		case 4: sleeptime = 2000;
				  break;
		case 6: sleeptime = 1000;
				  break;
		case 8: sleeptime = 100;
				  break;
		case 10: sleeptime = 50;
				  break;
		case 12: sleeptime = 20;
				  break;
		case 14: sleeptime = 10;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[1] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_sensor_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_sensor_dev->latch_mutex);
		if (err != 1){
			err("[latch_sensor] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[6] * 256 + data[7];
	p = data[4] * 256 + data[5];
	o = data[2] * 256 + data[3];

	if(data[6] >= 128)
		q = ~((data[6]^255)* 256 + (data[7]^255));

	if(data[4] >= 128)
		p = ~((data[4]^255)* 256 + (data[5]^255));

	if(data[2] >= 128)
		o = ~((data[2]^255)* 256 + (data[3]^255));
	log("[latch_sensor] %s  y=%d\n",__func__,p);
	return snprintf(buf,  PAGE_SIZE,"%d\n",p);
}

static ssize_t show_z(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 2: sleeptime = 4000;
				  break;
		case 4: sleeptime = 2000;
				  break;
		case 6: sleeptime = 1000;
				  break;
		case 8: sleeptime = 100;
				  break;
		case 10: sleeptime = 50;
				  break;
		case 12: sleeptime = 20;
				  break;
		case 14: sleeptime = 10;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[1] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_sensor_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_sensor_dev->latch_mutex);
		if (err != 1){
			err("[latch_sensor] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[6] * 256 + data[7];
	p = data[4] * 256 + data[5];
	o = data[2] * 256 + data[3];

	if(data[6] >= 128)
		q = ~((data[6]^255)* 256 + (data[7]^255));

	if(data[4] >= 128)
		p = ~((data[4]^255)* 256 + (data[5]^255));

	if(data[2] >= 128)
		o = ~((data[2]^255)* 256 + (data[3]^255));
	log("[latch_sensor] %s  z=%d\n",__func__,o);
	return snprintf(buf,  PAGE_SIZE,"%d\n",o);
}

static ssize_t show_latch_sensor_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data[8] = {0};
	int err = 0;

	mutex_lock(&latch_sensor_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x00, data);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	if (err != 1){
		err("[latch_sensor] read status error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x00 error");
	}
	log("[latch_sensor] %s  status=%d\n",__func__,data[0]);
	return snprintf(buf, PAGE_SIZE,"%x\n", data[0]);
}

static ssize_t store_latch_sensor_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return -EPERM;
}

static ssize_t show_latch_sensor_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	char * state;
	if(g_state==1){
		state="OPEN";
	}else if(g_state==2){
		state="CLOSE";
	}else{
		state="NONE";
	}
	log("[latch_sensor] %s  g_state=%d\n",__func__,g_state);
	return snprintf(buf, PAGE_SIZE,"%s\n", state);
}

static DEVICE_ATTR(mode, 0664, show_action_mode, store_action_mode);
static DEVICE_ATTR(interrupt, 0664,show_latch_sensor_interrupt_enable, store_latch_sensor_interrupt_enable);
static DEVICE_ATTR(stxyz, 0664,show_st_xyz, NULL);
static DEVICE_ATTR(X, 0664,show_x, NULL);
static DEVICE_ATTR(Y, 0664,show_y, NULL);
static DEVICE_ATTR(Z, 0664,show_z, NULL);
static DEVICE_ATTR(X1_threshold, 0664,show_1X_threshold, store_1X_threshold);
static DEVICE_ATTR(X2_threshold, 0664,show_2X_threshold, store_2X_threshold);
static DEVICE_ATTR(Y1_threshold, 0664,show_1Y_threshold, store_1Y_threshold);
static DEVICE_ATTR(trigger_update, 0664,NULL, store_trigger_update);
static DEVICE_ATTR(status, 0664, show_latch_sensor_status, store_latch_sensor_status);
static DEVICE_ATTR(state, 0664, show_latch_sensor_state, NULL);

static struct attribute *latch_sensor_attrs[] = {
	&dev_attr_mode.attr,
	&dev_attr_interrupt.attr,
	&dev_attr_stxyz.attr,
	&dev_attr_X.attr,
	&dev_attr_Y.attr,
	&dev_attr_Z.attr,
	&dev_attr_X1_threshold.attr,
	&dev_attr_X2_threshold.attr,
	&dev_attr_Y1_threshold.attr,
	&dev_attr_trigger_update.attr,
	&dev_attr_status.attr,
	&dev_attr_state.attr,
	NULL
};

static struct attribute_group latch_sensor_group = {
	.name = "latch_sensor",
	.attrs = latch_sensor_attrs
};

static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "latch_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	log("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
}

static int init_data(void)
{
	int ret = 0;

	// Memory allocation for data structure
	latch_sensor_dev = kzalloc(sizeof (struct latch_sensor_str), GFP_KERNEL);
	if (!latch_sensor_dev) {
		err("Memory allocation fails for latch sensor\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	latch_sensor_dev->wake_src=wakeup_source_create("LatchSensor_wake_lock");
	wakeup_source_add(latch_sensor_dev->wake_src);
	//wakeup_source_init(&latch_sensor_dev->wake_src, "LatchSensor_wake_lock");
	return 0;
init_data_err:
	err("Init Data ERROR\n");
	return ret;
}

static int init_irq (void)
{
	int ret = 0;

	/* GPIO to IRQ */
	ASUS_LATCH_SENSOR_IRQ = gpio_to_irq(ASUS_LATCH_SENSOR_GPIO);

	if (ASUS_LATCH_SENSOR_IRQ < 0) {
		err("[IRQ] gpio_to_irq ERROR, irq=%d.\n", ASUS_LATCH_SENSOR_IRQ);
	}else {
		log("[IRQ] gpio_to_irq IRQ %d successed on GPIO:%d\n", ASUS_LATCH_SENSOR_IRQ, ASUS_LATCH_SENSOR_GPIO);
	}

	ret = request_threaded_irq(ASUS_LATCH_SENSOR_IRQ, NULL, latch_sensor_reenable_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				INT_NAME, latch_sensor_dev);

	if (ret < 0)
		err("[IRQ] request_irq() ERROR %d.\n", ret);
	else {
		dbg("[IRQ] Enable irq !! \n");
		enable_irq_wake(ASUS_LATCH_SENSOR_IRQ);
	}

	return 0;
}

static int latch_sensor_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	int ret = 0;

    void* dummy = NULL;
    struct proc_dir_entry* proc_latch_data = NULL;
    struct proc_dir_entry* proc_debounce_data = NULL;
    struct proc_dir_entry* proc_sleep_data = NULL;

#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
    proc_latch_data = proc_create_data("latch_sensor_cmd2", 0660, NULL, &proc_latch_send, dummy);
    proc_debounce_data = proc_create_data("latch_sensor_debounce2", 0660, NULL, &proc_debounce_send, dummy);
    proc_sleep_data = proc_create_data("latch_sensor_sleep2", 0660, NULL, &proc_sleep_send, dummy);
#else
    proc_latch_data=NULL;
    proc_debounce_data=NULL;
    proc_sleep_data=NULL;
    dummy = NULL;
#endif

log("Probe +++\n");

	/* Initialization Data */
	ret = init_data();
	if (ret < 0)
		goto probe_err;

	mutex_init(&latch_sensor_dev->latch_mutex);

	ASUS_LATCH_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, "latch,reset_gpio", 0);
	if (ASUS_LATCH_SENSOR_GPIO < 0) {
	    log("[GPIO] GPIO =%d(%d)\n", ASUS_LATCH_SENSOR_GPIO, gpio_get_value(ASUS_LATCH_SENSOR_GPIO));
		return -1;
	} else {
		log("%s: reset gpio provided ok\n", __func__);
	}

	if (gpio_is_valid(ASUS_LATCH_SENSOR_GPIO)) {
		ret = devm_gpio_request_one(&client->dev, ASUS_LATCH_SENSOR_GPIO,
					    GPIOF_OUT_INIT_HIGH, "latch_sensor_rst");
		if (ret) {
			log("%s: rst request failed\n", __func__);
			return -1;
		}
	}

	gpio_set_value_cansleep(ASUS_LATCH_SENSOR_GPIO, 1);
	usleep_range(1000, 2000);
	/* GPIO */
	ASUS_LATCH_SENSOR_GPIO = of_get_named_gpio(client->dev.of_node, "latch,int-gpio", 0);
    log("[GPIO] GPIO =%d(%d)\n", ASUS_LATCH_SENSOR_GPIO, gpio_get_value(ASUS_LATCH_SENSOR_GPIO));
	gpio_free(ASUS_LATCH_SENSOR_GPIO);
	/* GPIO Request */
	report_client = client;
	set_pinctrl(&client->dev);

	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_LATCH_SENSOR_GPIO);
	if (ret < 0) {
		err("[GPIO] Unable to set the direction of gpio %d\n", ASUS_LATCH_SENSOR_GPIO);
		goto probe_err;
	}

	latch_sensor_dev->vdd_supply = regulator_get(&client->dev, "vdd");
	if (IS_ERR(latch_sensor_dev->vdd_supply)) {
        err("ret vdd regulator failed,ret=%d", ret);
        goto probe_err;
    }
	ret = regulator_enable(latch_sensor_dev->vdd_supply);
	if (ret) {
		err("enable vcc_i2c regulator failed,ret=%d", ret);
    }
	msleep(10);
	//check i2c function
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err("[latch_sensor] I2C function test error\n");
		goto probe_err;
	} else
		log("[latch_sensor] I2C function test pass\n");

	/* sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &latch_sensor_group);
	if (ret) {
		err("Latch sensor sysfs_create_group ERROR.\n");
		goto probe_err;
	}

	mutex_lock(&latch_sensor_dev->latch_mutex);
	latch_write_bytes(client,0x30,0);
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	msleep(5);
	mutex_lock(&latch_sensor_dev->latch_mutex);
	latch_write_bytes(client,0x21,40); //value is 0x28 ,Wide measurement range; low noise drive; mode4
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	msleep(5);

	mutex_lock(&latch_sensor_dev->latch_mutex);
	latch_write_threshold(client,0x22,g_threshold1X+32,g_threshold1X-32); //0x22, write X1 threshold
	latch_write_threshold(client,0x23,g_threshold2X+32,g_threshold2X-32); //0x22, write X2 threshold
	latch_write_threshold(client,0x24,g_threshold1Y+32,g_threshold1Y-32); //0x24, write Y1 threshold
	ret = latch_write_interrupt(client, 0x20, 0xe); //set SWX1EN(D1),SWX2EN(D2),SWY1EN(D3)
	if(ret < 0)
	{
		err("[latch_sensor]write interrupt [0x6] [0xa]  send error\n");
		mutex_unlock(&latch_sensor_dev->latch_mutex);
		goto probe_err;
	}
	mutex_unlock(&latch_sensor_dev->latch_mutex);
	msleep(5);

	/* Work Queue init */
    latch_sensor_wq = create_singlethread_workqueue("latch_sensor_wq");
    INIT_DEFERRABLE_WORK(&latch_sensor_dev->latch_sensor_work, debounce_latch_sensor_report_function);
    ret = init_irq();
    if (ret < 0)
         goto probe_err;
    queue_delayed_work(latch_sensor_wq, &latch_sensor_dev->latch_sensor_work, 0); //add thhis work to get the reboot status

    log("Probe ---\n");
	return 0;

probe_err:
	err("Probe ERROR\n");
	return ret;
}

/*****************************************************************************
* I2C Driver
*****************************************************************************/
static const struct i2c_device_id latch_id_table[] = {
        {DRIVER_NAME, 1},
        {},
};
MODULE_DEVICE_TABLE(i2c, latch_id_table);

static struct of_device_id latchsensor_match_table[] = {
	{ .compatible = "qcom,latch",},
	{},
};

static struct dev_pm_ops latch_pm = {
    .suspend = latch_sensor_suspend,
    .resume  = latch_sensor_resume,
};

static struct i2c_driver latch_sensor_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = latchsensor_match_table,
		.pm  = &latch_pm,
	},
	.probe          = latch_sensor_probe,
	.id_table	= latch_id_table,
};

//----------------for pm_ops callback----------------

static int __init latch_sensor_init(void)
{
	int err = 0;
	log("Driver latch_sensor +++\n");
	err = i2c_add_driver(&latch_sensor_driver);
	if (err != 0) {
		err("[Latch Sensor] i2c_driver_register fail, Error : %d\n", err);
		return err;
    }
	log("Driver latch_sensor ---\n");
	return err;
}

static void __exit latch_sensor_exit(void)
{
	log("Driver EXIT +++\n");
	e = regulator_disable(latch_sensor_dev->vdd_supply);
    if (e)
	{
    	err("disable ibb regulator failed,ret=%d\n", e);
    }
    mutex_destroy(&latch_sensor_dev->latch_mutex);
	free_irq(ASUS_LATCH_SENSOR_IRQ, latch_sensor_dev);
	wakeup_source_remove(latch_sensor_dev->wake_src);
	wakeup_source_destroy(latch_sensor_dev->wake_src);
	//wakeup_source_trash(&latch_sensor_dev->wake_src);
	unregister_chrdev(major,DRIVER_NAME);
	i2c_del_driver(&latch_sensor_driver);
	latch_sensor_dev=NULL;
	kfree(latch_sensor_dev);
	gpio_free(ASUS_LATCH_SENSOR_GPIO);
	log("Driver EXIT ---\n");
}

module_init(latch_sensor_init);
module_exit(latch_sensor_exit);

MODULE_DESCRIPTION("ak09970 Latch Sensor");
MODULE_LICENSE("GPL v2");
