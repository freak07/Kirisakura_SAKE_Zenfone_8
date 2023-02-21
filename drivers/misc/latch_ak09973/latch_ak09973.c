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
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>

#define LATCH_DEBUG_LOG 1

#ifdef LATCH_DEBUG_LOG
	#define log_ak09973(fmt, args...) printk(KERN_INFO"[%s] "fmt,DRIVER_NAME,##args)
	#define err_ak09973(fmt, args...) printk(KERN_ERR "[%s] "fmt,DRIVER_NAME,##args)
#else
	#define log_ak09973(fmt, args...)
	#define err_ak09973(fmt, args...)
#endif

#define DRIVER_NAME                   "latch_ak09973"
#define INT_NAME                      "LatchAk09973_INT"
#define REPORT_WAKE_LOCK_TIMEOUT      (1 * HZ)
#define DELAYED_WORK_TIME             500

static struct latch_ak09973_str {
    int sleep;
	int status;
	int enable;
	int hardcode;
    int debounce;
    struct i2c_client     *client;
	struct wakeup_source  *wake_src;
	struct regulator      *vdd_supply;
	struct mutex          latch_mutex;
	struct delayed_work   latch_ak09973_work;
}* latch_ak09973_dev;

struct device *dev = NULL;
static struct workqueue_struct 	*latch_ak09973_wq;
static struct i2c_client        *report_client;
static int major;
static int g_state=-1;
static int is_suspend=0;
static int g_threshold1X=-400;
//static int g_threshold2X=550;
static int g_threshold1Y=-250;
static int g_threshold1Z=3000;
static int g_threshold1V=800;
static int ASUS_LATCH_AK09973_GPIO;
static int ASUS_LATCH_AK09973_IRQ;
static int g_reg=-1;


static int latch_open(struct inode *inode, struct file *file)
{
	log_ak09973(" %s.\n",__func__);
	return 0;
}


static const struct file_operations latch_fops = {
	.owner = THIS_MODULE,
	.open = latch_open,
};

struct miscdevice latch_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asuslatch",
	.fops = &latch_fops
};


static int latch_setup(void)
{
	int ret;

	ret = misc_register(&latch_misc);
	if (ret < 0) {
		err_ak09973(" could not register mcu latch device\n",__func__);
	}
	return ret;

}


static int latch_ak09973_suspend(struct device *dev)
{
	log_ak09973("latch_ak09973 SUSPEND +++\n");
	is_suspend = 1;
	log_ak09973("latch_ak09973 SUSPEND ---\n");
	return 0;
}

static int latch_ak09973_resume(struct device *dev)
{
	log_ak09973("latch_ak09973 RESUME +++\n");
	is_suspend = 0;
	log_ak09973("latch_ak09973 RESUME ---\n");
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
		err_ak09973("[latch_ak09973] %s error %d,will retry\n",__func__,ret);
		ret = i2c_transfer(client->adapter,&msg, 1);
		if(ret <= 0)
		{
			err_ak09973("[latch_ak09973] %s error %d\n",__func__,ret);
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
		err_ak09973("[latch_ak09973] i2c_write_bytes:err %d\n", err);

	msleep(1);//wait for ic

	msgs.flags = I2C_M_RD;		//read
	msgs.addr = client->addr;
	msgs.len = 8;
	msgs.buf = data;

	err = i2c_transfer(client->adapter,&msgs, 1);
	if(err <= 0)
	{
		err_ak09973("[latch_ak09973] %s error %d,will retry\n",__func__,err);
		err = i2c_transfer(client->adapter,&msgs, 1);
		if(err <= 0)
		{
			err_ak09973("[latch_ak09973] %s error %d\n",__func__,err);
		}
	}
	return err;
}

static irqreturn_t latch_ak09973_reenable_irq(int irq, void *dev_id)
{

	unsigned char data[8]={0};
	int err = 0;
	int SWX=-1;
	int SWY=-1;
	int SWV=-1;

	if(is_suspend == 1){
		log_ak09973("%s,is_suspend=1\n",__func__);
		__pm_wakeup_event(latch_ak09973_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT+(4 * HZ));
		queue_delayed_work(latch_ak09973_wq, &latch_ak09973_dev->latch_ak09973_work, msecs_to_jiffies(DELAYED_WORK_TIME*2));
        return IRQ_HANDLED;
    }

	log_ak09973("[ISR] %s latch_ak09973_interrupt = %d\n",__func__,ASUS_LATCH_AK09973_IRQ);
	__pm_wakeup_event(latch_ak09973_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT+(1 * HZ));
	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(report_client, 0x17, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] %s:err %d\n",__func__,err);
		return IRQ_HANDLED;
	}

	SWX = (data[0]&0x2)>>1;  //ST-D1
	SWY = (data[0]&0x4)>>2;  //ST-D2
	SWV = (data[0]&0x10)>>4;  //ST-D2
	err = cancel_delayed_work(&latch_ak09973_dev->latch_ak09973_work);
	if(err == 1 ){
		log_ak09973("cancel pending work  SWX=%d  SWY=%d SWV=%d\n",SWX,SWY,SWV);
	}else{
		log_ak09973("no pending work  SWX=%d  SWY=%d SWV=%d\n",SWX,SWY,SWV);
	}

	queue_delayed_work(latch_ak09973_wq, &latch_ak09973_dev->latch_ak09973_work, msecs_to_jiffies(DELAYED_WORK_TIME));

	return IRQ_HANDLED;
}


static void debounce_latch_ak09973_report_function(struct work_struct *dat)
{
//	struct i2c_client *client = to_i2c_client(&client->dev);
	unsigned char data[8]={0};
	int err = 0;
	char * envp[2];
	int X_value=0;
	int Y_value=0;
	int Z_value=0;
	int V_value=0;
	int SWX=-1;
	int SWY=-1;
	int SWZ=-1;
	int SWV=-1;

	__pm_wakeup_event(latch_ak09973_dev->wake_src, REPORT_WAKE_LOCK_TIMEOUT);
	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(report_client, 0x17, data);
	if (err != 1)
		err_ak09973("[latch_ak09973] read  stxyz:err %d\n", err);

	mutex_unlock(&latch_ak09973_dev->latch_mutex);

	SWX = (data[0]&0x2)>>1;  //ST-D1
	SWY = (data[0]&0x4)>>2;  //ST-D2
	SWZ = (data[0]&0x8)>>3;  //ST-D3
	SWV = (data[0]&0x10)>>4; //ST-D4

	if(data[5] >= 128)
		X_value = ~((data[5]^255)* 256 + (data[6]^255));
	else
	        X_value = data[5]*256 + data[6];

	if(data[3] >= 128)
		Y_value = ~((data[3]^255)* 256 + (data[4]^255));
	else
		Y_value = data[3]*256 + data[4];

	if(data[1] >= 128)
		Z_value = ~((data[1]^255)* 256 + (data[2]^255));
	else
		Z_value = data[1]*256 + data[2];

	//read v
	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(report_client, 0x18, data);
	if (err != 1)
		err_ak09973("[latch_ak09973] read  stv:err %d\n", err);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	V_value = data[1] *256*256*256 + data[2]*256*256 + data[3]*256 +data[4];


	log_ak09973("interrupt  X_value=%d,Y_value=%d,Z_value=%d,V_value=%d,SWX=%d,SWY=%d,SWZ=%d,SWV=%d\n",X_value,Y_value,Z_value,V_value,SWX,SWY,SWZ,SWV);

	/*if( ((SWX==0) && (SWY==0)) )
	{
		envp[0] = "STATUS=CLOSE";
		envp[1] = NULL;
		kobject_uevent_env(&((latch_misc.this_device)->kobj), KOBJ_CHANGE, envp);
		g_state=2;
		if((SWX==0) && (SWY==0))
			log_ak09973("old latch g_threshold1X=%d\n",g_threshold1X);
		else
			log_ak09973("new latch g_threshold2X=%d\n",g_threshold2X);
	}else if( ((SWX==0) && (SWY==1)) )
	{
		envp[0] = "STATUS=OPEN";
		envp[1] = NULL;
		kobject_uevent_env(&((latch_misc.this_device)->kobj), KOBJ_CHANGE, envp);
		g_state=1;
		if((SWX==0) && (SWY==1))
			log_ak09973("old latch g_threshold1X=%d\n",g_threshold1X);
		else
			log_ak09973("new latch g_threshold2X=%d\n",g_threshold2X);
	}else if(SWX==1)
	{
		envp[0] = "STATUS=NONE";
		envp[1] = NULL;
		kobject_uevent_env(&((latch_misc.this_device)->kobj), KOBJ_CHANGE, envp);
		g_state=0;
	}*/

	//new check rule
	if( ((SWV==1) && (SWY==0)) )
	{
		if(X_value > 300 && V_value > 18000000)
		{
			envp[0] = "STATUS=CLOSE_X";
			g_state=2;
			envp[1] = NULL;
			kobject_uevent_env(&((latch_misc.this_device)->kobj), KOBJ_CHANGE, envp);
		}
		else{
			g_state=2;
		}
		envp[0] = "STATUS=CLOSE";
		envp[1] = NULL;
		kobject_uevent_env(&((latch_misc.this_device)->kobj), KOBJ_CHANGE, envp);

	}else if( ((SWV==0) || (SWY==1)) )
	{
		envp[0] = "STATUS=OPEN";
		envp[1] = NULL;
		kobject_uevent_env(&((latch_misc.this_device)->kobj), KOBJ_CHANGE, envp);
		g_state=1;
	}

}

static int latch_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr & 0xFF;
	buf[1] = value;

	err = i2c_write_bytes(client, buf, 2);
	if (err !=1)
		err_ak09973("[latch_ak09973] i2c_write_bytes:err %d\n", err);
	return err;
}

static int latch_write_threshold(struct i2c_client *client, short addr, int bop1x,int brp1x)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr & 0xFF; //0x22 set x threshold;  0x23,set y threshold
	buf[1] = (bop1x & 0xFF00)>>8; //bopx high byte
	buf[2] = bop1x & 0xFF; //bopx low byte
	buf[3] = (brp1x & 0xFF00)>>8;  //brpx high byte
	buf[4] = brp1x & 0xFF;  //brpx low byte

	err = i2c_write_bytes(client, buf, 5);
	if (err !=1)
		err_ak09973("[latch_ak09973] i2c_write_bytes:err %d\n", err);

	return err;
}

static ssize_t X_threshold_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	int request=0;
	struct i2c_client *client = report_client;

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	ret=latch_write_threshold(client,0x22,request+32,request-32);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);

	if(ret<0){
		log_ak09973("[latch_ak09973]%s: latch_write_threshold write X1threshold %d fail\n",__func__,request);
		return count;
	}

	g_threshold1X=request;
	log_ak09973("[latch_ak09973]%s: X1threshold=%d\n",__func__,request);
	return count;
}
static ssize_t X_threshold_show(struct class *c,struct class_attribute *attr, char *buf)
{
	log_ak09973("[latch_ak09973]%s: g_threshold1X=%d\n",__func__,g_threshold1X);
	return snprintf(buf, PAGE_SIZE,"g_threshold1X=%d\n", g_threshold1X);
}
static CLASS_ATTR_RW(X_threshold);

static ssize_t Y_threshold_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	int request=0;
	struct i2c_client *client = report_client;

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	ret=latch_write_threshold(client,0x23,request+30,request-30);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if(ret<0){
		log_ak09973("[latch_ak09973]%s: latch_write_threshold write threshold %d fail\n",__func__,request);
		return count;
	}
	g_threshold1Y=request;
	log_ak09973("[latch_ak09973]%s: threshold=%d\n",__func__,request);
	return count;
}


static ssize_t Y_threshold_show(struct class *c,struct class_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//unsigned char data[8]={0};
	//int err = 0;
	log_ak09973("[latch_ak09973]%s: g_threshold1Y=%d\n",__func__,g_threshold1Y);
	return snprintf(buf, PAGE_SIZE,"g_threshold1Y=%d\n", g_threshold1Y);
}
static CLASS_ATTR_RW(Y_threshold);
static ssize_t Z_threshold_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	int request=0;
	struct i2c_client *client = report_client;

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	ret=latch_write_threshold(client,0x24,request+32,request-32);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if(ret<0){
		log_ak09973("[latch_ak09973]%s: latch_write_threshold write threshold %d fail\n",__func__,request);
		return count;
	}
	g_threshold1Z=request;
	log_ak09973("[latch_ak09973]%s: threshold=%d\n",__func__,request);
	return count;
}

static ssize_t Z_threshold_show(struct class *c,struct class_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//unsigned char data[8]={0};
	//int err = 0;
	log_ak09973("[latch_ak09973]%s: g_threshold1Z=%d\n",__func__,g_threshold1Z);
	return snprintf(buf, PAGE_SIZE,"g_threshold1Z=%d\n", g_threshold1Z);
}
static CLASS_ATTR_RW(Z_threshold);

static ssize_t V_threshold_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	long int request=0;
	struct i2c_client *client = report_client;

	sscanf(buf, "%d", &request);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	ret=latch_write_threshold(client,0x25,request+30,request-30);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if(ret<0){
		log_ak09973("[latch_ak09973]%s: latch_write_threshold write threshold %d fail\n",__func__,request);
		return count;
	}
	g_threshold1V=request;
	log_ak09973("[latch_ak09973]%s: threshold=%d\n",__func__,request);
	return count;
}

static ssize_t V_threshold_show(struct class *c,struct class_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//unsigned char data[8]={0};
	//int err = 0;
	log_ak09973("[latch_ak09973]%s: g_threshold1V=%d\n",__func__,g_threshold1V);
	return snprintf(buf, PAGE_SIZE,"g_threshold1V=%d\n", g_threshold1V);
}
static CLASS_ATTR_RW(V_threshold);

static ssize_t trigger_update_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	log_ak09973("[latch_ak09973]%s: entry \n",__func__);
	queue_delayed_work(latch_ak09973_wq, &latch_ak09973_dev->latch_ak09973_work, 0);
	return count;
}
static CLASS_ATTR_WO(trigger_update);


static ssize_t mode_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8]={0};
	int err = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] show mode:err %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}
	log_ak09973("[latch_ak09973] %s  mode=%x\n",__func__,data[0]);
	return snprintf(buf,  PAGE_SIZE,"mode %x\n", data[0]);
}


static ssize_t mode_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	int request=0 ;

	struct i2c_client *client = report_client;
   	sscanf(buf, "%x", &request);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	ret = latch_write_bytes(client, 0x21, request);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if(ret < 0)
	{
		err_ak09973("[latch_ak09973] store mode error\n");
		return count;
	}
	log_ak09973("[latch_ak09973]write %d to mode\n",request);
	return count;
}
static CLASS_ATTR_RW(mode);

static int latch_write_interrupt(struct i2c_client *client, short addr, int value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = addr & 0xFF;
	//buf[1] = 0x00;
	buf[1] = (value & 0xFF00) >> 8; //byte[0]; CNTL1[15:8]
	buf[2] = value & 0xFF; //byte[1]; CNTL1[7:0]

	err = i2c_write_bytes(client, buf, 3);
	if (err !=1)
		err_ak09973("[latch_ak09973] i2c_write_bytes:err %d\n", err);

	return err;
}
static ssize_t interrupt_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8]={0};
	int err = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x20, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] interrupt show:err %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x20 error");
	}
	log_ak09973("[latch_ak09973] %s  data[0]=%d data[1]=%d\n",__func__,data[0],data[1]);
	return snprintf(buf,  PAGE_SIZE,"int %d-%d\n", data[0],data[1]);
}

static ssize_t interrupt_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	int  request=0;

	struct i2c_client *client = report_client;
   	sscanf(buf, "%x", &request);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	ret = latch_write_interrupt(client, 0x20, request);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if(ret < 0)
	{
		err_ak09973("[latch_ak09973]write interrupt send error\n");
		return count;
	}
	log_ak09973("[latch_ak09973]write %d to interrupr\n",request);
	return count;
}
static CLASS_ATTR_RW(interrupt);

static ssize_t stxyz_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 66: sleeptime = 200;
				  break;
		case 68: sleeptime = 100;
				  break;
		case 70: sleeptime = 50;
				  break;
		case 72: sleeptime = 20;
				  break;
		case 74: sleeptime = 10;
				  break;
		case 76: sleeptime = 2;
				  break;
		case 78: sleeptime = 1;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}


	if(data[0] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_ak09973_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_ak09973_dev->latch_mutex);
		if (err != 1){
			err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[5] * 256 + data[6];
	p = data[3] * 256 + data[4];
	o = data[1] * 256 + data[2];
	log_ak09973("[latch_ak09973] %s  status:%d-%d z:%ld y:%ld x:%ld\n",__func__,data[0],data[1],o,p,q);
	return snprintf(buf,  PAGE_SIZE,"status:%d-%d z:%ld y:%ld x:%ld\n", data[0],data[1],o,p,q);
}
static CLASS_ATTR_RO(stxyz);

static ssize_t X_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 66: sleeptime = 200;
				  break;
		case 68: sleeptime = 100;
				  break;
		case 70: sleeptime = 50;
				  break;
		case 72: sleeptime = 20;
				  break;
		case 74: sleeptime = 10;
				  break;
		case 76: sleeptime = 2;
				  break;
		case 78: sleeptime = 1;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[0] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_ak09973_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_ak09973_dev->latch_mutex);
		if (err != 1){
			err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[5] * 256 + data[6];
	p = data[3] * 256 + data[4];
	o = data[1] * 256 + data[2];

	if(data[5] >= 128)
		q = ~((data[5]^255)* 256 + (data[6]^255));

	if(data[3] >= 128)
		p = ~((data[3]^255)* 256 + (data[4]^255));

	if(data[1] >= 128)
		o = ~((data[1]^255)* 256 + (data[2]^255));
	log_ak09973("[latch_ak09973] %s  x=%d\n",__func__,q);
	return snprintf(buf,  PAGE_SIZE,"%d\n",q);
}
static CLASS_ATTR_RO(X);
static ssize_t Y_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 66: sleeptime = 200;
				  break;
		case 68: sleeptime = 100;
				  break;
		case 70: sleeptime = 50;
				  break;
		case 72: sleeptime = 20;
				  break;
		case 74: sleeptime = 10;
				  break;
		case 76: sleeptime = 2;
				  break;
		case 78: sleeptime = 1;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[0] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_ak09973_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_ak09973_dev->latch_mutex);
		if (err != 1){
			err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	q = data[5] * 256 + data[6];
	p = data[3] * 256 + data[4];
	o = data[1] * 256 + data[2];

	if(data[5] >= 128)
		q = ~((data[5]^255)* 256 + (data[6]^255));

	if(data[3] >= 128)
		p = ~((data[3]^255)* 256 + (data[4]^255));

	if(data[1] >= 128)
		o = ~((data[1]^255)* 256 + (data[2]^255));
	log_ak09973("[latch_ak09973] %s  y=%d\n",__func__,p);
	return snprintf(buf,  PAGE_SIZE,"%d\n",p);
}
static CLASS_ATTR_RO(Y);
static ssize_t Z_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int q,p,o = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 66: sleeptime = 200;
				  break;
		case 68: sleeptime = 100;
				  break;
		case 70: sleeptime = 50;
				  break;
		case 72: sleeptime = 20;
				  break;
		case 74: sleeptime = 10;
				  break;
		case 76: sleeptime = 2;
				  break;
		case 78: sleeptime = 1;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x17, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[0] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_ak09973_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x17, data);
		mutex_unlock(&latch_ak09973_dev->latch_mutex);
		if (err != 1){
			err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}
	q = data[5] * 256 + data[6];
	p = data[3] * 256 + data[4];
	o = data[1] * 256 + data[2];

	if(data[5] >= 128)
		q = ~((data[5]^255)* 256 + (data[6]^255));

	if(data[3] >= 128)
		p = ~((data[3]^255)* 256 + (data[4]^255));

	if(data[1] >= 128)
		o = ~((data[1]^255)* 256 + (data[2]^255));
	log_ak09973("[latch_ak09973] %s  z=%d\n",__func__,o);
	return snprintf(buf,  PAGE_SIZE,"%d\n",o);
}
static CLASS_ATTR_RO(Z);
static ssize_t V_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	unsigned char modedata[8] = {0};
	int err = 0;
	int sleeptime = 0;
	long int v = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x21, modedata);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x21 error");
	}

	switch(modedata[0])
	{
		case 66: sleeptime = 200;
				  break;
		case 68: sleeptime = 100;
				  break;
		case 70: sleeptime = 50;
				  break;
		case 72: sleeptime = 20;
				  break;
		case 74: sleeptime = 10;
				  break;
		case 76: sleeptime = 2;
				  break;
		case 78: sleeptime = 1;
				  break;
		default:
				  break;
	}

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x18, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
	}

	if(data[0] == 0)
	{
		msleep(sleeptime);
		mutex_lock(&latch_ak09973_dev->latch_mutex);
		err = i2c_read_bytes(client, 0x18, data);
		mutex_unlock(&latch_ak09973_dev->latch_mutex);
		if (err != 1){
			err_ak09973("[latch_ak09973] read stxyz error %d\n", err);
			return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x17 error");
		}
	}

	v = data[1] *256*256*256 + data[2]*256*256 + data[3]*256 +data[4];
	log_ak09973("[latch_ak09973] %s  v=%d\n",__func__,v);
	return snprintf(buf,  PAGE_SIZE,"%d\n", v);
}
static CLASS_ATTR_RO(V);
static ssize_t status_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	int err = 0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, 0x00, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read status error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read 0x00 error");
	}
	log_ak09973("[latch_ak09973] %s  status=%d\n",__func__,data[0]);
	return snprintf(buf, PAGE_SIZE,"%x\n", data[0]);
}
static CLASS_ATTR_RO(status);

static ssize_t state_show(struct class *c,struct class_attribute *attr, char *buf)
{
	char * state = 0;
	if(g_state==0){
		state="NONE";
	}else if(g_state==1){
		state="OPEN";
	}else if(g_state==2){
		state="CLOSE";
	}else if(g_state==3){
		state="CLOSE_X";
	}
	log_ak09973("[latch_ak09973] %s  g_state=%d\n",__func__,g_state);
	return snprintf(buf, PAGE_SIZE,"%s\n", state);
}
static CLASS_ATTR_RO(state);
static ssize_t reg_store(struct class *c,struct class_attribute *attr,const char *buf, size_t count)
{
	int reg=-1;
	sscanf(buf, "%x", &reg);
	log_ak09973("[latch_ak09973]%s: reg=%d\n",__func__,reg);
	g_reg=reg;
	return count;
}

static ssize_t reg_show(struct class *c,struct class_attribute *attr, char *buf)
{
	struct i2c_client *client = report_client;
	unsigned char data[8] = {0};
	int err = 0;
	int byte1 =0;
	int byte2 =0;

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	err = i2c_read_bytes(client, g_reg, data);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	if (err != 1){
		err_ak09973("[latch_ak09973] read status error %d\n", err);
		return snprintf(buf,  PAGE_SIZE,"%s\n", "read  error");
	}
	log_ak09973("[latch_ak09973] %s  status=%d\n",__func__,data[0]);
	byte1=data[0]*256+data[1];
	byte2=data[2]*256+data[3];
	if(data[0] >= 128)
		byte1 = ~((data[0]^255)* 256 + (data[1]^255));

	if(data[2] >= 128)
		byte2 = ~((data[2]^255)* 256 + (data[3]^255));
	return snprintf(buf, PAGE_SIZE," data[3]=0x%x, data[2]=0x%x, data[1]=0x%x, data[0]=0x%x,0x%x,0x%x\n", data[3],data[2],data[1],data[0],byte1,byte2);
}
static CLASS_ATTR_RW(reg);

static struct attribute *asus_latch_class_attrs[] = {
    &class_attr_X.attr,
    &class_attr_Y.attr,
    &class_attr_Z.attr,
    &class_attr_V.attr,
    &class_attr_X_threshold.attr,
    &class_attr_Y_threshold.attr,
    &class_attr_Z_threshold.attr,
    &class_attr_V_threshold.attr,
    &class_attr_interrupt.attr,
    &class_attr_mode.attr,
    &class_attr_reg.attr,
    &class_attr_state.attr,
    &class_attr_status.attr,
    &class_attr_stxyz.attr,
    &class_attr_trigger_update.attr,
    NULL,
};
ATTRIBUTE_GROUPS(asus_latch_class);

struct class asus_latch_class = {
    .name = "asus_latch",
    .class_groups = asus_latch_class_groups,
};


static void set_pinctrl(struct device *dev)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	key_pinctrl = devm_pinctrl_get(dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "latch_gpio_high");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	log_ak09973("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
}

static int init_data(void)
{
	int ret = 0;
	
	// Memory allocation for data structure 
	latch_ak09973_dev = kzalloc(sizeof (struct latch_ak09973_str), GFP_KERNEL);
	if (!latch_ak09973_dev) {
		err_ak09973("Memory allocation fails for latch ak09973\n");
		ret = -ENOMEM;
		goto init_data_err;
	}
	latch_ak09973_dev->wake_src=wakeup_source_create("LatchAk09973_wake_lock");
	wakeup_source_add(latch_ak09973_dev->wake_src);
	//wakeup_source_init(&latch_ak09973_dev->wake_src, "LatchAk09973_wake_lock");
	return 0;
init_data_err:
	err_ak09973("Init Data ERROR\n");
	return ret;
}

static int init_irq (void)
{
	int ret = 0;

	/* GPIO to IRQ */
	ASUS_LATCH_AK09973_IRQ = gpio_to_irq(ASUS_LATCH_AK09973_GPIO);

	if (ASUS_LATCH_AK09973_IRQ < 0) {
		err_ak09973("[IRQ] gpio_to_irq ERROR, irq=%d.\n", ASUS_LATCH_AK09973_IRQ);
	}else {
		log_ak09973("[IRQ] gpio_to_irq IRQ %d successed on GPIO:%d\n", ASUS_LATCH_AK09973_IRQ, ASUS_LATCH_AK09973_GPIO);
	}

	ret = request_threaded_irq(ASUS_LATCH_AK09973_IRQ, NULL, latch_ak09973_reenable_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |IRQF_ONESHOT,
				INT_NAME, latch_ak09973_dev);

	if (ret < 0)
		err_ak09973("[IRQ] request_irq() ERROR %d.\n", ret);
	else {
		log_ak09973("[IRQ] Enable irq !! \n");
		enable_irq_wake(ASUS_LATCH_AK09973_IRQ);
	}

	return 0;
}


static int latch_ak09973_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	int ret = 0;

	log_ak09973("Probe +++\n");
	ret = init_data();
	if (ret < 0)
		goto probe_err;

	mutex_init(&latch_ak09973_dev->latch_mutex);

	/* GPIO */
	ASUS_LATCH_AK09973_GPIO = of_get_named_gpio(client->dev.of_node, "latch,int-gpio", 0);
	log_ak09973("[GPIO] GPIO =%d(%d)\n", ASUS_LATCH_AK09973_GPIO, gpio_get_value(ASUS_LATCH_AK09973_GPIO));
	gpio_free(ASUS_LATCH_AK09973_GPIO);
	/* GPIO Request */
	report_client = client;
	set_pinctrl(&client->dev);

	/* GPIO Direction */
	ret = gpio_direction_input(ASUS_LATCH_AK09973_GPIO);
	if (ret < 0) {
		err_ak09973("[GPIO] Unable to set the direction of gpio %d\n", ASUS_LATCH_AK09973_GPIO);
		goto probe_err;
	}

	latch_ak09973_dev->vdd_supply = regulator_get(&client->dev, "vdd");
	if (IS_ERR(latch_ak09973_dev->vdd_supply)) {
		err_ak09973("ret vdd regulator failed,ret=%d", ret);
		goto probe_err;
	}
	ret = regulator_enable(latch_ak09973_dev->vdd_supply);
	if (ret) {
		err_ak09973("enable vcc_i2c regulator failed,ret=%d", ret);
	}
	msleep(10);
	//check i2c function
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err_ak09973("[latch_ak09973] I2C function test error\n");
		goto probe_err;
	} else
		log_ak09973("[latch_ak09973] I2C function test pass\n");
	
	mutex_lock(&latch_ak09973_dev->latch_mutex);
	latch_write_bytes(client,0x30,0);
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	msleep(5);
	mutex_lock(&latch_ak09973_dev->latch_mutex);
	latch_write_bytes(client,0x21,0x44); //value is 0x28 ,Wide measurement range; low noise drive; mode4
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	msleep(5);

	mutex_lock(&latch_ak09973_dev->latch_mutex);
	//latch_write_threshold(client,0x22,g_threshold1X+32,g_threshold1X-32); //0x22, write X threshold
	latch_write_threshold(client,0x23,g_threshold1Y+30,g_threshold1Y-30); //0x23, write Y threshold
	//latch_write_threshold(client,0x24,g_threshold1Z+30,g_threshold1Z-30); //0x24, write z threshold
	latch_write_threshold(client,0x25,g_threshold1V+30,g_threshold1V-30); //0x25, write v threshold
	ret = latch_write_interrupt(client, 0x20, 0x0814); //set POLV=1; SWVEN=1 ;SWYEN=1
	if(ret < 0)
	{
		err_ak09973("[latch_ak09973]write interrupt [0x00] [0x06]  send error\n");
		mutex_unlock(&latch_ak09973_dev->latch_mutex);
		goto probe_err;
	}
	mutex_unlock(&latch_ak09973_dev->latch_mutex);
	msleep(5);

	//register for class
	ret = latch_setup();
	if (ret < 0) {
		log_ak09973("[latch_ak09973]latch misc dev setup error\n");
		goto probe_err;
	}
	// Register the class node
	ret = class_register(&asus_latch_class);
	if (ret) {
		pr_err("%s: Failed to register asus_latch class\n", __func__);
		goto probe_err;
	}

	/* Work Queue init */
    latch_ak09973_wq = create_singlethread_workqueue("latch_ak09973_wq");
    INIT_DEFERRABLE_WORK(&latch_ak09973_dev->latch_ak09973_work, debounce_latch_ak09973_report_function);
    ret = init_irq();
    if (ret < 0)
         goto probe_err;
    queue_delayed_work(latch_ak09973_wq, &latch_ak09973_dev->latch_ak09973_work, 0); //add thhis work to get the reboot status

    log_ak09973("Probe ---\n");
	return 0;

probe_err:
	err_ak09973("Probe ERROR\n");
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

static struct of_device_id latchak09973_match_table[] = {
	{ .compatible = "qcom,latch-ak09973",},
	{},
};

static struct dev_pm_ops latch_pm = {
    .suspend = latch_ak09973_suspend,
    .resume  = latch_ak09973_resume,    
};

static struct i2c_driver latch_ak09973_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = latchak09973_match_table,
		.pm  = &latch_pm,
	},
	.probe          = latch_ak09973_probe,
	.id_table	= latch_id_table,
};

static int __init latch_ak09973_init(void)
{
	int err = 0;
	log_ak09973("Driver latch_ak09973 +++\n");
	err = i2c_add_driver(&latch_ak09973_driver);
	if (err != 0) {
		err_ak09973("[Latch Ak09973] i2c_driver_register fail, Error : %d\n", err);
		return err;
    }
	log_ak09973("Driver latch_ak09973 ---\n");
	return err;
}

static void __exit latch_ak09973_exit(void)
{
	int err = 0;
	log_ak09973("Driver EXIT +++\n");
	misc_deregister(&latch_misc);
	err = regulator_disable(latch_ak09973_dev->vdd_supply);
    if (err)
	{
    	err_ak09973("disable ibb regulator failed,ret=%d\n", err);
    }

    mutex_destroy(&latch_ak09973_dev->latch_mutex);
	free_irq(ASUS_LATCH_AK09973_IRQ, latch_ak09973_dev);
	wakeup_source_remove(latch_ak09973_dev->wake_src);
	wakeup_source_destroy(latch_ak09973_dev->wake_src);
	//wakeup_source_trash(&latch_ak09973_dev->wake_src);
	unregister_chrdev(major,DRIVER_NAME);
	i2c_del_driver(&latch_ak09973_driver);
	latch_ak09973_dev=NULL;
	kfree(latch_ak09973_dev);
	gpio_free(ASUS_LATCH_AK09973_GPIO);
	class_unregister(&asus_latch_class);
	log_ak09973("Driver EXIT ---\n");
}

module_init(latch_ak09973_init);
module_exit(latch_ak09973_exit);

MODULE_DESCRIPTION("Latch Ak09973");
MODULE_LICENSE("GPL v2");
