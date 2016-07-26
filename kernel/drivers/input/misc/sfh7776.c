/* drivers/i2c/chip/sfh7776.c - sfh7776 sensors driver
 *
 * Copyright (C) 2013 OSRAM, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more detaDLOG.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/wakelock.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/sensors.h>
//bangdc add
#include <linux/regulator/consumer.h>
//

#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#include <linux/sfh7776.h>

//#define SFH7776_DEBUG

#ifdef SFH7776_DEBUG
#define DLOG(x...) printk(KERN_INFO "[SFH7776 DEBUG] " x)
#define ELOG(x...) printk(KERN_INFO "[SFH7776 ERROR] " x)
#else
#define DLOG(x...)
#define ELOG(x...)
#endif

#define I2C_RETRY_COUNT 3

static void sfh7776_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sfh7776_irq_work, sfh7776_irq_do_work);

//static void als_polling_do_work(struct work_struct *w);
//static DECLARE_DELAYED_WORK(als_polling_work, als_polling_do_work);

//static void ps_polling_do_work(struct work_struct *w);
//static DECLARE_DELAYED_WORK(ps_polling_work, ps_polling_do_work);

static struct sensors_classdev sensors_light_cdev = {
		.name = "sfh7776-light",
		.vendor = "SFH",
		.version = 1,
		.handle = SENSORS_LIGHT_HANDLE,
		.type = SENSOR_TYPE_LIGHT,
		.max_range = "6553", //6553
		.resolution = "0.0125",
		.sensor_power = "0.15",
		.min_delay = 0,
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = 100, //ms
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
		.name = "sfh7776-proximity",
		.vendor = "SFH",
		.version = 1,
		.handle = SENSORS_PROXIMITY_HANDLE,
		.type = SENSOR_TYPE_PROXIMITY,
		.max_range = "5.0",
		.resolution = "5.0",
		.sensor_power = "0.18",
		.min_delay = 0,
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = 100, //ms
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
};

struct sfh7776_info {
	struct class *sfh7776_class;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	//datntd
	struct delayed_work pdwork;
	struct delayed_work ldwork; //hang doi polling data
	//datntd

	struct device *als_dev;
	struct device *ps_dev;

	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;

	int intr_pin;
	int irq;
	int pending_intr;
//datntd
	int current_level;
	int old_level;
//datntd
	int als_enable;
	int als_suspend;
	int ps_enable;
	int als_polling;
	int ps_polling;

	//int datntd;
	uint8_t ps_lt;
	uint8_t ps_ht;

	uint16_t als_level_tabl[ALS_LEVEL_NUM];
	uint16_t als_value_tabl[ALS_LEVEL_NUM + 1];

	int (*power)(int, uint8_t); /* power to the chip */
	struct wake_lock ps_wake_lock;
//bangdc add
	struct regulator *vdd;
	struct regulator *vcc_i2c;
//
	int als_debounce; /*debounce time after enabling als*/
	int als_deb_on; /*indicates if the debounce is on*/
	int als_deb_end; /*the jiffies representing the end of debounce*/
	int als_delay;
	int ps_debounce; /*debounce time after enabling ps*/
	int ps_deb_on; /*indicates if the debounce is on*/
	int ps_deb_end; /*the jiffies representing the end of debounce*/
	int ps_delay;
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
};

typedef enum {
	SFH_BIT_ALS = 1, SFH_BIT_PS = 2,
};

static struct sfh7776_info *drv_info = NULL;

static irqreturn_t sfh7776_irq_handler(int irq, void *data);

//set param from BoardConfig.mk
static int ps_current;
static int proximity_low;
static int proximity_high;

static int __init ps_lt_level(char *str)
{
	int newlevel;

	if (get_option(&str, &newlevel)) {
		proximity_low = newlevel;
		return 0;
	}
	return -EINVAL;
}
early_param("ps_lt", ps_lt_level);

static int __init ps_ht_level(char *str)
{
	int newlevel;

	if (get_option(&str, &newlevel)) {
		proximity_high = newlevel;
		return 0;
	}
	return -EINVAL;
}
early_param("ps_ht", ps_ht_level);

static int __init ps_current_level(char *str)
{
	int newlevel;

	if (get_option(&str, &newlevel)) {
		ps_current = newlevel;
		return 0;
	}
	return -EINVAL;
}
early_param("current", ps_current_level);
//end set param

static int sfh7776_init_gpio(int gpio) {
	int ret;

	DLOG("%s\n",__func__);

	ret = gpio_request(gpio, "sfh7776_intr");
	if (ret < 0) {
		ELOG("%s: gpio %d request failed (%d)\n", __func__, gpio, ret);
		return ret;
	}

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		ELOG("%s: failed to set gpio %d as input (%d)\n", __func__, gpio, ret);
		return ret;
	}

	return 0;
}

static int sfh7776_get_gpio(int gpio) {
//	return 1;
	return gpio_get_value(gpio);
}

static void sfh7776_free_gpio(int gpio) {
	gpio_free(gpio);
}
//bangdc add power init
static int sfh7776_power_init(struct sfh7776_info *data, bool on) {
	int rc;
	pr_info("BANGDC==============> %s\n", __func__);
	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->i2c_client->dev, "8941_l18");

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, 2850000, 2850000);
		if (rc) {
			pr_info("BANGDC==============> failed vdd%s\n", __func__);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->i2c_client->dev, "8941_lvs1");

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, 1800000, 1800000);
		if (rc) {
			pr_info("BANGDC==============> failed vcc_i2c%s\n", __func__);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, 2850000);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, 2850000);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, 1800000);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int sfh7776_power_on(struct sfh7776_info *data, bool on) {
	int rc;
	pr_info("BANGDC==============> %s\n", __func__);
	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		pr_info("BANGDC==============>vdd enable failed %s\n", __func__);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		pr_info("BANGDC==============>vcc_i2c enable failed %s\n", __func__);
		regulator_disable(data->vdd);
	}

	return rc;

	power_off: rc = regulator_disable(data->vdd);
	if (rc) {
		pr_info("BANGDC==============>vdd disable failed %s\n", __func__);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		pr_info("BANGDC==============>vcc_i2c disable failed %s\n", __func__);
		/*regulator_enable(data->vdd);*/
	}

	return rc;
}
//
static int sfh7776_init_irq(int irq) {
	int ret = 0;
	char buffer[2];
	struct sfh7776_info *sfh7776 = drv_info;

	DLOG("%s\n",__func__);

	ret = request_any_context_irq(irq, sfh7776_irq_handler,IRQF_TRIGGER_FALLING, "sfh7776", sfh7776);

	if (ret < 0) {
		ELOG("%s: request_irq(%d) failed for gpio %d (%d)\n", __func__, irq, sfh7776->intr_pin, ret);
		return ret;
	}
	return 0;
}

static void sfh7776_mask_irq(int irq, int mask) {
	if (mask) {
		disable_irq_nosync(irq);
	} else {
		enable_irq(irq);
	}
}

static void sfh7776_free_irq(int irq) {
	struct sfh7776_info *sfh7776 = drv_info;

	free_irq(irq, sfh7776);
}

static int sfh7776_read_bytes(char *rxData, int length) {
	struct sfh7776_info *sfh7776 = drv_info;
	uint8_t loop_i;
	int ret;
	struct i2c_msg msgs[] = {
			{
					.addr = sfh7776->i2c_client->addr,
					.flags = 0,
					.len = 1,
					.buf = rxData,
			},
			{
					.addr = sfh7776->i2c_client->addr,
					.flags = I2C_M_RD,
					.len = length,
					.buf = rxData,
			},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if ((ret = i2c_transfer(sfh7776->i2c_client->adapter, msgs, 2)) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		ELOG("%s retry over %d\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return ret;
}

static int sfh7776_write_bytes(char *txData, int length) {
	struct sfh7776_info *sfh7776 = drv_info;
	uint8_t loop_i;
	int ret;
	struct i2c_msg msg[] = {
			{
					.addr = sfh7776->i2c_client->addr,
					.flags = 0,
					.len = length,
					.buf = txData,
			},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if ((ret = i2c_transfer(sfh7776->i2c_client->adapter, msg, 1)) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		ELOG("%s retry over %d\n", __func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return ret;
}

static int sfh7776_power(int enable) {
	struct sfh7776_info *sfh7776 = drv_info;

	if (sfh7776->power)
		sfh7776->power(SFH7776_PWR, enable);

	return 0;
}

static int sfh7776_mode_reg_setting(u8 *reg_val, int als_on, int ps_on) {
	if (als_on == 0 && ps_on == 0)
		*reg_val = 0x0; //als:0 ps:0
	else if (als_on == 0 && ps_on == 1)
		*reg_val = 0x3; //als:0 ps:100ms
	else if (als_on == 1 && ps_on == 0)
		*reg_val = 0x5; //als:100ms/100ms ps:0
	else
		*reg_val = 0x6; //als:100ms/100ms ps:100ms

	return 0;
}

static int sfh7776_enable_als(int enable) {
	int ret = 0;
	char databuf[2];
	struct sfh7776_info *sfh7776 = drv_info;

	DLOG("%s\n", __func__);

	databuf[0] = SFH7776_MOD_CTRL;
	ret = sfh7776_read_bytes(databuf, 1);
	if (ret <= 0) {
		goto EXIT_ERR;
	}
	DLOG("SFH7776_MOD_CTRL als value = %x\n",databuf[0]);

	sfh7776_mode_reg_setting(&databuf[1], enable, sfh7776->ps_enable);
	databuf[0] = SFH7776_MOD_CTRL;
	DLOG("SFH7776_MOD_CTRL enable als value = %x\n",databuf[1]);
	ret = sfh7776_write_bytes(databuf, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}
	if (enable) {
		printk("als sensor is enabled\n");
		//sfh7776->als_deb_on = 1;
		//sfh7776->als_deb_end =  jiffies+sfh7776->als_debounce/(1000/HZ);
		if (sfh7776->als_polling == 1) {
			//queue_delayed_work(sfh7776->lp_wq, &als_polling_work,msecs_to_jiffies(sfh7776->als_delay));
			schedule_delayed_work(&sfh7776->ldwork,msecs_to_jiffies(sfh7776->als_delay));
		}

	} else {
		//sfh7776->als_deb_on = 0;
		//sfh7776->als_deb_end = 0;
		printk("als sensor is disabled\n");
		if (sfh7776->als_polling == 1) {
			//cancel_delayed_work(&als_polling_work);
			cancel_delayed_work_sync(&sfh7776->ldwork);
		}
	}
	sfh7776->als_enable = enable;
	return 0;

	EXIT_ERR:
	ELOG("%s:  fail\n", __func__);
	return ret;
}

static int sfh7776_enable_ps(int enable) {
	int ret = 0;
	char databuf[2];
	struct sfh7776_info *sfh7776 = drv_info;

	DLOG("%s\n", __func__);

	databuf[0] = SFH7776_MOD_CTRL;
	ret = sfh7776_read_bytes(databuf, 1);
	if (ret <= 0) {
		goto EXIT_ERR;
	}

	DLOG("SFH7776_MOD_CTRL ps value = %x enable=%d\n",databuf[0], enable);

	sfh7776_mode_reg_setting(&databuf[1], sfh7776->als_enable, enable);
	databuf[0] = SFH7776_MOD_CTRL;
	DLOG("SFH7776_MOD_CTRL enable ps value = %x\n",databuf[1]);
	ret = sfh7776_write_bytes(databuf, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}

	if (enable) {
		printk("ps sensor is enabled\n");
		//sfh7776->ps_deb_on = 1;
		//sfh7776->ps_deb_end = jiffies+sfh7776->ps_debounce/(1000/HZ);
		if (sfh7776->ps_polling == 1) {
			//queue_delayed_work(sfh7776->lp_wq, &ps_polling_work ,msecs_to_jiffies(sfh7776->ps_delay));//datntd edit ps_polling_work
			schedule_delayed_work(&sfh7776->pdwork,msecs_to_jiffies(sfh7776->ps_delay));
		}
	}
	else
	{
		printk("ps sensor is disabled\n");
		//sfh7776->ps_deb_on = 0;
		//sfh7776->ps_deb_end = 0;
		if (sfh7776->ps_polling == 1){
			//cancel_delayed_work(&ps_polling_work);//dantd edit ps_polling_work
			cancel_delayed_work_sync(&sfh7776->pdwork);
		}
	}

	sfh7776->ps_enable = enable;
	return 0;

	EXIT_ERR:
	ELOG("%s: fail\n", __func__);
	return ret;
}

static int ps_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable) {
	int ret;

	if (enable)
		ret = sfh7776_enable_ps(1);
	else
		ret = sfh7776_enable_ps(0);

	return ret;
}

static int ls_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable) {
	struct sfh7776_info
	*sfh7776 = container_of(sensors_cdev, struct sfh7776_info, ps_cdev);
	int ret;

	if (enable)
		ret = sfh7776_enable_als(1);
	else
		ret = sfh7776_enable_als(0);

	if (ret < 0) {
		dev_err(&sfh7776->i2c_client->dev, "%s: set auto light sensor fail\n",
				__func__);
		return -EIO;
	}

	return 0;
}
//datntd
static int sfh7776_read_als(uint16_t *raw_adc_value) {
	int ret = 0;
	//char value;
	u16 value;
	char buffer[2];
	//uint16_t vis_value, ir_value;
	u16 vis_value, ir_value, result;
	struct sfh7776_info *sfh7776 = drv_info;

	/*
	 * doc gia tri thanh ghi VIS
	 */
	buffer[0] = SFH7776_VIS_ADATA_LSB;
	ret = sfh7776_read_bytes(buffer, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}
	vis_value = buffer[0];
	DLOG("datntddddd sfh7776_read_als() lsb buffer[0] = 0x%x\n", buffer[0]);

	buffer[0] = SFH7776_VIS_ADATA_MSB;
	ret = sfh7776_read_bytes(buffer, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}
	DLOG("datntddddd sfh7776_read_als() msb buffer[0] = 0x%x\n", buffer[0]);
	vis_value = (buffer[0] << 8) | vis_value;
	DLOG("datntddddd sfh7776_read_als() vis_value = 0x%x\n", vis_value);

	/*
	 * doc gia tri thanh ghi IR
	 */
	buffer[0] = SFH7776_IR_ADATA_LSB;
	ret = sfh7776_read_bytes(buffer, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}
	ir_value = buffer[0];
	DLOG("datntddddd sfh7776_read_als() ir_lsb buffer[0] = 0x%x\n",
			buffer[0]);
	buffer[0] = SFH7776_IR_ADATA_MSB;
	ret = sfh7776_read_bytes(buffer, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}
	DLOG("datntddddd sfh7776_read_als() ir_msb buffer[0] = 0x%x\n",
			buffer[0]);
	ir_value = (buffer[0] << 8) | ir_value;
	DLOG("datntddddd sfh7776_read_als() ir_value = 0x%x\n", ir_value);

	value = (ir_value * 1000 / vis_value);
//datntd
	//if (value < 670)
	//	*raw_adc_value = (11071 * vis_value - 14286 * ir_value) / (128 * 1000);
	//else
	//	*raw_adc_value = (5876 * vis_value - 6536 * ir_value) / (128 * 1000);

	//if (value <= 1000)
	//	result = (14286 * vis_value - 11071 * ir_value) / (64 * 1000); //gain=64
	//else if (value < 2640)
	//	result = (3828 * vis_value - 1445 * ir_value) / (64 * 1000);
	//else     
	//	result = 8 * 1914 * vis_value / (64 * 1000);
	//if (result > 20000)
	//	result = 5000;

	result = (12286 * vis_value + 7246 * ir_value) / (64 * 1000); //gain=64

	*raw_adc_value = result;
//datntd

	//*raw_adc_value = value;
	DLOG("datntddddd sfh7776_read_als() ir_value/vis_value = %d\n", value);
	DLOG("%s: als_value_lux = %d vis_value=%d ir_value=%d\n", __func__, *raw_adc_value, vis_value, ir_value);
	return 0;

	EXIT_ERR:
	ELOG("%s: fail\n");
	return ret;
}

static int sfh7776_read_ps(uint16_t *raw_adc_value) {
	int ret = 0;
	char buffer[2];
	struct sfh7776_info *sfh7776 = drv_info;

	buffer[0] = SFH7776_PDATA_LSB;
	ret = sfh7776_read_bytes(buffer, 2);
	if (ret <= 0) {
		ELOG("%s:  fail\n", __func__);
		return ret;
	}
	DLOG("datntddddd sfh7776_read_ps() lsb buffer[0] = 0x%x\n", buffer[0]);
	*raw_adc_value = buffer[0];

	buffer[0] = SFH7776_PDATA_MSB;
	ret = sfh7776_read_bytes(buffer, 2);
	if (ret <= 0) {
		ELOG("%s:  fail\n", __func__);
		return ret;
	}

	DLOG("datntddddd sfh7776_read_ps() msb buffer[0] = 0x%x\n", buffer[0]);
	*raw_adc_value = (buffer[0] << 8) | *raw_adc_value;
	DLOG("datntddddd sfh7776_read_ps() raw_adc_value = 0x%x\n",
			*raw_adc_value);

	return 0;
}

static int sfh7776_get_als_value(uint16_t raw_adc_value) {
	int idx;
	int invalid = 0;
	struct sfh7776_info *sfh7776 = drv_info;

	DLOG("datntdddddd sfh7776_get_als_value() raw_adc_value = %d\n",
			raw_adc_value);

//datntd
	sfh7776->current_level = raw_adc_value;

	if (sfh7776->current_level == sfh7776->old_level) {
		return -1;
	}

	sfh7776->old_level = sfh7776->current_level;

	DLOG("datntdddddd sfh7776_get_als_value() current_level = %d\n", sfh7776->current_level);

	return sfh7776->current_level; //raw_adc_value*13;

}

static int sfh7776_get_ps_value(uint16_t raw_adc_value) {
	int val;
	int invalid = 0;
	static int val_temp = 5;
	struct sfh7776_info *sfh7776 = drv_info;

	if (raw_adc_value >= sfh7776->ps_ht) {
		val = 0; /*close*/
		val_temp = 0;
DLOG	("datntdddddddddd sfh7776_get_ps_value 1 %d ", val);
}
else if(raw_adc_value < sfh7776->ps_lt)
{
	//val = 1;  /*far away*/
	val = 5; /*far away*/
	val_temp = 5;
	DLOG("datntdddddddddd sfh7776_get_ps_value 2 %d ", val);
}

	return val;

}

static void sfh7776_als_report(int als_value) {
	struct sfh7776_info *sfh7776 = drv_info;
	DLOG("datntddddddddd %s: als report als_val=%d\n", __func__,als_value );
	if (als_value != -1) {
		input_report_abs(sfh7776->als_input_dev, ABS_MISC, als_value);
		input_sync(sfh7776->als_input_dev);
		DLOG("datntddd report light sensor value is okkkkkkkkkkkkkkk\n");
	} else
		DLOG("datntddd don't report light sensor value because als_value = %d\n",
				als_value);
}

static void sfh7776_ps_report(int ps_val) {
	struct sfh7776_info *sfh7776 = drv_info;

	DLOG("%s: ps report ps_val=%d\n", __func__,ps_val );

	/* 0 is close, 1 is far */
	input_report_abs(sfh7776->ps_input_dev, ABS_DISTANCE, ps_val);
	input_sync(sfh7776->ps_input_dev);

	//wake_lock_timeout(&(sfh7776->ps_wake_lock), 2*HZ);
}

static int sfh7776_check_intr(void) {
	int ret;
	u8 buffer[2];
	struct sfh7776_info *sfh7776 = drv_info;

	if (sfh7776_get_gpio(sfh7776->intr_pin) == 1) /*skip if no interrupt*/
		return 0;

	buffer[0] = SFH7776_INT_SETTING;
	ret = sfh7776_read_bytes(buffer, 1);
	if (ret <= 0) {
		goto EXIT_ERR;
	}

	DLOG("%s: INT REG=0x%02x\n", __func__, buffer[0]);

	if (0 != (buffer[0] & 0x80)) {
		ret = 0;
		set_bit(SFH_BIT_PS, &sfh7776->pending_intr);
	}
	if (0 != (buffer[0] & 0x40)) {
		ret = 0;
		set_bit(SFH_BIT_ALS, &sfh7776->pending_intr);
	}

	return ret;

EXIT_ERR:
	ELOG("%s: fail\n", __func__);
	return -1;
}

static void sfh7776_irq_do_work(struct work_struct *work) {
	struct sfh7776_info *sfh7776 = drv_info;
	int ret;
	uint16_t als_adc, ps_adc;

	int als_val, ps_val;
	DLOG("%s\n", __func__);
//datntd	if((ret = sfh7776_check_intr()) < 0)
//	{
//	    ELOG("%s: check intrs error\n", __func__);
//	    goto EXIT_ERR;
//	}

	if ((test_bit(SFH_BIT_PS, &sfh7776->pending_intr) == 1)
			&& (sfh7776->ps_polling == 0)) {
		DLOG("%s: ps interrupt trigged\n", __func__);
		clear_bit(SFH_BIT_PS, &sfh7776->pending_intr);
		sfh7776_read_ps(&ps_adc);
		DLOG("datntddd sfh7776_irq_do_work() ps_adc = 0x%x", ps_adc);
		ps_val = sfh7776_get_ps_value(ps_adc);
		//drv_info->datntd = ps_val;
		sfh7776_ps_report(ps_val);
		//queue_delayed_work(sfh7776->lp_wq, &sfh7776_irq_work,msecs_to_jiffies(sfh7776->ps_delay));//datntd edit ps_polling_work

	}
	if ((test_bit(SFH_BIT_ALS, &sfh7776->pending_intr) == 1)
			&& (sfh7776->als_polling == 0)) {
		DLOG("%s: als interrupt trigged\n", __func__);
		clear_bit(SFH_BIT_ALS, &sfh7776->pending_intr);
		sfh7776_read_als(&als_adc);
		als_val = sfh7776_get_als_value(als_adc);
		sfh7776_als_report(als_val);
	}

	EXIT_ERR: sfh7776_mask_irq(sfh7776->irq, 0);
}

static void als_polling_do_work(struct work_struct *w) {
	struct sfh7776_info *sfh7776 = drv_info;
	uint16_t als_adc;
	int als_val;

	DLOG("%s: als_enable=%d, delay=%d\n", __func__, sfh7776->als_enable, sfh7776->als_delay);
	if (sfh7776->als_enable == 0) {
		DLOG("datntdddd als_polling_do_work() is failed because als is disable");
		return;
	}
	DLOG("datntdddd als_polling_do_work()\n");
	sfh7776_read_als(&als_adc);
	als_val = sfh7776_get_als_value(als_adc);
	sfh7776_als_report(als_val);

	//queue_delayed_work(sfh7776->lp_wq, &als_polling_work, msecs_to_jiffies(sfh7776->als_delay));
	schedule_delayed_work(&sfh7776->ldwork,
			msecs_to_jiffies(sfh7776->als_delay));
}

static void ps_polling_do_work(struct work_struct *w) {
	struct sfh7776_info *sfh7776 = drv_info;
	uint16_t ps_adc;
	int ps_val;

	DLOG("%s: ps_enable=%d, delay=%d\n", __func__, sfh7776->ps_enable, sfh7776->ps_delay);
	if (sfh7776->ps_enable == 0)
		return;

	sfh7776_read_ps(&ps_adc);
	ps_val = sfh7776_get_ps_value(ps_adc);
	sfh7776_ps_report(ps_val);

	//queue_delayed_work(sfh7776->lp_wq, &ps_polling_work, msecs_to_jiffies(sfh7776->ps_delay));
	//datntd
	schedule_delayed_work(&sfh7776->pdwork,
			msecs_to_jiffies(sfh7776->ps_delay));
	//datntd

	DLOG("datntdddd ps_polling_do_work() polling data is okkkkkkkkkkkkkk\n");
}

static irqreturn_t sfh7776_irq_handler(int irq, void *data) {
	struct sfh7776_info *sfh7776 = data;

	DLOG("%s\n", __func__);

	sfh7776_mask_irq(sfh7776->irq, 1);

	queue_work(sfh7776->lp_wq, &sfh7776_irq_work); //datntd edit sfh7776_irq_work

	return IRQ_HANDLED;
}

static int als_open(struct inode *inode, struct file *file) {
	DLOG("%s\n", __func__);

	return 0;
}

static int als_release(struct inode *inode, struct file *file) {

	DLOG("%s\n", __func__);

	return 0;
}
/*
 static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
 {
 int enable,delay;
 uint16_t als_adc;//datntd add
 struct sfh7776_info *sfh7776 = drv_info;

 DLOG("%s cmd %d\n", __func__, _IOC_NR(cmd));

 switch (cmd)
 {
 case SFH_IOCTL_ALS_SET_ENABLED:
 if (get_user(enable, (unsigned long __user *)arg))
 return -EFAULT;
 if (0 == (sfh7776_enable_als(enable? 1:0)))
 {
 sfh7776->als_enable = enable;
 return 0;
 }
 else
 return -EFAULT;

 case SFH_IOCTL_ALS_GET_ENABLED:
 return put_user(sfh7776->als_enable, (unsigned long __user *)arg);
 case SFH_IOCTL_ALS_SET_DELAY:
 if (get_user(delay, (unsigned long __user *)arg))
 return -EFAULT;
 if (100 > delay)
 delay = 100;
 if (1000 < delay)
 delay = 1000;
 sfh7776->als_delay = delay;
 return 0;
 case SFH_IOCTL_ALS_GET_DELAY:
 return put_user(sfh7776->als_delay, (unsigned long __user *)arg);

 case SFH_IOCTL_ALS_GET_DATNTD:
 sfh7776_read_als(&als_adc);
 return put_user(sfh7776_get_als_value(als_adc), (unsigned long __user *)arg);
 default:
 ELOG("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
 return -EINVAL;
 }
 }*/

static const struct file_operations als_fops = {
		.owner = THIS_MODULE,
		.open = als_open,
		.release = als_release,
		.unlocked_ioctl = NULL,
};

static struct miscdevice als_misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = ALS_MISC_NAME,
		.fops = &als_fops
};

static int ps_open(struct inode *inode, struct file *file) {
	DLOG("%s\n", __func__);

	return 0;
}

static int ps_release(struct inode *inode, struct file *file) {
	DLOG("%s\n", __func__);

	return 0;
}

/*static long ps_ioctl(struct file *file, unsigned int cmd,
 unsigned long arg)
 {
 int enable,delay;
 uint16_t ps_adc;//datntd add
 struct sfh7776_info *sfh7776 = drv_info;

 DLOG("%s cmd %d\n", __func__, _IOC_NR(cmd));

 switch (cmd)
 {
 case SFH_IOCTL_PS_SET_ENABLED:
 if (get_user(enable, (unsigned long __user *)arg))
 return -EFAULT;
 if (0 == (sfh7776_enable_ps(enable? 1:0)))
 {
 sfh7776->ps_enable = enable;
 return 0;
 }
 else
 return -EFAULT;
 case SFH_IOCTL_PS_GET_ENABLED:
 return put_user(sfh7776->ps_enable, (unsigned long __user *)arg);
 case SFH_IOCTL_PS_SET_DELAY:
 if (get_user(delay, (unsigned long __user *)arg))
 return -EFAULT;
 if (100 > delay)
 delay = 100;
 if (1000 < delay)
 delay = 1000;
 sfh7776->ps_delay = delay;
 return 0;
 case SFH_IOCTL_PS_GET_DELAY:
 return put_user(sfh7776->ps_delay, (unsigned long __user *)arg);
 case SFH_IOCTL_PS_GET_DATNTD:
 sfh7776_read_ps(&ps_adc);
 return put_user(15.0, (unsigned long __user *)arg);
 default:
 ELOG("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
 return -EINVAL;
 }
 }*/

static const struct file_operations ps_fops = {
		.owner = THIS_MODULE,
		.open =ps_open,
		.release = ps_release,
		.unlocked_ioctl = NULL, //ps_ioctl
};

static struct miscdevice ps_misc = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = PS_MISC_NAME,
		.fops = &ps_fops
};
//datntd
static ssize_t ls_enable_show(struct device* dev, struct device_attribute *attr,
		char *buf) {
	int ret;
	struct sfh7776_info *sfh7776 = drv_info;
	ret = sprintf(buf, "light sensor enable=%d\n", sfh7776->als_enable);
	return ret;
}

static ssize_t ls_enable_store(struct device* dev,
		struct device_attribute *attr, const char* buf, size_t count) {
	int enable;
	struct sfh7776_info *sfh7776 = drv_info;
	sscanf(buf, "%d", &enable);
	printk("%s() enable=%d\n", __func__, enable);
	if (enable) {
		sfh7776_enable_als(1);
	} else {
		sfh7776_enable_als(0);
	}
	return count;
}
static DEVICE_ATTR(als_enable, 0664, ls_enable_show, ls_enable_store);
//datntd

static ssize_t als_adc_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	int ret;
	int idx;
	uint16_t raw_adc_value;
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776_read_als(&raw_adc_value);

	for (idx = 0; idx < ALS_LEVEL_NUM; idx++) {
		if (raw_adc_value < sfh7776->als_level_tabl[idx]) {
			break;
		}
	}

	ret = sprintf(buf, "ADC[0x%03X] => level %d\n",
			sfh7776->als_value_tabl[idx], sfh7776->als_level_tabl[idx]);

	return ret;
}

static DEVICE_ATTR(als_adc, 0664, als_adc_show, NULL);

static ssize_t als_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int ret;
	struct sfh7776_info *sfh7776 = drv_info;

	DLOG("%s: enable=%d, suspend=%d\n", __func__, sfh7776->als_enable, sfh7776->als_suspend);
	ret = sprintf(buf, "Light sensor Enable = %d, suspend = %d\n",
			sfh7776->als_enable, sfh7776->als_suspend);

	return ret;
}

static ssize_t als_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {

	int ls_auto = -1;
	int ret;
	struct sfh7776_info *sfh7776 = drv_info;

	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	DLOG("%s: ls_auto = %d\n", __func__, ls_auto);

	ret = sfh7776_enable_als(ls_auto);
	if (ret == 0)
		sfh7776->als_enable = ls_auto;
	else
		ELOG("%s: fail\n", __func__);

	return count;
}

static DEVICE_ATTR(als_auto, 0664, als_enable_show, als_enable_store);

static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	int ret;
	uint16_t raw_adc_value;
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776_read_ps(&raw_adc_value);

	ret = sprintf(buf, "read ps ADC value =  %d\n", raw_adc_value);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	int ret;
	int ps_en;
	struct sfh7776_info *sfh7776 = drv_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1)
		return -EINVAL;

	ret = sfh7776_enable_ps(ps_en);
	if (ret == 0)
		sfh7776->ps_enable = ps_en;
	else
		ELOG("%s: fail\n", __func__);

	return count;
}

static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

static int als_attach(void) {
	int ret = 0;
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776->als_input_dev = input_allocate_device();
	if (!sfh7776->als_input_dev) {
		ELOG("%s: could not allocate als input device\n", __func__);
		return -ENOMEM;
	}
	sfh7776->als_input_dev->name = "sfh7776_lightsensor"; //ALS_INPUT_NAME;
	set_bit(EV_ABS, sfh7776->als_input_dev->evbit);
	input_set_abs_params(sfh7776->als_input_dev, ABS_MISC, 0, 6554, 0, 0);

	ret = input_register_device(sfh7776->als_input_dev);
	if (ret < 0) {
		ELOG("%s: can not register ls input device\n", __func__);
		goto err_free_als_input_device;
	}

	ret = misc_register(&als_misc);
	if (ret < 0) {
		ELOG("%s: can not register ls misc device\n", __func__);
		goto err_unregister_als_input_device;
	}

	return ret;

err_unregister_als_input_device:
	input_unregister_device(sfh7776->als_input_dev);
err_free_als_input_device:
	input_free_device(sfh7776->als_input_dev);
	return ret;
}

static int ps_attach(void) {
	int ret = 0;
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776->ps_input_dev = input_allocate_device();
	if (!sfh7776->ps_input_dev) {
		ELOG("%s: could not allocate ps input device\n", __func__);
		return -ENOMEM;
	}
	sfh7776->ps_input_dev->name = "sfh7776_proximity"; //PS_INPUT_NAME;
	set_bit(EV_ABS, sfh7776->ps_input_dev->evbit);
	input_set_abs_params(sfh7776->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(sfh7776->ps_input_dev);
	if (ret < 0) {
		ELOG("%s: could not register ps input device\n", __func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&ps_misc);
	if (ret < 0) {
		ELOG("%s: could not register ps misc device\n", __func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(sfh7776->ps_input_dev);
err_free_ps_input_device:
	input_free_device(sfh7776->ps_input_dev);
	return ret;
}

//datntd
static int ps_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec) {
	struct sfh7776_info *sfh7776 = drv_info;

	if ((delay_msec < 1) || (delay_msec > 1000))
		return -EINVAL;

	sfh7776->ps_delay = delay_msec;

	return 0;
}

static int ls_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec) {
	struct sfh7776_info *sfh7776 = drv_info;

	if ((delay_msec < 1) || (delay_msec > 1000))
		return -EINVAL;

	sfh7776->als_delay = delay_msec;
	DLOG("datntdddd ls_poll_delay_set als_delay = %d\n", sfh7776->als_delay);
	return 0;
}
//datntd

static int sfh7776_client_init(void) {
	int ret = 0;
	u8 databuf[2];
	struct sfh7776_info *sfh7776 = drv_info;

//	databuf[0]= SFH7776_SYS_CTRL;
//	ret = sfh7776_read_bytes(databuf, 1);
//	if(ret <= 0)
//	{
//		goto EXIT_ERR;
//	}
//	DLOG("%s: sfh7776 SYS REG=0x%x\n", __func__, databuf[0]);
//	databuf[1] = 0x80 | databuf[0];
//	databuf[0] = SFH7776_SYS_CTRL;
//	ret = sfh7776_write_bytes(databuf, 2);
//	if(ret <= 0)
//	{
//		goto EXIT_ERR;
//	}
//  	msleep(5);

//	databuf[0] = SFH7776_MOD_CTRL;
//	databuf[1] = 0x06; //trang 0x02, den 0x03
//	DLOG("datntddddd init write data 0");
//	ret = sfh7776_write_bytes(databuf, 2);
//	if(ret <= 0)
//	{
//		goto EXIT_ERR;
//	}

	databuf[0] = SFH7776_GAIN_CURRENT_CTRL;
	databuf[1] = ps_current; //trang 0x02, den 0x03
	DLOG("datntddddd init set ps_current = 0x%x is ok",ps_current);
	ret = sfh7776_write_bytes(databuf, 2);
	if (ret <= 0) {
		goto EXIT_ERR;
	}

//	databuf[0] = SFH7776_INT_SETTING;
//	databuf[1] = 0x30;
//	ret = sfh7776_write_bytes(databuf, 2);
//	DLOG("datntddddd init write data 2");
//	if(ret <= 0)
//	{
//		goto EXIT_ERR;
//	}

	if (1 == sfh7776->ps_polling) {
		databuf[0] = SFH7776_PS_TL_LSB;
		databuf[1] = (u8) (sfh7776->ps_lt & 0x00FF);
		ret = sfh7776_write_bytes(databuf, 2);
		if (ret <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = SFH7776_PS_TL_MSB;
		databuf[1] = (u8) ((sfh7776->ps_lt & 0xFF00) >> 8);
		ret = sfh7776_write_bytes(databuf, 2);
		if (ret <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = SFH7776_PS_TH_LSB;
		databuf[1] = (u8) (sfh7776->ps_ht & 0x00FF);
		ret = sfh7776_write_bytes(databuf, 2);
		if (ret <= 0) {
			goto EXIT_ERR;
		}
		databuf[0] = SFH7776_PS_TH_MSB;
		databuf[1] = (u8) ((sfh7776->ps_ht & 0xFF00) >> 8);
		ret = sfh7776_write_bytes(databuf, 2);
		if (ret <= 0) {
			goto EXIT_ERR;
		}

	}

	if ((ret = sfh7776_init_gpio(sfh7776->intr_pin)) != 0) {
		ELOG("%s: init gpio: %d\n", __func__, ret);
		goto EXIT_ERR;
	}

	if ((ret = sfh7776_init_irq(sfh7776->irq)) != 0) {
		ELOG("%s: setup eint: %d\n", __func__, ret);
		goto GPIO_ERR;
	}

//datntd	if((ret = sfh7776_check_intr()) != 0)
//	{
//	    ELOG("%s: check/clear intr: %d\n", __func__, ret);
//	    goto INTR_ERR;
//	}

	return 0;
INTR_ERR:
	sfh7776_free_irq(sfh7776->irq);
GPIO_ERR:
	sfh7776_free_gpio(sfh7776->intr_pin);
EXIT_ERR:
	ELOG("%s: fail: %d\n", __func__, ret);
	return -1;
}

static int sfh7776_create_class_devs(void) {
	int ret = 0;
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776->sfh7776_class = class_create(THIS_MODULE, "sfh_sensors");
	if (IS_ERR(sfh7776->sfh7776_class)) {
		ret = PTR_ERR(sfh7776->sfh7776_class);
		sfh7776->sfh7776_class = NULL;
		goto err_create_class;
	}

	sfh7776->als_dev = device_create(sfh7776->sfh7776_class, NULL, 0, "%s",
			"lightsensor");
	if (unlikely(IS_ERR(sfh7776->als_dev))) {
		ret = PTR_ERR(sfh7776->als_dev);
		sfh7776->als_dev = NULL;
		goto err_create_als_device;
	}
	/* register the attributes */
	ret = device_create_file(sfh7776->als_dev, &dev_attr_als_adc);
	if (ret)
		goto err_create_als_device_file;

	ret = device_create_file(sfh7776->als_dev, &dev_attr_als_enable);
	if (ret)
		goto err_create_als_device_file;

	/* register the attributes */
	ret = device_create_file(sfh7776->als_dev, &dev_attr_als_auto);
	if (ret)
		goto err_create_als_device_file;

	sfh7776->ps_dev = device_create(sfh7776->sfh7776_class, NULL, 0, "%s",
			"proximity");
	if (unlikely(IS_ERR(sfh7776->ps_dev))) {
		ret = PTR_ERR(sfh7776->ps_dev);
		sfh7776->ps_dev = NULL;
		goto err_create_als_device_file;
	}

	/* register the attributes */
	ret = device_create_file(sfh7776->ps_dev, &dev_attr_ps_adc);
	if (ret)
		goto err_create_ps_device;

	return 0;

err_create_ps_device:
	device_unregister(sfh7776->ps_dev);
err_create_als_device_file:
	device_unregister(sfh7776->als_dev);
err_create_als_device:
	class_destroy(sfh7776->sfh7776_class);
err_create_class:
	return ret;
}

static int check_id(void) {
	int ret = -1;
	u8 databuf[2];
	databuf[0] = SFH7776_SYS_CTRL;
	ret = sfh7776_read_bytes(databuf, 1);
	if (ret <= 0) {
		printk("%s() i2c read error\n", __func__);
	}
	printk("%s() chip id=0x%x\n", __func__, databuf[0]);
	if (databuf[0] != CHIP_ID) {
		printk("%s() chip id is not %x\n", __func__, CHIP_ID);
		return -1;
	}
	return 0;
}

static int sfh7776_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	int ret = 0;
	char buffer[2];
	struct sfh7776_info *sfh7776;
	struct sfh7776_platform_data *pdata;
	int irq = 0;

	DLOG("%s\n", __func__);

	sfh7776 = kzalloc(sizeof(struct sfh7776_info), GFP_KERNEL);
	if (!sfh7776)
		return -ENOMEM;

	sfh7776->i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		ELOG("%s: Assign platform_data error!!\n", __func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
//bangdc add
	sfh7776_power_init(sfh7776, 1);
	sfh7776_power_on(sfh7776, 1);
//
	i2c_set_clientdata(client, sfh7776);

	sfh7776->intr_pin = pdata->intr;

	irq = gpio_to_irq(pdata->intr);
	sfh7776->irq = irq;

	sfh7776->pending_intr = 0;
	sfh7776->als_enable = 0;
	sfh7776->als_suspend = 0;
	sfh7776->ps_enable = 0;
	sfh7776->als_polling = pdata->als_polling;
	sfh7776->ps_polling = pdata->ps_polling;
	sfh7776->power = pdata->power;
	//sfh7776->ps_lt = pdata->ps_lt;
	//sfh7776->ps_ht = pdata->ps_ht;
	sfh7776->ps_lt = proximity_low;
	sfh7776->ps_ht = proximity_high;

	memcpy(sfh7776->als_level_tabl, pdata->als_level_table,
			sizeof(pdata->als_level_table));
	memcpy(sfh7776->als_value_tabl, pdata->als_value_table,
			sizeof(pdata->als_value_table));
	sfh7776->als_debounce = 50;
	sfh7776->als_deb_on = 0;
	sfh7776->als_deb_end = 0;
	sfh7776->als_delay = pdata->als_delay;
	sfh7776->ps_debounce = 10;
	sfh7776->ps_deb_on = 0;
	sfh7776->ps_deb_end = 0;
	sfh7776->ps_delay = pdata->ps_delay;

	drv_info = sfh7776;

	sfh7776_power(1);

	sfh7776->lp_wq = create_singlethread_workqueue("sfh7776_wq");
	if (!sfh7776->lp_wq) {
		ELOG("%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_platform_data_null;
	}

	wake_lock_init(&(sfh7776->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	/*check chip id*/
	if (check_id()) {
		ret = -EINVAL;
		goto err_check_id_failed;
	}

	if ((ret = sfh7776_client_init()) < 0) {
		ELOG("%s:  fail\n", __func__);
		goto err_create_singlethread_workqueue;
	}

	if ((ret = als_attach()) < 0) {
		ELOG("%s: als_attach error!!\n", __func__);
		goto err_als_attach;
	}

	if ((ret = ps_attach()) < 0) {
		ELOG("%s: ps_attach error!!\n", __func__);
		goto err_ps_attach;
	}

	if ((ret = sfh7776_create_class_devs()) < 0) {
		ELOG("%s: can't create class devs fail!\n", __func__);
		ret = -ENOMEM;
		goto err_create_class_devs;
	}
	//datntd
	sfh7776->als_cdev = sensors_light_cdev;
	sfh7776->als_cdev.sensors_enable = ls_enable_set;
	sfh7776->als_cdev.sensors_poll_delay = ls_poll_delay_set;
	sfh7776->als_cdev.min_delay = 1000;

	sfh7776->ps_cdev = sensors_proximity_cdev;
	sfh7776->ps_cdev.sensors_enable = ps_enable_set;
	sfh7776->ps_cdev.sensors_poll_delay = ps_poll_delay_set;
	sfh7776->ps_cdev.min_delay = 1000;

	ret = sensors_classdev_register(&client->dev, &sfh7776->als_cdev);
	if (ret)
		goto err_ps_attach;

	ret = sensors_classdev_register(&client->dev, &sfh7776->ps_cdev);
	if (ret)
		goto err_create_class_devs;

	INIT_DELAYED_WORK(&sfh7776->ldwork, als_polling_do_work);
	INIT_DELAYED_WORK(&sfh7776->pdwork, ps_polling_do_work);
	//datntnd
	DLOG("%s: Probe success!\n", __func__);
	return ret;

err_create_class_devs:
	misc_deregister(&ps_misc);
	input_unregister_device(sfh7776->ps_input_dev);
	input_free_device(sfh7776->ps_input_dev);
err_ps_attach:
	misc_deregister(&als_misc);
	input_unregister_device(sfh7776->als_input_dev);
	input_free_device(sfh7776->als_input_dev);
err_als_attach:
	sfh7776_free_irq(sfh7776->irq);
	sfh7776_free_gpio(sfh7776->intr_pin);
err_create_singlethread_workqueue:
	wake_lock_destroy(&(sfh7776->ps_wake_lock));
	destroy_workqueue(sfh7776->lp_wq);
err_platform_data_null:
	err_check_id_failed: kfree(sfh7776);
	return ret;
}

//datntd edit
static int sfh7776_suspend(struct i2c_client *client, pm_message_t mesg) {
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776->als_suspend = 1;

	if (sfh7776->als_enable) {
		if (sfh7776_enable_als(0) < 0)
			goto out;
		sfh7776->als_enable = 1;
	}

	return 0;

out:
	ELOG("%s: fail\n", __func__);
	return -EIO;
}

static int sfh7776_resume(struct i2c_client *client) {
	struct sfh7776_info *sfh7776 = drv_info;

	sfh7776->als_suspend = 0;

	if (sfh7776->als_enable) {
		if (sfh7776_enable_als(1) < 0)
			goto out;
	}

	return 0;
out:
	ELOG("%s: fail\n", __func__);
	return -EIO;
}
//datntd edit

static const struct i2c_device_id sfh7776_i2c_id[] = {
		{ SFH7776_I2C_NAME, 0 },
		{ }
};

static struct i2c_driver sfh7776_driver = {
		.id_table = sfh7776_i2c_id,
		.probe = sfh7776_probe,
		.suspend = sfh7776_suspend,
		.resume = sfh7776_resume,
		.driver = {
				.name = SFH7776_I2C_NAME,
				.owner = THIS_MODULE,
		},
};

static int __init sfh7776_init(void)
{
	DLOG("%s\n", __func__);

return i2c_add_driver(&sfh7776_driver);
}

static void __exit sfh7776_exit(void)
{
	i2c_del_driver(&sfh7776_driver);
}

module_init(sfh7776_init);
module_exit(sfh7776_exit);

MODULE_DESCRIPTION("SFH7776 Driver");
MODULE_AUTHOR("Peter Lee");
MODULE_LICENSE("GPL");
