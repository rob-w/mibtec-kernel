/*
 * Copyright (C) 2011 Simon Budig, <simon.budig@kernelconcepts.de>
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Glyn:
 *    http://www.glyn.com/Products/Displays
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include <linux/gpio.h>

#include <linux/input/edt-ft5x06.h>

#define DRIVER_VERSION "v0.8.4"

#define MAX_SUPPORT_POINTS		5

#define WORK_REGISTER_THRESHOLD   0x00
#define WORK_REGISTER_REPORT_RATE 0x08
#define WORK_REGISTER_GAIN        0x30
#define WORK_REGISTER_OFFSET      0x31
#define WORK_REGISTER_NUM_X       0x33
#define WORK_REGISTER_NUM_Y       0x34

#define M09_REGISTER_THRESHOLD			0x80
#define M09_REGISTER_REPORT_RATE		0xFE	///NOOP
#define M09_REGISTER_GAIN				0x92
#define M09_REGISTER_OFFSET				0x93
#define M09_REGISTER_NUM_X				0x94
#define M09_REGISTER_NUM_Y				0x95

#define NO_REGISTER				0x99
#define CHIP_RESET				0xFF

#define WORK_REGISTER_OPMODE      0x3c
#define FACTORY_REGISTER_OPMODE   0x01

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

#define EDT_NAME_LEN			23
#define EDT_SWITCH_MODE_RETRIES		10
#define EDT_SWITCH_MODE_DELAY		5 /* msec */
#define EDT_RAW_DATA_RETRIES		100
#define EDT_RAW_DATA_DELAY		1 /* msec */

static struct i2c_driver edt_ft5x06_ts_driver;

enum edt_ver {
	M06,
	M09,
};

struct edt_reg_addr {
	int reg_threshold;
	int reg_report_rate;
	int reg_gain;
	int reg_offset;
	int reg_num_x;
	int reg_num_y;
};

struct edt_ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	int irq;
	int irq_pin;
	int reset_pin;
	int num_x;
	int num_y;

#if defined(CONFIG_DEBUG_FS)
	struct dentry *debug_dir;
	u8 *raw_buffer;
	size_t raw_bufsize;
#endif

	struct mutex mutex;
	bool factory_mode;
	int threshold;
	int gain;
	int offset;
	int report_rate;
	int fingers;
	unsigned char rdbuf[20][30];
	int queue_size;
	int queue_ptn;
	int do_chip_reset;
	int invalidate_queue;
	struct timer_list queue_up_timer;
	bool events_valid;
	int filter_cnt;
	char name[EDT_NAME_LEN];
	struct edt_reg_addr reg_addr;
	enum edt_ver version;
};

static int edt_ft5x06_ts_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	return 0;
}

static void edt_ft5x06_report_event( void *dev_id)
{
	struct edt_ft5x06_ts_data *tsdata = dev_id;
	int i, type, x, y, id, tplen, offset;
	bool down;

	switch (tsdata->version){
	case M06:
		offset = 5;
		tplen = 4;
		break;
	case M09:
		offset = 1;
		tplen = 6;
		break;
	default:
		goto out;
	}

	for (i = 0; i < tsdata->fingers; i++) {
		u8 *buf = &tsdata->rdbuf[tsdata->queue_ptn][i * tplen + offset];

		id = (buf[2] >> 4) & 0x0f;
		type = buf[0] >> 6;

		if (type == TOUCH_EVENT_RESERVED)
			continue;

		/* ignore Touch Down and Reserved events */
		if (tsdata->version == M06 && type == TOUCH_EVENT_DOWN)
			continue;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;
		down = (type != TOUCH_EVENT_UP);

		input_report_key(tsdata->input, BTN_TOUCH, down);
		input_report_abs(tsdata->input, ABS_PRESSURE, down);
		if (!down)
			continue;
		input_report_abs(tsdata->input, ABS_X, x);
		input_report_abs(tsdata->input, ABS_Y, y);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);
		input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, id);
	}

	input_sync(tsdata->input);
out:
	return;
}

static bool edt_ft5x06_ts_check_crc(struct edt_ft5x06_ts_data *tsdata,
				    u8 *buf, int buflen)
{
	int i;
	u8 crc = 0;

	for (i = 0; i < buflen - 1; i++)
		crc ^= buf[i];

	if (crc != buf[buflen-1]) {
		dev_err(&tsdata->client->dev,
				    "crc error: 0x%02x expected, got 0x%02x\n",
				    crc, buf[buflen-1]);
		return false;
	}

	return true;
}

static irqreturn_t edt_ft5x06_ts_isr(int irq, void *dev_id)
{
	struct edt_ft5x06_ts_data *tsdata = dev_id;
	int datalen;
	int error;
	u8 cmd;

	switch (tsdata->version){
	case M06:
		cmd = 0xf9;
		datalen = 26;
		break;
	case M09:
		cmd = 0x02;
		datalen = 29;
		break;
	default:
		goto out;
	}

	mutex_lock(&tsdata->mutex);
	error = edt_ft5x06_ts_readwrite(tsdata->client,
					sizeof(cmd), &cmd,
					datalen, tsdata->rdbuf[tsdata->queue_ptn]);

	mutex_unlock(&tsdata->mutex);
	if (error < 0) {
		dev_err(&tsdata->client->dev,
			"Unable to write to i2c touchscreen!\n");
		goto out;
	}

	/* 0 fingers basicly mutes it */
	if (tsdata->fingers == 0)
		goto out;

	/*M09 does not send header or CRC*/
	if (tsdata->version == M06){
		if (tsdata->rdbuf[tsdata->queue_ptn][0] != 0xaa
			|| tsdata->rdbuf[tsdata->queue_ptn][1] != 0xaa
			|| tsdata->rdbuf[tsdata->queue_ptn][2] != 26) {
			dev_err(&tsdata->client->dev, "Unexpected header\n");
			goto out;
		}

		if (!edt_ft5x06_ts_check_crc(dev_id, tsdata->rdbuf[tsdata->queue_ptn], 26))
			goto out;
	}

	if (!tsdata->events_valid)
		tsdata->queue_ptn++;
	else
		edt_ft5x06_report_event(dev_id);

	if (tsdata->queue_ptn == tsdata->queue_size) {
		for (tsdata->queue_ptn = 0; tsdata->queue_ptn < tsdata->queue_size; tsdata->queue_ptn++) {
			mod_timer(&tsdata->queue_up_timer, jiffies + msecs_to_jiffies(tsdata->invalidate_queue));
			edt_ft5x06_report_event(dev_id);
		}
		tsdata->events_valid = 1;
		tsdata->queue_ptn = 0;
	}
	mod_timer(&tsdata->queue_up_timer, jiffies + msecs_to_jiffies(tsdata->invalidate_queue));
out:
	return IRQ_HANDLED;
}


static void edt_ft5x06_queue_up_timer(unsigned long data)
{
	struct edt_ft5x06_ts_data		*tsdata = (void *)data;

	/* count up if filter timeout came but had no valid amount */
	if (!tsdata->events_valid)
		tsdata->filter_cnt++;

	tsdata->events_valid = 0;
	tsdata->queue_ptn = 0;
	/* clear queue again */
	memset(tsdata->rdbuf, 0, sizeof(tsdata->rdbuf));
}

static int edt_ft5x06_register_write(struct edt_ft5x06_ts_data *tsdata,
					 u8 addr, u8 value)
{
	u8 wrbuf[4];
	int ret, cnt;

	switch (tsdata->version){
		case M06:
			wrbuf[0] = tsdata->factory_mode
					? 0xf3 : 0xfc;
			wrbuf[1] = tsdata->factory_mode
					? addr & 0x7f : addr & 0x3f;
			wrbuf[2] = value;
			wrbuf[3] = wrbuf[0] ^ wrbuf[1] ^ wrbuf[2];
			cnt = 4;
			break;
		case M09:
			wrbuf[0] = addr;
			wrbuf[1] = value;
			cnt = 2;
			break;
		default:
			return -EINVAL;
		}

	disable_irq(tsdata->irq);
	ret = edt_ft5x06_ts_readwrite(tsdata->client, cnt,
					wrbuf, 0, NULL);
	enable_irq(tsdata->irq);

	return ret;
}

static int edt_ft5x06_register_read(struct edt_ft5x06_ts_data *tsdata,
					u8 addr)
{
	u8 wrbuf[2], rdbuf[2];
	int ret, cnt;

	switch (tsdata->version){
		case M06:
			cnt = 2;
			wrbuf[0] = tsdata->factory_mode
					? 0xf3 : 0xfc;
			wrbuf[1] = tsdata->factory_mode
					? addr & 0x7f : addr & 0x3f;
			wrbuf[1] |= tsdata->factory_mode
					? 0x80 : 0x40;
			break;
		case M09:
			cnt = 1;
			wrbuf[0] = addr;
			break;
		default:
			return -EINVAL;
	}

	disable_irq(tsdata->irq);
	ret = edt_ft5x06_ts_readwrite(tsdata->client,
				      cnt, wrbuf,
				      cnt, rdbuf);

	enable_irq(tsdata->irq);
	if (tsdata->version == M06) {
		if ((wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]) != rdbuf[1])
			dev_err(&tsdata->client->dev,
				"crc error: 0x%02x expected, got 0x%02x\n",
				(wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]), rdbuf[1]);
	}

	return ret < 0 ? ret : rdbuf[0];
}

static int edt_ft5x06_ts_reset(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr reg_addr = tsdata->reg_addr;

	/* this pulls reset down, enabling the low active reset */
	gpio_direction_output (tsdata->reset_pin, 0);
	gpio_set_value (tsdata->reset_pin, 0);
	/* release reset */
	mdelay (300);
	gpio_set_value (tsdata->reset_pin, 1);
	mdelay(100);

	dev_info(&tsdata->client->dev, "reset performed, i2c transfers might fail now\n");
	edt_ft5x06_register_read(tsdata, reg_addr.reg_threshold);
	edt_ft5x06_register_read(tsdata, reg_addr.reg_threshold);

	edt_ft5x06_register_write(tsdata, reg_addr.reg_gain, tsdata->gain);
	edt_ft5x06_register_write(tsdata, reg_addr.reg_offset, tsdata->offset);
	edt_ft5x06_register_write(tsdata, reg_addr.reg_threshold, tsdata->threshold);
	if (reg_addr.reg_report_rate != M09_REGISTER_REPORT_RATE)
		edt_ft5x06_register_write(tsdata, reg_addr.reg_report_rate, tsdata->report_rate);

	return 0;
}

struct edt_ft5x06_attribute {
	struct device_attribute dattr;
	size_t field_offset;
	u8 limit_low;
	u8 limit_high;
	u8 addr_m06;
	u8 addr_m09;
};

#define EDT_ATTR(_field, _mode, _addr_m06, _addr_m09, _limit_low, _limit_high)		\
	struct edt_ft5x06_attribute edt_ft5x06_attr_##_field = {	\
		.dattr = __ATTR(_field, _mode,				\
				edt_ft5x06_setting_show,		\
				edt_ft5x06_setting_store),		\
		.field_offset =						\
			offsetof(struct edt_ft5x06_ts_data, _field),	\
		.limit_low = _limit_low,				\
		.limit_high = _limit_high,				\
		.addr_m06 = _addr_m06,					\
		.addr_m09 = _addr_m09,					\
	}

static void
edt_ft5x06_ts_get_parameters(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr reg_addr = tsdata->reg_addr;

	tsdata->threshold = edt_ft5x06_register_read(tsdata, reg_addr.reg_threshold);
	tsdata->gain = edt_ft5x06_register_read(tsdata, reg_addr.reg_gain);
	tsdata->offset = edt_ft5x06_register_read(tsdata, reg_addr.reg_offset);
	if (reg_addr.reg_report_rate != M09_REGISTER_REPORT_RATE)
		tsdata->report_rate = edt_ft5x06_register_read(tsdata, reg_addr.reg_report_rate);
	tsdata->num_x = edt_ft5x06_register_read(tsdata, reg_addr.reg_num_x);
	tsdata->num_y = edt_ft5x06_register_read(tsdata, reg_addr.reg_num_y);
}

static ssize_t edt_ft5x06_setting_show(struct device *dev,
					   struct device_attribute *dattr,
					   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)((char *)tsdata + attr->field_offset);
	int val;
	size_t count = 0;
	int error = 0;
	u8 addr;

	mutex_lock(&tsdata->mutex);

	if (tsdata->factory_mode) {
		error = -EIO;
		goto out;
	}

	switch (tsdata->version){
		case M06:
			addr = attr->addr_m06;
			break;
		case M09:
			addr = attr->addr_m09;
			break;
		default:
			addr = 0;
			break;
	}

	if (addr == CHIP_RESET) {
		val = 0;
	} else if(addr == M09_REGISTER_REPORT_RATE) {
		dev_err(&tsdata->client->dev,
				"report rate on M09 not supported\n");
		val = 0;
	} else if (addr != NO_REGISTER) {
		val = edt_ft5x06_register_read(tsdata, addr);
		if (val < 0) {
			error = val;
			dev_err(&tsdata->client->dev,
				"Failed to fetch attribute %s, error %d\n",
				dattr->attr.name, error);
			goto out;
		}

		if (val != *field) {
			dev_warn(&tsdata->client->dev,
				"%s: read (%d) and stored value (%d) differ\n",
				dattr->attr.name, val, *field);
			*field = val;
		}
	} else
		val = *field;

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static ssize_t edt_ft5x06_setting_store(struct device *dev,
					    struct device_attribute *dattr,
					    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)((char *)tsdata + attr->field_offset);
	unsigned int val;
	unsigned int backval = -1;
	int error;
	int try = 0;
	u8 addr = 0;

	mutex_lock(&tsdata->mutex);

	if (tsdata->factory_mode) {
		error = -EIO;
		goto out;
	}

	if (sscanf(buf, "%u", &val) != 1) {
		error = -ERANGE;
		goto out;
	}

	if (val < attr->limit_low || val > attr->limit_high) {
		error = -ERANGE;
		goto out;
	}

	switch (tsdata->version){
		case M06:
			addr = attr->addr_m06;
			break;
		case M09:
			addr = attr->addr_m09;
			break;
		default:
			addr = 0;
			break;
	}

	if (addr == CHIP_RESET) {
		error = edt_ft5x06_ts_reset(tsdata);
		if (error) {
			dev_err(&tsdata->client->dev,
				"Failed to reset\n");
			goto out;
		}
	} else if(addr == M09_REGISTER_REPORT_RATE) {
		dev_err(&tsdata->client->dev,
				"report rate on M09 not supported\n");
	} else if (addr != NO_REGISTER) {
		do {
			error = edt_ft5x06_register_write(tsdata, addr, val);
			if (error) {
				dev_err(&tsdata->client->dev,
					"Failed to update attribute %s, error: %d\n",
					dattr->attr.name, error);
				goto out;
			}
			backval = edt_ft5x06_register_read(tsdata, addr);
			edt_ft5x06_ts_get_parameters(tsdata);
			try++;
		} while (backval != val || try > 5);
	}

	*field = val;

out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}


static ssize_t edt_ft5x06_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct edt_ft5x06_ts_data *tsdata = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", tsdata->factory_mode ? 1 : 0);
}

static ssize_t edt_ft5x06_mode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct edt_ft5x06_ts_data *tsdata = dev_get_drvdata(dev);
	struct edt_reg_addr *reg_addr = &(tsdata->reg_addr);
	int i, ret = 0;
	unsigned int mode;

	if (sscanf(buf, "%u", &mode) != 1 || (mode | 1) != 1) {
		dev_err(dev, "Invalid value for operation mode\n");
		return -EINVAL;
	}

	/* no change, return without doing anything */
	if (mode == tsdata->factory_mode)
		return count;

	mutex_lock(&tsdata->mutex);
	if (!tsdata->factory_mode) { /* switch to factory mode */
		disable_irq(tsdata->irq);
		/* mode register is 0x3c when in the work mode */
		ret = edt_ft5x06_register_write(tsdata,
						    WORK_REGISTER_OPMODE, 0x03);
		if (ret < 0) {
			dev_err(dev, "failed to switch to factory mode (%d)\n",
				ret);
		} else {
			tsdata->factory_mode = 1;
			for (i = 0; i < 10; i++) {
				mdelay(5);
				/* mode register is 0x01 when in factory mode */
				ret = edt_ft5x06_register_read(tsdata, FACTORY_REGISTER_OPMODE);
				if (ret == 0x03)
					break;
			}
			if (i == 10)
				dev_err(dev,
					"not in factory mode after %dms.\n",
					i*5);
		}
	} else {  /* switch to work mode */
		/* mode register is 0x01 when in the factory mode */
		ret = edt_ft5x06_register_write(tsdata,
						    FACTORY_REGISTER_OPMODE,
						    0x01);
		if (ret < 0) {
			dev_err(dev, "failed to switch to work mode (%d)\n",
				ret);
		} else {
			tsdata->factory_mode = 0;
			for (i = 0; i < 10; i++) {
				mdelay(5);
				/* mode register is 0x01 when in factory mode */
				ret = edt_ft5x06_register_read(tsdata, WORK_REGISTER_OPMODE);
				if (ret == 0x01)
					break;
			}
			if (i == 10)
				dev_err(dev, "not in work mode after %dms.\n",
					i*5);

			/* restore parameters */
			edt_ft5x06_register_write(tsdata,
						      reg_addr->reg_threshold ,
						      tsdata->threshold);
			edt_ft5x06_register_write(tsdata,
						      reg_addr->reg_gain,
						      tsdata->gain);
			edt_ft5x06_register_write(tsdata,
						      reg_addr->reg_offset,
						      tsdata->offset);
			if (reg_addr->reg_report_rate)
				edt_ft5x06_register_write(tsdata,
						      reg_addr->reg_report_rate,
						      tsdata->report_rate);

			enable_irq(tsdata->irq);
		}
	}

	mutex_unlock(&tsdata->mutex);
	return count;
}


static ssize_t edt_ft5x06_raw_data_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct edt_ft5x06_ts_data *tsdata = dev_get_drvdata(dev);
	int i, ret;
	char *ptr, wrbuf[3];

	if (!tsdata->factory_mode) {
		dev_err(dev, "raw data not available in work mode\n");
		return -EIO;
	}

	mutex_lock(&tsdata->mutex);
	ret = edt_ft5x06_register_write(tsdata, 0x08, 0x01);
	for (i = 0; i < 100; i++) {
		ret = edt_ft5x06_register_read(tsdata, 0x08);
		if (ret < 1)
			break;
		udelay(1000);
	}

	if (i == 100 || ret < 0) {
		dev_err(dev, "waiting time exceeded or error: %d\n", ret);
		mutex_unlock(&tsdata->mutex);
		return ret < 0 ? ret : -ETIMEDOUT;
	}

	ptr = buf;
	wrbuf[0] = 0xf5;
	wrbuf[1] = 0x0e;
	for (i = 0; i <= tsdata->num_x; i++) {
		wrbuf[2] = i;
		ret = edt_ft5x06_ts_readwrite(tsdata->client,
					       3, wrbuf,
					       tsdata->num_y * 2, ptr);
		if (ret < 0) {
			mutex_unlock(&tsdata->mutex);
			return ret;
		}

		ptr += tsdata->num_y * 2;
	}

	mutex_unlock(&tsdata->mutex);
	return ptr - buf;
}

static EDT_ATTR(gain, S_IWUSR | S_IRUGO, WORK_REGISTER_GAIN,
		M09_REGISTER_GAIN, 0, 31);
static EDT_ATTR(offset, S_IWUSR | S_IRUGO, WORK_REGISTER_OFFSET,
		M09_REGISTER_OFFSET, 0, 31);
static EDT_ATTR(threshold, S_IWUSR | S_IRUGO,
		WORK_REGISTER_THRESHOLD, M09_REGISTER_THRESHOLD, 20, 80);
static EDT_ATTR(report_rate, S_IWUSR | S_IRUGO,
		WORK_REGISTER_REPORT_RATE, M09_REGISTER_REPORT_RATE, 3, 14);

static EDT_ATTR(fingers, S_IWUSR | S_IRUGO,
		NO_REGISTER, NO_REGISTER, 0, 5);

static EDT_ATTR(queue_size, S_IWUSR | S_IRUGO,
		NO_REGISTER, NO_REGISTER, 5, 20);

static EDT_ATTR(invalidate_queue, S_IWUSR | S_IRUGO,
		NO_REGISTER, NO_REGISTER, 28, 100);

//static EDT_ATTR(mode, S_IWUSR | S_IRUGO,
//		NO_REGISTER, NO_REGISTER, 0, 5);

//static EDT_ATTR(raw_data, S_IWUSR | S_IRUGO,
//		NO_REGISTER, NO_REGISTER, 0, 5);

static EDT_ATTR(filter_cnt, S_IWUSR | S_IRUGO,
		NO_REGISTER, NO_REGISTER, 0, 128);

static EDT_ATTR(do_chip_reset, S_IWUSR | S_IRUGO,
		CHIP_RESET, CHIP_RESET, 0, 1);

static struct attribute *edt_ft5x06_attrs[] = {
	&edt_ft5x06_attr_gain.dattr.attr,
	&edt_ft5x06_attr_offset.dattr.attr,
	&edt_ft5x06_attr_threshold.dattr.attr,
	&edt_ft5x06_attr_report_rate.dattr.attr,
	&edt_ft5x06_attr_fingers.dattr.attr,
	&edt_ft5x06_attr_queue_size.dattr.attr,
	&edt_ft5x06_attr_invalidate_queue.dattr.attr,
//	&edt_ft5x06_attr_mode.dattr.attr,
//	&edt_ft5x06_attr_raw_data.dattr.attr,
	&edt_ft5x06_attr_filter_cnt.dattr.attr,
	&edt_ft5x06_attr_do_chip_reset.dattr.attr,
	NULL
};

static const struct attribute_group edt_ft5x06_attr_group = {
	.attrs = edt_ft5x06_attrs,
};

static int edt_ft5x06_ts_identify(struct i2c_client *client,
					struct edt_ft5x06_ts_data *tsdata,
					char *fw_version)
{
	u8 rdbuf[EDT_NAME_LEN];
	char *p;
	int error;
	char *model_name = tsdata->name;

	//if we get less than EDT_NAME_LEN, we dont want  have garbage in there
	memset(rdbuf, 0, sizeof(rdbuf));

	error = edt_ft5x06_ts_readwrite(client, 1, "\xbb",
					EDT_NAME_LEN - 1, rdbuf);

	error = edt_ft5x06_ts_readwrite(client, 1, "\xbb",
					EDT_NAME_LEN - 1, rdbuf);
	if (error)
		return error;

	/* if we find something consistent, stay with that assumption
	 * at least M09 wont send 3 bytes here…
	 */

	if (!(strnicmp(rdbuf + 1, "EP0", 3))){
		dev_info(&tsdata->client->dev, "MO6 %s\n", rdbuf);

		tsdata->version = M06;
		/* remove last '$' end marker */
		rdbuf[EDT_NAME_LEN - 1] = '\0';
		if (rdbuf[EDT_NAME_LEN - 2] == '$')
			rdbuf[EDT_NAME_LEN - 2] = '\0';
			/* look for Model/Version separator */
		p = strchr(rdbuf, '*');
		if (p)
			*p++ = '\0';
		strlcpy(model_name, rdbuf + 1, EDT_NAME_LEN);
		strlcpy(fw_version, p ? p : "", EDT_NAME_LEN);
	} else {
		/*since there are only two versions around… (M06, M09) */
		tsdata->version = M09;

		error = edt_ft5x06_ts_readwrite(client, 1, "\xA6",
				2, rdbuf);

		if (error)
			return error;
		sprintf(fw_version, "%d", rdbuf[0]);

		error = edt_ft5x06_ts_readwrite(client, 1, "\xA8",
				1, rdbuf);
		if (error)
			return error;
		snprintf(model_name, EDT_NAME_LEN, "EP0%i%i0M09", rdbuf[0]>>4,
				rdbuf[0] & 0x0F);
	}
	return 0;
}

static void
edt_ft5x06_ts_set_regs(struct edt_ft5x06_ts_data *tsdata)
{
	struct edt_reg_addr *reg_addr = &(tsdata->reg_addr);
	switch (tsdata->version){
		case M06:
			reg_addr->reg_threshold = WORK_REGISTER_THRESHOLD;
			reg_addr->reg_report_rate = WORK_REGISTER_REPORT_RATE;
			reg_addr->reg_gain = WORK_REGISTER_GAIN;
			reg_addr->reg_offset = WORK_REGISTER_OFFSET;
			reg_addr->reg_num_x = WORK_REGISTER_NUM_X;
			reg_addr->reg_num_y = WORK_REGISTER_NUM_Y;
			break;
		case M09:
			reg_addr->reg_threshold = M09_REGISTER_THRESHOLD;
			reg_addr->reg_report_rate = M09_REGISTER_REPORT_RATE;
			reg_addr->reg_gain = M09_REGISTER_GAIN;
			reg_addr->reg_offset = M09_REGISTER_OFFSET;
			reg_addr->reg_num_x = M09_REGISTER_NUM_X;
			reg_addr->reg_num_y = M09_REGISTER_NUM_Y;
			break;
		default:
			/*just in case…*/
			reg_addr->reg_threshold = 0;
			reg_addr->reg_report_rate = 0;
			reg_addr->reg_gain = 0;
			reg_addr->reg_offset = 0;
			reg_addr->reg_num_x = 0;
			reg_addr->reg_num_y = 0;
			break;
	}
}

static int edt_ft5x06_ts_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct edt_ft5x06_ts_data *tsdata;
	struct input_dev *input;
	int error;
	int i;
	char fw_version[EDT_NAME_LEN];

	dev_dbg(&client->dev, "probing for EDT FT5x06 I2C\n");

	if (!client->irq) {
		dev_dbg (&client->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data!\n");
		dev_set_drvdata(&client->dev, NULL);
		return -ENOMEM;
	}

	dev_set_drvdata(&client->dev, tsdata);
	tsdata->client = client;

	tsdata->reset_pin = ((struct edt_ft5x06_platform_data *) client->dev.platform_data)->reset_pin;
	mutex_init(&tsdata->mutex);

	error = gpio_request(tsdata->reset_pin, NULL);
	if (error < 0) {
		dev_err (&client->dev,
		         "Failed to request GPIO %d as reset pin, error %d\n",
		         tsdata->reset_pin, error);
		error = -ENOMEM;
		goto err_free_tsdata;
	}

	/* this pulls reset down, enabling the low active reset */
	if (gpio_direction_output (tsdata->reset_pin, 0) < 0) {
		dev_info (&client->dev, "switching to output failed\n");
		error = -ENOMEM;
		goto err_free_reset_pin;
	}

	/* request IRQ pin */
	tsdata->irq = client->irq;
	tsdata->irq_pin = irq_to_gpio (tsdata->irq);

	error = gpio_request(tsdata->irq_pin, NULL);
	if (error < 0) {
		dev_err (&client->dev,
			"Failed to request GPIO %d for IRQ %d, error %d\n",
			tsdata->irq_pin, tsdata->irq, error);
		error = -ENOMEM;
		goto err_free_reset_pin;
	}
	gpio_direction_input(tsdata->irq_pin);

	/* release reset */
	mdelay (50);
	gpio_set_value (tsdata->reset_pin, 1);
	mdelay (100);

	mutex_lock(&tsdata->mutex);

	tsdata->factory_mode = 0;

	dev_info(&tsdata->client->dev, "Version %s\n", DRIVER_VERSION);

	error = edt_ft5x06_ts_identify(client, tsdata, fw_version);
	if (error) {
		dev_err(&client->dev, "touchscreen probe failed\n");
		goto err_free_irq_pin;
	}

	edt_ft5x06_ts_set_regs(tsdata);
	edt_ft5x06_ts_get_parameters(tsdata);

	/* default to 5 Fingers */
	tsdata->fingers = 5;

	/* default queue size to 8 */
	tsdata->queue_size = 8;

	/* default queue timeout to 30ms */
	tsdata->invalidate_queue = 30;

	mutex_unlock(&tsdata->mutex);

	dev_info(&client->dev,
			 "Model \"%s\", Rev. \"%s\", %dx%d sensors\n",
			tsdata->name, fw_version, tsdata->num_x, tsdata->num_y);
	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		goto err_free_irq_pin;
	}

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, tsdata->num_x * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, tsdata->num_y * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X,
			     0, tsdata->num_x * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			     0, tsdata->num_y * 64 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 15, 0, 0);

	input->name = tsdata->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input_set_drvdata(input, tsdata);

	tsdata->input = input;

	if ((error = input_register_device(input)))
		goto err_free_input_device;

	for (i = 0; i < tsdata->queue_size; i++)
		memset(tsdata->rdbuf, 0, sizeof(tsdata->rdbuf));

	setup_timer(&tsdata->queue_up_timer, edt_ft5x06_queue_up_timer,
		    (unsigned long)tsdata);

	tsdata->events_valid = 0;
	tsdata->queue_ptn = 0;

	if (request_threaded_irq(tsdata->irq, NULL, edt_ft5x06_ts_isr,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				 client->name, tsdata)) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input = NULL;
		error = -ENOMEM;
		goto err_unregister_device;
	}

	error = sysfs_create_group(&client->dev.kobj,
				   &edt_ft5x06_attr_group);
	if (error)
		goto err_free_irq;

	device_init_wakeup(&client->dev, 1);

	dev_info(&tsdata->client->dev,
		"EDT FT5x06 initialized: Driver %s IRQ pin %d, Reset pin %d.\n",
		DRIVER_VERSION, tsdata->irq_pin, tsdata->reset_pin);

	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
err_unregister_device:
	input_unregister_device (input);
err_free_input_device:
	kfree(input->name);
	input_free_device (input);
err_free_irq_pin:
	gpio_free(tsdata->irq_pin);
err_free_reset_pin:
	gpio_free(tsdata->reset_pin);
err_free_tsdata:
	kfree(tsdata);
	return error;
}

static int edt_ft5x06_ts_remove(struct i2c_client *client)
{
	struct edt_ft5x06_ts_data *tsdata = dev_get_drvdata(&client->dev);

	sysfs_remove_group(&client->dev.kobj, &edt_ft5x06_attr_group);

	free_irq(client->irq, tsdata);
	del_timer_sync(&tsdata->queue_up_timer);
	input_unregister_device(tsdata->input);
	kfree (tsdata->input->name);
	input_free_device(tsdata->input);
	gpio_free(tsdata->irq_pin);
	gpio_free(tsdata->reset_pin);
	kfree(tsdata);

	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static int edt_ft5x06_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct edt_ft5x06_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(tsdata->irq);

	return 0;
}

static int edt_ft5x06_ts_resume(struct i2c_client *client)
{
	struct edt_ft5x06_ts_data *tsdata = dev_get_drvdata(&client->dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(tsdata->irq);

	return 0;
}

static const struct i2c_device_id edt_ft5x06_ts_id[] =
{
	{ "edt-ft5x06", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, edt_ft5x06_ts_id);

static struct i2c_driver edt_ft5x06_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "edt_ft5x06",
	},
	.id_table = edt_ft5x06_ts_id,
	.probe    = edt_ft5x06_ts_probe,
	.remove   = edt_ft5x06_ts_remove,
	.suspend  = edt_ft5x06_ts_suspend,
	.resume   = edt_ft5x06_ts_resume,
};

static int __init edt_ft5x06_ts_init(void)
{
	return i2c_add_driver(&edt_ft5x06_ts_driver);
}
module_init(edt_ft5x06_ts_init);

static void __exit edt_ft5x06_ts_exit(void)
{
	i2c_del_driver(&edt_ft5x06_ts_driver);
}
module_exit(edt_ft5x06_ts_exit);

MODULE_AUTHOR("Simon Budig <simon.budig@kernelconcepts.de>");
MODULE_DESCRIPTION("EDT FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
