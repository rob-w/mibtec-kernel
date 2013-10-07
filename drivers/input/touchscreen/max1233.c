

/*
 * File:        drivers/input/touchscreen/max1233.c
 *
 * Based on: 	ads7846.c, ad7877.c
 *
 *		Copyright (C) 2006-2008 Michael Hennerich, Analog Devices Inc.
 *
 * Author:	Michael Hennerich, Analog Devices Inc.
 *
 * Created:	Nov, 10th 2006
 * Description:	MAX1233 based touchscreen, sensor (ADCs), DAC and GPIO driver
 *
 *
 * Modified:
 *
 * Bugs:        Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * History:
 * Copyright (c) 2005 David Brownell
 * Copyright (c) 2006 Nokia Corporation
 * Various changes: Imre Deak <imre.deak@nokia.com>
 *
 * Using code from:
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 */


#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/max1233.h>
//#include <linux/fb.h>
//#include <linux/backlight.h>
#include <asm/irq.h>

#define	TS_PEN_UP_TIMEOUT	msecs_to_jiffies(50)
#define KB_POLL_TIMER		msecs_to_jiffies(50)

#define TS_POLL_PERIOD  (5 * 1000000) 

/*--------------------------------------------------------------------------*/

#define MAX_SPI_FREQ_HZ			20000000
#define	MAX_12BIT			((1<<12)-1)


/*----------------REGISTER NAMES AND IDS ----------------------------------------------------------*/

#define MAX1233_PAGE0			0
#define MAX1233_PAGE1			1


/*---------------- PAGE 1 ----------------------------------------------------------*/

#define MAX1233_REG_X			0
#define MAX1233_REG_Y			1
#define MAX1233_REG_Z1			2
#define MAX1233_REG_Z2			3
#define MAX1233_REG_KPD			4
#define MAX1233_REG_BAT1		5
#define MAX1233_REG_BAT2		6
#define MAX1233_REG_AUX1		7
#define MAX1233_REG_AUX2		8
#define MAX1233_REG_TEMP1		9
#define MAX1233_REG_TEMP2		10
#define MAX1233_REG_DAC			11

#define MAX1233_REG_GPIO		15
#define MAX1233_REG_KPData1		16
#define MAX1233_REG_KPData2		17

/*---------------- PAGE 2 ----------------------------------------------------------*/

#define MAX1233_REG_ADCctrl		0
#define MAX1233_REG_KEYctrl		1
#define MAX1233_REG_DACctrl		2

#define MAX1233_REG_GPIO_PULLUPctrl	14
#define MAX1233_REG_GPIOctrl		15
#define MAX1233_REG_KPKeyMask		16
#define MAX1233_REG_KPColumnMask	17


/*----------------ADC Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_PENSTS(x)			((x & 0x1) << 15)	/* Touch detected		use TSOP */
#define MAX1233_ADSTS(x)			((x & 0x1) << 14)	/* conv.data avail		use TSOP */
/* both above need to be combined for writing selecting operation mode */

#define MAX1233_ADCSCANSEL(x)			((x & 0xF) << 10)	/* 		use function table */
#define MAX1233_RES(x)				((x & 0x3) << 8)	/* 		8/8/10/12 */
#define MAX1233_AVG(x)				((x & 0x3) << 6)	/* 		no/4/8/16 */
#define MAX1233_CNR(x)				((x & 0x3) << 4)	/* 		3.5/3.5/10/100 */
#define MAX1233_STLTIME(x)			((x & 0x7) << 1)	/* 		0/0.1/0.5/1/5/10/50/100 */
#define MAX1233_REFV(x)				((x & 0x1) << 0)	/* 		1V/2.5V */



#define MAX1233_TSOP_SCAN_ONE 		(MAX1233_ADSTS(0) | MAX1233_PENSTS(0))
#define MAX1233_TSOP_DET_SCAN_IRQ 	(MAX1233_ADSTS(0) | MAX1233_PENSTS(1))
#define MAX1233_TSOP_DET_IRQ 		(MAX1233_ADSTS(1) | MAX1233_PENSTS(0))
#define MAX1233_TSOP_DISABLE_DETECT 	(MAX1233_ADSTS(1) | MAX1233_PENSTS(1))

#define MAX1233_ADCSCAN_CONFRES		0
#define MAX1233_ADCSCAN_MEAS_XY		1
#define MAX1233_ADCSCAN_MEAS_XYZ	2
#define MAX1233_ADCSCAN_MEAS_X		3
#define MAX1233_ADCSCAN_MEAS_Y		4
#define MAX1233_ADCSCAN_MEAS_Z		5
#define MAX1233_ADCSCAN_MEAS_BAT1	6
#define MAX1233_ADCSCAN_MEAS_BAT2	7
#define MAX1233_ADCSCAN_MEAS_AUX1	8
#define MAX1233_ADCSCAN_MEAS_AUX2	9
#define MAX1233_ADCSCAN_MEAS_TEMP_SE	10
#define MAX1233_ADCSCAN_MEAS_BATAUX	11
#define MAX1233_ADCSCAN_MEAS_TEMP_DIFF	12
#define MAX1233_ADCSCAN_YDRIVES_ON	13
#define MAX1233_ADCSCAN_XDRIVES_ON	14
#define MAX1233_ADCSCAN_YXDRIVES_ON	15


/*----------------DAC Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_DAC_ENABLE(x)			((x & 0x1) << 15)	/* 		State of DAC */

#define BL_MAX_INTENSITY			255
#define BL_DEFAULT_INTENSITY		255

/*----------------KEYPAD Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_KEYSTS1(x)			((x & 0x1) << 15)	/* PRESS detected		use KEYOP */
#define MAX1233_KEYSTS0(x)			((x & 0x1) << 14)	/* conv.data avail		use KEYOP */
/* both above need to be combined for writing selecting operation mode */

#define MAX1233_DBNTIME(x)			((x & 0x7) << 11)	/* 2/10/20/50/60/80/100/120 */
#define MAX1233_HLDTIME(x)			((x & 0x7) << 8)	/* 100us/1/2/3/4/5/6/7 DEBOUNCE TIMES */



#define MAX1233_KEYOP_SCAN_ONE 		(MAX1233_KEYSTS1(0) | MAX1233_KEYSTS0(0))
#define MAX1233_KEYOP_DET_SCAN_IRQ 	(MAX1233_KEYSTS1(0) | MAX1233_KEYSTS0(1))
#define MAX1233_KEYOP_DET_IRQ 		(MAX1233_KEYSTS1(1) | MAX1233_KEYSTS0(0))
#define MAX1233_KEYOP_DISABLE_DETECT 	(MAX1233_KEYSTS1(1) | MAX1233_KEYSTS0(1))


/*----------------KEYPAD MASK Control Register  ----------------------------------------------------------*/
							/* READ				WRITE */

#define MAX1233_KEY15(x)			((x & 0x1) << 15)	/* 		detect KEY(4,4) */
#define MAX1233_KEY14(x)			((x & 0x1) << 14)	/* 		detect KEY(3,4) */
#define MAX1233_KEY13(x)			((x & 0x1) << 13)	/* 		detect KEY(2,4) */
#define MAX1233_KEY12(x)			((x & 0x1) << 12)	/* 		detect KEY(1,4) */
#define MAX1233_KEY11(x)			((x & 0x1) << 11)	/* 		detect KEY(4,3) */
#define MAX1233_KEY10(x)			((x & 0x1) << 10)	/* 		detect KEY(3,3) */
#define MAX1233_KEY9(x)				((x & 0x1) << 9)	/* 		detect KEY(2,3) */
#define MAX1233_KEY8(x)				((x & 0x1) << 8)	/* 		detect KEY(1,3) */
#define MAX1233_KEY7(x)				((x & 0x1) << 7)	/* 		detect KEY(4,2) */
#define MAX1233_KEY6(x)				((x & 0x1) << 6)	/* 		detect KEY(3,2) */
#define MAX1233_KEY5(x)				((x & 0x1) << 5)	/* 		detect KEY(2,2) */
#define MAX1233_KEY4(x)				((x & 0x1) << 4)	/* 		detect KEY(1,2) */
#define MAX1233_KEY3(x)				((x & 0x1) << 3)	/* 		detect KEY(4,1) */
#define MAX1233_KEY2(x)				((x & 0x1) << 2)	/* 		detect KEY(3,1) */
#define MAX1233_KEY1(x)				((x & 0x1) << 1)	/* 		detect KEY(2,1) */
#define MAX1233_KEY0(x)				((x & 0x1) << 0)	/* 		detect KEY(1,1) */


/*----------------KEYPAD COLUMN MASK Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_COLMASK4(x)			((x & 0x1) << 15)	/* 		detect KEYS(*,4) */
#define MAX1233_COLMASK3(x)			((x & 0x1) << 14)	/* 		detect KEYS(*,3) */
#define MAX1233_COLMASK2(x)			((x & 0x1) << 13)	/* 		detect KEYS(*,2) */
#define MAX1233_COLMASK1(x)			((x & 0x1) << 12)	/* 		detect KEYS(*,1) */


/*----------------GPIO Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_GP7(x)			((x & 0x1) << 15)	/* 		use as GPIO /KEY */
#define MAX1233_GP6(x)			((x & 0x1) << 14)	/* 		use as GPIO /KEY */
#define MAX1233_GP5(x)			((x & 0x1) << 13)	/* 		use as GPIO /KEY */
#define MAX1233_GP4(x)			((x & 0x1) << 12)	/* 		use as GPIO /KEY */
#define MAX1233_GP3(x)			((x & 0x1) << 11)	/* 		use as GPIO /KEY */
#define MAX1233_GP2(x)			((x & 0x1) << 10)	/* 		use as GPIO /KEY */
#define MAX1233_GP1(x)			((x & 0x1) << 9)	/* 		use as GPIO /KEY */
#define MAX1233_GP0(x)			((x & 0x1) << 8)	/* 		use as GPIO /KEY */

/*----------------GPIO PULLUP Control Register  ----------------------------------------------------------*/
									/* READ				WRITE */

#define MAX1233_PU7(x)			((x & 0x1) << 15)	/* 		disable PULLUP  */
#define MAX1233_PU6(x)			((x & 0x1) << 14)	/* 		disable PULLUP */
#define MAX1233_PU5(x)			((x & 0x1) << 13)	/* 		disable PULLUP */
#define MAX1233_PU4(x)			((x & 0x1) << 12)	/* 		disable PULLUP */
#define MAX1233_PU3(x)			((x & 0x1) << 11)	/* 		disable PULLUP */
#define MAX1233_PU2(x)			((x & 0x1) << 10)	/* 		disable PULLUP */
#define MAX1233_PU1(x)			((x & 0x1) << 9)	/* 		disable PULLUP */
#define MAX1233_PU0(x)			((x & 0x1) << 8)	/* 		disable PULLUP */


#define KEY_LEFT_OFFSET		12
#define KEY_MID_OFFSET		14
#define KEY_RIGHT_OFFSET	8
#define HAVE_KEYS		3

/* HELPER */

#define MAX1233_PAGEADDR(page, addr) (((0x1 & page)<<6) | (0x003F & addr)) /* use this to generate a register adress from page and reg ddr*/
#define MAX1233_RDADD(x)		(0x8000 | (0x7FFF & (x))) 	/* use with PAGEADDR , MAX1233_RDADD(MAX1233_PAGEADDR(page, addr))*/
#define MAX1233_WRADD(x)		(0x0000 | (0x7FFF & (x)))	/* use with PAGEADDR */

//#define MAX1233_RDADD(x)		((0x7FFF & (x)) << 8 | 0x0080)
//#define MAX1233_WRADD(x)		((0x7FFF & (x)) << 8 | 0x0000)


/*for reading a whole part in PAGE0*/
enum {
	MAX1233_SEQ_XPOS  = 0,
	MAX1233_SEQ_YPOS  = 1,
	MAX1233_SEQ_Z1    = 2,
	MAX1233_SEQ_Z2    = 3,
	MAX1233_SEQ_KPD   = 4,
	MAX1233_SEQ_BAT1  = 5,
	MAX1233_SEQ_BAT2  = 6,
	MAX1233_SEQ_AUX1  = 7,
	MAX1233_SEQ_AUX2  = 8,
	MAX1233_SEQ_TEMP1 = 9,
	MAX1233_SEQ_TEMP2 = 10,
	MAX1233_SEQ_DAC   = 11,
	MAX1233_NR_SENSE  = 12,
};


/*
 * Non-touchscreen sensors only use single-ended conversions.
 */

/*this is only for  interaction of one word back and forth*/
struct ser_req {
	u16			txbuf[2];
	u16			rxbuf[2];
	struct spi_message	msg;
	struct spi_transfer	xfer[1];
};

struct snd_data {
	u16 dat[2];
	};

struct snd_data		send_data[6];
struct snd_data		ksend_data[2];

struct _kb_key{
	int key;
	int down;
	int offset;
	};

struct max1233 {
	struct input_dev	*input;
	struct input_dev	*kb_input;

	char			phys[32];
	char 			kb_phys[32];

	struct spi_device	*spi;
	u16			model;
	u16			vref_delay_usecs;
	u16			x_plate_ohms;
	u16			y_plate_ohms;
	u16			kp_irq;
	u16			xmin;
	u16			xmax;
	u16			ymin;
	u16			ymax;
	u16			pressure_max;

	u16			cmd_crtl1;
	u16			cmd_crtl2;
	u16			cmd_dummy;
	u16			dac;

	u8			stopacq_polarity;
	u8			first_conversion_delay;
	u8			acquisition_time;
	u8			averaging;
	u8			pen_down_acc_interval;

	
	u16 conversion_send[MAX1233_NR_SENSE];
	u32 conversion_data[MAX1233_NR_SENSE];
	u16 kb_data[2];

	struct spi_transfer	xfer[6];
	struct spi_transfer	kxfer[6];
	struct spi_message	msg;
	struct spi_message	kb_msg;
	
	struct _kb_key		key[4];

	int intr_flag;
	int spi_active;
	int spi_kb_active;
	int irq_cnt;
	struct mutex		lock;
	struct timer_list	ts_timer;		/* P: lock */
	struct timer_list	kb_timer;

//	struct hrtimer		timer;
//	struct hrtimer		kb_timer;


	unsigned		pendown:1;	/* P: lock */
	unsigned		pending:1;	/* P: lock */

	unsigned		irq_disabled:1;	/* P: lock */
	unsigned		disabled:1;
	unsigned		gpio3:1;
	unsigned		gpio4:1;

};

static void max1233_enable(struct max1233 *ts);
static void max1233_disable(struct max1233 *ts);
static void max1233_callback_kp(void *handle);

int xold, yold;

static int device_suspended(struct device *dev)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	return dev->power.power_state.event != PM_EVENT_ON || ts->disabled;
}


/* this has been worked on and changed from 7877 */
/* reg has to carry complete page and adress description read flag will be set staticly */
/* use MAX1233_RDADD to generate */
static int max1233_read(struct device *dev, u16 reg, u16 *val)
{
	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	int			status;

	if (!req)
		return -ENOMEM;
	spi_message_init(&req->msg);
	req->txbuf[1] =  (u16) MAX1233_RDADD(reg);
	req->txbuf[0] = 0; //following dummys
	req->xfer[0].tx_buf = req->txbuf;
	req->xfer[0].rx_buf = req->rxbuf;
	req->xfer[0].len = 4;
	spi_message_add_tail(&req->xfer[0], &req->msg);

	status = spi_sync(spi, &req->msg);

//	printk("max1233_read: (#%X)> %X (status: %X)...(command: %X)\n", reg, req->rxbuf[0], status, (u16) MAX1233_RDADD(reg));

	if (req->msg.status)
		status = req->msg.status;

	if (!status) *val = req->rxbuf[0]; /* return if no error, else error will return*/

	kfree(req);
	return status;
}

static int max1233_write(struct device *dev, u16 reg, u16 val)
{
	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	int			status;

	if (!req)
		return -ENOMEM;

	spi_message_init(&req->msg);

	req->txbuf[1] = (u16) MAX1233_WRADD (reg);
	req->txbuf[0] = val;
	
	req->xfer[0].tx_buf = req->txbuf;
	req->xfer[0].len = 4;

	spi_message_add_tail(&req->xfer[0], &req->msg);

	status = spi_sync(spi, &req->msg);

//	printk("max1233_write: (#%X) < %X (status: %X)...(command: %X)\n", reg, val, status, (u16) MAX1233_WRADD (reg));

	if (req->msg.status)
		status = req->msg.status;

	kfree(req);

	return status;
}


static inline void max1233_resetup_ts(struct spi_device *spi, struct max1233 *ts)
{
	u16 tmp;

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), ts->cmd_crtl1);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_X), &tmp);
	return;
}

static ssize_t max1233_disable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t max1233_disable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;
	int i;

	i = simple_strtoul(buf, &endp, 10);

	if (i)
		max1233_disable(ts);
	else
		max1233_enable(ts);

	return count;
}

static DEVICE_ATTR(disable, 0664, max1233_disable_show, max1233_disable_store);

static ssize_t max1233_brightness_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1233	*ts = dev_get_drvdata(dev);
	u16 i;

	max1233_write(dev, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_DACctrl), 0);
	max1233_read(dev, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_DAC), &i); //setup

	ts->dac = (i & 0xFF);

	/// invert over 255
	ts->dac = 255 - ts->dac;

	return sprintf(buf, "%d\n", ts->dac);
}

static ssize_t max1233_brightness_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;
	int i;

	i = simple_strtoul(buf, &endp, 10);

	ts->dac = i & 0xFF;

	/// invert 
	ts->dac = 255 - ts->dac;
 
	max1233_write(dev, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_DACctrl), 0);
	max1233_write(dev, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_DAC), ts->dac); //setup

	return count;
}

static DEVICE_ATTR(brightness, 0664, max1233_brightness_show, max1233_brightness_store);


static u16 regaddr; /* this stores the addresses for operation */

static ssize_t max1233_regaddr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%u\n", regaddr);
}

static ssize_t max1233_regaddr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char *endp;

	regaddr = simple_strtoul(buf, &endp, 10);

	return count;
}

static DEVICE_ATTR(regaddr, 0664, max1233_regaddr_show, max1233_regaddr_store);


static u16 pageaddr; /* this stores the addresses for operation */

static ssize_t max1233_pageaddr_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%u\n", pageaddr);
}

static ssize_t max1233_pageaddr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char *endp;

	pageaddr = simple_strtoul(buf, &endp, 10);

	return count;
}

static DEVICE_ATTR(pageaddr, 0664, max1233_pageaddr_show, max1233_pageaddr_store);



static ssize_t max1233_regio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	u16 result = 0;
	u16 err = 0;

	err = max1233_read(dev, MAX1233_RDADD(MAX1233_PAGEADDR(pageaddr, regaddr)), &result);
	
	return sprintf(buf, "PAGE%u:REG%u > (%u) (%d) (%X)   (error: %u)\n", pageaddr, regaddr, result, result, result, err);

}

static ssize_t max1233_regio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char *endp;
	int i;
	
	u16 err = 0;

	i = simple_strtoul(buf, &endp, 10);

	err = max1233_write(dev, (u16) MAX1233_WRADD(MAX1233_PAGEADDR(pageaddr, regaddr)), i);

	return count;
}

static DEVICE_ATTR(regio, 0664, max1233_regio_show, max1233_regio_store);


static ssize_t max1233_dumpio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	u16 			*buff = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL); /*tx*/
	u16 			*buff2 = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL);
	int status;
	size_t s;
	int i;

	if (!req | ! buff)
		return -ENOMEM;

	spi_message_init(&req->msg);

	/*the max1233 needs 32 clock cycles to start sending ahead in the page after first word (word--32 clocks--more words in page*/

	buff[0] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(pageaddr, 0));
	req->xfer[1].tx_buf = buff;
	req->xfer[0].len = (1+32)*2;
	req->xfer[1].rx_buf = buff2;

	spi_message_add_tail(&req->xfer[0], &req->msg);

	status = spi_sync(spi, &req->msg);

	printk("max1233_dumpio: (#%X)>... (status: %X)...(command: %X)\n", MAX1233_PAGEADDR(pageaddr, 0), status, buff[0]);

	s = 0;
	
	for(i = 0;i<32;i++) /*in 0 we have nothing, there was the sending word*/
	{
		s += sprintf(buf + s, "PAGE%u:REG%u: > (%u) (%d) (%X)\n", pageaddr, i, buff2[i+1],buff2[i+1],buff2[i+1]);
	}
	s += sprintf(buf + s, "PAGE %u:end\n", pageaddr);


	kfree(buff);
	kfree(buff2);
	kfree(req);

	return s;
}

static DEVICE_ATTR(dumpio, 0664, max1233_dumpio_show, NULL);


static u16 doirqio; /* this stores the addresses for operation */

static ssize_t max1233_doirqio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 verify;


	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_GPIO), &verify); 
	return sprintf(buf, "%x\n", verify);
}

static ssize_t max1233_doirqio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
//	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	doirqio = simple_strtoul(buf, &endp, 10);

	return count;
}

static DEVICE_ATTR(doirqio, 0664, max1233_doirqio_show, max1233_doirqio_store);



static u16 toggleio; /* this stores the addresses for operation */

static ssize_t max1233_toggleio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
//	struct max1233	*ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", toggleio);
}

static ssize_t max1233_toggleio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct max1233 *ts = dev_get_drvdata(dev);
	char *endp;

	toggleio = simple_strtoul(buf, &endp, 10);

		if(toggleio) spi_async(ts->spi, &ts->msg); /* toggle retrieve of PAGE0 first 10 words*/


	return count;
}

static DEVICE_ATTR(toggleio, 0664, max1233_toggleio_show, max1233_toggleio_store);


static u32 ioaddr; /* this stores the addresses for operation */

static ssize_t max1233_ioaddr_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%X\n", ioaddr);
}

static ssize_t max1233_ioaddr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char *endp;

	ioaddr = simple_strtoul(buf, &endp, 16);

	return count;
}

static DEVICE_ATTR(ioaddr, 0664, max1233_ioaddr_show, max1233_ioaddr_store);


static u16 doio; /* this stores the addresses for operation */

static ssize_t max1233_doio_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%X\n", doio);
}

static ssize_t max1233_doio_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	char *endp;

	doio = simple_strtoul(buf, &endp, 16);

	return count;
}

static DEVICE_ATTR(doio, 0664, max1233_doio_show, max1233_doio_store);


static ssize_t max1233_volt_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 result;
	u16 cfg;
	struct max1233	*ts = dev_get_drvdata(dev);

	cfg = MAX1233_TSOP_SCAN_ONE |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_BATAUX) |
			MAX1233_RES(3) |
			MAX1233_AVG(3) |
			MAX1233_CNR(2) |
			MAX1233_STLTIME(3) |
			MAX1233_REFV(0);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), cfg);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_BAT1), &result);
	max1233_resetup_ts(spi, ts);
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(volt, 0444, max1233_volt_show, NULL);



static ssize_t max1233_zaxis_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 result;
	u16 cfg;
	struct max1233	*ts = dev_get_drvdata(dev);

	cfg = MAX1233_TSOP_SCAN_ONE |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_BATAUX) |
			MAX1233_RES(3) |
			MAX1233_AVG(3) |
			MAX1233_CNR(2) |
			MAX1233_STLTIME(3) |
			MAX1233_REFV(0);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), cfg);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_BAT2), &result);
	max1233_resetup_ts(spi, ts);
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(zaxis, 0444, max1233_zaxis_show, NULL);

static ssize_t max1233_xaxis_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 result;
	u16 cfg;
	struct max1233	*ts = dev_get_drvdata(dev);

	cfg = MAX1233_TSOP_SCAN_ONE |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_BATAUX) |
			MAX1233_RES(3) |
			MAX1233_AVG(0) |
			MAX1233_CNR(0) |
			MAX1233_STLTIME(0) |
			MAX1233_REFV(1);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), cfg);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_AUX1), &result);
	max1233_resetup_ts(spi, ts);
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(xaxis, 0444, max1233_xaxis_show, NULL);


static ssize_t max1233_yaxis_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 result;
	u16 cfg;
	struct max1233	*ts = dev_get_drvdata(dev);

	cfg = MAX1233_TSOP_SCAN_ONE |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_AUX2) |
			MAX1233_RES(3) |
			MAX1233_AVG(3) |
			MAX1233_CNR(2) |
			MAX1233_STLTIME(3) |
			MAX1233_REFV(0);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), cfg);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_AUX2), &result);
	max1233_resetup_ts(spi, ts);
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(yaxis, 0444, max1233_yaxis_show, NULL);

static ssize_t max1233_temp1_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 result;
	u16 cfg;
	struct max1233	*ts = dev_get_drvdata(dev);

	cfg = MAX1233_TSOP_SCAN_ONE |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_TEMP_DIFF) |
			MAX1233_RES(3) |
			MAX1233_AVG(0) |
			MAX1233_CNR(0) |
			MAX1233_STLTIME(0) |
			MAX1233_REFV(1);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), cfg);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_TEMP1), &result);
	max1233_resetup_ts(spi, ts);
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(temp1, 0444, max1233_temp1_show, NULL);

static ssize_t max1233_temp2_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct spi_device	*spi = to_spi_device(dev);
	u16 result;
	u16 cfg;
	struct max1233	*ts = dev_get_drvdata(dev);

	cfg = MAX1233_TSOP_SCAN_ONE |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_TEMP_DIFF) |
			MAX1233_RES(3) |
			MAX1233_AVG(0) |
			MAX1233_CNR(0) |
			MAX1233_STLTIME(0) |
			MAX1233_REFV(1);

	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), cfg);
	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_TEMP2), &result);
	max1233_resetup_ts(spi, ts);
	return sprintf(buf, "%d\n", result);
}

static DEVICE_ATTR(temp2, 0444, max1233_temp2_show, NULL);

static struct attribute *max1233_attributes[] = {
	&dev_attr_temp1.attr,
	&dev_attr_temp2.attr,
	&dev_attr_xaxis.attr,
	&dev_attr_yaxis.attr,
	&dev_attr_volt.attr,
	&dev_attr_zaxis.attr,
	&dev_attr_disable.attr,
	&dev_attr_brightness.attr,
//	&dev_attr_gpio4.attr,
	&dev_attr_regaddr.attr,
	&dev_attr_pageaddr.attr,
	&dev_attr_regio.attr,
	&dev_attr_dumpio.attr,
	&dev_attr_doirqio.attr,
	&dev_attr_toggleio.attr,
	&dev_attr_ioaddr.attr,
	&dev_attr_doio.attr,

	NULL
};

static const struct attribute_group max1233_attr_group = {
	.attrs = max1233_attributes,
};

/*--------------------------------------------------------------------------*/

/*
 * /DAV Data available Interrupt only kicks the kthread.
 * The kthread kicks the timer only to issue the Pen Up Event if
 * no new data is available
 *
 */

static void max1233_rx(void *ads)
{
	struct max1233		*ts = ads;
	struct input_dev	*input_dev = ts->input;
	unsigned		Rt;
	u32			x, y, z1, z2, p2;
	int t;


	x = ts->conversion_data[0];
	y = (ts->conversion_data[1] >> 16);
	z1 = (ts->conversion_data[1] & 0x00FF);
	z2 = (ts->conversion_data[2] >> 16);

//	printk("max1233: Prior  X:%d Y:%d Z1:%d Z2:%d X_OHM:%d Y_OHM:%d\n", 
//		x, y, z1, z2, ts->x_plate_ohms, ts->y_plate_ohms);

	if (x == MAX_12BIT)
		x = 0;

	if (likely(x && z1 && (y != MAX_12BIT) && !device_suspended(&ts->spi->dev))) {
		/* compute touch pressure resistance using equation #2 */

		Rt = MAX_12BIT - z1 ;
		Rt *= x;
		Rt *= ts->x_plate_ohms;
		Rt /= z1;

		p2 = MAX_12BIT - y;
		p2 *= ts->y_plate_ohms;

		Rt -= p2;
		Rt >>= 12;

	} else
		Rt = 0;

//	printk("max1233: Prior 2 X:%d Y:%d Z1:%d Z2:%d Rt %d X_OHM:%d Y_OHM:%d\n", 
//		x, y, z1, z2, Rt, ts->x_plate_ohms, ts->y_plate_ohms);

	if (Rt > 20000)
		return;

	x = (MAX_12BIT - x);
	y = (MAX_12BIT - y);
	swap(x, y);

	if (x > xold + 50
		|| x < xold - 50
		|| y > yold + 50
		|| y < yold - 50 ) {
//		printk("max1233: xold %d yold %d - ", xold, yold);
//		printk("NOT Reporting X:%d Y:%d Z1:%d Z2:%d Rt %d X_OHM:%d Y_OHM:%d\n",
//			x, y, z1, z2, Rt, ts->x_plate_ohms, ts->y_plate_ohms);
		msleep(5);
	} else {
//		printk("max1233: Reporting X:%d Y:%d Z1:%d Z2:%d Rt %d X_OHM:%d Y_OHM:%d\n",
//			x, y, z1, z2, Rt, ts->x_plate_ohms, ts->y_plate_ohms);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_report_abs(input_dev, ABS_X, x);
		input_report_abs(input_dev, ABS_Y, y);
		input_report_abs(input_dev, ABS_PRESSURE,Rt);
		input_sync(input_dev);
		msleep(5);
		t = mod_timer(&ts->ts_timer, jiffies + TS_PEN_UP_TIMEOUT);
	}
	xold = x;
	yold = y;
}

static void max1233_ts_timer(unsigned long handle)
{
	struct max1233	*ts = (void *)handle;
	struct input_dev *input_dev = ts->input;

	input_report_abs(input_dev, ABS_PRESSURE, 0);
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);
}


static irqreturn_t max1233_irq_ts(int irq, void *handle)
{
	struct max1233 *ts = handle;
	int status;

	msleep(10);
	status = spi_sync(ts->spi, &ts->msg);
	if (status)
		dev_err(&ts->spi->dev, "spi_sync --> %d\n", status);
	max1233_rx(ts);

	return IRQ_HANDLED;
}

static void max1233_callback_kp(void *handle)
{
	struct max1233 *ts = handle;
	struct input_dev *input_dev = ts->kb_input;
	int	i, status, gotimer;

	do {
		gotimer = 0;
		for (i=0; i<HAVE_KEYS; i++) {
			if ((((ts->kb_data[0]) & (1<<ts->key[i].offset)) == (1<<ts->key[i].offset))
				&& (((ts->kb_data[0]) & (0<<(ts->key[i].offset + 1))) == (0<<(ts->key[i].offset +1)))) {
				//printk("max1233: KP Report %d UP\n", i);
				ts->key[i].down = 0;
				input_report_key(input_dev, ts->key[i].key, 0);
			} else if ((((ts->kb_data[0]) & (0<<ts->key[i].offset)) == (0<<ts->key[i].offset))
				&& (((ts->kb_data[0]) & (1<<(ts->key[i].offset + 1))) == (1<<(ts->key[i].offset + 1)))) {
				//printk("max1233: KP Report %d DOWN\n", i);
				ts->key[i].down = 1;
				input_report_key(input_dev, ts->key[i].key, 1);
			}
		}

		for (i=0; i<HAVE_KEYS; i++) {
			if (ts->key[i].down) {
				//printk("max1233: %d DOWN starting timer\n", i);
				gotimer = 1;
			}
		}
		input_sync(input_dev);
		if (gotimer) {
			msleep(50);
			status = spi_sync(ts->spi, &ts->kb_msg);
			if (status)
				dev_err(&ts->spi->dev, "spi_sync --> %d\n", status);
		}
	} while (gotimer);
}

static void max1233_kb_timer(unsigned long handle)
{
	struct max1233		*ts = (void *)handle;
	int status;

//	printk("max1233_kb_timer()\n");
	status = spi_async(ts->spi, &ts->kb_msg);
	if (status)
		dev_err(&ts->spi->dev, "spi_sync --> %d\n", status);
}

static irqreturn_t max1233_irq_kp(int irq, void *handle)
{
	struct max1233 *ts = handle;
	int status;

	msleep(10);

	status = spi_sync(ts->spi, &ts->kb_msg);
	if (status)
		dev_err(&ts->spi->dev, "spi_sync --> %d\n", status);
	max1233_callback_kp(ts);

	return IRQ_HANDLED;
}

/* Must be called with ts->lock held */
static void max1233_disable(struct max1233 *ts)
{
	if (ts->disabled)
		return;

	ts->disabled = 1;

	if (!ts->pending) {
		mutex_lock(&ts->lock);
		ts->irq_disabled = 1;
		disable_irq(ts->spi->irq);
		mutex_unlock(&ts->lock);
	} else {
		/* the kthread will run at least once more, and
		 * leave everything in a clean state, IRQ disabled
		 */
		while (ts->pending)
			msleep(1);
	}
}

/* Must be called with ts->lock held */
static void max1233_enable(struct max1233 *ts)
{
	if (!ts->disabled)
		return;

	mutex_lock(&ts->lock);
	ts->disabled = 0;
	ts->irq_disabled = 0;
	enable_irq(ts->spi->irq);
	mutex_unlock(&ts->lock);
}

static int max1233_suspend(struct spi_device *spi, pm_message_t message)
{
	struct max1233 *ts = dev_get_drvdata(&spi->dev);

	spi->dev.power.power_state = message;
	max1233_disable(ts);

	return 0;

}

static int max1233_resume(struct spi_device *spi)
{
	struct max1233 *ts = dev_get_drvdata(&spi->dev);

	spi->dev.power.power_state = PMSG_ON;
	max1233_enable(ts);

	return 0;
}

static inline void max1233_setup_ts_def_msg(struct spi_device *spi, struct max1233 *ts)
{
	int i,y;
	struct spi_message	*m = &ts->msg;
	struct spi_message	*k = &ts->kb_msg;

	struct spi_transfer *x = &ts->xfer[0];
	struct spi_transfer *kx = &ts->kxfer[0];

	ts->cmd_crtl1 = MAX1233_TSOP_DET_SCAN_IRQ |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_MEAS_XYZ) |
			MAX1233_RES(3) |
			MAX1233_AVG(3) |
			MAX1233_CNR(2) |
			MAX1233_STLTIME(3) |
			MAX1233_REFV(0);


	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), ts->cmd_crtl1); //setup


	spi_message_init(m);
	m->complete = max1233_rx;
	m->context = ts; /*HANDLE to the callback*/

	y = 3;

	for (i=0; i<4; i++, x++) {
		send_data[i].dat[1] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(MAX1233_PAGE0, i));
		x->tx_buf = &send_data[i];
		x->rx_buf = &ts->conversion_data[i];
		x->len = 4;
		spi_message_add_tail(x,m);
		y--;
		}

	spi_message_init(k);
	k->complete = max1233_callback_kp;
	k->context = ts; /*HANDLE to the callback*/

	ksend_data[0].dat[1] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_GPIO));
	kx->tx_buf = &ksend_data[0];
	kx->rx_buf = &ts->kb_data[0];
	kx->len = 4;
	spi_message_add_tail(kx,k);
}

static int __devinit max1233_probe(struct spi_device *spi)
{
	struct max1233			*ts;
	struct input_dev		*input_dev;
	struct input_dev		*kb_dev;
	struct max1233_platform_data	*pdata = spi->dev.platform_data;
	int				err;
	u16				verify;

	printk("max1233_probe: tries probe...\n");

	if (!spi->irq) {
		dev_dbg(&spi->dev, "no IRQ?\n");
		return -ENODEV;
	}

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	spi->bits_per_word = 32;
	spi->mode = SPI_MODE_3;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_dbg(&spi->dev, "SPI CLK %d Hz?\n",spi->max_speed_hz);
		return -EINVAL;
	}

	ts = kzalloc(sizeof(struct max1233), GFP_KERNEL); //create driver struct
	input_dev = input_allocate_device();
	kb_dev = input_allocate_device();

	if (!ts || !input_dev || !kb_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, ts);
	spi->dev.power.power_state = PMSG_ON;

	ts->spi = spi;
	ts->input = input_dev;
	ts->intr_flag = 0;

	setup_timer(&ts->ts_timer, max1233_ts_timer, (unsigned long) ts);

	mutex_init(&ts->lock);

	ts->model = pdata->model ? : 1233;
	ts->vref_delay_usecs = pdata->vref_delay_usecs ? : 100;
	ts->x_plate_ohms = 820;
	ts->y_plate_ohms = 325;
	
	ts->pressure_max = pdata->pressure_max ? : ~0;
	ts->kp_irq =	pdata->kp_irq;

	ts->xmin = pdata->x_min;
	ts->xmax = pdata->x_max ? : MAX_12BIT;
	ts->ymin = pdata->y_min;
	ts->ymax = pdata->y_max ? : MAX_12BIT;
	

	ts->stopacq_polarity = pdata->stopacq_polarity;
	ts->first_conversion_delay = pdata->first_conversion_delay;
	ts->acquisition_time = pdata->acquisition_time;
	ts->averaging = pdata->averaging;
	ts->pen_down_acc_interval = pdata->pen_down_acc_interval;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0"," max1233" );
	input_dev->name = "MAX1233 Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &spi->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);

	input_set_abs_params(input_dev, ABS_X,
			pdata->x_min ? : 0,
			pdata->x_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			pdata->y_min ? : 0,
			pdata->y_max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			pdata->pressure_min, 
			MAX_12BIT, 
			0, 0);

	/// KEYBOARD 
	snprintf(ts->kb_phys, sizeof(ts->kb_phys), "%s/input-kb", "max1233");
	ts->kb_input = kb_dev;
	kb_dev->name ="MAX1233 Keyboard";
	kb_dev->phys = ts->kb_phys;
	kb_dev->dev.parent = &spi->dev;
	kb_dev->evbit[0] = BIT(EV_KEY);

	ts->key[0].key = KEY_F3;
	ts->key[0].offset = KEY_LEFT_OFFSET;
	ts->key[0].down = 0;
	input_set_capability(kb_dev, EV_KEY, ts->key[0].key);
	
	ts->key[1].key = KEY_F1;
	ts->key[1].offset = KEY_RIGHT_OFFSET;
	ts->key[1].down = 0;
	input_set_capability(kb_dev, EV_KEY, ts->key[1].key);
	
	ts->key[2].key = KEY_F2;
	ts->key[2].offset = KEY_MID_OFFSET;
	ts->key[2].down = 0;
	input_set_capability(kb_dev, EV_KEY, ts->key[2].key);

	setup_timer(&ts->kb_timer, max1233_kb_timer, (unsigned long) ts);

	/// TURN ON KEYPAD
	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_GPIO), 0xFF00);	
	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_KEYctrl), 0x8000);

	/// TURN ON DAC
	ts->cmd_crtl2 = MAX1233_DAC_ENABLE(1); //turn on dac
	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_DACctrl), ts->cmd_crtl2);

	/// CONFIGURE RES
	ts->cmd_crtl1 = MAX1233_TSOP_DET_SCAN_IRQ |
			MAX1233_ADCSCANSEL(MAX1233_ADCSCAN_CONFRES) |
			MAX1233_RES(1) |
			MAX1233_AVG(0) |
			MAX1233_CNR(2) |
			MAX1233_STLTIME(5) |
			MAX1233_REFV(0);
	max1233_write((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE1, MAX1233_REG_ADCctrl), ts->cmd_crtl1); //setup

	/// LOAD SPI MSG
	max1233_setup_ts_def_msg(spi, ts); //general setup

	/// REQUEST TS interrupt
	if (request_threaded_irq(spi->irq, NULL, max1233_irq_ts,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, spi->dev.driver->name, ts)) {
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err_free_mem;
	}

	/// REQUEST KP IRQ
	if (request_threaded_irq(ts->kp_irq, NULL, max1233_irq_kp,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "max1233_kb", ts)) {
		dev_dbg(&spi->dev, "irq %d busy?\n", spi->irq);
		err = -EBUSY;
		goto err_free_mem;
	}

	err = sysfs_create_group(&spi->dev.kobj, &max1233_attr_group);
	if (err)
		goto err_remove_attr;

	if (err)
		goto err_remove_attr;

	err = input_register_device(input_dev);
	if (err)
		goto err_idev;

	err = input_register_device(kb_dev);
	if (err)
		goto err_idev;

	max1233_read((struct device *) spi, MAX1233_PAGEADDR(MAX1233_PAGE0, MAX1233_REG_X), &verify);

	printk("max1233_probe: probe OK. Touchscreen enabled...\n");
	return 0;

err_idev:
	input_dev = NULL; /* so we don't try to free it later */

err_remove_attr:

	sysfs_remove_group(&spi->dev.kobj, &max1233_attr_group);
	free_irq(spi->irq, ts);

err_free_mem:
	input_free_device(input_dev);
	input_free_device(kb_dev);
	kfree(ts);
	dev_set_drvdata(&spi->dev, NULL);
	return err;
}

static int __devexit max1233_remove(struct spi_device *spi)
{
	struct max1233		*ts = dev_get_drvdata(&spi->dev); //reference allocated driver struct in kernel

	max1233_suspend(spi, PMSG_SUSPEND);

	sysfs_remove_group(&spi->dev.kobj, &max1233_attr_group);

	free_irq(ts->spi->irq, ts);
	free_irq(ts->kp_irq, ts);

	input_unregister_device(ts->input);
	input_unregister_device(ts->kb_input);

	kfree(ts); //free the driver struct

	dev_dbg(&spi->dev, "unregistered touchscreen\n");
	dev_set_drvdata(&spi->dev, NULL);

	return 0;
}

static struct spi_driver max1233_driver = {
	.driver = {
		.name	= "max1233",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= max1233_probe,
	.remove		= __devexit_p(max1233_remove),
	.suspend	= max1233_suspend,
	.resume		= max1233_resume,
};

static int __init max1233_init(void)
{
	printk("max1233_init() v0.8.4\n");
 	return (spi_register_driver(&max1233_driver));
}
module_init(max1233_init);

static void __exit max1233_exit(void)
{
	printk("max1233_exit()\n");
	spi_unregister_driver(&max1233_driver);
}
module_exit(max1233_exit);

MODULE_DESCRIPTION("max1233 TouchScreen Driver");
MODULE_LICENSE("GPL");
