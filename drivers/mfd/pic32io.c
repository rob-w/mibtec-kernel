/*
 * File:        mfd/pic32io.c
 *
 *		Copyright (C) 2010 Robert Woerle
 *
 * Author:	Robert Woerle
 *
 * Created:	Dec, 21th 2010
 * Description:	PIC32 powered I/O device with 1xCanbus, 2xRS485, 8 Analog 8 Digital I/O
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
 */
 
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <asm/irq.h>
#include <linux/spi/pic32io.h>

#define MAX_SPI_FREQ_HZ			20000000
#define	MAX_12BIT			((1<<12)-1)

#define CONFIG_TXBUFFER		0
#define TRANSMIT_SETTINGS	1
#define TRANSMIT_INIT		2
#define TRANSMIT_ABORT		3
#define TRANSMIT_ERROR		4
#define BITTIMING			5
#define READ_A_REG			6
#define RESET				7
#define CONFIG_RXBUFFER		8
#define CONFIG_MODE			9
#define NORMAL_MODE			10
#define LOOPBACK_MODE		11
#define CLEAR_MODE			12
#define FILTER				13
#define TRANSMIT_BUFFER		14
#define WRITE_A_REG			15

enum uart_regs {
	UART_DR              = 0x00,
	UART_SR              = 0x04,
	UART_LINE_CR         = 0x08,
	UART_BAUDDIV_MSB     = 0x0c,
	UART_BAUDDIV_LSB     = 0x10,
	UART_CR              = 0x14,
	UART_FR              = 0x18,
	UART_IIR             = 0x1c,
	UART_ILPR            = 0x20,
	UART_RTS_CR          = 0x24,
	UART_RTS_LEAD        = 0x28,
	UART_RTS_TRAIL       = 0x2c,
	UART_DRV_ENABLE      = 0x30,
	UART_BRM_CR          = 0x34,
	UART_RXFIFO_IRQLEVEL = 0x38,
	UART_TXFIFO_IRQLEVEL = 0x3c,
};

#define SR_FE (1<<0)
#define SR_PE (1<<1)
#define SR_BE (1<<2)
#define SR_OE (1<<3)

#define LINE_CR_BRK       (1<<0)
#define LINE_CR_PEN       (1<<1)
#define LINE_CR_EPS       (1<<2)
#define LINE_CR_STP2      (1<<3)
#define LINE_CR_FEN       (1<<4)
#define LINE_CR_5BIT      (0<<5)
#define LINE_CR_6BIT      (1<<5)
#define LINE_CR_7BIT      (2<<5)
#define LINE_CR_8BIT      (3<<5)
#define LINE_CR_BITS_MASK (3<<5)

#define CR_UART_EN (1<<0)
#define CR_SIREN   (1<<1)
#define CR_SIRLP   (1<<2)
#define CR_MSIE    (1<<3)
#define CR_RIE     (1<<4)
#define CR_TIE     (1<<5)
#define CR_RTIE    (1<<6)
#define CR_LBE     (1<<7)

#define FR_CTS  (1<<0)
#define FR_DSR  (1<<1)
#define FR_DCD  (1<<2)
#define FR_BUSY (1<<3)
#define FR_RXFE (1<<4)
#define FR_TXFF (1<<5)
#define FR_RXFF (1<<6)
#define FR_TXFE (1<<7)

#define IIR_MIS (1<<0)
#define IIR_RIS (1<<1)
#define IIR_TIS (1<<2)
#define IIR_RTIS (1<<3)
#define IIR_MASK 0xf

#define RTS_CR_AUTO (1<<0)
#define RTS_CR_RTS  (1<<1)
#define RTS_CR_COUNT (1<<2)
#define RTS_CR_MOD2  (1<<3)
#define RTS_CR_RTS_POL (1<<4)
#define RTS_CR_CTS_CTR (1<<5)
#define RTS_CR_CTS_POL (1<<6)
#define RTS_CR_STICK   (1<<7)

#define UART_PORT_SIZE 0x40

#define PIC32UART_MAJOR	204
#define PIC32UART_MINOR	16
#define PIC32UART_DEVNAME	"ttyPIC"
#define PIC32UART_NR		2

#define PIC32CAN_MAJOR	242	//define Majornumber


/*this is only for  interaction of one word back and forth*/
struct ser_req {
	u16			txbuf[2];
	u16			rxbuf[2];
	struct spi_message	msg;
	struct spi_transfer	xfer[1];
};

#define FRAME_BYTES		512

struct frames {
	u8	dat[FRAME_BYTES];
	};

/// we do 16 bytes * 6 to match with 96 fifo 
struct snd_data {
//	u8 dat[FRAME_BYTES];
//	u8 length;
	struct frames frameid[6];
	};
	
struct spi_event {
	/* For portability, we can't read 12 bit values using SPI (which
	 * would make the controller deliver them as native byteorder u16
	 * with msbs zeroed).  Instead, we read them as two 8-bit values,
	 * *** WHICH NEED BYTESWAPPING *** and range adjustment.
	 */
	u16	x;
	u16	y;
	u16	z1, z2;
	int	ignore;
};
	
enum {
	CANBUS = 0,
	RS485_1,
	RS485_2,
	IO_DIGIN,
	IO_DIGOUT,
	IO_AIN,
	IO_AOUT,
	ALLDEV,
	};

struct pic32io {
	struct spi_device	*spi;
	unsigned		irq_disabled:1;	
	unsigned		disabled:1;

	u8				dev_busy[3];

	spinlock_t		lock;
	unsigned		pending:1;	
	struct spi_message	msg[12];
//	struct pic32spi_packet	*packet;
	struct spi_transfer	xfer[12];
	struct snd_data		snd_dev[3];	
	struct snd_data		rcv_dev[3];
	unsigned int		pic2omap[3];
	unsigned int		omap2pic[3];
	u8					pic32can_rcv[184];
	u8					pic32can_rcv_rdy;
	unsigned			pic32can_disabled:1;
	u8					pic32can_snd[184];
	u8					pic32can_snd_rdy;
	wait_queue_head_t	can_alarm_wq;
};

struct pic32io *p_pic32io;

static inline void pic32spi_rcv_clr_buffer(struct pic32io *pic, int id);

static void pic32io_enable(struct pic32io *pic);
static void pic32io_disable(struct pic32io *pic);
static void pic32can_send_spi(struct pic32io *pic, int id, int length);

/*
static int device_suspended(struct device *dev)
{
	struct pic32io *pic = dev_get_drvdata(dev);
	return dev->power.power_state.event != PM_EVENT_ON || pic->disabled;
}
*/

static ssize_t pic32io_inputs(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

//	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	u16 			*buff = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL); /*tx*/
	u16 			*buff2 = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL);
//	int status;
	size_t s;
//	int i;

	printk("pic32io: inputs show\n");
	
	if (!req | ! buff)
		return -ENOMEM;

//	spi_message_init(&req->msg);

	
//	buff[0] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(pageaddr, 0));
//	req->xfer[1].tx_buf = buff;
//	req->xfer[0].len = (1+32)*2;
//	req->xfer[1].rx_buf = buff2;

//	spi_message_add_tail(&req->xfer[0], &req->msg);

//	status = spi_sync(spi, &req->msg);

//	printk("pic32io: inputs %d\n", 99 );

	s = 99;
	
//	for(i = 0;i<32;i++) /*in 0 we have nothing, there was the sending word*/
//	{
//		s += sprintf(buf + s, "PAGE%u:REG%u: > (%u) (%d) (%X)\n", pageaddr, i, buff2[i+1],buff2[i+1],buff2[i+1]);
//	}
//	s += sprintf(buf + s, "PAGE %u:end\n", pageaddr);


	kfree(buff);
	kfree(buff2);
	kfree(req);

	return sprintf(buf, "99\n");
}

static DEVICE_ATTR(inputs, 0444, pic32io_inputs, NULL);

static ssize_t pic32io_outputs(struct device *dev,
				     struct device_attribute *attr, char *buf)
{

//	struct spi_device	*spi = to_spi_device(dev);
	struct ser_req		*req = kzalloc(sizeof *req, GFP_KERNEL);
	u16 			*buff = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL); /*tx*/
	u16 			*buff2 = kcalloc ( (1+32)*2, sizeof(u16), GFP_KERNEL);
//	int status;
//	size_t s;
//	int i;

	printk("pic32io: outputs show\n");
	
	if (!req | ! buff)
		return -ENOMEM;

//	spi_message_init(&req->msg);
//	buff[0] = (u16) MAX1233_RDADD(MAX1233_PAGEADDR(pageaddr, 0));
//	req->xfer[1].tx_buf = buff;
//	req->xfer[0].len = (1+32)*2;
//	req->xfer[1].rx_buf = buff2;
//	spi_message_add_tail(&req->xfer[0], &req->msg);
//	status = spi_sync(spi, &req->msg);

	kfree(buff);
	kfree(buff2);
	kfree(req);

	return sprintf(buf, "99\n");
}

static DEVICE_ATTR(outputs, 0664, pic32io_outputs, NULL);

static struct attribute *pic32io_attributes[] = {
	&dev_attr_inputs.attr,
	&dev_attr_outputs.attr,
//	&dev_attr_doio.attr,
	NULL
};

static const struct attribute_group pic32io_attr_group = {
	.attrs = pic32io_attributes,
};

static void pic32spi_rx(void *ads)
{
	struct pic32io		*pic = ads;
	u16 y;

	printk("pic32spi: rx() 0 %d 1 %d 2 %d  rcv_rdy %d\n", pic->dev_busy[0], pic->dev_busy[1], pic->dev_busy[2], pic->pic32can_rcv_rdy);

	if (pic->dev_busy[2] && !pic->pic32can_rcv_rdy) {
		for (y=0; y<184; y++) {
			pic->pic32can_rcv[y] = pic->rcv_dev[2].frameid[0].dat[y];
			}
		pic->pic32can_rcv_rdy = 1;
		pic32spi_rcv_clr_buffer(pic, 2);
		}
	pic->dev_busy[2] = 0;
}

static void pic32can_send_spi(struct pic32io *pic, int id, int length)
{
	struct spi_message	*m = &pic->msg[id];
	struct spi_transfer *x = &pic->xfer[id];
	int i, status;
	
	printk("pic32spi: sending padding id %d with lenght %d \n", id, length);
		
	spi_message_init(m);
	i = 0;
	
	if (pic->pic32can_snd[0] && pic->pic32can_snd_rdy) {
		for (i=0; i<length; i++) {
			pic->snd_dev[id].frameid[0].dat[i] = pic->pic32can_snd[i];
			pic->pic32can_snd[i] = 0;
		}
	}
	else {
		for (i=0; i<length; i++)
			pic->snd_dev[id].frameid[0].dat[i] = 0;
	}
	
	pic->pic32can_snd_rdy = 0;
	
	x->len = length;
	x->tx_buf = &pic->snd_dev[id];
	x->rx_buf = &pic->rcv_dev[id];
//	m->complete = pic32spi_rx;
	m->context = pic;
	spi_message_add_tail(x,m);
	
	status = spi_sync(pic->spi, &pic->msg[id]);
	if (status)
		dev_err(&p_pic32io->spi->dev, "spi_sync out --> %d\n", status);
	

	
	return;
}

static irqreturn_t pic32io_hard_irq(int irq, void *handle)
{
	struct pic32io		*pic = handle;
	unsigned long flags;
	
	spin_lock_irqsave(&pic->lock, flags);
	disable_irq(OMAP_GPIO_IRQ(irq));
	spin_unlock_irqrestore(&pic->lock, flags);

	printk("pic32io: HARD_IRQ_%d\n", irq);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t pic32io_irq(int irq, void *handle)
{
	struct pic32io		*pic = handle;
	unsigned long flags;
	
	printk("pic32io: IRQ_%d\n", irq);
	
	switch (irq) {
		case 182:
		break;
		case 179:
		break;
		case 178:
			pic32can_send_spi(pic, 2, 184);
			pic32spi_rx(pic);
			spin_lock_irqsave(&pic->lock, flags);
			enable_irq(OMAP_GPIO_IRQ(irq));
			spin_unlock_irqrestore(&pic->lock, flags);
		break;
		default:
			printk("pic32io: ERROR unknown IRQ %d\n", irq);
		break;
	}
	return IRQ_HANDLED;
}

static void pic32io_disable(struct pic32io *pic)
{
	unsigned long flags;
	printk("pic32io disable()\n");
	if (pic->disabled)
		return;

	pic->disabled = 1;

	if (!pic->pending) {
		spin_lock_irqsave(&pic->lock, flags);
		pic->irq_disabled = 1;
		disable_irq(pic->spi->irq);
		spin_unlock_irqrestore(&pic->lock, flags);
	} else {
		/* the kthread will run at least once more, and
		 * leave everything in a clean state, IRQ disabled
		 */
		while (pic->pending)
			msleep(1);
	}
}

static void pic32io_enable(struct pic32io *pic)
{
	unsigned long flags;
	printk("pic32io: enable()\n");
	if (!pic->disabled)
		return;

	spin_lock_irqsave(&pic->lock, flags);
	pic->disabled = 0;
	pic->irq_disabled = 0;
	enable_irq(pic->spi->irq);
	spin_unlock_irqrestore(&pic->lock, flags);
}
 
static int pic32io_suspend(struct spi_device *spi, pm_message_t message)
{
	struct pic32io *pic = dev_get_drvdata(&spi->dev);
	printk("pic32io: suspend()\n");
	spi->dev.power.power_state = message;
	pic32io_disable(pic);

	return 0;
}

static int pic32io_resume(struct spi_device *spi)
{
	struct pic32io *pic = dev_get_drvdata(&spi->dev);
	printk("pic32io: resume()\n");
	spi->dev.power.power_state = PMSG_ON;
	pic32io_enable(pic);

	return 0;
}

static void pic32uart_break_ctl(struct uart_port *port, int break_state)
{
//	unsigned int line_cr;
	printk("pic32uart_%d: break_ctl()\n", port->line);
	spin_lock_irq(&port->lock);

/*	line_cr = readl(port->membase + UART_LINE_CR);
	if (break_state != 0)
		line_cr |= LINE_CR_BRK;
	else
		line_cr &= ~LINE_CR_BRK;
	writel(line_cr, port->membase + UART_LINE_CR);
*/
	spin_unlock_irq(&port->lock);
}

static void pic32uart_enable_ms(struct uart_port *port)
{
//	unsigned int val;
//	val = readl(port->membase + UART_CR);
//	writel(val | CR_MSIE, port->membase + UART_CR);
	printk("pic32uart_%d: enable_ms()\n", port->line);
}

static void pic32uart_stop_rx(struct uart_port *port)
{
//	unsigned int val;
//	val = readl(port->membase + UART_CR);
//	writel(val & ~CR_RIE,  port->membase + UART_CR);
	printk("pic32uart_%d: uart_stop_rx()\n", port->line);
}

static unsigned int pic32uart_tx_empty(struct uart_port *port)
{
	printk("pic32uart_%d: uart_tx_empty() or still stuff ?!\n", port->line);
//	return (gpio_get_value(p_pic32io->pic2omap[port->line])) ? TIOCSER_TEMT : 0;

//	return (UART_GET_LSR(port) & URLS_URTE) ? TIOCSER_TEMT : 0;
	return TIOCSER_TEMT;
}

static const char *pic32uart_type(struct uart_port *port)
{
	printk("pic32uart_%d: uart_type()\n", port->line);
	return port->type == PORT_16550 ? "PIC32IO" : NULL;
}

static unsigned int pic32uart_get_mctrl(struct uart_port *port)
{
//	unsigned int result = 0;
	unsigned int ret = TIOCM_DSR | TIOCM_CAR;
	printk("pic32uart_%d: uart_get_mctrl()\n", port->line);

//	if (readl(port->membase + UART_FR) & FR_CTS)
		ret |= TIOCM_CTS;

	return ret;
}

static void pic32uart_set_mctrl(struct uart_port *port, u_int mctrl)
{
//	unsigned int val;
	printk("pic32uart_%d: uart_set_mctrl()\n", port->line);

	/* FIXME: Locking needed ? */
	if (mctrl & TIOCM_RTS) {
//		val = readl(port->membase + UART_RTS_CR);
//		writel(val | RTS_CR_RTS, port->membase + UART_RTS_CR);
	}
}

static int pic32uart_startup(struct uart_port *port)
{
	int retval = 0;
	
	printk("pic32uart_%d: startup()\n", port->line);
	return retval;
}

static void pic32uart_shutdown(struct uart_port *port)
{
	printk("pic32uart_%d: shutdown()\n", port->line);
	gpio_set_value(p_pic32io->omap2pic[0], 0);
}

static void pic32uart_stop_tx(struct uart_port *port)
{
	printk("pic32uart_%d: stop_tx()\n", port->line);
}

static inline void pic32uart_transmit_buffer(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	struct spi_message	*m = &p_pic32io->msg[0];
	struct spi_transfer *x = &p_pic32io->xfer[0];
	unsigned int count = FRAME_BYTES;
	int i, status, frame;

		
	if (port->x_char) {
//		writel(port->x_char, port->membase + UART_DR);
		printk("pic32uart_%d: port->x_char ! \n", port->line);
		port->icount.tx++;
		port->x_char = 0;
		pic32uart_stop_tx(port);
		return;
	}

//	if ((uart_tx_stopped(port)) || (uart_circ_empty(xmit))) {
//		printk("pic32uart_%d: transmit_puffer() uart_circ_empty or stopped \n", port->line);
//		pic32uart_stop_tx(port);
//		return;
//	}

	spi_message_init(m);
	i = 0;
	frame = 0;
	do {
	
//		if ((!gpio_get_value(p_pic32io->omap2pic[port->line]))
//			&& (!gpio_get_value(p_pic32io->pic2omap[port->line])))
//		{
//		printk("pic32uart_%d: i %d cnt %d transmit_puffer() has %x \n", port->line,  i, count,  xmit->buf[xmit->tail]);
		
		p_pic32io->snd_dev[port->line].frameid[frame].dat[i] = xmit->buf[xmit->tail];
		i++;
		x->len = i;
//		i++;
			
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if ((count-- <= 1)) {
			x->tx_buf = &p_pic32io->snd_dev[port->line].frameid[frame];
			x->rx_buf = &p_pic32io->rcv_dev[port->line].frameid[frame];
			m->complete = pic32spi_rx;
			m->context = p_pic32io;
			spi_message_add_tail(x,m);

			printk("pic32uart_%d: sending frame %d with %d bytes to frame imit\n", port->line, frame, x->len);
	
			gpio_set_value(p_pic32io->omap2pic[port->line + 1], 1);
			p_pic32io->dev_busy[port->line + 1] = 1;
	
			status = spi_async(p_pic32io->spi, m/*&p_pic32io->msg[port->line]*/);
			if (status)
				dev_err(&p_pic32io->spi->dev, "spi_sync out --> %d\n", status);
			count = FRAME_BYTES;
			i = 0;
			x++;
			frame++;
			m++;
			spi_message_init(m);
		}
	} while (!uart_circ_empty(xmit)/* && (count-- > 0)*/);

	if (i) {
		x->tx_buf = &p_pic32io->snd_dev[port->line].frameid[frame];
		x->rx_buf = &p_pic32io->rcv_dev[port->line].frameid[frame];
		m->complete = pic32spi_rx;
		m->context = p_pic32io;
	
		spi_message_add_tail(x,m);
	
		printk("pic32uart_%d: sending frame %d with %d bytes circ empty\n", port->line, frame, x->len);
	
		gpio_set_value(p_pic32io->omap2pic[port->line + 1], 1);
		p_pic32io->dev_busy[port->line + 1] = 1;
	
		status = spi_async(p_pic32io->spi, m /*&p_pic32io->msg[port->line]*/);
		if (status)
			dev_err(&p_pic32io->spi->dev, "spi_sync out --> %d\n", status);
	}


//	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
//		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		pic32uart_stop_tx(port);
}

static void pic32uart_start_tx(struct uart_port *port)
{	
	printk("pic32uart_%d: start_tx()\n", port->line);

//	writel(
//	    readl(port->membase + UART_CR) | CR_TIE, port->membase + UART_CR);
//	if (!(readl(port->membase + UART_FR) & FR_TXFF))
	
	pic32uart_transmit_buffer(port);
}

static void pic32uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
	printk("pic32uart_%d: set_termios()\n", port->line);
}

static void pic32uart_release_port(struct uart_port *port)
{
	printk("pic32uart_%d: uart_release_port()\n", port->line);
}

static int pic32uart_request_port(struct uart_port *port)
{
	int retval = 0;
	printk("pic32uart_%d: uart_request_port()\n", port->line);
	return retval;
}

static void pic32uart_config_port(struct uart_port *port, int flags)
{
	printk("pic32uart_%d: uart_config_port()\n", port->line);
}

static int pic32uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int retval = 0;
	return retval;
	printk("pic32uart_%d: uart_verfiy_port()\n", port->line);
}

static struct uart_ops pic32uart_pops = {
	.tx_empty	= pic32uart_tx_empty,
	.set_mctrl	= pic32uart_set_mctrl,
	.get_mctrl	= pic32uart_get_mctrl,
	.stop_tx	= pic32uart_stop_tx,
	.start_tx	= pic32uart_start_tx,
	.stop_rx	= pic32uart_stop_rx,
	.enable_ms	= pic32uart_enable_ms,
	.break_ctl	= pic32uart_break_ctl,
	.startup	= pic32uart_startup,
	.shutdown	= pic32uart_shutdown,
	.set_termios	= pic32uart_set_termios,
	.type		= pic32uart_type,
	.release_port	= pic32uart_release_port,
	.request_port	= pic32uart_request_port,
	.config_port	= pic32uart_config_port,
	.verify_port	= pic32uart_verify_port,
};

struct pic32uart_port {
	struct uart_port	port;
};

/*
static struct pic32uart_port pic32uart_ports[] = {
	{
	.port = {
		.type = PORT_NETX,
//		.iotype = UPIO_MEM,
//		.membase = (char __iomem *)io_p2v(NETX_PA_UART0),
//		.mapbase = NETX_PA_UART0,
//		.irq = NETX_IRQ_UART0,
		.uartclk = 100000000,
		.fifosize = 96,
//		.flags = UPF_BOOT_AUTOCONF,
		.ops = &pic32uart_pops,
		.line = 0,
	},
	}, {
	.port = {
		.type = PORT_NETX,
//		.iotype = UPIO_MEM,
//		.membase = (char __iomem *)io_p2v(NETX_PA_UART1),
//		.mapbase = NETX_PA_UART2,
//		.irq = NETX_IRQ_UART2,
		.uartclk = 100000000,
		.fifosize = 96,
		.flags = UPF_BOOT_AUTOCONF,
		.ops = &pic32uart_pops,
		.line = 1,
	},
	}
};

static struct uart_driver pic32uart_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "serial_pic32io",
	.dev_name		= PIC32UART_DEVNAME,
	.major			= PIC32UART_MAJOR,
	.minor			= PIC32UART_MINOR,
	.nr				= ARRAY_SIZE(pic32uart_ports),
};
*/

unsigned int pic32can_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

//	poll_wait(filp, &p_pic32io->can_alarm_wq, wait);
//	if (p_pic32io->pic32can_rcv_rdy) {
//		mask |= POLLIN | POLLRDNORM; /* readable */	
//		printk("pic32can: poll rdy \n");
//	}
		
  //if (left != 1)     mask |= POLLOUT | POLLWRNORM; /* writable */
	return mask;
}

static int pic32can_open(struct inode *node, struct file *instanz)
{
	struct pic32io *pic = p_pic32io;
	unsigned long flags;
	
	printk("pic32can: open()\n");
	if (!pic->pic32can_disabled)
		return 0;

	pic->pic32can_disabled = 0;

	spin_lock_irqsave(&pic->lock, flags);
	enable_irq(OMAP_GPIO_IRQ(pic->pic2omap[2]));
	spin_unlock_irqrestore(&pic->lock, flags);
	return 0;
}

static int pic32can_close(struct inode *node, struct file *instanz)
{
	struct pic32io *pic = p_pic32io;
	unsigned long flags;
	
	if (pic->pic32can_disabled)
		return 0;

	pic->pic32can_disabled = 1;

	spin_lock_irqsave(&pic->lock, flags);
	disable_irq(OMAP_GPIO_IRQ(pic->pic2omap[2]));
	spin_unlock_irqrestore(&pic->lock, flags);
	return 0;
}

static int pic32can_read(struct file *instanz, char *USER, size_t len, loff_t *offset)
{
	struct pic32io *pic = p_pic32io;
	int i;
	int bytes_read = 0;
	
//	printk("pic32can_read() rcv_rdy %d \n", pic->pic32can_rcv_rdy);
	
	while(pic->pic32can_rcv_rdy) {
		for (i=0; i<184; i++) {
			put_user(pic->pic32can_rcv[i], USER++);
			bytes_read++;
		}
		pic->pic32can_rcv_rdy = 0;
	}
   return bytes_read;
}

static int pic32can_write(struct file *file, const char __user *data,
				 size_t len, loff_t *ppos)
{

	struct pic32io *pic = p_pic32io;
	int i;
	
	printk("pic32can_write() snd_rdy %d \n", pic->pic32can_snd_rdy);
	/// wait till the previous is copyied and therefore gone
	if (pic->pic32can_snd_rdy)
		return -EAGAIN;

	pic->pic32can_snd[0] = 1;
	for (i=0; i<len; i++) {
		if (get_user(pic->pic32can_snd[i+1], data++))
			return -EFAULT;
	}
	pic->pic32can_snd_rdy = 1;
    return len;	
}

static int pic32can_ioctl(struct CAN_Regs *inode, struct file *instanz, unsigned int cmd, struct CAN_Regs *USER)
{
	u16 i = 0;
	
	printk("pic32can: ioctl() CMD %d\n", cmd);
	switch(cmd) {	
		case CONFIG_TXBUFFER:
			printk("pic32can: ioctl() had CONFIG_TXBUFFER\n");
			printk("txbsidh %x\ntxbsidl %x\ntxbeid8 %x\ntxbeid0 %x\ntxbdlc %x", USER->txbsidh, USER->txbsidl, 
				USER->txbeid8, USER->txbeid0, USER->txbdlc);
				for(i=6;i<14;i++){
					printk("%x ", USER->txbdata[i-6]);
					}
		break;
		
		case TRANSMIT_SETTINGS:
			printk("pic32can: ioctl() had TRANSMIT_SETTINS\n");
		break;
		
		case TRANSMIT_INIT:
			printk("pic32can: ioctl() had TRANSMIT_INIT\n");
		break;
		
		case TRANSMIT_ABORT:
			printk("pic32can: ioctl() had TRANSMIT_ABORT\n");
		break;
		
		case TRANSMIT_ERROR:
			printk("pic32can: ioctl() had TRANSMIT_ERROR\n");
		break;
		
		case BITTIMING:
			printk("pic32can: ioctl() had BITTIMING \n");
		break;
		
		case READ_A_REG:
			printk("pic32can: ioctl() had READ_A_REG\n");
			put_user("22", "222");
		break;
		
		case RESET:
			printk("pic32can: ioctl() had RESET\n");
		break;
		
		case CONFIG_RXBUFFER:
			printk("pic32can: ioctl() had CONFIG_RXBUFFER\n");
		break;
		
		case CONFIG_MODE:
			printk("pic32can: ioctl() had CONFIG_MODE\n");
		break;
		
		case NORMAL_MODE:
			printk("pic32can: ioctl() had NORMAL_MODE\n");
		break;
		
		case LOOPBACK_MODE:
			printk("pic32can: ioctl() had LOOPBACK_MODE \n");
		break;
		
		case CLEAR_MODE:
			printk("pic32can: ioctl() had CLEAR_MODE\n");
		break;
		
		case FILTER:
			printk("pic32can: ioctl() had FILTER\n");
		break;
		
		case TRANSMIT_BUFFER:
			printk("pic32can: ioctl() had TRANSMIT_BUFFER\n");
		break;
		
		case WRITE_A_REG:
			printk("pic32can: ioctl() had WRITE_A_REG \n");
		break;
		default:
			printk("pic32can: ioctl() had DEFAULT ERRRRRRRRR Unknown IOCTL !!\n");
		break;

	}
	return 0;
}

/** connection point for high level driver */
static struct file_operations fops = {
	.owner   	= THIS_MODULE,
	.open    	= pic32can_open,
	.release	= pic32can_close,
	.read		= pic32can_read,
	.poll		= pic32can_poll,
	.write		= pic32can_write,
	.unlocked_ioctl		= pic32can_ioctl
};

static inline void pic32spi_rcv_clr_buffer(struct pic32io *pic, int id)
{
	int i, z;

	for (z=0; z<6; z++)
		for (i=0; i<FRAME_BYTES; i++)
			pic->rcv_dev[id].frameid[z].dat[i] = 0x00;
			
}

static int __devinit pic32io_probe(struct spi_device *spi)
{
	struct pic32io			*pic;
	struct pic32io_platform_data	*pdata = spi->dev.platform_data;
	int				err;
	int i, ret;
	unsigned long flags;
	
	printk("pic32io_probe: loading...\n");

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_dbg(&spi->dev, "SPI CLK %d Hz?\n",spi->max_speed_hz);
		return -EINVAL;
	}
	
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;	
	err = spi_setup(spi);
	if (err < 0)
		return err;
		
	pic = kzalloc(sizeof(struct pic32io), GFP_KERNEL); //create driver struct

	if (!pic) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	dev_set_drvdata(&spi->dev, pic); //register the local handler struct pic with driver in kernel
	
	spi->dev.power.power_state = PMSG_ON;
	pic->spi = spi;
	
	p_pic32io = pic;

	pic->omap2pic[0] = pdata->omap2pic1;
	pic->omap2pic[1] = pdata->omap2pic2;
	pic->omap2pic[2] = pdata->omap2pic3;
	
	pic->pic2omap[0] = pdata->pic2omap1;
	pic->pic2omap[1] = pdata->pic2omap2;
	pic->pic2omap[2] = pdata->pic2omap3;

	pic->pic32can_rcv_rdy = 0;
	pic->pic32can_snd_rdy = 0;
	
//	printk("pic32io: Have:\n\tP2O1 %d-%d\n\tP2O2 %d-%d\n\tP2O3 %d-%d\n\tO2P1 %d\n\tO2P2 %d\n\tO2P3 %d\n",
//		pic->pic2omap[0], OMAP_GPIO_IRQ(pic->pic2omap[0]), pic->pic2omap[1], OMAP_GPIO_IRQ(pic->pic2omap[1]), pic->pic2omap[2],  OMAP_GPIO_IRQ(pic->pic2omap[2]),
//		pic->omap2pic[0], pic->omap2pic[1], pic->omap2pic[2]);

	/// REQUEST  interrupt
/*	if (request_threaded_irq(OMAP_GPIO_IRQ(pic->pic2omap[0]), pic32io_hard_irq, pic32io_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			spi->dev.driver->name, pic)) { //ISR will receive ts as HANDLER struct
		dev_dbg(&spi->dev, "irq %d busy?\n",OMAP_GPIO_IRQ(pic->pic2omap[0]));
		err = -EBUSY;
		goto err_free_mem;
	}
	/// REQUEST  interrupt
	if (request_threaded_irq(OMAP_GPIO_IRQ(pic->pic2omap[1]), pic32io_hard_irq, pic32io_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			spi->dev.driver->name, pic)) { //ISR will receive ts as HANDLER struct
		dev_dbg(&spi->dev, "irq %d busy?\n", OMAP_GPIO_IRQ(pic->pic2omap[1]));
		err = -EBUSY;
		goto err_free_mem;
	}
*/	/// REQUEST  interrupt
	if (request_threaded_irq(OMAP_GPIO_IRQ(pic->pic2omap[2]), pic32io_hard_irq, pic32io_irq, IRQF_TRIGGER_RISING,
			spi->dev.driver->name, pic)) { //ISR will receive ts as HANDLER struct
		dev_dbg(&spi->dev, "irq %d busy?\n", OMAP_GPIO_IRQ(pic->pic2omap[2]));
		err = -EBUSY;
		goto err_free_mem;
	}
	
	/// DISABLE THE CAN IRQ 
	pic->pic32can_disabled = 1;
	spin_lock_irqsave(&pic->lock, flags);
	disable_irq(OMAP_GPIO_IRQ(pic->pic2omap[2]));
	spin_unlock_irqrestore(&pic->lock, flags);
	
	for (i=0; i<3; i++) {
		gpio_request(pic->omap2pic[i], "omap2pic1");
		if (err) {
			dev_err(&spi->dev, "failed to request  GPIO%d\n",
					pic->omap2pic[i]);
			return err;
		}
		gpio_direction_output(pic->omap2pic[i], 1);
		gpio_set_value(pic->omap2pic[i], 0);
		gpio_export(pic->omap2pic[i], 1);
	}

/*	ret = uart_register_driver(&pic32uart_reg);
	if (ret > 0)
		printk("pic32io: uart_register error %d\n", ret);
		
	for (i = 0; i < PIC32UART_NR; i++) {
		uart_add_one_port(&pic32uart_reg, &pic32uart_ports[i]);
		}
*/
	ret = register_chrdev(PIC32CAN_MAJOR, "pic32can", &fops);
	if (ret > 0)
		printk("pic32io: register chardev error %d\n", ret);
		

	err = sysfs_create_group(&spi->dev.kobj, &pic32io_attr_group);
	if (err)
		goto err_remove_attr;
		
	for (i=0; i<3; i++)
		pic32spi_rcv_clr_buffer(pic, i);
	
	printk("pic32io_probe: probe OK. Device enabled...\n");
	return 0;

err_remove_attr:

	sysfs_remove_group(&spi->dev.kobj, &pic32io_attr_group);
	free_irq(spi->irq, pic);

err_free_mem:
	kfree(pic);
	dev_set_drvdata(&spi->dev, NULL);
	return err;	
}

static int __devexit pic32io_remove(struct spi_device *spi)
{
//	int i;
	struct pic32io		*pic = dev_get_drvdata(&spi->dev); 

	pic32io_suspend(spi, PMSG_SUSPEND);

	gpio_set_value(pic->omap2pic[0], 0);
	gpio_set_value(pic->omap2pic[1], 0);
	gpio_set_value(pic->omap2pic[2], 0);

	free_irq(OMAP_GPIO_IRQ(p_pic32io->pic2omap[0]), pic);
	free_irq(OMAP_GPIO_IRQ(p_pic32io->pic2omap[1]), pic);
	free_irq(OMAP_GPIO_IRQ(p_pic32io->pic2omap[2]), pic);

	unregister_chrdev(PIC32CAN_MAJOR, "pic32can");
	
//	for (i = 0; i < PIC32UART_NR; i++)
//		uart_remove_one_port(&pic32uart_reg, &pic32uart_ports[i]);
		
//	uart_unregister_driver(&pic32uart_reg);

	sysfs_remove_group(&spi->dev.kobj, &pic32io_attr_group);

	kfree(pic); //free the driver struct

	dev_dbg(&spi->dev, "unregistered pic32io\n");
	dev_set_drvdata(&spi->dev, NULL);

	return 0;
	
}

static struct spi_driver pic32io_driver = {
	.driver = {
		.name	= "pic32io",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= pic32io_probe,
	.remove		= __devexit_p(pic32io_remove),
	.suspend	= pic32io_suspend,
	.resume		= pic32io_resume,
};

static int __init pic32io_init(void)
{
	printk("pic32io_init() v0.0.8 -debug\n");
 	return (spi_register_driver(&pic32io_driver));
}
module_init(pic32io_init);

static void __exit pic32io_exit(void)
{
	spi_unregister_driver(&pic32io_driver);
}
module_exit(pic32io_exit);

MODULE_DESCRIPTION("max1233 TouchScreen Driver");
MODULE_LICENSE("GPL");
