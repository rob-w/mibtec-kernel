/* linux/spi/pic32io.h */

/* 
 */
 

struct pic32io_platform_data {
	u16	version;
	u16	omap2pic1; //					= IGEP3_GPIO_IGEPIRQ1,
	u16	omap2pic2; //					= IGEP3_GPIO_IGEPIRQ2,
	u16	omap2pic3; //					= IGEP3_GPIO_IGEPIRQ3,
	u16	pic2omap1; //					= IGEP3_GPIO_PICIRQ1,
	u16	pic2omap2; //					= IGEP3_GPIO_PICIRQ2,
	u16	pic2omap3; //					= IGEP3_GPIO_PICIRQ3,
};

struct CAN_Regs{
	__u8 addr;
	__u8 databyte;
	__u8 buffernumber;
	__u8 filternumber;
	__u8 bit_timing;
	__u8 bfpctrl;
	__u8 txrtsctrl;
	__u8 canstat;
	__u8 canctrl;
	__u8 tec;
	__u8 rec;
	__u8 cnf3;
	__u8 cnf2;
	__u8 cnf1;
	__u8 caninte;
	__u8 canintf;
	__u8 eflg;
	__u8 txbctrl;
	__u8 txbsidh;
	__u8 txbsidl;
	__u8 txbeid8;
	__u8 txbeid0;
	__u8 txbdlc;
	__u8 txbdata[8];
	__u8 rxbctrl;
	__u8 rxbsidh;
	__u8 rxbsidl;
	__u8 rxbeid8;
	__u8 rxbeid0;
	__u8 rxbdlc;
	__u8 rxf0sidh;
	__u8 rxf0sidl;
	__u8 rxf1sidh;
	__u8 rxf1sidl;
	__u8 rxm0sidh;
	__u8 rxm0sidl;
	__u8 rxm1sidh;
	__u8 rxm1sidl;
	__u8 rxbdata[8];
};


