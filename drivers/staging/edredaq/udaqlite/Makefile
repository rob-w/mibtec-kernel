# (C) 2005 - 2009 Eagle Technology
# www.eagledaq.com - eagle@eagle.co.za
# USB MicroDAQLite Device Driver Makefile

O_TARGET	:=

EXTRA_CFLAGS += -I/usr/include -DEXPORT_SYMTAB
DRIVERS_DIR = /lib/modules/${shell uname -r}/kernel/drivers
KERNEL_DIR = /lib/modules/${shell uname -r}/build
INSTALL = /usr/bin/install
PWD	:= $(shell pwd)
DEVICE = udaqlite
DRIVER = $(DEVICE)drv.ko
VERSION = 1_1_8
SRCNAME =	$(DEVICE)_$(VERSION)
SRCFILES = $(shell find . \( -not -name . -prune -type f \) -print | grep -v -e CVS -e "\.tar\.gz" -e "\/\." -e releases -e BitKeeper -e SCCS -e test/sys | sort )

ifneq ($(KERNELRELEASE),)
obj-m	:= udaqlitedrv.o 
udaqlitedrv-objs := udaqlite.o usbio.o deviceio.o ioctl.o buffer.o fxloader.o
else
default:
	cp ../class/Module.symvers .
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) modules
endif

install:
	$(INSTALL) -d $(DRIVERS_DIR)/edredaq/$(DEVICE)
	$(INSTALL) -m 777 $(DRIVER) $(DRIVERS_DIR)/edredaq/$(DEVICE)

uninstall:
	rm -rf $(DRIVERS_DIR)/edredaq/$(DEVICE)

clean:
	@echo "Deleting temp files and directories..."
	@-find . \( -name '.*' -not -name '.' \) -print | xargs rm -rf
	@-find . \( -name '*.[aos]' \) -print | xargs rm -rf
	@-find . \( -name '*.ko' \) -print | xargs rm -rf
	@-find . \( -not -type f -not -name '.' \) -print | xargs rm -rf
	@-find . \( -name '*.*~' \) -print | xargs rm -rf
	@echo "Done"

release:
	@echo "Building release package..."
	@-rm -rf $(SRCNAME)	
	@-rm -rf *.tar, *.gz
	@mkdir $(SRCNAME)
	@for file in $(SRCFILES); do  \
		cp -p $$file $(SRCNAME)/$$file; \
	done 
	@tar -cf $(SRCNAME).tar $(SRCNAME)
	@gzip -9 $(SRCNAME).tar
	@rm -rf $(SRCNAME)
	@echo "Built $(SRCNAME).tar.gz"


