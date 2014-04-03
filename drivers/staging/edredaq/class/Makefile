# (C) 2005-2011 Eagle Technology
# www.eagledaq.com - eagle@eagle.co.za
# EDR Enhanced Data Acquisition Class Driver
# Written by Jan Zwiegers, jan@eagle.co.za

O_TARGET	:=

EXTRA_CFLAGS += -I/usr/edre/include
DRIVERS_DIR = /lib/modules/${shell uname -r}/kernel/drivers
KERNEL_DIR = /lib/modules/${shell uname -r}/build
INSTALL = /usr/bin/install
PWD	:= $(shell pwd)
DEVICE = edredaqdrv
DRIVER = $(DEVICE).ko
VERSION = 2_0_2
SRCNAME =	$(DEVICE)_$(VERSION)
SRCFILES = $(shell find . \( -not -name . -prune -type f \) -print | grep -v -e CVS -e "\.tar\.gz" -e "\/\." -e releases -e BitKeeper -e SCCS -e test/sys | sort )

ifneq ($(KERNELRELEASE),)
obj-m	:= edredaqdrv.o
edredaqdrv-objs := edreclass.o edredev.o edreproc.o edresysfs.o
else
default:
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
	rm -rf *.symvers
	rm -rf *.mod.c
	rm -rf *.gz
	rm -rf *.tar
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


