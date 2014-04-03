# EDR Enhanced Data Acquisition Makefile
obj-$(CONFIG_EDREDAQ_UDAQLITE)		+= udaqlite/
obj-$(CONFIG_EDREDAQ_CLASS)		+= class/

SUBDIRS = class udaqlite
FILES = INSTALL README Kconfig Makefile
RELEASE_VERSION = 2_1_20

DATE = $(shell date +%d_%m_%Y)

all:
	@for dir in $(SUBDIRS); do \
	    $(MAKE) -C $$dir; \
	done

clean:
	@for dir in $(SUBDIRS); do \
	    $(MAKE) -C $$dir clean; \
	done

install:
	@for dir in $(SUBDIRS); do \
	    $(MAKE) -C $$dir install; \
	done
	depmod -a

uninstall:
	@for dir in $(SUBDIRS); do \
	    $(MAKE) -C $$dir uninstall; \
	done
	depmod -a

backup:
	tar -cvf edre_linux_drivers_backup_V$(RELEASE_VERSION)_D$(DATE).tar *
	gzip edre_linux_drivers_backup_V$(RELEASE_VERSION)_D$(DATE).tar

release:
	tar -cvf edre_linux_drivers_$(RELEASE_VERSION).tar $(SUBDIRS) $(FILES)
	gzip edre_linux_drivers_$(RELEASE_VERSION).tar
