# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build.
#
#   make KDIR=/path/to/kernel/build ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

.PHONY: all clean
