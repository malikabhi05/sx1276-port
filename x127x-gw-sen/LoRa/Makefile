UNAME := $(shell uname)
ifeq (MINGW, $(findstring MINGW, $(UNAME)))
	export ZEPHYR_BASE = $(shell sh -c "pwd -W")/zephyr
else
	export ZEPHYR_BASE = $(CURDIR)/zephyr
endif

BOARD = quark_se_c1000_devboard
ARCH = x86
KERNEL_TYPE = unified

CONF_FILE = prj.conf

include ${ZEPHYR_BASE}/Makefile.test
