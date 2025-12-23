ifeq ($(SYSDRV_PARAM), )
    SYSDRV_PARAM:=../../../Makefile.param
    include $(SYSDRV_PARAM)
endif

export LC_ALL=C
SHELL:=/bin/bash

CURRENT_DIR := $(shell pwd)
PKG_BIN := out
# 计算sysdrv目录路径
SYSDRV_DIR := $(CURRENT_DIR)/../../..
# 如果SYSDRV_DIR_OUT_ROOTFS未定义，则计算它
ifeq ($(SYSDRV_DIR_OUT_ROOTFS),)
SYSDRV_DIR_OUT_ROOTFS := $(SYSDRV_DIR)/out/rootfs_$(SYSDRV_LIB_TYPE)_$(CHIP)
endif
# 使用相对路径指向rtk_hciattach源文件目录
RTK_HCIATTACH_SRC_DIR := $(SYSDRV_DIR)/source/rtk_hciattach_src

PKG_CONF_OPTS += $(SYSDRV_CROSS_CFLAGS) -Os
PKG_CONF_OPTS += -I$(RTK_HCIATTACH_SRC_DIR)

ifeq ($(PKG_USE_THUMB2),YES)
PKG_CONF_OPTS += -mthumb -Wa,-mimplicit-it=thumb -mthumb-interwork
endif

# 源文件列表
RTK_HCIATTACH_SRCS := \
	$(RTK_HCIATTACH_SRC_DIR)/hciattach.c \
	$(RTK_HCIATTACH_SRC_DIR)/hciattach_rtk.c \
	$(RTK_HCIATTACH_SRC_DIR)/rtb_fwc.c \
	$(RTK_HCIATTACH_SRC_DIR)/hciattach_h4.c

all:
	@echo "Building rtk_hciattach..."
	rm -rf $(PKG_BIN);
	mkdir -p $(CURRENT_DIR)/$(PKG_BIN)/usr/bin;
	$(SYSDRV_CROSS)-gcc $(PKG_CONF_OPTS) \
		$(RTK_HCIATTACH_SRCS) \
		-o $(PKG_BIN)/usr/bin/rtk_hciattach
	$(SYSDRV_CROSS)-strip -s $(PKG_BIN)/usr/bin/rtk_hciattach
	$(call MAROC_COPY_PKG_TO_SYSDRV_OUTPUT, $(SYSDRV_DIR_OUT_ROOTFS), $(CURRENT_DIR)/$(PKG_BIN))

clean: distclean

distclean:
	-rm -rf $(CURRENT_DIR)/$(PKG_BIN)

