
ifeq ($(APP_PARAM), )
    APP_PARAM:=../Makefile.param
    include $(APP_PARAM)
endif

export LC_ALL=C
SHELL:=/bin/bash

CURRENT_DIR := $(shell pwd)

PKG_NAME := mbox_localization
PKG_BIN ?= out

# 源文件
MB_SRC := main.c gcc_phat.c

# 编译、链接参数（尽量简单，依赖交叉工具链自带 sysroot）
MB_CFLAGS  ?= -O2 -Wall -Wextra
MB_LDFLAGS ?= -lasound -lm

# 只要这个 app 目录存在，就始终参与编译
PKG_TARGET := mbox_localization-build

ifeq ($(PKG_BIN),)
$(error ### $(CURRENT_DIR): PKG_BIN is NULL, Please Check !!!)
endif

all: $(PKG_TARGET)
	@echo "build $(PKG_NAME) done"

mbox_localization-build:
	@echo "==> build $(PKG_NAME) in $(CURRENT_DIR)"
	@rm -rf $(PKG_BIN); \
	mkdir -p $(PKG_BIN)/bin;
	$(RK_APP_CROSS)-gcc $(MB_CFLAGS) $(MB_SRC) -o $(PKG_BIN)/bin/mbox_localization $(MB_LDFLAGS)
	$(call MAROC_COPY_PKG_TO_APP_OUTPUT, $(RK_APP_OUTPUT), $(PKG_BIN))

clean:
	@rm -rf $(PKG_BIN)

distclean: clean

info:
	@echo "PKG_NAME = $(PKG_NAME)"
	@echo "SRC      = $(MB_SRC)"


