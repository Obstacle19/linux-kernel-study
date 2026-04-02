# SPDX-License-Identifier: GPL-2.0-only

# 把 当前目录（$(src)）加入头文件搜索路径
# 如果没有 -I，编译器只会在：当前 .c 文件所在目录、系统目录（如 /usr/include）里找
# -I$(src) 的作用是：告诉编译器：再额外去 $(src) 这个目录找头文件
# $(src) = 当前 Makefile 所在目录（源码目录）
ccflags-y += -I$(src)	# needed for trace events

# 如果：CONFIG_ANDROID_BINDER_IPC_RUST=y，rust_binder.o 被编译进内核
# 如果：CONFIG_ANDROID_BINDER_IPC_RUST=m，编译为模块 .ko
obj-$(CONFIG_ANDROID_BINDER_IPC_RUST) += rust_binder.o

# 把多个 .o 文件链接成 rust_binder.o
# 注意，在 Linux 内核里：.c/.rs → .o 的编译规则不是写在当前 Makefile 里，而是由 Kbuild 的全局规则自动完成的
# 其他 .rs 文件（如 allocation.rs）不是独立编译单元，而是被某一个 .rs 文件包含的模块（module）
rust_binder-y := \
	rust_binder_main.o	\
	rust_binderfs.o		\
	rust_binder_events.o	\
	page_range_helper.o
