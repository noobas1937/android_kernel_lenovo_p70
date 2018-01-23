include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  focaltech_driver.o focaltech_ex_fun.o focaltech_ctl.o fts_alsps.o
ifeq ($(ARCH),arm)
obj-y	+=  32bit/ft_gesture_lib.a
else
obj-y	+=  64bit/ft_gesture_lib.a
endif
