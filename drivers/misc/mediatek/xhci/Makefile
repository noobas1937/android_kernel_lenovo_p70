#
# Makefile for USB Host Lib
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/usb/host
ccflags-y += -I$(srctree)/drivers/usb/core

ifeq ($(CONFIG_USB_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_HDMI),"Sii8348")
	ccflags-y += 	-DMHL_SII8348
endif

obj-y	+= xhci-mtk-power.o xhci-mtk-scheduler.o xhci-mtk.o xhci-mtk-attrs.o