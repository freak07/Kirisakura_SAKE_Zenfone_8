
# SPDX-License-Identifier: GPL-2.0-only
#obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
obj-m                           += icm206xx.o
obj-y                           += ASH/
obj-y                           += laser/
obj-$(CONFIG_MACH_ASUS_ZS673KS)	+= snt8100fsr-linux/

ifeq ($(CONFIG_MACH_ASUS_SAKE),y)
KBUILD_CFLAGS += -DCONFIG_TMD2755_FLAG=1
KBUILD_CFLAGS += -DONE_PL_CHIP
KBUILD_CFLAGS += -DPSENSOR_CAL24
endif

ifeq ($(CONFIG_MACH_ASUS_PICASSO),y)
KBUILD_CFLAGS += -DONE_PL_CHIP
endif

ifeq ($(CONFIG_MACH_ASUS_ZS673KS),y)
KBUILD_CFLAGS += -DONE_PL_CHIP
KBUILD_CFLAGS += -DALSP_GPIO_NO_PULL
endif

ifeq ($(CONFIG_MACH_ASUS_VODKA),y)
KBUILD_CFLAGS += -DPSENSOR_CAL24
KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_PERF=1
KBUILD_CFLAGS += -DRECOVERY_PSENSOR
endif


#ifeq ($(ASUS_FTM), y)
#KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_NON_PERF=1
#else #Shipping
#KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_PERF=1
#endif #####ifdef ASUS_FTM

