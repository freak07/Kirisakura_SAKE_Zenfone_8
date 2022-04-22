obj-n	+=ASH_ATTR/ 
obj-n	+=ASH_Report/
obj-n	+=ASH_Factory/
obj-n	+=ASH_GPIO/
obj-n	+=ASH_Hardware/
obj-n	+=ASH_Algo/

obj-n 	+=ASH_testDriver/


ccflags-y += -DDEFAULT_SYMBOL_NAMESPACE=ANDROID_GKI_VFS_EXPORT_ONLY
ccflags-y += -DANDROID_GKI_VFS_EXPORT_ONLY=VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver

obj-m	:= sensors_vcnl36866.o
sensors_vcnl36866-y += ASH_Algo/ALSPSsensor.o
sensors_vcnl36866-y += ASH_ATTR/lsensor_ATTR.o ASH_ATTR/psensor_ATTR.o ASH_ATTR/ASH_ATTR.o
sensors_vcnl36866-y += ASH_Report/psensor_Report.o ASH_Report/LwithRGBsensor_Report.o
sensors_vcnl36866-y += ASH_Factory/psensor_Factory.o ASH_Factory/lsensor_Factory.o
sensors_vcnl36866-y += ASH_GPIO/ALSPSsensor_GPIO.o
sensors_vcnl36866-y += ASH_Hardware/ASH_Hardware.o ASH_Hardware/ALSPSsensor_Hardware/ALSPSsensor_Hardware.o ASH_Hardware/ALSPSsensor_Hardware/vcnl36866/vcnl36866.o

ifeq ($(CONFIG_MACH_ASUS_SAKE),y)
sensors_vcnl36866-y += ASH_Hardware/ALSPSsensor_Hardware/ams_tmd2755/ams_tmd2755.o
sensors_vcnl36866-y += ASH_Hardware/ALSPSsensor_Hardware/ams_tmd2755/ams_tmd2755_prox.o
sensors_vcnl36866-y += ASH_Hardware/ALSPSsensor_Hardware/ams_tmd2755/ams_tmd2755_als.o
sensors_vcnl36866-y += ASH_Hardware/ALSPSsensor_Hardware/ams_tmd2755/ams_i2c.o
endif

ifeq ($(CONFIG_MACH_ASUS_VODKA),y)
sensors_vcnl36866-y += ASH_Algo/ALSPSsensor_2nd.o
sensors_vcnl36866-y += ASH_ATTR/lsensor_2nd_ATTR.o ASH_ATTR/psensor_2nd_ATTR.o
sensors_vcnl36866-y += ASH_GPIO/ALSPSsensor_GPIO_2nd.o
sensors_vcnl36866-y +=  ASH_Hardware/ALSPSsensor_Hardware/ALSPSsensor_Hardware_2nd.o ASH_Hardware/ALSPSsensor_Hardware/vcnl36866_2nd/vcnl36866_2nd.o 
endif
