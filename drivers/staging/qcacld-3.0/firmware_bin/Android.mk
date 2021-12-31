LOCAL_PATH := $(call my-dir)

ifeq ($(ASUS_BUILD_PROJECT),ZS673KS)
$(shell cp $(LOCAL_PATH)/bdwlan.elf $(TARGET_OUT_VENDOR)/firmware/)
$(shell cp $(LOCAL_PATH)/bdwlan.e34 $(TARGET_OUT_VENDOR)/firmware/)
$(shell cp $(LOCAL_PATH)/bdwlan.e0d $(TARGET_OUT_VENDOR)/firmware/)
endif

ifeq ($(ASUS_BUILD_PROJECT),ZS590KS)
$(shell cp $(LOCAL_PATH)/sake_bdwlan.elf $(TARGET_OUT_VENDOR)/firmware/bdwlan.elf)
endif

ifeq ($(ASUS_BUILD_PROJECT),ZS672KS)
$(shell cp $(LOCAL_PATH)/vodka_bdwlan.elf $(TARGET_OUT_VENDOR)/firmware/bdwlan.elf)
$(shell cp $(LOCAL_PATH)/vodka_bdwlan_er1.elf $(TARGET_OUT_VENDOR)/firmware/bdwlan_er1.elf)
endif

ifeq ($(ASUS_BUILD_PROJECT),ZS675KW)
$(shell cp $(LOCAL_PATH)/picasso_bdwlan.e47 $(TARGET_OUT_VENDOR)/firmware/bdwlan.elf)
endif

$(shell cp $(LOCAL_PATH)/WCNSS_qcom_cfg.ini $(TARGET_OUT_VENDOR)/firmware/wlan/qca_cld/wlan/WCNSS_qcom_cfg.ini)

$(shell cp $(LOCAL_PATH)/amss.bin $(TARGET_OUT_VENDOR)/firmware/)
$(shell cp $(LOCAL_PATH)/m3.bin $(TARGET_OUT_VENDOR)/firmware/)
$(shell cp $(LOCAL_PATH)/Data.msc $(TARGET_OUT_VENDOR)/firmware/)
$(shell cp $(LOCAL_PATH)/regdb.bin $(TARGET_OUT_VENDOR)/firmware/)
