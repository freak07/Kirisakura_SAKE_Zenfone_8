
# +++ ASUS_BSP : add for asus dtbo
$(warning [Build] build dtbo CONFIG_BUILD_ARM64_DT_OVERLAY = $(CONFIG_BUILD_ARM64_DT_OVERLAY))
$(warning [Build] build dtbo ASUS_BUILD_PROJECT = $(ASUS_BUILD_PROJECT))
$(warning [Build] build dtbo CONFIG_ARCH_QTI_VM = $(CONFIG_ARCH_QTI_VM))
# --- ASUS_BSP : add for asus dtbo

ifneq ($(CONFIG_ARCH_QTI_VM), y)
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)

# +++ ASUS_BSP : add for asus dtbo
ifneq (,$(filter ZS673KS,$(ASUS_BUILD_PROJECT)))
$(warning build dtbo for ZS673KS...)

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                ZS673KS-EVB-overlay.dtbo \
                ZS673KS-EVB2-overlay.dtbo \
                ZS673KS-SR1-overlay.dtbo \
                ZS673KS-ER1-overlay.dtbo \
                ZS673KS-ER2-overlay.dtbo \
                ZS673KS-PR1-overlay.dtbo \
                ZS673KS-MP-overlay.dtbo \

lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-ER1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-ER2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-PR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
ZS673KS-MP-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb

else ifneq (,$(filter PICASSO,$(ASUS_BUILD_PROJECT)))

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                PICASSO-EVB-overlay.dtbo \
                PICASSO-EVB2-overlay.dtbo \
                PICASSO-SR1-overlay.dtbo \
                PICASSO-ER1-overlay.dtbo \
                PICASSO-ER2-overlay.dtbo \
                PICASSO-PR1-overlay.dtbo \
                PICASSO-PR2-overlay.dtbo \
                PICASSO-MP-overlay.dtbo \

PICASSO-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-ER1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-ER2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-PR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-PR2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
PICASSO-MP-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb

else ifneq (,$(filter SAKE,$(ASUS_BUILD_PROJECT)))
$(warning build dtbo for SAKE...)

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                SAKE-EVB-overlay.dtbo \
                SAKE-EVB2-overlay.dtbo \
                SAKE-SR1-overlay.dtbo \
                SAKE-SR2-overlay.dtbo \
                SAKE-ER1-overlay.dtbo \
                SAKE-ER2-overlay.dtbo \
                SAKE-PR-overlay.dtbo \
                SAKE-MP-overlay.dtbo \

lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-SR2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-ER1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-ER2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-PR-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
SAKE-MP-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb	
else ifneq (,$(filter VODKA,$(ASUS_BUILD_PROJECT)))
$(warning build dtbo for VODKA...)

dtbo-$(CONFIG_ARCH_LAHAINA) += \
                VODKA-EVB-overlay.dtbo \
                VODKA-EVB2-overlay.dtbo \
                VODKA-SR1-overlay.dtbo \
                VODKA-SR2-overlay.dtbo \
                VODKA-ER1-overlay.dtbo \
                VODKA-ER2-overlay.dtbo \
                VODKA-PR-overlay.dtbo \
                VODKA-MP-overlay.dtbo \

lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-EVB-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-EVB2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-SR1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-SR2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-ER1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-ER2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-PR-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
VODKA-MP-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
# --- ASUS_BSP : add for asus dtbo
else
dtbo-$(CONFIG_ARCH_LAHAINA) += lahaina-rumi-overlay.dtbo \
	lahaina-mtp-overlay.dtbo \
	lahaina-mtp-hsp-overlay.dtbo \
	lahaina-cdp-overlay.dtbo \
	lahaina-qrd-overlay.dtbo \
	lahaina-atp-overlay.dtbo \
	lahaina-hdk-overlay.dtbo \
	lahaina-hhg-overlay.dtbo \
	lahaina-hhg-hsp-pro-overlay.dtbo \
	lahaina-qrd-hsp-overlay.dtbo \
	lahaina-qrd-module-overlay.dtbo \
	lahainap-mtp-overlay.dtbo \
	lahainap-cdp-overlay.dtbo \
	lahainap-atp-overlay.dtbo \
	lahainap-qrd-overlay.dtbo \
	lahaina-cdp-v2.2-overlay.dtbo \
	lahaina-mtp-v2-overlay.dtbo \
	lahaina-mtp-v2.1-overlay.dtbo \
	lahaina-mtp-hsp-pro-overlay.dtbo \
	lahaina-cdp-hsp-pro-overlay.dtbo \
	lahaina-qrd-hsp-pro-overlay.dtbo

lahaina-rumi-overlay.dtbo-base := lahaina.dtb
lahaina-mtp-hsp-overlay.dtbo-base := lahaina.dtb
lahaina-mtp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-cdp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-atp-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-hdk-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-hhg-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-hhg-hsp-pro-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-qrd-hsp-overlay.dtbo-base := lahaina.dtb
lahaina-qrd-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb
lahaina-qrd-module-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb lahaina-v2.1.dtb

lahainap-mtp-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahainap-cdp-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahainap-atp-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahainap-qrd-overlay.dtbo-base := lahainap.dtb lahainap-v2.dtb lahainap-v2.1.dtb
lahaina-cdp-v2.2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-mtp-v2-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-mtp-v2.1-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-mtp-hsp-pro-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-cdp-hsp-pro-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
lahaina-qrd-hsp-pro-overlay.dtbo-base := lahaina.dtb lahaina-v2.dtb \
					lahaina-v2.1.dtb
endif
else
dtb-$(CONFIG_ARCH_LAHAINA) += lahaina-rumi.dtb \
	lahaina-mtp-hsp.dtb \
	lahaina-mtp.dtb \
	lahaina-cdp.dtb \
	lahaina-atp.dtb \
	lahaina-hdk.dtb \
	lahaina-qrd-hsp.dtb \
	lahaina-qrd.dtb \
	lahaina-qrd-module.dtb \
	lahainap-mtp.dtb \
	lahainap-cdp.dtb \
	lahainap-atp.dtb \
	lahainap-qrd.dtb \
	lahaina-v2-mtp.dtb \
	lahaina-v2-cdp.dtb \
	lahaina-v2-atp.dtb \
	lahaina-v2-hdk.dtb \
	lahaina-v2-qrd.dtb \
	lahaina-v2-qrd-module.dtb \
	lahainap-v2-mtp.dtb \
	lahainap-v2-cdp.dtb \
	lahainap-v2-atp.dtb \
	lahainap-v2-qrd.dtb \
	lahaina-cdp-v2.2.dtb \
	lahaina-v2.1-mtp.dtb \
	lahaina-v2.1-cdp.dtb \
	lahaina-v2.1-atp.dtb \
	lahaina-v2.1-hdk.dtb \
	lahaina-v2.1-hhg.dtb \
	lahaina-v2.1-hhg-hsp-pro.dtb \
	lahaina-v2.1-qrd.dtb \
	lahaina-v2.1-qrd-module.dtb \
	lahainap-v2.1-mtp.dtb \
	lahainap-v2.1-cdp.dtb \
	lahainap-v2.1-atp.dtb \
	lahainap-v2.1-qrd.dtb \
	lahaina-mtp-v2.dtb \
	lahaina-mtp-v2.1.dtb \
	lahaina-v2.1-mtp-hsp-pro.dts \
	lahaina-v2.1-cdp-hsp-pro.dts \
	lahaina-v2.1-qrd-hsp-pro.dts
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SCUBA) += scuba-iot-idp-overlay.dtbo

scuba-iot-idp-overlay.dtbo-base := scuba-iot.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_HOLI) += holi-rumi-overlay.dtbo \
	holi-mtp-overlay.dtbo \
	holi-cdp-overlay.dtbo \
	holi-cdp-lcd-overlay.dtbo \
	holi-qrd-overlay.dtbo \
	holi-atp-overlay.dtbo \
	holi-mtp-usbc-overlay.dtbo

holi-rumi-overlay.dtbo-base := holi.dtb
holi-mtp-overlay.dtbo-base := holi.dtb
holi-cdp-overlay.dtbo-base := holi.dtb
holi-cdp-lcd-overlay.dtbo-base := holi.dtb
holi-qrd-overlay.dtbo-base := holi.dtb
holi-atp-overlay.dtbo-base := holi.dtb
holi-mtp-usbc-overlay.dtbo-base := holi.dtb
else
dtb-$(CONFIG_ARCH_HOLI) += holi-rumi.dtb \
	holi-mtp.dtb \
	holi-cdp.dtb \
	holi-cdp-lcd.dtb \
	holi-qrd.dtb \
	holi-atp.dtb \
	holi-mtp-usbc.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SHIMA) +=    \
	shima-rumi-overlay.dtbo \
	shima-idp-overlay.dtbo  \
	shima-idp-120fps-overlay.dtbo \
	shima-atp-overlay.dtbo \
	shima-qrd-overlay.dtbo \
	shima-idps-digital-mics-overlay.dtbo \
	shima-idps-digital-mics-120fps-overlay.dtbo

shima-rumi-overlay.dtbo-base := shima.dtb
shima-idp-overlay.dtbo-base := shima.dtb
shima-idp-120fps-overlay.dtbo-base := shima.dtb
shima-atp-overlay.dtbo-base := shima.dtb
shima-qrd-overlay.dtbo-base := shima.dtb
shima-idps-digital-mics-overlay.dtbo-base := shima.dtb
shima-idps-digital-mics-120fps-overlay.dtbo-base := shima.dtb
else
dtb-$(CONFIG_ARCH_SHIMA) += shima-rumi.dtb  \
			    shima-idp.dtb   \
			    shima-idp-120fps.dtb  \
			    shima-atp.dtb \
			    shima-qrd.dtb \
			    shima-idps-digital-mics.dtb \
			    shima-idps-digital-mics-120fps.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_YUPIK) +=    \
	yupik-rumi-overlay.dtbo \
	yupik-idp-overlay.dtbo \
	yupik-atp-overlay.dtbo \
	yupik-qrd-overlay.dtbo \
	yupik-idps-digital-mics-overlay.dtbo \
	yupik-idps-amoled-overlay.dtbo \
	yupikp-idp-overlay.dtbo \
	yupikp-atp-overlay.dtbo \
	yupikp-qrd-overlay.dtbo \
	yupikp-idps-digital-mics-overlay.dtbo \
	yupikp-idps-amoled-overlay.dtbo \
	yupik-iot-idp-overlay.dtbo \
	yupik-iot-qrd-overlay.dtbo \
	yupik-iot-hsp-overlay.dtbo \
	yupik-iot-idps-amoled-overlay.dtbo \
	yupikp-iot-idp-overlay.dtbo \
	yupikp-iot-qrd-overlay.dtbo \
	yupikp-iot-hsp-overlay.dtbo \
	yupikp-iot-idps-amoled-overlay.dtbo


yupik-rumi-overlay.dtbo-base := yupik.dtb
yupik-idp-overlay.dtbo-base := yupik.dtb
yupik-atp-overlay.dtbo-base := yupik.dtb
yupik-qrd-overlay.dtbo-base := yupik.dtb
yupik-idps-digital-mics-overlay.dtbo-base := yupik.dtb
yupik-idps-amoled-overlay.dtbo-base := yupik.dtb
yupikp-idp-overlay.dtbo-base := yupikp.dtb
yupikp-atp-overlay.dtbo-base := yupikp.dtb
yupikp-qrd-overlay.dtbo-base := yupikp.dtb
yupikp-idps-digital-mics-overlay.dtbo-base := yupikp.dtb
yupikp-idps-amoled-overlay.dtbo-base := yupikp.dtb
yupik-iot-idp-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-qrd-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-hsp-overlay.dtbo-base := yupik-iot.dtb
yupik-iot-idps-amoled-overlay.dtbo-base := yupik-iot.dtb
yupikp-iot-idp-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-qrd-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-hsp-overlay.dtbo-base := yupikp-iot.dtb
yupikp-iot-idps-amoled-overlay.dtbo-base := yupikp-iot.dtb
else
dtb-$(CONFIG_ARCH_YUPIK) += yupik-rumi.dtb \
	yupik-idp.dtb \
	yupik-atp.dtb \
	yupik-qrd.dtb \
	yupik-idps-digital-mics.dtb \
	yupik-idps-amoled.dtb \
	yupikp-idp.dtb \
	yupikp-atp.dtb \
	yupikp-qrd.dtb \
	yupikp-idps-digital-mics.dtb \
	yupikp-idps-amoled.dtb \
	yupik-iot-idp.dtb \
	yupik-iot-qrd.dtb \
	yupik-iot-hsp.dtb \
	yupik-iot-idps-amoled.dtb \
	yupikp-iot-idp.dtb \
	yupikp-iot-qrd.dtb \
	yupikp-iot-hsp.dtb \
	yupikp-iot-idps-amoled.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_MONACO) +=    \
	monaco-rumi-overlay.dtbo

monaco-rumi-overlay.dtbo-base := monaco.dtb
else
dtb-$(CONFIG_ARCH_MONACO) += monaco-rumi.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_SA8155) += \
		sa8155-adp-star-overlay.dtbo \
		sa8155p-adp-star-overlay.dtbo \
		sa8155-v2-adp-air-overlay.dtbo \
		sa8155p-v2-adp-air-overlay.dtbo

sa8155-adp-star-overlay.dtbo-base := sa8155.dtb sa8155-v2.dtb
sa8155p-adp-star-overlay.dtbo-base := sa8155p.dtb sa8155p-v2.dtb
sa8155-v2-adp-air-overlay.dtbo-base := sa8155.dtb sa8155-v2.dtb
sa8155p-v2-adp-air-overlay.dtbo-base := sa8155p.dtb sa8155p-v2.dtb
else
dtb-$(CONFIG_ARCH_SA8155)       += sa8155-adp-star.dtb \
	sa8155p-adp-star.dtb \
	sa8155-v2-adp-star.dtb \
	sa8155p-v2-adp-star.dtb \
	sa8155-v2-adp-air.dtb \
	sa8155p-v2-adp-air.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
        dtbo-$(CONFIG_ARCH_SA6155) += \
		sa6155-adp-star-overlay.dtbo \
		sa6155p-adp-star-overlay.dtbo \
		sa6155p-adp-star-lpass-overlay.dtbo \
		sa6155-adp-air-overlay.dtbo \
		sa6155p-adp-air-overlay.dtbo \
		sa6155p-v2-adp-star-overlay.dtbo \
		sa6155p-v2-adp-air-overlay.dtbo \
		sa6155p-v2-adp-air-lpass-overlay.dtbo

sa6155-adp-star-overlay.dtbo-base := sa6155.dtb
sa6155p-adp-star-overlay.dtbo-base := sa6155p.dtb
sa6155p-adp-star-lpass-overlay.dtbo-base :=sa6155p.dtb
sa6155p-v2-adp-star-overlay.dtbo-base := sa6155p.dtb
sa6155-adp-air-overlay.dtbo-base := sa6155.dtb
sa6155p-adp-air-overlay.dtbo-base := sa6155p.dtb
sa6155p-v2-adp-air-overlay.dtbo-base := sa6155p.dtb
sa6155p-v2-adp-air-lpass-overlay.dtbo-base := sa6155p.dtb
else
dtb-$(CONFIG_ARCH_SA6155)       += sa6155-adp-star.dtb \
	sa6155p-adp-star.dtb \
	sa6155p-adp-star-lpass.dtb \
	sa6155-adp-air.dtb \
	sa6155p-adp-air.dtb \
	sa6155p-v2-adp-star.dtb \
	sa6155p-v2-adp-air.dtb \
	sa6155p-v2-adp-air-lpass.dtb
endif

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_SA8195) += \
		sa8195p-adp-star-overlay.dtbo \
		sa8195p-v2-adp-air-overlay.dtbo

sa8195p-adp-star-overlay.dtbo-base := sa8195p.dtb
sa8195p-v2-adp-air-overlay.dtbo-base := sa8195p.dtb
else
dtb-$(CONFIG_ARCH_SA8195)	+= sa8195p-adp-star.dtb \
	sa8195p-v2-adp-air.dtb
endif
endif

ifeq ($(CONFIG_ARCH_LAHAINA), y)
ifeq ($(CONFIG_ARCH_QTI_VM), y)
dtb-$(CONFIG_ARCH_QTI_VM) += lahaina-vm-mtp.dtb \
	lahaina-vm-cdp.dtb \
	lahaina-vm-qrd.dtb \
	lahaina-vm-mtp-v2.dtb \
	lahaina-vm-mtp-v2.1.dtb
endif
endif

dtb-$(CONFIG_ARCH_SDXLEMUR) += sdxlemur-rumi.dtb \
	sdxlemur-cdp.dtb \
	sdxlemur-mtp.dtb \
	sdxlemur-mtp-mbb.dtb \
	sdxlemur-mtp-cpe.dtb \
	sdxlemur-mtp-mbb-emmc.dtb \
	sdxlemur-mtp-mbb-ntn3-pcie.dtb \
	sdxlemur-mtp-mbb-pcie-ep.dtb \
	sdxlemur-mtp-mbb-pcie-rc-ep.dtb \
	sdxlemur-mtp-mbb.dtb \
	sdxlemur-rcm-pcie-ep.dtb \
	sdxlemur-rcm-rc.dtb \
	sdxlemur-v2-cdp.dtb \
	sdxlemur-v2-mtp-mbb.dtb \
	sdxlemur-v2-mtp-cpe.dtb \
	sdxlemur-v2-mtp-mbb-emmc.dtb \
	sdxlemur-v2-mtp-mbb-ntn3-pcie.dtb \
	sdxlemur-v2-mtp-mbb-pcie-ep.dtb \
	sdxlemur-v2-mtp-mbb-pcie-rc-ep.dtb \
	sdxlemur-v2-rcm-pcie-ep.dtb \
	sdxlemur-v2-rcm-rc.dtb


ifeq ($(CONFIG_ARCH_SHIMA), y)
ifeq ($(CONFIG_ARCH_QTI_VM), y)
dtb-$(CONFIG_ARCH_QTI_VM) += shima-vm-rumi.dtb \
	shima-vm-atp.dtb \
	shima-vm-idp.dtb \
	shima-vm-qrd.dtb
endif
endif

dtb-$(CONFIG_ARCH_SDXNIGHTJAR) += sdxnightjar-mtp.dtb \
	sdxnightjar-cdp.dtb \
	sdxnightjar-qrd.dtb \
	sdxnightjar-dualwifi-mtp.dtb

ifeq ($(CONFIG_ARCH_YUPIK), y)
ifeq ($(CONFIG_ARCH_QTI_VM), y)
dtb-$(CONFIG_ARCH_QTI_VM) += yupik-vm-rumi.dtb \
	yupik-vm-atp.dtb \
	yupik-vm-idp.dtb \
	yupik-vm-qrd.dtb
endif
endif

dtb-$(CONFIG_QTI_QUIN_GVM) += sa8155-vm-la.dtb \
	sa8155-vm-lv.dtb \
	sa8155-vm-la-mt.dtb \
	sa8155-vm-lv-mt.dtb \
	sa8195-vm-la.dtb \
	sa8195-vm-lv.dtb \
	sa8195-vm-la-mt.dtb \
	sa8195-vm-lv-mt.dtb \
	sa8195-vm-lv-lxc.dtb \
	sa6155p-vm-la.dtb

ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_DIREWOLF) += direwolf-ivi-rumi-overlay.dtbo \
		direwolf-ivi-adp-star-overlay.dtbo \
		direwolf-adas-rumi-overlay.dtbo \
		direwolf-adas-adp-star-overlay.dtbo

direwolf-ivi-rumi-overlay.dtbo-base := direwolf-ivi.dtb
direwolf-ivi-adp-star-overlay.dtbo-base := direwolf-ivi.dtb
direwolf-adas-rumi-overlay.dtbo-base := direwolf-adas.dtb
direwolf-adas-adp-star-overlay.dtbo-base := direwolf-adas.dtb
else
dtb-$(CONFIG_ARCH_DIREWOLF) += direwolf-ivi-rumi.dtb \
	direwolf-ivi-adp-star.dtb \
	direwolf-adas-rumi.dtb \
	direwolf-adas-adp-star.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo
