dtb-$(CONFIG_MACH_PXA1908DKB) += j1-r00.dtb pxa1908-dkb.dtb
dtb-$(CONFIG_MACH_J1LTE) += pxa1908-j1lte-15.dtb \
				pxa1908-j1lte-00.dtb \
				pxa1908-j1lte-01.dtb \
				pxa1908-j1lte-02.dtb \
				pxa1908-j1lte-03.dtb \
				pxa1908-j1lte-04.dtb \
				pxa1908-j1lte-05.dtb \
				pxa1908-j1lte-06.dtb \
				pxa1908-j1lte-07.dtb \
				pxa1908-j1ltechncmcc-00.dtb \
				pxa1908-j1ltechncmcc-01.dtb \
				pxa1908-j1ltechncmcc-02.dtb
dtb-$(CONFIG_MACH_XCOVER3LTE) += pxa1908-xcover3lte-00.dtb \
				pxa1908-xcover3lte-03.dtb \
				pxa1908-xcover3lte-04.dtb
dtb-$(CONFIG_MACH_J1NLTE) += pxa1908-j1nlte-02.dtb \
				pxa1908-j1nlte-03.dtb \
				pxa1908-j1nlte-04.dtb \
				pxa1908-j1nlte-05.dtb \
				pxa1908-j1nlte-06.dtb \
				pxa1908-j1nlte-07.dtb
dtb-$(CONFIG_MACH_J1NLTE_SEA_OPEN) += pxa1908-j1nlteseaopen-00.dtb
dtb-$(CONFIG_MACH_DEGASVELTE) += pxa1908-degasvelte-00.dtb \
				pxa1908-degasvelte-02.dtb
targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
