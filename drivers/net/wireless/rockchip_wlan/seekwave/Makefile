# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_WLAN_VENDOR_SEEKWAVE) += skw.o

CONFIG_SKW_STA_SME_EXT=y
CONFIG_SKW_SAP_SME_EXT=y
CONFIG_SKW_SCAN_RADOM_MAC=y
CONFIG_SKW_TX_WORKQUEUE=y
CONFIG_SKW_VENDOR=y
CONFIG_SKW_HIGH_PRIORITY=y
CONFIG_SKW_REGD_SELF_MANAGED=y
CONFIG_SKW_TDLS=y
#CONFIG_SKW_GKI=y
#CONFIG_SKW_DFS_MASTER=y
#CONFIG_SKW_EDMA=y
#CONFIG_SKW_REPEATER=y
#CONFIG_SKW_LEGACY_P2P=y
#CONFIG_SKW_CALIB_DPD=y
#CONFIG_SKW_CHIP_ID="EA6621Q"
CONFIG_SKW_LOG_LEVEL=SKW_DEBUG
CONFIG_SKW_PROJECT_NAME="SEEKWAVE"
CONFIG_SKW_DEFAULT_COUNTRY="CN"

skw-objs += skw_core.o
skw-objs += skw_iface.o
skw-objs += skw_cfg80211.o
skw-objs += skw_msg.o
skw-objs += skw_rx.o
skw-objs += skw_tx.o
skw-objs += skw_regd.o
skw-objs += skw_mlme.o
skw-objs += skw_timer.o
skw-objs += skw_log.o
skw-objs += skw_iw.o
skw-objs += skw_work.o
skw-objs += skw_mbssid.o
skw-objs += skw_debugfs.o
skw-objs += skw_util.o
skw-objs += skw_recovery.o
skw-objs += trace.o

skw-$(CONFIG_SKW_CALIB_DPD) += skw_calib.o
skw-$(CONFIG_SKW_EDMA) += skw_edma.o
skw-$(CONFIG_SKW_VENDOR) += skw_vendor.o
skw-$(CONFIG_SKW_DFS_MASTER) += skw_dfs.o
skw-$(CONFIG_SKW_TDLS) += skw_tdls.o

ccflags-$(CONFIG_SKW_REPEATER) += -DSKW_ARP_NAT -DSKW_DHCP_BROADCAST
ccflags-$(CONFIG_SKW_STA_SME_EXT) += -DSKW_STA_SME_EXTERNAL
ccflags-$(CONFIG_SKW_SAP_SME_EXT) += -DSKW_SAP_SME_EXTERNAL
ccflags-$(CONFIG_SKW_SCAN_RADOM_MAC) += -DSKW_SCAN_RANDOM_MAC
ccflags-$(CONFIG_SKW_TX_WORKQUEUE) += -DSKW_TX_WORKQUEUE
ccflags-$(CONFIG_SKW_DFS_MASTER) += -DSKW_DFS_MASTER
ccflags-$(CONFIG_SKW_VENDOR) += -DSKW_VENDOR
ccflags-$(CONFIG_SKW_CALIB_DPD) += -DSKW_CALIB_DPD -DSKW_IMPORT_NS
ccflags-$(CONFIG_SKW_HIGH_PRIORITY) += -DSKW_HIGH_PRIORITY
ccflags-$(CONFIG_SKW_REGD_SELF_MANAGED) += -DSKW_REGD_SELF_MANAGED
ccflags-$(CONFIG_SKW_GKI) += -DSKW_GKI_DRV
ccflags-$(CONFIG_SKW_EDMA) += -DSKW_EDMA
ccflags-$(CONFIG_SKW_TDLS) += -DSKW_TDLS

ifneq ($(CONFIG_SKW_DEFAULT_COUNTRY),)
ccflags-y += -DSKW_DEFAULT_COUNTRY=$(CONFIG_SKW_DEFAULT_COUNTRY)
endif

ifneq ($(CONFIG_SKW_LOG_LEVEL),)
ccflags-y += -DSKW_LOG_LEVEL=$(CONFIG_SKW_LOG_LEVEL)
endif

ifneq ($(CONFIG_SKW_CHIP_ID),)
ccflags-y += -DSKW_CHIP_ID=$(CONFIG_SKW_CHIP_ID)
endif

ifneq ($(CONFIG_SKW_PROJECT_NAME),)
ccflags-y += -DSKW_PROJECT_NAME=$(CONFIG_SKW_PROJECT_NAME)
endif

skw_extra_flags += -I$(srctree)/include/linux/platform_data/
ccflags-y += $(skw_extra_flags)

CFLAGS_trace.o := -I$(src)

$(obj)/skw_core.o : $(obj)/version.h
$(obj)/version.h: $(src)/genver.pl
	@$(PERL) -s $(src)/genver.pl $@

clean-files := version.h
