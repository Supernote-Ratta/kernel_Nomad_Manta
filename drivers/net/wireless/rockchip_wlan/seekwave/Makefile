# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_WLAN_VENDOR_SEEKWAVE) += skw.o
#obj-m += skw.o

skw-objs += skw_core.o
skw-objs += skw_iface.o
skw-objs += skw_cfg80211.o
skw-objs += skw_msg.o
skw-objs += skw_rx.o
skw-objs += skw_tx.o
skw-objs += skw_vendor.o
skw-objs += skw_regd.o
skw-objs += skw_mlme.o
skw-objs += skw_timer.o
skw-objs += skw_log.o
skw-objs += skw_iw.o
skw-objs += skw_work.o
skw-objs += skw_tdls.o
skw-objs += skw_mbssid.o
skw-objs += skw_edma.o
skw-objs += skw_debugfs.o
skw-objs += skw_util.o
skw-objs += skw_calib.o
skw-objs += skw_recovery.o
skw-objs += trace.o

ccflags-y += -I$(srctree)/include/linux/platform_data/

ifneq ($(skw_extra_flags),)
ccflags-y += $(skw_extra_flags)
endif

ccflags-y += -DSKW_HIGH_PRIORITY
ccflags-y += -DSKW_SAP_SME_EXTERNAL
ccflags-y += -DSKW_STA_SME_EXTERNAL

#ccflags-y += -DSKW_SCAN_RANDOM_MAC
ccflags-y += -DSKW_REGD_SELF_MANAGED
ccflags-y += -DSKW_DEFAULT_COUNTRY="CN"

#ccflags-y += -DSKW_DPD_CALI_EN
ccflags-y += -DSKW_DEDICATED_P2P_DEV
ccflags-y += -DSKW_LOG_LEVEL=SKW_DEBUG
#ccflags-y += -DSKW_RX_ASSOC_RESP_EXT
#ccflags-y += -DSKW_GKI_DRV

#ccflags-y += -DSKW_ARP_NAT
#ccflags-y += -DSKW_DHCP_BROADCAST

#ccflags-y += -DSKW_CREDIT_DEBUG
ccflags-y += -Wno-implicit-fallthrough

# INI file select
ccflags-y += -DSKW_CHIP_ID="EA6621Q"
ccflags-y += -DSKW_PROJECT_NAME="SEEKWAVE"

ccflags-y += -DSKW_TX_WORKQUEUE

CFLAGS_trace.o := -I$(src)

$(obj)/skw_core.o : $(obj)/version.h
$(obj)/version.h: $(src)/genver.pl
	@$(PERL) -s $(src)/genver.pl $@

clean-files := version.h
