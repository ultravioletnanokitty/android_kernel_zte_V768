#******************************************************************************
#*
#*    Copyright 2007  Broadcom Corporation
#*    All Rights Reserved
#*
#*    No portions of this material may be reproduced in any form without the
#*    written permission of:
#*
#*       Broadcom Corporation
#*       16215 Alton Parkway
#*       Irvine, California 92619-7013
#*
#*    All information contained in this document is Broadcom Corporation
#*    company private, proprietary, and trade secret.
#*
#******************************************************************************
EXTRA_CFLAGS += -DUNDER_LINUX
EXTRA_CFLAGS += -DRPC_INCLUDED
EXTRA_CFLAGS += -DSTACK_wedge

EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_rpc/rpc_CIB/public/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ipc/ipc_CIB/public/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_ril/
EXTRA_CFLAGS += -I ../modules/drivers/char/brcm/fuse_log/
include ../modules/drivers/char/brcm/fuse_ril/CAPI2_CIB/capi2_include_paths.gmk

obj-$(CONFIG_BRCM_CNTIN) = gps_cntin_driver.o
obj-$(CONFIG_BRCM_CNTIN) += gps_driver.o
