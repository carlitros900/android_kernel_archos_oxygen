#ifndef __MT65XX_LCM_LIST_H__
#define __MT65XX_LCM_LIST_H__

#include <lcm_drv.h>

#if defined(MTK_LCM_DEVICE_TREE_SUPPORT)
extern LCM_DRIVER lcm_common_drv;
#else
extern LCM_DRIVER LT070ME05000_MIPI_lcm_drv;//add by tubao
#endif

#ifdef BUILD_LK
extern void mdelay(unsigned long msec);
#endif

#endif
