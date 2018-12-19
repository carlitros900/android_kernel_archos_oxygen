#ifndef _MT_HDMI_DEBUG_H_
#define _MT_HDMI_DEBUG_H_

#ifdef CONFIG_MTK_INTERNAL_HDMI_SUPPORT
#include "hdmitable.h"
#include "hdmiedid.h"

extern unsigned char _bHdcpOff;

extern HDMI_AV_INFO_T _stAvdAVInfo;
extern struct HDMI_SINK_AV_CAP_T _HdmiSinkAvCap;
extern unsigned char cDstStr[50];
extern const unsigned char _cFsStr[][7];
extern unsigned char hdmi_plug_test_mode;
/* unsigned int hdmi_plug_out_delay; */
/* unsigned int hdmi_plug_in_delay; */
extern unsigned char is_user_mute_hdmi;
extern unsigned char is_user_mute_hdmi_audio;
extern void hdmi_force_plug_out(void);
extern void hdmi_force_plug_in(void);
extern unsigned char hdmi_hdcp_for_avmute;
extern void mt_hdmi_show_info(char *pbuf);
extern void mt_hdmi_debug_write(const char *pbuf);

#endif
#endif
