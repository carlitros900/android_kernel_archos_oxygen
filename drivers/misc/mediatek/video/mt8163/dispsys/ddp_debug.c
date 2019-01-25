#define LOG_TAG "DEBUG"

#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <mt-plat/aee.h>
#include "disp_assert_layer.h"
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#include "m4u.h"

#include "disp_drv_ddp.h"

#include "ddp_debug.h"
#include "ddp_reg.h"
#include "ddp_drv.h"
#include "ddp_wdma.h"
#include "ddp_hal.h"
#include "ddp_path.h"
#include "ddp_aal.h"
#include "ddp_pwm.h"
#include "ddp_info.h"
#include "ddp_dsi.h"
#include "ddp_ovl.h"

#include "ddp_manager.h"
#include "ddp_log.h"
#include "ddp_met.h"
#include "display_recorder.h"
#include "disp_session.h"
#include "primary_display.h"

#pragma GCC optimize("O0")

#define ddp_aee_print(string, args...) do {\
	char ddp_name[100];\
	snprintf(ddp_name, 100, "[DDP]"string, ##args); \
	aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_MMPROFILE_BUFFER, ddp_name, "[DDP] error"string, ##args);  \
	pr_err("DDP " "error: "string, ##args);  \
} while (0)

/* --------------------------------------------------------------------------- */
/* External variable declarations */
/* --------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------- */
/* Debug Options */
/* --------------------------------------------------------------------------- */

static struct dentry *debugfs;
static struct dentry *debugDir;


static struct dentry *debugfs_dump;

static const long int DEFAULT_LOG_FPS_WND_SIZE = 30;
static int debug_init;


unsigned char pq_debug_flag = 0;
unsigned char aal_debug_flag = 0;

static unsigned int dbg_log_level;
static unsigned int irq_log_level;
static unsigned int dump_to_buffer;

unsigned int gOVLBackground = 0xFF000000;
unsigned int gUltraEnable = 1;
unsigned int gDumpMemoutCmdq = 0;
unsigned int gEnableUnderflowAEE = 0;

unsigned int disp_low_power_enlarge_blanking = 0;
unsigned int disp_low_power_disable_ddp_clock = 0;
unsigned int disp_low_power_disable_fence_thread = 0;
unsigned int disp_low_power_remove_ovl = 1;
unsigned int gSkipIdleDetect = 0;
unsigned int gDumpClockStatus = 1;
#ifdef DISP_ENABLE_SODI_FOR_VIDEO_MODE
unsigned int gEnableSODIControl = 1;
unsigned int gPrefetchControl = 1;
#else
unsigned int gEnableSODIControl = 0;
unsigned int gPrefetchControl = 0;
#endif

unsigned int gDisableSODIForTriggerLoop = 1;
unsigned int gRDMAUltraSetting = 0x1b013bea;	/*so we can modify RDMA ultra at run-time */

/* enable it when use UART to grab log */
unsigned int gEnableUartLog = 0;
/* mutex SOF at raing edge of vsync, can save more time for cmdq config */
unsigned int gEnableMutexRisingEdge = 0;
/* only write dirty register, reduce register number write by cmdq */
unsigned int gEnableReduceRegWrite = 0;

unsigned int gDumpConfigCMD = 0;
unsigned int gESDEnableSODI = 1;
unsigned int gEnableOVLStatusCheck = 1;
unsigned int force_sec = 0;
static char STR_HELP[] =
	"USAGE:\n"
	"       echo [ACTION]>/d/dispsys\n"
	"ACTION:\n"
	"       regr:addr\n              :regr:0xf400c000\n"
	"       regw:addr,value          :regw:0xf400c000,0x1\n"
	"       dbg_log:0|1|2            :0 off, 1 dbg, 2 all\n"
	"       irq_log:0|1              :0 off, !0 on\n"
	"       met_on:[0|1],[0|1],[0|1] :fist[0|1]on|off,other [0|1]direct|decouple\n"
	"       backlight:level\n"
	"       dump_aal:arg\n"
	"       mmp\n"
	"       dump_reg:moduleID\n" "       dump_path:mutexID\n" "       dpfd_ut1:channel\n";
/* --------------------------------------------------------------------------- */
/* Command Processor */
/* --------------------------------------------------------------------------- */
static char dbg_buf[2048];
static unsigned int is_reg_addr_valid(unsigned int isVa, unsigned long addr)
{
	unsigned int i = 0;

	for (i = 0; i < DISP_REG_NUM; i++) {
		if ((isVa == 1) && (addr >= dispsys_reg[i]) && (addr <= dispsys_reg[i] + 0x1000))
			break;
		if ((isVa == 0) && (addr >= ddp_reg_pa_base[i])
		    && (addr <= ddp_reg_pa_base[i] + 0x1000))
			break;
	}

	if (i < DISP_REG_NUM) {
		DDPMSG("addr valid, isVa=0x%x, addr=0x%lx, module=%s!\n", isVa, addr,
		       ddp_get_reg_module_name(i));
		return 1;
	}
	DDPERR("is_reg_addr_valid return fail, isVa=0x%x, addr=0x%lx!\n", isVa, addr);
	return 0;
}


static void process_dbg_opt(const char *opt)
{
	int ret = 0;
	char *p;

	static disp_session_config config;
	char *buf = dbg_buf + strlen(dbg_buf);


	if (0 == strncmp(opt, "regr:", 5)) {
		unsigned long addr = 0;

		p = (char *)opt + 5;
		ret = kstrtoul(p, 16, (unsigned long int *)&addr);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);

		if (is_reg_addr_valid(1, addr) == 1) {
			unsigned int regVal = DISP_REG_GET(addr);

			DDPMSG("regr: 0x%lx = 0x%08X\n", addr, regVal);
			sprintf(buf, "regr: 0x%lx = 0x%08X\n", addr, regVal);
		} else {
			sprintf(buf, "regr, invalid address 0x%lx\n", addr);
			goto Error;
		}
	} else if (0 == strncmp(opt, "regw:", 5)) {
		unsigned long addr = 0;
		unsigned int val = 0;

		p = (char *)opt + 5;
		ret = kstrtoul(p, 16, (unsigned long int *)&addr);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);
		ret = kstrtoul(p + 1, 16, (unsigned long int *)&val);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		if (is_reg_addr_valid(1, addr) == 1) {
			unsigned int regVal;

			DISP_CPU_REG_SET(addr, val);
			regVal = DISP_REG_GET(addr);
			DDPMSG("regw: 0x%lx, 0x%08X = 0x%08X\n", addr, val, regVal);
			sprintf(buf, "regw: 0x%lx, 0x%08X = 0x%08X\n", addr, val, regVal);
		} else {
			sprintf(buf, "regw, invalid address 0x%lx\n", addr);
			goto Error;
		}
	} else if (0 == strncmp(opt, "rdma_ultra:", 11)) {
		p = (char *)opt + 11;
		ret = kstrtoul(p, 16, (unsigned long int *)&gRDMAUltraSetting);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);

		DISP_CPU_REG_SET(DISP_REG_RDMA_MEM_GMC_SETTING_0, gRDMAUltraSetting);
		sprintf(buf, "rdma_ultra, gRDMAUltraSetting=0x%x, reg=0x%x\n", gRDMAUltraSetting,
			DISP_REG_GET(DISP_REG_RDMA_MEM_GMC_SETTING_0));
	} else if (0 == strncmp(opt, "dbg_log:", 8)) {
		unsigned int enable = 0;

		p = (char *)opt + 8;
		ret = kstrtoul(p, 10, (unsigned long int *)&enable);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		if (enable)
			dbg_log_level = 1;
		else
			dbg_log_level = 0;

		sprintf(buf, "dbg_log: %d\n", dbg_log_level);
	} else if (0 == strncmp(opt, "irq_log:", 8)) {
		unsigned int enable = 0;

		p = (char *)opt + 8;
		ret = kstrtoul(p, 10, (unsigned long int *)&enable);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		if (enable)
			irq_log_level = 1;
		else
			irq_log_level = 0;

		sprintf(buf, "irq_log: %d\n", irq_log_level);
	} else if (0 == strncmp(opt, "met_on:", 7)) {
		int met_on = 0;
		int rdma0_mode = 0;
		int rdma1_mode = 0;

		p = (char *)opt + 7;
		ret = kstrtoul(p, 10, (unsigned long int *)&met_on);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);
		ret = kstrtoul(p + 1, 10, (unsigned long int *)&rdma0_mode);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);
		ret = kstrtoul(p + 1, 10, (unsigned long int *)&rdma1_mode);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		ddp_init_met_tag(met_on, rdma0_mode, rdma1_mode);
		DDPMSG("process_dbg_opt, met_on=%d,rdma0_mode %d, rdma1 %d\n", met_on, rdma0_mode,
		       rdma1_mode);
		sprintf(buf, "met_on:%d,rdma0_mode:%d,rdma1_mode:%d\n", met_on, rdma0_mode,
			rdma1_mode);
	} else if (0 == strncmp(opt, "backlight:", 10)) {
		unsigned int level = 0;

		p = (char *)opt + 10;
		ret = kstrtoul(p, 10, (unsigned long int *)&level);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		if (level) {
			disp_bls_set_backlight(level);
			sprintf(buf, "backlight: %d\n", level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "pwm0:", 5) || 0 == strncmp(opt, "pwm1:", 5)) {
		unsigned int level = 0;

		p = (char *)opt + 5;
		ret = kstrtoul(p, 10, (unsigned long int *)&level);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		if (level) {
			disp_pwm_id_t pwm_id = DISP_PWM0;

			if (opt[3] == '1')
				pwm_id = DISP_PWM1;

			disp_pwm_set_backlight(pwm_id, level);
			sprintf(buf, "PWM 0x%x : %d\n", pwm_id, level);
		} else {
			goto Error;
		}
	} else if (0 == strncmp(opt, "aal_dbg:", 8)) {
		ret = kstrtoul(opt + 8, 10, (unsigned long int *)&aal_dbg_en);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);

		sprintf(buf, "aal_dbg_en = 0x%x\n", aal_dbg_en);
	} else if (0 == strncmp(opt, "dump_reg:", 9)) {
		unsigned int module = 0;

		p = (char *)opt + 9;
		ret = kstrtoul(p, 10, (unsigned long int *)&module);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		DDPMSG("process_dbg_opt, module=%d\n", module);
		if (module < DISP_MODULE_NUM) {
			ddp_dump_reg(module);
			sprintf(buf, "dump_reg: %d\n", module);
		} else {
			DDPMSG("process_dbg_opt2, module=%d\n", module);
			goto Error;
		}
	} else if (0 == strncmp(opt, "dump_path:", 10)) {
		unsigned int mutex_idx = 0;

		p = (char *)opt + 10;
		ret = kstrtoul(p, 10, (unsigned long int *)&mutex_idx);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);


		DDPMSG("process_dbg_opt, path mutex=%d\n", mutex_idx);
		dpmgr_debug_path_status(mutex_idx);
		sprintf(buf, "dump_path: %d\n", mutex_idx);
	} else if (0 == strncmp(opt, "debug:", 6)) {
		unsigned int enable = 0;

		p = (char *)opt + 6;
		ret = kstrtoul(p, 10, (unsigned long int *)&enable);
		if (ret)
			pr_err("DISP/%s: errno %d\n", __func__, ret);

		if (enable == 1) {
			DDPMSG("[DDP] debug=1, trigger AEE\n");
			/* aee_kernel_exception("DDP-TEST-ASSERT", "[DDP] DDP-TEST-ASSERT"); */
		} else if (enable == 2) {
			ddp_mem_test();
		} else if (enable == 3) {
			ddp_lcd_test();
		} else if (enable == 4) {
			/* DDPAEE("test 4"); */
		} else if (enable == 5) {

			if (gDDPError == 0)
				gDDPError = 1;
			else
				gDDPError = 0;

			sprintf(buf, "bypass PQ: %d\n", gDDPError);
			DDPMSG("bypass PQ: %d\n", gDDPError);
		} else if (enable == 6) {
			unsigned int i = 0;
			int *modules = ddp_get_scenario_list(DDP_SCENARIO_PRIMARY_DISP);
			int module_num = ddp_get_module_num(DDP_SCENARIO_PRIMARY_DISP);

			pr_debug("dump path status:");
			for (i = 0; i < module_num; i++)
				pr_debug("%s-", ddp_get_module_name(modules[i]));

			pr_debug("\n");

			ddp_dump_analysis(DISP_MODULE_CONFIG);
			ddp_dump_analysis(DISP_MODULE_MUTEX);
			for (i = 0; i < module_num; i++)
				ddp_dump_analysis(modules[i]);

			if (primary_display_is_decouple_mode()) {
				ddp_dump_analysis(DISP_MODULE_OVL0);
				ddp_dump_analysis(DISP_MODULE_OVL1);
				ddp_dump_analysis(DISP_MODULE_WDMA0);
			}

			ddp_dump_reg(DISP_MODULE_CONFIG);
			ddp_dump_reg(DISP_MODULE_MUTEX);

			if (primary_display_is_decouple_mode()) {
				ddp_dump_reg(DISP_MODULE_OVL0);
				ddp_dump_reg(DISP_MODULE_OVL1);
				ddp_dump_reg(DISP_MODULE_WDMA0);
			}

			for (i = 0; i < module_num; i++)
				ddp_dump_reg(modules[i]);
		} else if (enable == 7) {
			if (dbg_log_level < 3)
				dbg_log_level++;
			else
				dbg_log_level = 0;

			pr_debug("DDP: dbg_log_level=%d\n", dbg_log_level);
			sprintf(buf, "dbg_log_level: %d\n", dbg_log_level);
		} else if (enable == 8) {
			DDPDUMP("clock_mm setting:%u\n", DISP_REG_GET(DISP_REG_CONFIG_C11));
			if ((DISP_REG_GET(DISP_REG_CONFIG_C11) & 0xff000000) != 0xff000000)
				DDPDUMP
				    ("error, MM clock bit 24~bit31 should be 1, but real value=0x%x",
				     DISP_REG_GET(DISP_REG_CONFIG_C11));
		} else if (enable == 9) {
			gOVLBackground = 0xFF0000FF;
			pr_debug("DDP: gOVLBackground=%d\n", gOVLBackground);
			sprintf(buf, "gOVLBackground: %d\n", gOVLBackground);
		} else if (enable == 10) {
			gOVLBackground = 0xFF000000;
			pr_debug("DDP: gOVLBackground=%d\n", gOVLBackground);
			sprintf(buf, "gOVLBackground: %d\n", gOVLBackground);
		} else if (enable == 11) {
			unsigned int i = 0;
			char *buf_temp = buf;

			for (i = 0; i < DISP_REG_NUM; i++) {
				DDPDUMP("i=%d, module=%s, reg_va=0x%lx\n", i,
					ddp_get_reg_module_name(i), dispsys_reg[i]);
				sprintf(buf_temp, "i=%d, module=%s, reg_va=0x%lx\n", i,
					ddp_get_reg_module_name(i), dispsys_reg[i]);
				buf_temp += strlen(buf_temp);
			}
		} else if (enable == 12) {
			if (gUltraEnable == 0)
				gUltraEnable = 1;
			else
				gUltraEnable = 0;

			pr_debug("DDP: gUltraEnable=%d\n", gUltraEnable);
			sprintf(buf, "gUltraEnable: %d\n", gUltraEnable);
		} else if (enable == 13) {
			int ovl_status = ovl_get_status();

			config.type = DISP_SESSION_MEMORY;
			config.device_id = 0;
			disp_create_session(&config);
			pr_debug("old status=%d, ovl1 status=%d\n", ovl_status, ovl_get_status());
			sprintf(buf, "old status=%d, ovl1 status=%d\n", ovl_status,
				ovl_get_status());
		} else if (enable == 14) {
			int ovl_status = ovl_get_status();

			disp_destroy_session(&config);
			pr_debug("old status=%d, ovl1 status=%d\n", ovl_status, ovl_get_status());
			sprintf(buf, "old status=%d, ovl1 status=%d\n", ovl_status,
				ovl_get_status());
		} else if (enable == 16) {
			if (gDumpMemoutCmdq == 0)
				gDumpMemoutCmdq = 1;
			else
				gDumpMemoutCmdq = 0;

			pr_debug("DDP: gDumpMemoutCmdq=%d\n", gDumpMemoutCmdq);
			sprintf(buf, "gDumpMemoutCmdq: %d\n", gDumpMemoutCmdq);
		} else if (enable == 21) {
			if (gEnableSODIControl == 0)
				gEnableSODIControl = 1;
			else
				gEnableSODIControl = 0;

			pr_debug("DDP: gEnableSODIControl=%d\n", gEnableSODIControl);
			sprintf(buf, "gEnableSODIControl: %d\n", gEnableSODIControl);
		} else if (enable == 22) {
			if (gPrefetchControl == 0)
				gPrefetchControl = 1;
			else
				gPrefetchControl = 0;

			pr_debug("DDP: gPrefetchControl=%d\n", gPrefetchControl);
			sprintf(buf, "gPrefetchControl: %d\n", gPrefetchControl);
		} else if (enable == 23) {
			if (disp_low_power_enlarge_blanking == 0)
				disp_low_power_enlarge_blanking = 1;
			else
				disp_low_power_enlarge_blanking = 0;

			pr_debug("DDP: disp_low_power_enlarge_blanking=%d\n",
			       disp_low_power_enlarge_blanking);
			sprintf(buf, "disp_low_power_enlarge_blanking: %d\n",
				disp_low_power_enlarge_blanking);

		} else if (enable == 24) {
			if (disp_low_power_disable_ddp_clock == 0)
				disp_low_power_disable_ddp_clock = 1;
			else
				disp_low_power_disable_ddp_clock = 0;

			pr_debug("DDP: disp_low_power_disable_ddp_clock=%d\n",
			       disp_low_power_disable_ddp_clock);
			sprintf(buf, "disp_low_power_disable_ddp_clock: %d\n",
				disp_low_power_disable_ddp_clock);

		} else if (enable == 25) {
			if (disp_low_power_disable_fence_thread == 0)
				disp_low_power_disable_fence_thread = 1;
			else
				disp_low_power_disable_fence_thread = 0;

			pr_debug("DDP: disp_low_power_disable_fence_thread=%d\n",
			       disp_low_power_disable_fence_thread);
			sprintf(buf, "disp_low_power_disable_fence_thread: %d\n",
				disp_low_power_disable_fence_thread);

		} else if (enable == 26) {
			if (disp_low_power_remove_ovl == 0)
				disp_low_power_remove_ovl = 1;
			else
				disp_low_power_remove_ovl = 0;

			pr_debug("DDP: disp_low_power_remove_ovl=%d\n", disp_low_power_remove_ovl);
			sprintf(buf, "disp_low_power_remove_ovl: %d\n", disp_low_power_remove_ovl);

		} else if (enable == 27) {
			if (gSkipIdleDetect == 0)
				gSkipIdleDetect = 1;
			else
				gSkipIdleDetect = 0;

			pr_debug("DDP: gSkipIdleDetect=%d\n", gSkipIdleDetect);
			sprintf(buf, "gSkipIdleDetect: %d\n", gSkipIdleDetect);

		} else if (enable == 28) {
			if (gDumpClockStatus == 0)
				gDumpClockStatus = 1;
			else
				gDumpClockStatus = 0;

			pr_debug("DDP: gDumpClockStatus=%d\n", gDumpClockStatus);
			sprintf(buf, "gDumpClockStatus: %d\n", gDumpClockStatus);

		} else if (enable == 29) {
			if (gEnableUartLog == 0)
				gEnableUartLog = 1;
			else
				gEnableUartLog = 0;

			pr_debug("DDP: gEnableUartLog=%d\n", gEnableUartLog);
			sprintf(buf, "gEnableUartLog: %d\n", gEnableUartLog);

		} else if (enable == 30) {
			if (gEnableMutexRisingEdge == 0) {
				gEnableMutexRisingEdge = 1;
				DISP_REG_SET_FIELD(0, SOF_FLD_MUTEX0_SOF_TIMING,
						   DISP_REG_CONFIG_MUTEX0_SOF, 1);
			} else {
				gEnableMutexRisingEdge = 0;
				DISP_REG_SET_FIELD(0, SOF_FLD_MUTEX0_SOF_TIMING,
						   DISP_REG_CONFIG_MUTEX0_SOF, 0);
			}

			pr_debug("DDP: gEnableMutexRisingEdge=%d\n", gEnableMutexRisingEdge);
			sprintf(buf, "gEnableMutexRisingEdge: %d\n", gEnableMutexRisingEdge);

		} else if (enable == 31) {
			if (gEnableReduceRegWrite == 0)
				gEnableReduceRegWrite = 1;
			else
				gEnableReduceRegWrite = 0;

			pr_debug("DDP: gEnableReduceRegWrite=%d\n", gEnableReduceRegWrite);
			sprintf(buf, "gEnableReduceRegWrite: %d\n", gEnableReduceRegWrite);

		} else if (enable == 32) {
			DDPAEE("DDP: (32)gEnableReduceRegWrite=%d\n", gEnableReduceRegWrite);
		} else if (enable == 33) {
			if (gDumpConfigCMD == 0)
				gDumpConfigCMD = 1;
			else
				gDumpConfigCMD = 0;

			pr_debug("DDP: gDumpConfigCMD=%d\n", gDumpConfigCMD);
			sprintf(buf, "gDumpConfigCMD: %d\n", gDumpConfigCMD);

		} else if (enable == 34) {
			if (gESDEnableSODI == 0)
				gESDEnableSODI = 1;
			else
				gESDEnableSODI = 0;

			pr_debug("DDP: gESDEnableSODI=%d\n", gESDEnableSODI);
			sprintf(buf, "gESDEnableSODI: %d\n", gESDEnableSODI);

		} else if (enable == 35) {
			if (gEnableOVLStatusCheck == 0)
				gEnableOVLStatusCheck = 1;
			else
				gEnableOVLStatusCheck = 0;

			pr_debug("DDP: gEnableOVLStatusCheck=%d\n", gEnableOVLStatusCheck);
			sprintf(buf, "gEnableOVLStatusCheck: %d\n", gEnableOVLStatusCheck);

		} else if (enable == 36) {
			if (gDisableSODIForTriggerLoop == 0)
				gDisableSODIForTriggerLoop = 1;
			else
				gDisableSODIForTriggerLoop = 0;
			pr_debug("DDP: gDisableSODIForTriggerLoop=%d\n", gDisableSODIForTriggerLoop);
			sprintf(buf, "gDisableSODIForTriggerLoop: %d\n",
				gDisableSODIForTriggerLoop);
		} else if (enable == 37) {
			if (force_sec == 0)
				force_sec = 1;
			else
				force_sec = 0;
			pr_err("force_sec: %d\n", force_sec);
			sprintf(buf, "force_sec: %d\n", force_sec);
		}
	} else if (0 == strncmp(opt, "mmp", 3)) {
		init_ddp_mmp_events();
	} else {
		dbg_buf[0] = '\0';
		goto Error;
	}

	return;

Error:
	DDPERR("parse command error!\n%s\n\n%s", opt, STR_HELP);
}


static void process_dbg_cmd(char *cmd)
{
	char *tok;

	DDPDBG("cmd: %s\n", cmd);
	memset(dbg_buf, 0, sizeof(dbg_buf));
	while ((tok = strsep(&cmd, " ")) != NULL)
		process_dbg_opt(tok);
}


/* --------------------------------------------------------------------------- */
/* Debug FileSystem Routines */
/* --------------------------------------------------------------------------- */

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}


static char cmd_buf[512];

static ssize_t debug_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
	if (strlen(dbg_buf))
		return simple_read_from_buffer(ubuf, count, ppos, dbg_buf, strlen(dbg_buf));
	else
		return simple_read_from_buffer(ubuf, count, ppos, STR_HELP, strlen(STR_HELP));

}


static ssize_t debug_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(cmd_buf) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax)
		count = debug_bufmax;

	if (copy_from_user(&cmd_buf, ubuf, count))
		return -EFAULT;

	cmd_buf[count] = 0;

	process_dbg_cmd(cmd_buf);

	return ret;
}


static const struct file_operations debug_fops = {
	.read = debug_read,
	.write = debug_write,
	.open = debug_open,
};

static ssize_t debug_dump_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{

	dprec_logger_dump_reset();
	dump_to_buffer = 1;
	/* dump all */
	dpmgr_debug_path_status(-1);
	dump_to_buffer = 0;
	return simple_read_from_buffer(buf, size, ppos, dprec_logger_get_dump_addr(),
				       dprec_logger_get_dump_len());
}


static const struct file_operations debug_fops_dump = {
	.read = debug_dump_read,
};

void ddp_debug_init(void)
{
	if (!debug_init) {
		debug_init = 1;
		debugfs = debugfs_create_file("dispsys",
					      S_IFREG | S_IRUGO, NULL, (void *)0, &debug_fops);


		debugDir = debugfs_create_dir("disp", NULL);
		if (debugDir) {

			debugfs_dump = debugfs_create_file("dump",
							   S_IFREG | S_IRUGO, debugDir, NULL,
							   &debug_fops_dump);
		}
	}
}

unsigned int ddp_debug_analysis_to_buffer(void)
{
	return dump_to_buffer;
}

unsigned int ddp_debug_dbg_log_level(void)
{
	return dbg_log_level;
}

unsigned int ddp_debug_irq_log_level(void)
{
	return irq_log_level;
}


void ddp_debug_exit(void)
{
	debugfs_remove(debugfs);
	debugfs_remove(debugfs_dump);
	debug_init = 0;
}

int ddp_mem_test(void)
{
	return -1;
}

int ddp_lcd_test(void)
{
	return -1;
}

char *disp_get_fmt_name(DP_COLOR_ENUM color)
{
	switch (color) {
	case DP_COLOR_FULLG8:
		return "fullg8";
	case DP_COLOR_FULLG10:
		return "fullg10";
	case DP_COLOR_FULLG12:
		return "fullg12";
	case DP_COLOR_FULLG14:
		return "fullg14";
	case DP_COLOR_UFO10:
		return "ufo10";
	case DP_COLOR_BAYER8:
		return "bayer8";
	case DP_COLOR_BAYER10:
		return "bayer10";
	case DP_COLOR_BAYER12:
		return "bayer12";
	case DP_COLOR_RGB565:
		return "rgb565";
	case DP_COLOR_BGR565:
		return "bgr565";
	case DP_COLOR_RGB888:
		return "rgb888";
	case DP_COLOR_BGR888:
		return "bgr888";
	case DP_COLOR_RGBA8888:
		return "rgba";
	case DP_COLOR_BGRA8888:
		return "bgra";
	case DP_COLOR_ARGB8888:
		return "argb";
	case DP_COLOR_ABGR8888:
		return "abgr";
	case DP_COLOR_I420:
		return "i420";
	case DP_COLOR_YV12:
		return "yv12";
	case DP_COLOR_NV12:
		return "nv12";
	case DP_COLOR_NV21:
		return "nv21";
	case DP_COLOR_I422:
		return "i422";
	case DP_COLOR_YV16:
		return "yv16";
	case DP_COLOR_NV16:
		return "nv16";
	case DP_COLOR_NV61:
		return "nv61";
	case DP_COLOR_YUYV:
		return "yuyv";
	case DP_COLOR_YVYU:
		return "yvyu";
	case DP_COLOR_UYVY:
		return "uyvy";
	case DP_COLOR_VYUY:
		return "vyuy";
	case DP_COLOR_I444:
		return "i444";
	case DP_COLOR_YV24:
		return "yv24";
	case DP_COLOR_IYU2:
		return "iyu2";
	case DP_COLOR_NV24:
		return "nv24";
	case DP_COLOR_NV42:
		return "nv42";
	case DP_COLOR_GREY:
		return "grey";
	default:
		return "undefined";
	}

}
