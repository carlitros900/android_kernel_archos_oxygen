#include <linux/delay.h>
#include <mt-plat/upmu_common.h>
#include <mt-plat/battery_common.h>

/* ============================================================*/
/*extern function*/
/* ============================================================*/

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)

int hw_charging_get_charger_type(void)
{
	return STANDARD_HOST;
}

#else

static void hw_bc11_init(void)
{
	Charger_Detect_Init();

	/*RG_BC11_BIAS_EN=1*/
	upmu_set_rg_bc11_bias_en(0x1);
	/*RG_BC11_VSRC_EN[1:0]=00*/
	upmu_set_rg_bc11_vsrc_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*BC11_RST=1*/
	upmu_set_rg_bc11_rst(0x1);
	/*BC11_BB_CTRL=1*/
	upmu_set_rg_bc11_bb_ctrl(0x1);
	/*msleep(10);*/
	mdelay(50);
}

static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;

	/*RG_BC11_IPU_EN[1.0] = 10*/
	upmu_set_rg_bc11_ipu_en(0x2);
	/*RG_BC11_IPD_EN[1.0] = 01*/
	upmu_set_rg_bc11_ipd_en(0x1);
	/*RG_BC11_VREF_VTH = [1:0]=01*/
	upmu_set_rg_bc11_vref_vth(0x1);
	/*RG_BC11_CMP_EN[1.0] = 10*/
	upmu_set_rg_bc11_cmp_en(0x2);
	/*msleep(20);*/
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);

	return wChargerAvail;
}

/*****
static unsigned int hw_bc11_stepA1(void)
{
unsigned int wChargerAvail = 0;

upmu_set_rg_bc11_ipu_en(0x2);
upmu_set_rg_bc11_vref_vth(0x2);
upmu_set_rg_bc11_cmp_en(0x2);
mdelay(80);
wChargerAvail = upmu_get_rgs_bc11_cmp_out();
upmu_set_rg_bc11_ipu_en(0x0);
upmu_set_rg_bc11_cmp_en(0x0);

return  wChargerAvail;
}
*****/

static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;

	/*RG_BC11_VSRC_EN[1.0] = 10*/
	upmu_set_rg_bc11_vsrc_en(0x2);
	/*RG_BC11_IPD_EN[1:0] = 01*/
	upmu_set_rg_bc11_ipd_en(0x1);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);
	/*RG_BC11_CMP_EN[1.0] = 01*/
	upmu_set_rg_bc11_cmp_en(0x1);

	/*msleep(80);*/
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	/*RG_BC11_VSRC_EN[1:0]=00*/
	upmu_set_rg_bc11_vsrc_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);

	return  wChargerAvail;
}

static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*RG_BC11_IPU_EN[1:0]=10*/
	upmu_set_rg_bc11_ipu_en(0x2);
	/*RG_BC11_VREF_VTH = [1:0]=10*/
	upmu_set_rg_bc11_vref_vth(0x1);
	/*RG_BC11_CMP_EN[1.0] = 01*/
	upmu_set_rg_bc11_cmp_en(0x1);

	/*msleep(80);*/
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=00*/
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}

static void hw_bc11_done(void)
{
	/*RG_BC11_VSRC_EN[1:0]=00*/
	upmu_set_rg_bc11_vsrc_en(0x0);
	/*RG_BC11_VREF_VTH = [1:0]=0*/
	upmu_set_rg_bc11_vref_vth(0x0);
	/*RG_BC11_CMP_EN[1.0] = 00*/
	upmu_set_rg_bc11_cmp_en(0x0);
	/*RG_BC11_IPU_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipu_en(0x0);
	/*RG_BC11_IPD_EN[1.0] = 00*/
	upmu_set_rg_bc11_ipd_en(0x0);
	/*RG_BC11_BIAS_EN=0*/
	upmu_set_rg_bc11_bias_en(0x0);

	Charger_Detect_Release();
}

int hw_charging_get_charger_type(void)
{
	CHARGER_TYPE ret = CHARGER_UNKNOWN;

	hw_bc11_init();

	if (1 == hw_bc11_DCD()) {
				ret = NONSTANDARD_CHARGER;
	} else {
		if (1 == hw_bc11_stepA2()) {
			if (1 == hw_bc11_stepB2())
				ret = STANDARD_CHARGER;
			else
				ret = CHARGING_HOST;
		} else {
			ret = STANDARD_HOST;
		}
	}
	hw_bc11_done();
	battery_log(BAT_LOG_CRTI, "CHR_Type_num = %d\r\n", ret);
	return ret;
}
#endif
