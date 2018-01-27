#if 0//add by 
#ifndef BUILD_LK
#include <linux/string.h>
#else
#include <string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#else
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
	#include <mach/charging.h>
#endif
//add by 
#include <mach/charging.h>
#endif //add by 

///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <mt-plat/mt_gpio.h>
#include <mt-plat/upmu_common.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif
#endif

#include "lcm_drv.h"

///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
///======================>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>yyp
#ifdef BUILD_LK

#ifdef GPIO_LCM_PWR
#define GPIO_LCD_PWR      GPIO_LCM_PWR
#else
#define GPIO_LCD_PWR      0xFFFFFFFF
#endif

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, (output > 0) ? GPIO_OUT_ONE : GPIO_OUT_ZERO);
}
#else

/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static struct regulator *lcm_vgp_back_up;//add by 
static struct regulator *lcm_vpa_back_up;//add by 
static struct pinctrl *lcmctrl;
static struct pinctrl *lcmrst;//add by 
static struct pinctrl_state *lcd_pwr_high;
static struct pinctrl_state *lcd_pwr_low;
static struct pinctrl_state *lcd_rst_high;//add by 
static struct pinctrl_state *lcd_rst_low;//add by 

static int lcm_get_gpio(struct device *dev)
{
	int ret = 0;
#if 0//add by  GPIO90
	lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmctrl);
	}
	/*lcm power pin lookup */
	lcd_pwr_high = pinctrl_lookup_state(lcmctrl, "lcm_pwr_high");
	if (IS_ERR(lcd_pwr_high)) {
		ret = PTR_ERR(lcd_pwr_high);
		pr_debug("%s : pinctrl err, lcd_pwr_high¥n", __func__);
	}
	lcd_pwr_low = pinctrl_lookup_state(lcmctrl, "lcm_pwr_low");
	if (IS_ERR(lcd_pwr_low)) {
		ret = PTR_ERR(lcd_pwr_low);
		pr_debug("%s : pinctrl err, lcd_pwr_low¥n", __func__);
	}
#else// for control GPIO90  复位引脚
	lcmrst = devm_pinctrl_get(dev);
	if (IS_ERR(lcmrst)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmrst);
	}
	/*lcm power pin lookup */
	lcd_rst_high = pinctrl_lookup_state(lcmrst, "lcm_rst_high");
	if (IS_ERR(lcd_pwr_high)) {
		ret = PTR_ERR(lcd_rst_high);
		pr_debug("%s : pinctrl err, lcd_rst_high¥n", __func__);
	}
	lcd_rst_low = pinctrl_lookup_state(lcmrst, "lcm_rst_low");
	if (IS_ERR(lcd_rst_low)) {
		ret = PTR_ERR(lcd_rst_low);
		pr_debug("%s : pinctrl err, lcd_rst_low¥n", __func__);
	}

#endif//add by  GPIO90   复位引脚
	return ret;
}

void lcm_set_gpio(int val)
{
	if (val == 0) {
		pinctrl_select_state(lcmrst, lcd_rst_low);
		pr_debug("LCM: lcm set power off¥n");
	} else {
		pinctrl_select_state(lcmrst, lcd_rst_high);
		pr_debug("LCM: lcm set power on¥n");
	}
}

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_debug("LCM: lcm_get_vgp_supply is going¥n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d¥n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.¥n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage¥n", ret);

	lcm_vgp = lcm_vgp_ldo;
//add by   ==============>>>>>>>>>>> for control VPA power start
	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vpa");
		if (IS_ERR(lcm_vgp_ldo)) {
			ret = PTR_ERR(lcm_vgp_ldo);
			dev_err(dev, "failed to get reg-lcm-vpa, %d\n", ret);
			return ret;
		}
	
		pr_info("LCM: lcm get supply ok.\n");
	
		ret = regulator_enable(lcm_vgp_ldo);
		/* get current voltage settings */
		ret = regulator_get_voltage(lcm_vgp_ldo);
		pr_info("lcm LDO voltage = %d in LK stage\n", ret);
	
		lcm_vpa_back_up = lcm_vgp_ldo;
//add by   ==============>>>>>>>>>>> for control VPA power end
//add by   ==============>>>>>>>>>>>> for control VIBR start
	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vgp-vibr");
		if (IS_ERR(lcm_vgp_ldo)) {
			ret = PTR_ERR(lcm_vgp_ldo);
			dev_err(dev, "failed to get reg-lcm-vgp-vibr, %d\n", ret);
			return ret;
		}
	
		pr_info("LCM: lcm get supply ok.\n");
	
		ret = regulator_enable(lcm_vgp_ldo);
		/* get current voltage settings */
		ret = regulator_get_voltage(lcm_vgp_ldo);
		pr_info("lcm LDO voltage = %d in LK stage\n", ret);
	
		lcm_vgp_back_up = lcm_vgp_ldo;

//add by   ==============>>>>>>>>>>>> for control VIBR end


#if 1//add by  用于控制 GPIO83  电源
	lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmctrl);
	}
	/*lcm power pin lookup */
	lcd_pwr_high = pinctrl_lookup_state(lcmctrl, "lcm_pwr_high");
	if (IS_ERR(lcd_pwr_high)) {
		ret = PTR_ERR(lcd_pwr_high);
		pr_debug("%s : pinctrl err, lcd_pwr_high\n", __func__);
	}
	lcd_pwr_low = pinctrl_lookup_state(lcmctrl, "lcm_pwr_low");
	if (IS_ERR(lcd_pwr_low)) {
		ret = PTR_ERR(lcd_pwr_low);
		pr_debug("%s : pinctrl err, lcd_pwr_low\n", __func__);
	}

#endif//add by   用于控制 GPIO83  电源
	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;


	if (NULL == lcm_vpa_back_up)
		return 0;

//add by   ==>>>>>> for control VPA power start
	ret = regulator_set_voltage(lcm_vpa_back_up, 3300000, 3300000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vpa_back_up);
		if (volt == 3300000)
			pr_err("LCM: check regulator lcm_vpa_back_up voltage=3300000 pass!\n");
		else
			pr_err("LCM: check regulator lcm_vpa_back_up voltage=3300000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vpa_back_up);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}
//add by   ==>>>>>> for control VPA power end


	//add by   for contrl vibr power start start
	ret = regulator_set_voltage(lcm_vgp_back_up, 3300000, 3300000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vgp_back_up);
		if (volt == 3300000)
			pr_err("LCM: check regulator lcm_vgp_back_up voltage=3300000 pass!\n");
		else
			pr_err("LCM: check regulator lcm_vgp_back_up voltage=3300000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vgp_back_up);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}
	//add by   for contrl vibr power start end
	//add by  用于控制GPIO83 开启电源
	pinctrl_select_state(lcmctrl, lcd_pwr_high);
	#if 1
	pr_debug("LCM: lcm_vgp_supply_enable¥n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V¥n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	//ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d¥n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 1800000)
		pr_err("LCM: check regulator voltage=3200000 pass!¥n");
	else
		pr_err("LCM: check regulator voltage=3200000 fail! (voltage: %d)¥n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d¥n", ret);
		return ret;
	}
#endif
	return 0;//ret;
}

int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]¥n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d¥n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass¥n");
	}

	isenable = regulator_is_enabled(lcm_vpa_back_up);

			pr_info("LCM: lcm query regulator enable status[0x%d]\n", isenable);

			if (isenable) {
				ret = regulator_disable(lcm_vpa_back_up);
				if (ret != 0) {
					pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
					return ret;
				}
				/* verify */
				isenable = regulator_is_enabled(lcm_vpa_back_up);
				if (!isenable)
					pr_err("LCM: lcm regulator disable pass\n");
			}


	isenable = regulator_is_enabled(lcm_vgp_back_up);

			pr_info("LCM: lcm query regulator enable status[0x%d]\n", isenable);

			if (isenable) {
				ret = regulator_disable(lcm_vgp_back_up);
				if (ret != 0) {
					pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
					return ret;
				}
				/* verify */
				isenable = regulator_is_enabled(lcm_vgp_back_up);
				if (!isenable)
					pr_err("LCM: lcm regulator disable pass\n");
			}


	pinctrl_select_state(lcmctrl, lcd_pwr_low);//add by 
	return ret;
}

static int lcm_probe(struct device *dev)
{
	lcm_get_vgp_supply(dev);
	lcm_get_gpio(dev);

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	pr_notice("LCM: Register lcm driver¥n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver¥n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done¥n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800) //(800)
#define FRAME_HEIGHT (1280) //(1280)
#if 0//add by 
#define GPIO_LCD_RST_EN      GPIO90
#define GPIO_LCD_STB_EN      GPIO66
#define GPIO_LCD_LED_EN      GPIO83 //GPIO76
#define GPIO_LCD_LED_PWM_EN      (GPIO86 | 0x80000000) //GPIO116
#endif //add by 


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

//static LCM_UTIL_FUNCS lcm_util = {0};  //for fixed warning issue
#if 1//add by 
static LCM_UTIL_FUNCS lcm_util = 
{
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};
#endif


#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define   LCM_DSI_CMD_MODE							0
#define TRUE 1
#define FALSE 0
#ifndef BUILD_LK
static bool fgisFirst = TRUE;
#endif
void poweron_3300mv(bool enabled)
{
#ifdef BUILD_LK
	if (enabled) 
	{ 
	/* VGP3_PMU 3.3V */ 
	upmu_set_rg_vgp1_vosel(7); 
	upmu_set_rg_vgp1_en(1); 
	} 
	else 
	{ 
	upmu_set_rg_vgp1_en(0); 
	upmu_set_rg_vgp1_vosel(0); 
	} 
#else
	if (enabled) 
	{
//    hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "LCM");
	}
	else
	{
//	hwPowerDown(MT6323_POWER_LDO_VGP1, "LCM");//
	}
#endif
}

//add by  ==============>>>>>>>>>>>>>>>>

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
	MDELAY(20);

	upmu_set_rg_vgp3_vosel(0);
	upmu_set_rg_vgp3_en(0x0);

#else
	pr_debug("[Kernel/LCM] lcm_suspend_power() enter\n");
	printk(" ==>>> come here %s  \n",__FUNCTION__);
	lcm_set_gpio(0);
	MDELAY(20);

//	upmu_set_rg_vgp1_en(0x0);
	lcm_vgp_supply_disable();
	MDELAY(20);

#endif
}

static void lcm_resume_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(20);
	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(0x1);

#else
	pr_debug("[Kernel/LCM] lcm_resume_power() enter\n");
	printk(" ==>>> come here %s  \n",__FUNCTION__);

	MDELAY(20);
	lcm_set_gpio(1);
	MDELAY(20);
	lcm_vgp_supply_enable();
	MDELAY(30);

#endif
}




// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if 0
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE; //SYNC_EVENT_VDO_MODE;
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		//params->dsi.packet_size=256;

		// Video mode setting		
		//params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		
		params->dsi.vertical_sync_active				= 5; //4; //6; //(12-4-4); //1;
		params->dsi.vertical_backporch					= 3; //6; //4; //6; //10;
		params->dsi.vertical_frontporch					= 8;//6; //4//6; //10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 5; //40; //44; //(120-40-40); //1;
		params->dsi.horizontal_backporch				= 59;//44; //40; //44; //57;
		params->dsi.horizontal_frontporch				= 16;//44; //40; //44; //32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

        params->dsi.PLL_CLOCK = 225;//LCM_DSI_6589_PLL_CLOCK_NULL; //LCM_DSI_6589_PLL_CLOCK_396_5;
    

		//params->dsi.cont_clock = 1;
		//params->dsi.noncont_clock = TRUE; 
		//params->dsi.noncont_clock_period = 2; // Unit : frames

}
//extern void DSI_clk_HS_mode(unsigned char enter);
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

static struct LCM_setting_table lcm_initialization_setting[] = {

	/*
	Note :

	Data ID will depends on the following rule.

		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/


{0xB9,3,{0xFF,0x83,0x94}},
{REGFLAG_DELAY, 10, {}},
//{0xBA,2,{0x23,0x83}},
//{REGFLAG_DELAY, 10, {}},
{0xB1,15,{ 0x64, 0x10, 0x30, 0x43, 0x34 ,0x11 ,0xF1 ,0x81 ,0x70 ,0xD9 ,0x34 ,0x80 ,0xC0 ,0xD2 ,0x41}},
{REGFLAG_DELAY, 10, {}},
{0xB2,12,{ 0x45 ,0x64 ,0x0F ,0x09 ,0x40 ,0x1C ,0x08 ,0x08 ,0x1C ,0x4D ,0x00 ,0x00}},
{REGFLAG_DELAY, 10, {}},
{0xB4,22,{ 0x07 ,0x6E ,0x07 ,0x71 ,0x6F ,0x70 ,0x00 ,0x00 ,0x01 ,0x6E ,0x0F ,0x6E ,0x07 ,0x71 ,0x6F ,0x70 ,0x00 ,0x00 ,0x01 ,0x6E ,0x0F ,0x6E}},
{REGFLAG_DELAY, 10, {}},
{0xb6,2,{0x6F ,0x6F}},
{REGFLAG_DELAY, 10, {}},
{0xCC,1, {0X01}},
{REGFLAG_DELAY, 10, {}},
{0xD3,32, { 0x00 ,0x08 ,0x00 ,0x01 ,0x07 ,0x00 ,0x08 ,0x32 ,0x10 ,0x0A ,0x00 ,0x05 ,0x00 ,0x20 ,0x0A ,0x05 ,0x09 ,0x00 ,0x32 ,0x10 ,0x08 ,0x00 ,0x35 ,0x33 ,0x0D ,0x07 ,0x47 ,0x0D ,0x07 ,0x47 ,0x0F ,0x08}},
{REGFLAG_DELAY, 10, {}},
{0xD5,44, { 0x03 ,0x02 ,0x03 ,0x02 ,0x01 ,0x00 ,0x01 ,0x00 ,0x07 ,0x06 ,0x07 ,0x06 ,0x05 ,0x04 ,0x05 ,0x04 ,0x21 ,0x20 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x23 ,0x22 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18 ,0x18}},
{REGFLAG_DELAY, 10, {}},
{0xE0,42, { 0x03 ,0x17 ,0x1C ,0x2D ,0x30 ,0x3B ,0x27 ,0x40 ,0x08 ,0x0B ,0x0D ,0x18 ,0x0F ,0x12 ,0x15 ,0x13 ,0x14 ,0x07 ,0x12 ,0x14 ,0x17 ,0x03 ,0x17 ,0x1C ,0x2D ,0x30 ,0x3B ,0x27 ,0x40 ,0x08 ,0x0B ,0x0D ,0x18 ,0x0F ,0x12 ,0x15 ,0x13 ,0x14 ,0x07 ,0x12 ,0x14 ,0x17}},
{REGFLAG_DELAY, 10, {}},
{0xCC,1,  {0x09}},
{REGFLAG_DELAY, 10, {}},
{0x11,0,{0x00}},
{REGFLAG_DELAY, 250, {}},
{0xC9,5,  {0x1F ,0x2E ,0x1E ,0x1E ,0x10}},
{REGFLAG_DELAY, 100, {}},
{0x29,0,{0x00}},
{REGFLAG_DELAY, 200, {}},

	// Note
	// Strongly recommend not to set Sleep out / DisplDSI_clk_HS_modeay On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }

}
static void lcm_init_lcm(void)
{
	unsigned int data_array[16];
	printk("[LK/LCM] lcm_init_lcm() enter/n");
if(fgisFirst == TRUE)
{
     fgisFirst = FALSE;
	MDELAY(100);
}
lcm_vgp_supply_enable();
lcm_set_gpio(1);
MDELAY(30);
lcm_set_gpio(0);
MDELAY(30);
lcm_set_gpio(1);
MDELAY(130);
data_array[0]= 0x00043902;
data_array[1]= 0x9483ffb9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(30);
data_array[0]= 0x00103902;
data_array[1]= 0x301064b1;
data_array[2]= 0xf1113443;
data_array[3]= 0x34d97081;
data_array[4]= 0x41d2c080;
dsi_set_cmdq(data_array, 5, 1);
MDELAY(30);
data_array[0]= 0x000d3902;
data_array[1]= 0x0f6445b2;
data_array[2]= 0x081c4009;
data_array[3]= 0x004d1c08;
data_array[4]= 0x00000000;
dsi_set_cmdq(data_array, 5, 1);
MDELAY(30);
data_array[0]= 0x00173902;
data_array[1]= 0x076e07b4;
data_array[2]= 0x00706f71;
data_array[3]= 0x0f6e0100;
data_array[4]= 0x6f71076e;
data_array[5]= 0x01000070;
data_array[6]= 0x006e0f6e;
dsi_set_cmdq(data_array, 7, 1);
data_array[0]= 0x00033902;
data_array[1]= 0x006f6fb6;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x09cc1500;
dsi_set_cmdq(data_array, 1, 1);
data_array[0]= 0x00213902;
data_array[1]= 0xd3000800;
data_array[2]= 0x08000701;
data_array[3]= 0x000a1032;
data_array[4]= 0x0a200005;
data_array[5]= 0x32000905;
data_array[6]= 0x35000810;
data_array[7]= 0x47070d33;
data_array[8]= 0x0f47070d;
data_array[9]= 0x00000008;
dsi_set_cmdq(data_array, 10, 1);
MDELAY(30);
data_array[0]= 0x002d3902;
data_array[1]= 0x030203d5;
data_array[2]= 0x01000102;
data_array[3]= 0x07060700;
data_array[4]= 0x05040506;
data_array[5]= 0x18202104;
data_array[6]= 0x18181818;
data_array[7]= 0x18181818;
data_array[8]= 0x23181818;
data_array[9]= 0x18181822;
data_array[10]= 0x18181818;
data_array[11]= 0x18181818;
data_array[12]= 0x00000018;
dsi_set_cmdq(data_array, 13, 1);
MDELAY(30);
data_array[0]= 0x002b3902;
data_array[1]= 0x1c1703e0;
data_array[2]= 0x273b302d;
data_array[3]= 0x0d0b0840;
data_array[4]= 0x15120f18;
data_array[5]= 0x12071413;
data_array[6]= 0x17031714;
data_array[7]= 0x3b302d1c;
data_array[8]= 0x0b084027;
data_array[9]= 0x120f180d;
data_array[10]= 0x07141315;
data_array[11]= 0x00171412;
dsi_set_cmdq(data_array, 12, 1);
MDELAY(30);
data_array[0] = 0x00110500;        //exit sleep mode 
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(250);   
data_array[0]= 0x00063902;
data_array[1]= 0x1e2e1fc9;
data_array[2]= 0x0000101e;
dsi_set_cmdq(data_array, 3, 1);
data_array[0] = 0x00290500;        //exit sleep mode 
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(20);
}

static   void lcm_init_power(void)
{

	MDELAY(20);
}

static void lcm_suspend(void)
{
	//unsigned int data_array[16];
#if 0///add by 
	//data_array[0]=0x00280500; // Display Off
	//dsi_set_cmdq(data_array, 1, 1);
	#if 1
	mt_set_gpio_mode(GPIO_LCD_LED_PWM_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_PWM_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_PWM_EN, GPIO_OUT_ZERO);
	#endif
	MDELAY(20);
	//
	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ZERO);
	MDELAY(20);
#if (defined BUILD_UBOOT)	
#elif (defined BUILD_LK)
#else
if(fgisFirst == TRUE)
{
     fgisFirst = FALSE;
        hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "LCM");
}
    MDELAY(20);     
	hwPowerDown(MT6323_POWER_LDO_VGP1, "LCM");//
#if 0//defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
	printk(">>>>>>>>>>>>>>>>%s %d START>>>>>>>>>>>>¥n",__func__, __LINE__);
	int status=0;
	chr_control_interface(CHARGING_CMD_SET_CHR_SET,&status);
	printk(">>>>>>>>>>>>>>>>%s %d END>>>>>>>>>>>>¥n",__func__, __LINE__);
}
#endif	
    MDELAY(20); 
#endif
#endif//add by 
	printk(" ==>>> come here %s  \n",__FUNCTION__);
    MDELAY(20); 
}
static void lcm_resume(void)
{

    //MDELAY(30);	
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter¥n");
	//mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	#if 1
	mt_set_gpio_mode(GPIO_LCD_LED_PWM_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_PWM_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_PWM_EN, GPIO_OUT_ONE);
	#endif
	MDELAY(20);
	MDELAY(20);
		
	//mt_set_gpio_mode(GPIO_LCD_STB_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_STB_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_STB_EN, GPIO_OUT_ONE);
	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
	MDELAY(20);
#elif (defined BUILD_UBOOT)
#else
#if 0//defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
	printk(">>>>>>>>>>>>>>>>%s %d START>>>>>>>>>>>>¥n",__func__, __LINE__);
	int status=1;
	chr_control_interface(CHARGING_CMD_SET_CHR_SET,&status);
	printk(">>>>>>>>>>>>>>>>%s %d END>>>>>>>>>>>>¥n",__func__, __LINE__);
}
#endif

#endif
	printk(" ==>>> come here %s  \n",__FUNCTION__);
	if(1)
	lcm_init_lcm();
	else
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
		MDELAY(180);
}
         


LCM_DRIVER ZS080BH3107E3H711_lcm_drv = 
{
    .name			= "ZS080BH3107E3H711",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_lcm,
	.init_power     = lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,

    };
