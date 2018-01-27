#if 0//add by tubao
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
//add by tubao
#include <mach/charging.h>
#endif //add by tubao

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
static struct regulator *lcm_vgp_back_up;//add by tubao
static struct pinctrl *lcmctrl;
static struct pinctrl *lcmrst;//add by tubao
static struct pinctrl_state *lcd_pwr_high;
static struct pinctrl_state *lcd_pwr_low;
static struct pinctrl_state *lcd_rst_high;//add by tubao
static struct pinctrl_state *lcd_rst_low;//add by tubao

static int lcm_get_gpio(struct device *dev)
{
	int ret = 0;
#if 0//add by tubao GPIO90
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

#endif//add by tubao GPIO90   复位引脚
	return ret;
}

void lcm_set_gpio(int val)
{
	if (val == 0) {
		pinctrl_select_state(lcmrst, lcd_rst_low);
		pr_debug("LCM: lcm set power off¥n");
		printk("tubao ==>>> come here %s reset==0 \n",__FUNCTION__);
	} else {
		pinctrl_select_state(lcmrst, lcd_rst_high);
		pr_debug("LCM: lcm set power on¥n");
		printk("tubao ==>>> come here %s reset==1 \n",__FUNCTION__);
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
//add by tubao  ==============>>>>>>>>>>>> for control VIBR start
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

//add by tubao  ==============>>>>>>>>>>>> for control VIBR end


#if 1//add by tubao 用于控制 GPIO83  电源
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

#endif//add by tubao  用于控制 GPIO83  电源
	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

#if 0
	pr_debug("LCM: lcm_vgp_supply_enable¥n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V¥n");
	/* set voltage to 1.8V */
	//ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d¥n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 3300000)
		pr_err("LCM: check regulator voltage=3200000 pass!¥n");
	else
		pr_err("LCM: check regulator voltage=3200000 fail! (voltage: %d)¥n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d¥n", ret);
		return ret;
	}
#endif
	//add by tubao  for contrl vibr power start start
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
	//add by tubao  for contrl vibr power start end
	//add by tubao 用于控制GPIO83 开启电源
	pinctrl_select_state(lcmctrl, lcd_pwr_high);
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
//add by tubao ==>>>> start  === disable VIBR POWER
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

//add by tubao ==>>>> end  === disable VIBR POWER


	//用于控制GPIO83 关闭电源
	pinctrl_select_state(lcmctrl, lcd_pwr_low);//add by tubao
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




//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================
//<<<<<<<<<<<<<<<<<<<<<<=====================


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800) //(800)
#define FRAME_HEIGHT (1280) //(1280)
#if 0//add by tubao
#define GPIO_LCD_RST_EN      GPIO90
#define GPIO_LCD_STB_EN      GPIO66
#define GPIO_LCD_LED_EN      GPIO83 //GPIO76
#define GPIO_LCD_LED_PWM_EN      (GPIO86 | 0x80000000) //GPIO116
#endif //add by tubao


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

//static LCM_UTIL_FUNCS lcm_util = {0};  //for fixed warning issue
#if 1//add by tubao
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

//add by tubao ==============>>>>>>>>>>>>>>>>

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
	lcm_set_gpio(0);
	MDELAY(20);

	upmu_set_rg_vgp1_en(0x0);
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

	
	lcm_set_gpio(1);
	MDELAY(20);
	upmu_set_rg_vgp1_vosel(0x7);
	upmu_set_rg_vgp1_en(0x1);
	lcm_vgp_supply_enable();
	MDELAY(20);

#endif
}


//add by tubao  <<<<<<<<<<<<<<<<====================

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
		//The following defined the fomat for data coming from LCD engine.
		//params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		//params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		//params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		//params->dsi.packet_size=256;

		// Video mode setting		
		//params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		//params->dsi.word_count=1920*3; //720*3;	

		
		params->dsi.vertical_sync_active				= 5; //4; //6; //(12-4-4); //1;
		params->dsi.vertical_backporch					= 3; //6; //4; //6; //10;
		params->dsi.vertical_frontporch					= 8;//6; //4//6; //10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 5; //40; //44; //(120-40-40); //1;
		params->dsi.horizontal_backporch				= 59;//44; //40; //44; //57;
		params->dsi.horizontal_frontporch				= 16;//44; //40; //44; //32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_div1=1; //0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =22;//31;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		//params->dsi.fbk_sel =0;
		//params->dsi.fbk_div =45;
        params->dsi.PLL_CLOCK = 225;//LCM_DSI_6589_PLL_CLOCK_NULL; //LCM_DSI_6589_PLL_CLOCK_396_5;
    
		//params->dsi.CLK_ZERO = 262; //47;
		//params->dsi.HS_ZERO = 117; //36;

		//params->dsi.cont_clock = 1;
		//params->dsi.noncont_clock = TRUE; 
		//params->dsi.noncont_clock_period = 2; // Unit : frames

}
//extern void DSI_clk_HS_mode(unsigned char enter);
static void lcm_init_lcm(void)
{
	//unsigned int data_array[16];
	unsigned int data_array[16];
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_lcm() enter¥n");
	printf("[LK/LCM] lcm_init_lcm() hengqiu add test enter¥n");
    upmu_set_rg_vgp1_vosel(0x7);
    upmu_set_rg_vgp1_en(0x1);
	printf("[LK/LCM] lcm_init() hengqiu set 3.3v en¥n");
	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
	MDELAY(20);
	//mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	
	//mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	
	//mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);

	//mt_set_gpio_mode(GPIO_LCD_STB_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_STB_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_STB_EN, GPIO_OUT_ONE);
#if 0//defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
	mt_set_gpio_mode(GPIO_MSDC2_CMD, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_MSDC2_CMD,GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_MSDC2_CMD, GPIO_OUT_ONE);
}
#endif
#elif (defined BUILD_UBOOT)
#else
if(fgisFirst == TRUE)
{
     fgisFirst = FALSE;
//        hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "LCM");
	MDELAY(100);
}
#if 0//defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
	printk(">>>>>>>>>>>>>>>>%s %d START>>>>>>>>>>>>¥n",__func__, __LINE__);
	int status=0;
	chr_control_interface(CHARGING_CMD_SET_CHR_SET,&status);
	printk(">>>>>>>>>>>>>>>>%s %d END>>>>>>>>>>>>¥n",__func__, __LINE__);
}
#endif
#endif
#if 0//add by tubao
	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_LCD_LED_PWM_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_PWM_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_PWM_EN, GPIO_OUT_ONE);
#endif //add by tubao
	MDELAY(10);
#if 0//add by tubao ===>>> start
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	
	MDELAY(20);
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	
	MDELAY(50);
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(150);
#endif//add by tubao  <<======end
#if 1	
data_array[0]= 0x00053902;
data_array[1]= 0xa555aaff;
data_array[2]= 0x00000080;
dsi_set_cmdq(data_array, 3, 1);
data_array[0]= 0x00033902;
data_array[1]= 0x0000116f;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00033902;
data_array[1]= 0x000020f7;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;
data_array[1]= 0x0000066f;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;
data_array[1]= 0x0000a0f7;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;
data_array[1]= 0x0000196f;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;
data_array[1]= 0x000012f7;
dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00023902;
data_array[1]= 0x000003f4;
dsi_set_cmdq(data_array, 2, 1);




data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000008;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000168b1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000008b6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00043902;
data_array[1]= 0x080201b8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x004444bb;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000000bc;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x106802bd;
data_array[2]= 0x00000010;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000080c8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000108;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x004f4fb3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001010b4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000505b5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003535b9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002525ba;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000068bc;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000068bd;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x00000cc0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000000ca;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000208;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000001ee;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00113902;
data_array[1]= 0x000000b0;
data_array[2]= 0x002a000f;
data_array[3]= 0x00540040;
data_array[4]= 0x00930076;
data_array[5]= 0x000000c5;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x00113902;
data_array[1]= 0x01f000b1;
data_array[2]= 0x01660132;
data_array[3]= 0x02ff01bb;
data_array[4]= 0x02420201;
data_array[5]= 0x00000085;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x00113902;
data_array[1]= 0x02af02b2;
data_array[2]= 0x030503e0;
data_array[3]= 0x03540335;
data_array[4]= 0x03a00384;
data_array[5]= 0x000000c4;
dsi_set_cmdq(data_array, 6, 1);

data_array[0]= 0x00053902;
data_array[1]= 0x03f203b3;
data_array[2]= 0x000000ff;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000308;
dsi_set_cmdq(data_array, 3, 1);					

data_array[0]= 0x00033902;
data_array[1]= 0x000000b0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000000b1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x170008b2;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x000005b6;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00063902;
data_array[1]= 0xa00053ba;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00063902;
data_array[1]= 0xa00053bb;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00053902;
data_array[1]= 0x000000c0;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00053902;
data_array[1]= 0x000000c1;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000060c4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x0000c0c5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000508;
dsi_set_cmdq(data_array, 3, 1);	

data_array[0]= 0x00033902;
data_array[1]= 0x000617b0;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00033902;
data_array[1]= 0x000617b1;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00033902;
data_array[1]= 0x000617b2;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00033902;
data_array[1]= 0x000617b3;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00033902;
data_array[1]= 0x000617b4;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00033902;
data_array[1]= 0x000617b5;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x00000cb8;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x000000b9;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x000000ba;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x00000abb;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x000002bc;
dsi_set_cmdq(data_array, 2, 1);	

data_array[0]= 0x00063902;
data_array[1]= 0x010103bd;
data_array[2]= 0x00000303;
dsi_set_cmdq(data_array, 3, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x000007c0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x0000a2c4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002003c8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002101c9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00043902;
data_array[1]= 0x010000cc;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00043902;
data_array[1]= 0x010000cd;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0xfc0400d1;
data_array[2]= 0x00001407;
dsi_set_cmdq(data_array, 3, 1);	

data_array[0]= 0x00063902;
data_array[1]= 0x000510d2;
data_array[2]= 0x00001603;
dsi_set_cmdq(data_array, 3, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x000006e5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e7;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006ea;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00023902;
data_array[1]= 0x000030ed;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000608;
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001117b0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001016b1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001812b2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001913b3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003100b4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003431b5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002934b6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x00332ab7;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002d2eb8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003408b9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000834ba;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2dbb;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002a34bc;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003429bd;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003134be;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000031bf;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001319c0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001218c1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001610c2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001711c3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003434e5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001812c4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001913c5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001117c6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001016c7;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003108c8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003431c9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002934ca;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x00332acb;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2dcc;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003400cd;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000034ce;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002d2ecf;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x002a34d0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003429d1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003134d2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x000831d3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001610d4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001711d5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001319d6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x001218d7;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00033902;
data_array[1]= 0x003434e6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x000000d8;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);	

data_array[0]= 0x00063902;
data_array[1]= 0x000000d9;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);	

data_array[0]= 0x00023902;
data_array[1]= 0x000000e7;
dsi_set_cmdq(data_array, 2, 1);

   data_array[0] = 0x00110500;        //exit sleep mode 
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(200);   
    
    data_array[0] = 0x00290500;        //exit sleep mode 
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(20);
data_array[0]= 0x00023902;
data_array[1]= 0x00000035;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000108;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(20);
	
#elif 0//D??ｨ｢
	data_array[0]=0x00043902;
data_array[1]=0x038198ff;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000001;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000002;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00005603;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001304;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000005;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000606;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000207;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000008;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000109;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000020a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000000b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000010c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000010d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000000e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000050f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000510;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000011;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000012;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000013;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000014;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000015;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000016;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000017;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000018;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000019;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000001a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000001b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000001c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000001d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000441e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000801f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000220;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000321;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000022;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000023;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000024;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000025;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000026;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000027;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00007328;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000329;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000002a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000002b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000002c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000002d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000002e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000002f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000030;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000031;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000032;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000033;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000434;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000035;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000736;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000037;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00008038;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000039;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000003a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000003b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000003c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000003d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000003e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000003f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000040;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000041;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000042;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000043;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000044;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000050;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002351;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00004452;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00006753;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00008954;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000ab55;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000056;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001157;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002258;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00003359;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000445a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000555b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000665c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000775d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000115e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000115f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000d60;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001061;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000c62;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000e63;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001264;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000f65;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001366;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000667;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000268;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000269;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000026a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000026b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000026c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000026d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000146e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000156f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000270;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000071;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000172;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000873;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000274;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001175;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000d76;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001077;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000c78;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000e79;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000127a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000f7b;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000137c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000087d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000027e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000027f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000280;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000281;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000282;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000283;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001484;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001585;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000286;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000087;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000188;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000689;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000028a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00043902;
data_array[1]=0x048198ff;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000156c;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002b6e;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000336f;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000243a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000158d;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000ba87;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00007626;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000d1b2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000007b5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001f35;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000107a;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00043902;
data_array[1]=0x018198ff;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000a22;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00000031;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00004c53;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00008a55;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000af50;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000af51;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001460;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000008a0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000022a1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000031a2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000011a3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000015a4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000029a5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001ea6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001fa7;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000a9a8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001da9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002aaa;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000098ab;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001eac;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001cad;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000051ae;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000026af;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002ab0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000057b1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000067b2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000039b3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000008c0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000022c1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000031c2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000011c3;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000015c4;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000029c5;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001ec6;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001fc7;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x0000a9c8;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001dc9;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002aca;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000098cb;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001ecc;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00001ccd;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000052ce;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000027cf;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x00002bd0;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000055d1;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000067d2;
dsi_set_cmdq(data_array, 2, 1);

data_array[0]=0x00023902;
data_array[1]=0x000039d3;
dsi_set_cmdq(data_array, 2, 1);


data_array[0]=0x00043902;
data_array[1]=0x008198ff;
dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(150);

data_array[0]= 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(30);//5000

data_array[0]= 0x00350500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(30);//5000
#else
data_array[0]=0x00053902;
data_array[1]=0xa555aaff;
data_array[2]=0x00000080;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00033902;
data_array[1]=0x0000116f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00033902;
data_array[1]=0x000020f7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x0000066f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x0000a0f7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x0000196f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000012f7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000003f4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x0000086f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000040fa;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x0000116f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000001f3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x0000066f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000003fc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00063902;
data_array[1]=0x52aa55f0;
data_array[2]=0x00000008;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000168b1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000008b6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x0000026f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x000008b8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00033902;
data_array[1]=0x004454bb;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00033902;
data_array[1]=0x000505bc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x000001c7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00063902;
data_array[1]=0x1eb002bd;
data_array[2]=0x0000001e;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000701c5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000080c8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00063902;
data_array[1]=0x52aa55f0;
data_array[2]=0x00000108;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000505b0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000505b1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000000b2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000190bc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000190bd;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000000ca;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000004c0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000029be;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x002828b3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x001212b4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x003535b9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x002525ba;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00063902;
data_array[1]=0x52aa55f0;
data_array[2]=0x00000208;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);


data_array[0]=0x00023902;
data_array[1]=0x000001ee;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00053902;
data_array[1]=0x150609ef;
data_array[2]=0x00000018;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);


data_array[0]=0x00073902;
data_array[1]=0x000000b0;
data_array[2]=0x00550024;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x0000066f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00073902;
data_array[1]=0x007700b0;
data_array[2]=0x00c00094;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x00000c6f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00053902;
data_array[1]=0x01e300b0;
data_array[2]=0x0000001a;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00073902;
data_array[1]=0x014601b1;
data_array[2]=0x00bc0188;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x0000066f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00073902;
data_array[1]=0x020b02b1;
data_array[2]=0x004d024b;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00023902;
data_array[1]=0x00000c6f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]=0x00053902;
data_array[1]=0x028802b1;
data_array[2]=0x000000c9;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00073902;
data_array[1]=0x03f302b2;
data_array[2]=0x004e0329;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);


data_array[0]=0x00023902;
data_array[1]=0x0000066f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);


data_array[0]=0x00073902;
data_array[1]=0x037d03b2;
data_array[2]=0x00be039b;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);


data_array[0]=0x00023902;
data_array[1]=0x00000c6f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);


data_array[0]=0x00053902;
data_array[1]=0x03d303b2;
data_array[2]=0x000000e9;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00053902;
data_array[1]=0x03fb03b3;
data_array[2]=0x000000ff;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00063902;
data_array[1]=0x52aa55f0;
data_array[2]=0x00000608;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e0bb0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);


data_array[0]= 0x00033902;
data_array[1]= 0x002e2eb1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00092eb2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00292ab3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00191bb4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001517b5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001113b6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e01b7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2eb8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2eb9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2eba;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ebb;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00002ebc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001210bd;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001614be;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001a18bf;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002a29c0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e08c1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ec2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000a2ec3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ee5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);





data_array[0]= 0x00033902;
data_array[1]= 0x002e0ac4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ec5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00002ec6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00292ac7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001210c8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001614c9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001a18ca;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e08cb;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ecc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ecd;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ece;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ecf;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00092ed0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x00191bd1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001517d2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x001113d3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002a29d4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e01d5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ed6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000b2ed7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x002e2ee6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000000d8;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000000d9;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000000e7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000308;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000020b0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000020b1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000005b2;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000005b6;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000005b7;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000057ba;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x000057bb;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00053902;
data_array[1]= 0x000000c0;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00053902;
data_array[1]= 0x000000c1;
data_array[2]= 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000060c4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000040c5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x52aa55f0;
data_array[2]= 0x00000508;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x030103bd;
data_array[2]= 0x00000303;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000617b0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000617b1;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000617b2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000617b3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000617b4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x000617b5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000000b8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000000b9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000000ba;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000002bb;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000000bc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000007c0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000080c4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x0000a4c5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x003005c8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00033902;
data_array[1]= 0x003101c9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00043902;
data_array[1]= 0x3c0000cc;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00043902;
data_array[1]= 0x3c0000cd;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x090500d1;
data_array[2]= 0x00001007;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00063902;
data_array[1]= 0x0e0500d2;
data_array[2]= 0x00001007;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e5;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e7;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e8;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006e9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000006ea;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000030ed;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x0000116f;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0]= 0x00023902;
data_array[1]= 0x000001f3;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00350500;         
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(10);   

data_array[0] = 0x00110500;        //exit sleep mode 
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(200);   

data_array[0] = 0x00290500;        //exit sleep mode 
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(20);
	
#endif
}

static   void lcm_init_power(void)
{
//	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
//	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
	MDELAY(20);
}

static void lcm_suspend(void)
{
	//unsigned int data_array[16];
#if 0///add by tubao
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
#endif//add by tubao
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);
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
   // hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "LCM");
//	MDELAY(20);
#endif
//	lcm_init_power();
	//unsigned int data_array[16];
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);
	lcm_init_lcm();

	//data_array[0] = 0x00101500; // Sleep Out
	//dsi_set_cmdq(data_array, 1, 1);
//	MDELAY(10);

	//data_array[0] = 0x00290500; // Display On
	//dsi_set_cmdq(data_array, 1, 1);

}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

LCM_DRIVER KD101N42_40NA_C1_MIPI_lcm_drv =
{
    .name			= "KD101N42_40NA_C1_MIPI_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_lcm,
	.init_power     = lcm_init_power,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
