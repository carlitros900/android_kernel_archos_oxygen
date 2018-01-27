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
static struct regulator *lcm_vpa_back_up;//add by tubao
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
//add by tubao  ==============>>>>>>>>>>> for control VPA power start
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
//add by tubao  ==============>>>>>>>>>>> for control VPA power end
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

#if 1
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


//add by tubao  ==>>>>>> for control VPA power start
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
//add by tubao  ==>>>>>> for control VPA power end


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

//add by tubao  ===>>>>>>> shutdown VPA power start
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

//add by tubao  ===>>>>>>> shutdown VPA power end


	//add by tubao   shutdown VIBR power  start
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


	//add by tubao   shutdown VIBR power  end
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
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER



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
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);
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
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);

	
	lcm_set_gpio(1);
	MDELAY(20);
//	upmu_set_rg_vgp1_vosel(0x7);
//	upmu_set_rg_vgp1_en(0x1);
	lcm_vgp_supply_enable();
	MDELAY(20);

#endif
}


//add by tubao  <<<<<<<<<<<<<<<<====================

//add by tubao ====>>> start
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


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


{0xB0,1,{0x00}},
{0xB1,1,{0x21}},
{0xB2,1,{0x4b}},
{0xB3,1,{0x68}},
{0xB7,1,{0x73}},
{0xBA,1,{0xAB}},
{0xBB,1,{0xE8}},
{0xBE,1,{0x3F}},
{0xBD,1,{0xFF}},
{0xC3,1,{0x70}},
{0xC4,1,{0x19}},
{0xC5,1,{0xB9}},
{0xC6,1,{0x19}},                 
{0xC7,1,{0x17}},                  
{0xCA,1,{0x00}},                  
{0xCB,1,{0x3F}},                  
{0xCC,1,{0x30}},                                  
{0xCD,1,{0x27}},                                  
{0xCE,1,{0x27}},                                  
{0xCF,1,{0x16}},                                  
{0xD0,1,{0x0E}},                                  
{0xD1,1,{0x0E}},                                  
{0xD2,1,{0x13}},                                  
{0xD3,1,{0x14}},                                  
{0xD4,1,{0x14}},                                  
{0xD5,1,{0x0A}},                                  
{0xD6,1,{0x3F}}, 
//page 2
                                 
{0xD7,1,{0x30}},                                  
{0xDB,1,{0x27}},                                  
{0xD9,1,{0x27}},                                  
{0xDA,1,{0x16}},                                  
{0xDB,1,{0x0E}},                                  
{0xDC,1,{0x0E}},   
//page 3
                               
{0xDD,1,{0x13}},                                  
{0xDE,1,{0x14}},                                      
{0xDF,1,{0x14}},                                                  
{0xE0,1,{0x0A}},                                                  
{0xE5,1,{0x52}},                                                  
{0xB0,1,{0x02}},                                                  
{0xB1,1,{0x25}},                                                  
{0xB2,1,{0x00}},                                                  
{0xB3,1,{0x11}},                                                  
{0xB4,1,{0x12}},                                                  
{0xB5,1,{0x09}}, 
//page 4
                                                 
{0xB6,1,{0x0B}},                                                  
{0xB7,1,{0x05}},                                                  
{0xB8,1,{0x07}},                                                  
{0xB9,1,{0x01}},                                                  
{0xBA,1,{0x00}},                                                  
{0xBB,1,{0x00}},                                                  
{0xBC,1,{0x00}},                                                  
{0xBD,1,{0x00}},                                                  
{0xBE,1,{0x00}},                                                  
{0xBF,1,{0x00}},                                                  
{0xC0,1,{0x03}},                                                  
{0xC1,1,{0x00}},                                                  
{0xC2,1,{0x00}},                                                  
{0xC3,1,{0x00}},                                                  
{0xC4,1,{0x00}},                                                  
{0xC5,1,{0x00}},                                                  
{0xC6,1,{0x00}},                                                      
{0xC7,1,{0x25}},                                                  
{0xC8,1,{0x00}},                                                  
{0xC9,1,{0x11}},                                                      
{0xCA,1,{0x12}},                                                      
{0xCB,1,{0x0A}},                                                      
{0xCC,1,{0x0C}},                                                      
{0xCD,1,{0x06}},                                                      
{0xCE,1,{0x08}},                                                              
{0xCF,1,{0x02}},                                                      
{0xD0,1,{0x00}},                                                     
{0xD1,1,{0x00}},  
//page 5
                                                   
{0xD2,1,{0x00}},                                                     
{0xD3,1,{0x00}},                                                     
{0xD4,1,{0x00}},                                                         
{0xD5,1,{0x00}},                                                         
{0xD6,1,{0x04}},                                                     
{0xD7,1,{0x00}},                                                             
{0xD8,1,{0x00}},                                                     
{0xD9,1,{0x00}},                                                         
{0xDA,1,{0x00}},                                                             
{0xDB,1,{0x00}},                                                         
{0xDC,1,{0x00}},                                                     
{0xB0,1,{0x03}},                                                      
{0xB6,1,{0x10}},                 
{0xB7,1,{0x00}},
{0xBa,1,{0x21}},
{0xBe,1,{0x21}},
{0xc3,1,{0x03}},
{0xC4,1,{0x0a}},                                            
{0xc5,1,{0x03}}, 
{0xC6,1,{0x06}},                                                         
{0xC7,1,{0x01}},                                                              
{0xCA,1,{0x3c}},                                                          
{0xCB,1,{0x04}},                                                          
{0xCC,1,{0x05}},                                                              
{0x11,0,{0x00}},
{REGFLAG_DELAY, 200, {}},
{0x29,0,{0x00}},
{REGFLAG_DELAY, 50, {}},

	// Note
	// Strongly recommend not to set Sleep out / DisplDSI_clk_HS_modeay On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif
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



//add by tubao ====>>> end



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

		
		params->dsi.vertical_sync_active				= 2;//4;//5; //4; //6; //(12-4-4); //1;
		params->dsi.vertical_backporch					= 10;//3; //6; //4; //6; //10;
		params->dsi.vertical_frontporch					= 12;//8;//6; //4//6; //10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 24;//5; //40; //44; //(120-40-40); //1;
		params->dsi.horizontal_backporch				= 24;//132;//59;//44; //40; //44; //57;
		params->dsi.horizontal_frontporch				= 72;//24;//16;//44; //40; //44; //32;
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
//	unsigned int data_array[16];
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

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

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

LCM_DRIVER SE20811010280036_lcm_drv = 
{
    .name			= "SE20811010280036_lcm_drv",
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
