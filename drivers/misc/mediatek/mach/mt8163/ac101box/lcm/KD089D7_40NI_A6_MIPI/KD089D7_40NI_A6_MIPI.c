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
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

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
	if (IS_ERR(lcd_pwr_low)) {
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
	struct regulator *lcm_vpa_ldo;

	pr_debug("LCM: lcm_get_vgp_supply is going¥n");

	lcm_vpa_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vpa_ldo)) {
		ret = PTR_ERR(lcm_vpa_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d¥n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.¥n");

	ret = regulator_enable(lcm_vpa_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vpa_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage¥n", ret);

	lcm_vgp = lcm_vpa_ldo;


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
#if 0
	int ret;
	unsigned int volt;

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
//add by tubao ==>>>start
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//add by tubao  <<<<======end


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

//add by tubao ================>>>>>>>>>>>>>>>start
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
{0xFF,4,{0xAA,0x55,0xA5,0x80}},////======Internal  setting======      
{REGFLAG_DELAY, 10, {}},          
{0x6F,2,{0x11,0x00}},           
{REGFLAG_DELAY, 10, {}},                                                
{0xF7,2,{0x20,0x00}},   
{REGFLAG_DELAY, 10, {}},                                                        
{0x6F,1,{0x06}},    
{REGFLAG_DELAY, 10, {}},                                                            
{0xF7,1,{0xA0}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0x6F,1,{0x19}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0xF7,1,{0x12}},      
{REGFLAG_DELAY, 10, {}},                                                          
{0xF4,1,{0x03}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0x6F,1,{0x08}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0xFA,1,{0x40}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0x6F,1,{0x11}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0xF3,1,{0x01}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0x6F,1,{0x06}}, 
{REGFLAG_DELAY, 10, {}},                                                               
{0xFC,1,{0x03}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},////======page0  relative  ========      
{REGFLAG_DELAY, 10, {}},   
{0xC8,1,{0x80}},       
{REGFLAG_DELAY, 10, {}},                                                         
{0xB1,2,{0x68,0x01}},   
{REGFLAG_DELAY, 10, {}},                                                        
{0xB6,1,{0x08}}, 
{REGFLAG_DELAY, 10, {}},                                                               
{0x6F,1,{0x02}},       
{REGFLAG_DELAY, 10, {}},                                                         
{0xB8,1,{0x08}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0xBB,2,{0x74,0x44}},  
{REGFLAG_DELAY, 10, {}},                                                         
{0xBC,2,{0x05,0x05}},  
{REGFLAG_DELAY, 10, {}},                                                         
{0xC7,1,{0x01}},        
{REGFLAG_DELAY, 10, {}},                                                        
{0xBD,5,{0x02,0xB0,0x0C,0x12,0x00}},
{REGFLAG_DELAY, 10, {}},                                            
{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},////======  page1  relative ======= 
{REGFLAG_DELAY, 10, {}},        
{0xB0,2,{0x05,0x05}},               
{REGFLAG_DELAY, 10, {}},                                            
{0xB1,2,{0x05,0x05}},   
{REGFLAG_DELAY, 10, {}},                                                        
{0xBC,2,{0x90,0x01}},  
{REGFLAG_DELAY, 10, {}},                                                         
{0xBD,2,{0x90,0x01}},                                                           
{0xCA,1,{0x00}},                                                                
{0xC0,1,{0x0C}},                                                                
{0xBE,1,{0x4C}},                                                                
 {REGFLAG_DELAY, 10, {}},                                                                                
{0xB3,2,{0x37,0x37}},                                                           
{0xB4,2,{0x0F,0x0F}},                                                           
{0xB9,2,{0x45,0x45}},                                                           
{0xBA,2,{0x25,0x25}},  
{REGFLAG_DELAY, 10, {}},                                                          
{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},////======page2  relative========   
{REGFLAG_DELAY, 10, {}},         
{0xEE,1,{0x02}},                                                                
{0xEF,4,{0x09,0x06,0x15,0x18}},                                                 
{0xB0,6,{0x00,0x00,0x00,0x19,0x00,0x31}},
{REGFLAG_DELAY, 10, {}},                                        
{0x6F,1,{0x06}},                                                                
{0xB0,6,{0x00,0x4B,0x00,0x63,0x00,0x95}},   
{REGFLAG_DELAY, 10, {}},                                     
{0x6F,1,{0x0C}},                                                                
{0xB0,4,{0x00,0xB4,0x00,0xF3}},                                                 
{0xB1,6,{0x01,0x24,0x01,0x70,0x01,0xA6}},  
{REGFLAG_DELAY, 10, {}},                                      
{0x6F,1,{0x06}},                                                                
{0xB1,6,{0x01,0xFC,0x02,0x49,0x02,0x4D}},
{REGFLAG_DELAY, 10, {}},                                        
{0x6F,1,{0x0C}},                                                                
{0xB1,4,{0x02,0x9A,0x02,0xF3}},                                                 
{0xB2,6,{0x03,0x23,0x03,0x61,0x03,0x83}},  
{REGFLAG_DELAY, 10, {}},                                      
{0x6F,1,{0x06}},                                                                
{0xB2,6,{0x03,0xA2,0x03,0xB6,0x03,0xCB}},
{REGFLAG_DELAY, 10, {}},                                        
{0x6F,1,{0x0C}},                                                                
{0xB2,4,{0x03,0xD9,0x03,0xE8}},                                                 
{0xB3,4,{0x03,0xF7,0x03,0xFF}},                                                 
{0xBC,6,{0x00,0x00,0x00,0x19,0x00,0x31}},
{REGFLAG_DELAY, 10, {}},                                        
{0x6F,1,{0x06}},                                                                
{0xBC,6,{0x00,0x4B,0x00,0x63,0x00,0x95}},                                       
{0x6F,1,{0x0C}},                                                                
{0xBC,4,{0x00,0xB4,0x00,0xF3}},                                                 
{0xBD,6,{0x01,0x24,0x01,0x70,0x01,0xA6}}, 
{REGFLAG_DELAY, 10, {}},                                        
{0x6F,6,{0x06}},                                                                
{0xBD,6,{0x01,0xFC,0x02,0x49,0x02,0x4D}},                                       
{0x6F,1,{0x0C}},                                                                
{0xBD,4,{0x02,0x9A,0x02,0xF3}},                                                 
{REGFLAG_DELAY, 10, {}},                                                                                 
{0xBE,6,{0x03,0x23,0x03,0x61,0x03,0x83}},                                       
{0x6F,1,{0x06}},                                                                
{0xBE,6,{0x03,0xA2,0x03,0xB6,0x03,0xCB}}, 
{REGFLAG_DELAY, 10, {}},                                       
{0x6F,1,{0x0C}},                                                                
{0xBE,4,{0x03,0xD9,0x03,0xE8}},                                                 
{0xBF,4,{0x03,0xF7,0x03,0xFF}},       
{REGFLAG_DELAY, 10, {}},                                           
{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},//=======>>>>>>>GOA relative========   
{REGFLAG_DELAY, 10, {}},      
{0xB0,2,{0x0B,0x31}},                                                           
{0xB1,2,{0x31,0x09}},                                                           
{0xB2,2,{0x2A,0x29}},                                                           
{0xB3,2,{0x1B,0x19}},                                                           
{0xB4,2,{0x17,0x15}},                                                           
{0xB5,2,{0x31,0x13}},                                                           
{0xB6,2,{0x31,0x11}},                                                           
{0xB7,2,{0x11,0x11}},                                                           
{0xB8,2,{0x31,0x01}},                                                           
{0xB9,2,{0x31,0x31}},                                                           
{0xBA,2,{0x31,0x31}},                                                           
{0xBB,2,{0x00,0x31}},                                                           
{0xBC,2,{0x31,0x10}},                                                           
{0xBD,2,{0x10,0x12}},                                                           
{0xBE,2,{0x31,0x31}},                                                           
{0xBF,2,{0x14,0x16}},                                                           
{0xC0,2,{0x18,0x1A}},                                                           
{0xC1,2,{0x29,0x2A}},                                                           
{0xC2,2,{0x08,0x31}},                                                           
{0xC3,2,{0x31,0x0A}},                                                           
{0xE5,2,{0x31,0x31}},                                                           
{0xC4,2,{0x0A,0x31}},                                                           
{0xC5,2,{0x31,0x00}},                                                           
{0xC6,2,{0x2A,0x29}},                                                           
{0xC7,2,{0x10,0x12}},                                                           
{0xC8,2,{0x14,0x16}},                                                           
{0xC9,2,{0x31,0x18}},                                                           
{0xCA,2,{0x31,0x1A}},                                                           
{0xCB,2,{0x1A,0x1A}},                                                           
{0xCC,2,{0x31,0x08}},                                                           
{0xCD,2,{0x31,0x31}},                                                           
{0xCE,2,{0x31,0x31}},                                                           
{0xCF,2,{0x09,0x31}},                                                           
{0xD0,2,{0x31,0x1B}},                                                           
{0xD1,2,{0x1B,0x19}},                                                           
{0xD2,2,{0x31,0x31}},                                                           
{0xD3,2,{0x17,0x15}},                                                           
{0xD4,2,{0x13,0x11}},                                                           
{0xD5,2,{0x29,0x2A}},                                                           
{0xD6,2,{0x01,0x31}},                                                           
{0xD7,2,{0x31,0x0B}},                                                           
{0xE6,2,{0x31,0x31}},   
{REGFLAG_DELAY, 10, {}},                                                         
{0xD8,5,{0x00,0x00,0x00,0x00,0x00}},                                            
{0xD9,5,{0x00,0x00,0x00,0x00,0x00}},                                            
{0xE7,1,{0x00}},                      
{REGFLAG_DELAY, 10, {}},                                           
{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},//=======>>>>>>>PAGE3      
{REGFLAG_DELAY, 10, {}},                  
{0xB0,2,{0x20,0x00}},                                                           
{0xB1,2,{0x20,0x00}},                                                           
{0xB2,5,{0x05,0x00,0x00,0x00,0x00}},                                            
{0xB6,5,{0x05,0x00,0x00,0x00,0x00}},                                            
{0xB7,5,{0x05,0x00,0x00,0x00,0x00}},                                            
{0xBA,5,{0x57,0x00,0x00,0x00,0x00}},                                            
{0xBB,5,{0x57,0x00,0x00,0x00,0x00}},                                            
{0xC0,4,{0x00,0x00,0x00,0x00}},                                                 
{0xC1,4,{0x00,0x00,0x00,0x00}},                                                 
{0xC4,1,{0x60}},                                                                
{0xC5,1,{0x40}},          
{REGFLAG_DELAY, 10, {}},                                                       
{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},//=======>>>>>>>PAGE5                       
{0xBD,5,{0x03,0x01,0x03,0x03,0x03}},                                            
{0xB0,2,{0x17,0x06}},                                                           
{0xB1,2,{0x17,0x06}},                                                           
{0xB2,2,{0x17,0x06}},                                                           
{0xB3,2,{0x17,0x06}},                                                           
 {REGFLAG_DELAY, 10, {}},                                                                                
{0xB4,2,{0x17,0x06}},                                                           
{0xB5,2,{0x17,0x06}},                                                           
{REGFLAG_DELAY, 10, {}},                                                                                 
{0xB8,1,{0x00}},                                                                
{0xB9,1,{0x00}},                                                                
{0xBA,1,{0x00}},                                                                
{0xBB,1,{0x02}},                                                                
{0xBC,1,{0x00}},                                                                
{0xC0,1,{0x07}},                                                                
{0xC4,1,{0x80}},                                                                
{0xC5,1,{0xA4}},                                                                
{0xC8,2,{0x05,0x30}},                                                           
{0xC9,2,{0x01,0x31}},                                                           
{0xCC,3,{0x00,0x00,0x3C}},                                                      
{0xCD,3,{0x00,0x00,0x3C}},                                                      
{0xD1,5,{0x00,0x05,0x03,0x07,0x10}},                                            
{0xD2,5,{0x00,0x05,0x09,0x07,0x10}},                                            
{0xE5,1,{0x06}},                                                                
{0xE6,1,{0x06}},                                                                
{0xE7,1,{0x06}},                                                                
{0xE8,1,{0x06}},                                                                
{0xE9,1,{0x06}},                                                                
{0xEA,1,{0x06}},                                                                
{0xED,1,{0x30}},                                                                
{0x6F,1,{0x11}},                                                                
{0xF3,1,{0x01}},                                                                
{0x35,0,{0x00}},                                                                
 {REGFLAG_DELAY, 30, {}},                                                                                
{0x11,0,{0x00}},                                                                
{REGFLAG_DELAY, 150, {}}, 
{0x29,0,{0x00}},
{REGFLAG_DELAY, 200, {}}, 

#if 0//add by tubao  ===>>>>> start
{0xB9,3,{0xFF,0x83,0x94}},
{REGFLAG_DELAY, 10, {}},
{0xBA,2,{0x23,0x83}},
{REGFLAG_DELAY, 10, {}},
{0xB1,15,{0x6C,0x0D,0x0D,0x29,0x09,0x17,0xF7,0x81,0x69,0xD8,0x23,0x80,0xC0,0xD2,0x58}},
{REGFLAG_DELAY, 10, {}},
{0xB2,6,{0x00,0x64,0x10,0x07,0x40,0x1C}},
{REGFLAG_DELAY, 10, {}},
{0xB4,12,{0x00,0xFF,0x01,0x6E,0x01,0x6B,0x01,0x6B,0x01,0x6C,0x01,0x6C}},
{REGFLAG_DELAY, 10, {}},
{0xD2,1,{0x00}},
{REGFLAG_DELAY, 10, {}},
{0xD3,32, {0x00,0x00,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x05,0x00,0x02,0x00,0x05,0x0C,0x05,0x09,0x00,0x00,0x00,0x00,0x00,0x37,0x33,0x09,0x06,0x37,0x09,0x06,0x37,0x0A,0x08}},
{REGFLAG_DELAY, 10, {}},
{0xD5,44, {0x05,0x15,0x01,0x11,0x04,0x14,0x00,0x10,0x02,0x12,0x06,0x16,0x03,0x13,0x07,0x17,0x20,0x21,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1A,0x1A,0x1B,0x1B,0x18,0x18,0x18,0x18,0x19,0x19,0x24,0x25,0x18,0x18}},
{REGFLAG_DELAY, 10, {}},
{0xD6,44, {0x02,0x12,0x06,0x16,0x03,0x13,0x07,0x17,0x05,0x15,0x01,0x11,0x04,0x14,0x00,0x10,0x24,0x25,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1A,0x1A,0x1B,0x1B,0x18,0x18,0x19,0x19,0x18,0x18,0x20,0x21,0x18,0x18}},
{REGFLAG_DELAY, 10, {}},
{0xE0,42, {0x00,0x00,0x01,0x2E,0x34,0x3F,0x0F,0x3A,0x06,0x08,0x0A,0x16,0x0E,0x12,0x14,0x13,0x14,0x07,0x11,0x14,0x17,0x00,0x00,0x02,0x2D,0x33,0x3F,0x0F,0x3A,0x04,0x08,0x0A,0x16,0x0E,0x11,0x14,0x12,0x14,0x08,0x13,0x13,0x18}},
{REGFLAG_DELAY, 10, {}},
{0xBF,3,  {0x41,0x0E,0x00}},
{REGFLAG_DELAY, 10, {}},
{0xC0,2,  {0x03,0x1E}},
{REGFLAG_DELAY, 10, {}},
{0xCC,1,  {0x09}},
{REGFLAG_DELAY, 10, {}},
{0xC6,1,  {0x1D}},
{REGFLAG_DELAY, 10, {}},
{0xC7,4,  {0x00,0xC0,0x00,0xC0}},
{REGFLAG_DELAY, 100, {}},
{0x11,0,{0x00}},
{REGFLAG_DELAY, 150, {}},
{0x29,0,{0x00}},
{REGFLAG_DELAY, 200, {}},
#endif//add by tubao  <<<<==========end
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
	upmu_set_vpa_en(0x0);
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
	upmu_set_vpa_vosel(0x3f);
	upmu_set_vpa_en(0x1);
//	upmu_set_rg_vgp1_vosel(0x7);
//	upmu_set_rg_vgp1_en(0x1);
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

		
		params->dsi.vertical_sync_active				= 8; //4; //6; //(12-4-4); //1;
		params->dsi.vertical_backporch					= 16; //6; //4; //6; //10;
		params->dsi.vertical_frontporch					= 16;//6; //4//6; //10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 8; //40; //44; //(120-40-40); //1;
		params->dsi.horizontal_backporch				= 48;//44; //40; //44; //57;
		params->dsi.horizontal_frontporch				= 48;//44; //40; //44; //32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_div1=1; //0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =22;//31;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		//params->dsi.fbk_sel =0;
		//params->dsi.fbk_div =45;
        params->dsi.PLL_CLOCK = 220;//LCM_DSI_6589_PLL_CLOCK_NULL; //LCM_DSI_6589_PLL_CLOCK_396_5;
    
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
	printk("tubao ===>>> come here  %s  write the register \n",__FUNCTION__);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);


#if 0	
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

LCM_DRIVER KD089D7_40NI_A6_drv = 
{
    .name			= "KD089D7_40NI_A6",
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
