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


#include "lcm_drv.h"

#define  POWER_VGP  		1
#define  POWER_VPA			1
#define  POWER_VIBR			0
#define PX101_DEBUG			0

#ifdef PX101_DEBUG
#define printk_info(fmt,args...)  \
		do {					   \
			printk("[PX101][%s]"fmt,__func__,##args);				   \
			}while(0)
#else
#define printk_info(fmt,args...)
#endif

/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static struct regulator *lcm_vgp_vibr;//add by alex

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


	return ret;
}

void lcm_set_gpio(int val)
{
	if (val == 0) {
		pinctrl_select_state(lcmrst, lcd_rst_low);
		printk_info("LCM: lcm set power off \n");
	} else {
		pinctrl_select_state(lcmrst, lcd_rst_high);
		printk_info("LCM: lcm set power on \n");
	}
}

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	printk_info("LCM: lcm_get_vgp_supply is going \n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) 
	{
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d \n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok. \n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage \n", ret);

	lcm_vgp = lcm_vgp_ldo;
	
//add by tubao  ==============>>>>>>>>>>> for control VPA power start
	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vpa");
		if (IS_ERR(lcm_vgp_ldo)) 
		{
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
		
//add by alex  ==============>>>>>>>>>>>> for control VIBR start
		lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vgp-vibr");
		if (IS_ERR(lcm_vgp_ldo)) 
		{
			ret = PTR_ERR(lcm_vgp_ldo);
			dev_err(dev, "failed to get reg-lcm-vgp-vibr, %d\n", ret);
			return ret;
		}
	
		printk_info("LCM: lcm get supply ok.\n");
	
		ret = regulator_enable(lcm_vgp_ldo);
		/* get current voltage settings */
		ret = regulator_get_voltage(lcm_vgp_ldo);
		pr_info("lcm LDO voltage = %d in LK stage\n", ret);
	
		lcm_vgp_vibr = lcm_vgp_ldo;

//add by tubao  ==============>>>>>>>>>>>> for control VIBR end


#if 1//add by tubao 用于控制 GPIO83  电源
	lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmctrl)) 
	{
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

	pr_debug("LCM: lcm_vgp_supply_enable¥n");

#if POWER_VGP
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

#if POWER_VPA
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
#endif 

#if POWER_VIBR
	//add by tubao  for contrl vibr power start start
	ret = regulator_set_voltage(lcm_vgp_vibr, 1800000, 1800000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vgp_vibr);
		if (volt == 1800000)
			pr_err("LCM: check regulator lcm_vgp_vibr voltage=1800000 pass!\n");
		else
			pr_err("LCM: check regulator lcm_vgp_vibr voltage=1800000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vgp_vibr);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}
	//add by tubao  for contrl vibr power start end
#endif

	pinctrl_select_state(lcmctrl, lcd_pwr_high);
	return 0;//ret;
}

int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;
#if POWER_VGP
	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);
	printk_info("LCM: lcm query regulator enable status[0x%d] \n", isenable);
	if (isenable) 
	{
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) 
		{
			pr_err("LCM: lcm failed to disable lcm_vgp: %d \n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass¥n");
	}
#endif

#if POWER_VPA
//add by tubao  ===>>>>>>> shutdown VPA power start
	isenable = regulator_is_enabled(lcm_vpa_back_up);
	printk_info("LCM: lcm query regulator enable status[0x%d]\n", isenable);
	if (isenable) 
	{
		ret = regulator_disable(lcm_vpa_back_up);
		if (ret != 0)
		{
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vpa_back_up);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}
#endif


#if POWER_VIBR
	//add by tubao   shutdown VIBR power  start
	isenable = regulator_is_enabled(lcm_vgp_vibr);
	pr_info("LCM: lcm query regulator enable status[0x%d]\n", isenable);
	if (isenable) 
	{
		ret = regulator_disable(lcm_vgp_vibr);
		if (ret != 0) 
		{
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp_vibr);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}


	//add by tubao   shutdown VIBR power  end
#endif

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
	pr_notice("LCM: Register lcm driver \n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done \n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");




// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800) 
#define FRAME_HEIGHT (1280) 

#define REGFLAG_DELAY                                       0xFE
#define REGFLAG_END_OF_TABLE                                0xFF   // END OF REGISTERS MARKER



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

#define LCM_DSI_CMD_MODE	0

#define TRUE 1
#define FALSE 0


static void lcm_suspend_power(void)
{
	pr_debug("[Kernel/LCM] lcm_suspend_power() enter\n");
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);
	lcm_set_gpio(0);
	MDELAY(20);

//	upmu_set_rg_vgp1_en(0x0);
	lcm_vgp_supply_disable();
	MDELAY(20);


}
static void lcm_resume_power(void)
{

	pr_debug("[Kernel/LCM] lcm_resume_power() enter\n");
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);

	lcm_set_gpio(1);
	MDELAY(20);

	lcm_vgp_supply_enable();
	MDELAY(20);

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


 {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
 {0x6F,1,{0x01}},  
 {0xB1,1,{0x01}},  
 {0xC8,1,{0x80}},  
 {0xBD,5,{0x01,0xA0,0x0C,0x08,0x01}},  
 {0xB6,1,{0x01}},  
 {0x6F,1,{0x02}},  
 {0xB8,1,{0x0C}},  
 {0xBB,2,{0x11,0x11}},  
 {0xBC,3,{0x05,0x05}},  
 {0xC7,1,{0x01}},  
 {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},  
 {0xB0,2,{0x0A,0x0A}},  
 {0xB1,2,{0x0A,0x0A}},  
 {0xBC,2,{0xA8,0x00}},  
 {0xBD,2,{0xA8,0x00}},  
 {0xCA,1,{0x00}},  
 {0xC0,1,{0x04}},  
 {0xB5,2,{0x03,0x03}},  
 {0xBE,1,{0x30}},  
 {0xB3,2,{0x37,0x37}},  
 {0xB4,2,{0x0F,0x0F}},  
 {0xB8,2,{0x05,0x05}},  
 {0xB9,2,{0x46,0x46}},  
 {0xBA,2,{0x15,0x15}},  
 {0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},  
 {0xEE,1,{0x01}},  
 {0xB0,16,{0x00,0x00,0x00,0x13,0x00,0x34,0x00,0x4F,0x00,0x67,0x00,0x8E,0x00,0xAE,0x00,0xE3}},  
 {0xB1,16,{0x01,0x0C,0x01,0x4E,0x01,0x81,0x01,0xD1,0x02,0x12,0x02,0x14,0x02,0x4E,0x02,0x8E}},  
 {0xB2,16,{0x02,0xB7,0x02,0xED,0x03,0x0F,0x03,0x39,0x03,0x54,0x03,0x74,0x03,0x86,0x03,0x9A}},  
 {0xB3,4,{0x03,0xAC,0x03,0xFF}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x06}},  
 { 0xB0,2 ,{0x2D,0x2E}},  
 { 0xB1, 2,{0x29,0x2A}},  
 { 0xB2, 2,{0x16,0x18}},  
 { 0xB3, 2,{0x10,0x12}},  
 { 0xB4, 2,{0x00,0x31}},  
 { 0xB5, 2,{0x31,0x31}},  
 { 0xB6, 2,{0x31,0x02}},  
 { 0xB7, 2,{0x31,0x31}},  
 { 0xB8, 2,{0x31,0x31}},  
 { 0xB9, 2,{0x31,0x31}},  
 { 0xBA, 2,{0x31,0x31}},  
 { 0xBB, 2,{0x31,0x31}},  
 { 0xBC, 2,{0x31,0x31}},  
 { 0xBD, 2,{0x03,0x31}},  
 { 0xBE, 2,{0x31,0x31}},  
 { 0xBF, 2,{0x31,0x01}},  
 { 0xC0, 2,{0x13,0x11}},  
 { 0xC1, 2,{0x19,0x17}},  
 { 0xC2, 2,{0x2A,0x29}},  
 { 0xC3, 2,{0x2E,0x2D}},  
 { 0xE5, 2,{0x31,0x31}},  
 { 0xC4, 2,{0x2E,0x2D}},  
 { 0xC5, 2,{0x29,0x2A}},  
 { 0xC6, 2,{0x13,0x11}},  
 { 0xC7, 2,{0x19,0x17}},  
 { 0xC8, 2,{0x03,0x31}},  
 { 0xC9, 2,{0x31,0x31}},  
 { 0xCA, 2,{0x31,0x01}},  
 { 0xCB, 2,{0x31,0x31}},  
 { 0xCC, 2,{0x31,0x31}},  
 { 0xCD, 2,{0x31,0x31}},  
 { 0xCE, 2,{0x31,0x31}},  
 { 0xCF, 2,{0x31,0x31}},  
 { 0xD0, 2,{0x31,0x31}},  
 { 0xD1, 2,{0x00,0x31}},  
 { 0xD2, 2,{0x31,0x31}},  
 { 0xD3, 2,{0x31,0x02}},  
 { 0xD4, 2,{0x16,0x18}},  
 { 0xD5, 2,{0x10,0x12}},  
 { 0xD6, 2,{0x2A,0x29}},  
 { 0xD7, 2,{0x2D,0x2E}},
 { 0xE6, 2,{0x31,0x31}},
 { 0xD8, 5,{0x00,0x00,0x00,0x00,0x00}},
 { 0xD9, 5,{0x00,0x00,0x00,0x00,0x00}},
 { 0xE7, 1,{0x00}}, 
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}}, 
 { 0xED, 1,{0x30}}, 
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x03 }}, 
 { 0xB1, 2,{0x20,0x00}},
 { 0xB0, 2,{0x20,0x00}},
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}}, 
 { 0xE5, 1,{0x00}}, 
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}}, 
 { 0xB0, 2,{0x17,0x06}}, 
 { 0xB8, 1,{0x00}}, 
 { 0xBD, 5,{0x03,0x03,0x00,0x03,0x03}}, 
 { 0xB1, 2,{0x17,0x06}},  
 { 0xB9, 2,{0x00,0x03}},  
 { 0xB2, 2,{0x17,0x06}},  
 { 0xBA, 2,{0x00,0x00}},  
 { 0xB3, 2,{0x17,0x06}},  
 { 0xBB, 2,{0x02,0x03}},  
 { 0xB4, 2,{0x17,0x06}},  
 { 0xB5, 2,{0x17,0x06}},  
 { 0xB6, 2,{0x17,0x06}}, 
 { 0xB7, 2,{0x17,0x06}}, 
 { 0xBC, 2,{0x02,0x03}}, 
 { 0xE5, 1,{0x06}},  
 { 0xE6, 1,{0x06}},  
 { 0xE7, 1,{0x00}},  
 { 0xE8, 1,{0x06}},  
 { 0xE9, 1,{0x06}},  
 { 0xEA, 1,{0x06}},  
 { 0xEB, 1,{0x00}},  
 { 0xEC, 1,{0x00}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}}, 
 { 0xC0, 1,{0x0A}},  
 { 0xC1, 1,{0x08}},  
 { 0xC2, 1,{0xA6}},  
 { 0xC3, 1,{0x05}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}},  
 { 0xB2, 5,{0x04,0x00,0x86,0x00,0x00}},  
 { 0xB3, 5,{0x04,0x00,0x86,0x00,0x00}},  
 { 0xB4, 5,{0x04,0x00,0x17,0x00,0x00}},  
 { 0xB5, 5,{0x04,0x00,0x17,0x00,0x00}},
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
 { 0xC4, 1,{0x00}},  
 { 0xC5, 1,{0x02}},  
 { 0xC6, 1,{0x22}},  
 { 0xC7, 1,{0x03}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x03 }},  
 { 0xB6, 5,{0x02,0x00,0x19,0x00,0x00} },  
 { 0xB7, 5,{0x02,0x00,0x19,0x00,0x00} },  
 { 0xB8, 5,{0x02,0x00,0x19,0x00,0x00 }},  
 { 0xB9, 5,{0x02,0x00,0x19,0x00,0x00}},
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
 { 0xC8, 2,{0x06,0x20}},  
 { 0xC9, 2,{0x02,0x20}},  
 { 0xCA, 2,{0x01,0x60}},  
 { 0xCB, 2,{0x01,0x60}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}},  
 { 0xBA, 5,{0x44,0x00,0x86,0x00,0x00}},  
 { 0xBB, 5,{0x44,0x00,0x86,0x00,0x00}},  
 { 0xBC, 5,{0x44,0x00,0x1A,0x00,0x00}},  
 { 0xBD, 5,{0x44,0x00,0x1A,0x00,0x00}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},  
 { 0xD1, 5,{0x00,0x05,0x01,0x07,0x10}},  
 { 0xD2, 5,{0x10,0x05,0x05,0x03,0x10}},  
 { 0xD3, 5,{0x20,0x00,0x43,0x07,0x10}},
 { 0xD4, 5,{0x30,0x00,0x43,0x07,0x10}},
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}}, 
 { 0xD0, 7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
 { 0xD5, 11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 { 0xD6, 11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
 { 0xD7, 11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 
 { 0xD8, 5,{0x00,0x00,0x00,0x00,0x00}},
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x05}},
 { 0xCC, 3,{0x00,0x00,0x3C}},  
 { 0xCD, 3,{0x00,0x00,0x3C}},  
 { 0xCE, 3,{0x00,0x00,0x3C}},  
 { 0xCF, 3,{0x00,0x00,0x3C}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}}, 
 { 0xC0, 4,{0x00,0x34,0x00,0x00}},  
 { 0xC1, 4,{0x00,0x00,0x34,0x00}},  
 { 0xC2, 4,{0x00,0x00,0x34,0x00}},  
 { 0xC3, 4,{0x00,0x00,0x34,0x00}},  
 { 0xF0, 5,{0x55,0xAA,0x52,0x08,0x03}}, 
 { 0xC4, 1,{0x60}},  
 { 0xC5, 1,{0xC0}},  
 { 0xC6, 1,{0x00}},  
 { 0xC7, 1,{0x00}},  



{REGFLAG_DELAY, 10, {}}, 
{0x11, 0, {}},
{REGFLAG_DELAY, 120, {}}, 
{0x29, 0, {}},



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

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
	params->dsi.mode   = BURST_VDO_MODE;
        #endif
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
		//params->dsi.packet_size=256;

	// Video mode setting
		//params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//params->dsi.word_count=768*3;


		
		params->dsi.vertical_sync_active				=1;//5; //4; //6; //(12-4-4); //1;
		params->dsi.vertical_backporch					= 22;//3; //6; //4; //6; //10;
		params->dsi.vertical_frontporch					= 18;//8;//6; //4//6; //10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 1;//5; //40; //44; //(120-40-40); //1;
		params->dsi.horizontal_backporch				= 79;//59;//44; //40; //44; //57;
		params->dsi.horizontal_frontporch				= 80;//16;//44; //40; //44; //32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		// 1 Every lane speed
		//params->dsi.pll_div1=1; //0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =22;//31;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		//params->dsi.fbk_sel =0;
		//params->dsi.fbk_div =45;
        params->dsi.PLL_CLOCK = 290;//380;//LCM_DSI_6589_PLL_CLOCK_NULL; //LCM_DSI_6589_PLL_CLOCK_396_5;
    
		//params->dsi.CLK_ZERO = 262; //47;
		//params->dsi.HS_ZERO = 117; //36;

		params->dsi.cont_clock = 2;
		//params->dsi.noncont_clock = TRUE; 
		//params->dsi.noncont_clock_period = 2; // Unit : frames


}
//extern void DSI_clk_HS_mode(unsigned char enter);

static void lcm_init_lcm(void)
{
	unsigned int data_array[16];

	MDELAY(100);
	printk_info("send cmd \n");
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

	MDELAY(200);
	
	data_array[0] = 0x00110500;        //exit sleep mode 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(120);   

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
	printk_info("start \n");
    	MDELAY(20); 
	return;
}
static void lcm_resume(void)
{
	printk_info("start init  \n");
	lcm_init_lcm();
	return;
	
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

LCM_DRIVER PX101IH28810194A_MIPI_lcm_drv = 
{
    .name			= "PX101IH28810194A_MIPI_lcm_drv",
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
