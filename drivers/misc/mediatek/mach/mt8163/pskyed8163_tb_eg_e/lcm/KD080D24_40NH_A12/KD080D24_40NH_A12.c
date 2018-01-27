#ifndef BUILD_LK
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


// ---------------------------------------------------------------------------
// Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH (800)
#define FRAME_HEIGHT (1280)

#define LCM_ID 0x1a94
#define LCM_DSI_CMD_MODE 0

//#define GPIO_BL_EN	   (GPIO118 | 0x80000000)

//#define GPIO_LCM_STBY_2V8	   (GPIO113 | 0x80000000)
//#define GPIO_LCM_RST_2V8     (GPIO112 | 0x80000000)
//#define GPIO_LCM_AVDD_EN	   (GPIO119 | 0x80000000)
//#define GPIO_LCM_CHARGER_EN	   (GPIO19 | 0x80000000)

#define GPIO_LCM_VDD_EN	  GPIO119

//#define GPIO_LCD_RST_EN     GPIO112

#define GPIO_LCD_BL_EN     GPIO118

//#define GPIO_LCD_RST_EN  (GPIO90 | 0x80000000)
#undef GPIO_LCM_RST

#define GPIO_LCM_RST     GPIO90


#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
// Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {
	.set_gpio_out = NULL,
};

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

extern void DSI_clk_HS_mode(unsigned char enter);



// ---------------------------------------------------------------------------
// Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

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


{0xD8, 1, {0x66}}, 
{0x19,18, {0x00, 0x00, 0xFC, 0x51, 0xF2, 0x3F, 0xFF, 0xF3, 0x3F, 0x91, 0x00, 0x00, 0x00, 0xDC, 0xFB, 0x1F, 0x01, 0xC3}},
{0x19,18, {0x10, 0x00, 0xCF, 0x61, 0x90, 0x00, 0xA0, 0x6D, 0x95, 0x42, 0x75, 0x92, 0x95, 0x6D, 0x24, 0x6B, 0xFF, 0x3D}},
{0x19,18, {0x20, 0x00, 0x4B, 0xC6, 0x31, 0xC9, 0xDA, 0x7F, 0xCF, 0x92, 0x1B, 0xD5, 0x49, 0x56, 0xA6, 0x54, 0xA1, 0x50}},
{0x19,18, {0x30, 0x00, 0x20, 0x40, 0x48, 0x11, 0xBA, 0x54, 0xE0, 0x48, 0x86, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x7F, 0x3F}},
{0x19,18, {0x40, 0x00, 0x84, 0x18, 0x40, 0x30, 0x14, 0xC2, 0xFF, 0x37, 0xAC, 0x04, 0x00, 0x00, 0x78, 0x89, 0x03, 0xC0}},
{0x19,18, {0x50, 0x00, 0x00, 0x62, 0x80, 0x00, 0xE2, 0x29, 0xC2, 0x86, 0x14, 0x21, 0xFB, 0x01, 0x3C, 0x00, 0x14, 0x64}},
{0x19,18, {0x60, 0x00, 0x64, 0xA0, 0x10, 0x0A, 0x1E, 0x1C, 0x36, 0x9C, 0x08, 0x01, 0x3A, 0x1E, 0x80, 0x91, 0x02, 0x3C}},
{0x19,18, {0x70, 0x00, 0x86, 0x3F, 0x63, 0x20, 0x00, 0xB8, 0x86, 0x8C, 0x43, 0x18, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE}},
{0x19,18, {0x80, 0x00, 0xFE, 0xFE, 0xFE, 0xFE, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xE8}},
{0x19,18, {0x90, 0x00, 0x0F, 0x09, 0x01, 0x00, 0x14, 0x00, 0x80, 0x5C, 0x11, 0xA0, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40}},
{0x19,18, {0xA0, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0xC8, 0x80, 0x10}},
{0x19,18, {0xB0, 0x00, 0xE0, 0xC1, 0x83, 0x07, 0x0F, 0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
{0x19,18, {0xC0, 0x00, 0x00, 0x80, 0x00, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
{0x19,18, {0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x6E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0x82, 0x14, 0x14}},
{0x19,18, {0xE0, 0x00, 0x82, 0x82, 0x14, 0x14, 0xA5, 0x5A, 0x96, 0x96, 0xA5, 0x5A, 0x96, 0x96, 0xEB, 0xEB, 0x7D, 0x7D}},
{0x19,18, {0xF0, 0x00, 0xEB, 0xEB, 0x7D, 0x7D, 0x00, 0x00, 0x12, 0x84, 0x00, 0x00, 0x48, 0x21, 0x12, 0x84, 0x48, 0x21}},
{0x19,6 , {0xDC, 0x01, 0x21, 0x48, 0x84, 0x12}},
{0x19,18, {0xE0, 0x01, 0x12, 0x84, 0x69, 0x69, 0x48, 0x21, 0xA5, 0x5A, 0x69, 0x69, 0xA5, 0x5A, 0x96, 0x96, 0x5A, 0xA5}},
{0x19,18, {0xF0, 0x01, 0x69, 0x69, 0xDE, 0xB7, 0xA5, 0x5A, 0x7B, 0xED, 0xDE, 0xB7, 0x7B, 0xED, 0xED, 0x7B, 0xB7, 0xDE}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
{0xD8, 1,{0x66}}, 
{0x19, 18, {0x30, 0x00, 0x21, 0x40, 0x48, 0x11, 0xBA, 0x54, 0x80, 0x48, 0x06, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x7F, 0x3F}},
{REGFLAG_DELAY, 50, {}},
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct regulator *lcm_vgp;
static struct regulator *lcm_vgp_back_up;

static struct pinctrl *lcmrst;
static struct pinctrl_state *lcd_rst_high;
static struct pinctrl_state *lcd_rst_low;

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
	if (IS_ERR(lcd_rst_high)) {
		ret = PTR_ERR(lcd_rst_high);
		pr_info("%s : pinctrl err, lcd_rst_high\n", __func__);
	}
	lcd_rst_low = pinctrl_lookup_state(lcmrst, "lcm_rst_low");
	if (IS_ERR(lcd_rst_low)) {
		ret = PTR_ERR(lcd_rst_low);
		pr_info("%s : pinctrl err, lcd_rst_low\n", __func__);
	}

	return ret;
}
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	pr_info("LCM: lcm_get_vgp_supply is going\n");
	

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vgp");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_info("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_info("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

    
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
	pr_err("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}
late_initcall(lcm_init);
module_exit(lcm_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

static int lcd_power_ctl(unsigned char enabled)
{
	int ret=0;
	unsigned int volt,isenable;
	
		pr_info("LCM: lcm_vgp_supply_enable\n");
	
		if (NULL == lcm_vgp || lcm_vgp_back_up==NULL)
			return 0;
	if(enabled)
		{
		pr_info("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
		/* set voltage to 1.8V */
		//ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
		ret = regulator_set_voltage(lcm_vgp, 2800000, 2800000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vgp);
		if (volt == 2800000)
			pr_err("LCM: check regulator voltage=3300000 pass!\n");
		else
			pr_err("LCM: check regulator voltage=3300000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}
		
		ret = regulator_set_voltage(lcm_vgp_back_up, 2800000, 2800000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vgp_back_up);
		if (volt == 2800000)
			pr_err("LCM: check regulator lcm_vgp_back_up voltage=3300000 pass!\n");
		else
			pr_err("LCM: check regulator lcm_vgp_back_up voltage=3300000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vgp_back_up);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}
		}
	else
		{
		     isenable = regulator_is_enabled(lcm_vgp);

			pr_info("LCM: lcm query regulator enable status[0x%d]\n", isenable);

			if (isenable) {
				ret = regulator_disable(lcm_vgp);
				if (ret != 0) {
					pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
					return ret;
				}
				/* verify */
				isenable = regulator_is_enabled(lcm_vgp);
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
		}
	return ret;

}

static void lcd_reset(unsigned char enabled)
{
  if (enabled == 0) {
		pinctrl_select_state(lcmrst, lcd_rst_low);
		pr_info("LCM: lcm set power off\n");
	} else {
		pinctrl_select_state(lcmrst, lcd_rst_high);
		pr_info("LCM: lcm set power on\n");
	} 
}




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
// LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

extern void DSI_clk_HS_mode(unsigned char enter);

static void lcm_get_params(LCM_PARAMS *params)
{

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->dsi.mode   = BURST_VDO_MODE;

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
	params->dsi.packet_size=256;

	// Video mode setting
	params->dsi.intermediat_buffer_num = 2; //because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//params->dsi.word_count=720*3;
	params->dsi.word_count=800*3;


#if 0
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 10;//4; //10;
	params->dsi.vertical_frontporch 				= 10;//3; //10;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 2;
	params->dsi.horizontal_backporch				= 57;//34; //57;
	params->dsi.horizontal_frontporch				= 32;//10; //32;
	params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

#else
		params->dsi.vertical_sync_active				= 5;//2;  //4
	params->dsi.vertical_backporch					= 8;//16;  //4
	params->dsi.vertical_frontporch					= 8;//9;  //8
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active			= 10;//18;  //14
	params->dsi.horizontal_backporch				= 59;//92;  //140
	params->dsi.horizontal_frontporch				= 16;//92; //16
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#endif

	params->dsi.PLL_CLOCK=230; //275;
       params->dsi.cont_clock = 1;
#if 0
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div =11; // 16  8 13  11
#endif
    params->physical_width = 108;
    params->physical_height = 172;
}




static void lcm_init_lcm(void)
{

	lcd_power_ctl(1);
	MDELAY(50);//Must > 5ms
    //lcd_reset(1);
	//MDELAY(50);//Must > 5ms
	lcd_reset(0);
	MDELAY(50);//Must > 5ms
	lcd_reset(1);
    MDELAY(100);


	//init_lcm_registers();



#else
	printk("%s, kernel", __func__);
#endif

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
		MDELAY(180);
//	lcm_set_gpio_output(GPIO_LCD_BL_EN,1);
}




static void lcm_suspend(void)
{
   	printk("%s, kernel", __func__);
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(180);
	lcd_reset(0);
	MDELAY(10);
	lcd_power_ctl(0);
    MDELAY(10);
}



static void lcm_resume(void)
{
    printk("%s, kernel", __func__);
	lcm_init_lcm();
}

static unsigned int lcm_compare_id(void)
{
 
  
 return  1;
   
   
}

static void lcm_init_power(void)
{	
    printk("%s, kernel", __func__);
	MDELAY(20);//Must > 5ms
	lcd_power_ctl(1);
	
	
	//MDELAY(120);		
}
LCM_DRIVER KD080D24_40NH_A12_lcm_drv =
{
	.name = "KD080D24_40NH_A12",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	//.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
	.init_power		= lcm_init_power,
};
