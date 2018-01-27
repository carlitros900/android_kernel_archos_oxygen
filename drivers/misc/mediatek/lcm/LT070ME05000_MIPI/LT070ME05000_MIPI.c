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
#define FRAME_WIDTH (1200)
#define FRAME_HEIGHT (1920)

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




//static struct regulator *lcm_vgp;
static struct regulator *lcm_vpa;//add by tubao
static struct regulator *lcm_vgp_back_up;

static struct pinctrl *lcmrst;
static struct pinctrl_state *lcd_rst_high;
static struct pinctrl_state *lcd_rst_low;
static struct pinctrl *lcmbl_ctrl;//add by tubao
static struct pinctrl_state *lcd_bl_high;//add by tubao
static struct pinctrl_state *lcd_bl_low;//add by tubao

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
//add by tubao  ====>>>>> start
	lcmbl_ctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmbl_ctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmbl_ctrl);
	}
	/*lcm power pin lookup */
	lcd_bl_high = pinctrl_lookup_state(lcmbl_ctrl, "lcm_bl_ctrl_high");
	if (IS_ERR(lcd_bl_high)) {
		ret = PTR_ERR(lcd_bl_high);
		pr_info("%s : pinctrl err, lcm_bl_ctrl_low\n", __func__);
	}
	lcd_bl_low = pinctrl_lookup_state(lcmbl_ctrl, "lcm_bl_ctrl_low");
	if (IS_ERR(lcd_bl_low)) {
		ret = PTR_ERR(lcd_bl_low);
		pr_info("%s : pinctrl err, lcm_bl_ctrl_low\n", __func__);
	}
//add by tubao  ====>>>>> end
	return ret;
}
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;
	struct regulator *lcm_vpa_ldo;//add by tubao

	pr_info("LCM: lcm_get_vgp_supply is going\n");
	

	lcm_vpa_ldo = devm_regulator_get(dev, "reg-lcm-vpa");
	if (IS_ERR(lcm_vpa_ldo)) {
		ret = PTR_ERR(lcm_vpa_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_info("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vpa_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vpa_ldo);
	pr_info("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vpa = lcm_vpa_ldo;

    
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
//add by tubao start
int lcm_vpa_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vpa )
		return 0;
//add by tubao  ====>>>>>>>>>>>>>>>control VPA POWER start
	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	//ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	ret = regulator_set_voltage(lcm_vpa, 3200000, 3200000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vpa);
	if (volt == 3200000)
		pr_err("LCM: check regulator voltage=3200000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=3200000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vpa);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vpa: %d\n", ret);
		return ret;
	}
	
//add by tubao  ====>>>>>>>>>>>>>>>control VPA POWER end
	return ret;
}

int lcm_vpa_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vpa)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vpa);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vpa);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vpa);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}
//add by tubao end

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
#if 0
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
		ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vgp);
		if (volt == 3300000)
			pr_err("LCM: check regulator voltage=3300000 pass!\n");
		else
			pr_err("LCM: check regulator voltage=3300000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}
		
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
#endif
#if 1
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

#endif
#if 1
static void lcd_bl_contrl(unsigned char enabled)
{
	if (enabled == 0) {
		pinctrl_select_state(lcmbl_ctrl,lcd_bl_low);
		pr_info("LCM: lcm set power off\n");
	} else {
		pinctrl_select_state(lcmbl_ctrl,lcd_bl_high);
		pr_info("LCM: lcm set power on\n");
	} 
}//add by tubao
#endif
/*static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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

}*/

//add by tubao  ===>>>>>>>>>>>>>> start
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
	{0x01,0,{0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_DELAY, 20, {}},
	{0xB0,1, {0x04}},
	
	{REGFLAG_DELAY, 20, {}},
	{0xB3,5, {0x04, 0x08, 0x00, 0x22, 0x00}},
	
	{REGFLAG_DELAY, 20, {}},
    {0xB4,1, {0x0C}},
	{0xB6,2, {0x3A, 0xD3}},
	{0x51,1,{0xe6}},
	{0x53,1,{0x2c}},
	{0x3a,1,{0x77}},
	
	{REGFLAG_DELAY, 20, {}},
	{0x2a,4,{0x00,0x00,0x04,0xaf}},
	
	{REGFLAG_DELAY, 20, {}},
	{0x2b,4,{0x00,0x00,0x07,0x7f}},
	
	{REGFLAG_DELAY, 20, {}},
	{0x2c,0,{0x00}},
	{0x11,0,{0x00}},
	{REGFLAG_DELAY, 150, {}},
    {0x29,0,{0x00}},
	{REGFLAG_DELAY, 20, {}},
	{0xB3,5,{0x14,0x08,0x00,0x22,0x00}},	
	{REGFLAG_END_OF_TABLE, 0x00, {}},
	{REGFLAG_DELAY, 20, {}},

};
#if 0
#define GPIO_LCM_RST     GPIO90
static void lcd_reset(unsigned char enabled)
{
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
    if (enabled)
    {
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    }
    else
    {
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    }
}
#endif

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
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
//add by tubao  <<<<<<<<<<========end


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
	params->dsi.vertical_sync_active				= 2;//2;  //4
	params->dsi.vertical_backporch					= 6;//16;  //4
	params->dsi.vertical_frontporch					= 8;//9;  //8
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active			= 2;//18;  //14
	params->dsi.horizontal_backporch				= 60;//92;  //140
	params->dsi.horizontal_frontporch				= 115;//92; //16
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#endif

	params->dsi.PLL_CLOCK=500; //275;
       params->dsi.cont_clock = 1;
#if 0
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div =11; // 16  8 13  11
#endif
//   params->physical_width = 108;
//    params->physical_height = 172;
}


static void lcm_init_lcm(void)
{
printk("tubao ==>>> come here  %s  \n",__FUNCTION__);
	lcd_bl_contrl(1);
	lcd_reset(0);
	//lcd_power_en(0);
	MDELAY(20);
	lcm_vpa_supply_enable();
//	upmu_set_vpa_vosel(0x3f);
//	upmu_set_vpa_en(1);
	MDELAY(5);
	upmu_set_rg_vibr_vosel(0x07);//add by tubao
	upmu_set_rg_vibr_en(0x1);//add by tubao enable the VIBR POWER
	MDELAY(30);
	//MDELAY(20);//Must > 5ms
//	lcd_power_en(1);
	MDELAY(8);//Must > 5ms
	lcd_reset(1);
	MDELAY(30);//Must > 5ms
	lcd_reset(0);
	MDELAY(30);//Must > 5ms
	lcd_reset(1);
    MDELAY(20);

//	init_lcm_registers();

#else
	printk("%s, kernel", __func__);
#endif

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
//		MDELAY(180);
//	lcm_set_gpio_output(GPIO_LCD_BL_EN,1);
}




static void lcm_suspend(void)
{
printk("tubao ==>>> come here  %s  \n",__FUNCTION__);
   	printk("%s, kernel", __func__);
	lcd_bl_contrl(0);
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	lcd_reset(0);
	MDELAY(10);
	lcm_vpa_supply_disable();//shutdown VPA power

//	upmu_set_vpa_en(0);
	upmu_set_rg_vibr_en(0x0);//add by tubao enable the VIBR POWER
}



static void lcm_resume(void)
{
printk("tubao ==>>> come here  %s  \n",__FUNCTION__);
    printk("%s, kernel", __func__);
	lcm_init_lcm();
}

static unsigned int lcm_compare_id(void)
{
 return  1;
}

/*static void lcm_init_power(void)
{	
    printk("%s, kernel", __func__);
	MDELAY(10);//Must > 5ms
	lcd_power_ctl(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(0);
	MDELAY(10);//Must > 5ms
	lcd_reset(1);
	MDELAY(120);		
}*/

LCM_DRIVER LT070ME05000_MIPI_lcm_drv =
{
	.name = "LT070ME05000_MIPI_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	//.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
	//.init_power		= lcm_init_power,
};
