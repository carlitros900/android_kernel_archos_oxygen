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


#define LCM_DSI_CMD_MODE 0

#define GPIO_LCM_VDD_EN	  GPIO119

//#define GPIO_LCD_RST_EN     GPIO112

#define GPIO_LCD_BL_EN     GPIO118

//#define GPIO_LCD_RST_EN  (GPIO90 | 0x80000000)
#undef GPIO_LCM_RST

#define GPIO_LCM_RST     GPIO90


#define REGFLAG_DELAY             							0XfFE
#define REGFLAG_END_OF_TABLE      							0xfFF   // END OF REGISTERS MARKER
#undef LCM_ID
#define LCM_ID   0x61
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
	
	{REGFLAG_DELAY, 16, {}},
	
  
 
// GIP Timing Setting
	{0xFF,3,{0x61,0x36,0x07}},  
	{0x03,1,{0x20}},  
	{0x04,1,{0x04}},  
	{0x05,1,{0x00}}, 
	{0x06,1,{0x00}},  
	{0x07,1,{0x00}},  
	{0x08,1,{0x00}}, 
	{0x09,1,{0x00}},
	{0x0A,1,{0x01}},  
	{0x0B,1,{0x01}},
        {0x0C,1,{0x01}},
        {0x0D,1,{0x1E}},
        {0x0E,1,{0x01}},
        {0x0F,1,{0x01}},

{0x10,1,{0x40}},
{0x11,1,{0x02}},
{0x12,1,{0x05}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0x00}},
	{0x16,1,{0x01}},  
	{0x17,1,{0x01}},  
	{0x18,1,{0x00}}, 
	{0x19,1,{0x00}},  
	{0x1A,1,{0x00}},  
	{0x1B,1,{0xC0}},
{0x1C,1,{0xBB}},
{0x1D,1,{0x0B}},
{0x1E,1,{0x00}},
{0x1F,1,{0x00}},
{0x20,1,{0x00}},
{0x21,1,{0x00}},
{0x22,1,{0x00}},
{0x23,1,{0xC0}},
{0x24,1,{0x30}},
{0x25,1,{0x00}},
{0x26,1,{0x00}},
{0x27,1,{0x03}},

	{0x30,1,{0x01}},  
	{0x31,1,{0x23}},  
	{0x32,1,{0x55}}, 
	{0x33,1,{0x67}},  
	{0x34,1,{0x89}},  
	{0x35,1,{0xAB}}, 
	{0x36,1,{0x01}},  
	{0x37,1,{0x23}},  
	{0x38,1,{0x45}}, 
	{0x39,1,{0x67}},  
	{0x3A,1,{0x44}}, 
	{0x3B,1,{0x55}},  
	{0x3C,1,{0x66}},  
	{0x3D,1,{0x77}}, 

	{0x50,1,{0x00}},  
	{0x51,1,{0x0D}}, 
	{0x52,1,{0x0D}},  
	{0x53,1,{0x0C}},  
	{0x54,1,{0x0C}}, 
	{0x55,1,{0x0F}},  
	{0x56,1,{0x0F}},  
	{0x57,1,{0x0E}}, 
	{0x58,1,{0x0E}},  
	{0x59,1,{0x06}},  
	{0x5A,1,{0x07}}, 
	{0x5B,1,{0x1F}},  
	{0x5C,1,{0x1F}},  
	{0x5D,1,{0x1F}}, 
	{0x5E,1,{0x1F}},  
	{0x5F,1,{0x1F}},
	{0x60,1,{0x1F}},
	{0x67,1,{0x06}},
	{0x68,1,{0x13}},
	{0x69,1,{0x0F}},
	{0x6A,1,{0x12}},
 	{0x6B,1,{0x0E}},
	{0x6C,1,{0x11}},
	{0x6D,1,{0x0D}},
	{0x6E,1,{0x10}},
	{0x6F,1,{0x0C}},

	{0x70,1,{0x14}},
	{0x71,1,{0x15}},
	{0x72,1,{0x08}},
	{0x73,1,{0x01}},
	{0x74,1,{0x00}},
	{0x75,1,{0x02}},
	{0x76,1,{0x02}},
	{0x83,1,{0x01}},

// Page8 for Blanking OSC*2 
	{0xFF,3,{0x61,0x36,0x08}},  
	{0x1C,1,{0xA0}}, 

// R4Ch_00h: Enable ; R4Ch_08h : Disable
	{0xFF,3,{0x61,0x36,0x08}},  
	{0x4C,1,{0x00}},  
	{0x78,1,{0x04}},  
	{0x8E,1,{0x12}}, 
	{0x76,1,{0xB4}}, 
	{0x93,1,{0x02}}, 

// Page1 for Vcom correction
	{0xFF,3,{0x61,0x36,0x01}},  
//VCOM1 UD=L
	{0x31,1,{0x19}},  
//VCOM2 UD=H
	{0x50,1,{0xA9}},  
// VREG1OUT=4.67V
	{0x51,1,{0xA4}},  
// VREG2OUT=-4.6V       	    	
	{0x53,1,{0x8D}},  
	
// Page1 for POS gamma correction
	{0xFF,3,{0x61,0x36,0x01}},  
	{0xA0,1,{0x0E}},  
	{0xA1,1,{0x14}},  
	{0xA2,1,{0x1B}},  
	{0xA3,1,{0x24}},  
	{0xA4,1,{0x1C}},  
	{0xA5,1,{0x22}},  
	{0xA6,1,{0x25}},  
	{0xA7,1,{0x24}},  
	{0xA8,1,{0x29}},  
	{0xA9,1,{0x2D}},  
	{0xAA,1,{0x32}},  
	{0xAB,1,{0x3A}},  
	{0xAC,1,{0x39}},  
	{0xAD,1,{0x30}},  
	{0xAE,1,{0x2C}},  
	{0xAF,1,{0x35}},  
	{0xB0,1,{0x33}},  

// Page1 for NEG gamma correction
	{0xFF,3,{0x61,0x36,0x01}},  
	{0xC0,1,{0x0C}},  
	{0xC1,1,{0x15}},  
	{0xC2,1,{0x1D}},  
	{0xC3,1,{0x24}},  
	{0xC4,1,{0x1B}},  
	{0xC5,1,{0x1F}},  
	{0xC6,1,{0x22}},  
	{0xC7,1,{0x21}},  
	{0xC8,1,{0x27}},  
	{0xC9,1,{0x2C}},  
	{0xCA,1,{0x31}},  
	{0xCB,1,{0x38}},  
	{0xCC,1,{0x38}},  
	{0xCD,1,{0x30}},  
	{0xCE,1,{0x2D}},  
	{0xCF,1,{0x31}},  
	{0xD0,1,{0x34}},  

	{0xFF,3,{0x61,0x36,0x08}},  
	{0xAB,1,{0x24}},  

	{0xFF,3,{0x61,0x36,0x06}},  
	{0x72,1,{0x01}},  
//Sleep out
	{0xFF,3,{0x61,0x36,0x00}}, 
	{0x11,1,{0x00}},   
	{REGFLAG_DELAY, 150, {}},
//Display on    
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 16, {}},
        
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct regulator *lcm_vgp;
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
	
		if (NULL == lcm_vgp)
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
	params->dsi.vertical_sync_active				= 2,//3;//2;  //4
	params->dsi.vertical_backporch					= 6,//13;//16;  //4
	params->dsi.vertical_frontporch					= 4,//9;//9;  //8
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active			= 4,//18;//18;  //14
	params->dsi.horizontal_backporch				= 140,//92;//92;  //140
	params->dsi.horizontal_frontporch				= 140,//92;//92; //16
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#endif
	
	params->dsi.PLL_CLOCK=240; //275;
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
	MDELAY(10);//Must > 5ms
	lcd_power_ctl(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(0);
	MDELAY(10);//Must > 5ms
	lcd_reset(1);
    MDELAY(120);

	//init_lcm_registers();
	 push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
	MDELAY(180);
	
}




static void lcm_suspend(void)
{
   	printk("%s, kernel", __func__);
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
    unsigned char buffer[3];
    unsigned int data_array[16];
	printk("%s, kernel", __func__);

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);	

    data_array[0] = 0x00043902;// read id return two byte,version and id
    data_array[1] = 0x013661ff;// read id return two byte,version and id
    dsi_set_cmdq(data_array, 2, 1);
	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x00, buffer, 1);

   printk("id=%x\n", buffer[0]);

   return (LCM_ID == buffer[0])? 1: 0;
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

LCM_DRIVER M080WXBI40_CA_01_lcm_drv = 
{
	.name = "M080WXBI40_CA_01",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	//.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
	//.init_power		= lcm_init_power,
};
