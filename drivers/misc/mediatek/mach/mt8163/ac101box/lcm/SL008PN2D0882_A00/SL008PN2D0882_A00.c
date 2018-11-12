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


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};









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
	params->dsi.vertical_sync_active				= 3;//2;  //4
	params->dsi.vertical_backporch					= 13;//16;  //4
	params->dsi.vertical_frontporch					= 9;//9;  //8
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active			= 18;//18;  //14
	params->dsi.horizontal_backporch				= 92;//92;  //140
	params->dsi.horizontal_frontporch				= 92;//92; //16
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


static void init_lcm_registers(void)
{
 	unsigned int data_array[16];
	
	//static int tmp6=0x54;
//	static int tmp7=0x54;

	data_array[0]=0x00043902;
data_array[1]=0x9483FFB9; 
dsi_set_cmdq(data_array, 2, 1); 
MDELAY(10);





data_array[0]=0x00113902;
data_array[1]=0x008213BA; 
data_array[2]=0x1000C516; 
data_array[3]=0x03240FFF; 
data_array[4]=0x20252421; 
data_array[5]=0x00000008; 


dsi_set_cmdq(data_array, 6, 1);
MDELAY(1);




data_array[0]=0x00123902;
data_array[1]=0x040001B1; 
data_array[2]=0xF11203C4; 
data_array[3]=0x3F3F2C24;
data_array[4]=0xE6000257; 
data_array[5]=0x0000A6E2;  
dsi_set_cmdq(data_array, 6, 1);
MDELAY(1); 




data_array[0]=0x00073902;
data_array[1]=0x0EC800B2; 
data_array[2]=0x00110030; 

dsi_set_cmdq(data_array, 3, 1);
MDELAY(1); 



data_array[0]=0x00203902;
data_array[1]=0x320480B4; 
data_array[2]=0x15540810; 
data_array[3]=0x0810220F; 
data_array[4]=0x0A444347; 
data_array[5]=0x0244434B; 
data_array[6]=0x06025555; 
data_array[7]=0x0A5F0644; 
data_array[8]=0x0805706B; 

dsi_set_cmdq(data_array, 9, 1);
MDELAY(1); 







data_array[0]=0x00023902;
data_array[1]=0x000042B6; //vcom  电压
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);









data_array[0]=0x00373902;
data_array[1]=0x000000D5; 
data_array[2]=0x01000A00; 
data_array[3]=0x66220022; 
data_array[4]=0x23010111; 
data_array[5]=0xBC9A6745; 
data_array[6]=0x8845BBAA; 
data_array[7]=0x88888888; 
data_array[8]=0x88678888; 
data_array[9]=0x29810888;
data_array[10]=0x48088883; 
data_array[11]=0x68288581; 
data_array[12]=0x48888783; 
data_array[13]=0x00000085; 
data_array[14]=0x00013C00; 
dsi_set_cmdq(data_array, 15, 1);
MDELAY(1); 









/*data_array[0]=0x00373902;
data_array[1]=0x000000D5; 
data_array[2]=0x01000A00; 
data_array[3]=0x00220022; 
data_array[4]=0xA9CB5477; 
data_array[5]=0x10325476; 
data_array[6]=0x8811BBAA; 
data_array[7]=0x88888888; 
data_array[8]=0x88768888; 
data_array[9]=0xA9CB5488;
data_array[10]=0x10325476; 
data_array[11]=0x8811BBAA; 
data_array[12]=0x88888888; 
data_array[13]=0x88768888; 
data_array[14]=0x00013C88; 
dsi_set_cmdq(data_array, 15, 1);
MDELAY(1); */







data_array[0]=0x00023902;
data_array[1]=0x000009CC;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);






data_array[0]=0x00053902;
data_array[1]=0x100206BF;
data_array[2]=0x00000004;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);






data_array[0]=0x00053902;
data_array[1]=0x001000C7;
data_array[2]=0x00000010;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);

data_array[0]=0x002B3902;
data_array[1]=0x13120eE0;
 
data_array[2]=0x1e2f2531;
 
data_array[3]=0x0e0c0639;
 
data_array[4]=0x14131513;
 
data_array[5]=0x120e1811; 

data_array[6]=0x2f253113;
 
data_array[7]=0x0c06391e; 

data_array[8]=0x1315130e;
 
data_array[9]=0x0b181114;

data_array[10]=0x0b120717; 
data_array[11]=0x00120717;   
dsi_set_cmdq(data_array, 12, 1);
MDELAY(1); 

data_array[0]=0x00033902;
data_array[1]=0x00170CC0;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00033902;
data_array[1]=0x000808C6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
data_array[0]=0x00023902;
data_array[1]=0x000032D4;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);
	data_array[0] = 0x00110500; 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(100);
	
	data_array[0] = 0x00290500; 
	dsi_set_cmdq(data_array, 1, 1); 
	MDELAY(100);

}


static void lcm_init_lcm(void)
{

	lcd_power_ctl(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(0);
	MDELAY(10);//Must > 5ms
	lcd_reset(1);
    MDELAY(120);


	init_lcm_registers();



#else
	printk("%s, kernel", __func__);
#endif

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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

LCM_DRIVER SL008PN2D0882_A00_DSI_MIPI_lcm_drv = 
{
	.name = "SL008PN2D0882_A00",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	//.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	//.compare_id    = lcm_compare_id,
	//.init_power		= lcm_init_power,
};
