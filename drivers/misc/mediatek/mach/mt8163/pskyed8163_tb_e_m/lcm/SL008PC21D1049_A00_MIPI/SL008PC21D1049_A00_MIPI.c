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
	

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vpa");
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
	params->dsi.vertical_sync_active				= 1;//2;  //4
	params->dsi.vertical_backporch					= 25;//16;  //4
	params->dsi.vertical_frontporch					= 35;//9;  //8
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active			= 1;//18;  //14
	params->dsi.horizontal_backporch				= 60;//92;  //140
	params->dsi.horizontal_frontporch				= 80;//92; //16
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
#endif

	params->dsi.PLL_CLOCK=500;//380; //275;
       params->dsi.cont_clock = 1;
#if 0
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
	params->dsi.fbk_div =11; // 16  8 13  11
#endif
    params->physical_width = 107.64;
    params->physical_height = 172.22;
}


static void init_lcm_registers(void)
{
 	unsigned int data_array[16];

#if 1//add by tubao start
data_array[0]= 0x00073902;data_array[1]= 0x0000b4F0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000C8C;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00001AFD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AA83;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00001184;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000EC0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000012C1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000025C2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000034C3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003FC4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000049C5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000052C6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000059C7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000060C8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C8C9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C7CA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D6CB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D9CC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D7CD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D1CE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D3CF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D4D0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FFD1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000008D2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002ED3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003BD4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AAD5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000B3D6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BCD7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C5D8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D0D9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000DBDA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000E7DB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000F7DC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FEDD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000000DE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EDF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000EE0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000012E1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000025E2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000034E3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003FE4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000049E5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000052E6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000059E7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000060E8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C8E9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C7EA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D6EB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D9EC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D7ED;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D1EE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D3EF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D4F0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FFF1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000008F2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EF3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003BF4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AAF5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000B3F6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BCF7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C5F8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D0F9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000DBFA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000E7FB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000F7FC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FEFD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000000FE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EFF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00004BA9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BB83;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002284;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000EC0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000012C1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000025C2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000034C3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003FC4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000049C5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000052C6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000059C7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000060C8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C8C9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C7CA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D6CB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D9CC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D7CD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D1CE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D3CF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D4D0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FFD1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000008D2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002ED3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003BD4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AAD5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000B3D6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BCD7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C5D8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D0D9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000DBDA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000E7DB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000F7DC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FEDD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000000DE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EDF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000EE0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000012E1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000025E2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000034E3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003FE4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000049E5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000052E6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000059E7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000060E8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C8E9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C7EA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D6EB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D9EC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D7ED;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D1EE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D3EF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D4F0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FFF1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000008F2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EF3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003BF4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AAF5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000B3F6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BCF7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C5F8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D0F9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000DBFA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000E7FB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000F7FC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FEFD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000000FE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EFF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000CC83;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003384;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000EC0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000012C1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000025C2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000034C3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003FC4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000049C5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000052C6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000059C7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000060C8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C8C9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C7CA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D6CB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D9CC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D7CD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D1CE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D3CF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D4D0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FFD1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000008D2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002ED3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003BD4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AAD5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000B3D6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BCD7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C5D8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D0D9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000DBDA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000E7DB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000F7DC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FEDD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000000DE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EDF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00000EE0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000012E1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000025E2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000034E3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003FE4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000049E5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000052E6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000059E7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000060E8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C8E9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C7EA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D6EB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D9EC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D7ED;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D1EE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D3EF;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D4F0;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FFF1;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000008F2;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EF3;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00003BF4;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000AAF5;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000B3F6;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000BCF7;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000C5F8;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000D0F9;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000DBFA;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000E7FB;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000F7FC;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x0000FEFD;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x000000FE;dsi_set_cmdq(data_array, 2, 1);
data_array[0]= 0x00073902;data_array[1]= 0x00002EFF;dsi_set_cmdq(data_array, 2, 1);

data_array[0] = 0x00110500;        //exit sleep mode 
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(200);   
    
data_array[0] = 0x00290500;        //exit sleep mode 
dsi_set_cmdq(data_array, 1, 1); 
MDELAY(120);

#endif



}


static void lcm_init_lcm(void)
{

	lcd_power_ctl(1);
	MDELAY(50);//Must > 5ms
	lcd_reset(1);
	MDELAY(50);//Must > 5ms
	lcd_reset(0);
	MDELAY(50);//Must > 5ms
	lcd_reset(1);
    MDELAY(120);


	init_lcm_registers();



#else
	printk("%s, kernel", __func__);
#endif

//	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
//		MDELAY(180);
//	lcm_set_gpio_output(GPIO_LCD_BL_EN,1);
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

LCM_DRIVER SL008PC21D1049_A00_MIPI_lcm_drv =
{
	.name = "SL008PC21D1049_A00_MIPI",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	//.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
	//.init_power		= lcm_init_power,
};
