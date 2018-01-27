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
static struct pinctrl *lcmrst;//add by tubao

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
	if (IS_ERR(lcd_rst_high)) {
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

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vgp");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm-vgp LDO, %d¥n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.¥n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage¥n", ret);

	lcm_vgp = lcm_vgp_ldo;


	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	//add by tubao  for contrl vibr power start start
	ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
		if (ret != 0) {
			pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
			return ret;
		}
	
		/* get voltage settings again */
		volt = regulator_get_voltage(lcm_vgp);
		if (volt == 3300000)
			pr_err("LCM: check regulator lcm_vgp voltage=3300000 pass!\n");
		else
			pr_err("LCM: check regulator lcm_vgp voltage=3300000 fail! (voltage: %d)\n", volt);
	
		ret = regulator_enable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
			return ret;
		}

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
	MDELAY(10);
	lcm_vgp_supply_disable();


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

	lcm_vgp_supply_enable();
	MDELAY(10);
	lcm_set_gpio(1);
	MDELAY(30);
	lcm_set_gpio(0);
	MDELAY(30);
	lcm_set_gpio(1);
	MDELAY(100);

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

	// enable tearing-free
	//params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	//params->dsi.mode	 = SYNC_PULSE_VDO_MODE;
	params->dsi.mode    = BURST_VDO_MODE;
	//params->dsi.mode	 = SYNC_EVENT_VDO_MODE; 

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	//params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	//params->dsi.data_format.trans_seq	 = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	//params->dsi.data_format.padding 	 = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	//params->dsi.packet_size=256;

	// Video mode setting		
	//params->dsi.intermediat_buffer_num = 2;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//params->dsi.word_count=800*3;	


	params->dsi.vertical_sync_active = 3;//4; //4;
	params->dsi.vertical_backporch =  4;//4; //14;  //16;
	params->dsi.vertical_frontporch = 8;//8; //16; //15;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8; //4;
	params->dsi.horizontal_backporch = 8; //6;
	params->dsi.horizontal_frontporch = 32; //16; //44; //60;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;


	params->dsi.PLL_CLOCK=200;//180; //
	
#if defined(BUILD_LK)
	printf("[LCD] PLL_CLOCK=%d\n",params->dsi.PLL_CLOCK);
#else
    printk("[LCD] PLL_CLOCK=%d\n",params->dsi.PLL_CLOCK);
#endif
    	params->dsi.cont_clock = 1;
    	//params->dsi.horizontal_bllp = 0x1F4; //word count=0x340

}
//extern void DSI_clk_HS_mode(unsigned char enter);
static void lcm_init_lcm(void)
{
	//unsigned int data_array[16];
	unsigned int data_array[16];

	data_array[0]=0x00043902;
	data_array[1]=0x083661FF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000A01C;
	dsi_set_cmdq(data_array, 2, 1);
	/////
	data_array[0]=0x00043902;
	data_array[1]=0x013661FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x00000056;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x00009053;
	//data_array[1]=0x0000A653;
	dsi_set_cmdq(data_array, 2, 1);
	//
	data_array[0]=0x00043902;
	data_array[1]=0x083661FF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000004C;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	//data_array[1]=0x0000046C;
	data_array[1]=0x0000026C;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000893;
	dsi_set_cmdq(data_array, 2, 1);
	//
	data_array[0]=0x00043902;
	data_array[1]=0x073661FF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000051A;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001F16;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000050D;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000030A;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x0000350E;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001F0B;
	dsi_set_cmdq(data_array, 2, 1);
	//
	data_array[0]=0x00043902;
	data_array[1]=0x013661FF;
	dsi_set_cmdq(data_array, 2, 1);
	#if 0 //xingkun
	data_array[0]=0x00023902;
	data_array[1]=0x00000CA0;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000FA1;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000015A2;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001CA3;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000DA4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001BA5;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000020A6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000022A7;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00002AA8;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000031A9;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000036AA;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003DAB;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003CAC;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000032AD;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000033AE;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000032AF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000035B0;
	dsi_set_cmdq(data_array, 2, 1);
	#else
	data_array[0]=0x00023902;
	data_array[1]=0x00000bA0;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000cA1;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000eA2;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000016A3;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000DA4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000012A5;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000017A6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000019A7;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000024A8;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00002dA9;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000033AA;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003cAB;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003CAC;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000032AD;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000033AE;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000032AF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000035B0;
	dsi_set_cmdq(data_array, 2, 1);
	#endif
	//////////
	data_array[0]=0x00043902;
	data_array[1]=0x013661FF;
	dsi_set_cmdq(data_array, 2, 1);
	#if 0//xingkun
	data_array[0]=0x00023902;
	data_array[1]=0x00000CC0;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000FC1;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000015C2;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001CC3;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001DC4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001BC5;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000020C6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000022C7;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00002AC8;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000031C9;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000036CA;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003DCB;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003DCC;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000032CD;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000033CE;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000032CF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000035D0;
	dsi_set_cmdq(data_array, 2, 1);
	#else
	data_array[0]=0x00023902;
	data_array[1]=0x00000CC0;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000FC1;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000012C2;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001aC3;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000010C4;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000014C5;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001bC6;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00001eC7;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000028C8;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00002fC9;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000035CA;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003DCB;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00003cCC;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000034CD;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000034CE;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000033CF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000036D0;
	dsi_set_cmdq(data_array, 2, 1);	
	#endif
	//////
	data_array[0]=0x00043902;
	data_array[1]=0x083661FF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x000024AB;
	dsi_set_cmdq(data_array, 2, 1);
	//xingkun
	data_array[0]=0x00023902;
	data_array[1]=0x00000040;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x00000BE9;
	dsi_set_cmdq(data_array, 2, 1);
	////
	data_array[0]=0x00043902;
	data_array[1]=0x013661FF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	//data_array[1]=0x00001436;
	data_array[1]=0x00000036; //xingkun
	dsi_set_cmdq(data_array, 2, 1);
	////
	data_array[0]=0x00043902;
	data_array[1]=0x063661FF;
	dsi_set_cmdq(data_array, 2, 1);
	data_array[0]=0x00023902;
	data_array[1]=0x00000172;
	dsi_set_cmdq(data_array, 2, 1);
	////
	data_array[0]=0x00043902;
	data_array[1]=0x003661FF;
	dsi_set_cmdq(data_array, 2, 1);


    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);

    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);//5000
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
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);
}
static void lcm_resume(void)
{
	printk("tubao ==>>> come here %s  \n",__FUNCTION__);
	lcm_init_lcm();
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

LCM_DRIVER FY08021DI27A22_CB_MIPI_lcm_drv =
{
    .name			= "FY08021DI27A22_CB_MIPI",
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
