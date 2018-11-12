#ifndef BUILD_LK
#include <linux/string.h>
#endif
#ifdef BUILD_LK
#include <platform/mt_reg_base.h>
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#include <debug.h>
#include <platform/upmu_common.h>

#elif (defined BUILD_UBOOT)
#include <asm/arch/mt6577_gpio.h>
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

// ---------------------------------------------------------------------------
// Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH (1024)
#define FRAME_HEIGHT (600)


#if defined(MTK_LCM_RGB_800X480_SUPPORT)
#undef	FRAME_WIDTH
#undef	FRAME_HEIGHT
#define FRAME_WIDTH (800)
#define FRAME_HEIGHT (480)
#endif

#define GPIO_WR32(addr, data)   DRV_WriteReg16(addr,data)
#define GPIO_RD32(addr)         DRV_Reg16(addr)



//Jitao modify begin
#define HSYNC_PULSE_WIDTH 20 //20
#define HSYNC_BACK_PORCH 140 //140
#define HSYNC_FRONT_PORCH 160 //160
#define VSYNC_PULSE_WIDTH 3 //  3
#define VSYNC_BACK_PORCH 20 //20
//#if defined(CONFIG_RGB_T8370)
#define VSYNC_FRONT_PORCH 4 // 12
#if 1
extern void mt_gpio_opt_mode(int gpio,int mode);
extern void mt_gpio_opt_dir(int gpio,int dir);
extern void mt_gpio_opt_output(int gpio,int val);
#endif
#ifdef BUILD_LK
//#define GPIO_LCD_RST_EN GPIO90
//#define GPIO_LCD_STB_EN GPIO89


#ifdef GPIO_LCM_PWR
#define GPIO_LCD_PWR GPIO_LCM_PWR
#else
#define GPIO_LCD_PWR 0xFFFFFFFF
#endif

#ifdef GPIO_LCM_PWR_EN
#define GPIO_LCD_PWR_EN GPIO_LCM_PWR_EN
#else
#define GPIO_LCD_PWR_EN 0xFFFFFFFF
#endif

#ifdef GPIO_LCM_PWR2_EN
#define GPIO_LCD_PWR2_EN GPIO_LCM_PWR2_EN
#else
#define GPIO_LCD_PWR2_EN 0xFFFFFFFF
#endif


#ifdef GPIO_LCM_RST//usr as bl enable
#define GPIO_LCD_RST_EN GPIO_LCM_RST
#else
#define GPIO_LCD_RST_EN 0xFFFFFFFF
#endif

#ifdef GPIO_LCM_STB
#define GPIO_LCD_STB_EN GPIO_LCM_STB
#else
#define GPIO_LCD_STB_EN 0xFFFFFFFF
#endif


#ifdef GPIO_LCM_LVL_SHIFT_EN
#define GPIO_SHIFT_EN GPIO_LCM_LVL_SHIFT_EN
#else
#define GPIO_SHIFT_EN 0xFFFFFFFF
#endif

#ifdef GPIO_LCM_BL_EN
#define GPIO_LCD_BL_EN GPIO_LCM_BL_EN
#else
#define GPIO_LCD_BL_EN 0xFFFFFFFF
#endif


#ifdef GPIO_LCM_BRIDGE_EN
#define GPIO_LCD_BRIDGE_EN GPIO_LCM_BRIDGE_EN
#else
#define GPIO_LCD_BRIDGE_EN 0xFFFFFFFF
#endif

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{

if(GPIO == 0xFFFFFFFF)
{

printf("[LK/LCM] GPIO_LCD_PWR = 0x%x \n",GPIO_LCD_PWR);
printf("[LK/LCM] GPIO_LCD_PWR_EN = 0x%x\n",GPIO_LCD_PWR_EN);
printf("[LK/LCM] GPIO_LCD_PWR2_EN = 0x%x \n",GPIO_LCD_PWR2_EN);
printf("[LK/LCM] GPIO_LCD_RST_EN = 0x%x \n",GPIO_LCD_RST_EN);
printf("[LK/LCM] GPIO_LCD_STB_EN = 0x%x\n",GPIO_LCD_STB_EN);
printf("[LK/LCM] GPIO_SHIFT_EN = 0x%x\n",GPIO_SHIFT_EN);
printf("[LK/LCM] GPIO_LCD_BL_EN = 0x%x\n",GPIO_LCD_BL_EN);
printf("[LK/LCM] GPIO_LCD_BRIDGE_EN = 0x%x \n",GPIO_LCD_BRIDGE_EN);

return;
}

mt_set_gpio_mode(GPIO, GPIO_MODE_00);
mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
mt_set_gpio_out(GPIO, (output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}

#else

/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static struct pinctrl *lcmctrl;
static struct pinctrl_state *lcd_pwr_high;
static struct pinctrl_state *lcd_pwr_low;
static struct pinctrl_state *rgb_pins_default;


static int lcm_get_gpio(struct device *dev)
{
	int ret = 0;

	lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmctrl);
	}
	/*lcm power pin lookup */
	rgb_pins_default = pinctrl_lookup_state(lcmctrl, "default");
	if (IS_ERR(rgb_pins_default)) {
		ret = PTR_ERR(rgb_pins_default);
		pr_debug("%s : pinctrl err, lcd_pwr_high\n", __func__);
	}
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
	return ret;
}

void lcm_set_gpio(int val)
{
	if (val == 0) {
		pinctrl_select_state(lcmctrl, lcd_pwr_low);
		pr_debug("LCM: lcm set power off\n");
	} else {
		pinctrl_select_state(lcmctrl, lcd_pwr_high);
		pr_debug("LCM: lcm set power on\n");
	}
}

void lcm_set_rgb_driving(void)
{
	pinctrl_select_state(lcmctrl, rgb_pins_default);
	pr_debug("LCM: lcm set rgb driving current\n");
}

extern int get_adc_value(int channel);

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

        ret = get_adc_value(4);
	pr_err("LCM: lcm_get_vgp_supply is going ===>adc: %d\n", ret);

       
//reg-lcm-ext
	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_debug("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	pr_debug("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_debug("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	pr_debug("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
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

	return ret;
}

int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

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
	pr_notice("LCM: Register lcm driver\n");
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

#endif



// ---------------------------------------------------------------------------
// Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};


//#define UDELAY(n) 
//#define MDELAY(n) 
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
// Local Functions
// ---------------------------------------------------------------------------

static __inline void send_ctrl_cmd(unsigned int cmd)
{

}

static __inline void send_data_cmd(unsigned int data)
{

}

static __inline void set_lcm_register(unsigned int regIndex,
unsigned int regData)
{

}



static void lcm_init_power(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_init_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(20);
	upmu_set_rg_vibr_vosel(0x5);
	upmu_set_rg_vibr_en(0x1);
		
#else
	pr_debug("[Kernel/LCM] lcm_init_power() enter\n");
#endif

}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
	MDELAY(20);
	
	upmu_set_rg_vibr_vosel(0);
	upmu_set_rg_vibr_en(0);	
			
#else
	printk("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio(0);
	MDELAY(20);
	
	lcm_vgp_supply_disable();
	MDELAY(20);
	#if 1
	mt_gpio_opt_mode(16,0);
    mt_gpio_opt_dir(16,0);
	mt_gpio_opt_output(16,0);
	mt_gpio_opt_mode(22,0);
    mt_gpio_opt_dir(22,0);
	mt_gpio_opt_output(22,0);
	mt_gpio_opt_mode(17,0);
    mt_gpio_opt_dir(17,0);
	mt_gpio_opt_output(17,0);
	mt_gpio_opt_mode(19,0);
    mt_gpio_opt_dir(19,0);
	mt_gpio_opt_output(19,0);
	mt_gpio_opt_mode(20,0);
    mt_gpio_opt_dir(20,0);
	mt_gpio_opt_output(20,0);
	#endif
#endif
}

static void lcm_resume_power(void)
{
    int ret = 0;
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(20);
	upmu_set_rg_vibr_vosel(0x5);
	upmu_set_rg_vibr_en(0x1);
				
#else
	printk("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio(1);
	MDELAY(20);
	
	ret = lcm_vgp_supply_enable();
	MDELAY(100);
#if 1

	mt_gpio_opt_mode(16,2);
	mt_gpio_opt_mode(22,2);
	mt_gpio_opt_mode(17,2);
    mt_gpio_opt_mode(20,2);
    mt_gpio_opt_mode(19,2);
#endif
     
#endif
}

// ---------------------------------------------------------------------------
// LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DPI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if defined(CONFIG_RGB_T8370)
    params->dpi.PLL_CLOCK = 51.2;//67;  //67MHz
#else
    params->dpi.PLL_CLOCK = 54;  //67MHz
#endif

    /* RGB interface configurations */
    //params->dpi.mipi_pll_clk_ref  = 0;
    //params->dpi.mipi_pll_clk_div1 = 0x80000101;  //lvds pll 65M
    //params->dpi.mipi_pll_clk_div2 = 0x800a0000;
    //params->dpi.dpi_clk_div       = 2;          
    //params->dpi.dpi_clk_duty      = 1;
	params->dpi.width = FRAME_WIDTH;
	params->dpi.height = FRAME_HEIGHT;

    params->dpi.clk_pol           = LCM_POLARITY_RISING;
    params->dpi.de_pol            = LCM_POLARITY_RISING;
    params->dpi.vsync_pol         = LCM_POLARITY_FALLING;
    params->dpi.hsync_pol         = LCM_POLARITY_FALLING;

    params->dpi.hsync_pulse_width = HSYNC_PULSE_WIDTH;
    params->dpi.hsync_back_porch  = HSYNC_BACK_PORCH;
    params->dpi.hsync_front_porch = HSYNC_FRONT_PORCH;
    params->dpi.vsync_pulse_width = VSYNC_PULSE_WIDTH;
    params->dpi.vsync_back_porch  = VSYNC_BACK_PORCH;
    params->dpi.vsync_front_porch = VSYNC_FRONT_PORCH;

    params->dpi.lvds_tx_en = 0;
#if defined(CONFIG_RGB_T8370)
    params->dpi.ssc_disable = 1;
#else
    params->dpi.ssc_disable = 0;
	  params->dsi.ssc_range = 4;
#endif
    params->dpi.format            = LCM_DPI_FORMAT_RGB666;   // format is 24 bit
    params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;
#ifdef BUILD_LK
    params->dpi.io_driving_current         = LCM_DRIVING_CURRENT_8MA;
#endif

}


static void lcm_init_lcm(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init() enter\n");

	SET_RESET_PIN(1);
	MDELAY(20);

	SET_RESET_PIN(0);
	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(20);
#else
	pr_err("[Kernel/LCM] lcm_init() enter\n");
#endif
}


static void lcm_suspend(void)
{
#ifdef BUILD_LK
printf("[LK/LCM] lcm_suspend() enter\n");

//lcm_set_gpio_output(GPIO_LCD_BL_EN, 0);
//MDELAY(200);

//lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_ZERO);
//MDELAY(20);
//lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_ONE);
//MDELAY(5);
lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_ZERO);
MDELAY(20);

lcm_set_gpio_output(GPIO_LCD_STB_EN,GPIO_OUT_ZERO);
MDELAY(20);

lcm_set_gpio_output(GPIO_LCD_PWR, 0);
lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
lcm_set_gpio_output(GPIO_LCD_PWR2_EN, 0);


MDELAY(20); 

#elif (defined BUILD_UBOOT)
// do nothing in uboot
#else
printk("[LCM] lcm_suspend() enter\n");
/*
#if defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
    int status=0;
	printk(KERN_INFO ">>>>>>>>>>>>>>>>%s %d START>>>>>>>>>>>>\n",__func__, __LINE__);
	
	chr_control_interface(CHARGING_CMD_SET_CHR_SET,&status);
	printk(KERN_INFO ">>>>>>>>>>>>>>>>%s %d END>>>>>>>>>>>>\n",__func__, __LINE__);
}

#endif 
*/

#endif

}


static void lcm_resume(void)
{
#ifdef BUILD_LK
printf("[LK/LCM] lcm_resume() enter\n");
//VGP6 3.3V

#ifdef MTK_PMIC_MT6397
upmu_set_rg_vgp6_vosel(0x7);
upmu_set_rg_vgp6_sw_en(0x1);
#else
upmu_set_rg_vibr_vosel(0x5);
upmu_set_rg_vibr_en(0x1);
#endif
//pmic_config_interface(0x424, 0x1, 0x1, 15); 
//pmic_config_interface(0x45a, 0x07, 0x07, 5);

// hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_3300, "LCM");

lcm_set_gpio_output(GPIO_LCD_STB_EN,GPIO_OUT_ONE);
MDELAY(20); 

#if 1
lcm_set_gpio_output(GPIO_LCD_PWR, 1);
lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
lcm_set_gpio_output(GPIO_LCD_PWR2_EN, 1);

//MDELAY(20);
#endif

//lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_ONE);
MDELAY(20);
//lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_ZERO);
//MDELAY(5);
lcm_set_gpio_output(GPIO_LCD_RST_EN,GPIO_OUT_ONE);
MDELAY(200);

//lcm_set_gpio_output(GPIO_LCD_BL_EN, 1);

#elif (defined BUILD_UBOOT)
// do nothing in uboot
#else
printk("[LCM] lcm_resume() enter\n");
//lcm_set_rgb_driving();
/*
#if defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
	printk(KERN_INFO ">>>>>>>>>>>>>>>>%s %d START>>>>>>>>>>>>\n",__func__, __LINE__);
	int status=1;
	chr_control_interface(CHARGING_CMD_SET_CHR_SET,&status);
	printk(KERN_INFO ">>>>>>>>>>>>>>>>%s %d END>>>>>>>>>>>>\n",__func__, __LINE__);
}

#endif
*/
	//MDELAY(20);
#endif

lcm_init_lcm();


}

LCM_DRIVER lxjc070whm270_18a_8370_rgb_lcm_drv = 
{
.name = "lxjc070whm270_18a_8370_rgb",
.set_util_funcs = lcm_set_util_funcs,
.get_params = lcm_get_params,
.init = lcm_init_lcm,
.suspend = lcm_suspend,
.resume = lcm_resume,
	.init_power		= lcm_init_power,
	.resume_power 	= lcm_resume_power,
	.suspend_power 	= lcm_suspend_power,
};
