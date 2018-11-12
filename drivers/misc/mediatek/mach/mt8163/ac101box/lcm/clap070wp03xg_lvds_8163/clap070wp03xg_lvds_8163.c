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
#include <mt-plat/charging.h>

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
/*#include "ddp_irq.h"*/

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
static struct pinctrl_state *lcd_pwr_high;
static struct pinctrl_state *lcd_pwr_low;

static int lcm_get_gpio(struct device *dev)
{
	int ret = 0;

	lcmctrl = devm_pinctrl_get(dev);
	if (IS_ERR(lcmctrl)) {
		dev_err(dev, "Cannot find lcm pinctrl!");
		ret = PTR_ERR(lcmctrl);
	}
	/*lcm power pin lookup */
	lcd_pwr_high = pinctrl_lookup_state(lcmctrl, "lcm_pwr_high");
	if (IS_ERR(lcd_pwr_high)) {
		ret = PTR_ERR(lcd_pwr_high);
		printk("%s : pinctrl err, lcd_pwr_high\n", __func__);
	}
	lcd_pwr_low = pinctrl_lookup_state(lcmctrl, "lcm_pwr_low");
	if (IS_ERR(lcd_pwr_low)) {
		ret = PTR_ERR(lcd_pwr_low);
		printk("%s : pinctrl err, lcd_pwr_low\n", __func__);
	}
	return ret;
}

void lcm_set_gpio(int val)
{
	if (val == 0) {
		pinctrl_select_state(lcmctrl, lcd_pwr_low);
		printk("LCM: lcm set power off\n");
	} else {
		pinctrl_select_state(lcmctrl, lcd_pwr_high);
		printk("LCM: lcm set power on\n");
	}
}

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;

	printk("LCM: lcm_get_vgp_supply is going\n");

#if defined(CONFIG_LCM_VDD3V3_SELECT_VIBR)
	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vibr");
#else
	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm-vgp1");
#endif
	if (IS_ERR(lcm_vgp_ldo)) {
		ret = PTR_ERR(lcm_vgp_ldo);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	printk("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo);
	printk("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp = lcm_vgp_ldo;

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	printk("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

	printk("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	ret = regulator_set_voltage(lcm_vgp, 3300000, 3300000);
	if (ret != 0) {
		printk("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp);
	if (volt == 3300000)
		printk("LCM: check regulator voltage=3300000 pass!\n");
	else
		printk("LCM: check regulator voltage=1800000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		printk("LCM: Failed to enable lcm_vgp: %d\n", ret);
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

	printk("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			printk("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			printk("LCM: lcm regulator disable pass\n");
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
		printk("LCM: failed to register disp driver\n");
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
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (1024)//(800)
#define FRAME_HEIGHT (600)//(1280)

#define HSYNC_PULSE_WIDTH 40 
#define HSYNC_BACK_PORCH  160
#define HSYNC_FRONT_PORCH 120
#define VSYNC_PULSE_WIDTH 7
#define VSYNC_BACK_PORCH  18
#define VSYNC_FRONT_PORCH 10


/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))



/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
static void lcm_init_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ONE);
	MDELAY(20);
	upmu_set_rg_vgp1_vosel(3);
	upmu_set_rg_vgp1_en(0x1);

#else
	printk("[Kernel/LCM] lcm_init_power() enter\n");
#endif

}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR, GPIO_OUT_ZERO);
	MDELAY(20);

	upmu_set_rg_vgp1_vosel(0);
	upmu_set_rg_vgp1_en(0);

#else
	printk("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio(0);
	MDELAY(20);

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
	upmu_set_rg_vgp1_vosel(3);
	upmu_set_rg_vgp1_en(0x1);

#else
	printk("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_set_gpio(1);
	MDELAY(20);

	lcm_vgp_supply_enable();
	MDELAY(20);

#endif
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DPI;
	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

    params->dpi.PLL_CLOCK = 67;  //67MHz

	params->dpi.width = FRAME_WIDTH;
	params->dpi.height = FRAME_HEIGHT;

	params->dpi.clk_pol = LCM_POLARITY_FALLING;
	params->dpi.de_pol = LCM_POLARITY_RISING;
	params->dpi.vsync_pol = LCM_POLARITY_FALLING;
	params->dpi.hsync_pol = LCM_POLARITY_FALLING;

	params->dpi.hsync_pulse_width = HSYNC_PULSE_WIDTH;
	params->dpi.hsync_back_porch = HSYNC_BACK_PORCH;
	params->dpi.hsync_front_porch = HSYNC_FRONT_PORCH;
	params->dpi.vsync_pulse_width = VSYNC_PULSE_WIDTH;
	params->dpi.vsync_back_porch = VSYNC_BACK_PORCH;
	params->dpi.vsync_front_porch = VSYNC_FRONT_PORCH;

    params->dpi.lvds_tx_en = 1;
    params->dpi.ssc_disable = 1;
    params->dpi.format            = LCM_DPI_FORMAT_RGB888;   // format is 24 bit
    params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;

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
	printk("[Kernel/LCM] lcm_init() enter\n");
#endif
}

void lcm_suspend(void)
{

#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend() enter\n");

	SET_RESET_PIN(1);
	MDELAY(10);

	SET_RESET_PIN(0);
	MDELAY(10);
#else
	printk("[Kernel/LCM] lcm_suspend() enter\n");
	SET_RESET_PIN(1);
	MDELAY(10);

	SET_RESET_PIN(0);
	MDELAY(10);
#endif
}

void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter\n");

	SET_RESET_PIN(1);
	MDELAY(20);

	SET_RESET_PIN(0);
	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(20);

#else
	printk("[Kernel/LCM] lcm_resume() enter\n");
	SET_RESET_PIN(1);
	MDELAY(20);

	SET_RESET_PIN(0);
	MDELAY(20);

	SET_RESET_PIN(1);
	MDELAY(20);
#endif
}

LCM_DRIVER clap070wp03xg_lvds_8163_lcm_drv = {
	.name = "clap070wp03xg_lvds_8163",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};
