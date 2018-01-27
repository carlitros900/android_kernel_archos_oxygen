/*
 *
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * mt65xx leds driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/of.h>
/* #include <linux/leds-mt65xx.h> */
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/delay.h>
/* #include <mach/mt_pwm.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>*/

#ifdef CONFIG_MTK_AAL_SUPPORT
#include <ddp_aal.h>
/* #include <linux/aee.h> */
#endif

#include <mt-plat/mt_pwm.h>
#include <mt-plat/upmu_common.h>

#include "leds_sw.h"
#include "leds_hal.h"

/* for LED&Backlight bringup, define the dummy API */
#ifndef CONFIG_MTK_PMIC
u16 pmic_set_register_value(u32 flagname, u32 val)
{
	return 0;
}
#endif

#if 0/*ndef CONFIG_MTK_VIDEOX*/
static int disp_bls_set_backlight(int level_1024)
{
	return 0;
}

static int mtkfb_set_backlight_level(unsigned int level)
{
	return 0;
}
#endif

#ifndef CONFIG_MTK_PWM
s32 pwm_set_spec_config(struct pwm_spec_config *conf)
{
	return 0;
}

void mt_pwm_disable(u32 pwm_no, u8 pmic_pad)
{
}
#endif

static DEFINE_MUTEX(leds_mutex);
static DEFINE_MUTEX(leds_pmic_mutex);

/****************************************************************************
 * variables
 ***************************************************************************/
/* struct cust_mt65xx_led* bl_setting_hal = NULL; */
static unsigned int bl_brightness_hal = 102;
static unsigned int bl_duty_hal = 21;
static unsigned int bl_div_hal = CLK_DIV1;
static unsigned int bl_frequency_hal = 32000;
//for button led don't do ISINK disable first time
static int button_flag_isink0 = 0;
static int button_flag_isink1 = 0;
static int button_flag_isink2 = 0;
static int button_flag_isink3 = 0;
struct wake_lock leds_suspend_lock;

char *leds_name[MT65XX_LED_TYPE_TOTAL] = {
	"red",
	"green",
	"blue",
	"jogball-backlight",
	"keyboard-backlight",
	"button-backlight",
	"lcd-backlight",
};

struct cust_mt65xx_led *pled_dtsi = NULL;
/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/
static int debug_enable_led_hal = 1;
#define LEDS_DEBUG(format, args...) do { \
	if (debug_enable_led_hal) {	\
		pr_debug("[LED]"format, ##args);\
	} \
} while (0)

/****************************************************************************
 * custom APIs
***************************************************************************/
extern unsigned int brightness_mapping(unsigned int level);
/*****************PWM *************************************************/
#define PWM_DIV_NUM 8
static int time_array_hal[PWM_DIV_NUM]={256,512,1024,2048,4096,8192,16384,32768};
static unsigned int div_array_hal[PWM_DIV_NUM] = {1,2,4,8,16,32,64,128}; 
static unsigned int backlight_PWM_div_hal = CLK_DIV1;	/* this para come from cust_leds. */

/******************************************************************************
   for DISP backlight High resolution 
******************************************************************************/
#ifdef LED_INCREASE_LED_LEVEL_MTKPATCH
#define MT_LED_INTERNAL_LEVEL_BIT_CNT 10
#endif

// Use Old Mode of PWM to suppoort 256 backlight level
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256
extern unsigned int Cust_GetBacklightLevelSupport_byPWM(void);

/****************************************************************************
 * func:return global variables
 ***************************************************************************/

void mt_leds_wake_lock_init(void)
{
	wake_lock_init(&leds_suspend_lock, WAKE_LOCK_SUSPEND, "leds wakelock");
}

unsigned int mt_get_bl_brightness(void)
{
	return bl_brightness_hal;
}

unsigned int mt_get_bl_duty(void)
{
	return bl_duty_hal;
}

unsigned int mt_get_bl_div(void)
{
	return bl_div_hal;
}

unsigned int mt_get_bl_frequency(void)
{
	return bl_frequency_hal;
}

unsigned int *mt_get_div_array(void)
{
	return &div_array_hal[0];
}

void mt_set_bl_duty(unsigned int level)
{
	bl_duty_hal = level;
}

void mt_set_bl_div(unsigned int div)
{
	bl_div_hal = div;
}

void mt_set_bl_frequency(unsigned int freq)
{
	bl_frequency_hal = freq;
}

struct cust_mt65xx_led *get_cust_led_dtsi(void)
{
	struct device_node *led_node = NULL;
	bool isSupportDTS = false;
	int i, ret;
	int mode, data;
	int pwm_config[5] = { 0 };

	LEDS_DEBUG("get_cust_led_dtsi: get the leds info from device tree\n");
	if (pled_dtsi == NULL) {
		/* this can allocat an new struct array */
		pled_dtsi = kmalloc(MT65XX_LED_TYPE_TOTAL *
						      sizeof(struct
							     cust_mt65xx_led),
						      GFP_KERNEL);
		if (pled_dtsi == NULL) {
			LEDS_DEBUG("get_cust_led_dtsi kmalloc fail\n");
			goto out;
		}

		for (i = 0; i < MT65XX_LED_TYPE_TOTAL; i++) {

			char node_name[32] = "mediatek,";

			pled_dtsi[i].name = leds_name[i];

			led_node =
			    of_find_compatible_node(NULL, NULL,
						    strcat(node_name,
							   leds_name[i]));
			if (!led_node) {
				LEDS_DEBUG("Cannot find LED node from dts\n");
				pled_dtsi[i].mode = 0;
				pled_dtsi[i].data = -1;
			} else {
				isSupportDTS = true;
				ret =
				    of_property_read_u32(led_node, "led_mode",
							 &mode);
				if (!ret) {
					pled_dtsi[i].mode = mode;
					LEDS_DEBUG
					    ("The %s's led mode is : %d\n",
					     pled_dtsi[i].name,
					     pled_dtsi[i].mode);
				} else {
					LEDS_DEBUG
					    ("led dts can not get led mode");
					pled_dtsi[i].mode = 0;
				}

				ret =
				    of_property_read_u32(led_node, "data",
							 &data);
				if (!ret) {
					pled_dtsi[i].data = data;
					LEDS_DEBUG
					    ("The %s's led data is : %ld\n",
					     pled_dtsi[i].name,
					     pled_dtsi[i].data);
				} else {
					LEDS_DEBUG
					    ("led dts can not get led data");
					pled_dtsi[i].data = -1;
				}

				ret =
				    of_property_read_u32_array(led_node,
							       "pwm_config",
							       pwm_config,
							       ARRAY_SIZE
							       (pwm_config));
				if (!ret) {
					LEDS_DEBUG
					    ("The %s's pwm config data is %d %d %d %d %d\n",
					     pled_dtsi[i].name, pwm_config[0],
					     pwm_config[1], pwm_config[2],
					     pwm_config[3], pwm_config[4]);
					pled_dtsi[i].config_data.clock_source =
					    pwm_config[0];
					pled_dtsi[i].config_data.div =
					    pwm_config[1];
					pled_dtsi[i].config_data.low_duration =
					    pwm_config[2];
					pled_dtsi[i].config_data.High_duration =
					    pwm_config[3];
					pled_dtsi[i].config_data.pmic_pad =
					    pwm_config[4];

				} else
					LEDS_DEBUG
					    ("led dts can not get pwm config data.\n");

				switch (pled_dtsi[i].mode) {
				case MT65XX_LED_MODE_CUST_LCM:
					pled_dtsi[i].data =
					    (long)mtkfb_set_backlight_level;
					LEDS_DEBUG
					    ("kernel:the backlight hw mode is LCM.\n");
					break;
				case MT65XX_LED_MODE_CUST_BLS_PWM:
					pled_dtsi[i].data =
					    (long)disp_bls_set_backlight;
					LEDS_DEBUG
					    ("kernel:the backlight hw mode is BLS.\n");
					break;
				default:
					break;
				}
			}
		}

		if (!isSupportDTS) {
			kfree(pled_dtsi);
			pled_dtsi = NULL;
		}
	}
 out:
	return pled_dtsi;
}

struct cust_mt65xx_led *mt_get_cust_led_list(void)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_dtsi();
	return cust_led_list;
}

/****************************************************************************
 * internal functions
 ***************************************************************************/
static int brightness_mapto64(int level)
{
	if (level < 30)
		return (level >> 1) + 7;
	else if (level <= 120)
		return (level >> 2) + 14;
	else if (level <= 160)
		return level / 5 + 20;
	else
		return (level >> 3) + 33;
}

static int find_time_index(int time)
{
	int index = 0;

	while (index < 8) {
		if (time < time_array_hal[index])
			return index;
		index++;
	}
	return PWM_DIV_NUM - 1;
}

int mt_led_set_pwm(int pwm_num, struct nled_setting *led)
{
	struct pwm_spec_config pwm_setting;
	int time_index = 0;

	pwm_setting.pwm_no = pwm_num;
	pwm_setting.mode = PWM_MODE_OLD;

	LEDS_DEBUG("led_set_pwm: mode=%d,pwm_no=%d\n", led->nled_mode,
		   pwm_num);
	/* We won't choose 32K to be the clock src of old mode because of system performance. */
	/* The setting here will be clock src = 26MHz, CLKSEL = 26M/1625 (i.e. 16K) */
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;

	switch (led->nled_mode) {
	/* Actually, the setting still can not to turn off NLED. We should disable PWM to turn off NLED. */
	case NLED_OFF:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 0;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;

	case NLED_ON:
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = 30 / 2;
		pwm_setting.clk_div = CLK_DIV1;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100 / 2;
		break;

	case NLED_BLINK:
		LEDS_DEBUG("LED blink on time = %d offtime = %d\n",
			   led->blink_on_time, led->blink_off_time);
		time_index =
		    find_time_index(led->blink_on_time + led->blink_off_time);
		LEDS_DEBUG("LED div is %d\n", time_index);
		pwm_setting.clk_div = time_index;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH =
		    (led->blink_on_time +
		     led->blink_off_time) * MIN_FRE_OLD_PWM /
		    div_array_hal[time_index];
		pwm_setting.PWM_MODE_OLD_REGS.THRESH =
		    (led->blink_on_time * 100) / (led->blink_on_time +
						  led->blink_off_time);
	}

	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	pwm_set_spec_config(&pwm_setting);

	return 0;
}

/************************ led breath function*****************************/
/*************************************************************************
//func is to swtich to breath mode from PWM mode of ISINK
//para: enable: 1 : breath mode; 0: PWM mode;
*************************************************************************/
#if 0
static int led_switch_breath_pmic(enum mt65xx_led_pmic pmic_type,
				  struct nled_setting *led, int enable)
{
	/* int time_index = 0; */
	/* int duty = 0; */
	LEDS_DEBUG("led_blink_pmic: pmic_type=%d\n", pmic_type);

	if ((pmic_type != MT65XX_LED_PMIC_NLED_ISINK0
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK1
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK2
	     && pmic_type != MT65XX_LED_PMIC_NLED_ISINK3)
	    || led->nled_mode != NLED_BLINK) {
		return -1;
	}
	if (1 == enable) {
		switch (pmic_type) {
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK1:
			pmic_set_register_value(PMIC_ISINK_CH1_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK2:
			pmic_set_register_value(PMIC_ISINK_CH2_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH2_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM2_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM2_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH2_EN,NLED_ON); */
			break;
		case MT65XX_LED_PMIC_NLED_ISINK3:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_BREATH_MODE);
			pmic_set_register_value(PMIC_ISINK_CH3_STEP, ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TR1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TR2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TF1_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TF2_SEL,
						0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TON_SEL,
						0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TOFF_SEL,
						0x03);
			pmic_set_register_value(PMIC_ISINK_DIM3_DUTY, 15);
			pmic_set_register_value(PMIC_ISINK_DIM3_FSEL, 11);
			/* pmic_set_register_value(PMIC_ISINK_CH3_EN,NLED_ON); */
			break;
		default:
			break;
		}
	} else {
		switch (pmic_type) {
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,
						ISINK_PWM_MODE);
			break;
		default:
			break;
		}
	}
	return 0;

}
#endif

#define PMIC_PERIOD_NUM 9
/* 100 * period, ex: 0.01 Hz -> 0.01 * 100 = 1 */
int pmic_period_array[] = { 250, 500, 1000, 1250, 1666, 2000, 2500, 10000 };

/* int pmic_freqsel_array[] = {99999, 9999, 4999, 1999, 999, 499, 199, 4, 0}; */
int pmic_freqsel_array[] = { 0, 4, 199, 499, 999, 1999, 1999, 1999 };

static int find_time_index_pmic(int time_ms) {
	int i;

	for (i = 0; i < PMIC_PERIOD_NUM; i++) {
		if (time_ms <= pmic_period_array[i])
			return i;
	}
	return PMIC_PERIOD_NUM - 1;
}

int mt_led_blink_pmic(enum mt65xx_led_pmic pmic_type, struct nled_setting *led)
{
	int time_index = 0;
	int duty = 0;

	LEDS_DEBUG("led_blink_pmic: pmic_type=%d\n", pmic_type);

	if((pmic_type != MT65XX_LED_PMIC_NLED_ISINK0 && pmic_type!= MT65XX_LED_PMIC_NLED_ISINK1 && 
		pmic_type!= MT65XX_LED_PMIC_NLED_ISINK2 && pmic_type!= MT65XX_LED_PMIC_NLED_ISINK3) || led->nled_mode != NLED_BLINK) {
		return -1;
	}

	LEDS_DEBUG("LED blink on time = %d offtime = %d\n",
		   led->blink_on_time, led->blink_off_time);
	time_index =
	    find_time_index_pmic(led->blink_on_time + led->blink_off_time);
	LEDS_DEBUG("LED index is %d  freqsel=%d\n", time_index,
		   pmic_freqsel_array[time_index]);
	duty =
	    32 * led->blink_on_time / (led->blink_on_time +
				       led->blink_off_time);
	/* pmic_set_register_value(PMIC_RG_G_DRV_2M_CK_PDN(0X0); // DISABLE POWER DOWN ,Indicator no need) */
	/*pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN, 0x0);	*//* Disable power down */
	switch (pmic_type) {
	case MT65XX_LED_PMIC_NLED_ISINK0:
			  upmu_set_rg_isink0_ck_pdn(0);
			  upmu_set_rg_isink0_ck_sel(0);
		upmu_set_isink_ch0_mode(ISINK_PWM_MODE);	/* 6320 spec */
			  upmu_set_isink_ch0_step(0x3);//16mA
			  upmu_set_isink_dim0_duty(duty);
			  upmu_set_isink_dim0_fsel(pmic_freqsel_array[time_index]);
			  upmu_set_isink_breath0_trf_sel(0x0);
			  upmu_set_isink_breath0_ton_sel(0x02);
			  upmu_set_isink_breath0_toff_sel(0x05);			
			  upmu_set_isink_ch0_en(0x01);

		break;
	case MT65XX_LED_PMIC_NLED_ISINK1:
			  upmu_set_rg_isink1_ck_pdn(0);
			  upmu_set_rg_isink1_ck_sel(0);
			  upmu_set_isink_ch1_mode(ISINK_PWM_MODE);
			  upmu_set_isink_ch1_step(0x3);//16mA
			  upmu_set_isink_dim1_duty(duty);
			  upmu_set_isink_dim1_fsel(pmic_freqsel_array[time_index]);
			  upmu_set_isink_breath1_trf_sel(0x0);
			  upmu_set_isink_breath1_ton_sel(0x02);
			  upmu_set_isink_breath1_toff_sel(0x05);			
			  upmu_set_isink_ch1_en(0x01);

		break;
	case MT65XX_LED_PMIC_NLED_ISINK2:	/* keypad LR  16mA */
			  upmu_set_rg_isink2_ck_pdn(0);
			  upmu_set_rg_isink2_ck_sel(0);
		upmu_set_isink_ch2_mode(ISINK_PWM_MODE);	/* 6320 spec */
			  upmu_set_isink_ch2_step(0x3);//16mA
			  upmu_set_isink_dim2_duty(duty);
			  upmu_set_isink_dim2_fsel(pmic_freqsel_array[time_index]);
			  upmu_set_isink_breath2_trf_sel(0x0);
			  upmu_set_isink_breath2_ton_sel(0x02);
			  upmu_set_isink_breath2_toff_sel(0x05);
			  upmu_set_isink_ch2_en(0x01);

			break;	
		case MT65XX_LED_PMIC_NLED_ISINK3:
			  upmu_set_rg_isink3_ck_pdn(0);
			  upmu_set_rg_isink3_ck_sel(0);
			  upmu_set_isink_ch3_mode(ISINK_PWM_MODE);
			  upmu_set_isink_ch3_step(0x3);//16mA
			  upmu_set_isink_dim3_duty(duty);
			  upmu_set_isink_dim3_fsel(pmic_freqsel_array[time_index]);
			  upmu_set_isink_breath3_trf_sel(0x0);
			  upmu_set_isink_breath3_ton_sel(0x02);
			  upmu_set_isink_breath3_toff_sel(0x05);
			  upmu_set_isink_ch3_en(0x01);

		break;
	default:
		break;
	}
	return 0;
}

int mt_backlight_set_pwm(int pwm_num, u32 level, u32 div,
			 struct PWM_config *config_data)
{
	struct pwm_spec_config pwm_setting;
	unsigned int BacklightLevelSupport =
	    Cust_GetBacklightLevelSupport_byPWM();
	pwm_setting.pwm_no = pwm_num;

	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT)
		pwm_setting.mode = PWM_MODE_OLD;
	else
		pwm_setting.mode = PWM_MODE_FIFO;	/* New mode fifo and periodical mode */

	pwm_setting.pmic_pad = config_data->pmic_pad;

	if (config_data->div) {
		pwm_setting.clk_div = config_data->div;
		backlight_PWM_div_hal = config_data->div;
	} else
		pwm_setting.clk_div = div;

	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT) {
		if (config_data->clock_source)
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
		else
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;	/* actually.
			it's block/1625 = 26M/1625 = 16KHz @ MT6571 */

		pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
		pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 255;	/* 256 level */
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;

		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:duty is %d/%d\n",
			   BacklightLevelSupport, level,
			   pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:clk_src/div is %d%d\n",
			   BacklightLevelSupport, pwm_setting.clk_src,
			   pwm_setting.clk_div);
		if (level > 0 && level < 256) {
			pwm_set_spec_config(&pwm_setting);
			LEDS_DEBUG
			    ("[LEDS][%d]backlight_set_pwm: old mode: thres/data_width is %d/%d\n",
			     BacklightLevelSupport,
			     pwm_setting.PWM_MODE_OLD_REGS.THRESH,
			     pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		} else {
			LEDS_DEBUG("[LEDS][%d]Error level in backlight\n",
				   BacklightLevelSupport);
			mt_pwm_disable(pwm_setting.pwm_no,
				       config_data->pmic_pad);
		}
		return 0;

	} else {
		if (config_data->clock_source) {
			pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		} else {
			pwm_setting.clk_src =
			    PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625;
		}

		if (config_data->High_duration && config_data->low_duration) {
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION =
			    config_data->High_duration;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION =
			    pwm_setting.PWM_MODE_FIFO_REGS.HDURATION;
		} else {
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 4;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 4;
		}

		pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 31;
		pwm_setting.PWM_MODE_FIFO_REGS.GDURATION =
		    (pwm_setting.PWM_MODE_FIFO_REGS.HDURATION + 1) * 32 - 1;
		pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;

		LEDS_DEBUG("[LEDS]backlight_set_pwm:duty is %d\n", level);
		LEDS_DEBUG
		    ("[LEDS]backlight_set_pwm:clk_src/div/high/low is %d%d%d%d\n",
		     pwm_setting.clk_src, pwm_setting.clk_div,
		     pwm_setting.PWM_MODE_FIFO_REGS.HDURATION,
		     pwm_setting.PWM_MODE_FIFO_REGS.LDURATION);

		if (level > 0 && level <= 32) {
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =
			    (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		} else if (level > 32 && level <= 64) {
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 1;
			level -= 32;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 =
			    (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		} else {
			LEDS_DEBUG("[LEDS]Error level in backlight\n");
			mt_pwm_disable(pwm_setting.pwm_no,
				       config_data->pmic_pad);
		}

		return 0;

	}
}

void mt_led_pwm_disable(int pwm_num)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_dtsi();

	mt_pwm_disable(pwm_num, cust_led_list->config_data.pmic_pad);
}

void mt_backlight_set_pwm_duty(int pwm_num, u32 level, u32 div,
			       struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_set_pwm_div(int pwm_num, u32 level, u32 div,
			      struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_get_pwm_fsel(unsigned int bl_div, unsigned int *bl_frequency)
{

}

void mt_store_pwm_register(unsigned int addr, unsigned int value)
{

}

unsigned int mt_show_pwm_register(unsigned int addr)
{
	return 0;
}

int mt_brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
		int tmp_level = level;
		static bool backlight_init_flag = false;
	static bool first_time = true;
		static unsigned char duty_mapping[108] = {
       		 0,	0,	0,	0,	0,	6,	1,	2,	1,	10,	1,	12,
        	 6,	14,	2,	3,	16,	2,	18,	3,	6,	10,	22, 3,
        	12,	8,	6,	28,	4,	30,	7,	10,	16,	6,	5,	18,
        	12,	7,	6,	10,	8,	22,	7,	9,	16,	12,	8,	10,
        	13,	18,	28,	9,	30,	20,	15,	12,	10,	16,	22,	13,
        	11,	14,	18,	12,	19,	15,	26,	13,	16,	28,	21,	14,
        	22,	30,	18,	15,	19,	16,	25,	20,	17,	21,	27,	18,
        	22,	28,	19,	30,	24,	20,	31,	25,	21,	26,	22,	27,
        	23,	28,	24,	30,	25,	31,	26,	27,	28,	29,	30,	31,
    	};
    	static unsigned char current_mapping[108] = {
        	1,	2,	3,	4,	5,	0,	3,	2,	4,	0,	5,	0,
        	1,	0,	4,	3,	0,	5,	0,	4,	2,	1,	0,	5,
        	1,	2,	3,	0,	5,	0,	3,	2,	1,	4,	5,	1,
        	2,	4,	5,	3,	4,	1,	5,	4,	2,	3,	5,	4,
        	3,	2,	1,	5,	1,	2,	3,	4,	5,	3,	2,	4,
        	5,	4,	3,	5,	3,	4,	2,	5,	4,	2,	3,	5,
        	3,	2,	4,	5,	4,	5,	3,	4,	5,	4,	3,	5,
        	4,	3,	5,	3,	4,	5,	3,	4,	5,	4,	5,	4,
        	5,	4,	5,	4,	5,	4,	5,	5,	5,	5,	5,	5,
    	};

	LEDS_DEBUG("[LED]PMIC#%d:%d\n", pmic_type, level);

	if (pmic_type == MT65XX_LED_PMIC_LCD_ISINK)
	{
		if(backlight_init_flag == false)
		{
            upmu_set_rg_drv_2m_ck_pdn(0x0); // Disable power down 

            // For backlight: Current: 24mA, PWM frequency: 20K, Duty: 20~100, Soft start: off, Phase shift: on
            // ISINK0
            upmu_set_rg_isink0_ck_pdn(0x0); // Disable power down    
            upmu_set_rg_isink0_ck_sel(0x1); // Freq = 1Mhz for Backlight
			upmu_set_isink_ch0_mode(ISINK_PWM_MODE);
            upmu_set_isink_ch0_step(0x5); // 24mA
            upmu_set_isink_sfstr0_en(0x0); // Disable soft start
			upmu_set_rg_isink0_double_en(0x1); // Enable double current
			upmu_set_isink_phase_dly_tc(0x0); // TC = 0.5us
			upmu_set_isink_phase0_dly_en(0x1); // Enable phase delay
            upmu_set_isink_chop0_en(0x1); // Enable CHOP clk
            // ISINK1
            upmu_set_rg_isink1_ck_pdn(0x0); // Disable power down   
            upmu_set_rg_isink1_ck_sel(0x1); // Freq = 1Mhz for Backlight
			upmu_set_isink_ch1_mode(ISINK_PWM_MODE);
            upmu_set_isink_ch1_step(0x5); // 24mA
            upmu_set_isink_sfstr1_en(0x0); // Disable soft start
			upmu_set_rg_isink1_double_en(0x1); // Enable double current
			upmu_set_isink_phase1_dly_en(0x1); // Enable phase delay
            upmu_set_isink_chop1_en(0x1); // Enable CHOP clk         
            // ISINK2
            upmu_set_rg_isink2_ck_pdn(0x0); // Disable power down   
            upmu_set_rg_isink2_ck_sel(0x1); // Freq = 1Mhz for Backlight
			upmu_set_isink_ch2_mode(ISINK_PWM_MODE);
            upmu_set_isink_ch2_step(0x5); // 24mA
            upmu_set_isink_sfstr2_en(0x0); // Disable soft start
			upmu_set_rg_isink2_double_en(0x1); // Enable double current
			upmu_set_isink_phase2_dly_en(0x1); // Enable phase delay
            upmu_set_isink_chop2_en(0x1); // Enable CHOP clk   
            // ISINK3
            upmu_set_rg_isink3_ck_pdn(0x0); // Disable power down   
            upmu_set_rg_isink3_ck_sel(0x1); // Freq = 1Mhz for Backlight
			upmu_set_isink_ch3_mode(ISINK_PWM_MODE);
            upmu_set_isink_ch3_step(0x5); // 24mA
            upmu_set_isink_sfstr3_en(0x0); // Disable soft start
			upmu_set_rg_isink3_double_en(0x1); // Enable double current
			upmu_set_isink_phase3_dly_en(0x1); // Enable phase delay
            upmu_set_isink_chop3_en(0x1); // Enable CHOP clk                
			backlight_init_flag = true;
		}
		
		if (level) 
		{
			level = brightness_mapping(tmp_level);
			if(level == ERROR_BL_LEVEL)
				level = 255;
		    if(level == 255)
            {
                level = 108;
            }
            else
            {
                level = ((level * 108) / 255) + 1;
            }
            LEDS_DEBUG("[LED]Level Mapping = %d \n", level);
            LEDS_DEBUG("[LED]ISINK DIM Duty = %d \n", duty_mapping[level-1]);
            LEDS_DEBUG("[LED]ISINK Current = %d \n", current_mapping[level-1]);
            upmu_set_isink_dim0_duty(duty_mapping[level-1]);
            upmu_set_isink_dim1_duty(duty_mapping[level-1]);
            upmu_set_isink_dim2_duty(duty_mapping[level-1]);
            upmu_set_isink_dim3_duty(duty_mapping[level-1]);
            upmu_set_isink_ch0_step(current_mapping[level-1]);
            upmu_set_isink_ch1_step(current_mapping[level-1]);
            upmu_set_isink_ch2_step(current_mapping[level-1]);
            upmu_set_isink_ch3_step(current_mapping[level-1]);
            upmu_set_isink_dim0_fsel(0x2); // 20Khz
            upmu_set_isink_dim1_fsel(0x2); // 20Khz
            upmu_set_isink_dim2_fsel(0x2); // 20Khz
            upmu_set_isink_dim3_fsel(0x2); // 20Khz            
            upmu_set_isink_ch0_en(0x1); // Turn on ISINK Channel 0
            upmu_set_isink_ch1_en(0x1); // Turn on ISINK Channel 1
            upmu_set_isink_ch2_en(0x1); // Turn on ISINK Channel 2
            upmu_set_isink_ch3_en(0x1); // Turn on ISINK Channel 3
			bl_duty_hal = level;
		}
		else 
		{
            upmu_set_isink_ch0_en(0x0); // Turn off ISINK Channel 0
            upmu_set_isink_ch1_en(0x0); // Turn off ISINK Channel 1
            upmu_set_isink_ch2_en(0x0); // Turn off ISINK Channel 2
            upmu_set_isink_ch3_en(0x0); // Turn off ISINK Channel 3
			bl_duty_hal = level;
		}
        mutex_unlock(&leds_pmic_mutex);
		return 0;
	}
		else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK0)
		{
			if((button_flag_isink0==0) && (first_time == true)) {//button flag ==0, means this ISINK is not for button backlight
				if(button_flag_isink1==0)
					upmu_set_isink_ch1_en(0); 
				if(button_flag_isink2==0)
					upmu_set_isink_ch2_en(0);
				if(button_flag_isink3==0)
					upmu_set_isink_ch3_en(0);
				first_time = false;
			}

				upmu_set_rg_drv_32k_ck_pdn(0x0); // Disable power down  
				upmu_set_rg_isink0_ck_pdn(0);
				upmu_set_rg_isink0_ck_sel(0);
				upmu_set_isink_ch0_mode(ISINK_PWM_MODE);
				upmu_set_isink_ch0_step(0x3);//16mA
				//hwPWMsetting(PMIC_PWM_1, 15, 8);
				upmu_set_isink_dim0_duty(15);
				upmu_set_isink_dim0_fsel(0);//6323 1KHz
				
			
			if (level) 
			{

				upmu_set_isink_ch0_en(0x01);
				
			}
			else 
			{
				upmu_set_isink_ch0_en(0x0);
			}
			mutex_unlock(&leds_pmic_mutex);
			return 0;
		}
		else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK1)
		{
			if((button_flag_isink1==0) && (first_time == true)) {//button flag ==0, means this ISINK is not for button backlight
				if(button_flag_isink0==0)
					upmu_set_isink_ch0_en(0); 
				if(button_flag_isink2==0)
					upmu_set_isink_ch2_en(0);
				if(button_flag_isink3==0)
					upmu_set_isink_ch3_en(0);
				first_time = false;
			}

				upmu_set_rg_drv_32k_ck_pdn(0x0); // Disable power down  
				upmu_set_rg_isink1_ck_pdn(0);
				upmu_set_rg_isink1_ck_sel(0);
				upmu_set_isink_ch1_mode(ISINK_PWM_MODE);
				upmu_set_isink_ch1_step(0x3);//16mA
				//hwPWMsetting(PMIC_PWM_2, 15, 8);
				upmu_set_isink_dim1_duty(15);
				upmu_set_isink_dim1_fsel(0);//6323 1KHz

			if (level) 
			{
			   #ifdef NO_NEED_USB_LED
			    LEDS_DEBUG("[LED]NO_NEED_USB_LED +++++++++++++++\n");
				if(	FACTORY_BOOT == get_boot_mode() || META_BOOT == get_boot_mode()) {
					upmu_set_isink_ch1_en(0x1);
				}else {
					upmu_set_isink_ch1_en(0x0);
				}
			   #else
				upmu_set_isink_ch1_en(0x01);
			   #endif
			}
			else 
			{
				upmu_set_isink_ch1_en(0x0);
			}
			mutex_unlock(&leds_pmic_mutex);
			return 0;
		}
		else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK2)
		{
			if((button_flag_isink2==0) && (first_time == true)) {//button flag ==0, means this ISINK is not for button backlight
				if(button_flag_isink0==0)
					upmu_set_isink_ch0_en(0); 
				if(button_flag_isink1==0)
					upmu_set_isink_ch1_en(0);
				if(button_flag_isink3==0)
					upmu_set_isink_ch3_en(0);
				first_time = false;
			}
				upmu_set_rg_drv_32k_ck_pdn(0x0); // Disable power down  
				upmu_set_rg_isink2_ck_pdn(0);
				upmu_set_rg_isink2_ck_sel(0);
				upmu_set_isink_ch2_mode(ISINK_PWM_MODE);
				upmu_set_isink_ch2_step(0x3);//16mA
				//hwPWMsetting(PMIC_PWM_2, 15, 8);
				upmu_set_isink_dim2_duty(15);
				upmu_set_isink_dim2_fsel(0);//6323 1KHz


			if (level) 
			{
				upmu_set_isink_ch2_en(0x01);
			}
			else 
			{
				upmu_set_isink_ch2_en(0x0);
			}
			mutex_unlock(&leds_pmic_mutex);
			return 0;
		}
		else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK3)
		{
			if((button_flag_isink3==0) && (first_time == true)) {//button flag ==0, means this ISINK is not for button backlight
				if(button_flag_isink0==0)
					upmu_set_isink_ch0_en(0); 
				if(button_flag_isink1==0)
					upmu_set_isink_ch1_en(0);
				if(button_flag_isink2==0)
					upmu_set_isink_ch2_en(0);
				first_time = false;
			}

				upmu_set_rg_drv_32k_ck_pdn(0x0); // Disable power down  
				upmu_set_rg_isink3_ck_pdn(0);
				upmu_set_rg_isink3_ck_sel(0);
				upmu_set_isink_ch3_mode(ISINK_PWM_MODE);
				upmu_set_isink_ch3_step(0x3);//16mA
				//hwPWMsetting(PMIC_PWM_1, 15, 8);
				upmu_set_isink_dim3_duty(15);
				upmu_set_isink_dim3_fsel(0);//6323 1KHz

			
			if (level) 
			{
				upmu_set_isink_ch3_en(0x01);
				
			}
			else 
			{
				upmu_set_isink_ch3_en(0x0);
			}
			mutex_unlock(&leds_pmic_mutex);
			return 0;
		}
	  #if 0
		else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK01)
		{
	
			//hwBacklightISINKTuning(0x0, 0x3, 0x0, 0);  //register mode, ch0_step=4ma ,disable CABC
			//hwBacklightISINKTurnOn(0x0);  //turn on ISINK0
	
			//if(led_init_flag[1] == false)
			{

				upmu_set_isinks_ch0_mode(PMIC_PWM_0);
				upmu_set_isinks_ch0_step(0x0);//4mA
				upmu_set_isink_dim0_duty(1);
				upmu_set_isink_dim0_fsel(1);//6320 1.5KHz
				//hwBacklightISINKTuning(MT65XX_LED_PMIC_NLED_ISINK5, PMIC_PWM_2, 0x0, 0);
				upmu_set_isinks_ch1_mode(PMIC_PWM_0);
				upmu_set_isinks_ch1_step(0x3);//4mA
				//hwPWMsetting(PMIC_PWM_2, 15, 8);
				upmu_set_isink_dim1_duty(15);
				upmu_set_isink_dim1_fsel(11);//6320 0.25KHz
				led_init_flag[0] = true;
				led_init_flag[1] = true;
			}
			if (level) 
			{
				//upmu_top2_bst_drv_ck_pdn(0x0);
				
				//upmu_set_rg_spk_div_pdn(0x01);
				//upmu_set_rg_spk_pwm_div_pdn(0x01);
				upmu_set_rg_bst_drv_1m_ck_pdn(0x0);
				#ifdef ISINK_CHOP_CLK
				//upmu_set_isinks0_chop_en(0x1);
				//upmu_set_isinks1_chop_en(0x1);
				#endif
				upmu_set_isinks_ch0_en(0x01);
				upmu_set_isinks_ch1_en(0x01);
			}
			else 
			{
				upmu_set_isinks_ch0_en(0x00);
				upmu_set_isinks_ch1_en(0x00);
				//upmu_set_rg_bst_drv_1m_ck_pdn(0x1);
				//upmu_top2_bst_drv_ck_pdn(0x1);
			}
			return 0;
		}
	   #endif

		mutex_unlock(&leds_pmic_mutex);	
		return -1;

}

#if defined(CONFIG_MTK_CHARGER_LED_SUPPORT)
int mt_brightness_get_insink_state(enum mt65xx_led_pmic pmic_type)
{
	int val = 0;
	switch(pmic_type) {
		case MT65XX_LED_PMIC_LCD_ISINK:
			break;

		case MT65XX_LED_PMIC_NLED_ISINK0:
			val = upmu_get_isink_ch0_en();
			udelay(1);
			val = upmu_get_isink_ch0_en();
			udelay(1);
			val = upmu_get_isink_ch0_en();
			LEDS_DEBUG("=====>%s %d ISINK(0) val=%d=====>\n",
				__func__, __LINE__, val);
			break;
	
		case MT65XX_LED_PMIC_NLED_ISINK1:
			val = upmu_get_isink_ch1_en();
			udelay(1);
			val = upmu_get_isink_ch1_en();
			udelay(1);
			val = upmu_get_isink_ch1_en();
			LEDS_DEBUG("=====>%s %d ISINK(1) val=%d=====>\n",
				__func__, __LINE__, val);
			break;
	
		case MT65XX_LED_PMIC_NLED_ISINK2:
			val = upmu_get_isink_ch2_en();
			udelay(1);
			val = upmu_get_isink_ch2_en();
			udelay(1);
			val = upmu_get_isink_ch2_en();
			LEDS_DEBUG("=====>%s %d ISINK(2) val=%d=====>\n",
				__func__, __LINE__, val);
			break;
		
		case MT65XX_LED_PMIC_NLED_ISINK3:
			val = upmu_get_isink_ch3_en();
			udelay(1);
			val = upmu_get_isink_ch3_en();
			udelay(1);
			val = upmu_get_isink_ch3_en();
			LEDS_DEBUG("=====>%s %d ISINK(3) val=%d=====>\n",
				__func__, __LINE__,val);
			break;

		default:
			break;
	}
	return val;
}
#endif

int mt_brightness_set_pmic_duty_store(u32 level, u32 div)
{
	return -1;
}

int mt_mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level)
{
	struct nled_setting led_tmp_setting = { 0, 0, 0 };
	int tmp_level = level;
	static bool button_flag = false;
	unsigned int BacklightLevelSupport =
	    Cust_GetBacklightLevelSupport_byPWM();

    LEDS_DEBUG("mt65xx_leds_set_cust: set brightness, name:%s, mode:%d, level:%d\n", 
		cust->name, cust->mode, level);
	switch (cust->mode) {

	case MT65XX_LED_MODE_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0) {
			bl_brightness_hal = level;
			if (level == 0) {
				mt_pwm_disable(cust->data,
					       cust->config_data.pmic_pad);

			} else {

				if (BacklightLevelSupport ==
				    BACKLIGHT_LEVEL_PWM_256_SUPPORT)
					level = brightness_mapping(tmp_level);
				else
					level = brightness_mapto64(tmp_level);
				mt_backlight_set_pwm(cust->data, level,
						     bl_div_hal,
						     &cust->config_data);
			}
			bl_duty_hal = level;

		} else {
			if (level == 0) {
				led_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(cust->data, &led_tmp_setting);
				mt_pwm_disable(cust->data,
					       cust->config_data.pmic_pad);
			} else {
				led_tmp_setting.nled_mode = NLED_ON;
				mt_led_set_pwm(cust->data, &led_tmp_setting);
			}
		}
		return 1;

	case MT65XX_LED_MODE_GPIO:
		LEDS_DEBUG("brightness_set_cust:go GPIO mode!!!!!\n");
		return ((cust_set_brightness) (cust->data)) (level);

	case MT65XX_LED_MODE_PMIC:
			//for button baclight used SINK channel, when set button ISINK, don't do disable other ISINK channel
			if((strcmp(cust->name,"button-backlight") == 0)) {
				if(button_flag==false) {
					switch (cust->data) {
						case MT65XX_LED_PMIC_NLED_ISINK0:
							button_flag_isink0 = 1;
							break;
						case MT65XX_LED_PMIC_NLED_ISINK1:
							button_flag_isink1 = 1;
							break;
						case MT65XX_LED_PMIC_NLED_ISINK2:
							button_flag_isink2 = 1;
							break;
						case MT65XX_LED_PMIC_NLED_ISINK3:
							button_flag_isink3 = 1;
							break;
						default:
							break;
					}
					button_flag=true;
				}
			}					
		return mt_brightness_set_pmic(cust->data, level, bl_div_hal);

	case MT65XX_LED_MODE_CUST_LCM:
		if (strcmp(cust->name, "lcd-backlight") == 0)
			{
			bl_brightness_hal = level;
            }
			LEDS_DEBUG("brightness_set_cust:backlight control by LCM\n");
		return ((cust_brightness_set) (cust->data)) (level, bl_div_hal);

	case MT65XX_LED_MODE_CUST_BLS_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0)
			bl_brightness_hal = level;
		return ((cust_set_brightness) (cust->data)) (level);

	case MT65XX_LED_MODE_NONE:
	default:
		break;
	}
	return -1;
}

void mt_mt65xx_led_work(struct work_struct *work)
{
	struct mt65xx_led_data *led_data =
	    container_of(work, struct mt65xx_led_data, work);

	LEDS_DEBUG("%s:%d\n", led_data->cust.name, led_data->level);
	mutex_lock(&leds_mutex);
	mt_mt65xx_led_set_cust(&led_data->cust, led_data->level);
	mutex_unlock(&leds_mutex);
}

void mt_mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct mt65xx_led_data *led_data =
	    container_of(led_cdev, struct mt65xx_led_data, cdev);
	/* unsigned long flags; */
	/* spin_lock_irqsave(&leds_lock, flags); */

#ifdef CONFIG_MTK_AAL_SUPPORT
	if (led_data->level != level) {
		led_data->level = level;
		if (strcmp(led_data->cust.name, "lcd-backlight") != 0) {
			LEDS_DEBUG("Set NLED directly %d at time %lu\n",
				   led_data->level, jiffies);
			schedule_work(&led_data->work);
		} else {
			if (level != 0
			    && level * CONFIG_LIGHTNESS_MAPPING_VALUE < 255) {
				level = 1;
			} else {
				level =
				    (level * CONFIG_LIGHTNESS_MAPPING_VALUE) /
				    255;
			}
			LEDS_DEBUG
			    ("Set Backlight directly %d at time %lu, mapping level is %d\n",
			     led_data->level, jiffies, level);
			/* mt_mt65xx_led_set_cust(&led_data->cust, led_data->level); */
			disp_aal_notify_backlight_changed((((1 <<
							     MT_LED_INTERNAL_LEVEL_BIT_CNT)
							    - 1) * level +
							   127) / 255);
		}
	}
#else
	/* do something only when level is changed */
	if (led_data->level != level) {
		led_data->level = level;
		if (strcmp(led_data->cust.name, "lcd-backlight") != 0) {
			LEDS_DEBUG("Set NLED directly %d at time %lu\n",
				   led_data->level, jiffies);
			schedule_work(&led_data->work);
		} else {
			if (level != 0
			    && level * CONFIG_LIGHTNESS_MAPPING_VALUE < 255) {
				level = 1;
			} else {
				level =
				    (level * CONFIG_LIGHTNESS_MAPPING_VALUE) /
				    255;
			}
			LEDS_DEBUG
			    ("Set Backlight directly %d at time %lu, mapping level is %d\n",
			     led_data->level, jiffies, level);
			if (MT65XX_LED_MODE_CUST_BLS_PWM == led_data->cust.mode) {
				mt_mt65xx_led_set_cust(&led_data->cust,
						       ((((1 <<
							   MT_LED_INTERNAL_LEVEL_BIT_CNT)
							  - 1) * level +
							 127) / 255));
			} else {
				mt_mt65xx_led_set_cust(&led_data->cust, level);
			}
		}
	}
	/* spin_unlock_irqrestore(&leds_lock, flags); */
#endif
/* if(0!=aee_kernel_Powerkey_is_press()) */
/* aee_kernel_wdt_kick_Powkey_api("mt_mt65xx_led_set",WDT_SETBY_Backlight); */
}

int mt_mt65xx_blink_set(struct led_classdev *led_cdev,
			unsigned long *delay_on, unsigned long *delay_off)
{
	struct mt65xx_led_data *led_data =
	    container_of(led_cdev, struct mt65xx_led_data, cdev);
	static int got_wake_lock;
	struct nled_setting nled_tmp_setting = { 0, 0, 0 };

	/* only allow software blink when delay_on or delay_off changed */
	if (*delay_on != led_data->delay_on
	    || *delay_off != led_data->delay_off) {
		led_data->delay_on = *delay_on;
		led_data->delay_off = *delay_off;
		if (led_data->delay_on && led_data->delay_off) {	/* enable blink */
			led_data->level = 255;	/* when enable blink  then to set the level  (255) */
			/* AP PWM all support OLD mode */
			if (led_data->cust.mode == MT65XX_LED_MODE_PWM) {
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time =
				    led_data->delay_off;
				nled_tmp_setting.blink_on_time =
				    led_data->delay_on;
				mt_led_set_pwm(led_data->cust.data,
					       &nled_tmp_setting);
				return 0;
			} else if ((led_data->cust.mode == MT65XX_LED_MODE_PMIC)
				   && (led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK0
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK1
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK2)) {
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time =
				    led_data->delay_off;
				nled_tmp_setting.blink_on_time =
				    led_data->delay_on;
				mt_led_blink_pmic(led_data->cust.data,
						  &nled_tmp_setting);
				return 0;
			} else if (!got_wake_lock) {
				wake_lock(&leds_suspend_lock);
				got_wake_lock = 1;
			}
		} else if (!led_data->delay_on && !led_data->delay_off) {	/* disable blink */
			/* AP PWM all support OLD mode */
			if (led_data->cust.mode == MT65XX_LED_MODE_PWM) {
				nled_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(led_data->cust.data,
					       &nled_tmp_setting);
				return 0;
			} else if ((led_data->cust.mode == MT65XX_LED_MODE_PMIC)
				   && (led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK0
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK1
				       || led_data->cust.data ==
				       MT65XX_LED_PMIC_NLED_ISINK2)) {
				mt_brightness_set_pmic(led_data->cust.data, 0,
						       0);
				return 0;
			} else if (got_wake_lock) {
				wake_unlock(&leds_suspend_lock);
				got_wake_lock = 0;
			}
		}
		return -1;
	}

	/* delay_on and delay_off are not changed */
	return 0;
}

#if defined(CONFIG_MTK_CHARGER_LED_SUPPORT)
void mt_mt65xx_charger_led_on(enum mt65xx_led_type color)
{
        //struct cust_mt65xx_led *cust_led_list = get_cust_led_list();
        struct cust_mt65xx_led *cust_led_list = mt_get_cust_led_list();
	if (color <= 2)
	        mt_mt65xx_led_set_cust(&cust_led_list[color],255);
	else
		LEDS_DEBUG("[LED]Out of color range\n");
}
void mt_mt65xx_charger_led_off(enum mt65xx_led_type color)
{
        //struct cust_mt65xx_led *cust_led_list = get_cust_led_list();
        struct cust_mt65xx_led *cust_led_list = mt_get_cust_led_list();
	if (color <= 2)
        	mt_mt65xx_led_set_cust(&cust_led_list[color],0);
	else
		LEDS_DEBUG("[LED]Out of color range\n");
}

int mt_mt65xx_get_charger_led_state(enum mt65xx_led_type color)
{
       int val;
        //struct cust_mt65xx_led *cust_led_list = get_cust_led_list();
        struct cust_mt65xx_led *cust_led_list = mt_get_cust_led_list();
       if (color <= 2) {
               val = mt_brightness_get_insink_state(cust_led_list[color].data);
       } else {
               LEDS_DEBUG("[LED]Out of color range\n");
               return -1;
       }
       return val;
}
#endif
