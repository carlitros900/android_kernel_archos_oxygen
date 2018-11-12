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
/*#include "ddp_irq.h"*/

#ifdef BUILD_LK

#ifdef GPIO_LCM_PWR
#define GPIO_LCD_PWR      GPIO_LCM_PWR
#else
#define GPIO_LCD_PWR      0xFFFFFFFF
#endif

#else

/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static struct regulator *lcm_vgp1;
static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_RST_EN;

void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm_kd");

	GPIO_LCD_PWR_EN = of_get_named_gpio(node, "lcm_power_gpio", 0);
	GPIO_LCD_RST_EN = of_get_named_gpio(node, "lcm_reset_gpio", 0);
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
}

/* get LDO supply */
static int lcm_get_vgp_supply(struct device *dev)
{
	int ret;
	struct regulator *lcm_vgp_ldo;
    struct regulator *lcm_vgp_ldo1;
	pr_info("LCM: lcm_get_vgp_supply is going\n");

	lcm_vgp_ldo = devm_regulator_get(dev, "reg-lcm");
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
	
	lcm_vgp_ldo1 = devm_regulator_get(dev, "reg1-lcm");
	if (IS_ERR(lcm_vgp_ldo1)) {
		ret = PTR_ERR(lcm_vgp_ldo1);
		dev_err(dev, "failed to get reg-lcm LDO, %d\n", ret);
		return ret;
	}

	pr_info("LCM: lcm get supply ok.\n");

	ret = regulator_enable(lcm_vgp_ldo1);
	/* get current voltage settings */
	ret = regulator_get_voltage(lcm_vgp_ldo1);
	pr_info("lcm LDO voltage = %d in LK stage\n", ret);

	lcm_vgp1 = lcm_vgp_ldo1;

	return ret;
}

int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	pr_info("LCM: lcm_vgp_supply_enable\n");

	if (NULL == lcm_vgp)
		return 0;

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
		pr_err("LCM: check regulator voltage=3200000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=3200000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp);
	if (ret != 0) {
		pr_err("LCM: Failed to enable lcm_vgp: %d\n", ret);
		return ret;
	}

	if (NULL == lcm_vgp1)
		return 0;

	pr_info("LCM: set regulator voltage lcm_vgp voltage to 1.8V\n");
	/* set voltage to 1.8V */
	//ret = regulator_set_voltage(lcm_vgp, 1800000, 1800000);
	ret = regulator_set_voltage(lcm_vgp1, 1800000, 1800000);
	if (ret != 0) {
		pr_err("LCM: lcm failed to set lcm_vgp voltage: %d\n", ret);
		return ret;
	}

	/* get voltage settings again */
	volt = regulator_get_voltage(lcm_vgp1);
	if (volt == 1800000)
		pr_err("LCM: check regulator voltage=3200000 pass!\n");
	else
		pr_err("LCM: check regulator voltage=3200000 fail! (voltage: %d)\n", volt);

	ret = regulator_enable(lcm_vgp1);
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
	
	if (NULL == lcm_vgp1)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp1);

	pr_info("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	if (isenable) {
		ret = regulator_disable(lcm_vgp1);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp1);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}

	return ret;
}

static int lcm_probe(struct device *dev)
{
	lcm_get_vgp_supply(dev);
	lcm_get_gpio_infor();

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
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */

#define FRAME_WIDTH  (1920)
#define FRAME_HEIGHT (1080)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

//static LCM_UTIL_FUNCS lcm_util = {0};  //for fixed warning issue
static LCM_UTIL_FUNCS lcm_util = 
{
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};
typedef struct
{
  unsigned char dev_addr;	
  unsigned char addr;
  unsigned char data;
}it6151_setting_table;

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
#define DP_I2C_ADDR 	(0x5C << 0)
#define MIPI_I2C_ADDR 	(0x6C << 0)
#define REGFLAG_DELAY 	(0xAB)
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_resume_power(void);

static   void lcm_init_power(void)
{
    lcm_resume_power();
	printk("[Kernel/LCM] lcm_init_power() enter\n");
}
static void lcm_suspend_power(void)
{
	printk("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
	MDELAY(20);
	lcm_vgp_supply_disable();
	MDELAY(20);
}

static void lcm_resume_power(void)
{
	printk("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_vgp_supply_enable();
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
	MDELAY(20);
}
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


#ifdef BUILD_LK
#define IT6151_BUSNUM	I2C2

static kal_uint32 it6151_i2c_write_byte(unsigned char dev_addr,unsigned char addr, unsigned char data)
{
  	kal_uint32 ret_code = I2C_OK;
  	unsigned char write_data[I2C_FIFO_SIZE], len;
	struct mt_i2c_t i2c;
	
	i2c.id = IT6151_BUSNUM;
	i2c.addr = dev_addr;
	i2c.mode = ST_MODE;
	i2c.speed = 100;

	write_data[0]= addr;
  	write_data[1] = data;
	len = 2;

	#ifdef IT6151_DEBUG
  /* dump write_data for check */
	printf("[it6151_i2c_write] dev_addr = 0x%x, write_data[0x%x] = 0x%x \n", dev_addr, write_data[0], write_data[1]);
	#endif
	
	ret_code = i2c_write(&i2c, write_data, len);

  	return ret_code;
}

static kal_uint32 it6151_i2c_read_byte(unsigned char dev_addr,unsigned char addr, unsigned char *dataBuffer)
{
    printk("aaaaaaaret\n");
  	kal_uint32 ret_code = I2C_OK;
	unsigned char len;
	struct mt_i2c_t i2c;
	
	*dataBuffer = addr;

	i2c.id = IT6151_BUSNUM;
	i2c.addr = dev_addr;
	i2c.mode = ST_MODE;
	i2c.speed = 100;
	len = 1;
    printk("bbbbret\n");
	ret_code = i2c_write_read(&i2c, dataBuffer, len, len);
    printk("cccret\n");
	#ifdef IT6151_DEBUG
	/* dump write_data for check */
  	printf("[it6151_read_byte] dev_addr = 0x%x, read_data[0x%x] = 0x%x \n", dev_addr, addr, *dataBuffer);
	#endif
	printk("dddret\n");
  	return ret_code;
}
 
 /******************************************************************************
 *IIC drvier,:protocol type 2 add by chenguangjian end
 ******************************************************************************/
#else
extern int it6151_i2c_read_byte(unsigned char dev_addr, unsigned char addr, unsigned char *returnData);
extern int it6151_i2c_write_byte(unsigned char dev_addr, unsigned char addr, unsigned char writeData);
#endif

/////////////////////////////////////////////////////////////////////
///       for it6151 defines start                   ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////


//#define PANEL_RESOLUTION_1280x800_NOUFO
//#define PANEL_RESOLUTION_2048x1536_NOUFO_18B
//#define PANEL_RESOLUTION_2048x1536
// #define PANEL_RESOLUTION_2048x1536_NOUFO // FOR INTEL Platform
// #define PANEL_RESOLUTION_1920x1200p60RB
#define PANEL_RESOLUTION_1920x1080p60
//#define PANEL_RESULUTION_1536x2048
//#define PANEL_RESOLUTION_1366x768

#define MIPI_4_LANE 	(3)
#define MIPI_3_LANE 	(2)
#define MIPI_2_LANE 	(1)
#define MIPI_1_LANE	(0)

// MIPI Packed Pixel Stream
#define RGB_24b         (0x3E)
#define RGB_30b         (0x0D)
#define RGB_36b         (0x1D)
#define RGB_18b_P       (0x1E)
#define RGB_18b_L       (0x2E)
#define YCbCr_16b       (0x2C)
#define YCbCr_20b       (0x0C)
#define YCbCr_24b       (0x1C)

// DPTX reg62[3:0]
#define B_DPTXIN_6Bpp   (0)
#define B_DPTXIN_8Bpp   (1)
#define B_DPTXIN_10Bpp  (2)
#define B_DPTXIN_12Bpp  (3)

#define B_LBR    		(1)
#define B_HBR    		(0)

#define B_4_LANE 		(3)
#define B_2_LANE 		(1)
#define B_1_LANE 		(0)

#define B_SSC_ENABLE   	(1)
#define B_SSC_DISABLE   (0)

///////////////////////////////////////////////////////////////////////////
//CONFIGURE
///////////////////////////////////////////////////////////////////////////
#define TRAINING_BITRATE	(B_HBR)
#define DPTX_SSC_SETTING	(B_SSC_DISABLE)//(B_SSC_ENABLE)//(B_SSC_DISABLE)
#define HIGH_PCLK			(1)
#define MP_MCLK_INV			(1)
#define MP_CONTINUOUS_CLK	(1)
#define MP_LANE_DESKEW		(1)
#define MP_PCLK_DIV			(2)
#define MP_LANE_SWAP		(0)
#define MP_PN_SWAP			(0)

#define DP_PN_SWAP			(0)
#define DP_AUX_PN_SWAP		(0)
#define DP_LANE_SWAP		(0)	//(0) our convert board need to LANE SWAP for data lane
#define FRAME_RESYNC		(0)
#define LVDS_LANE_SWAP		(0)
#define LVDS_PN_SWAP		(0)
#define LVDS_DC_BALANCE		(0)

#define LVDS_6BIT			(0) // '0' for 8 bit, '1' for 6 bit
#define VESA_MAP		    (1) // '0' for JEIDA , '1' for VESA MAP

#define INT_MASK			(3)
#define MIPI_INT_MASK		(0)
#define TIMER_CNT			(0x0A)
///////////////////////////////////////////////////////////////////////
// Global Setting
///////////////////////////////////////////////////////////////////////
#ifdef PANEL_RESOLUTION_1280x800_NOUFO
#define PANEL_WIDTH 1280
#define VIC 0
#define MP_HPOL 0
#define MP_VPOL 1
#define DPTX_LANE_COUNT  B_2_LANE
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT		RGB_24b
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESOLUTION_1920x1080p60
#define PANEL_WIDTH 1920
#define VIC 0x10
#define MP_HPOL 1
#define MP_VPOL 1
#define DPTX_LANE_COUNT  B_2_LANE
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT		RGB_18b_P//RGB_24b
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESOLUTION_1920x1200p60RB
#define PANEL_WIDTH 1920
#define VIC 0 // non-Zero value for CEA setting, check the given input format.
#define MP_HPOL 1
#define MP_VPOL 0
#define DPTX_LANE_COUNT  B_2_LANE
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT		RGB_24b
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESOLUTION_2048x1536
#define PANEL_WIDTH 2048
#define VIC 0 // non-Zero value for CEA setting, check the given input format.
#define MP_HPOL 0
#define MP_VPOL 1
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define DPTX_LANE_COUNT  B_4_LANE
#define EN_UFO 1
#define MIPI_PACKED_FMT		RGB_24b
#define MP_H_RESYNC			0
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESOLUTION_2048x1536_NOUFO
#define PANEL_WIDTH 2048
#define VIC 0 // non-Zero value for CEA setting, check the given input format.
#define MP_HPOL 0
#define MP_VPOL 1
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define DPTX_LANE_COUNT  B_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT		RGB_24b
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESOLUTION_2048x1536_NOUFO_18B
#define PANEL_WIDTH 2048
#define VIC 0 // non-Zero value for CEA setting, check the given input format.
#define MP_HPOL 0
#define MP_VPOL 1
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define DPTX_LANE_COUNT  B_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT		RGB_18b_P
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESULUTION_1536x2048
#define PANEL_WIDTH 1536
#define VIC 0 // non-Zero value for CEA setting, check the given input format.
#define MP_HPOL 0
#define MP_VPOL 1
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define DPTX_LANE_COUNT  B_4_LANE
#define EN_UFO 1
#define MIPI_PACKED_FMT		RGB_24b
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#ifdef PANEL_RESOLUTION_1366x768
#define PANEL_WIDTH 1368
#define VIC 0x10
#define MP_HPOL 1
#define MP_VPOL 1
#define DPTX_LANE_COUNT  B_1_LANE
#define MIPI_LANE_COUNT  MIPI_4_LANE
#define EN_UFO 0
#define MIPI_PACKED_FMT			RGB_24b
#define MP_H_RESYNC			1
#define MP_V_RESYNC			0
#endif

#define MIPI_EVENT_MODE		(0)
#define	MIPI_HSYNC_W		(8)
#define MIPI_VSYNC_W		(2)
///////////////////////////////////////////////////////////////////////////

//#define DP_I2C_ADDR 0x5C
//#define MIPI_I2C_ADDR 0x6C

/////////////////////////////////////////////////////////////////////
///       for it6151 defines end                   /////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// Function
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

void IT6151_DPTX_init(void)
{   
#ifndef BUILD_LK
	//printk("\IT6151_DPTX_init !!!\n");
#else
	//printf("[LK/LCM] IT6151_DPTX_init\n");
#endif	
	it6151_i2c_write_byte(DP_I2C_ADDR,0x05,0x29);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x05,0x00);
	
	it6151_i2c_write_byte(DP_I2C_ADDR,0x09,INT_MASK);// Enable HPD_IRQ,HPD_CHG,VIDSTABLE
	it6151_i2c_write_byte(DP_I2C_ADDR,0x0A,0x00);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x0B,0x00);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xC5,0xC1);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xB5,0x00);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xB7,0x80);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xC4,0xF0);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x06,0xFF);// Clear all interrupt
	it6151_i2c_write_byte(DP_I2C_ADDR,0x07,0xFF);// Clear all interrupt
	it6151_i2c_write_byte(DP_I2C_ADDR,0x08,0xFF);// Clear all interrupt
	
	it6151_i2c_write_byte(DP_I2C_ADDR,0x05,0x00);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x0c,0x08);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x21,0x05);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x3a,0x04);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x5f,0x06);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xc9,0xf5);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xca,0x4c);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xcb,0x37);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xce,0x80);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xd3,0x03);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xd4,0x60);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xe8,0x11);
	it6151_i2c_write_byte(DP_I2C_ADDR,0xec,VIC);
	MDELAY(5);			

	it6151_i2c_write_byte(DP_I2C_ADDR,0x23,0x42);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x24,0x07);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x25,0x01);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x26,0x00);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x27,0x10);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x2B,0x05);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x23,0x40);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x22,(DP_AUX_PN_SWAP<<3)|(DP_PN_SWAP<<2)|0x03);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x16,(DPTX_SSC_SETTING<<4)|(DP_LANE_SWAP<<3)|(DPTX_LANE_COUNT<<1)|TRAINING_BITRATE);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x0f,0x01);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x76,0xa7);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x77,0xaf);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x7e,0x8f);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x7f,0x07);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x80,0xef);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x81,0x5f);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x82,0xef);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x83,0x07);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x88,0x38);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x89,0x1f);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x8a,0x48);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x0f,0x00);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x5c,0xf3);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x17,0x04);
	it6151_i2c_write_byte(DP_I2C_ADDR,0x17,0x01);
	MDELAY(5);	
}

int	IT6151_init(void)
{
	unsigned char VenID[2] = {0};
	unsigned char DevID[2] = {0}; 
	unsigned char RevID = {0};
	//unsigned char cmdBuffer;
	
	int ret = -1;
#ifndef BUILD_LK
	printk("IT6151_init\n");
#else
	printf("[LCM/lk]IT6151_init\n");
#endif
    printk("00000000000ret\n");
	ret = it6151_i2c_read_byte(DP_I2C_ADDR, 0x00, &VenID[0]);
	printk("11111111111ret\n");
	printk("11111111111ret=%d\n",ret);
	if(ret == 0)
#ifndef BUILD_LK
		printk("it6151_i2c_read_byte = 0,failed\n");
#else
		printf("[LCM/LK]it6151_i2c_read_byte = 0,failed\n");
#endif
	it6151_i2c_read_byte(DP_I2C_ADDR, 0x01, &VenID[1]);
	it6151_i2c_read_byte(DP_I2C_ADDR, 0x02, &DevID[0]);
	it6151_i2c_read_byte(DP_I2C_ADDR, 0x03, &DevID[1]);
	it6151_i2c_read_byte(DP_I2C_ADDR, 0x04, &RevID);	
				
#ifndef BUILD_LK	
	printk("Current DPDevID=%02X%02X\n", DevID[1], DevID[0]);
	printk("Current DPVenID=%02X%02X\n", VenID[1], VenID[0]);
	printk("Current DPRevID=%02X\n\n", RevID);	
#else
	printf(" lk Current DPDevID=%02X%02X\n", DevID[1], DevID[0]);
	printf(" lk Current DPVenID=%02X%02X\n", VenID[1], VenID[0]);
	printf(" lk Current DPRevID=%02X\n\n", RevID);	
#endif
				
	if( VenID[0]==0x54 && VenID[1]==0x49 && DevID[0]==0x51 && DevID[1]==0x61 ){

#ifndef BUILD_LK	
		printk(" Test 1 DP_I2C_ADDR=0x%x, MIPI_I2C_ADDR=0x%x\n", DP_I2C_ADDR, MIPI_I2C_ADDR);
#else
		printf("[LK/LCM] Test 1 DP_I2C_ADDR=0x%x, MIPI_I2C_ADDR=0x%x\n", DP_I2C_ADDR, MIPI_I2C_ADDR);
#endif
		it6151_i2c_write_byte(DP_I2C_ADDR,0x05,0x04);// DP SW Reset
		it6151_i2c_write_byte(DP_I2C_ADDR,0xfd,(MIPI_I2C_ADDR<<1)|1);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x05,0x00);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x0c,(MP_LANE_SWAP<<7)|(MP_PN_SWAP<<6)|(MIPI_LANE_COUNT<<4)|EN_UFO);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x11,MP_MCLK_INV);

        if(RevID == 0xA1){			
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x19, MP_LANE_DESKEW); 
		}else{
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x19,(MP_CONTINUOUS_CLK<<1) | MP_LANE_DESKEW); 
   		}
				
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x27, MIPI_PACKED_FMT);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x28,((PANEL_WIDTH/4-1)>>2)&0xC0);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x29,(PANEL_WIDTH/4-1)&0xFF);
		
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x2e,0x34);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x2f,0x01);
		
		
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x4e,(MP_V_RESYNC<<3)|(MP_H_RESYNC<<2)|(MP_VPOL<<1)|(MP_HPOL));
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x80,(EN_UFO<<5)|MP_PCLK_DIV);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x84,0x8f);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x09,MIPI_INT_MASK);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x92,TIMER_CNT);	

		#if (MIPI_EVENT_MODE == 1)
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x33,0x80 | MIPI_HSYNC_W >> 8);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x32,MIPI_HSYNC_W & 0xFF);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x3D,0x80 | MIPI_VSYNC_W >> 8);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x3C,MIPI_VSYNC_W & 0xFF);
		#endif	

		IT6151_DPTX_init();

		return 0;
	}

#ifndef BUILD_LK	
	printk(" Test 2 DP_I2C_ADDR=0x%x, MIPI_I2C_ADDR=0x%x\n", DP_I2C_ADDR, MIPI_I2C_ADDR);
#endif

	it6151_i2c_read_byte(MIPI_I2C_ADDR, 0x00, &VenID[0]);
	it6151_i2c_read_byte(MIPI_I2C_ADDR, 0x01, &VenID[1]);
	it6151_i2c_read_byte(MIPI_I2C_ADDR, 0x02, &DevID[0]);
	it6151_i2c_read_byte(MIPI_I2C_ADDR, 0x03, &DevID[1]);
	it6151_i2c_read_byte(MIPI_I2C_ADDR, 0x04, &RevID);

#ifndef BUILD_LK
	printk("Current MPDevID=%02X%02X\n", DevID[1], DevID[0]);
	printk("Current MPVenID=%02X%02X\n", VenID[1], VenID[0]);
	printk("Current MPRevID=%02X\n\n", RevID);
#else
	printf("[lk]Current MPDevID=%02X%02X\n", DevID[1], DevID[0]);
	printf("[lk]Current MPVenID=%02X%02X\n", VenID[1], VenID[0]);
	printf("[lk]Current MPRevID=%02X\n\n", RevID);
#endif
	if( VenID[0]==0x54 && VenID[1]==0x49 && DevID[0]==0x51 && DevID[1]==0x61 ){
	
#ifndef BUILD_LK	
			printk(" Test 1 DP_I2C_ADDR=0x%x, MIPI_I2C_ADDR=0x%x\n", DP_I2C_ADDR, MIPI_I2C_ADDR);
#else
			printf("[LK/LCM] Test 1 DP_I2C_ADDR=0x%x, MIPI_I2C_ADDR=0x%x\n", DP_I2C_ADDR, MIPI_I2C_ADDR);
#endif
			it6151_i2c_write_byte(DP_I2C_ADDR,0x05,0x04);// DP SW Reset
			it6151_i2c_write_byte(DP_I2C_ADDR,0xfd,(MIPI_I2C_ADDR<<1)|1);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x05,0x00);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x0c,(MP_LANE_SWAP<<7)|(MP_PN_SWAP<<6)|(MIPI_LANE_COUNT<<4)|EN_UFO);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x11,MP_MCLK_INV);
	
			if(RevID == 0xA1){			
				it6151_i2c_write_byte(MIPI_I2C_ADDR,0x19, MP_LANE_DESKEW); 
			}else{
				it6151_i2c_write_byte(MIPI_I2C_ADDR,0x19,(MP_CONTINUOUS_CLK<<1) | MP_LANE_DESKEW); 
			}
					
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x27, MIPI_PACKED_FMT);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x28,((PANEL_WIDTH/4-1)>>2)&0xC0);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x29,(PANEL_WIDTH/4-1)&0xFF);
			
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x2e,0x34);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x2f,0x01);
			
			
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x4e,(MP_V_RESYNC<<3)|(MP_H_RESYNC<<2)|(MP_VPOL<<1)|(MP_HPOL));
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x80,(EN_UFO<<5)|MP_PCLK_DIV);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x84,0x8f);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x09,MIPI_INT_MASK);
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x92,TIMER_CNT);

			#if (MIPI_EVENT_MODE == 1)
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x33,0x80 | MIPI_HSYNC_W >> 8);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x32,MIPI_HSYNC_W & 0xFF);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x3D,0x80 | MIPI_VSYNC_W >> 8);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x3C,MIPI_VSYNC_W & 0xFF);
		#endif			
			IT6151_DPTX_init();
	
			return 0;
		}

	if(VenID[0]==0x54 && VenID[1]==0x49 && DevID[0]==0x21 && DevID[1]==0x61 ){
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x05,0x33);
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x05,0x40);
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x05,0x00);
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x0c,(MP_LANE_SWAP<<7)|(MP_PN_SWAP<<6)|(MIPI_LANE_COUNT<<4));
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x11, MP_MCLK_INV); 
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x19,(MP_CONTINUOUS_CLK<<1) | MP_LANE_DESKEW);  
			it6151_i2c_write_byte(MIPI_I2C_ADDR,0x4B,(FRAME_RESYNC<<4));
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x4E,(MP_V_RESYNC<<3)|(MP_H_RESYNC<<2)|(MP_VPOL<<1)|(MP_HPOL));      
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x72,0x01); 
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x73,0x03); 
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0x80,MP_PCLK_DIV); 
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0xC0,(HIGH_PCLK<< 4) | 0x0F);   
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0xC1,0x01);  
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0xC2,0x47);  
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0xC3,0x67);  
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0xC4,0x04);  
		    it6151_i2c_write_byte(MIPI_I2C_ADDR,0xCB,(LVDS_PN_SWAP<<5)|(LVDS_LANE_SWAP<<4)|(LVDS_6BIT<<2)|(LVDS_DC_BALANCE<<1)| VESA_MAP);  

			#if (MIPI_EVENT_MODE == 1)
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x33,0x80 | MIPI_HSYNC_W >> 8);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x32,MIPI_HSYNC_W & 0xFF);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x3D,0x80 | MIPI_VSYNC_W >> 8);
		it6151_i2c_write_byte(MIPI_I2C_ADDR,0x3C,MIPI_VSYNC_W & 0xFF);
		#endif	
			return 1;
  }	
	return -1;
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if 0
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE; //SYNC_EVENT_VDO_MODE;
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		//params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		//params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		//params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB666;//LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		//params->dsi.packet_size=256;

		// Video mode setting		
		//params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_18BIT_RGB666;//LCM_PACKED_PS_24BIT_RGB888;//LCM_LOOSELY_PS_18BIT_RGB666;//LCM_PACKED_PS_18BIT_RGB666;
		//params->dsi.word_count=1920*3; //720*3;	

		
		params->dsi.vertical_sync_active				= 6; //4; //6; //(12-4-4); //1;
		params->dsi.vertical_backporch					= 16;//8; //6; //4; //6; //10;
		params->dsi.vertical_frontporch					= 14;//8;//6; //4//6; //10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 40;//4; //40; //44; //(120-40-40); //1;
		params->dsi.horizontal_backporch				= 31;//60;//43;//44; //40; //44; //57;
		params->dsi.horizontal_frontporch				= 31;//60;//45;//108;//44; //40; //44; //32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_div1=1; //0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =22;//31;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
		//params->dsi.fbk_sel =0;
		//params->dsi.fbk_div =45;
        	params->dsi.PLL_CLOCK = 440;//180;//216;//240;//LCM_DSI_6589_PLL_CLOCK_NULL; //LCM_DSI_6589_PLL_CLOCK_396_5;
    
		//params->dsi.CLK_ZERO = 262; //47;
		//params->dsi.HS_ZERO = 117; //36;

		params->dsi.cont_clock = 1;
		//params->dsi.noncont_clock = TRUE; 
		//params->dsi.noncont_clock_period = 2; // Unit : frames

		params->dsi.ssc_disable = 1;
#if 0
		params->dsi.HS_TRAIL = 0x8;
		params->dsi.HS_ZERO = 0xA;
		params->dsi.HS_PRPR = 0x6;
		params->dsi.LPX = 0x5;
		params->dsi.DA_HS_EXIT = 0x8;
#endif

}
extern void DSI_clk_HS_mode(unsigned char enter);
static void lcm_init_lcm(void)
{
	//unsigned int data_array[16];
	
	printk("=====>KD101N37_40NA_A5_REVA_MIPI %s=====>\n", __func__);
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_init() enter\n");
	printf("[LK/LCM] lcm_init() hengqiu add test enter\n");
	upmu_set_strup_ext_pmic_en(0x1);
    upmu_set_vpa_vosel(0x38);
    upmu_set_vpa_en(1);
	printf("[LK/LCM] lcm_init() hengqiu set 3.3v en\n");
	/*
#if defined(CONFIG_MTK_DC_DET_VIA_ADC) || defined(CONFIG_MTK_DC_DET_VIA_GPIO)
{
	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
#define ALCO_POGO_GPIO_EN (GPIO29 | 0x80000000)
	mt_set_gpio_mode(ALCO_POGO_GPIO_EN, GPIO_MODE_00);
        mt_set_gpio_dir(ALCO_POGO_GPIO_EN,GPIO_DIR_OUT);
        mt_set_gpio_out(ALCO_POGO_GPIO_EN, GPIO_OUT_ONE);
}
#endif
*/
	MDELAY(20);
#else
    printk("[LCM] lcm_init() enter\n");
/*
{ //add pogo enable start
#define ALCO_POGO_GPIO_EN (GPIO29 | 0x80000000)
	mt_set_gpio_mode(ALCO_POGO_GPIO_EN, GPIO_MODE_00);
        mt_set_gpio_dir(ALCO_POGO_GPIO_EN,GPIO_DIR_OUT);
        mt_set_gpio_out(ALCO_POGO_GPIO_EN, GPIO_OUT_ONE);
} //add pogo enable end
*/
#endif

	//mt_set_gpio_mode(GPIO_LCD_LCM_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_LCM_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_LCM_EN, GPIO_OUT_ONE);
	//MDELAY(20);

	//mt_set_gpio_mode(GPIO_LCD_LED_PWM_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_LED_PWM_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_LED_PWM_EN, GPIO_OUT_ONE);
}



static void lcm_suspend(void)
{
	//standby mode
	//unsigned char temp;
	//it6151_i2c_read_byte(DP_I2C_ADDR, 0x22, &temp);
	//temp = temp | 0x10;
	//it6151_i2c_write_byte(DP_I2C_ADDR, 0x22, temp);
	//unsigned int data_array[16];

	//data_array[0]=0x00280500; // Display Off
	//dsi_set_cmdq(data_array, 1, 1);
	#if 1
//	mt_set_gpio_mode(GPIO_LCD_LED_PWM_EN, GPIO_MODE_00);
//	mt_set_gpio_dir(GPIO_LCD_LED_PWM_EN, GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_LCD_LED_PWM_EN, GPIO_OUT_ZERO);
	#endif
//	MDELAY(20);
	//
//	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
//	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ZERO);
	MDELAY(20);
	/*
{ //add pogo disable start
#define ALCO_POGO_GPIO_EN (GPIO29 | 0x80000000)
	mt_set_gpio_mode(ALCO_POGO_GPIO_EN, GPIO_MODE_00);
        mt_set_gpio_dir(ALCO_POGO_GPIO_EN,GPIO_DIR_OUT);
        mt_set_gpio_out(ALCO_POGO_GPIO_EN, GPIO_OUT_ZERO);
} // add pogo disable end	
*/
#ifndef BUILD_LK
    printk("[LCM] lcm_suspend() enter\n");
#endif
}
static void lcm_resume(void)
{

    //MDELAY(30);	
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_resume() enter\n");
	//mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	upmu_set_rg_vgp1_vosel(0x7);
    upmu_set_rg_vgp1_en(0x1);
	#if 1
	mt_set_gpio_mode(GPIO_LCD_LED_PWM_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_PWM_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_PWM_EN, GPIO_OUT_ONE);
	#endif
	MDELAY(20);
	MDELAY(20);
		
	//mt_set_gpio_mode(GPIO_LCD_STB_EN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_STB_EN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_LCD_STB_EN, GPIO_OUT_ONE);
	mt_set_gpio_mode(GPIO_LCD_LED_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_LED_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
	MDELAY(20);

#else
     printk("[LCM] lcm_resume() enter\n");
    //upmu_set_vpa_vosel(0x38);
    //upmu_set_vpa_en(1);
    //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3300, "LCM");
	//MDELAY(100);
#endif
//	lcm_init_power();
	//unsigned int data_array[16];
    lcm_init_lcm();

	//data_array[0] = 0x00101500; // Sleep Out
	//dsi_set_cmdq(data_array, 1, 1);
//	MDELAY(10);

	//data_array[0] = 0x00290500; // Display On
	//dsi_set_cmdq(data_array, 1, 1);

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

LCM_DRIVER hn116wx1_100_dsi_edp_A15_A17_lcm_drv = 
{
    .name			= "hn116wx1_100_dsi_edp_A15_A17",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init = lcm_init_lcm,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
    };
