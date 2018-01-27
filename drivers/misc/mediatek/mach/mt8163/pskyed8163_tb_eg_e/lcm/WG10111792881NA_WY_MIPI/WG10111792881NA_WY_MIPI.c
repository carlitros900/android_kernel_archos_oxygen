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

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif

#include "lcm_drv.h"

/*static unsigned int GPIO_LCD_PWR_EN;*/
static struct regulator *lcm_vgp;
static unsigned int GPIO_LCD_PWR_EN;
static unsigned int GPIO_LCD_RST_EN;

static void lcm_get_gpio_infor(void)
{
	static struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm_wy");

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

	pr_debug("LCM: lcm_get_vgp_supply is going\n");

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

static int lcm_vgp_supply_enable(void)
{
	int ret;
	unsigned int volt;

	printk("=====%s %s %d=====\n",__FILE__, __func__, __LINE__);
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

static int lcm_vgp_supply_disable(void)
{
	int ret = 0;
	unsigned int isenable;

	printk("===== %s %s %d=====\n",__FILE__,__func__, __LINE__);
	if (NULL == lcm_vgp)
		return 0;

	/* disable regulator */
	isenable = regulator_is_enabled(lcm_vgp);

	pr_debug("LCM: lcm query regulator enable status[0x%d]\n", isenable);

	do {
		ret = regulator_disable(lcm_vgp);
		if (ret != 0) {
			pr_err("LCM: lcm failed to disable lcm_vgp: %d\n", ret);
			return ret;
		}
		/* verify */
		isenable = regulator_is_enabled(lcm_vgp);
		if (!isenable)
			pr_err("LCM: lcm regulator disable pass\n");
	}while(isenable);

	return ret;
}

static int lcm_probe(struct device *dev)
{
	printk("=====%s %s %d=====\n",__FILE__,__func__, __LINE__);
	lcm_get_vgp_supply(dev);
	lcm_get_gpio_infor();

	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm_wy",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "lcm_wy",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
#ifdef CONFIG_OF
		   .of_match_table = lcm_of_ids,
#endif
		   },
};

static int __init lcm_init(void)
{
	printk("=====>%s %s %d=====>\n",__FILE__,__func__,__LINE__);
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}
		pr_notice("LCM: Register lcm driver success\n");
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
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = 
{
	.set_reset_pin = NULL,
	.udelay = NULL,
	.mdelay = NULL,
};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define UDELAY(n) (lcm_util.udelay(n))
//============================================================================
#define MDELAY(n) (lcm_util.mdelay(n))
//============================================================================
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
//============================================================================
#define wrtie_cmd(cmd)	\
		lcm_util.dsi_write_cmd(cmd)
//============================================================================
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//============================================================================
#define read_reg(cmd) \
		lcm_util.dsi_dcs_read_lcm_reg(cmd)
//============================================================================
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   
//============================================================================
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
//============================================================================
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
//============================================================================

#define REGFLAG_DELAY		(0xFC)
#define REGFLAG_END_OF_TABLE    (0xFD)

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[128];
};

#define LCM_DSI_CMD_MODE		0

static void lcm_init_power(void)
{
	pr_debug("[Kernel/LCM] lcm_init_power() enter\n");
}

static void lcm_suspend_power(void)
{
	pr_debug("[Kernel/LCM] lcm_suspend_power() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 0);
	MDELAY(20);
	lcm_vgp_supply_disable();
	MDELAY(20);
}

static void lcm_resume_power(void)
{
	pr_debug("[Kernel/LCM] lcm_resume_power() enter\n");
	lcm_vgp_supply_enable();
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
	MDELAY(20);
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		printk("=====>%s %s=====>\n",__FILE__,__func__);

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;
		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		#if 0
		// enable tearing-free
		params->dbi.te_mode 		= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity	= LCM_POLARITY_RISING;
		#endif

	        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        	#else
		params->dsi.mode   = BURST_VDO_MODE;
		//SYNC_PULSE_VDO_MODE; //BURST_VDO_MODE; //SYNC_EVENT_VDO_MODE;
        	#endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		//params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		//params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		//params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		//params->dsi.packet_size=256;

		// Video mode setting		
		//params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		//params->dsi.word_count=FRAME_WIDTH*3; //720*3;	

		
		params->dsi.vertical_sync_active	= 10;
		params->dsi.vertical_backporch		= 18;
		params->dsi.vertical_frontporch		= 18;
		params->dsi.vertical_active_line	= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active	= 4;
		params->dsi.horizontal_backporch	= 8;
		params->dsi.horizontal_frontporch	= 12;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

        	params->dsi.PLL_CLOCK = 205;
		 params->dsi.cont_clock = 0;
}

/****************************************************************/
//update initial param for IC boe_nt35521 0.01
static struct LCM_setting_table lcm_initialization_setting[] = 
{
    {0xFF,4,{0xAA,0x55,0xA5,0x80}},
    
    {0x6F,2,{0x11,0x00}},
    {0xF7,2,{0x20,0x00}},	
    
    {0x6F,1,{0x06}},	
    {0xF7,1,{0xA0}},	
    {0x6F,1,{0x19}},	
    {0xF7,1,{0x12}},	
    {0xF4,1,{0x03}},	
    // new Vcom floating 	
    //{0x6F,1,{0x08}},	
    //{0xFA,1,{0x40}},	
    //{0x6F,1,{0x11}},	
    //{0xF3,1,{0x01}},	
   
    //========== page0 relative ==========
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},	
    {0xB1,2,{0x68,0x01}},	
    {0xB6,1,{0x08}},	
    {0x6F,1,{0x02}},	
    {0xB8,1,{0x08}},	
    {0xBB,2,{0x54,0x44}},	
    {0xBC,2,{0x05,0x05}},
    {0xC7,1,{0x01}},	
    {0xBD,5,{0x02,0xB0,0x1E,0x1E,0x00}},
    {0xC5,2,{0x01,0x07}},	
    {0xC8,1,{0x80}},	
    //========== page1 relative ==========
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},	
    {0xB0,2,{0x05,0x05}},
    {0xB1,2,{0x05,0x05}},	
    {0xB2,2,{0x00,0x00}},	
    {0xBC,2,{0x90,0x01}},	
    {0xBD,2,{0x90,0x01}},	
    {0xCA,1,{0x00}},	
    {0xC0,1,{0x04}},	
    {0xBE,1,{0x29}},	
    {0xB3,2,{0x28,0x28}},	
    {0xB4,2,{0x12,0x12}},	
    {0xB9,2,{0x35,0x35}},	
    {0xBA,2,{0x25,0x25}},	
    
    //========== page2 relative ==========//gamma setting
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},	
    {0xEE,1,{0x01}},	
    {0xEF,4,{0x09,0x06,0x15,0x18}},	
    {0xB0,6,{0x00,0x00,0x00,0x24,0x00,0x55}},	
    {0x6F,1,{0x06}},	
    {0xB0,6,{0x00,0x77,0x00,0x94,0x00,0xC0}},	
    {0x6F,1,{0x0C}},	
    {0xB0,4,{0x00,0xE3,0x01,0x1A}},
    {0xB1,6,{0x01,0x46,0x01,0x88,0x01,0xBC}},	
    {0x6F,1,{0x06}},	
    {0xB1,6,{0x02,0x0B,0x02,0x4B,0x02,0x4D}},	
    {0x6F,1,{0x0C}},	
    {0xB1,4,{0x02,0x88,0x02,0xC9}},	
    {0xB2,6,{0x02,0xF3,0x03,0x29,0x03,0x4E}},	
    {0x6F,1,{0x06}},	
    {0xB2,6,{0x03,0x7D,0x03,0x9B,0x03,0xBE}},	
    {0x6F,1,{0x0C}},	
    {0xB2,4,{0x03,0xD3,0x03,0xE9}},	
    {0xB3,4,{0x03,0xFB,0x03,0xFF}},  		
    
    //========== page6 GOA relative ==========
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},	
    {0xB0,2,{0x0B,0x2E}},	
    {0xB1,2,{0x2E,0x2E}},	
    {0xB2,2,{0x2E,0x09}},	
    {0xB3,2,{0x2A,0x29}},	
    {0xB4,2,{0x1B,0x19}},	
    {0xB5,2,{0x17,0x15}},	
    {0xB6,2,{0x13,0x11}},	
    {0xB7,2,{0x01,0x2E}},	
    {0xB8,2,{0x2E,0x2E}},	
    {0xB9,2,{0x2E,0x2E}},	
    {0xBA,2,{0x2E,0x2E}},	
    {0xBB,2,{0x2E,0x2E}},	
    {0xBC,2,{0x2E,0x00}},	
    {0xBD,2,{0x10,0x12}},	
    {0xBE,2,{0x14,0x16}},	
    {0xBF,2,{0x18,0x1A}},	
    {0xC0,2,{0x29,0x2A}},	
    {0xC1,2,{0x08,0x2E}},	
    {0xC2,2,{0x2E,0x2E}},	
    {0xC3,2,{0x2E,0x0A}},	
    {0xE5,2,{0x2E,0x2E}},	
    {0xC4,2,{0x0A,0x2E}},	
    {0xC5,2,{0x2E,0x2E}},	
    {0xC6,2,{0x2E,0x00}},	
    {0xC7,2,{0x2A,0x29}},	
    {0xC8,2,{0x10,0x12}},	
    {0xC9,2,{0x14,0x16}},	
    {0xCA,2,{0x18,0x1A}},	
    {0xCB,2,{0x08,0x2E}},	
    {0xCC,2,{0x2E,0x2E}},	
    {0xCD,2,{0x2E,0x2E}},	
    {0xCE,2,{0x2E,0x2E}},	
    {0xCF,2,{0x2E,0x2E}},	
    {0xD0,2,{0x2E,0x09}},	
    {0xD1,2,{0x1B,0x19}},	
    {0xD2,2,{0x17,0x15}},	
    {0xD3,2,{0x13,0x11}},	
    {0xD4,2,{0x29,0x2A}},	
    {0xD5,2,{0x01,0x2E}},	
    {0xD6,2,{0x2E,0x2E}},	
    {0xD7,2,{0x2E,0x0B}},	
    {0xE6,2,{0x2E,0x2E}},	
    {0xD8,5,{0x00,0x00,0x00,0x00,0x00}},	
    {0xD9,5,{0x00,0x00,0x00,0x00,0x00}},	
    {0xE7,1,{0x00}},	
    
    //========== page3 relative ==========
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},	
    {0xB0,2,{0x20,0x00}},	
    {0xB1,2,{0x20,0x00}},	
    {0xB2,5,{0x05,0x00,0x00,0x00,0x00}},	
    {0xB6,5,{0x05,0x00,0x00,0x00,0x00}},	
    {0xB7,5,{0x05,0x00,0x00,0x00,0x00}},	
    {0xBA,5,{0x57,0x00,0x00,0x00,0x00}},	
    {0xBB,5,{0x57,0x00,0x00,0x00,0x00}},	
    {0xC0,4,{0x00,0x00,0x00,0x00}},
    {0xC1,4,{0x00,0x00,0x00,0x00}},	
    {0xC4,1,{0x60}},	
    {0xC5,1,{0x40}},	
    
    //========== page5 relative ==========
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},	
    {0xBD,5,{0x03,0x01,0x03,0x03,0x03}},	
    {0xB0,2,{0x17,0x06}},	
    {0xB1,2,{0x17,0x06}},	
    {0xB2,2,{0x17,0x06}},	
    {0xB3,2,{0x17,0x06}},	
    {0xB4,2,{0x17,0x06}},	
    {0xB5,2,{0x17,0x06}},	
    {0xB8,1,{0x00}},	
    {0xB9,1,{0x00}},	
    {0xBA,1,{0x00}},	
    {0xBB,1,{0x02}},	
    {0xBC,1,{0x00}},	
    {0xC0,1,{0x07}},	
    {0xC4,1,{0x80}},	
    {0xC5,1,{0xA4}},	
    {0xC8,2,{0x05,0x30}},	
    {0xC9,2,{0x01,0x31}},	
    {0xCC,3,{0x00,0x00,0x3C}},	
    {0xCD,3,{0x00,0x00,0x3C}},	
    {0xD1,5,{0x00,0x05,0x09,0x07,0x10}},	
    {0xD2,5,{0x00,0x05,0x0E,0x07,0x10}},	
    {0xE5,1,{0x06}},	
    {0xE6,1,{0x06}},	
    {0xE7,1,{0x06}},	
    {0xE8,1,{0x06}},	
    {0xE9,1,{0x06}},	
    {0xEA,1,{0x06}},	
    {0xED,1,{0x30}},	
    {0x6F,1,{0x11}},	
    {0xF3,1,{0x01}},	
    {0x35,1,{0x00}},
    
    //BIST mode
    //{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    //{0xEF,2,{0x07,0xFF}},
    //{0xEE,4,{0x87,0x78,0x02,0x40}},
    
    //==========================================================================//	
    {0x11,	1,	{0x00}},
    {REGFLAG_DELAY, 20, {}},
    
    {0x29,	1,	{0x00}},
    {REGFLAG_DELAY, 20, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
   	unsigned int i;
       
   
    //printf("=====>%s %d count:%d=====>\n",__func__, __LINE__,count);
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
	//printf("=====>%s %d cmd:0x%X=====>\n",__func__, __LINE__,cmd);

        switch (cmd) {
            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_init_lcm(void)
{
	//unsigned int data_array[16];
	printk("=====>%s %s=====>\n",__FILE__,__func__);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
	MDELAY(20);
}

static void lcm_suspend(void)
{
	printk("=====>%s %s=====>\n",__FILE__,__func__);
}

static void lcm_resume(void)
{
	printk("=====>%s %s=====>\n",__FILE__,__func__);

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

#if 0
static struct LCM_setting_table lcm_readid_setting[] = {
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},	
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};

static unsigned int lcm_compare_id(void)
{
#define	NT35521S_ID	(0x5521) 
	unsigned int lcm_id = 0;
	unsigned char buffer[4]= {0};

	printk("=====>%s %s=====>\n",__FILE__,__func__);

	upmu_set_strup_ext_pmic_en(0x1);
    	upmu_set_vpa_vosel(0x38);
    	upmu_set_vpa_en(1);
	MDELAY(30);

	push_table(lcm_readid_setting, sizeof(lcm_readid_setting) / sizeof(struct LCM_setting_table), 1);  
	read_reg_v2(0xC5,buffer,4);
	MDELAY(10);

	lcm_id = (buffer[1] << 8) | buffer[3];
//	printf("=====>%s %d id0=0x%X,id1=0x%X,id3=0x%X,id4=0x%X=====>\n",__func__,__LINE__,buffer[0],buffer[1],buffer[2],buffer[3]);
	printk("=====>NOW LCM ID is 0x%X=====>\n",lcm_id);
	if (NT35521S_ID == lcm_id) {
		printk("=====>lcm_id is correct=====>\n");
		return 1;
	} else {
		printk("=====>lcm_id is wrong=====>\n");
		return  0;	
	}
	//return (NT35521S_ID == lcm_id)?1:0;
}
#endif

LCM_DRIVER WG10111792881NA_WY_MIPI_drv = 
{
    .name			= "WG10111792881NA_WY_MIPI",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init 		= lcm_init_lcm,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power = lcm_init_power,
//	.compare_id	= lcm_compare_id,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
