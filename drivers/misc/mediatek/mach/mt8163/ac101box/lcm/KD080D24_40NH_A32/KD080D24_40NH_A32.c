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
#define dsi_set_cmdq_V3(para_tbl,size,force_update)       \
        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)  
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
	MDELAY(200);
//	lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
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
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
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

		
		params->dsi.vertical_sync_active	= 2;
		params->dsi.vertical_backporch		= 16;
		params->dsi.vertical_frontporch		= 9;
		params->dsi.vertical_active_line	= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active	= 20;
		params->dsi.horizontal_backporch	= 64;
		params->dsi.horizontal_frontporch	= 70;
		params->dsi.horizontal_active_pixel	= FRAME_WIDTH;

        params->dsi.PLL_CLOCK = 225;
}

/****************************************************************/
//update initial param for IC boe_nt35521 0.01
#if 0
static LCM_setting_table_V3 lcm_initialization_setting[] = 
{
    {0x39,0xB9,3, {0xFF,0x83,0x94}},
	{0x15,0xBA,2, {0x73,0x83}},
	{0x39,0xB1,15,{0x6c,0x15,0x15,0x24,0xE4,0x11,0xf1,0x80,0xe4,0xd7,0x23,0x80,0xc0,
                   0xd2,0x58}},
	{0x39,0xB2,11,{0x00,0x64,0x10,0x07,0x20,0x1C,0x08,0x08,0x1c,0x4d,0x00}},
	{0x39,0xB4,12,{0x00,0xff,0x03,0x5A,0x03,0x5A,0x03,0x5A,0x01,0x6a,0x01,0x6a}},
	{0x15,0xB6,2, {0x60,0x60}},
	{0x05,0xCC,1, {0x09}},
	{0x39,0xD3,30,{0x00,0x06,0x00,0x40,0x1A,0x08,0x00,0x32,0x10,0x07,0x00,0x07,0x54,
                   0x15,0x0f,0x05,0x04,0x02,0x12,0x10,0x05,0x07,0x33,0x33,0x0B,0x0B,
                   0x37,0x10,0x07,0x07}},
	{0x39,0xD5,44,{0x19,0x19,0x18,0x18,0x1A,0x1A,0x1B,0x1B,0x04,0x05,0x06,0x07,0x00,
                   0x01,0x02,0x03,0x20,0x21,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
                   0x18,0x18,0x18,0x18,0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
                   0x18,0x18,0x18,0x18,0x18}},
    {0x39,0xD6,44, {0x18,0x18,0x19,0x19,0x1A,0x1A,0x1B,0x1B,0x03,0x02,0x01,0x00,0x07,
                    0x06,0x05,0x04,0x23,0x22,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
                    0x18,0x18,0x18,0x18,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
                    0x18,0x18,0x18,0x18,0x18}},
	{0x39,0xE0,42,{0x00,0x06,0x0C,0x31,0x34,0x3F,0x1D,0x41,0x06,0x0A,0x0C,0x17,0x0F,
                   0x12,0x15,0x13,0x14,0x07,0x12,0x15,0x16,0x00,0x06,0x0B,0x30,0x34,
                   0x3F,0x1D,0x40,0x07,0x0A,0x0D,0x18,0x0E,0x12,0x14,0x12,0x14,0x08,
                   0x13,0x14,0x19}},
    {0x05,0xD2,1, {0x55}},
    {0x15,0xC0,2, {0x30,0x14}},
	{0x39,0xBF,3, {0x41,0x0E,0x01}},
	{0x39,0xC7,4, {0x00,0xC0,0x40,0xC0}},
	{0x05,0xDF,1, {0x8E}},
	{0x05,0x11,1, {0x00}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 150, {}},//{REGFLAG_DELAY,200,{}},
	{0x05,0x29,1, {0x00}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3, 200, {}},//{REGFLAG_DELAY,50,{}}, 
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
#endif

static void lcm_init_lcm(void)
{
	unsigned int data_array[16];
	printk("=====>%s %s=====>\n",__FILE__,__func__);
    
    data_array[0]= 0x00043902;
	data_array[1]= 0x9483ffb9;
	dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]= 0x00033902;
	data_array[1]= 0x008373ba;
	dsi_set_cmdq(data_array, 2, 1);
                   
    data_array[0]= 0x00103902;
	data_array[1]= 0x15156cb1;
	data_array[2]= 0xf111e424;
	data_array[3]= 0x23d7e480;
	data_array[4]= 0x58d2c080;
	dsi_set_cmdq(data_array, 5, 1);                   
    
    data_array[0]= 0x000c3902;
	data_array[1]= 0x106400b2;
	data_array[2]= 0x081c2007;
	data_array[3]= 0x004d1c08;
	dsi_set_cmdq(data_array, 4, 1);      
    
    data_array[0]= 0x000d3902;
	data_array[1]= 0x03ff00b4;
	data_array[2]= 0x035a035a;
	data_array[3]= 0x016a015a;
	data_array[4]= 0x0000006a;
	dsi_set_cmdq(data_array, 5, 1);
    
    data_array[0]= 0x00033902;
	data_array[1]= 0x003333b6;
	dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]= 0x00023902;
	data_array[1]= 0x000009cc;
	dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]= 0x001f3902;
	data_array[1]= 0x000600d3;
	data_array[2]= 0x00081a40;
	data_array[3]= 0x00071032;
	data_array[4]= 0x0f155407;
	data_array[5]= 0x12020405;
	data_array[6]= 0x33070510;
	data_array[7]= 0x370b0b33;
	data_array[8]= 0x00070710;
	dsi_set_cmdq(data_array, 9, 1);                   
                   
    data_array[0]= 0x002d3902;
	data_array[1]= 0x181919d5;
	data_array[2]= 0x1b1a1a18;
	data_array[3]= 0x0605041b;
	data_array[4]= 0x02010007;
	data_array[5]= 0x18212003;
	data_array[6]= 0x18181818;
	data_array[7]= 0x18181818;
	data_array[8]= 0x22181818;
	data_array[9]= 0x18181823;
	data_array[10]= 0x18181818;
	data_array[11]= 0x18181818;
	data_array[12]= 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);                      
                    
    data_array[0]= 0x002d3902;
	data_array[1]= 0x191818d6;
	data_array[2]= 0x1b1a1a19;
	data_array[3]= 0x0102031b;
	data_array[4]= 0x05060700;
	data_array[5]= 0x18222304;
	data_array[6]= 0x18181818;
	data_array[7]= 0x18181818;
	data_array[8]= 0x21181818;
	data_array[9]= 0x18181820;
	data_array[10]= 0x18181818;
	data_array[11]= 0x18181818;
	data_array[12]= 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);                      
                    
    data_array[0]= 0x002b3902;
	data_array[1]= 0x0c0600e0;
	data_array[2]= 0x1d3f3431;
	data_array[3]= 0x0c0a0641;
	data_array[4]= 0x15120f17;
	data_array[5]= 0x12071413;
	data_array[6]= 0x06001615;
	data_array[7]= 0x3f34300b;
	data_array[8]= 0x0a07401d;
	data_array[9]= 0x120e180d;
	data_array[10]= 0x08141214;
	data_array[11]= 0x00191413;
	dsi_set_cmdq(data_array, 12, 1);                  
                   
    data_array[0]= 0x00023902;
	data_array[1]= 0x000055d2;
	dsi_set_cmdq(data_array, 2, 1);    
    
    data_array[0]= 0x00033902;
	data_array[1]= 0x001430c0;
	dsi_set_cmdq(data_array, 2, 1);    
    
    data_array[0]= 0x00043902;
	data_array[1]= 0x010e41bf;
	dsi_set_cmdq(data_array, 2, 1);    
    
    data_array[0]= 0x00053902;
	data_array[1]= 0x40c000c7;
	data_array[1]= 0x000000c0;
	dsi_set_cmdq(data_array, 3, 1);    
    
    data_array[0]= 0x00023902;
	data_array[1]= 0x00008edf;
	dsi_set_cmdq(data_array, 2, 1);    
    
    data_array[0]= 0x00110502;
	dsi_set_cmdq(data_array, 1, 1);    
    
    MDELAY(150);
    
    data_array[0]= 0x00290502;
	dsi_set_cmdq(data_array, 1, 1);     
    
	//dsi_set_cmdq_V3(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);  
	MDELAY(200);
    lcm_set_gpio_output(GPIO_LCD_PWR_EN, 1);
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

LCM_DRIVER KD080D24_40NH_A32_drv= 
{
    .name			= "KD080D24_40NH_A32",
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
