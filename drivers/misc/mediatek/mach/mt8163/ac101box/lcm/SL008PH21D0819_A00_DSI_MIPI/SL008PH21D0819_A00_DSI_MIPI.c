#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
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
//#define GPIO_LCM_RST     (GPIO112 | 0x80000000)
//#define GPIO_LCM_AVDD_EN	   (GPIO119 | 0x80000000)
//#define GPIO_LCM_CHARGER_EN	   (GPIO19 | 0x80000000)

#define GPIO_LCM_VDD_EN	  GPIO83

#define GPIO_LCD_RST_EN     GPIO112

#define GPIO_LCD_BL_EN     GPIO118

#define GPIO_LCD_RST_EN  (GPIO90 | 0x80000000)

//#define GPIO_LCD_STBY_EN     GPIO113


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

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{

    if(GPIO == 0xFFFFFFFF)
    {
    #ifdef BUILD_LK
    #elif (defined BUILD_UBOOT)
         // do nothing in uboot
    #else	
	 //printf("kernel] lcm_set gpio()  \n");
    #endif
        return;
    }

    mt_set_gpio_mode(GPIO, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO, (output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}

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

	
	{0xF0,	2,	{0x5A,0x5A}},
	
	{0xF1,	2,	{0x5A,0x5A}},
	
	{0xFC,	2,	{0xA5,0xA5}}, 
	
	{0xD0,	2,	{0x00,0x10}}, 
	
	{0xC3,	3,  {0x40,0x00,0x28}},
	
	{REGFLAG_DELAY, 20, {}},
	
	{0x36, 	1,	{0x04}},
			
	{0xF6, 	6,	{0x63,0x20,0x86,0x00,0x00,0x10}}, 
	
	{0x11, 	0,	{}}, //sleep out
	
	{REGFLAG_DELAY, 120, {}},
	
	{0x36, 	1,	{0x00}},
	
	{0xF0,	2,	{0xA5,0xA5}},
	
	{0xF1,	2,	{0xA5,0xA5}},
	
	{0xFC,	2,	{0x5A,0x5A}}, 
	
	{0x29,	0,	{}},
	{REGFLAG_DELAY, 200, {}},
	
	// Note
	// Strongly recommend not to set Sleep out / DisplDSI_clk_HS_modeay On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcd_power_en(unsigned char enabled)
{
#if 0
   if (enabled)
		mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
   else	
        mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO); 
#else
	if (enabled)
    {
		upmu_set_rg_vgp1_vosel(0x07);
		upmu_set_rg_vgp1_en(0x1);
	}else{
		upmu_set_rg_vgp1_en(0x0);
	}
#endif		
}

static void lcd_reset(unsigned char enabled)
{
    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
    if (enabled)
    {
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    }
    else
    {	
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);    	
    }
}


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
	dsi_set_cmdq(&data_array, 2, 1); 
	MDELAY(10);

	data_array[0]=0x00053902;
	data_array[1]=0x7d0000b0;
	data_array[2]=0x0000000c;
	dsi_set_cmdq(&data_array, 3, 1); 
	MDELAY(10);
	
	data_array[0]=0x00033902;
	data_array[1]=0x008333ba; 
		
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00103902;
	data_array[1]=0x15156cb1; 
	data_array[2]=0xf1110424;
	data_array[3]=0x2397e480; 
	data_array[4]=0x58D2C080;  
	dsi_set_cmdq(&data_array, 5, 1); 
	MDELAY(1); 

	data_array[0]=0x000C3902;
	data_array[1]=0x106400B2; 
	data_array[2]=0x081C2007; 
	data_array[3]=0x004D1C08;	
	dsi_set_cmdq(&data_array, 4, 1);
	MDELAY(1);

	data_array[0]=0x000D3902;
	data_array[1]=0x03FF00B4; 
	data_array[2]=0x03640364;
	data_array[3]=0x01740164; 
	data_array[4]=0x00000074; 
	dsi_set_cmdq(&data_array, 5, 1); 
	MDELAY(1); 

	data_array[0]=0x00263902;
	data_array[1]=0x000600D3; 
	data_array[2]=0x00081A40; 
	data_array[3]=0x00071032;
	data_array[4]=0x0F155407;
	data_array[5]=0x12020405;
	data_array[6]=0x33070510; 
	data_array[7]=0x370B0B33;
	data_array[8]=0x08070710;
	data_array[9]=0x0a000000;
	data_array[10]=0x00000100;    
	dsi_set_cmdq(&data_array, 11, 1);
	MDELAY(1);
	
	data_array[0]=0x002D3902;
	data_array[1]=0x181919D5; 
	data_array[2]=0x1B1A1A18; 
	data_array[3]=0x0605041B; 
	data_array[4]=0x02010007; 
	data_array[5]=0x18212003; 
	data_array[6]=0x18181818; 
	data_array[7]=0x18181818; 
	data_array[8]=0x22181818; 
	data_array[9]=0x18181823;
	data_array[10]=0x18181818; 
	data_array[11]=0x18181818;
	data_array[12]=0x00000018; 
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1);

	data_array[0]=0x002D3902;
	data_array[1]=0x191818D6; 
	data_array[2]=0x1B1A1A19; 
	data_array[3]=0x0102031B; 
	data_array[4]=0x05060700; 
	data_array[5]=0x18222304; 
	data_array[6]=0x18181818; 
	data_array[7]=0x18181818; 
	data_array[8]=0x21181818;  
	data_array[9]=0x18181820;  
	data_array[10]=0x18181818;
	data_array[11]=0x18181818;  
	data_array[12]=0x00000018;  
	dsi_set_cmdq(&data_array, 13, 1);
	MDELAY(1); 

	data_array[0]=0x002B3902;
	data_array[1]=0x0C0600E0; 
	data_array[2]=0x1D3F3431; 
	data_array[3]=0x0C0A0641; 
	data_array[4]=0x15120F17; 
	data_array[5]=0x12071413; 
	data_array[6]=0x06001615; 
	data_array[7]=0x3F34300B; 
	data_array[8]=0x0A07401D;  
	data_array[9]=0x120E180D;  
	data_array[10]=0x08141214;
	data_array[11]=0x00191413;   
	dsi_set_cmdq(&data_array, 12, 1);
	MDELAY(1); 
	
	//data_array[0]=0x00033902;
    //data_array[1]=0x00|i|0x68|0xB6; 
	//i+=4;
	
	//j+=4;
	//int tmp=0x54;
	//int tmp1 = data_array[1] &0xff;
	//data_array[1]= ((data_array[1] >> 8 ) & 0xff | tmp) << 8 | tmp1;
	//int tmp3 = data_array[1] &0x0000ff00;
	
	//data_array[0]=0x00033902;
	//data_array[1]=0x005454B6; //0x003939B6;  //5454:12   
	
//	data_array[1]= ((((data_array[1] >> 16 )| tmp6) << 16 )|(((data_array[1] >> 8 )| tmp7) << 8 ))|0xB6;
	//tmp6 +2;
	//tmp7 +2;
	
	//data_array[1] = (tmp6<<24)|(tmp7<<16)|0xB6;
	//data_array[1]= ( tmp6 << 16 )|(tmp7 << 8 )|0xB6;
	//data_array[0]=0x00033902;
	//data_array[1]= ((data_array[1] >> 8 )| tmp7) << 8 ;
	//tmp6 +2;
	//tmp7 +2;
	

	data_array[0]=0x00033902;
    data_array[1]=0x003636B6; //0x003939B6;  //5454:12   
	dsi_set_cmdq(&data_array, 2, 1); 
	MDELAY(1);

	data_array[0]=0x00023902;
	data_array[1]=0x000005CC; //0x000009CC;//09
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x000066D2;//55
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00033902;
	data_array[1]=0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00043902;
	data_array[1]=0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00053902;
	data_array[1]=0x00C000C7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
	data_array[1]=0x00008EDF;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x00000736;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0] = 0x00110500; 
	dsi_set_cmdq(&data_array, 1, 1); 
	MDELAY(100);
	
	data_array[0] = 0x00290500; 
	dsi_set_cmdq(&data_array, 1, 1); 
	MDELAY(100);

}


static void lcm_init(void)
{
   
#if 1 //def BUILD_LK
	//printf("%s, LK \n", __func__);
	lcd_reset(0);
	lcd_power_en(0);
	upmu_set_rg_vgp1_vosel(0x7);
		upmu_set_rg_vgp1_en(0x1);
	MDELAY(20);//Must > 5ms
	lcd_power_en(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(1);

	MDELAY(10);//Must > 5ms
	lcd_reset(0);
	MDELAY(10);//Must > 5ms
	lcd_reset(1);
 
    MDELAY(100);
      // DSI_clk_HS_mode(1);

	//MDELAY(30);//Must > 50ms

	init_lcm_registers();
	  
	MDELAY(180);
	lcm_set_gpio_output(GPIO_LCD_BL_EN,1);

#else
	printk("%s, kernel", __func__);
#endif

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}




static void lcm_suspend(void)
{
	//unsigned int data_array[16];
	lcd_reset(0);
   
	MDELAY(10);
	lcd_power_en(0);
//#ifdef BUILD_LK
//	printf("%s, LK \n", __func__);
//#else
	//printk("%s, kernel", __func__);
//#endif

//	lcm_set_gpio_output(GPIO_LCD_BL_EN,0);
  //      MDELAY(200);

//	data_array[0] = 0x00280500;  //display off						  
//	dsi_set_cmdq(data_array, 1, 1);
//	MDELAY(100);

	//lcd_reset(0);
//	MDELAY(10);
//	lcd_power_en(0);
//	DSI_clk_HS_mode(0);
//	MDELAY(10);
}



static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("%s, LK \n", __func__);
#else
	printk("%s, kernel", __func__);
#endif

	lcm_init();
}
#define LCM_ID 0x8394
static unsigned int lcm_compare_id(void)
{
    unsigned char buffer[3];
    unsigned int array[16];
        upmu_set_rg_vgp1_vosel(0x7);
		upmu_set_rg_vgp1_en(0x0);
		 MDELAY(5);	
		 upmu_set_rg_vgp1_en(0x1);
   SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	lcd_reset(1);
	 MDELAY(10);
    SET_RESET_PIN(0);
	lcd_reset(0);
    MDELAY(10);
    SET_RESET_PIN(1);
	lcd_reset(1);
    MDELAY(50);	
  array[0] = 0x00043902;

   array[1] = 0x9483FFB9;

   dsi_set_cmdq(&array, 2, 1);

   MDELAY(10);

 

  array[0] = 0x00033902;                         

  array[1] = 0x008373BA;

  dsi_set_cmdq(&array, 2, 1);

  MDELAY(5);

 

   array[0] = 0x00033700;// read id return two byte,version and id

   dsi_set_cmdq(array, 1, 1);  //dsi_set_cmdq(array, 3, 1);

   

   read_reg_v2(0x04, buffer, 3);



printk("buffer zxl>x>>>>>>>>>>>>>>>>=%x,%x,=%x\n",buffer[0],buffer[1],buffer[2]);
  
 return (LCM_ID == (buffer[0]<<8 | buffer[1]) )? 1: 0;
   
}

static void lcm_init_power(void)
{

		upmu_set_rg_vgp1_vosel(0x7);
		upmu_set_rg_vgp1_en(0x1);
	MDELAY(50);//Must > 5ms
	lcd_power_en(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(1);
	MDELAY(20);//Must > 5ms
	lcd_reset(0);
	MDELAY(10);//Must > 5ms
	lcd_reset(1);
    MDELAY(120);
}

LCM_DRIVER SL008PH21D0819_A00_DSI_MIPI_lcm_drv = 
{
	.name = "SL008PH21D0819_A00_DSI_MIPI",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
	.init_power		= lcm_init_power,
};
