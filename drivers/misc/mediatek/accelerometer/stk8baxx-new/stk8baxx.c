/*
 *  stk8baxx.c - Linux kernel modules for sensortek  stk8ba50 / stk8ba50-R /
 *  stk8ba53 accelerometer
 *
 *  Copyright (C) 2012~2016 Lex Hsieh / Sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/math64.h>
#include <linux/version.h>
#define POWER_NONE_MACRO MT65XX_POWER_NONE
// #include <linux/batch.h>
#include <accel.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
#include "cust_acc.h"
#include "hwmsensor.h"
#include "hwmsen_dev.h"
#include "sensors_io.h"
#include "hwmsen_helper.h"
#else
#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#endif	/* #if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0) */
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
#include "stk8baxx.h"

/*----------------------------------------------------------------------------*/
//#define CONFIG_SENSORS_STK8BA53
#define CONFIG_SENSORS_STK8BA50

#define STK_DRIVER_VERSION	  "3.5.1"
#define STK_SELFTEST_VERSION	"1.1"
// #define STK_TUNE
// #define STK8BAXX_HOLD_ODR
// #define STK_PERMISSION_THREAD
#define CONFIG_STK8BAXX_LOWPASS
// #define STK_ZG_FILTER
// #define STK_ENABLE_AT_BOOT
#define STK_DEBUG_PRINT

/*----------------------------------------------------------------------------*/
#define STK8BAXX_INIT_ODR		    10		// 9=31Hz, 10=62Hz, 11=125Hz
#define  STK8BAXX_SPTIME_NO		  3
#define  STK8BAXX_SPTIME_BASE	 0x9
const static int STK8BAXX_SAMPLE_TIME[STK8BAXX_SPTIME_NO] = {32000, 16000, 8000};

#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#else
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
/*----------------------------------------------------------------------------*/
#define STK8BAXX_RNG_2G			0x3
#define STK8BAXX_RNG_4G			0x5
#define STK8BAXX_RNG_8G			0x8
#define STK8BAXX_RNG_16G		0xC

#ifdef CONFIG_SENSORS_STK8BA53
//Parameters under +-4g dynamic range
#define STK_DEF_DYNAMIC_RANGE	STK8BAXX_RNG_4G

#if (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_4G)
#define STK_LSB_1G			512
#elif (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_2G)
#define STK_LSB_1G			1024
#elif (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_8G)
#define STK_LSB_1G			256
#elif (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_16G)
#define STK_LSB_1G			128
#endif

#define STK_ZG_COUNT		(STK_LSB_1G / 128)
#define STK_TUNE_XYOFFSET 	(STK_LSB_1G * 3 / 10)
#define STK_TUNE_ZOFFSET 	(STK_LSB_1G * 3 / 5)
#define STK_TUNE_NOISE 		(STK_LSB_1G / 10)
#else
//Parameters under +-2g dynamic range
#define STK_DEF_DYNAMIC_RANGE	STK8BAXX_RNG_2G

#if (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_2G)
#define STK_LSB_1G			256
#elif (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_4G)
#define STK_LSB_1G			128
#elif (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_8G)
#define STK_LSB_1G			64
#elif (STK_DEF_DYNAMIC_RANGE == STK8BAXX_RNG_16G)
#define STK_LSB_1G			32
#endif
#define STK_ZG_COUNT		(STK_LSB_1G / 128 + 1)
#define STK_TUNE_XYOFFSET 	(STK_LSB_1G * 3 / 10)
#define STK_TUNE_ZOFFSET 	(STK_LSB_1G * 3 / 5)
#define STK_TUNE_NOISE 		(STK_LSB_1G / 10)
#endif

#define STK_TUNE_NUM 60
#define STK_TUNE_DELAY 60

#define STK_EVENT_SINCE_EN_LIMIT_DEF	(2)

/*---------------------calibration parameters-------------------------------*/
#define STK_SAMPLE_NO				10
#define STK_ACC_CALI_VER0			0x18
#define STK_ACC_CALI_VER1			0x03
#define STK_ACC_CALI_END				'\0'
#define STK_ACC_CALI_FILE 				"/data/misc/sensor/stkacccali.conf"
//#define STK_ACC_CALI_FILE 		"/system/etc/stkacccali.conf"
#define STK_ACC_CALI_FILE_SDCARD		"/sdcard/.stkacccali.conf"
#define STK_ACC_CALI_FILE_SIZE 		25

#define STK_K_SUCCESS_TUNE			0x04
#define STK_K_SUCCESS_FT2			0x03
#define STK_K_SUCCESS_FT1			0x02
#define STK_K_SUCCESS_FILE			0x01
#define STK_K_NO_CALI				0xFF
#define STK_K_RUNNING				0xFE
#define STK_K_FAIL_LRG_DIFF			0xFD
#define STK_K_FAIL_OPEN_FILE			0xFC
#define STK_K_FAIL_W_FILE				0xFB
#define STK_K_FAIL_R_BACK				0xFA
#define STK_K_FAIL_R_BACK_COMP		0xF9
#define STK_K_FAIL_I2C				0xF8
#define STK_K_FAIL_K_PARA				0xF7
#define STK_K_FAIL_OUT_RG			0xF6
#define STK_K_FAIL_ENG_I2C			0xF5
#define STK_K_FAIL_FT1_USD			0xF4
#define STK_K_FAIL_FT2_USD			0xF3
#define STK_K_FAIL_WRITE_NOFST		0xF2
#define STK_K_FAIL_OTP_5T				0xF1
#define STK_K_FAIL_PLACEMENT			0xF0

#define POSITIVE_Z_UP		0
#define NEGATIVE_Z_UP	1
#define POSITIVE_X_UP		2
#define NEGATIVE_X_UP	3
#define POSITIVE_Y_UP		4
#define NEGATIVE_Y_UP	5
/*------------------stk8baxx registers-------------------------*/
#define 	STK8BAXX_XOUT1			0x02
#define 	STK8BAXX_XOUT2			0x03
#define 	STK8BAXX_YOUT1			0x04
#define 	STK8BAXX_YOUT2			0x05
#define 	STK8BAXX_ZOUT1			0x06
#define 	STK8BAXX_ZOUT2			0x07
#define 	STK8BAXX_INTSTS1		0x09
#define 	STK8BAXX_INTSTS2		0x0A
#define 	STK8BAXX_EVENTINFO1	0x0B
#define 	STK8BAXX_EVENTINFO2	0x0C
#define 	STK8BAXX_RANGESEL		0x0F
#define 	STK8BAXX_BWSEL			0x10
#define 	STK8BAXX_POWMODE		0x11
#define  	STK8BAXX_DATASETUP		0x13
#define  	STK8BAXX_SWRST			0x14
#define  	STK8BAXX_INTEN1			0x16
#define  	STK8BAXX_INTEN2			0x17
#define  	STK8BAXX_INTMAP1		0x19
#define  	STK8BAXX_INTMAP2		0x1A
#define  	STK8BAXX_INTMAP3		0x1B
#define  	STK8BAXX_DATASRC		0x1E
#define  	STK8BAXX_INTCFG1		0x20
#define  	STK8BAXX_INTCFG2		0x21
#define  	STK8BAXX_LGDLY			0x22
#define  	STK8BAXX_LGTHD			0x23
#define  	STK8BAXX_HLGCFG		0x24
#define  	STK8BAXX_HGDLY			0x25
#define  	STK8BAXX_HGTHD			0x26
#define  	STK8BAXX_SLOPEDLY		0x27
#define  	STK8BAXX_SLOPETHD		0x28
#define  	STK8BAXX_TAPTIME		0x2A
#define  	STK8BAXX_TAPCFG		0x2B
#define  	STK8BAXX_ORIENTCFG		0x2C
#define  	STK8BAXX_ORIENTTHETA	0x2D
#define  	STK8BAXX_FLATTHETA		0x2E
#define  	STK8BAXX_FLATHOLD		0x2F
#define  	STK8BAXX_SLFTST			0x32
#define  	STK8BAXX_INTFCFG		0x34
#define  	STK8BAXX_OFSTCOMP1	0x36
#define  	STK8BAXX_OFSTCOMP2	0x37
#define  	STK8BAXX_OFSTFILTX		0x38
#define  	STK8BAXX_OFSTFILTY		0x39
#define  	STK8BAXX_OFSTFILTZ		0x3A
#define  	STK8BAXX_OFSTUNFILTX	0x3B
#define  	STK8BAXX_OFSTUNFILTY	0x3C
#define  	STK8BAXX_OFSTUNFILTZ	0x3D

/*	ZOUT1 register	*/
#define STK8BAXX_O_NEW			0x01

/*	SWRST register	*/
#define  	STK8BAXX_SWRST_VAL		0xB6

/*	STK8BAXX_POWMODE register	*/
#define STK8BAXX_MD_SUSPEND	0x80
#define STK8BAXX_MD_NORMAL		0x00
#define STK8BAXX_MD_SLP_MASK	0x1E

/*	RANGESEL register	*/
#define STK8BAXX_RANGE_MASK	0x0F

/* OFSTCOMP1 register*/
#define STK8BAXX_OF_CAL_DRY_MASK	0x10
#define CAL_AXIS_X_EN					0x20
#define CAL_AXIS_Y_EN					0x40
#define CAL_AXIS_Z_EN					0x60
#define CAL_OFST_RST					0x80

/* OFSTCOMP2 register*/
#define CAL_TG_X0_Y0_ZPOS1		0x20
#define CAL_TG_X0_Y0_ZNEG1		0x40

#define STK8BAXX_I2C_SLAVE_ADDR		0x18

#define STK8BA50_ID  		0x09	// reg 0x22, STK8BAXX_LGDLY
#define STK8BA50R_ID		0x86	// reg 0x0
#define STK8BA53_ID		0x87	// reg 0x0

#define STK_SELFTEST_STAT_DRIVER_ERR	(-3)
#define STK_SELFTEST_STAT_RUNNING	(-2)
#define STK_SELFTEST_STAT_NA			(-1)
#define STK_SELFTEST_STAT_NO_ERROR	(0)
#define STK_SELFTEST_STAT_FAIL_X		(1)
#define STK_SELFTEST_STAT_FAIL_Y		(2)
#define STK_SELFTEST_STAT_FAIL_Z		(4)

#define STK_SELFTEST_SAMPLE_NO		(100)

/* Set +-2g mode for all chips under selftest */
#ifdef CONFIG_SENSORS_STK8BA53
#define STK_SELFTEST_LSB_1G		1024
#else
#define STK_SELFTEST_LSB_1G		256
#endif
#define STK_SELFTEST_SATU		(STK_SELFTEST_LSB_1G * 1990 / 1000)

#define STK_SELFTEST_OFFSET_X		(STK_SELFTEST_LSB_1G * 6 / 10)
#define STK_SELFTEST_OFFSET_Y		(STK_SELFTEST_LSB_1G * 6 / 10)
#define STK_SELFTEST_OFFSET_Z		(STK_SELFTEST_LSB_1G * 6 / 10)

#define STK_SELFTEST_NOISE_X			(STK_SELFTEST_LSB_1G / 10)
#define STK_SELFTEST_NOISE_Y			(STK_SELFTEST_LSB_1G / 10)
#define STK_SELFTEST_NOISE_Z			(STK_SELFTEST_LSB_1G / 10)

/*----------------------------------------------------------------------------*/
#define DEBUG 1

#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               pr_info(GSE_TAG"%s start\n", __func__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_info(GSE_TAG "%s: "fmt,  __func__, ##args)
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id stk8baxx_i2c_id[] = { {STK8BAXX_DEV_NAME, 0}, {} };

/*the adapter id will be available in customization*/
/* static unsigned short stk8baxx_force[] = {0x00, STK8BAXX_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END}; */
/* static const unsigned short *const stk8baxx_forces[] = { stk8baxx_force, NULL }; */
/* static struct i2c_client_address_data stk8baxx_addr_data = { .forces = stk8baxx_forces,}; */
/*----------------------------------------------------------------------------*/
static int stk8baxx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int stk8baxx_i2c_remove(struct i2c_client *client);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static int gsensor_setup_irq(void);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
static int gsensor_local_init(void);
static int gsensor_remove(void);
static int gsensor_set_delay(u64 ns);
static int stk8baxx_suspend(struct i2c_client *client, pm_message_t msg);
static int stk8baxx_resume(struct i2c_client *client);
static int STK8BAXX_SetOffset(struct i2c_client *client, char buf[]);
static int stk8baxx_store_in_file(u8 offset[], u8 status, u32 variance[]);
/* static int stk8baxx_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info); */
/*----------------------------------------------------------------------------*/
static DEFINE_MUTEX(gsensor_mutex);
static DEFINE_MUTEX(gsensor_scp_en_mutex);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static bool enable_status;
#endif	/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

static int gsensor_init_flag = -1;	/* 0<==>OK -1 <==> fail */
static struct acc_hw stk_accel_cust;
static struct acc_hw *hw = &stk_accel_cust;
static struct stk8baxx_i2c_data *obj_i2c_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
typedef struct GSENSOR_VECTOR3D GSENSOR_VECTOR3D;
typedef struct SENSOR_DATA	SENSOR_DATA;
#endif
static GSENSOR_VECTOR3D gsensor_gain;
static struct i2c_client *stk8baxx_i2c_client = NULL;
static struct stk8baxx_i2c_data *obj_i2c_data = NULL;
static DEFINE_MUTEX(stk8baxx_i2c_mutex);
/*----------------------------------------------------------------------------*/

static struct acc_init_info stk8baxx_init_info = {
	.name = "stk8baxx",
	.init = gsensor_local_init,
	.uninit = gsensor_remove,
};

/*----------------------------------------------------------------------------*/
enum ADX_TRC {
    ADX_TRC_FILTER = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL = 0x04,
    ADX_TRC_CALI = 0X08,
    ADX_TRC_INFO = 0X10,
};
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][STK8BAXX_AXES_NUM];
	int sum[STK8BAXX_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct stk8baxx_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	struct work_struct irq_work;
	int SCP_init_done;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	struct data_resolution *reso;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	s16 cali_sw[STK8BAXX_AXES_NUM + 1];
	s8 offset[STK8BAXX_AXES_NUM + 1];	/*+1: for 4-byte alignment */
	s16 data[STK8BAXX_AXES_NUM + 1];
#if defined(CONFIG_STK8BAXX_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
	atomic_t				event_since_en;
	atomic_t				event_since_en_limit;
	atomic_t				recv_reg;
	s16						re_enable;
	char stk_tune_offset_record[3];
#ifdef STK_TUNE
	int stk_tune_offset[3];
	int stk_tune_sum[3];
	int stk_tune_max[3];
	int stk_tune_min[3];
	int stk_tune_index;
	int stk_tune_done;
	s64 stk_tune_square_sum[3];
#endif
	bool sensor_power;
	bool first_enable;
	atomic_t cali_status;
	int selftest_status;
	uint32_t gsensor_delay;
	int pid;
};

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_OF
static struct of_device_id stk8baxx_match_table[] = {
	{ .compatible = "mediatek,gsensor", },
	//{ .compatible = "mediatek,stk8baxx", },
	{ },
};
#endif
static struct i2c_driver stk8baxx_i2c_driver = {
	.driver = {
		/* .owner          = THIS_MODULE, */
		.name = STK8BAXX_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = stk8baxx_match_table,
#endif
	},
	.probe = stk8baxx_i2c_probe,
	.remove = stk8baxx_i2c_remove,
	/* .detect                         = stk8baxx_i2c_detect, */
	.suspend = stk8baxx_suspend,
	.resume = stk8baxx_resume,
	.id_table = stk8baxx_i2c_id,
	/* .address_data = &stk8baxx_addr_data, */
};

/*----------------------------------------------------------------------------*/
static struct data_resolution stk8baxx_offset_resolution = {{7, 8}, 128};	// the same for both stk8ba53 and stk8ba50/-R

#ifdef CONFIG_SENSORS_STK8BA53
static struct data_resolution stk8baxx_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/

	{{ 0, 98}, 1024},   /*+/-2g in 12-bit resolution:  0.98 mg/LSB*/
	{{ 1, 95}, 512},   /*+/-4g	in 12-bit resolution:  1.95 mg/LSB*/
	{{3, 9},	256},   /*+/-8g	in 12-bit resolution: 3.90 mg/LSB*/
	{{7, 81},	128},   /*+/-16g in 12-bit resolution: 7.81 mg/LSB*/
};
#else
static struct data_resolution stk8baxx_data_resolution[] = {
	/* combination by {FULL_RES,RANGE}*/
	{{ 3, 9}, 256},   /*+/-2g	in 10-bit resolution:  3.9 mg/LSB*/
	{{ 7, 8}, 128},   /*+/-4g	in 10-bit resolution:  7.8 mg/LSB*/
	{{15, 6},	64},   /*+/-8g	in 10-bit resolution: 15.6 mg/LSB*/
	{{31, 2},	32},   /*+/-16g in 10-bit resolution: 31.2 mg/LSB*/
};
#endif
/*----------------------------------------------------------------------------*/

#ifndef C_I2C_FIFO_SIZE
#define C_I2C_FIFO_SIZE 	8
#endif

static int stk8baxx_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	struct i2c_msg msgs[2] = {{0}, {0}};
	int err;

	mutex_lock(&stk8baxx_i2c_mutex);	
	
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf= &beg;
	
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;
		
	if (!client) {
		mutex_unlock(&stk8baxx_i2c_mutex);			
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&stk8baxx_i2c_mutex);	
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;/*no error*/
	}
	mutex_unlock(&stk8baxx_i2c_mutex);	
	return err;
}

/*----------------------------------------------------------------------------*/

static s32 stk8baxx_i2c_smbus_write_byte_data(const struct i2c_client *client, u8 command,
        u8 value)
{
	int err;	
	unsigned char data[11] = {0};
	struct i2c_msg msg = {0};
	int tx_retry;
	int overall_retry;

	mutex_lock(&stk8baxx_i2c_mutex);		
    if (!client) {
		mutex_unlock(&stk8baxx_i2c_mutex);
		return -EINVAL;
	}
	data[0] = command;
	data[1] = value;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	for(overall_retry = 0; overall_retry < 3; overall_retry++) {		
		for (tx_retry = 0; tx_retry < 3; tx_retry++) {
			err = i2c_transfer(client->adapter, &msg, 1);
			if (err == 1)
				break;
			else
				mdelay(5);
		}
		
		if (tx_retry >= 3) {
			printk(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);		
			mutex_unlock(&stk8baxx_i2c_mutex);
			return -EIO;
		}
		
		// skip read-only CAL_RDY
		if(command == STK8BAXX_OFSTCOMP1 || command == STK8BAXX_SWRST) {
			mutex_unlock(&stk8baxx_i2c_mutex);
			return 0;
		}
		
		err = i2c_smbus_read_byte_data(client, command);
		if(err < 0) {
			GSE_ERR("i2c transfer failed, err=%d\n", err);
			mutex_unlock(&stk8baxx_i2c_mutex);
			return STK8BAXX_ERR_I2C;
		}
		
		if((u8)err == value) {
			mutex_unlock(&stk8baxx_i2c_mutex);
			return 0;
		}
	}
	GSE_ERR("read back error,w=0x%x,r=0x%x\n", value, err);		
	mutex_unlock(&stk8baxx_i2c_mutex);	
	return -EIO;
}
/*
static s32 stk8baxx_i2c_smbus_write_byte_data(const struct i2c_client *client, u8 command,
        u8 value)
{
	s32 err;
	int overall_retry;
	int tx_retry;
	u8 databuf[2];

	mutex_lock(&stk8baxx_i2c_mutex);	
	
	memset(databuf, 0, sizeof(u8)*2);
	databuf[0] = command;
	databuf[1]= value;
	//printk("stk8baxx_i2c_smbus_write_byte_data command is 0x%x, value is 0x%x \n", command, value);
	for(overall_retry = 0; overall_retry < 3; overall_retry++) {
		for (tx_retry = 0; tx_retry <= 3; tx_retry++) {
			//	err = i2c_smbus_write_byte_data(client, command, value);
			err = i2c_master_send(client, databuf, 0x2);
			//	printk("stk8baxx_i2c_smbus_write_byte_data:tx_retry is %d, i2c_smbus_write_byte_data err is %d \n", tx_retry,err);
			if(err > 0)
				break;
			else
				msleep(10);
		}

		if (tx_retry > 3) {
			mutex_unlock(&stk8baxx_i2c_mutex);
			GSE_ERR("i2c transfer error, tx_retry over 3\n");
			return -EIO;
		}
		// skip read-only CAL_RDY
		if(command == STK8BAXX_OFSTCOMP1 || command == STK8BAXX_SWRST) {
			mutex_unlock(&stk8baxx_i2c_mutex);
			return 0;
		}
		err = i2c_smbus_read_byte_data(client, command);
		if(err < 0) {
			mutex_unlock(&stk8baxx_i2c_mutex);
			GSE_ERR("i2c transfer failed, err=%d\n", err);
			return STK8BAXX_ERR_I2C;
		}
		if((u8)err == value) {
			mutex_unlock(&stk8baxx_i2c_mutex);
			return 0;
		}
	}
	GSE_ERR("read back error,w=0x%x,r=0x%x\n", value, err);
	
	mutex_unlock(&stk8baxx_i2c_mutex);
	return -EIO;
}
*/
/*----------------------------------------------------------------------------*/
static void stk8baxx_power(struct acc_hw *hw, unsigned int on)
{
#if 0
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO) {	// have externel LDO
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on) {	// power status not change
			GSE_LOG("ignore power control: %d\n", on);
		} else if(on) {	// power on
			if(!hwPowerOn(hw->power_id, hw->power_vol, "STK8BAXX")) {
				GSE_ERR("power on fails!!\n");
			}
		} else {	// power off
			if (!hwPowerDown(hw->power_id, "STK8BAXX")) {
				GSE_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
#endif
}
/*----------------------------------------------------------------------------*/
#ifdef STK_TUNE
static int32_t stk_get_file_content(char * r_buf, int8_t buf_size)
{
	struct file  *cali_file;
	mm_segment_t fs;
	ssize_t ret;
	int err;

	cali_file = filp_open(STK_ACC_CALI_FILE, O_RDONLY,0);
	if(IS_ERR(cali_file)) {
		err = PTR_ERR(cali_file);
		printk("%s: filp_open error, no offset file,err=%d\n", __func__, err);
		return -ENOENT;
	} else {
		fs = get_fs();
		set_fs(get_ds());
		ret = cali_file->f_op->read(cali_file,r_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret < 0) {
			printk("%s: read error, ret=%d\n", __func__, (int)ret);
			filp_close(cali_file,NULL);
			return -EIO;
		}
		set_fs(fs);
	}

	filp_close(cali_file,NULL);
	return 0;
}
/*----------------------------------------------------------------------------*/

static void stk_handle_first_en(struct i2c_client *client)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	char offset[3];
	char mode;
	int aa;

	if ((stk_get_file_content(r_buf, STK_ACC_CALI_FILE_SIZE)) == 0) {
		if(r_buf[0] == STK_ACC_CALI_VER0 && r_buf[1] == STK_ACC_CALI_VER1
		        && r_buf[STK_ACC_CALI_FILE_SIZE - 1] == STK_ACC_CALI_END) {
			offset[0] = r_buf[3];
			offset[1] = r_buf[5];
			offset[2] = r_buf[7];
			mode = r_buf[8];
			STK8BAXX_SetOffset(client, offset);

			obj->stk_tune_offset_record[0] = offset[0];
			obj->stk_tune_offset_record[1] = offset[1];
			obj->stk_tune_offset_record[2] = offset[2];
			printk("%s: set offset:%d,%d,%d, mode=%d\n", __func__, offset[0], offset[1], offset[2], mode);
			printk("%s: variance=%u,%u,%u\n", __func__,
			       (r_buf[9] << 24 | r_buf[10] << 16 | r_buf[11] << 8 | r_buf[12]),
			       (r_buf[13] << 24 | r_buf[14] << 16 | r_buf[15] << 8 | r_buf[16]),
			       (r_buf[17] << 24 | r_buf[18] << 16 | r_buf[19] << 8 | r_buf[20]));
			atomic_set(&obj->cali_status, mode);
		} else {
			printk("%s: cali version number error! r_buf=0x%x,0x%x,0x%x,0x%x,0x%x\n",
			       __func__, r_buf[0], r_buf[1], r_buf[2], r_buf[3], r_buf[4]);
			for(aa=0; aa<STK_ACC_CALI_FILE_SIZE; aa++)
				GSE_LOG("buf[%d]=%x\n", aa, r_buf[aa]);
			//return -EINVAL;
		}
	} else if(obj->stk_tune_offset_record[0]!=0 || obj->stk_tune_offset_record[1]!=0 || obj->stk_tune_offset_record[2]!=0) {
		STK8BAXX_SetOffset(client, obj->stk_tune_offset_record);
		obj->stk_tune_done = 1;
		atomic_set(&obj->cali_status, STK_K_SUCCESS_TUNE);
		printk("%s: set offset:%d,%d,%d\n", __func__, obj->stk_tune_offset_record[0],
		       obj->stk_tune_offset_record[1],obj->stk_tune_offset_record[2]);
	} else {
		offset[0] = offset[1] = offset[2] = 0;
		stk8baxx_store_in_file(offset, STK_K_NO_CALI, NULL);
		atomic_set(&obj->cali_status, STK_K_NO_CALI);
	}
	GSE_LOG("finish, cali_status = 0x%x\n", atomic_read(&obj->cali_status));
	return;
}
#endif
/*----------------------------------------------------------------------------*/
static int STK8BAXX_SetDataResolution(struct i2c_client *client ,u8 dataresolution)
{
	int err;
	u8 databuf;
	int res = 0;
	u8  reso=0;
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	
	GSE_LOG("dataresolution= %d!\n", dataresolution);

	switch(dataresolution) {
	case STK8BAXX_RNG_2G:
		databuf= STK8BAXX_RNG_2G;
		reso = 0;
		break;
	case STK8BAXX_RNG_4G:
		databuf = STK8BAXX_RNG_4G;
		reso = 1;
		break;
	case STK8BAXX_RNG_8G:
		databuf = STK8BAXX_RNG_8G;
		reso = 2;
		break;
	case STK8BAXX_RNG_16G:
		databuf = STK8BAXX_RNG_16G;
		reso = 3;
		break;
	default:
		databuf = STK8BAXX_RNG_2G;
		GSE_ERR(" unknown range, set as STK8BAXX_RNG_2G\n");
	}

	if(reso < sizeof(stk8baxx_data_resolution)/sizeof(stk8baxx_data_resolution[0])) {
		obj->reso = &stk8baxx_data_resolution[reso];
		GSE_LOG("reso=%x!! OK \n",reso);
		err = 0;
	} else {
		GSE_ERR("choose sensitivity  fail!!\n");
		err = -EINVAL;
	}
	/*
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}
	*/
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_RANGESEL, databuf);
	if(res < 0) {
		err = STK8BAXX_ERR_I2C;
	}
	
	/*
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}
	
	udelay(500);
	*/
	return err;
}

static int STK8BAXX_SetCaliScaleOfst(struct i2c_client *client, int acc[3])
{
	// struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int result;
	int xyz_sensitivity = 256;
	int axis;

	result = i2c_smbus_read_byte_data(client, STK8BAXX_RANGESEL);
	if (result < 0) {
		GSE_ERR("failed to read acc data, error=%d\n", result);
		return result;
	}

	result &= STK8BAXX_RANGE_MASK;
	GSE_LOG("range=0x%x\n", result);
#ifdef CONFIG_SENSORS_STK8BA53
	switch(result) {
	case STK8BAXX_RNG_2G:
		xyz_sensitivity = stk8baxx_data_resolution[0].sensitivity;
		// xyz_sensitivity = 1024;
		break;
	case STK8BAXX_RNG_4G:
		xyz_sensitivity = stk8baxx_data_resolution[1].sensitivity;
		// xyz_sensitivity = 512;
		break;
	case STK8BAXX_RNG_8G:
		xyz_sensitivity = stk8baxx_data_resolution[2].sensitivity;
		// xyz_sensitivity = 256;
		break;
	case STK8BAXX_RNG_16G:
		xyz_sensitivity = stk8baxx_data_resolution[3].sensitivity;
		// xyz_sensitivity = 128;
		break;
	default:
		xyz_sensitivity = stk8baxx_data_resolution[0].sensitivity;
		// xyz_sensitivity = 512;
	}
#else
	switch(result) {
	case STK8BAXX_RNG_2G:
		xyz_sensitivity = stk8baxx_data_resolution[0].sensitivity;
		// xyz_sensitivity = 256;
		break;
	case STK8BAXX_RNG_4G:
		xyz_sensitivity = stk8baxx_data_resolution[1].sensitivity;
		// xyz_sensitivity = 128;
		break;
	case STK8BAXX_RNG_8G:
		xyz_sensitivity = stk8baxx_data_resolution[2].sensitivity;
		// xyz_sensitivity = 64;
		break;
	case STK8BAXX_RNG_16G:
		xyz_sensitivity = stk8baxx_data_resolution[3].sensitivity;
		// xyz_sensitivity = 32;
		break;
	default:
		xyz_sensitivity = stk8baxx_data_resolution[0].sensitivity;
		// xyz_sensitivity = 256;
	}
#endif

	// offset sensitivity is fixed to 128 LSB/g for all range setting
	for(axis=0; axis<3; axis++) {
		acc[axis] = acc[axis] * stk8baxx_offset_resolution.sensitivity / xyz_sensitivity;
	}
	return 0;
}

#ifdef STK_TUNE
static void STK8BAXX_ResetPara(struct stk8baxx_i2c_data *obj)
{
	int ii;
	for(ii=0; ii<3; ii++) {
		obj->stk_tune_sum[ii] = 0;
		obj->stk_tune_min[ii] = 4096;
		obj->stk_tune_max[ii] = -4096;
		obj->stk_tune_square_sum[ii] = 0LL;
	}
	return;
}
/*----------------------------------------------------------------------------*/
static void STK8BAXX_Tune(struct i2c_client *client, s16 acc[])
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int ii;
	char offset[3];
	const s64 var_enlarge_scale = 64;
	u32 variance[3];
	s64 s64_temp;

	if (obj->stk_tune_done !=0)
		return;

	if(atomic_read(&obj->event_since_en) >= STK_TUNE_DELAY) {
		if ((abs(acc[0]) <= STK_TUNE_XYOFFSET) && (abs(acc[1]) <= STK_TUNE_XYOFFSET)
		        && (abs(abs(acc[2]) - obj->reso->sensitivity) <= STK_TUNE_ZOFFSET))
			obj->stk_tune_index++;
		else
			obj->stk_tune_index = 0;

		if (obj->stk_tune_index==0)
			STK8BAXX_ResetPara(obj);
		else {
			for(ii=0; ii<3; ii++) {
				obj->stk_tune_sum[ii] += acc[ii];
				obj->stk_tune_square_sum[ii] += acc[ii] * acc[ii];
				if(acc[ii] > obj->stk_tune_max[ii])
					obj->stk_tune_max[ii] = acc[ii];
				if(acc[ii] < obj->stk_tune_min[ii])
					obj->stk_tune_min[ii] = acc[ii];
			}
		}

		if(obj->stk_tune_index == STK_TUNE_NUM) {
			for(ii=0; ii<3; ii++) {
				if((obj->stk_tune_max[ii] - obj->stk_tune_min[ii]) > STK_TUNE_NOISE) {
					obj->stk_tune_index = 0;
					STK8BAXX_ResetPara(obj);
					return;
				}
			}

			obj->stk_tune_offset[0] = (obj->stk_tune_sum[0]/STK_TUNE_NUM);
			obj->stk_tune_offset[1] = (obj->stk_tune_sum[1]/STK_TUNE_NUM);
			if (acc[2] > 0)
				obj->stk_tune_offset[2] = (obj->stk_tune_sum[2]/STK_TUNE_NUM - obj->reso->sensitivity);
			else
				obj->stk_tune_offset[2] = (obj->stk_tune_sum[2]/STK_TUNE_NUM + obj->reso->sensitivity);

			STK8BAXX_SetCaliScaleOfst(client, obj->stk_tune_offset);

			for(ii=0; ii<3; ii++) {
				offset[ii] = (u8) (-obj->stk_tune_offset[ii]);
				obj->stk_tune_offset_record[ii] = offset[ii];

				obj->stk_tune_square_sum[ii] *= var_enlarge_scale * var_enlarge_scale;
				s64_temp = obj->stk_tune_sum[ii] * var_enlarge_scale;
				obj->stk_tune_square_sum[ii] = div64_long(obj->stk_tune_square_sum[ii], STK_TUNE_NUM - 1);
				s64_temp = s64_temp * s64_temp;
				s64_temp = div64_long(s64_temp, STK_TUNE_NUM);
				s64_temp = div64_long(s64_temp, STK_TUNE_NUM - 1);
				variance[ii] = (u32)(obj->stk_tune_square_sum[ii] - s64_temp);
			}
			STK8BAXX_SetOffset(client, offset);
			stk8baxx_store_in_file(offset, STK_K_SUCCESS_TUNE, variance);
			obj->stk_tune_done = 1;
			atomic_set(&obj->cali_status, STK_K_SUCCESS_TUNE);
			atomic_set(&obj->event_since_en, 0);
			printk("%s:TUNE done, %d,%d,%d\n", __func__, offset[0], offset[1],
				offset[2]);
			printk("%s:TUNE done, var=%u,%u,%u\n", __func__, variance[0], 
				variance[1],variance[2]);
		}
	}

	return;
}
#endif
/*----------------------------------------------------------------------------*/

static void STK8BAXX_SignConv(s16 acc_data[], u8 acc_reg_data[])
{
#ifdef CONFIG_SENSORS_STK8BA53
	acc_data[STK8BAXX_AXIS_X] = acc_reg_data[STK8BAXX_AXIS_X*2+1]<<8 | acc_reg_data[STK8BAXX_AXIS_X*2];
	acc_data[STK8BAXX_AXIS_X] >>= 4;
	acc_data[STK8BAXX_AXIS_Y] = acc_reg_data[STK8BAXX_AXIS_Y*2+1]<<8 | acc_reg_data[STK8BAXX_AXIS_Y*2];
	acc_data[STK8BAXX_AXIS_Y] >>= 4;
	acc_data[STK8BAXX_AXIS_Z] = acc_reg_data[STK8BAXX_AXIS_Z*2+1]<<8 | acc_reg_data[STK8BAXX_AXIS_Z*2];
	acc_data[STK8BAXX_AXIS_Z] >>= 4;
#else
	acc_data[STK8BAXX_AXIS_X] = acc_reg_data[STK8BAXX_AXIS_X*2+1]<<8 | acc_reg_data[STK8BAXX_AXIS_X*2];
	acc_data[STK8BAXX_AXIS_X] >>= 6;
	acc_data[STK8BAXX_AXIS_Y] = acc_reg_data[STK8BAXX_AXIS_Y*2+1]<<8 | acc_reg_data[STK8BAXX_AXIS_Y*2];
	acc_data[STK8BAXX_AXIS_Y] >>= 6;
	acc_data[STK8BAXX_AXIS_Z] = acc_reg_data[STK8BAXX_AXIS_Z*2+1]<<8 | acc_reg_data[STK8BAXX_AXIS_Z*2];
	acc_data[STK8BAXX_AXIS_Z] >>= 6;
#endif
	return;
}

#define CHECK_CODE_SIZE  (STK_LSB_1G + 1)

static int stkcheckcode[CHECK_CODE_SIZE][CHECK_CODE_SIZE];

static int STK8BAXX_GetCode(int a0, int a1)
{
	int a=0,i=0,j=0,b=0,d=0,n=0,dd=0;
    int c = STK_LSB_1G*STK_LSB_1G - a0*a0 - a1*a1;
    if(c <0)
        return 0;	
	a = a + c;
	if(a!=i)
	{
		if(a<i)
			a = i-a;
		for(i=0;i*i<a;i++)
			j = (i+1)*10;
		i = (i-1)*10;
		d = 100;	
		a = a*d;
		while(d>10)	
		{
			b = (i+j)>>1;
			if(b*b>a)
				j = b;
			else
				i = b;
			d = a-i*i;
			if(dd == d)
				n++;
			else
				n = 0;
			if(n>=3)
				break;
			dd = d;
		}
		if((i%10)>=5)
			a = (i/10)+1;
		else
			a = (i/10);
	}
	return a;
}

static void STK8BAXX_CheckInit(void)
{
	int i,j;
	
	for(i=0;i<CHECK_CODE_SIZE;i++)
		for(j=0;j<CHECK_CODE_SIZE;j++)
			stkcheckcode[i][j] = STK8BAXX_GetCode(i,j);	
}

static int STK8BAXX_CheckCode(s16 acc[])
{
	int a, b;
	if(acc[0]>0)
		a = acc[0];
	else
		a = -acc[0];
	if(acc[1]>0)
		b = acc[1];
	else
		b = -acc[1];
	if(a>=CHECK_CODE_SIZE || b>=CHECK_CODE_SIZE)
		acc[2] = 0;
	else
		acc[2] = (s16)stkcheckcode[a][b];
	return 0;
}

static int STK8BAXX_CheckReading(struct i2c_client *client, s16 acc[], bool clear)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	static int check_result = 0;
	static int event_no = 0;
#ifdef CONFIG_SENSORS_STK8BA53
	const int verifing[] = {-2048,2047};
#else
	const int verifing[] = {-512,511};
#endif

	if(event_no > 20)
		return 0;

	if(acc[0] == verifing[0] || acc[0] == verifing[1] || acc[1] == verifing[0] || acc[1] == verifing[1] ||
	        acc[2] == verifing[0] || acc[2] == verifing[1]) {
		//	GSE_LOG("acc:%o,%o,%o\n", acc[0], acc[1], acc[2]);
		printk("%s: acc:%o,%o,%o\n", __func__, acc[0], acc[1], acc[2]);
		check_result++;
	}

	if(clear) {
		if(check_result >= 3) {
			if(acc[0] != verifing[0] && acc[0] != verifing[1] && acc[1] != verifing[0]
			        && acc[1] != verifing[1])
				atomic_set(&obj->event_since_en_limit, STK_EVENT_SINCE_EN_LIMIT_DEF + 6);
			else
				atomic_set(&obj->event_since_en_limit, 10000);
			GSE_LOG(" incorrect reading\n");
			return 1;
		}
		check_result = 0;
	}
	event_no++;
	return 0;
}

static int STK8BAXX_ReadData(struct i2c_client *client, s16 data[STK8BAXX_AXES_NUM])
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int result;
	u8 acc_reg[6];
#ifdef STK_ZG_FILTER
	s16 zero_fir = 0;
#endif
	int k_status = atomic_read(&obj->cali_status);
	s16 acc_xyz[3] = {0};

	result = stk8baxx_hwmsen_read_block(client,STK8BAXX_XOUT1, acc_reg, 6);
	if (result < 0) {
		GSE_ERR(" failed to read acc data, error=%d\n", result);
		return result;
	}

	STK8BAXX_SignConv(data, acc_reg);
	acc_xyz[STK8BAXX_AXIS_X] = data[STK8BAXX_AXIS_X];
	acc_xyz[STK8BAXX_AXIS_Y] = data[STK8BAXX_AXIS_Y];
	acc_xyz[STK8BAXX_AXIS_Z] = data[STK8BAXX_AXIS_Z];
	
	if(atomic_read(&obj->event_since_en) == (STK_EVENT_SINCE_EN_LIMIT_DEF + 1) 
		|| atomic_read(&obj->event_since_en) == (STK_EVENT_SINCE_EN_LIMIT_DEF + 2))
		STK8BAXX_CheckReading(client, acc_xyz, false);
	else if(atomic_read(&obj->event_since_en) == (STK_EVENT_SINCE_EN_LIMIT_DEF + 3))
		STK8BAXX_CheckReading(client, acc_xyz, true);
	else if(atomic_read(&obj->event_since_en_limit) == (STK_EVENT_SINCE_EN_LIMIT_DEF + 6))
		STK8BAXX_CheckCode(acc_xyz);

	data[STK8BAXX_AXIS_X] = acc_xyz[STK8BAXX_AXIS_X];
	data[STK8BAXX_AXIS_Y] = acc_xyz[STK8BAXX_AXIS_Y];
	data[STK8BAXX_AXIS_Z] = acc_xyz[STK8BAXX_AXIS_Z];
	
	if(atomic_read(&obj->trace) & ADX_TRC_RAWDATA) {
		GSE_LOG("acc_reg = 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", acc_reg[0], 
			acc_reg[1], acc_reg[2], acc_reg[3], acc_reg[4], acc_reg[5]);
	}
	
#ifdef STK_DEBUG_PRINT	
	GSE_LOG("data=%d,%d,%d\n", data[STK8BAXX_AXIS_X], data[STK8BAXX_AXIS_Y], 
		data[STK8BAXX_AXIS_Z]);
#endif

	if(k_status == STK_K_RUNNING || obj->selftest_status == STK_SELFTEST_STAT_RUNNING)
		return 0;
	
#ifdef STK_TUNE
	if((k_status&0xF0) != 0) {
		acc_xyz[STK8BAXX_AXIS_X] = data[STK8BAXX_AXIS_X];
		acc_xyz[STK8BAXX_AXIS_Y] = data[STK8BAXX_AXIS_Y];
		acc_xyz[STK8BAXX_AXIS_Z] = data[STK8BAXX_AXIS_Z];
		STK8BAXX_Tune(client, acc_xyz);
		data[STK8BAXX_AXIS_X] = acc_xyz[STK8BAXX_AXIS_X];
		data[STK8BAXX_AXIS_Y] = acc_xyz[STK8BAXX_AXIS_Y];
		data[STK8BAXX_AXIS_Z] = acc_xyz[STK8BAXX_AXIS_Z];
	}
#endif
#ifdef CONFIG_STK8BAXX_LOWPASS
	if(atomic_read(&obj->filter)) {
		if(atomic_read(&obj->fir_en) && !atomic_read(&obj->suspend)) {
			int idx, firlen = atomic_read(&obj->firlen);
			if(obj->fir.num < firlen) {
				obj->fir.raw[obj->fir.num][STK8BAXX_AXIS_X] = data[STK8BAXX_AXIS_X];
				obj->fir.raw[obj->fir.num][STK8BAXX_AXIS_Y] = data[STK8BAXX_AXIS_Y];
				obj->fir.raw[obj->fir.num][STK8BAXX_AXIS_Z] = data[STK8BAXX_AXIS_Z];
				obj->fir.sum[STK8BAXX_AXIS_X] += data[STK8BAXX_AXIS_X];
				obj->fir.sum[STK8BAXX_AXIS_Y] += data[STK8BAXX_AXIS_Y];
				obj->fir.sum[STK8BAXX_AXIS_Z] += data[STK8BAXX_AXIS_Z];
				if(atomic_read(&obj->trace) & ADX_TRC_FILTER) {
					GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", 
					obj->fir.num, 
					obj->fir.raw[obj->fir.num][STK8BAXX_AXIS_X], 
					obj->fir.raw[obj->fir.num][STK8BAXX_AXIS_Y], 
					obj->fir.raw[obj->fir.num][STK8BAXX_AXIS_Z],
					obj->fir.sum[STK8BAXX_AXIS_X], 
					obj->fir.sum[STK8BAXX_AXIS_Y], 
					obj->fir.sum[STK8BAXX_AXIS_Z]);
				}
				obj->fir.num++;
				obj->fir.idx++;
			} else {
				idx = obj->fir.idx % firlen;
				obj->fir.sum[STK8BAXX_AXIS_X] -= obj->fir.raw[idx][STK8BAXX_AXIS_X];
				obj->fir.sum[STK8BAXX_AXIS_Y] -= obj->fir.raw[idx][STK8BAXX_AXIS_Y];
				obj->fir.sum[STK8BAXX_AXIS_Z] -= obj->fir.raw[idx][STK8BAXX_AXIS_Z];
				obj->fir.raw[idx][STK8BAXX_AXIS_X] = data[STK8BAXX_AXIS_X];
				obj->fir.raw[idx][STK8BAXX_AXIS_Y] = data[STK8BAXX_AXIS_Y];
				obj->fir.raw[idx][STK8BAXX_AXIS_Z] = data[STK8BAXX_AXIS_Z];
				obj->fir.sum[STK8BAXX_AXIS_X] += data[STK8BAXX_AXIS_X];
				obj->fir.sum[STK8BAXX_AXIS_Y] += data[STK8BAXX_AXIS_Y];
				obj->fir.sum[STK8BAXX_AXIS_Z] += data[STK8BAXX_AXIS_Z];
				obj->fir.idx++;

				data[STK8BAXX_AXIS_X] = obj->fir.sum[STK8BAXX_AXIS_X]/firlen;
				data[STK8BAXX_AXIS_Y] = obj->fir.sum[STK8BAXX_AXIS_Y]/firlen;
				data[STK8BAXX_AXIS_Z] = obj->fir.sum[STK8BAXX_AXIS_Z]/firlen;
				if(atomic_read(&obj->trace) & ADX_TRC_FILTER) {
					GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", 
							idx, obj->fir.raw[idx][STK8BAXX_AXIS_X], 
							obj->fir.raw[idx][STK8BAXX_AXIS_Y], 
							obj->fir.raw[idx][STK8BAXX_AXIS_Z],
					        obj->fir.sum[STK8BAXX_AXIS_X], 
							obj->fir.sum[STK8BAXX_AXIS_Y], 
							obj->fir.sum[STK8BAXX_AXIS_Z],
					        data[STK8BAXX_AXIS_X], data[STK8BAXX_AXIS_Y], 
							data[STK8BAXX_AXIS_Z]);
				}
			}
		}
	}
#endif
#ifdef STK_ZG_FILTER
	if( abs(data[STK8BAXX_AXIS_X]) <= STK_ZG_COUNT)
		data[STK8BAXX_AXIS_X] *= zero_fir;
	if( abs(data[STK8BAXX_AXIS_Y]) <= STK_ZG_COUNT)
		data[STK8BAXX_AXIS_Y] *= zero_fir;
	if( abs(data[STK8BAXX_AXIS_Z]) <= STK_ZG_COUNT)
		data[STK8BAXX_AXIS_Z] *= zero_fir;
#endif

	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_SetOffset(struct i2c_client *client, char buf[])
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}

	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTX, buf[0]);
	if(err < 0) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTY, buf[1]);
	if(err < 0) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTZ, buf[2]);
	if(err < 0) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}
	msleep(12);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int STK8BAXX_ReadOffset(struct i2c_client *client, s8 ofs[STK8BAXX_AXES_NUM])
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}

	err = stk8baxx_hwmsen_read_block(client,STK8BAXX_OFSTFILTX, ofs, 3);
	if (err < 0) {
		GSE_LOG(KERN_ERR "failed to read acc data, error=%d\n", err);
		return err;
	}

	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}

	return err;
}
/*---------------------------------------------------------------------------*/
static int STK8BAXX_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf;
	int res = 0;
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int k_status = atomic_read(&obj->cali_status);

	printk("%s, enable=%2d\n", __func__, enable?1:0);
	
	if(enable == obj->sensor_power) {
		GSE_LOG("Sensor power status need not to be set again!!!\n");
		return STK8BAXX_SUCCESS;
	}

	if( (atomic_read(&obj->suspend)==1) && (enable==true)) {
		obj->re_enable = 1;
		GSE_LOG("IN suspend status, Sensor power status need not to be set!!!\n");
		return STK8BAXX_SUCCESS;
	}

	if(obj->first_enable && k_status != STK_K_RUNNING && enable == true) {
		obj->first_enable = false;
#ifdef STK_TUNE
		stk_handle_first_en(client);
#endif
	}
	
	if(enable == true) {
		res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTX, obj->stk_tune_offset_record[0]);
		if(res < 0) {
			GSE_ERR("write offset fail: %d\n", res);
			return res;
		}
		res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTY, obj->stk_tune_offset_record[1]);
		if(res < 0) {
			GSE_ERR("write offset fail: %d\n", res);
			return res;
		}
		res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTZ, obj->stk_tune_offset_record[2]);
		if(res < 0) {
			GSE_ERR("write offset fail: %d\n", res);
			return res;
		}
		printk("%s:write offset[0] = %d, offset[1] = %d, offset[2] = %d\n",__func__, obj->stk_tune_offset_record[0],obj->stk_tune_offset_record[1], obj->stk_tune_offset_record[2]);

		atomic_set(&obj->event_since_en, 0);
		databuf = STK8BAXX_MD_NORMAL;
#ifdef STK_TUNE
		if((k_status&0xF0) != 0 && obj->stk_tune_done == 0) {
			obj->stk_tune_index = 0;
			STK8BAXX_ResetPara(obj);
		}
#endif
	} else {
		databuf = STK8BAXX_MD_SUSPEND;
	}
	
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, databuf);
	if(res < 0) {
		GSE_LOG("set power mode failed!\n");
		return STK8BAXX_ERR_I2C;
	} else if(atomic_read(&obj->trace) & ADX_TRC_INFO) {
		GSE_LOG("set power mode ok %d!\n", databuf);
	}
	
	udelay(500);

	obj->sensor_power = enable;

	return STK8BAXX_SUCCESS;

}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_ResetCalibration(struct i2c_client *client)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	//bool old_sensorpowmode = obj->sensor_power;
	GSE_FUN();
	
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}

	//STK8BAXX_SetPowerMode(obj->client,false);
	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTX, 0);
	if(err < 0) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTY, 0);
	if(err < 0) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTFILTZ, 0);
	if(err < 0) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}

	//if(old_sensorpowmode == true)
	//	STK8BAXX_SetPowerMode(obj->client,true);
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}
	msleep(12);
	return err;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_ReadCalibration(struct i2c_client *client, int dat[STK8BAXX_AXES_NUM])
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	
	GSE_FUN();
	
	if ((err = STK8BAXX_ReadOffset(client, obj->offset))) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
	
	if(stk8baxx_offset_resolution.sensitivity > obj->reso->sensitivity) {
		mul = stk8baxx_offset_resolution.sensitivity/obj->reso->sensitivity;
		dat[obj->cvt.map[STK8BAXX_AXIS_X]] = obj->cvt.sign[STK8BAXX_AXIS_X]*(obj->offset[STK8BAXX_AXIS_X]/mul);
		dat[obj->cvt.map[STK8BAXX_AXIS_Y]] = obj->cvt.sign[STK8BAXX_AXIS_Y]*(obj->offset[STK8BAXX_AXIS_Y]/mul);
		dat[obj->cvt.map[STK8BAXX_AXIS_Z]] = obj->cvt.sign[STK8BAXX_AXIS_Z]*(obj->offset[STK8BAXX_AXIS_Z]/mul);
	} else {
		mul = obj->reso->sensitivity / stk8baxx_offset_resolution.sensitivity;
		dat[obj->cvt.map[STK8BAXX_AXIS_X]] = obj->cvt.sign[STK8BAXX_AXIS_X]*(obj->offset[STK8BAXX_AXIS_X]*mul);
		dat[obj->cvt.map[STK8BAXX_AXIS_Y]] = obj->cvt.sign[STK8BAXX_AXIS_Y]*(obj->offset[STK8BAXX_AXIS_Y]*mul);
		dat[obj->cvt.map[STK8BAXX_AXIS_Z]] = obj->cvt.sign[STK8BAXX_AXIS_Z]*(obj->offset[STK8BAXX_AXIS_Z]*mul);
	}
	GSE_LOG("read cali offX=%x ,offY=%x ,offZ=%x\n",obj->offset[STK8BAXX_AXIS_X],obj->offset[STK8BAXX_AXIS_Y],obj->offset[STK8BAXX_AXIS_Z]);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_ReadCalibrationEx(struct i2c_client *client, int act[STK8BAXX_AXES_NUM], int raw[STK8BAXX_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;
	
	GSE_FUN();
	
	if((err = STK8BAXX_ReadOffset(client, obj->offset))) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	if(stk8baxx_offset_resolution.sensitivity > obj->reso->sensitivity) {
		mul = stk8baxx_offset_resolution.sensitivity/obj->reso->sensitivity;
		raw[STK8BAXX_AXIS_X] = obj->offset[STK8BAXX_AXIS_X]/mul + obj->cali_sw[STK8BAXX_AXIS_X];
		raw[STK8BAXX_AXIS_Y] = obj->offset[STK8BAXX_AXIS_Y]/mul + obj->cali_sw[STK8BAXX_AXIS_Y];
		raw[STK8BAXX_AXIS_Z] = obj->offset[STK8BAXX_AXIS_Z]/mul + obj->cali_sw[STK8BAXX_AXIS_Z];
	} else {
		mul = obj->reso->sensitivity / stk8baxx_offset_resolution.sensitivity;
		raw[STK8BAXX_AXIS_X] = obj->offset[STK8BAXX_AXIS_X]*mul + obj->cali_sw[STK8BAXX_AXIS_X];
		raw[STK8BAXX_AXIS_Y] = obj->offset[STK8BAXX_AXIS_Y]*mul + obj->cali_sw[STK8BAXX_AXIS_Y];
		raw[STK8BAXX_AXIS_Z] = obj->offset[STK8BAXX_AXIS_Z]*mul + obj->cali_sw[STK8BAXX_AXIS_Z];
	}
	/*
		mul = stk8baxx_offset_resolution.sensitivity/obj->reso->sensitivity;
		raw[STK8BAXX_AXIS_X] = obj->offset[STK8BAXX_AXIS_X]/mul + obj->cali_sw[STK8BAXX_AXIS_X];
		raw[STK8BAXX_AXIS_Y] = obj->offset[STK8BAXX_AXIS_Y]/mul + obj->cali_sw[STK8BAXX_AXIS_Y];
		raw[STK8BAXX_AXIS_Z] = obj->offset[STK8BAXX_AXIS_Z]/mul + obj->cali_sw[STK8BAXX_AXIS_Z];
	*/
	act[obj->cvt.map[STK8BAXX_AXIS_X]] = obj->cvt.sign[STK8BAXX_AXIS_X]*raw[STK8BAXX_AXIS_X];
	act[obj->cvt.map[STK8BAXX_AXIS_Y]] = obj->cvt.sign[STK8BAXX_AXIS_Y]*raw[STK8BAXX_AXIS_Y];
	act[obj->cvt.map[STK8BAXX_AXIS_Z]] = obj->cvt.sign[STK8BAXX_AXIS_Z]*raw[STK8BAXX_AXIS_Z];

	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_SetBWSEL(struct i2c_client *client, u8 dataformat)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0, err;
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}

	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_BWSEL, STK8BAXX_INIT_ODR);
	if(res < 0) {
		return STK8BAXX_ERR_I2C;
	}
	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}

	}
	return 0;
}
/*----------------------------------------------------------------------------*/

static int STK8BAXX_SetDelay(struct i2c_client *client, uint32_t sdelay_ns)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	unsigned char sr_no;
	int result;

	uint32_t sdelay_us = sdelay_ns / 1000;

#ifdef STK8BAXX_HOLD_ODR
	for(sr_no=0; sr_no<STK8BAXX_SPTIME_NO; sr_no++) {
		if(sdelay_us >= STK8BAXX_SAMPLE_TIME[sr_no])
			break;
	}

	if(sr_no == 0) {
		sdelay_ns = STK8BAXX_SAMPLE_TIME[0]*1000;
	} else if(sr_no == (STK8BAXX_SPTIME_NO)) {
		sdelay_ns = STK8BAXX_SAMPLE_TIME[STK8BAXX_SPTIME_NO-1]*1000;
	}
	sr_no += STK8BAXX_SPTIME_BASE;
#else
	sr_no = STK8BAXX_INIT_ODR;
#endif
	GSE_LOG("sdelay_ns=%ud, sr_no=%d\n", sdelay_ns, sr_no);
	result = STK8BAXX_SetBWSEL(client,sr_no);

	if (result < 0) {
		GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_BWSEL, result);
		return result;
	}
	obj->gsensor_delay = sdelay_us;
	return result;
}
/*----------------------------------------------------------------------------*/

static s64 STK8BAXX_GetDelay(struct stk8baxx_i2c_data *obj)
{
	return obj->gsensor_delay;
}

/*----------------------------------------------------------------------------*/
static int STK8BAXX_VerifyCali(struct stk8baxx_i2c_data *obj, uint32_t delay_ms)
{
	unsigned char axis, state;
	int acc_ave[3] = {0, 0, 0};
	const unsigned char verify_sample_no = 3;
	const unsigned char verify_diff = 25;
	int ret = 0;

	msleep(delay_ms);
	for(state=0; state<verify_sample_no; state++) {
		msleep(delay_ms);
		STK8BAXX_ReadData(obj->client, obj->data);
		acc_ave[0] += obj->data[STK8BAXX_AXIS_X];
		acc_ave[1] += obj->data[STK8BAXX_AXIS_Y];
		acc_ave[2] += obj->data[STK8BAXX_AXIS_Z];
#ifdef STK_DEBUG_CALI
		GSE_LOG(" acc=%d,%d,%d\n", obj->data[STK8BAXX_AXIS_X], obj->data[STK8BAXX_AXIS_Y], obj->data[STK8BAXX_AXIS_Z]);
#endif
	}

	for(axis=0; axis<3; axis++)
		acc_ave[axis] /= verify_sample_no;

	if(obj->cvt.sign[STK8BAXX_AXIS_X] >0)
		acc_ave[2] -= STK_LSB_1G;
	else
		acc_ave[2] += STK_LSB_1G;

	if(abs(acc_ave[0]) > verify_diff || abs(acc_ave[1]) > verify_diff || abs(acc_ave[2]) > verify_diff) {
		GSE_LOG("Check data x:%d, y:%d, z:%d\n",acc_ave[0],acc_ave[1],acc_ave[2]);
		GSE_ERR("Check Fail, Calibration Fail\n");
		ret = -STK_K_FAIL_LRG_DIFF;
	}
#ifdef STK_DEBUG_CALI
	else {
		GSE_LOG("Check data pass\n");
	}
#endif

	return ret;
}


static int STK8BAXX_SetCaliDo(struct stk8baxx_i2c_data *obj, unsigned int delay_ms)
{
	int sample_no, axis;
	int acc_ave[3] = {0, 0, 0};
	u8 offset[3];
	u8 offset_in_reg[3];
	int result;

	msleep(delay_ms*3);
	for(sample_no=0; sample_no<STK_SAMPLE_NO; sample_no++) {
		msleep(delay_ms);
		STK8BAXX_ReadData(obj->client, obj->data);
		acc_ave[0] += obj->data[STK8BAXX_AXIS_X];
		acc_ave[1] += obj->data[STK8BAXX_AXIS_Y];
		acc_ave[2] += obj->data[STK8BAXX_AXIS_Z];
#ifdef STK_DEBUG_CALI
		GSE_LOG(" acc=%d,%d,%d\n", obj->data[STK8BAXX_AXIS_X], obj->data[STK8BAXX_AXIS_Y], obj->data[STK8BAXX_AXIS_Z]);
#endif
	}

	for(axis=0; axis<3; axis++) {
		if(acc_ave[axis] >= 0)
			acc_ave[axis] = (acc_ave[axis] + STK_SAMPLE_NO / 2) / STK_SAMPLE_NO;
		else
			acc_ave[axis] = (acc_ave[axis] - STK_SAMPLE_NO / 2) / STK_SAMPLE_NO;
	}

	GSE_LOG(" z axis cvt.sign =%d", obj->cvt.sign[STK8BAXX_AXIS_X]);
	if(obj->cvt.sign[STK8BAXX_AXIS_X] >0)
		acc_ave[2] -= STK_LSB_1G;
	else
		acc_ave[2] += STK_LSB_1G;

	STK8BAXX_SetCaliScaleOfst(obj->client, acc_ave);

	for(axis=0; axis<3; axis++) {
		offset[axis] = -acc_ave[axis];
	}
	GSE_LOG( "New offset for reg:%d,%d,%d\n", offset[0], offset[1], offset[2]);

	STK8BAXX_SetOffset(obj->client, offset);
	result = stk8baxx_hwmsen_read_block(obj->client,STK8BAXX_OFSTFILTX, offset_in_reg, 3);
	if (result < 0) {
		GSE_ERR(" failed to read offset data, error=%d\n", result);
		return result;
	}

	for(axis=0; axis<3; axis++) {
		if(offset[axis] != offset_in_reg[axis]) {
			GSE_ERR(" set offset to register fail!, offset[%d]=%d,offset_in_reg[%d]=%d\n",
			       axis,offset[axis], axis, offset_in_reg[axis]);
			atomic_set(&obj->cali_status, STK_K_FAIL_WRITE_NOFST);
			return -STK_K_FAIL_WRITE_NOFST;
		}
	}

	result = STK8BAXX_VerifyCali(obj, delay_ms);
	if(result) {
		GSE_ERR(" calibration check fail, result=0x%x\n", result);
		atomic_set(&obj->cali_status, -result);
		return result;
	}

	result = stk8baxx_store_in_file(offset, STK_K_SUCCESS_FILE, NULL);
	if (result < 0) {
		GSE_ERR("failed to stk8baxx_store_in_file, error=%d\n", result);
		atomic_set(&obj->cali_status, STK_K_FAIL_W_FILE);
		return result;
	}
	atomic_set(&obj->cali_status, STK_K_SUCCESS_FILE);
#ifdef STK_TUNE
	obj->stk_tune_offset_record[0] = 0;
	obj->stk_tune_offset_record[1] = 0;
	obj->stk_tune_offset_record[2] = 0;
	obj->stk_tune_done = 1;
#endif
	return 0;
}

static int STK8BAXX_SetCali(struct i2c_client *client)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int result;
	s64 ord_delay;
	bool org_sensor_power;
	uint32_t real_delay_ms;

	GSE_FUN();

	atomic_set(&obj->cali_status, STK_K_RUNNING);
	org_sensor_power = obj->sensor_power;

	if(!obj->sensor_power) {
		result = STK8BAXX_SetPowerMode(client,true);
		if (result < 0) {
			GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_POWMODE, result);
			return result;
		}
	}
	ord_delay = STK8BAXX_GetDelay(obj);
	STK8BAXX_SetDelay(client, 8000000);
	real_delay_ms = STK8BAXX_GetDelay(obj);
	real_delay_ms /= USEC_PER_MSEC;

	result = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_OFSTCOMP1, CAL_OFST_RST);
	if(result < 0) {
		GSE_ERR("write offset fail: %d\n", result);
		goto k_exit;
	}

	result = STK8BAXX_SetCaliDo(obj, (unsigned int)real_delay_ms);
	if (result < 0) {
		GSE_ERR( "failed to STK8BAXX_SetCaliDo, error=%d\n", result);
		atomic_set(&obj->cali_status, -result);
		goto k_exit;
	}

	STK8BAXX_SetDelay(client, ord_delay);
	if(!org_sensor_power) {
		result = STK8BAXX_SetPowerMode(client,false);
		if (result < 0) {
			GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_POWMODE, result);
			goto k_exit;
		}
	}
	GSE_LOG("STK8BAXX_SetCali successfully");
	return 0;

k_exit:
	STK8BAXX_SetDelay(client, ord_delay);
	if(!org_sensor_power) {
		result = STK8BAXX_SetPowerMode(client,false);
		if (result < 0) {
			GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_POWMODE, result);
			return result;
		}
	}

	return result;
}

/*----------------------------------------------------------------------------*/
static int STK8BAXX_WriteCalibration(struct i2c_client *client, int dat[STK8BAXX_AXES_NUM])
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[STK8BAXX_AXES_NUM], raw[STK8BAXX_AXES_NUM];
	int lsb = stk8baxx_offset_resolution.sensitivity;
	int divisor;
	
	printk("%s:write cali data dat[0] = %d,dat[1] = %d,dat[2] = %d\n",__func__,dat[0],dat[1],dat[2]);
	if((err = STK8BAXX_ReadCalibrationEx(client, cali, raw))) {	/*offset will be updated in obj->offset*/
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	        raw[STK8BAXX_AXIS_X], raw[STK8BAXX_AXIS_Y], raw[STK8BAXX_AXIS_Z],
	        obj->offset[STK8BAXX_AXIS_X], obj->offset[STK8BAXX_AXIS_Y], obj->offset[STK8BAXX_AXIS_Z],
	        obj->cali_sw[STK8BAXX_AXIS_X], obj->cali_sw[STK8BAXX_AXIS_Y], obj->cali_sw[STK8BAXX_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[STK8BAXX_AXIS_X] += dat[STK8BAXX_AXIS_X];
	cali[STK8BAXX_AXIS_Y] += dat[STK8BAXX_AXIS_Y];
	cali[STK8BAXX_AXIS_Z] += dat[STK8BAXX_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n",
	        dat[STK8BAXX_AXIS_X], dat[STK8BAXX_AXIS_Y], dat[STK8BAXX_AXIS_Z]);


	if(lsb > obj->reso->sensitivity) {
		divisor = lsb/obj->reso->sensitivity;
		obj->offset[STK8BAXX_AXIS_X] = (s8)(obj->cvt.sign[STK8BAXX_AXIS_X]*(cali[obj->cvt.map[STK8BAXX_AXIS_X]])*(divisor));
		obj->offset[STK8BAXX_AXIS_Y] = (s8)(obj->cvt.sign[STK8BAXX_AXIS_Y]*(cali[obj->cvt.map[STK8BAXX_AXIS_Y]])*(divisor));
		obj->offset[STK8BAXX_AXIS_Z] = (s8)(obj->cvt.sign[STK8BAXX_AXIS_Z]*(cali[obj->cvt.map[STK8BAXX_AXIS_Z]])*(divisor));

		/*convert software calibration using standard calibration*/
		obj->cali_sw[STK8BAXX_AXIS_X] = obj->cvt.sign[STK8BAXX_AXIS_X]*(cali[obj->cvt.map[STK8BAXX_AXIS_X]])%(divisor);
		obj->cali_sw[STK8BAXX_AXIS_Y] = obj->cvt.sign[STK8BAXX_AXIS_Y]*(cali[obj->cvt.map[STK8BAXX_AXIS_Y]])%(divisor);
		obj->cali_sw[STK8BAXX_AXIS_Z] = obj->cvt.sign[STK8BAXX_AXIS_Z]*(cali[obj->cvt.map[STK8BAXX_AXIS_Z]])%(divisor);
	} else {
		divisor = obj->reso->sensitivity / lsb;

		obj->offset[STK8BAXX_AXIS_X] = (s8)(obj->cvt.sign[STK8BAXX_AXIS_X]*(cali[obj->cvt.map[STK8BAXX_AXIS_X]])/(divisor));
		obj->offset[STK8BAXX_AXIS_Y] = (s8)(obj->cvt.sign[STK8BAXX_AXIS_Y]*(cali[obj->cvt.map[STK8BAXX_AXIS_Y]])/(divisor));
		obj->offset[STK8BAXX_AXIS_Z] = (s8)(obj->cvt.sign[STK8BAXX_AXIS_Z]*(cali[obj->cvt.map[STK8BAXX_AXIS_Z]])/(divisor));

		/*convert software calibration using standard calibration*/
		/*
		obj->cali_sw[STK8BAXX_AXIS_X] = obj->cvt.sign[STK8BAXX_AXIS_X]*(cali[obj->cvt.map[STK8BAXX_AXIS_X]])%(divisor);
		obj->cali_sw[STK8BAXX_AXIS_Y] = obj->cvt.sign[STK8BAXX_AXIS_Y]*(cali[obj->cvt.map[STK8BAXX_AXIS_Y]])%(divisor);
		obj->cali_sw[STK8BAXX_AXIS_Z] = obj->cvt.sign[STK8BAXX_AXIS_Z]*(cali[obj->cvt.map[STK8BAXX_AXIS_Z]])%(divisor);
		*/
	}

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	        obj->offset[STK8BAXX_AXIS_X] + obj->cali_sw[STK8BAXX_AXIS_X],
	        obj->offset[STK8BAXX_AXIS_Y] + obj->cali_sw[STK8BAXX_AXIS_Y],
	        obj->offset[STK8BAXX_AXIS_Z] + obj->cali_sw[STK8BAXX_AXIS_Z],
	        obj->offset[STK8BAXX_AXIS_X], obj->offset[STK8BAXX_AXIS_Y], obj->offset[STK8BAXX_AXIS_Z],
	        obj->cali_sw[STK8BAXX_AXIS_X], obj->cali_sw[STK8BAXX_AXIS_Y], obj->cali_sw[STK8BAXX_AXIS_Z]);

	msleep(1);

	STK8BAXX_SetOffset(client, obj->offset);
  //get MTK standard Calibration parameters and write to offset registeres
	err = stk8baxx_i2c_smbus_write_byte_data(obj->client, STK8BAXX_OFSTFILTX, obj->offset[0]);
	if(err < 0)
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	err = stk8baxx_i2c_smbus_write_byte_data(obj->client, STK8BAXX_OFSTFILTY, obj->offset[1]);
	if(err < 0)
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
	err = stk8baxx_i2c_smbus_write_byte_data(obj->client, STK8BAXX_OFSTFILTZ, obj->offset[2]);
	if(err < 0)
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}	

	obj->stk_tune_offset_record[0] = obj->offset[0];
	obj->stk_tune_offset_record[1] = obj->offset[1];
	obj->stk_tune_offset_record[2] = obj->offset[2];
	printk("%s:write offset[0] = %d, offset[1] = %d, offset[2] = %d\n",__func__, obj->stk_tune_offset_record[0],obj->stk_tune_offset_record[1], obj->stk_tune_offset_record[2]);

	msleep(12);

	return err;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_SetReset(struct i2c_client *client)
{
	int res = 0;

	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_SWRST, STK8BAXX_SWRST_VAL);
	if(res < 0) {
		GSE_LOG("issue sw reset to 0x%x, result=%d\n", client->addr, res);
	}
	msleep(1);

	return STK8BAXX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
//set detect range
/*
static int STK8BAXX_SetDataRange(struct i2c_client *client, u8 dataformat)
{

	u8 databuf[2];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);
	databuf[0] = STK8BAXX_RANGESEL;
	databuf[1] = dataformat;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return STK8BAXX_ERR_I2C;
	}

	return 0;

}
*/
/*----------------------------------------------------------------------------*/
/*
static int STK8BAXX_SetDataFormat(struct i2c_client *client, u8 dataformat)
{

	u8 databuf[2];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2);
	databuf[0] = STK8BAXX_RANGESEL;
	databuf[1] = dataformat;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return STK8BAXX_ERR_I2C;
	}

	return 0;

}
*/
/*----------------------------------------------------------------------------*/
static int STK8BAXX_SetWatchDog(struct i2c_client *client, u8 intenable)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_INTFCFG, intenable);
	if(err < 0)
		return STK8BAXX_ERR_I2C;

	if(!obj->sensor_power) {
		err = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_POWMODE, STK8BAXX_MD_SUSPEND);
		if(err < 0) {
			GSE_LOG("set power mode failed!\n");
			return STK8BAXX_ERR_I2C;
		}
	}

	return STK8BAXX_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int STK8BAXX_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	struct stk8baxx_i2c_data *obj = (struct stk8baxx_i2c_data*)i2c_get_clientdata(client);

	if((NULL == buf)||(bufsize<=30)) {
		return -1;
	}

	if(NULL == client) {
		*buf = 0;
		return -2;
	}

	if(obj->pid == STK8BA50_ID)
		snprintf(buf, bufsize, "stk8ba50\n");
	else if(obj->pid == STK8BA50R_ID)
		snprintf(buf, bufsize, "stk8ba50r\n");
	else if(obj->pid == STK8BA53_ID)
		snprintf(buf, bufsize, "stk8ba53\n");

	return 0;
}

/*----------------------------------------------------------------------------*/

static int STK8BAXX_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct stk8baxx_i2c_data *obj = (struct stk8baxx_i2c_data*)i2c_get_clientdata(client);
	int acc[STK8BAXX_AXES_NUM];
	int res = 0;

	if(NULL == buf) {
		return -1;
	}
	if(NULL == client) {
		*buf = 0;
		return -2;
	}
	
	if(obj->sensor_power == false) {
		res = STK8BAXX_SetPowerMode(client, true);
		if(res) {
			GSE_ERR("Power on stk8baxx error %d!\n", res);
		}
	}

	if((res = STK8BAXX_ReadData(client, obj->data))) {
		return -3;
	} else {
		obj->data[STK8BAXX_AXIS_X] += obj->cali_sw[STK8BAXX_AXIS_X];
		obj->data[STK8BAXX_AXIS_Y] += obj->cali_sw[STK8BAXX_AXIS_Y];
		obj->data[STK8BAXX_AXIS_Z] += obj->cali_sw[STK8BAXX_AXIS_Z];
		//GSE_ERR("Mapped gsensor cali_sw: %d, %d, %d!\n", obj->cali_sw[STK8BAXX_AXIS_X], obj->cali_sw[STK8BAXX_AXIS_Y], obj->cali_sw[STK8BAXX_AXIS_Z]);
		/*remap coordinate*/
		//GSE_ERR("Mapped gsensor cali_sw: %d, %d, %d!\n", obj->cvt.sign[STK8BAXX_AXIS_X], obj->cvt.sign[STK8BAXX_AXIS_Y], obj->cvt.sign[STK8BAXX_AXIS_Z]);
		acc[obj->cvt.map[STK8BAXX_AXIS_X]] = obj->cvt.sign[STK8BAXX_AXIS_X]*obj->data[STK8BAXX_AXIS_X];
		acc[obj->cvt.map[STK8BAXX_AXIS_Y]] = obj->cvt.sign[STK8BAXX_AXIS_Y]*obj->data[STK8BAXX_AXIS_Y];
		acc[obj->cvt.map[STK8BAXX_AXIS_Z]] = obj->cvt.sign[STK8BAXX_AXIS_Z]*obj->data[STK8BAXX_AXIS_Z];

		// GSE_ERR("Mapped gsensor data: %d, %d, %d!\n", acc[STK8BAXX_AXIS_X], acc[STK8BAXX_AXIS_Y], acc[STK8BAXX_AXIS_Z]);

		//Out put the mg
		acc[STK8BAXX_AXIS_X] = acc[STK8BAXX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[STK8BAXX_AXIS_Y] = acc[STK8BAXX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[STK8BAXX_AXIS_Z] = acc[STK8BAXX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

		if(atomic_read(&obj->event_since_en) < 1200)
			atomic_inc(&obj->event_since_en);
		
		if(atomic_read(&obj->event_since_en) < atomic_read(&obj->event_since_en_limit)) {
			sprintf(buf, "%04x %04x %04x", 0, 0, 9810);
			GSE_LOG("event_since_en=%d,event_since_en_limit=%d\n", 
				atomic_read(&obj->event_since_en), atomic_read(&obj->event_since_en_limit));			
			
			if(atomic_read(&obj->trace) & ADX_TRC_IOCTL) {
				GSE_LOG("fixed gsensor data: %s!\n", buf);		
			}				
			return 0;
		}
		sprintf(buf, "%04x %04x %04x", acc[STK8BAXX_AXIS_X], acc[STK8BAXX_AXIS_Y], acc[STK8BAXX_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL) {
			GSE_LOG("gsensor data: %s!\n", buf);
			// GSE_LOG("gsensor data:  sensitivity x=%d \n",gsensor_gain.z);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_ReadRawData(struct i2c_client *client, char *buf)
{
	struct stk8baxx_i2c_data *obj = (struct stk8baxx_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client) {
		return EINVAL;
	}

	/*
	if(obj->sensor_power == false)
	{
		res = STK8BAXX_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on stk8baxx error %d!\n", res);
		}
	}
	*/

	if((res = STK8BAXX_ReadData(client, obj->data))) {
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	} else {
		sprintf(buf, "%04x %04x %04x", obj->data[STK8BAXX_AXIS_X],
		        obj->data[STK8BAXX_AXIS_Y], obj->data[STK8BAXX_AXIS_Z]);

	}
	// GSE_LOG("gsensor data: %s!\n", buf);
	return 0;
}
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
int STK8BAXX_SCP_SetPowerMode(bool enable, int sensorType)
{
	static bool gsensor_scp_en_status;
	static unsigned int gsensor_scp_en_map;
	SCP_SENSOR_HUB_DATA req;
	int len;
	int err = 0;

	mutex_lock(&gsensor_scp_en_mutex);

	if (sensorType >= 32) {
		GSE_ERR("Out of index!\n");
		return -1;
	}

	if (true == enable)
		gsensor_scp_en_map |= (1 << sensorType);
	else
		gsensor_scp_en_map &= ~(1 << sensorType);

	if (0 == gsensor_scp_en_map)
		enable = false;
	else
		enable = true;

	if (gsensor_scp_en_status != enable) {
		gsensor_scp_en_status = enable;

		req.activate_req.sensorType = ID_ACCELEROMETER;
		req.activate_req.action = SENSOR_HUB_ACTIVATE;
		req.activate_req.enable = enable;
		len = sizeof(req.activate_req);
		err = SCP_sensorHub_req_send(&req, &len, 1);
		if (err)
			GSE_ERR("SCP_sensorHub_req_send fail!\n");
	}

	mutex_unlock(&gsensor_scp_en_mutex);

	return err;
}
EXPORT_SYMBOL(STK8BAXX_SCP_SetPowerMode);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
/*----------------------------------------------------------------------------*/
static int STK8BAXX_ShowAllReg(char *show_buffer)
{
	struct stk8baxx_i2c_data *obj = obj_i2c_data;
	u8 buffer[8] = "";
	int aa,bb, no;
	int result;
	bool org_sensor_power;
	org_sensor_power = obj->sensor_power;
	int len = 0;
	
	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(!obj->sensor_power) {
		result = STK8BAXX_SetPowerMode(obj->client,true);
		if (result < 0) {
			GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_POWMODE, result);
			return result;
		}
	}

	for(bb=0; bb<=8; bb++) {
		result = stk8baxx_hwmsen_read_block(obj->client, bb*8, buffer, 8);
		if (result < 0) {
			GSE_ERR(" failed to read acc data, error=%d\n", result);
			return result;
		}

		for(aa=0; aa<8; aa++) {
			no = bb * 8 + aa;
			GSE_LOG("stk reg[0x%x]=0x%x\n", no, buffer[aa]);
			if(show_buffer != NULL)
				len += snprintf(show_buffer+len, PAGE_SIZE-len, "0x%02x,", buffer[aa]);
		}
	}
	
	if(show_buffer != NULL)
		len += snprintf(show_buffer+len, PAGE_SIZE-len, "\n");
			
	if(!org_sensor_power) {
		result = STK8BAXX_SetPowerMode(obj->client,false);
		if (result < 0) {
			GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_POWMODE, result);
			return result;
		}
	}
	
	return len;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_CheckID(struct stk8baxx_i2c_data *obj)
{
	int res = 0;	
	
	res = i2c_smbus_read_byte_data(obj->client, STK8BAXX_LGDLY);
	if(res < 0) {
		GSE_ERR("i2c transfer failed, err=%d\n", res);
		return res;
	}
	
	if(res == STK8BA50_ID) {
		GSE_LOG("real chip is stk8ba50\n");
		obj->pid = STK8BA50_ID;
	} else {
		res = i2c_smbus_read_byte_data(obj->client, 0x0);
		if(res < 0)	{
			GSE_ERR("failed to read acc data, error=%d\n", res);
			return res;
		}
		GSE_LOG("0x0=0x%x\n", res);
		
		if(res == STK8BA50R_ID) {
			GSE_LOG("real chip is stk8ba50-R\n");
			obj->pid = STK8BA50R_ID;
		} else {
			GSE_LOG("real chip is stk8ba53\n");
			obj->pid = STK8BA53_ID;
		}
	}

#ifdef CONFIG_SENSORS_STK8BA53
	if(obj->pid != STK8BA53_ID) {
		GSE_ERR("stk8ba53 is not attached, pid=0x%x\n", obj->pid);
		return -ENODEV;
	}
#else
	if(obj->pid != STK8BA50_ID && obj->pid != STK8BA50R_ID) {
		GSE_ERR("stk8ba50/stk8ba50-R is not attached, pid=0x%x\n", obj->pid);
		return -ENODEV;
	}
#endif

	return STK8BAXX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int STK8BAXX_TestOffsetNoise(void)
{
	int res;
	uint32_t real_delay_ms;
	unsigned int sample_no;
	int acc_ave[3] = {0, 0, 0};
	int acc_max[3] = {INT_MIN, INT_MIN, INT_MIN};
	int acc_min[3] = {INT_MAX, INT_MAX, INT_MAX};
	int noise[3] = {0, 0, 0};
	int axis;
	int err_code = 0;

	res = STK8BAXX_SetReset(obj_i2c_data->client);
	if(res != STK8BAXX_SUCCESS) {
		GSE_ERR("stk8baxx set reset error\n");
		return res;
	}

	res = STK8BAXX_SetDelay(obj_i2c_data->client, 8000000);	
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set delay error\n");
		return res;
	}
	
	res = stk8baxx_i2c_smbus_write_byte_data(obj_i2c_data->client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
	if(res < 0) {
		GSE_LOG("set power mode failed!\n");
		return res;
	}
	
	res = STK8BAXX_SetDataResolution(obj_i2c_data->client, STK8BAXX_RNG_2G);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set data reslution error\n");
		return res;
	}
	
	real_delay_ms = STK8BAXX_GetDelay(obj_i2c_data);
	real_delay_ms /= USEC_PER_MSEC;

	msleep(real_delay_ms * (STK_EVENT_SINCE_EN_LIMIT_DEF + 3));
	
	for(sample_no =0; sample_no < STK_SELFTEST_SAMPLE_NO; sample_no++) {
		msleep(real_delay_ms);
		STK8BAXX_ReadData(obj_i2c_data->client, obj_i2c_data->data);
		GSE_LOG("%s: acc=%d,%d,%d\n", __func__, obj_i2c_data->data[STK8BAXX_AXIS_X], obj_i2c_data->data[STK8BAXX_AXIS_Y], obj_i2c_data->data[STK8BAXX_AXIS_Z]);
		
		for(axis=0; axis<3; axis++) {
			acc_ave[axis] += obj_i2c_data->data[axis];
			
			if(obj_i2c_data->data[axis] > acc_max[axis])
				 acc_max[axis] = obj_i2c_data->data[axis];
			 
			if(obj_i2c_data->data[axis] < acc_min[axis])
				 acc_min[axis] = obj_i2c_data->data[axis];				 
		}
	}

	for(axis=0; axis<3; axis++) {
		if(acc_ave[axis] >= 0)
			acc_ave[axis] = (acc_ave[axis] + STK_SELFTEST_SAMPLE_NO / 2) / STK_SELFTEST_SAMPLE_NO;
		else
			acc_ave[axis] = (acc_ave[axis] - STK_SELFTEST_SAMPLE_NO / 2) / STK_SELFTEST_SAMPLE_NO;
		
		noise[axis] = acc_max[axis] - acc_min[axis];
	}

	GSE_LOG(" z axis cvt.sign =%d\n", obj_i2c_data->cvt.sign[STK8BAXX_AXIS_X]);
	GSE_LOG("acc_ave=%d,%d,%d,noise=%d,%d,%d\n", acc_ave[0], acc_ave[1], acc_ave[2], noise[0], noise[1], noise[2]);
	
	if (acc_ave[0] == 0 && acc_ave[1] == 0 && acc_ave[2] == 0) {
		GSE_ERR("NO_OUTPUT\n");
		err_code |= (STK_SELFTEST_STAT_FAIL_X | STK_SELFTEST_STAT_FAIL_Y | STK_SELFTEST_STAT_FAIL_Z);
	}

	if (abs(acc_ave[0]) >= STK_SELFTEST_SATU) {
		GSE_ERR("X_OFFSET_SATURATION, %d\n", abs(acc_ave[0]));
		err_code |= STK_SELFTEST_STAT_FAIL_X;
	}

	if (abs(acc_ave[1]) >= STK_SELFTEST_SATU) {
		GSE_ERR("Y_OFFSET_SATURATION, %d\n", abs(acc_ave[1]));
		err_code |= STK_SELFTEST_STAT_FAIL_Y;
	}

	if (abs(acc_ave[2]) >= STK_SELFTEST_SATU) {
		GSE_ERR("Z_OFFSET_SATURATION, %d\n", abs(acc_ave[2]));
		err_code |= STK_SELFTEST_STAT_FAIL_Z;
	}

	if (noise[0] == 0) {
		GSE_ERR("X_INVALID\n");
		err_code |= STK_SELFTEST_STAT_FAIL_X;
	}

	if (noise[1] == 0) {
		GSE_ERR("Y_INVALID\n");
		err_code |= STK_SELFTEST_STAT_FAIL_Y;
	}

	if (noise[2] == 0) {
		GSE_ERR("Z_INVALID\n");
		err_code |= STK_SELFTEST_STAT_FAIL_Z;
	}

	if (abs(acc_ave[0]) >= STK_SELFTEST_OFFSET_X) {
		GSE_ERR("X_OFFSET_OUT_OF_SPEC, %d\n", abs(acc_ave[0]));
		err_code |= STK_SELFTEST_STAT_FAIL_X;
	}

	if (abs(acc_ave[1]) >= STK_SELFTEST_OFFSET_Y) {
		GSE_ERR("Y_OFFSET_OUT_OF_SPEC, %d\n", abs(acc_ave[1]));
		err_code |= STK_SELFTEST_STAT_FAIL_Y;
	}

	if(acc_ave[2] > 0)
		acc_ave[2] -= STK_SELFTEST_LSB_1G;
	else
		acc_ave[2] += STK_SELFTEST_LSB_1G;
	
	if (abs(acc_ave[2]) >= STK_SELFTEST_OFFSET_Z) {
		GSE_ERR("Z_OFFSET_OUT_OF_SPEC, %d\n", abs(acc_ave[2]));
		err_code |= STK_SELFTEST_STAT_FAIL_Z;
	}

	if (noise[0] >= STK_SELFTEST_NOISE_X) {
		GSE_ERR("X_NOISE_FAIL, %d\n", noise[0]);
		err_code |= STK_SELFTEST_STAT_FAIL_X;
	}

	if (noise[1] >=STK_SELFTEST_NOISE_Y) {
		GSE_ERR("Y_NOISE_FAIL, %d\n", noise[1]);
		err_code |= STK_SELFTEST_STAT_FAIL_Y;
	}

	if (noise[2] >= STK_SELFTEST_NOISE_Z) {
		GSE_ERR("Z_NOISE_FAIL, %d\n", noise[2]);
		err_code |= STK_SELFTEST_STAT_FAIL_Z;
	}
	
	obj_i2c_data->selftest_status = err_code;	

	return STK8BAXX_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#ifdef STK_PERMISSION_THREAD
static struct task_struct *STKPermissionThread = NULL;

static int stk_permission_thread(void *data)
{
	int ret = 0;
	int retry = 0;
	mm_segment_t fs = get_fs();
	set_fs(KERNEL_DS);
	msleep(10000);
	do {
		msleep(5000);
		ret = sys_fchmodat(AT_FDCWD, "/sys/devices/platform/gsensor/driver/cali" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/devices/platform/gsensor/driver/recv" , 0666);
		ret = sys_fchmodat(AT_FDCWD, "/sys/devices/platform/gsensor/driver/send" , 0666);
		ret = sys_chmod(STK_ACC_CALI_FILE , 0666);
		ret = sys_fchmodat(AT_FDCWD, STK_ACC_CALI_FILE , 0666);
		ret = sys_chmod(STK_ACC_CALI_FILE_SDCARD , 0666);
		ret = sys_fchmodat(AT_FDCWD, STK_ACC_CALI_FILE_SDCARD , 0666);
		//if(ret < 0)
		//	printk("fail to execute sys_fchmodat, ret = %d\n", ret);
		if(retry++ > 10)
			break;
	} while(ret == -ENOENT);
	set_fs(fs);
	GSE_LOG("exit, retry=%d\n", retry);
	return 0;
}
#endif	/*	#ifdef STK_PERMISSION_THREAD	*/

static int stk8baxx_write_file(int mode, char write_buf[])
{
	struct file  *cali_file;
	char r_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	mm_segment_t fs;
	ssize_t ret;
	int8_t i;
	int err;

	if(mode == 0)
		cali_file = filp_open(STK_ACC_CALI_FILE, O_CREAT | O_RDWR,0666);
	else
		cali_file = filp_open(STK_ACC_CALI_FILE_SDCARD, O_CREAT | O_RDWR,0666);

	if(IS_ERR(cali_file)) {
		err = PTR_ERR(cali_file);
		if(mode == 0)
			GSE_ERR(" filp_open error!err=%d,path=%s\n", err, STK_ACC_CALI_FILE);
		else
			GSE_ERR(" filp_open error!err=%d,path=%s\n", err, STK_ACC_CALI_FILE_SDCARD);
		return -STK_K_FAIL_OPEN_FILE;
	} else {
		fs = get_fs();
		set_fs(get_ds());

		ret = cali_file->f_op->write(cali_file,write_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret != STK_ACC_CALI_FILE_SIZE) {
			GSE_ERR(" write error!\n");
			filp_close(cali_file,NULL);
			return -STK_K_FAIL_W_FILE;
		}
		cali_file->f_pos=0x00;
		ret = cali_file->f_op->read(cali_file,r_buf, STK_ACC_CALI_FILE_SIZE,&cali_file->f_pos);
		if(ret < 0) {
			GSE_ERR(" read error!\n");
			filp_close(cali_file,NULL);
			return -STK_K_FAIL_R_BACK;

		}
		set_fs(fs);

		for(i=0; i<STK_ACC_CALI_FILE_SIZE; i++) {
			if(r_buf[i] != write_buf[i]) {
				GSE_ERR(" read back error, r_buf[%x](0x%x) != write_buf[%x](0x%x)\n", i, r_buf[i], i, write_buf[i]);
				filp_close(cali_file,NULL);
				return -STK_K_FAIL_R_BACK_COMP;
			}
		}
	}
	filp_close(cali_file,NULL);

#ifdef STK_PERMISSION_THREAD
	fs = get_fs();
	set_fs(KERNEL_DS);
	if(mode == 0) {
		ret = sys_chmod(STK_ACC_CALI_FILE , 0666);
		ret = sys_fchmodat(AT_FDCWD, STK_ACC_CALI_FILE , 0666);
	} else {
		ret = sys_chmod(STK_ACC_CALI_FILE_SDCARD , 0666);
		ret = sys_fchmodat(AT_FDCWD, STK_ACC_CALI_FILE_SDCARD , 0666);
	}
	set_fs(fs);
#endif
	return 0;
}

static int stk8baxx_store_in_file(u8 offset[], u8 status, u32 variance[])
{
	char w_buf[STK_ACC_CALI_FILE_SIZE] = {0};
	int err;

	w_buf[0] = STK_ACC_CALI_VER0;
	w_buf[1] = STK_ACC_CALI_VER1;
	w_buf[3] = offset[0];
	w_buf[5] = offset[1];
	w_buf[7] = offset[2];
	w_buf[8] = status;

	if(variance == NULL) {
		w_buf[9] = 0;
		w_buf[10] = 0;
		w_buf[11] = 0;
		w_buf[12] = 0;
		w_buf[13] = 0;
		w_buf[14] = 0;
		w_buf[15] = 0;
		w_buf[16] = 0;
		w_buf[17] = 0;
	} else {
		w_buf[9] =  ((variance[0] >> 24) & 0xFF);
		w_buf[10] = ((variance[0] >> 16) & 0xFF);
		w_buf[11] = ((variance[0] >> 8) & 0xFF);
		w_buf[12] = (variance[0] & 0xFF);
		w_buf[13] = ((variance[1] >> 24) & 0xFF);
		w_buf[14] = ((variance[1] >> 16) & 0xFF);
		w_buf[15] = ((variance[1] >> 8) & 0xFF);
		w_buf[16] = (variance[1] & 0xFF);
		w_buf[17] = ((variance[2] >> 24) & 0xFF);
		w_buf[18] = ((variance[2] >> 16) & 0xFF);
		w_buf[19] = ((variance[2] >> 8) & 0xFF);
		w_buf[20] = (variance[2] & 0xFF);
	}
	w_buf[STK_ACC_CALI_FILE_SIZE - 2] = '\0';
	w_buf[STK_ACC_CALI_FILE_SIZE - 1] = STK_ACC_CALI_END;
	printk("%s: xyz: %d,%d,%d\n",__func__,offset[0],offset[1],offset[2]);

	stk8baxx_write_file(1, w_buf);
	err = stk8baxx_write_file(0, w_buf);
	if(err == 0)
		GSE_LOG("successfully\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk8baxx_init_client(struct i2c_client *client, int reset_cali)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
#ifdef STK_TUNE
	int aa;
#endif

#ifdef CONFIG_SENSORS_STK8BA53
	GSE_LOG("GSE_LOG Initialize stk8ba53\n");
	printk("\n");
	printk("printk Initialize stk8ba53\n");
#else
	GSE_LOG("GSE_LOG Initialize stk8ba50/stk8ba50-r\n");
	printk("\n");
	printk("printk Initialize stk8ba50/stk8ba50-r\n");
#endif

	obj->sensor_power = false;

	res = STK8BAXX_SetReset(client);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set reset error\n");
		return res;
	}

	res = STK8BAXX_SetPowerMode(client,true);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set power mode error\n");
		return res;
	}

	res = STK8BAXX_CheckID(obj);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx check id error\n");
		return res;
	}
	
#ifdef CUSTOM_KERNEL_SENSORHUB
	res = gsensor_setup_irq();
	if(res != STK8BAXX_SUCCESS) {
		return res;
	}

	/* map new data int to int1 */
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_INTMAP2, 0x01);
	if (res < 0) {
		GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_INTMAP2, res);
		return res;
	}
	/*	enable new data int */
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_INTEN2, 0x10);
	if (res < 0) {
		GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_INTEN2, res);
		return res;
	}
	/*	non-latch int	*/
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_INTCFG2, 0x00);
	if (res < 0) {
		GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_INTCFG2, res);
		return res;
	}
	/*	filtered data source for new data int	*/
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_DATASRC, 0x00);
	if (res < 0) {
		GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_DATASRC, res);
		return res;
	}
	/*	int1, push-pull, active high	*/
	res = stk8baxx_i2c_smbus_write_byte_data(client, STK8BAXX_INTCFG1, 0x01);
	if (res < 0) {
		GSE_ERR("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_INTCFG1, res);
		return res;
	}
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

	/*	According to STK_DEF_DYNAMIC_RANGE */
	res = STK8BAXX_SetDataResolution(client, STK_DEF_DYNAMIC_RANGE);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set data reslution error\n");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	/*	ODR = 62 Hz	*/
	res = STK8BAXX_SetBWSEL(client, STK8BAXX_INIT_ODR);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set data format error\n");
		return res;
	}
	obj->gsensor_delay = 16000;
	
	/*	i2c watchdog enable, 1 ms timer perios	*/
	res = STK8BAXX_SetWatchDog(client,0x04);
	if (res < 0) {
		GSE_LOG("failed to write reg 0x%x, error=0x%x\n", STK8BAXX_INTFCFG, res);
		return res;
	}

	if(0 != reset_cali) {
		/*reset calibration only in power on*/
		res = STK8BAXX_ResetCalibration(client);
		if(res != STK8BAXX_SUCCESS) {
			GSE_LOG("stk8baxx set cali error\n");
			return res;
		}
	}
#ifdef STK_ENABLE_AT_BOOT
	res = STK8BAXX_SetPowerMode(client, true);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set power mode error\n");
		return res;
	}
#else
	res = STK8BAXX_SetPowerMode(client, false);
	if(res != STK8BAXX_SUCCESS) {
		GSE_LOG("stk8baxx set power mode error\n");
		return res;
	}
#endif

#ifdef STK_TUNE
	if(atomic_read(&obj->suspend) == 0 ) {
		for(aa=0; aa<3; aa++) {
			obj->stk_tune_offset[aa] = 0;
			obj->stk_tune_offset_record[aa] = 0;
			obj->stk_tune_sum[aa] = 0;
			obj->stk_tune_max[aa] = 0;
			obj->stk_tune_min[aa] = 0;
			obj->stk_tune_square_sum[aa] = 0LL;
		}
		obj->stk_tune_done = 0;
		obj->stk_tune_index = 0;
		obj->first_enable = true;
	}
#endif

#ifdef CONFIG_STK8BAXX_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	atomic_set(&obj->event_since_en_limit, STK_EVENT_SINCE_EN_LIMIT_DEF);
	
	GSE_LOG("stk8baxx Init OK\n");
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = stk8baxx_i2c_client;
	char strbuf[STK8BAXX_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	STK8BAXX_ReadChipInfo(client, strbuf, STK8BAXX_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}


/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = stk8baxx_i2c_client;
	char strbuf[STK8BAXX_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	STK8BAXX_ReadSensorData(client, strbuf, STK8BAXX_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int err, len = 0, mul;
	int tmp[STK8BAXX_AXES_NUM];
	struct i2c_client *client = stk8baxx_i2c_client;
	struct stk8baxx_i2c_data *obj;
	s8 all_offset[STK8BAXX_AXES_NUM];

	obj = i2c_get_clientdata(client);
	GSE_LOG("show_cali_value \n");
	if(NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	GSE_LOG("en = %d, en_limit = %d , power = %d\n", atomic_read(&obj->event_since_en), atomic_read(&obj->event_since_en_limit), obj->sensor_power);
	if((err = STK8BAXX_ReadOffset(client, obj->offset))) {
		return -EINVAL;
	} else if((err = STK8BAXX_ReadCalibration(client, tmp))) {
		return -EINVAL;
	} else {

		if(stk8baxx_offset_resolution.sensitivity < obj->reso->sensitivity) {
			mul = obj->reso->sensitivity/stk8baxx_offset_resolution.sensitivity;
			all_offset[STK8BAXX_AXIS_X] = obj->offset[STK8BAXX_AXIS_X]*mul + obj->cali_sw[STK8BAXX_AXIS_X];
			all_offset[STK8BAXX_AXIS_Y] = obj->offset[STK8BAXX_AXIS_Y]*mul + obj->cali_sw[STK8BAXX_AXIS_Y];
			all_offset[STK8BAXX_AXIS_Z] = obj->offset[STK8BAXX_AXIS_Z]*mul + obj->cali_sw[STK8BAXX_AXIS_Z];

		} else {
			mul = stk8baxx_offset_resolution.sensitivity / obj->reso->sensitivity;
			all_offset[STK8BAXX_AXIS_X] = obj->offset[STK8BAXX_AXIS_X]/mul + obj->cali_sw[STK8BAXX_AXIS_X];
			all_offset[STK8BAXX_AXIS_Y] = obj->offset[STK8BAXX_AXIS_Y]/mul + obj->cali_sw[STK8BAXX_AXIS_Y];
			all_offset[STK8BAXX_AXIS_Z] = obj->offset[STK8BAXX_AXIS_Z]/mul + obj->cali_sw[STK8BAXX_AXIS_Z];
			mul = -1;
		}

		len += snprintf(buf+len, PAGE_SIZE-len, "%x\n[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", atomic_read(&obj->cali_status), mul,
		                obj->offset[STK8BAXX_AXIS_X], obj->offset[STK8BAXX_AXIS_Y], obj->offset[STK8BAXX_AXIS_Z],
		                obj->offset[STK8BAXX_AXIS_X], obj->offset[STK8BAXX_AXIS_Y], obj->offset[STK8BAXX_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		                obj->cali_sw[STK8BAXX_AXIS_X], obj->cali_sw[STK8BAXX_AXIS_Y], obj->cali_sw[STK8BAXX_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
		                all_offset[STK8BAXX_AXIS_X], all_offset[STK8BAXX_AXIS_Y], all_offset[STK8BAXX_AXIS_Z],
		                tmp[STK8BAXX_AXIS_X], tmp[STK8BAXX_AXIS_Y], tmp[STK8BAXX_AXIS_Z]);

		return len;
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = stk8baxx_i2c_client;
	int err, x, y, z, sstate;
	int dat[STK8BAXX_AXES_NUM];

	if(!strncmp(buf, "rst", 3)) {
		if((err = STK8BAXX_ResetCalibration(client))) {
			GSE_ERR("reset offset err = %d\n", err);
		}
	} else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[STK8BAXX_AXIS_X] = x;
		dat[STK8BAXX_AXIS_Y] = y;
		dat[STK8BAXX_AXIS_Z] = z;
		if((err = STK8BAXX_WriteCalibration(client, dat)))
			//if(err = STK8BAXX_SetCali(client))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}
	} else if(1 == sscanf(buf, "%d", &sstate)) {
		GSE_LOG("sstate = %d\n", sstate);
		if(sstate == 1) {
			if((err = STK8BAXX_SetCali(client))) {
				GSE_ERR("calibration err = %d\n", err);
			}
		}
	} else {
		GSE_ERR("invalid format\n");
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_STK8BAXX_LOWPASS
	struct i2c_client *client = stk8baxx_i2c_client;
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++) {
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][STK8BAXX_AXIS_X], obj->fir.raw[idx][STK8BAXX_AXIS_Y], obj->fir.raw[idx][STK8BAXX_AXIS_Z]);
		}

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[STK8BAXX_AXIS_X], obj->fir.sum[STK8BAXX_AXIS_Y], obj->fir.sum[STK8BAXX_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[STK8BAXX_AXIS_X]/len, obj->fir.sum[STK8BAXX_AXIS_Y]/len, obj->fir.sum[STK8BAXX_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "firlen = %d, en=%d, filter=%d\n", atomic_read(&obj->firlen), atomic_read(&obj->fir_en), atomic_read(&obj->filter));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_STK8BAXX_LOWPASS
	struct i2c_client *client = stk8baxx_i2c_client;
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if (kstrtoint(buf, 10, &firlen)) {
		GSE_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		GSE_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&obj->firlen, firlen);
		if (0 == firlen) {
			atomic_set(&obj->fir_en, 0);
		} else {
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct stk8baxx_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (1 == sscanf(buf, "0x%x", &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct stk8baxx_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
		                obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id,
		                obj->hw->power_vol);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	struct stk8baxx_i2c_data *obj = obj_i2c_data;
	ssize_t len = 0;
	
	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if (obj->sensor_power)
		GSE_LOG("Gsensor is in work mode, sensor_power = %d\n", obj->sensor_power);
	else
		GSE_LOG("Gsensor is in standby mode, sensor_power = %d\n", obj->sensor_power);

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", obj->sensor_power);	
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_allreg_value(struct device_driver *ddri, char *buf)
{
	int ret;
	
	ret = STK8BAXX_ShowAllReg(buf);
	return (ssize_t)ret;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_send_value(struct device_driver *ddri, char *buf)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_send_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct stk8baxx_i2c_data *obj = obj_i2c_data;
	int addr, cmd;
	u8 dat;

	if(!obj) {
		GSE_ERR("obj is null!!\n");
		return 0;
	}

	else if(2 != sscanf(buf, "%x %x", &addr, &cmd)) {
		GSE_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	GSE_LOG("send(%02X, %02X) = %d\n", addr, cmd,
	        stk8baxx_i2c_smbus_write_byte_data(obj->client, (u8)addr, (u8)cmd));

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_recv_value(struct device_driver *ddri, char *buf)
{
	if(!obj_i2c_data) {
		GSE_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj_i2c_data->recv_reg));
}
/*----------------------------------------------------------------------------*/
static ssize_t store_recv_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	int res = 0;

	if(!obj_i2c_data) {
		GSE_ERR("obj_i2c_data is null!!\n");
		return 0;
	} else if(1 != sscanf(buf, "%x", &addr)) {
		GSE_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	res = i2c_smbus_read_byte_data(obj_i2c_data->client, addr);
	if(res < 0)	{
		GSE_ERR("failed to read 0x%x, error=%d\n", res, addr);
		return res;
	}
	GSE_LOG("recv 0x%02X = 0x%02X\n", addr, res);
	atomic_set(&obj_i2c_data->recv_reg, res);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	if(!obj_i2c_data) {
		GSE_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	GSE_LOG(":version=%s\n", STK_SELFTEST_VERSION);
	
	return scnprintf(buf, PAGE_SIZE, "%02X\n", obj_i2c_data->selftest_status);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int res = 0;
	int enabled_func;
	int err_code;
	bool org_sensor_power = obj_i2c_data->sensor_power;
	
	GSE_FUN();
	
	if(!obj_i2c_data) {
		GSE_ERR("obj_i2c_data is null!!\n");
		return 0;
	}
	
	if(1 != sscanf(buf, "%x", &enabled_func)) {
		GSE_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	GSE_LOG(":version=%s\n", STK_SELFTEST_VERSION);	
	obj_i2c_data->selftest_status = STK_SELFTEST_STAT_RUNNING;
	
	if(!org_sensor_power) {
		res = stk8baxx_i2c_smbus_write_byte_data(obj_i2c_data->client, STK8BAXX_POWMODE, STK8BAXX_MD_NORMAL);
		if(res < 0) {
			GSE_LOG("set power mode failed!\n");
			obj_i2c_data->selftest_status = STK_SELFTEST_STAT_DRIVER_ERR;
			return STK8BAXX_ERR_I2C;
		}
	}
	
	res = STK8BAXX_CheckID(obj_i2c_data);
	if(res < 0)	{
		obj_i2c_data->selftest_status = STK_SELFTEST_STAT_DRIVER_ERR;
		return res;
	}
	
	res = STK8BAXX_ShowAllReg(NULL);
	if(res < 0)	{
		obj_i2c_data->selftest_status = STK_SELFTEST_STAT_DRIVER_ERR;
		return res;
	}
	
	if(enabled_func & 0x0001) {
		res = STK8BAXX_TestOffsetNoise();
		if(res < 0)	{
			obj_i2c_data->selftest_status = STK_SELFTEST_STAT_DRIVER_ERR;
			return res;
		}
	}
	GSE_LOG("stk8baxx selftest_status=%d\n", obj_i2c_data->selftest_status);
	
	stk8baxx_init_client(obj_i2c_data->client, 0);	
	
#ifdef STK_ENABLE_AT_BOOT
	if(!org_sensor_power) {
		res = STK8BAXX_SetPowerMode(obj_i2c_data->client, false);
		if(res != STK8BAXX_SUCCESS) {
			GSE_LOG("stk8baxx set power mode error\n");
			return res;
		}
	}
#else
	if(org_sensor_power) {
		res = STK8BAXX_SetPowerMode(obj_i2c_data->client, true);
		if(res != STK8BAXX_SUCCESS) {
			GSE_LOG("stk8baxx set power mode error\n");
			return res;
		}
	}
#endif	
		
	return count;	
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);
static DRIVER_ATTR(allreg, S_IRUGO, show_allreg_value, NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, show_send_value,  store_send_value);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, show_recv_value,  store_recv_value);
static DRIVER_ATTR(selftest,    S_IWUSR | S_IRUGO, show_selftest_value,  store_selftest_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *stk8baxx_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_cali,	/*show calibration data */
	&driver_attr_firlen,	/*filter length: 0: disable, others: enable */
	&driver_attr_trace,	/*trace log */
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_allreg,
	&driver_attr_send,
	&driver_attr_recv,
	&driver_attr_selftest,
};

/*----------------------------------------------------------------------------*/
static int stk8baxx_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(stk8baxx_attr_list) / sizeof(stk8baxx_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, stk8baxx_attr_list[idx]);
		if (err) {
			GSE_ERR("driver_create_file (%s) = %d\n", stk8baxx_attr_list[idx]->attr.name,
			        err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int stk8baxx_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(stk8baxx_attr_list) / sizeof(stk8baxx_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, stk8baxx_attr_list[idx]);

	return err;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static void gsensor_irq_work(struct work_struct *work)
{
	struct stk8baxx_i2c_data *obj = obj_i2c_data;
	struct scp_acc_hw scp_hw;
	union STK8BAXX_CUST_DATA *p_cust_data;
	SCP_SENSOR_HUB_DATA data;
	int max_cust_data_size_per_packet;
	int i;
	uint sizeOfCustData;
	uint len;
	char *p = (char *)&scp_hw;

	GSE_FUN();

	scp_hw.i2c_num = obj->hw->i2c_num;
	scp_hw.direction = obj->hw->direction;
	scp_hw.power_id = obj->hw->power_id;
	scp_hw.power_vol = obj->hw->power_vol;
	scp_hw.firlen = obj->hw->firlen;
	memcpy(scp_hw.i2c_addr, obj->hw->i2c_addr, sizeof(obj->hw->i2c_addr));
	scp_hw.power_vio_id = obj->hw->power_vio_id;
	scp_hw.power_vio_vol = obj->hw->power_vio_vol;
	scp_hw.is_batch_supported = obj->hw->is_batch_supported;

	p_cust_data = (union STK8BAXX_CUST_DATA *) data.set_cust_req.custData;
	sizeOfCustData = sizeof(scp_hw);
	max_cust_data_size_per_packet =
	    sizeof(data.set_cust_req.custData) - offsetof(STK8BAXX_SET_CUST, data);

	for (i = 0; sizeOfCustData > 0; i++) {
		data.set_cust_req.sensorType = ID_ACCELEROMETER;
		data.set_cust_req.action = SENSOR_HUB_SET_CUST;
		p_cust_data->setCust.action = STK8BAXX_CUST_ACTION_SET_CUST;
		p_cust_data->setCust.part = i;
		if (sizeOfCustData > max_cust_data_size_per_packet)
			len = max_cust_data_size_per_packet;
		else
			len = sizeOfCustData;

		memcpy(p_cust_data->setCust.data, p, len);
		sizeOfCustData -= len;
		p += len;

		len +=
		    offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + offsetof(STK8BAXX_SET_CUST,
		            data);
		SCP_sensorHub_req_send(&data, &len, 1);
	}

	p_cust_data = (union STK8BAXX_CUST_DATA *) &data.set_cust_req.custData;

	data.set_cust_req.sensorType = ID_ACCELEROMETER;
	data.set_cust_req.action = SENSOR_HUB_SET_CUST;
	p_cust_data->resetCali.action = STK8BAXX_CUST_ACTION_RESET_CALI;
	len = offsetof(SCP_SENSOR_HUB_SET_CUST_REQ, custData) + sizeof(p_cust_data->resetCali);
	SCP_sensorHub_req_send(&data, &len, 1);

	obj->SCP_init_done = 1;
}

/*----------------------------------------------------------------------------*/
static int gsensor_irq_handler(void *data, uint len)
{
	struct stk8baxx_i2c_data *obj = obj_i2c_data;
	SCP_SENSOR_HUB_DATA_P rsp = (SCP_SENSOR_HUB_DATA_P) data;

	GSE_FUN();
	GSE_LOG("len = %d, type = %d, action = %d, errCode = %d\n", len, rsp->rsp.sensorType,
	        rsp->rsp.action, rsp->rsp.errCode);

	if (!obj)
		return -1;

	switch (rsp->rsp.action) {
	case SENSOR_HUB_NOTIFY:
		switch (rsp->notify_rsp.event) {
		case SCP_INIT_DONE:
			schedule_work(&obj->irq_work);
			/* schedule_delayed_work(&obj->irq_work, HZ); */
			break;
		default:
			GSE_ERR("Error sensor hub notify");
			break;
		}
		break;
	default:
		GSE_ERR("Error sensor hub action");
		break;
	}

	return 0;
}

static int gsensor_setup_irq(void)
{
	int err = 0;
#ifdef GSENSOR_UT
	GSE_FUN();
#endif
	err = SCP_sensorHub_rsp_registration(ID_ACCELEROMETER, gsensor_irq_handler);
	return err;
}
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int stk8baxx_open(struct inode *inode, struct file *file)
{
	file->private_data = stk8baxx_i2c_client;

	if (file->private_data == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int stk8baxx_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long stk8baxx_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct stk8baxx_i2c_data *obj = (struct stk8baxx_i2c_data *)i2c_get_clientdata(client);
	char strbuf[STK8BAXX_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];
	
	GSE_LOG("cmd=0x%x\n", cmd);	
	
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err) {
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		GSE_LOG("GSENSOR_IOCTL_INIT");
		stk8baxx_init_client(client, 0);
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		GSE_LOG("GSENSOR_IOCTL_READ_CHIPINFO");
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		STK8BAXX_ReadChipInfo(client, strbuf, STK8BAXX_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		GSE_LOG("GSENSOR_IOCTL_READ_SENSORDATA");
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		STK8BAXX_ReadSensorData(client, strbuf, STK8BAXX_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_OFFSET:
		GSE_LOG("GSENSOR_IOCTL_READ_OFFSET\n");
		/*
		data = (void __user *) arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D))) {
			err = -EFAULT;
			break;
		}	
		*/
		break;
		
	case GSENSOR_IOCTL_READ_GAIN:
		GSE_LOG("GSENSOR_IOCTL_READ_GAIN");
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D))) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		GSE_LOG("GSENSOR_IOCTL_READ_RAW_DATA");
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		STK8BAXX_ReadRawData(client, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		GSE_LOG("GSENSOR_IOCTL_SET_CALI\n");
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			GSE_ERR("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		} else {
			cali[STK8BAXX_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[STK8BAXX_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[STK8BAXX_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			err = STK8BAXX_WriteCalibration(client, cali);
			GSE_LOG("GSENSOR_IOCTL_SET_CALI!!sensor_data .x =%d,sensor_data .z =%d,sensor_data .z =%d \n",sensor_data.x,sensor_data.y,sensor_data.z);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		GSE_LOG("GSENSOR_IOCTL_CLR_CALI");
		err = STK8BAXX_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		GSE_LOG("GSENSOR_IOCTL_GET_CALI\n");
		data = (void __user*)arg;
		if(data == NULL) {
			err = -EINVAL;
			break;
		}
		// set low ODR to reduce noise
		STK8BAXX_SetDelay(client, 13333000);
		printk("%s: STK8BAXX_ReadCalibration \n",__FUNCTION__);
		if((err = STK8BAXX_ReadCalibration(client, cali))) {
			break;
		}

		sensor_data.x = cali[STK8BAXX_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.y = cali[STK8BAXX_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.z = cali[STK8BAXX_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		printk("%s:cali data copy to user file cali[STK8BAXX_AXIS_X] = %d cali[STK8BAXX_AXIS_Y] = %d cali[STK8BAXX_AXIS_Z] = %d\n",__func__,cali[STK8BAXX_AXIS_X],cali[STK8BAXX_AXIS_Y],cali[STK8BAXX_AXIS_Z]);
		if(copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_COMPAT
static long stk8baxx_compat_ioctl(struct file *file, unsigned int cmd,
                                  unsigned long arg)
{
	long err = 0;
	void __user *arg32 = compat_ptr(arg);

	GSE_LOG("cmd=0x%x\n", cmd);	
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		GSE_LOG("COMPAT_GSENSOR_IOCTL_READ_SENSORDATA");
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		GSE_LOG("COMPAT_GSENSOR_IOCTL_SET_CALI");
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		GSE_LOG("COMPAT_GSENSOR_IOCTL_GET_CALI");
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		GSE_LOG("COMPAT_GSENSOR_IOCTL_CLR_CALI");
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
	}

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
static const struct file_operations stk8baxx_fops = {
	/* .owner = THIS_MODULE, */
	.open = stk8baxx_open,
	.release = stk8baxx_release,
	.unlocked_ioctl = stk8baxx_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= stk8baxx_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice stk8baxx_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &stk8baxx_fops,
};

static int stk8baxx_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);

		mutex_lock(&gsensor_mutex);		
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
		err = STK8BAXX_SCP_SetPowerMode(false, ID_ACCELEROMETER);
#else				/* #ifndef CONFIG_CUSTOM_KERNEL_SENSORHUB */
		err = STK8BAXX_SetPowerMode(obj->client, false);
#endif				/* #ifndef CONFIG_CUSTOM_KERNEL_SENSORHUB */
		mutex_unlock(&gsensor_mutex);	
		if (err) {
			GSE_ERR("write power control fail!!\n");
			return err;
		}
#ifndef CONFIG_CUSTOM_KERNEL_SENSORHUB
		stk8baxx_power(obj->hw, 0);
#endif
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int stk8baxx_resume(struct i2c_client *client)
{
	struct stk8baxx_i2c_data *obj = i2c_get_clientdata(client);
	int err;

	GSE_FUN();

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
#ifndef CONFIG_CUSTOM_KERNEL_SENSORHUB
	stk8baxx_power(obj->hw, 1);
#endif
	mutex_lock(&gsensor_mutex);		
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	err = STK8BAXX_SCP_SetPowerMode(obj->sensor_power, ID_ACCELEROMETER);
#else
	err = stk8baxx_init_client(client, 0);
#endif
	mutex_unlock(&gsensor_mutex);	
	if (err) {
		GSE_ERR("initialize client fail!!\n");
		return err;
	}

	atomic_set(&obj->suspend, 0);

	return 0;
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int gsensor_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
static int scp_gsensor_enable_nodata(int en)
{
	int err = 0;

	mutex_lock(&gsensor_mutex);
	/*
	if (((en == 0) && (obj_i2c_data->sensor_power == false)) || ((en == 1) && (obj_i2c_data->sensor_power == true))) {
		enable_status = obj_i2c_data->sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
		
	} else {
		enable_status = !obj_i2c_data->sensor_power;
		if (atomic_read(&obj_i2c_data->suspend) == 0) {
			*/
			enable_status = en ? true:false;
			err = STK8BAXX_SCP_SetPowerMode(enable_status, ID_ACCELEROMETER);
			if (0 == err)
				obj_i2c_data->sensor_power = enable_status;
			GSE_LOG("set sensor_power = %d\n", obj_i2c_data->sensor_power);			
	/*	 
			GSE_LOG("Gsensor not in suspend gsensor_SetPowerMode!, enable_status = %d\n",
			 enable_status);
			 
		} else {
			GSE_LOG
			("Gsensor in suspend and can not enable or disable!enable_status = %d\n",
			 enable_status);
		}
	}
	*/
	mutex_unlock(&gsensor_mutex);

	if (err != STK8BAXX_SUCCESS) {
		GSE_LOG("scp_sensor_enable_nodata fail!\n");
		return -1;
	}

	GSE_LOG("OK!!!\n");
	return 0;
}
#else
static int gsensor_enable_nodata(int en)
{
	int err = 0;
	bool enable = false;
	
#ifdef GSENSOR_UT
	GSE_FUN();
#endif
	mutex_lock(&gsensor_mutex);
	/* if (atomic_read(&obj_i2c_data->suspend) == 0) { */
		enable = en ? true:false;
		err = STK8BAXX_SetPowerMode(obj_i2c_data->client, enable);
		GSE_LOG("set sensor_power = %d\n", obj_i2c_data->sensor_power);
	/* 
	} else {
		GSE_LOG("Gsensor in suspend and can not enable or disable!current sensor_power = %d\n",
				obj_i2c_data->sensor_power);
	}
	*/
	/*
	if (((en == 0) && (obj_i2c_data->sensor_power == false)) || ((en == 1) && (obj_i2c_data->sensor_power == true))) {
		enable_status = obj_i2c_data->sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		enable_status = !obj_i2c_data->sensor_power;
		if (atomic_read(&obj_i2c_data->suspend) == 0) {
			err = STK8BAXX_SetPowerMode(obj_i2c_data->client,enable_status);
			GSE_LOG("Gsensor not in suspend gsensor_SetPowerMode!, enable_status = %d\n",
			        enable_status);
		} else {
			GSE_LOG("Gsensor in suspend and can not enable or disable!enable_status = %d\n",
			        enable_status);
		}
	}
	*/
	mutex_unlock(&gsensor_mutex);

	if (err != STK8BAXX_SUCCESS) {
		GSE_ERR(" fail!\n");
		return -1;
	}

	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int gsensor_set_delay(u64 ns)
{
	int err = 0;
	int value;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
#ifdef GSENSOR_UT
	GSE_FUN();
#endif

	value = (int)ns / 1000 / 1000;
	GSE_LOG("delay=(%d) ms\n", value);

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	req.set_delay_req.sensorType = ID_ACCELEROMETER;
	req.set_delay_req.action = SENSOR_HUB_SET_DELAY;
	req.set_delay_req.delay = value;
	len = sizeof(req.activate_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}
#else				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	mutex_lock(&gsensor_mutex);
	STK8BAXX_SetDelay(obj_i2c_data->client, (uint32_t)ns);
	mutex_unlock(&gsensor_mutex);
	if (err != STK8BAXX_SUCCESS) {
		GSE_ERR("Set delay parameter error!\n");
		return -1;
	}
#if defined(CONFIG_STK8BAXX_LOWPASS)
	obj_i2c_data->fir.num = 0;
	obj_i2c_data->fir.idx = 0;
	obj_i2c_data->fir.sum[STK8BAXX_AXIS_X] = 0;
	obj_i2c_data->fir.sum[STK8BAXX_AXIS_Y] = 0;
	obj_i2c_data->fir.sum[STK8BAXX_AXIS_Z] = 0;
	atomic_set(&obj_i2c_data->filter, 1);
#endif

#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	return 0;
}

/*----------------------------------------------------------------------------*/
static int gsensor_get_data(int *x, int *y, int *z, int *status)
{
	int err = 0;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	SCP_SENSOR_HUB_DATA req;
	int len;
#else
	char buff[STK8BAXX_BUFSIZE];
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	/* GSE_FUN(); */

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	req.get_data_req.sensorType = ID_ACCELEROMETER;
	req.get_data_req.action = SENSOR_HUB_GET_DATA;
	len = sizeof(req.get_data_req);
	err = SCP_sensorHub_req_send(&req, &len, 1);
	if (err) {
		GSE_ERR("SCP_sensorHub_req_send!\n");
		return err;
	}

	if (ID_ACCELEROMETER != req.get_data_rsp.sensorType ||
	        SENSOR_HUB_GET_DATA != req.get_data_rsp.action || 0 != req.get_data_rsp.errCode) {
		GSE_ERR("error : %d\n", req.get_data_rsp.errCode);
		return req.get_data_rsp.errCode;
	}
	*x = (int)req.get_data_rsp.int16_Data[0] * GRAVITY_EARTH_1000 / 1000;
	*y = (int)req.get_data_rsp.int16_Data[1] * GRAVITY_EARTH_1000 / 1000;
	*z = (int)req.get_data_rsp.int16_Data[2] * GRAVITY_EARTH_1000 / 1000;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	if (atomic_read(&obj_i2c_data->trace) & ADX_TRC_IOCTL)
		GSE_LOG("x = %d, y = %d, z = %d\n", *x, *y, *z);
#else				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
	mutex_lock(&gsensor_mutex);
	STK8BAXX_ReadSensorData(obj_i2c_data->client, buff, STK8BAXX_BUFSIZE);
	mutex_unlock(&gsensor_mutex);
	err = sscanf(buff, "%x %x %x", x, y, z);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/
/* static int stk8baxx_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) */
/* { */
/* strcpy(info->type, STK8BAXX_DEV_NAME); */
/* return 0; */
/* } */

/*----------------------------------------------------------------------------*/
static int stk8baxx_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct stk8baxx_i2c_data *obj;
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
	int err = 0;

	printk("%s: driver version: %s\n", __FUNCTION__, STK_DRIVER_VERSION);

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct stk8baxx_i2c_data));

	obj->hw = hw;
        client->addr=obj->hw->i2c_addr[0];
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->irq_work, gsensor_irq_work);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->event_since_en, 0);
	atomic_set(&obj->event_since_en_limit, STK_EVENT_SINCE_EN_LIMIT_DEF);
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	obj->SCP_init_done = 0;
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

#ifdef CONFIG_STK8BAXX_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if(atomic_read(&obj->firlen) > 0) {
		atomic_set(&obj->fir_en, 1);
		atomic_set(&obj->filter, 1);
	}
#endif
	obj->selftest_status = STK_SELFTEST_STAT_NA;
	stk8baxx_i2c_client = new_client;
	STK8BAXX_CheckInit();
	
	err = stk8baxx_init_client(new_client, 1);
	if (err)
		goto exit_init_failed;

	err = misc_register(&stk8baxx_device);
	if (err) {
		GSE_ERR("stk8baxx_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = stk8baxx_create_attr(&stk8baxx_init_info.platform_diver_addr->driver);
	if (err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = gsensor_open_report_data;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ctl.enable_nodata = scp_gsensor_enable_nodata;
#else
	ctl.enable_nodata = gsensor_enable_nodata;
#endif
	ctl.set_delay = gsensor_set_delay;
	ctl.is_report_input_direct = false;
	ctl.is_use_common_factory = false;
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ctl.is_support_batch = obj->hw->is_batch_supported;
#else
	ctl.is_support_batch = false;
#endif

	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_create_attr_failed;
	}

	data.get_data = gsensor_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err\n");
		goto exit_create_attr_failed;
	}

	err = batch_register_support_info(ID_ACCELEROMETER, ctl.is_support_batch, 102, 0);	//divisor is 1000/9.8
	if (err) {
		GSE_ERR("register gsensor batch support err = %d\n", err);
		goto exit_create_attr_failed;
	}

	gsensor_init_flag = 0;
	printk("%s: OK\n", __func__);

	return 0;

exit_create_attr_failed:
	misc_deregister(&stk8baxx_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
	kfree(obj);
exit:
	printk("%s: err = %d\n", __func__, err);
	gsensor_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int stk8baxx_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = stk8baxx_delete_attr(&stk8baxx_init_info.platform_diver_addr->driver);
	if (err)
		GSE_ERR("stk8baxx_delete_attr fail: %d\n", err);

	err = misc_deregister(&stk8baxx_device);
	if (err)
		GSE_ERR("misc_deregister fail: %d\n", err);

	stk8baxx_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*----------------------------------------------------------------------------*/
static int gsensor_local_init(void)
{
	GSE_FUN();

	stk8baxx_power(hw, 1);
	
	if (i2c_add_driver(&stk8baxx_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	
	if (-1 == gsensor_init_flag)
		return -1;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gsensor_remove(void)
{
	GSE_FUN();
	stk8baxx_power(hw, 0);
	i2c_del_driver(&stk8baxx_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init stk8baxx_init(void)
{
#ifdef CONFIG_MTK_LEGACY
	struct i2c_board_info i2c_stk8baxx= { I2C_BOARD_INFO("STK8BAXX", STK8BAXX_I2C_SLAVE_ADDR)};
#endif

	printk("%s: init\n", __func__);
	
	hw = get_accel_dts_func("mediatek,stk8baxx", hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");
#ifdef CONFIG_MTK_LEGACY
	i2c_register_board_info( hw->i2c_num, &i2c_stk8baxx, 1);
#endif
	printk("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	acc_driver_add(&stk8baxx_init_info);

#ifdef STK_PERMISSION_THREAD
	STKPermissionThread = kthread_run(stk_permission_thread,"stk","Permissionthread");
	if(IS_ERR(STKPermissionThread))
		STKPermissionThread = NULL;
#endif // STK_PERMISSION_THREAD		
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit stk8baxx_exit(void)
{
	GSE_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(stk8baxx_init);
module_exit(stk8baxx_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("stk8baxx I2C driver");
MODULE_AUTHOR("lex_hsieh@sensortek.com.tw");
