/* BMA150 motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
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
//#include "upmu_sw.h"
#include "upmu_common.h"
#include "batch.h"

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <cust_acc.h>
#include "mxc400x.h"

#include <accel.h>
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
#include <SCP_sensorHub.h>
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_BMA222 222
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/

#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define MXC400X_AXIS_X          0
#define MXC400X_AXIS_Y          1
#define MXC400X_AXIS_Z          2
#define MXC400X_AXES_NUM        3
#define MXC400X_DATA_LEN        6

static int cali_sensor_data[MXC400X_AXES_NUM];

/*********/
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mxc400x_i2c_id[] = { { MXC400X_DEV_NAME, 0 }, { }, };

/* static struct i2c_board_info __initdata i2c_BMA222={ I2C_BOARD_INFO(BMA222_DEV_NAME, 0x18)}; */

#define COMPATIABLE_NAME "mediatek,mxc400x"

/*----------------------------------------------------------------------------*/
static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mxc400x_i2c_remove(struct i2c_client *client);
static int mxc400x_suspend(struct i2c_client *client, pm_message_t msg);
static int mxc400x_resume(struct i2c_client *client);

static int  mxc400x_local_init(void);
static int mxc400x_remove(void);
static int gsensor_set_delay(u64 ns);
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
	s16 raw[C_MAX_FIR_LENGTH][MXC400X_AXES_NUM];
	int sum[MXC400X_AXES_NUM];
	int num;
	int idx;
};
/*----------------------------------------------------------------------------*/
struct mxc400x_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
		 struct hwmsen_convert	 cvt;
		 atomic_t layout; 

	/*misc */
	struct data_resolution *reso;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
		 s16					 cali_sw[MXC400X_AXES_NUM+1];

	/*data */
	s8 offset[MXC400X_AXES_NUM + 1];	/*+1: for 4-byte alignment */
	s16 data[MXC400X_AXES_NUM + 1];


#if defined(CONFIG_MXC400X_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
	u8 bandwidth;
};
/*----------------------------------------------------------------------------*/

static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{},
};

// static unsigned short force[] = { 0, 0x15, I2C_CLIENT_END, I2C_CLIENT_END };
// static const unsigned short *const forces[] = { force, NULL };

static struct i2c_driver mxc400x_i2c_driver = {
	.driver = {
/* .owner          = THIS_MODULE, */
		 .name			 = MXC400X_DEV_NAME,
		   .of_match_table = accel_of_match,
		   },
	.probe = mxc400x_i2c_probe,
	.remove = mxc400x_i2c_remove,
	.suspend = mxc400x_suspend,
	.resume = mxc400x_resume,
	.id_table = mxc400x_i2c_id,
// 	.address_data = (const unsigned short *)forces,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *mxc400x_i2c_client;
static struct mxc400x_i2c_data *obj_i2c_data;
static bool sensor_power = true;
static int sensor_suspend;
static struct GSENSOR_VECTOR3D gsensor_gain;

static struct mutex mxc400x_mutex;
static bool enable_status;

static int gsensor_init_flag = -1;	/* 0<==>OK -1 <==> fail */
static struct acc_init_info mxc400x_init_info = {
		.name = "mxc400x",
		.init = mxc400x_local_init,
		.uninit = mxc400x_remove,
};

/*----------------------------------------------------------------------------*/
#if 1
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               pr_err(GSE_TAG"%s\n", __func__)
#define GSE_ERR(fmt, args...)    pr_err(GSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_err(GSE_TAG fmt, ##args)
#else
#define GSE_TAG
#define GSE_FUN(f)
#define GSE_ERR(fmt, args...)
#define GSE_LOG(fmt, args...)
#endif

struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;


/*----------------------------------------------------------------------------*/
static struct data_resolution mxc400x_data_resolution[] = {
    {{ 0, 9}, 1024},   /*+/-2g  in 12-bit resolution:  0.9 mg/LSB*/
    {{ 1, 9}, 512},   /*+/-4g  in 12-bit resolution:  1.9 mg/LSB*/
    {{ 3, 9},  256},   /*+/-8g  in 12-bit resolution: 3.9 mg/LSB*/      
};
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc400x_offset_resolution = {{3, 9}, 256};

/*----------------------------------------------------------------------------*/
static int mxc400x_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr;
	int err;
	struct i2c_msg msgs[2] = { {0}, {0} };

	//mutex_lock(&gsensor_mutex);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	if (!client) {

		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);

		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
	if (err != 2) {
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;

}



/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static void mxc400x_power(struct acc_hw *hw, unsigned int on) 
{
	 static unsigned int power_on = 0;

	 if(hw->power_id != POWER_NONE_MACRO)		 // have externel LDO
	 {		  
		 GSE_LOG("power %s\n", on ? "on" : "off");
		 if(power_on == on)  // power status not change
		 {
			 GSE_LOG("ignore power control: %d\n", on);
		 }
		 else if(on) // power on
		 {
			// if(!hwPowerOn(hw->power_id, hw->power_vol, "MXC400X"))
			 {
				// GSE_ERR("power on fails!!\n");
			 }
		 }
		 else	 // power off
		 {
			// if (!hwPowerDown(hw->power_id, "MXC400X"))
			 {
				// GSE_ERR("power off fail!!\n");
			 }			   
		 }
	 }
	 power_on = on;    
}
/*static int MXC400X_SaveData(int buf[MXC400X_AXES_NUM])
{ 
   
	mutex_lock(&mxc400x_mutex);
//    memcpy(cali_sensor_data, buf, sizeof(cali_sensor_data));
	cali_sensor_data[0] = buf[0];
	cali_sensor_data[1] = buf[1];
	cali_sensor_data[2] = buf[2];
	
	mutex_unlock(&mxc400x_mutex);

    return 0;
}*/
#if 0
static int  MXC400X_RxData(struct i2c_client *client, s16 data[MXC400X_AXES_NUM])
{
	u8 addr = MXC400X_REG_X;
	u8 buf[MXC400X_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	else if(err == cust_hwmsen_read_block(client, addr, buf, MXC400X_DATA_LEN))
#else
	else if(err == mxc400x_i2c_read_block(client, addr, buf, MXC400X_DATA_LEN))
#endif
	{
		GSE_ERR("error: %d\n", err);
	}
	else 
    {
        
	 	data[MXC400X_AXIS_X] = (s16)(buf[0] << 8 | buf[1]) >> 4;
		data[MXC400X_AXIS_Y] = (s16)(buf[2] << 8 | buf[3]) >> 4;
		data[MXC400X_AXIS_Z] = (s16)(buf[4] << 8 | buf[5]) >> 4;
       
	}

	return err;	
}
#endif

/*static int MXC400X_ReadTemp(struct i2c_client *client, s8 *data)
{     
	u8 addr = MXC400X_REG_TEMP;
	int err = 0;
    s8 temp = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	else if(err == cust_hwmsen_read_block(client, addr, &temp, 0x01))
#else
	else if(mxc400x_i2c_read_block(client, addr, &temp, 1))
#endif
	{
		GSE_ERR("error: %d\n", err);
		err = -EINVAL;
	}
    *data = temp;
  
	return err;
}*/
static int MXC400X_ReadDant(struct i2c_client *client, s16 dant[MXC400X_AXES_NUM+1])
{
	u8 addr = MXC400X_REG_X;
	u8 buf[MXC400X_DATA_LEN+1] = {0};
	int err = 0;

	if(NULL == client)
	{
        GSE_LOG("client is null\n");
		err = -EINVAL;
	}
#if defined(SW_SIMULATION)
	if(cust_hwmsen_read_block(client, addr, buf, MXC400X_DATA_LEN+1))
#else
	if(mxc400x_i2c_read_block(client, addr, buf, MXC400X_DATA_LEN+1))
#endif
	{
		GSE_ERR("error: %d\n", err);
	}
	else 
    {        
   
		dant[MXC400X_AXIS_X] = (s16)((s16)buf[0] << 8 | (s16)buf[1]) >> 4;
		dant[MXC400X_AXIS_Y] = (s16)((s16)buf[2] << 8 | (s16)buf[3]) >> 4;
		dant[MXC400X_AXIS_Z] = (s16)((s16)buf[4] << 8 | (s16)buf[5]) >> 4;
		dant[MXC400X_AXIS_Z+1] = (s16)buf[6];

	   
	}
    
	return err;	
}

/*----------------------------------------------------------------------------*/
static int MXC400X_SetDataResolution(struct mxc400x_i2c_data *obj)
{
 	obj->reso = &mxc400x_data_resolution[2];
	return MXC400X_SUCCESS;
}

static int MXC400X_ReadOffset(struct i2c_client *client, s8 ofs[MXC400X_AXES_NUM])
{    
	int err;

	err = 0;
#ifdef SW_CALIBRATION
	ofs[0]=ofs[1]=ofs[2]=0x0;
#endif
	/* printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]); */

	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_ResetCalibration(struct i2c_client *client)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);

	int err = 0;

#ifdef GSENSOR_UT
	GSE_FUN();
#endif


	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_ReadCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
    struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
    int  err = 0;
    int mul;

	GSE_FUN();
#ifdef SW_CALIBRATION
	mul = 0;		/* only SW Calibration, disable HW Calibration */
#else
	err = MXC400X_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
    	mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
#endif

	dat[obj->cvt.map[MXC400X_AXIS_X]] =
	    obj->cvt.sign[MXC400X_AXIS_X] * (obj->offset[MXC400X_AXIS_X] * mul +
					    obj->cali_sw[MXC400X_AXIS_X]);
	dat[obj->cvt.map[MXC400X_AXIS_Y]] =
	    obj->cvt.sign[MXC400X_AXIS_Y] * (obj->offset[MXC400X_AXIS_Y] * mul +
					    obj->cali_sw[MXC400X_AXIS_Y]);
	dat[obj->cvt.map[MXC400X_AXIS_Z]] =
	    obj->cvt.sign[MXC400X_AXIS_Z] * (obj->offset[MXC400X_AXIS_Z] * mul +
					    obj->cali_sw[MXC400X_AXIS_Z]);

	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_ReadCalibrationEx(struct i2c_client *client, int act[MXC400X_AXES_NUM],
				    int raw[MXC400X_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data */
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	err = 0;


#ifdef SW_CALIBRATION
	mul = 0;		/* only SW Calibration, disable HW Calibration */
#else
	err = MXC400X_ReadOffset(client, obj->offset);
	if (err) {
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}
		mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
#endif

	raw[MXC400X_AXIS_X] = obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X];
	raw[MXC400X_AXIS_Y] = obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y];
	raw[MXC400X_AXIS_Z] = obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z];

	act[obj->cvt.map[MXC400X_AXIS_X]] = obj->cvt.sign[MXC400X_AXIS_X]*raw[MXC400X_AXIS_X];
	act[obj->cvt.map[MXC400X_AXIS_Y]] = obj->cvt.sign[MXC400X_AXIS_Y]*raw[MXC400X_AXIS_Y];
	act[obj->cvt.map[MXC400X_AXIS_Z]] = obj->cvt.sign[MXC400X_AXIS_Z]*raw[MXC400X_AXIS_Z];                        

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_WriteCalibration(struct i2c_client *client, int dat[MXC400X_AXES_NUM])
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[MXC400X_AXES_NUM], raw[MXC400X_AXES_NUM];

	err = MXC400X_ReadCalibrationEx(client, cali, raw);
	if (0 != err) {	/*offset will be updated in obj->offset */
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
		raw[MXC400X_AXIS_X], raw[MXC400X_AXIS_Y], raw[MXC400X_AXIS_Z],
		obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
		obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[MXC400X_AXIS_X] += dat[MXC400X_AXIS_X];
	cali[MXC400X_AXIS_Y] += dat[MXC400X_AXIS_Y];
	cali[MXC400X_AXIS_Z] += dat[MXC400X_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[MXC400X_AXIS_X], dat[MXC400X_AXIS_Y], dat[MXC400X_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]]);
	obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]]);
	obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]]);	
#else
	int divisor = obj->reso->sensitivity/lsb;//modified
	obj->offset[MXC400X_AXIS_X] = (s8)(obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]])/(divisor));
	obj->offset[MXC400X_AXIS_Y] = (s8)(obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]])/(divisor));
	obj->offset[MXC400X_AXIS_Z] = (s8)(obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[MXC400X_AXIS_X] = obj->cvt.sign[MXC400X_AXIS_X]*(cali[obj->cvt.map[MXC400X_AXIS_X]])%(divisor);
	obj->cali_sw[MXC400X_AXIS_Y] = obj->cvt.sign[MXC400X_AXIS_Y]*(cali[obj->cvt.map[MXC400X_AXIS_Y]])%(divisor);
	obj->cali_sw[MXC400X_AXIS_Z] = obj->cvt.sign[MXC400X_AXIS_Z]*(cali[obj->cvt.map[MXC400X_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[MXC400X_AXIS_X]*divisor + obj->cali_sw[MXC400X_AXIS_X], 
		obj->offset[MXC400X_AXIS_Y]*divisor + obj->cali_sw[MXC400X_AXIS_Y], 
		obj->offset[MXC400X_AXIS_Z]*divisor + obj->cali_sw[MXC400X_AXIS_Z], 
		obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z],
		obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y], obj->cali_sw[MXC400X_AXIS_Z]);

	err = hwmsen_write_block(obj->client, MXC400X_REG_OFSX, obj->offset, MXC400X_AXES_NUM);
	if (err) {
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif
	mdelay(1);
	return err;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC400X_REG_ID;   
	client->addr = 0x15;
	printk("%s, addr=%x\n",__func__, client->addr);
#if defined(SW_SIMULATION)
	res = cust_i2c_master_read(client, databuf, 0x1);
#else
	res = i2c_master_send(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_MXC400X_CheckDeviceID;
	}
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
#endif
	if(res <= 0)
	{
		goto exit_MXC400X_CheckDeviceID;
	}

	
	databuf[0]= (databuf[0]&0x3f);
	
	if(databuf[0]!= MXC400X_ID)
	{
		return MXC400X_ERR_IDENTIFICATION;
	}
	
	GSE_LOG("MXC400X_CheckDeviceID %d done!\n ", databuf[0]);

	return MXC400X_SUCCESS;

	exit_MXC400X_CheckDeviceID:
	if (res <= 0)
	{
		GSE_ERR("MXC400X_CheckDeviceID %d failt!\n ", MXC400X_ERR_I2C);
		return MXC400X_ERR_I2C;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	//struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	
	client->addr = 0x15;
	if(enable == sensor_power)
	{
		printk("Sensor power status is newest!\n");
		return MXC400X_SUCCESS;
	}
	if(enable == 1)
	{
	    printk("MXC400X_SetPowerMode is power on\n");
		databuf[1] = MXC400X_AWAKE;
	}
	else
	{
	    printk("MXC400X_SetPowerMode is power off\n");
		databuf[1] = MXC400X_SLEEP;
	}
	databuf[0] = MXC400X_REG_CTRL;
#if defined(SW_SIMULATION)
	res = cust_i2c_master_send(client, databuf, 0x2);
#else	
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return MXC400X_ERR_I2C;
	}
	sensor_power = enable;
	mdelay(300);//lyon
	
	return MXC400X_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int MXC400X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10] = { 0 };
	int res = 0;
	client->addr = 0x15;

	memset(databuf, 0, sizeof(u8)*10);  
	databuf[0] = MXC400X_REG_CTRL;
	databuf[1] = MXC400X_RANGE_8G;
#if defined(SW_SIMULATION)
	res = cust_i2c_master_send(client, databuf, 0x2);
#else
	res = i2c_master_send(client, databuf, 0x2);
#endif
	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return MXC400X_ERR_I2C;
	}  

	return MXC400X_SetDataResolution(obj);    
}

/*----------------------------------------------------------------------------*/
static int MXC400X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	//int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	
	return MXC400X_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int mxc400x_init_client(struct i2c_client *client, int reset_cali)
{
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();


	res = MXC400X_CheckDeviceID(client);
	if (res != MXC400X_SUCCESS)
		return res;

	res = MXC400X_SetBWRate(client, MXC400X_BW_50HZ);
	if(res != MXC400X_SUCCESS ) 
	{
		return res;
	}


	res = MXC400X_SetDataFormat(client, MXC400X_RANGE_8G);
	if (res != MXC400X_SUCCESS)
		return res;
	/* printk("BMA222_SetBWRate OK!\n"); */

	res = MXC400X_SetPowerMode(client, enable_status);
	if(res != MXC400X_SUCCESS)
		return res;
	/* printk("BMA222_SetDataFormat OK!\n"); */

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


	if (0 != reset_cali) {
		/*reset calibration only in power on */
		 res = MXC400X_ResetCalibration(client);
		 if(res != MXC400X_SUCCESS)
			return res;
	}
	GSE_LOG("mxc400x_init_client OK!\n");
#ifdef CONFIG_MXC400X_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
	mdelay(20);
	return MXC400X_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((NULL == buf) || (bufsize <= 30))
		return -1;

	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MXC400X Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[MXC400X_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	if (NULL == buf)
		return -1;
	if (NULL == client) {
		*buf = 0;
		return -2;
	}

	if (sensor_suspend == 1)
		return 0;

	res = MXC400X_ReadDant(client, obj->data);
	if (res != 0) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
		obj->data[MXC400X_AXIS_X] += obj->cali_sw[MXC400X_AXIS_X];
		obj->data[MXC400X_AXIS_Y] += obj->cali_sw[MXC400X_AXIS_Y];
		obj->data[MXC400X_AXIS_Z] += obj->cali_sw[MXC400X_AXIS_Z];

	acc[obj->cvt.map[MXC400X_AXIS_X]] =
	    obj->cvt.sign[MXC400X_AXIS_X] * obj->data[MXC400X_AXIS_X];
	acc[obj->cvt.map[MXC400X_AXIS_Y]] =
	    obj->cvt.sign[MXC400X_AXIS_Y] * obj->data[MXC400X_AXIS_Y];
	acc[obj->cvt.map[MXC400X_AXIS_Z]] =
	    obj->cvt.sign[MXC400X_AXIS_Z] * obj->data[MXC400X_AXIS_Z];

	acc[MXC400X_AXIS_X] =
	    acc[MXC400X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[MXC400X_AXIS_Y] =
	    acc[MXC400X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	acc[MXC400X_AXIS_Z] =
	    acc[MXC400X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;


	sprintf(buf, "%04x %04x %04x", acc[MXC400X_AXIS_X], acc[MXC400X_AXIS_Y],acc[MXC400X_AXIS_Z]);
//	buf[0] = acc[MXC400X_AXIS_X];
//	buf[1] = acc[MXC400X_AXIS_Y];
//	buf[2] = acc[MXC400X_AXIS_Z];

	if (atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		GSE_LOG("gsensor data: %s!\n", buf);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MXC400X_ReadRawData(struct i2c_client *client, char *buf)
{
	struct mxc400x_i2c_data *obj;
	int res = 0;

	obj = (struct mxc400x_i2c_data *)i2c_get_clientdata(client);

	if (!buf || !client)
		return -EINVAL;

	res = MXC400X_ReadDant(client, obj->data);
	if (0 != res) {
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}

	sprintf(buf, "MXC400X_ReadRawData %04x %04x %04x", obj->data[MXC400X_AXIS_X],
		obj->data[MXC400X_AXIS_Y], obj->data[MXC400X_AXIS_Z]);


	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	char strbuf[MXC400X_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	MXC400X_ReadChipInfo(client, strbuf, MXC400X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}


/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	char strbuf[MXC400X_BUFSIZE];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MXC400X_ReadSensorData(client, strbuf, MXC400X_BUFSIZE);
	/* BMA150_ReadRawData(client, strbuf); */
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}


/*----------------------------------------------------------------------------*/
#if 1
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[MXC400X_AXES_NUM];

	if (NULL == client) {
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	err = MXC400X_ReadOffset(client, obj->offset);
	if (0 != err)
		return -EINVAL;

	err = MXC400X_ReadCalibration(client, tmp);
	if (0 != err)
		return -EINVAL;

		mul = obj->reso->sensitivity/mxc400x_offset_resolution.sensitivity;
	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
		     "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
		     obj->offset[MXC400X_AXIS_X], obj->offset[MXC400X_AXIS_Y],
		     obj->offset[MXC400X_AXIS_Z], obj->offset[MXC400X_AXIS_X],
		     obj->offset[MXC400X_AXIS_Y], obj->offset[MXC400X_AXIS_Z]);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
		     obj->cali_sw[MXC400X_AXIS_X], obj->cali_sw[MXC400X_AXIS_Y],
		     obj->cali_sw[MXC400X_AXIS_Z]);

	len +=
	    snprintf(buf + len, PAGE_SIZE - len,
		     "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			obj->offset[MXC400X_AXIS_X]*mul + obj->cali_sw[MXC400X_AXIS_X],
			obj->offset[MXC400X_AXIS_Y]*mul + obj->cali_sw[MXC400X_AXIS_Y],
			obj->offset[MXC400X_AXIS_Z]*mul + obj->cali_sw[MXC400X_AXIS_Z],
			tmp[MXC400X_AXIS_X], tmp[MXC400X_AXIS_Y], tmp[MXC400X_AXIS_Z]);

	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = mxc400x_i2c_client;  
	int err, x, y, z;
	int dat[MXC400X_AXES_NUM];

	if (!strncmp(buf, "rst", 3)) {
		err = MXC400X_ResetCalibration(client);
		if (0 != err)
			GSE_ERR("reset offset err = %d\n", err);
	} else if (3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
		dat[MXC400X_AXIS_X] = x;
		dat[MXC400X_AXIS_Y] = y;
		dat[MXC400X_AXIS_Z] = z;

		err = MXC400X_WriteCalibration(client, dat);
		if (0 != err)
			GSE_ERR("write calibration err = %d\n", err);
	} else {
		GSE_ERR("invalid format\n");
	}

	return count;
}
#endif

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC400X_LOWPASS
	struct i2c_client *client = mxc400x_i2c_client;
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);

	if (atomic_read(&obj->firlen)) {
		int idx, len = atomic_read(&obj->firlen);

		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for (idx = 0; idx < len; idx++) {
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][MXC400X_AXIS_X],
				obj->fir.raw[idx][MXC400X_AXIS_Y], obj->fir.raw[idx][MXC400X_AXIS_Z]);
		}

		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[MXC400X_AXIS_X],
			obj->fir.sum[MXC400X_AXIS_Y], obj->fir.sum[MXC400X_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[MXC400X_AXIS_X] / len,
			obj->fir.sum[MXC400X_AXIS_Y] / len, obj->fir.sum[MXC400X_AXIS_Z] / len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_MXC400X_LOWPASS
	struct i2c_client *client = mxc400x_i2c_client;  
	struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if (!kstrtoint(buf, 10, &firlen)) {
		GSE_ERR("invallid format\n");
	} else if (firlen > C_MAX_FIR_LENGTH) {
		GSE_ERR("exceeds maximum filter length\n");
	} else {
		atomic_set(&obj->firlen, firlen);
		if (NULL == firlen) {
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
	struct mxc400x_i2c_data *obj = obj_i2c_data;

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
	struct mxc400x_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (!kstrtoint(buf, 16, &trace))
		atomic_set(&obj->trace, trace);
	else
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct mxc400x_i2c_data *obj = obj_i2c_data;

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

	if (sensor_power)
		GSE_LOG("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		GSE_LOG("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", sensor_power);
}
#if 0
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc400x_i2c_client;  
	struct mxc400x_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = mxc400x_i2c_client;  
	struct mxc400x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}
	
	return count;            
}
#endif

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo, S_IWUSR | S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(cali, S_IWUSR | S_IRUGO, show_cali_value, store_cali_value);
static DRIVER_ATTR(firlen, S_IWUSR | S_IRUGO, show_firlen_value, store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
//static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(powerstatus, S_IRUGO, show_power_status_value, NULL);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *mxc400x_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_cali,	/*show calibration data */
	&driver_attr_firlen,	/*filter length: 0: disable, others: enable */
	&driver_attr_trace,	/*trace log */
//	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_powerstatus,
};

/*----------------------------------------------------------------------------*/
static int mxc400x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc400x_attr_list)/sizeof(mxc400x_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, mxc400x_attr_list[idx]);
		if (0 != err) {
			GSE_ERR("driver_create_file (%s) = %d\n", mxc400x_attr_list[idx]->attr.name,
				err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc400x_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc400x_attr_list)/sizeof(mxc400x_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;


	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, mxc400x_attr_list[idx]);


	return err;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int mxc400x_open(struct inode *inode, struct file *file)
{
	 file->private_data = mxc400x_i2c_client;

	if (file->private_data == NULL) {
		 GSE_ERR("null mxc400x!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int mxc400x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
/* static int bma222_ioctl(struct inode *inode, struct file *file, unsigned int cmd, */
/* unsigned long arg) */
static long mxc400x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct mxc400x_i2c_data *obj = (struct mxc400x_i2c_data*)i2c_get_clientdata(client);  
	char strbuf[MXC400X_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];
	//int temp_flag=0;
	//char vec[3] = {0};
	//int value[3] = {0};
	//s8 temp = 0 ;
	//u8 test=0;
	//u8 buf;
	//u8 reg[2];
	//int vec_buf[3] = {0};

	/* GSE_FUN(f); */
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
			mxc400x_init_client(client, 0); 		 
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

			MXC400X_ReadChipInfo(client, strbuf, MXC400X_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
			MXC400X_SetPowerMode(client,true);	
			MXC400X_ReadSensorData(client, strbuf, MXC400X_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		if (copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
			MXC400X_ReadRawData(client, strbuf);
		if (copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
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
			cali[MXC400X_AXIS_X] =
			    sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[MXC400X_AXIS_Y] =
			    sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			cali[MXC400X_AXIS_Z] =
			    sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
			err = MXC400X_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
			err = MXC400X_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		err = MXC400X_ReadCalibration(client, cali);
		if (0 != err)
			break;

		sensor_data.x = cali[MXC400X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.y = cali[MXC400X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		sensor_data.z = cali[MXC400X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;
                
               

//  end.....
	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}


#ifdef CONFIG_COMPAT
static long mxc400x_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA,
					       (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	default:
		GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}
#endif
/*----------------------------------------------------------------------------*/
static struct file_operations mxc400x_fops = {
	.owner = THIS_MODULE,
		 .open = mxc400x_open,
		 .release = mxc400x_release,
		 .unlocked_ioctl = mxc400x_unlocked_ioctl,
#ifdef CONFIG_COMPAT
		.compat_ioctl = mxc400x_compat_ioctl,
#endif
};

static struct miscdevice mxc400x_device = {
		 .minor = MISC_DYNAMIC_MINOR,
		 .name = "gsensor",
		 .fops = &mxc400x_fops,
};

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int mxc400x_suspend(struct i2c_client *client, pm_message_t msg) 
{
#if 0
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);	  
	 int err = 0; 

	if (msg.event == PM_EVENT_SUSPEND) {
		if (obj == NULL) {
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		 mutex_lock(&mxc400x_mutex);
		atomic_set(&obj->suspend, 1);
		err = MXC400X_SetPowerMode(obj->client, false);
		if (err != 0) {
			GSE_ERR("write power control fail!!\n");
			 mutex_unlock(&mxc400x_mutex);
			return -EINVAL;
		}
		 mutex_unlock(&mxc400x_mutex);
	}
	return err;
#endif 
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc400x_resume(struct i2c_client *client)
{
#if 0
	 struct mxc400x_i2c_data *obj = i2c_get_clientdata(client);		  
	int err=0;

	if (obj == NULL) {
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
     mutex_lock(&mxc400x_mutex);
	 if(err == mxc400x_init_client(client, 0))
	 {
		 GSE_ERR("initialize client fail!!\n");
		 mutex_unlock(&mxc400x_mutex);
		 return -EINVAL;		
	 }
	 atomic_set(&obj->suspend, 0);
     mutex_unlock(&mxc400x_mutex);
		return err;
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL */
static int gsensor_open_report_data(int open)
{
	/* should queuq work to report event if  is_report_input_direct=true */
	return 0;
}

/*----------------------------------------------------------------------------*/
/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL */
static int gsensor_enable_nodata(int en)
{
	int err = 0;



	if (((en == 0) && (sensor_power == false)) || ((en == 1) && (sensor_power == true))) {
		enable_status = sensor_power;
		GSE_LOG("Gsensor device have updated!\n");
	} else {
		enable_status = !sensor_power;
		if (atomic_read(&obj_i2c_data->suspend) == 0) {

			err = MXC400X_SetPowerMode(obj_i2c_data->client, enable_status);

			GSE_LOG("Gsensor not in suspend BMA222_SetPowerMode!, enable_status = %d\n",
				enable_status);
		} else {
			GSE_LOG
			    ("Gsensor in suspend and can not enable or disable!enable_status = %d\n",
			     enable_status);
		}
	}


    if(err != MXC400X_SUCCESS)
	{
		GSE_ERR("gsensor_enable_nodata fail!\n");
		return -1;
	}

	GSE_ERR("gsensor_enable_nodata OK!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int gsensor_set_delay(u64 ns)
{


	return 0;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int gsensor_get_data(int *x, int *y, int *z, int *status)
{

	char buff[MXC400X_BUFSIZE];

	int ret = 0;
	MXC400X_ReadSensorData(obj_i2c_data->client, buff, MXC400X_BUFSIZE);

	ret = sscanf(buff, "%x %x %x", x, y, z);
	//*x = buff[0];
	//*y = buff[1];
	*z =  cali_sensor_data[2];
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc400x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	 struct mxc400x_i2c_data *obj;
	struct acc_control_path ctl = { 0 };
	struct acc_data_path data = { 0 };
	int err = 0;
	int retry = 0;

	GSE_FUN();

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}

	 memset(obj, 0, sizeof(struct mxc400x_i2c_data));

	obj->hw = hw;

	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (0 != err) {
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}
#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	INIT_WORK(&obj->irq_work, gsensor_irq_work);
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */

	obj_i2c_data = obj;
	obj->client = client;
#ifdef FPGA_EARLY_PORTING
	obj->client->timing = 100;
#else
	//obj->client->timing = 400;
#endif
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#ifdef CONFIG_MXC400X_LOWPASS
	if (obj->hw->firlen > C_MAX_FIR_LENGTH)
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	else
		atomic_set(&obj->firlen, obj->hw->firlen);

	if (atomic_read(&obj->firlen) > 0)
		atomic_set(&obj->fir_en, 1);
#endif

	mxc400x_i2c_client = new_client;

	for (retry = 0; retry < 3; retry++) {
		err = mxc400x_init_client(new_client, 1);
		if (0 != err) {
			GSE_ERR("mxc400x_device init cilent fail time: %d\n", retry);
			continue;
		}
	}
	if (err != 0)
		goto exit_init_failed;


	err = misc_register(&mxc400x_device);
	if (0 != err) {
		GSE_ERR("mxc400x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	err = mxc400x_create_attr(&mxc400x_init_info.platform_diver_addr->driver);
	if (0 != err) {
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data = gsensor_open_report_data;
	ctl.enable_nodata = gsensor_enable_nodata;
	ctl.set_delay = gsensor_set_delay;
	/* ctl.batch = gsensor_set_batch; */
	ctl.is_report_input_direct = false;

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
	ctl.is_support_batch = obj->hw->is_batch_supported;
#else
	ctl.is_support_batch = false;
#endif

	err = acc_register_control_path(&ctl);
	if (err) {
		GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = gsensor_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if (err) {
		GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}

	err = batch_register_support_info(ID_ACCELEROMETER,
					  ctl.is_support_batch, 102, 0);
	if (err) {
		GSE_ERR("register gsensor batch support err = %d\n", err);
		goto exit_create_attr_failed;
	}
	mutex_init(&mxc400x_mutex);

	gsensor_init_flag = 0;
	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	misc_deregister(&mxc400x_device);
exit_misc_device_register_failed:
exit_init_failed:
	/* i2c_detach_client(new_client); */
exit_kfree:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	gsensor_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc400x_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	err = mxc400x_delete_attr(&mxc400x_init_info.platform_diver_addr->driver);
	if (err != 0)
		GSE_ERR("mxc400x_delete_attr fail: %d\n", err);

	err = misc_deregister(&mxc400x_device);
	if (0 != err)
		GSE_ERR("misc_deregister fail: %d\n", err);

	mxc400x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int  mxc400x_local_init(void)
{
	GSE_FUN();

	mxc400x_power(hw, 1);
	if(i2c_add_driver(&mxc400x_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if (-1 == gsensor_init_flag)
		return -1;
	/* printk("fwq loccal init---\n"); */
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc400x_remove(void)
{
	GSE_FUN();
	 mxc400x_power(hw, 0);	 
	 i2c_del_driver(&mxc400x_i2c_driver);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init mxc400x_driver_init(void)
{
	GSE_FUN();
	hw = get_accel_dts_func(COMPATIABLE_NAME, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");
	GSE_LOG("%s: i2c_number=%d i2c_addr=%x\n", __func__, hw->i2c_num, hw->i2c_addr[0]);
	acc_driver_add(&mxc400x_init_info);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit mxc400x_driver_exit(void)
{
	GSE_FUN();
}

module_init(mxc400x_driver_init);
module_exit(mxc400x_driver_exit);


MODULE_AUTHOR("Lyon Miao<xlmiao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC inc");
MODULE_LICENSE("GPL");
