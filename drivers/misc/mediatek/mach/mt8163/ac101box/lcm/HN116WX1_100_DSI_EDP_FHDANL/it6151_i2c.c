#ifndef BUILD_LK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
//#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/module.h>
#include <linux/init.h>
//#include <cust_acc.h>
//#include <linux/hwmsensor.h>
//#include <linux/hwmsen_dev.h>
//#include <linux/sensors_io.h>
//#include <linux/hwmsen_helper.h>
//#include <linux/xlog.h>
//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>

/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define IT6151_EDP_SLAVE_ADDR_WRITE						(0x5C << 0)
#define	IT6151_MIPIRX_SLAVE_ADDR_WRITE 				(0x6C << 0)

#define DEVICE_NAME "IT6151"

static struct i2c_client *it6151_mipirx = NULL;
static struct i2c_client *it6151_edp = NULL;

static const struct i2c_device_id it6151_i2c_id[] = 
{
	{"it6151_edp",		0},
	{"it6151_mipirx",	0},
};

static int it6151_i2c_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);

static struct i2c_driver it6151_i2c_driver = {
  .driver = {
      .name    = DEVICE_NAME,
  },
  .probe       = it6151_i2c_driver_probe,
  .id_table    = it6151_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
static DEFINE_MUTEX(it6151_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write fan5405] 
  *
  *********************************************************/
int it6151_i2c_read_byte(unsigned char dev_addr, unsigned char addr, unsigned char *returnData)
{
  char     cmd_buf[1]={0x00};
  char     readData = 0;
  int      ret=0;

#ifdef IT6151_DEBUG
	/* dump write_data for check */
  printk("[KE/it6151_read_byte] dev_addr = 0x%x, read_data[0x%x] = 0x%x \n", dev_addr, addr, *returnData);
#endif

  mutex_lock(&it6151_i2c_access);

	if(dev_addr == IT6151_MIPIRX_SLAVE_ADDR_WRITE)
	{
#if defined(CONFIG_MTK_I2C_EXTENSION)		
		it6151_mipirx->ext_flag=((it6151_mipirx->ext_flag) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
#endif		
		cmd_buf[0] = addr;
		ret = i2c_master_send(it6151_mipirx, &cmd_buf[0], (1<<8 | 1));
		
		if (ret < 0)
		{
#if defined(CONFIG_MTK_I2C_EXTENSION)
			it6151_mipirx->ext_flag=0;
#endif
			mutex_unlock(&it6151_i2c_access);
			
			return 0;
		}
		
		readData = cmd_buf[0];
		*returnData = readData;
		
#if defined(CONFIG_MTK_I2C_EXTENSION)		
		it6151_mipirx->ext_flag=0;
#endif
	}
	else if(dev_addr == IT6151_EDP_SLAVE_ADDR_WRITE)
	{
#if defined(CONFIG_MTK_I2C_EXTENSION)
	  it6151_edp->ext_flag=((it6151_edp->ext_flag) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
#endif
	  cmd_buf[0] = addr;
	  ret = i2c_master_send(it6151_edp, &cmd_buf[0], (1<<8 | 1));
		
	  if (ret < 0)
	  {
#if defined(CONFIG_MTK_I2C_EXTENSION)
	    it6151_edp->ext_flag=0;
#endif
	    mutex_unlock(&it6151_i2c_access);

			return 0;
	  }
	  
	  readData = cmd_buf[0];
	  *returnData = readData;
#if defined(CONFIG_MTK_I2C_EXTENSION)
	  it6151_edp->ext_flag=0;
#endif
	}
	else
	{
		printk("[it6151_i2c_read_byte]error:  no this dev_addr! \n");
	}
	
  mutex_unlock(&it6151_i2c_access);
	
  return 1;
}

#if 0
//int it6151_i2c_read_byte_2(struct i2c_client *client, u8 addr, u8 *data, u8 len)
int it6151_i2c_read_byte_2(unsigned char dev_addr, unsigned char addr, unsigned char *returnData)
{
	u8 beg = addr; 
	struct i2c_msg msgs[2] = 
	{
		{
			.addr = dev_addr,	 
			.flags = 0,
			.len = 1,				 
			.buf= &beg
		},
		{
			.addr = dev_addr,	 
			.flags = I2C_M_RD,
			.len = len, 			 
			.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) 
	{		 
		GSE_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		GSE_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else 
	{
		err = 0;/*no error*/
	}
	return err;
}
#endif

int it6151_i2c_write_byte(unsigned char dev_addr, unsigned char addr, unsigned char writeData)
{
  char    write_data[2] = {0};
  int     ret=0;

#ifdef IT6151_DEBUG
  /* dump write_data for check */
	printk("[KE/it6151_i2c_write] dev_addr = 0x%x, write_data[0x%x] = 0x%x \n", dev_addr, addr, writeData);
#endif
  
  mutex_lock(&it6151_i2c_access);
  
  write_data[0] = addr;
  write_data[1] = writeData;

	if(dev_addr == IT6151_MIPIRX_SLAVE_ADDR_WRITE)
	{
		it6151_mipirx->addr = IT6151_MIPIRX_SLAVE_ADDR_WRITE;
#if defined(CONFIG_MTK_I2C_EXTENSION)
		it6151_mipirx->ext_flag=((it6151_mipirx->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;
#endif
		ret = i2c_master_send(it6151_mipirx, write_data, 2);
		
		if(ret < 0)
		{
#if defined(CONFIG_MTK_I2C_EXTENSION)
			it6151_mipirx->ext_flag=0;
#endif
			mutex_unlock(&it6151_i2c_access);
			
			return 0;
		}
#if defined(CONFIG_MTK_I2C_EXTENSION)		
		it6151_mipirx->ext_flag=0;
#endif
	}
	else if(dev_addr == IT6151_EDP_SLAVE_ADDR_WRITE)
	{
		it6151_edp->addr = IT6151_EDP_SLAVE_ADDR_WRITE;
#if defined(CONFIG_MTK_I2C_EXTENSION)
	  it6151_edp->ext_flag=((it6151_edp->ext_flag) & I2C_MASK_FLAG) | I2C_DIRECTION_FLAG;
#endif
	  ret = i2c_master_send(it6151_edp, write_data, 2);

		if (ret < 0)
	  {
#if defined(CONFIG_MTK_I2C_EXTENSION)
	    it6151_edp->ext_flag=0;
#endif
	    mutex_unlock(&it6151_i2c_access);
			
	    return 0;
	  }
#if defined(CONFIG_MTK_I2C_EXTENSION)
	  it6151_edp->ext_flag=0;
#endif
	}	
	else
	{
     printk("[it6151_i2c_write_byte]error:  no this dev_addr! \n");
	}
	
  mutex_unlock(&it6151_i2c_access);
	
  return 1;
}

static int match_id(const struct i2c_client *client, const struct i2c_device_id *id)
{
	if (strcmp(client->name, id->name) == 0)
		return true;
	else
		return false;
}

static int it6151_i2c_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
  int err=0; 

	printk("[it6151_i2c_driver_probe] start!\n");

	if(match_id(client, &it6151_i2c_id[0]))
	{
	  if (!(it6151_mipirx = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) 
		{
	    err = -ENOMEM;
	    goto exit;
	  }
		
	  memset(it6151_mipirx, 0, sizeof(struct i2c_client));

	  it6151_mipirx = client;    
	}
	else if(match_id(client, &it6151_i2c_id[1]))
	{
		if (!(it6151_edp = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) 
		{
			err = -ENOMEM;
			goto exit;
		}
		
		memset(it6151_edp, 0, sizeof(struct i2c_client));
	
		it6151_edp = client; 	 
	}
	else
	{
		printk("[it6151_i2c_driver_probe] error!\n");

		err = -EIO;
		goto exit;
	}

	printk("[it6151_i2c_driver_probe] %s i2c sucess!\n", client->name);
	
  return 0;

exit:
  return err;

}

#define IT6151_BUSNUM 2

static struct i2c_board_info __initdata it6151_I2C[] = 
{
	{I2C_BOARD_INFO("it6151_edp", 		IT6151_EDP_SLAVE_ADDR_WRITE)},
	{I2C_BOARD_INFO("it6151_mipirx", 	IT6151_MIPIRX_SLAVE_ADDR_WRITE)},
};

static int __init it6151_init(void)
{    
  printk("[it6151_init] init start\n");
  
  i2c_register_board_info(IT6151_BUSNUM, it6151_I2C, 2);

  if(i2c_add_driver(&it6151_i2c_driver)!=0)
  {
    printk("[it6151_init] failed to register it6151 i2c driver.\n");
  }
  else
  {
    printk("[it6151_init] Success to register it6151 i2c driver.\n");
  }

  return 0;
}

static void __exit it6151_exit(void)
{
  i2c_del_driver(&it6151_i2c_driver);
}

module_init(it6151_init);
module_exit(it6151_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C it6151 Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
#endif
