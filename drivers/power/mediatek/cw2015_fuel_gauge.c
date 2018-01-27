#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>


#define CW2015_ENABLE_LOG 1 //CHANGE   Customer need to change this for enable/disable log
#define CW2015_I2C_BUSNUM 0 //CHANGE   Customer need to change this number according to the principle of hardware
#define USB_CHARGING_FILE "/sys/class/power_supply/usb/online" // Chaman
#define DC_CHARGING_FILE "/sys/class/power_supply/ac/online"


#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_VTEMPL              0xC
#define REG_VTEMPH              0xD
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        // ATHD = 0%

#define BATTERY_UP_MAX_CHANGE   420             // the max time allow battery change quantity
#define BATTERY_DOWN_CHANGE   60                // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE_RUN 30          // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min
#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE 1800

#define CHARGING_ON 1
#define NO_CHARGING 0


#define cw2015_printk(fmt, arg...)        printk("cw2015 : %s : " fmt, __FUNCTION__ ,##arg)

#define CW2015_NAME "cw2015_fgadc"

#define SIZE_BATINFO    64

static u8 config_info[SIZE_BATINFO] = {
			0x15,0x7F,0x60,0x62,0x4D,0x77,0x52,0x42,
			0x4C,0x4B,0x48,0x40,0x3A,0x38,0x37,0x31,
			0x25,0x15,0x12,0x08,0x11,0x25,0x3A,0x4E,
			0x41,0x17,0x0D,0x71,0x19,0x32,0x41,0x56,
			0x68,0x66,0x63,0x6A,0x3D,0x1B,0x60,0x2D,
			0x00,0x2B,0x52,0x87,0x8F,0x91,0x94,0x52,
			0x82,0x8C,0x92,0x96,0x83,0x70,0x95,0xCB,
			0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x3E,0x31,
			/*0x15,0x89,0x5C,0x5D,0x4D,0x6F,0x5B,0x42,
			0x4E,0x4C,0x4A,0x41,0x3D,0x3A,0x38,0x31,
			0x25,0x1C,0x0F,0x0E,0x0E,0x24,0x3B,0x4A,
			0x45,0x1E,0x0D,0x71,0x1B,0x36,0x40,0x54,
			0x6D,0x71,0x70,0x73,0x3F,0x1B,0x73,0x37,
			0x00,0x2A,0x52,0x87,0x8F,0x91,0x94,0x52,
			0x82,0x8C,0x92,0x96,0x99,0x75,0x92,0xCB,
			0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x46,0xAE,*/
};

struct cw_battery {
    struct i2c_client *client;

    struct workqueue_struct *cw2015_workqueue;
	struct delayed_work charger_detect_work;

    long sleep_time_capacity_change;
    long run_time_capacity_change;

    long sleep_time_charge_start;
    long run_time_charge_start;

    int charger_mode;
    int capacity;
    int voltage;
    int status;
    int time_to_empty;
    //int alt;
};

static struct cw_battery *cw_bat = NULL;
static int cw_probe_ok = 0;
int cw_update_config_info(struct cw_battery *cw_bat);

int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
	int ret = 0;
	//ret = i2c_smbus_read_byte_data(client, reg);
	msleep(10);	
	ret = i2c_smbus_read_i2c_block_data( client, reg, 1, buf );
	/*
	if(ret < 0) {
		return ret;
	} */
	cw2015_printk("%2x = %2x\n", reg, buf[0]);
	return ret;
}
		
int cw_write(struct i2c_client *client, u8 reg, u8 const buf[])
{
	int ret = 0;
	//ret = i2c_smbus_write_byte_data(client, reg, buf[0]);
	msleep(10);	
	ret = i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0] );
	cw2015_printk("%2x = %2x\n", reg, buf[0]);
	return ret;
}
	
int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
	int ret = 0;
	//unsigned int data = 0;
	//data = i2c_smbus_read_word_data(client, reg);
	msleep(10);	
	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, buf );
	cw2015_printk("%2x = %2x %2x\n", reg, buf[0], buf[1]);
	//buf[0] = data & 0x00FF;
	//buf[1] = (data & 0xFF00)>>8;
	return ret;
}

static void cw_update_time_member_charge_start(struct cw_battery *cw_bat)
{
        struct timespec ts;
        int new_run_time;
        int new_sleep_time;

        ktime_get_ts(&ts);
        new_run_time = ts.tv_sec;

        get_monotonic_boottime(&ts);
        new_sleep_time = ts.tv_sec - new_run_time;

        cw_bat->run_time_charge_start = new_run_time;
        cw_bat->sleep_time_charge_start = new_sleep_time; 
}

static void cw_update_time_member_capacity_change(struct cw_battery *cw_bat)          
{
        struct timespec ts;
        int new_run_time;
        int new_sleep_time;

        ktime_get_ts(&ts);
        new_run_time = ts.tv_sec;

        get_monotonic_boottime(&ts);
        new_sleep_time = ts.tv_sec - new_run_time;

        cw_bat->run_time_capacity_change = new_run_time;
        cw_bat->sleep_time_capacity_change = new_sleep_time; 
}
/*
static int cw_quickstart(struct cw_battery *cw_bat)      
{
        int ret = 0;
        u8 reg_val = MODE_QUICK_START;

        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if(ret < 0) {
                return ret;
        }
        
        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if(ret < 0) {
                return ret;
        }
        return 1;
}
*/
int cw2015_get_capacity(void)
{
			int cw_capacity;
			int ret;
			u8 reg_val[2];
	
			struct timespec ts;
			long new_run_time;
			long new_sleep_time;
			long capacity_or_aconline_time;
			int allow_change;
			int allow_capacity;
			static int if_quickstart = 0;
			static int jump_flag =0;
			static int reset_loop =0;
			int charge_time;
			u8 reset_val;
	
	
			cw2015_printk("\n");
			while(!cw_probe_ok){
				msleep(1000);
			}
			// ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
			ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
			if (ret < 0)
					return ret;
	
			cw_capacity = reg_val[0];
			if ((cw_capacity < 0) || (cw_capacity > 100)) {
					cw2015_printk("Error:  cw_capacity = %d\n", cw_capacity);
					reset_loop++;
					
				if (reset_loop >5){ 
					
					reset_val = MODE_SLEEP; 			  
					ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
					if (ret < 0)
						return ret;
					reset_val = MODE_NORMAL;
					msleep(10);
					ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
					if (ret < 0)
						return ret;
					ret = cw_update_config_info(cw_bat);
					if (ret) 
						return ret;
					reset_loop =0;	
								 
				}
										 
				return cw_bat->capacity; //cw_capacity Chaman change because I think customer didn't want to get error capacity.
			}else {
				reset_loop =0;
			}
	
			if (cw_capacity == 0) 
					cw2015_printk("the cw201x capacity is 0 !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
			else 
					cw2015_printk("the cw201x capacity is %d, funciton: %s\n", cw_capacity, __func__);

				
			ktime_get_ts(&ts);
			new_run_time = ts.tv_sec;
	
			get_monotonic_boottime(&ts);
			new_sleep_time = ts.tv_sec - new_run_time;

			/* case 1 : aviod swing */
			if (((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity > (cw_bat->capacity - 9)))
							|| ((cw_bat->charger_mode == 0) && (cw_capacity == (cw_bat->capacity + 1)))) {
	
					if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) { 		
								cw_capacity = cw_bat->capacity;
						}
			}
	  		/* case 2 : aviod no charge full */
			if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {
	
					capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
					capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
					allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
					if (allow_change > 0) {
							allow_capacity = cw_bat->capacity + allow_change; 
							cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
							jump_flag =1;
					} else if (cw_capacity <= cw_bat->capacity) {
							cw_capacity = cw_bat->capacity; 
					}
	
			}	   
	  		/*case 3 : avoid battery level jump to CW_BAT */
			else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {
					capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
					capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
					allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
					if (allow_change > 0) {
							allow_capacity = cw_bat->capacity - allow_change; 
							if (cw_capacity >= allow_capacity){
								jump_flag =0;
							}
							else{
									cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
							}
					} else if (cw_capacity <= cw_bat->capacity) {
							cw_capacity = cw_bat->capacity;
					}
			}
			/*case 4 :  avoid battery level jump to 0% at a moment from more than 2% */
			if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {
					allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
					allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);
	
					allow_capacity = cw_bat->capacity - allow_change;
					cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;
					reg_val[0] = MODE_NORMAL;
					ret = cw_write(cw_bat->client, REG_MODE, reg_val);
					if (ret < 0)
						return ret;   
			}
	  /*充电时，Soc长时间为0，则POR*/
#if 1	
		if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
		{		  
					charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
					if ((charge_time > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
						   reset_val = MODE_SLEEP;				 
					   ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
					   if (ret < 0)
						  return ret;
					   reset_val = MODE_NORMAL;
					   msleep(10);
					   ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
					   if (ret < 0)
						  return ret;
					   ret = cw_update_config_info(cw_bat);
					   if (ret) 
						  return ret;
							if_quickstart = 1;
					}
		} else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0)) {
				if_quickstart = 0;
			}
	
#endif

#ifdef SYSTEM_SHUTDOWN_VOLTAGE
			if ((cw_bat->charger_mode == 0) && (cw_capacity <= 20) && (cw_bat->voltage <= SYSTEM_SHUTDOWN_VOLTAGE)){			 
					if (if_quickstart == 10){  
						
						allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
						allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);
	
						allow_capacity = cw_bat->capacity - allow_change;
						cw_capacity = (allow_capacity >= 0) ? allow_capacity: 0;
						
						if (cw_capacity < 1){					
							cw_quickstart(cw_bat);
							if_quickstart = 12;
							cw_capacity = 0;
						}
					} else if (if_quickstart <= 10)
							if_quickstart =if_quickstart+2;
			} else if ((cw_bat->charger_mode > 0)&& (if_quickstart <= 12)) {
					if_quickstart = 0;
			}
#endif
			if(cw_capacity >= 0 && cw_capacity <= 100 && cw_capacity != cw_bat->capacity){
				cw_bat->capacity = cw_capacity;
				cw_update_time_member_capacity_change(cw_bat);
			}
			return cw_bat->capacity;

}
/*
static int cw2015_get_time_to_empty(void)
{
        int ret;
        u8 reg_val;
        u16 value16;

        ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
        if (ret < 0)
                return ret;

        value16 = reg_val;

        ret = cw_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
        if (ret < 0)
                return ret;

        value16 = ((value16 << 8) + reg_val) & 0x1fff;
		if(value16 >= 0 && cw_bat->time_to_empty != value16)
			cw_bat->time_to_empty = value16;
		
        return cw_bat->time_to_empty;
}
*/
int cw2015_get_voltage(void)
{    
    int ret;
    u8 reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage;
    
    cw2015_printk("\n");
	while(!cw_probe_ok){
		msleep(1000);
	}
    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
        return ret;
    }
    value16 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
          return ret;
    }
    value16_1 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if(ret < 0) {
        return ret;
    }
    value16_2 = (reg_val[0] << 8) + reg_val[1];

    if(value16 > value16_1) {     
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    if(value16_1 > value16_2) {
    value16_3 =value16_1;
    value16_1 =value16_2;
    value16_2 =value16_3;
    }

    if(value16 >value16_1) {     
    value16_3 =value16;
    value16 =value16_1;
    value16_1 =value16_3;
    }            

    voltage = value16_1 * 312 / 1024;
    cw2015_printk("[FGADC] cw_get_vol voltage = %d\n", voltage);

	if(voltage >= 0 && cw_bat->voltage != voltage)
		cw_bat->voltage = voltage;
    return cw_bat->voltage;
}


int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val;
    int i;
    u8 reset_val;

    cw2015_printk("\n");
    cw2015_printk("[FGADC] test config_info = 0x%x\n",config_info[0]);

    
    // make sure no in sleep mode
    ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
    if(ret < 0) {
        return ret;
    }

    reset_val = reg_val;
    if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        return -1;
    }

    // update new battery info
    for (i = 0; i < SIZE_BATINFO; i++) {
        ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info[i]);
        if(ret < 0) 
			return ret;
    }

    reg_val |= CONFIG_UPDATE_FLG;   // set UPDATE_FLAG
    reg_val &= 0x07;                // clear ATHD
    reg_val |= ATHD;                // set ATHD
    ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
    if(ret < 0) 
		return ret;
    // read back and check
    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if(ret < 0) {
        return ret;
    }

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
		printk("Error: The new config set fail\n");
		//return -1;
    }

    if ((reg_val & 0xf8) != ATHD) {
		printk("Error: The new ATHD set fail\n");
		//return -1;
    }

    // reset
    reset_val &= ~(MODE_RESTART);
    reg_val = reset_val | MODE_RESTART;
    ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
    if(ret < 0) return ret;

    msleep(10);
    
    ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
    if(ret < 0) return ret;
	
	cw2015_printk("cw2015 update config success!\n");
	
    return 0;
}

static int cw2015_init(struct cw_battery *cw_bat)
{
    int ret;
    int i;
    u8 reg_val = MODE_SLEEP;

	/* Didn't check the MODE register, write normal mode to the MODE register */
	/*
    ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
    if (ret < 0)
        return ret;
	*/
	
    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        reg_val = MODE_NORMAL;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0) 
            return ret;
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
    	return ret;

    if ((reg_val & 0xf8) != ATHD) {
        reg_val &= 0x07;    /* clear ATHD */
        reg_val |= ATHD;    /* set ATHD */
        ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0)
            return ret;
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0) 
        return ret;

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
		cw2015_printk("update config flg is true, need update config\n");
        ret = cw_update_config_info(cw_bat);
        if (ret < 0) {
			printk("%s : update config fail\n", __func__);
            return ret;
        }
    } else {
    	for(i = 0; i < SIZE_BATINFO; i++) { 
        ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
        if (ret < 0)
        	return ret;
                        
        if (config_info[i] != reg_val)
            break;
        }
        if (i != SIZE_BATINFO) {
			cw2015_printk("config didn't match, need update config\n");
        	ret = cw_update_config_info(cw_bat);
            if (ret < 0)
                                return ret;
        }
    }

        for (i = 0; i < 30; i++) {
                ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
                if (ret < 0)
                        return ret;
                else if (reg_val <= 0x64) 
                        break;
                
                msleep(120);
        }
        if (i >=30 ){
        	 reg_val = MODE_SLEEP;
             ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
             cw2015_printk("cw2015 input unvalid power error, cw2015 join sleep mode\n");
             return -1;
        } 

		cw2015_printk("cw2015 init success!\n");	
        return 0;
}

int check_charging_state(const char *filename)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int read_size = 8;
	int state = 0;
	char buf[read_size];
	int ret;

	cw2015_printk("\n");
	fp = filp_open(filename, O_RDONLY, 0644);
	if (IS_ERR(fp))
		return -1;
	
	fs = get_fs();
	set_fs(KERNEL_DS);
	
	pos = 0;
	ret = vfs_read(fp, buf, read_size, &pos);
	if(ret < 0)
		return -1;
	
	filp_close(fp,NULL);
	set_fs(fs);
	state = buf[0] - '0';
	cw2015_printk(" filename = %s  state = %d \n", filename, state);
	return state;
}

static void charger_detect_function(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	int if_charging = 0;

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, charger_detect_work);
	/* Chaman need to fix here
		1, check usb charger
		2, check dc charger
		3, change the value cw_bat->charger_mode
		4, if charger state change, update time member charger start --cw_update_time_member_charge_start
	*/
	if(check_charging_state(USB_CHARGING_FILE) == 1 
		|| check_charging_state(DC_CHARGING_FILE) == 1)
	{
		if_charging = CHARGING_ON;
	}else{
		if_charging = NO_CHARGING;
	}
	if(if_charging != cw_bat->charger_mode){
		cw_bat->charger_mode = if_charging;
		cw_update_time_member_charge_start(cw_bat);
	}

	queue_delayed_work(cw_bat->cw2015_workqueue, &cw_bat->charger_detect_work , msecs_to_jiffies(1000));
}

static int cw2015_fgadc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    int loop = 0;
    //int ret_device_file = 0;

    cw_bat = kmalloc(sizeof(struct cw_battery), GFP_KERNEL);
    if (!cw_bat) {
		cw2015_printk("cw_bat create fail!\n");
        return -ENOMEM;
    }
printk("%s,name=%s,i2c_addr=%x\n",__FUNCTION__,client->name,client->addr);


    i2c_set_clientdata(client, cw_bat);

    cw_bat->client = client;
    cw_bat->capacity = 1;
    cw_bat->voltage = 0;
    cw_bat->status = 0;
	cw_bat->charger_mode = NO_CHARGING;
	
    msleep(1500);
    ret = cw2015_init(cw_bat);
    while ((loop++ < 200) && (ret != 0)) {
	msleep(50);
        ret = cw2015_init(cw_bat);
    }
    if (ret) {
		printk("%s : cw2015 init fail!\n", __func__);
        return ret;	
    }
	cw_update_time_member_capacity_change( cw_bat);
	cw_update_time_member_charge_start(cw_bat);

	cw_bat->cw2015_workqueue = create_singlethread_workqueue("cw2015_gauge");
	INIT_DELAYED_WORK(&cw_bat->charger_detect_work, charger_detect_function);
	queue_delayed_work(cw_bat->cw2015_workqueue, &cw_bat->charger_detect_work , msecs_to_jiffies(50));
	cw_probe_ok = 1;
	
	cw2015_printk("cw2015 driver probe success!\n");
    return 0;
}


static int cw2015_fgadc_remove(struct i2c_client *client)	 
{
	cw2015_printk("\n");
	return 0;
}

/*
static int cw2015_fgadc_detect(struct i2c_client *client, struct i2c_board_info *info) 
{	 
	cw2015_printk("\n");
	strcpy(info->type, CW2015_NAME);
	return 0;
}
*/
static const struct i2c_device_id cw2015_i2c_id[] = {
	{CW2015_NAME, 0},
	{}
};
#ifdef CONFIG_OF
static const struct of_device_id cw2015_fgadc_of_match[] = {
	{.compatible = "mediatek,cw2015_fgadc"},
	{},
};
#endif

struct i2c_driver cw2015_fgadc_driver = {
	.probe		  = cw2015_fgadc_probe,
	.remove 	  = cw2015_fgadc_remove,
	//.detect 	  = cw2015_fgadc_detect,
	.driver 	  = {
		.name = CW2015_NAME,
#ifdef CONFIG_OF
	    .of_match_table = cw2015_fgadc_of_match,
#endif
			
	},
	.id_table = cw2015_i2c_id,
};
/*
static struct i2c_board_info __initdata fgadc_dev = { 
	I2C_BOARD_INFO(CW2015_NAME, 0x62) 
};

static int cw2015_platform_probe(struct platform_device *pdev) 
{
	cw2015_printk("1 \n");
	msleep(200);
	if(i2c_add_driver(&cw2015_fgadc_driver))
	{
		printk("add driver error\n");
		return -1;
	} 
	return 0;
}*/
/*
static int cw2015_platform_remove(struct platform_device *pdev)
{   
	i2c_del_driver(&cw2015_fgadc_driver);
	return 0;
}
*/
/*static struct platform_driver cw2015_platform_driver = {
	.probe      = cw2015_platform_probe,
	.remove     = cw2015_platform_remove,    
	.driver     = {
		.name  = "cw2015_platform",	
	}
};*/

/*
struct platform_device cw2015_platform_device = {
    .name   = "cw2015_platform",
    .id     = -1,
};
*/

static int __init cw2015_fgadc_init(void)
{
	cw2015_printk("\n");
	printk("%s 1\n", __func__);
   // i2c_register_board_info(CW2015_I2C_BUSNUM, &fgadc_dev, 1);
	//platform_device_register(&cw2015_platform_device);
	//platform_driver_register(&cw2015_platform_driver);
    i2c_add_driver(&cw2015_fgadc_driver);
    return 0; 
}


static void __exit cw2015_fgadc_exit(void)
{
	//platform_driver_unregister(&cw2015_platform_driver);
    i2c_del_driver(&cw2015_fgadc_driver);
}

//late_initcall(cw2015_fgadc_init);   //use fs_initcall to make cw2015 registration earlier
late_initcall(cw2015_fgadc_init);
module_exit(cw2015_fgadc_exit);

MODULE_AUTHOR("Chaman Qi");
MODULE_DESCRIPTION("CW2015 FGADC Device Driver");
MODULE_LICENSE("GPL");
