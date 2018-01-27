#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/switch.h>
  
static struct switch_dev lcm_name_data;
extern char *lcm_name;
static int lcm_name_init(void)
{
        int ret=0;

		if(lcm_name==NULL)
			{
			printk("[lcm_name] err lcm name is null!!\n");
		return 1;	
			}
	    lcm_name_data.name =lcm_name ;
		lcm_name_data.index = 0;
	
		
		ret = switch_dev_register(&lcm_name_data);
		if(ret)
		{
			printk("[lcm_name]switch_dev_register returned:%d!\n", ret);
			return 1;
		}
		return 0;

}
static void lcm_name_exit(void)
{
  switch_dev_unregister(&lcm_name_data);

}

late_initcall(lcm_name_init);
module_exit(lcm_name_exit);

MODULE_DESCRIPTION("emdoor lcm name driver");
MODULE_AUTHOR("colin <colin.zeng@emdoor.com>");
MODULE_LICENSE("GPL");

