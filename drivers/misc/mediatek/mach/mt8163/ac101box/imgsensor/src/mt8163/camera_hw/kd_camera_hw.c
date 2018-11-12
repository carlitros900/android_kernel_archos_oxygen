#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG printk//PK_DBG_FUNC	
#define PK_ERR(fmt, arg...) pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  printk(PFX  fmt, ##args)

#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif


/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
//struct pinctrl_state *cam_ldo0_h = NULL;
//struct pinctrl_state *cam_ldo0_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h)) {
		ret = PTR_ERR(cam0_pnd_h);
		pr_debug("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l)) {
		ret = PTR_ERR(cam0_pnd_l);
		pr_debug("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}


	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h)) {
		ret = PTR_ERR(cam0_rst_h);
		pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l)) {
		ret = PTR_ERR(cam0_rst_l);
		pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h)) {
		ret = PTR_ERR(cam1_pnd_h);
		pr_debug("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l)) {
		ret = PTR_ERR(cam1_pnd_l);
		pr_debug("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}


	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h)) {
		ret = PTR_ERR(cam1_rst_h);
		pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
	}


	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l)) {
		ret = PTR_ERR(cam1_rst_l);
		pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
	}
	/*externel LDO enable */
#if 0//add by tubao  we have not use the ldo
	cam_ldo0_h = pinctrl_lookup_state(camctrl, "cam_ldo0_1");
	if (IS_ERR(cam_ldo0_h)) {
		ret = PTR_ERR(cam_ldo0_h);
		pr_debug("%s : pinctrl err, cam_ldo0_h\n", __func__);
	}


	cam_ldo0_l = pinctrl_lookup_state(camctrl, "cam_ldo0_0");
	if (IS_ERR(cam_ldo0_l)) {
		ret = PTR_ERR(cam_ldo0_l);
		pr_debug("%s : pinctrl err, cam_ldo0_l\n", __func__);
	}
#endif//add by tubao  <<<<========
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	switch (PwrType) {
	case CAMRST:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_rst_l);
			else
				pinctrl_select_state(camctrl, cam0_rst_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_rst_l);
			else
				pinctrl_select_state(camctrl, cam1_rst_h);
		}
		break;
	case CAMPDN:
		if (PinIdx == 0) {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else
				pinctrl_select_state(camctrl, cam0_pnd_h);
		} else {
			if (Val == 0)
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else
				pinctrl_select_state(camctrl, cam1_pnd_h);
		}

		break;
	case CAMLDO:
#if 0//add by tubao ==>>> start  we  have not use the ldo
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo0_l);
		else
			pinctrl_select_state(camctrl, cam_ldo0_h);
#endif//add by tubao <<<=========end
		break;
	default:
		PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}




int cntVCAMD = 0;
int cntVCAMA = 0;
int cntVCAMIO = 0;
int cntVCAMAF = 0;
int cntVCAMD_SUB = 0;

static DEFINE_SPINLOCK(kdsensor_pw_cnt_lock);


bool _hwPowerOnCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, int powerVolt, char *mode_name)
{
	if (_hwPowerOn(PinIdx, powerId, powerVolt)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD += 1;
		else if (powerId == VCAMA)
			cntVCAMA += 1;
		else if (powerId == VCAMIO)
			cntVCAMIO += 1;
		else if (powerId == VCAMAF)
			cntVCAMAF += 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB += 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

bool _hwPowerDownCnt(int PinIdx, KD_REGULATOR_TYPE_T powerId, char *mode_name)
{

	if (_hwPowerDown(PinIdx, powerId)) {
		spin_lock(&kdsensor_pw_cnt_lock);
		if (powerId == VCAMD)
			cntVCAMD -= 1;
		else if (powerId == VCAMA)
			cntVCAMA -= 1;
		else if (powerId == VCAMIO)
			cntVCAMIO -= 1;
		else if (powerId == VCAMAF)
			cntVCAMAF -= 1;
		else if (powerId == SUB_VCAMD)
			cntVCAMD_SUB -= 1;
		spin_unlock(&kdsensor_pw_cnt_lock);
		return true;
	}
	return false;
}

void checkPowerBeforClose(int PinIdx, char *mode_name)
{

	int i = 0;

	PK_DBG
	    ("[checkPowerBeforClose]cntVCAMD:%d, cntVCAMA:%d,cntVCAMIO:%d, cntVCAMAF:%d, cntVCAMD_SUB:%d,\n",
	     cntVCAMD, cntVCAMA, cntVCAMIO, cntVCAMAF, cntVCAMD_SUB);


	for (i = 0; i < cntVCAMD; i++)
		_hwPowerDown(PinIdx, VCAMD);
	for (i = 0; i < cntVCAMA; i++)
		_hwPowerDown(PinIdx, VCAMA);
	for (i = 0; i < cntVCAMIO; i++)
		_hwPowerDown(PinIdx, VCAMIO);
	for (i = 0; i < cntVCAMAF; i++)
		_hwPowerDown(PinIdx, VCAMAF);
	for (i = 0; i < cntVCAMD_SUB; i++)
		_hwPowerDown(PinIdx, SUB_VCAMD);

	cntVCAMD = 0;
	cntVCAMA = 0;
	cntVCAMIO = 0;
	cntVCAMAF = 0;
	cntVCAMD_SUB = 0;

}



int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On,
		       char *mode_name)
{

	u32 pinSetIdx = 0;
	static int count_tmp = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

	u32 pinSet[3][8] = {

		{CAMERA_CMRST_PIN,
		 CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 CAMERA_CMPDN_PIN,
		 CAMERA_CMPDN_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{CAMERA_CMRST1_PIN,
		 CAMERA_CMRST1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 CAMERA_CMPDN1_PIN,
		 CAMERA_CMPDN1_PIN_M_GPIO,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 },
		{GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,	/* mode */
		 GPIO_OUT_ONE,	/* ON state */
		 GPIO_OUT_ZERO,	/* OFF state */
		 GPIO_CAMERA_INVALID,
		 GPIO_CAMERA_INVALID,
		 GPIO_OUT_ONE,
		 GPIO_OUT_ZERO,
		 }
	};


	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	 else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	 else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;
	count_tmp ++;
	printk("tubao ==>>> kdCISModulePowerOn  count_tmp = %d   \n",count_tmp);
	printk("tubao >>>>>>>===============%s %d pinSetIdx=%d currSensorName=%s============\n",__func__,__LINE__, pinSetIdx, currSensorName);
	if (On) {

		ISP_MCLK1_EN(1);
//		if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC2355_RAW))){
//		if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName,SENSOR_DRVNAME_GC2235MIPI_RAW ))){
		if (pinSetIdx == 0 ){
			/* First Power Pin low and Reset Pin Low */
			printk("tubao ===============%s %d pinSetIdx=%d currSensorName=%s============\n",__func__,__LINE__, pinSetIdx, currSensorName);
			printk("==>Step1 %s pinSetIdx%d set CAMERA_CMPDN_PIN high\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
			//mdelay(1);

			printk("==>Step2 %s pinSetIdx%d set CAMERA_CMRST_PIN low\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			printk("==>Step5 %s pinSetIdx%d set digital power 1.8V\n", currSensorName, pinSetIdx);
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

			//mdelay(1);
			
			printk("==>Step4 %s pinSetIdx%d set IO power 1.8V\n", currSensorName, pinSetIdx);
			/* VCAM_A */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(20);

			printk("==>Step3 %s pinSetIdx%d set analog power 2.8V\n", currSensorName, pinSetIdx);
			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(10);

#if 1
			printk("==>Step6 %s pinSetIdx%d set flashlight power 2.8V\n", currSensorName, pinSetIdx);
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMAF, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
#endif
			mdelay(5);

		/*******************************************************************/	
			printk("==>Step7 %s pinSetIdx%d set CAMERA_CMPDN1_PIN high\n", currSensorName, 1-pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(1-pinSetIdx, CAMPDN, pinSet[1-pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);

			mdelay(1);
			printk("==>Step8 %s pinSetIdx%d set CAMERA_CMRST1_PIN high\n", currSensorName, 1-pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMRST])
					mtkcam_gpio_set(1-pinSetIdx, CAMRST, pinSet[1-pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
			
			
			mdelay(5);
		/*******************************************************************/	
			/* enable active sensor */
			printk("==>Step9 %s pinSetIdx%d set CAMERA_CMPDN_PIN high\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
			mdelay(30);
			
			printk("==>Step10 %s pinSetIdx%d set CAMERA_CMPDN_PIN high\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
					mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
		} 
//		else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC0312_YUV))) {
//		else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC0310_MIPI_YUV))) {
		else if (pinSetIdx == 1 ) {
			printk("tubao ===============%s %d pinSetIdx=%d currSensorName=%s============\n",__func__,__LINE__, pinSetIdx, currSensorName);
			/* First Power Pin low and Reset Pin Low */
			printk("==>Step1 %s pinSetIdx%d set CAMERA_CMPDN1_PIN high\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
			mdelay(1);

			printk("==>Step2 %s pinSetIdx%d set CAMERA_CMRST1_PIN low\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
			mdelay(1);

			/* VCAM_A */
			printk("==>Step3 %s pinSetIdx%d set analog power 2.8V\n", currSensorName, pinSetIdx);
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMA, VOL_2800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_IO */
			printk("==>Step4 %s pinSetIdx%d set IO power 1.8V\n", currSensorName, pinSetIdx);
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);
			
			printk("==>Step5 %s pinSetIdx%d set digital power 1.8V\n", currSensorName, pinSetIdx);
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMD, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAMD), power id = %d\n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(5);

			/* enable active sensor */
			/*************************************************************/
			printk("==>Step6 %s pinSetIdx%d set CAMERA_CMPDN_PIN high\n", currSensorName, 1-pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(1-pinSetIdx, CAMPDN, pinSet[1-pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]);
			mdelay(1);

			printk("==>Step7 %s pinSetIdx%d set CAMERA_CMRST_PIN high\n", currSensorName, 1-pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[1-pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(1-pinSetIdx, CAMRST, pinSet[1-pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
			mdelay(1);
			/************************************************************/
			printk("==>Step8 %s pinSetIdx%d set CAMERA_CMPDN1_PIN high\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
			mdelay(30);

			printk("==>Step9 %s pinSetIdx%d set CAMERA_CMRST1_PIN high\n", currSensorName, pinSetIdx);
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]);
		}
		else {
		
			/* VCAM_IO */
			if (TRUE != _hwPowerOnCnt(pinSetIdx, VCAMIO, VOL_1800, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable IO power (VCAM_IO), power id = %d\n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
		}
	}
	else { /* power OFF */	
		ISP_MCLK1_EN(0);
//		if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC2355_RAW))) {
		if (pinSetIdx == 0 ) {
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
			/* Set Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);
			
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
#if 1//add by tubao
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMAF, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAMAF), power id = %d\n", VCAMAF);
				goto _kdCISModulePowerOn_exit_;
			}
#endif

			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAMD), power id = %d \n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}

		}
//		else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(currSensorName, SENSOR_DRVNAME_GC0312_YUV))) {
		else if (pinSetIdx == 1 ) {
			/* Set Power Pin low and Reset Pin Low */
			/* Set Power Pin low and Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN])
				mtkcam_gpio_set(pinSetIdx, CAMPDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]);
			/* Set Reset Pin Low */
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
				mtkcam_gpio_set(pinSetIdx, CAMRST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]);

			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMD, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAMD), power id = %d \n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}

			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
	
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMA, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
			
		}
		else {
			/* VCAM_IO */
			if (TRUE != _hwPowerDownCnt(pinSetIdx, VCAMIO, mode_name)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d\n", VCAMIO);
				/* return -EIO; */
				goto _kdCISModulePowerOn_exit_;
			}
		}
	}

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;

}
EXPORT_SYMBOL(kdCISModulePowerOn);

