#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>

#include <mach/mt_gpio.h>		// For gpio control

/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10       11    12       13      14       15    16
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5  1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif



/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_timeOutTimeMs=0;

static BOOL g_is_torch_mode = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif





static struct work_struct workTimeOut;



/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);


#define LEDS_TORCH_MODE 		1
#define LEDS_FLASH_MODE 		0
#define LEDS_CUSTOM_MODE_THRES 	20

#ifndef GPIO_CAMERA_FLASH_MODE_PIN 
#define GPIO_CAMERA_FLASH_MODE_PIN GPIO144
#define GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO GPIO_MODE_00
#endif

#ifndef GPIO_CAMERA_FLASH_EN_PIN 
#define GPIO_CAMERA_FLASH_EN_PIN GPIO143
#define GPIO_CAMERA_FLASH_EN_PIN_M_GPIO GPIO_MODE_00
#endif

#define FLASH_ON_PULSE 15

int FL_enable(void)
{
	int i =0;
	PK_DBG("FL_enable.g_is_torch_mode=%d\n",g_is_torch_mode);	
	if(g_is_torch_mode)
	{
		mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, 0);
		mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 1);
		//g_is_torch_mode=0;
	}
	else
	{
		mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 0);
		mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, 1);
		for(i =0 ; i< FLASH_ON_PULSE;i++)
		{
			udelay(100);
			mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,0);
			udelay(100);
			mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,1);
		}
	}

    return 0;
}

int FL_disable(void)
{
	PK_DBG("FL_disable");
	mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN, 0);	
	mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 0);	

    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("FL_dim_duty %d, g_is_torch_mode %d, timeout %d\n", duty, g_is_torch_mode, g_timeOutTimeMs);	
	if(duty == 0)	{
		//mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 1);
		g_is_torch_mode = 1;		
	}
	else{
		//mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, 0);	
		g_is_torch_mode = 0;		
	}

	if(g_timeOutTimeMs < 1001)
	{
		PK_ERR("FL_dim_duty %d > thres %d, timeout %d", duty, g_timeOutTimeMs);	
		//mt_set_gpio_out(GPIO_CAMERA_FLASH_MODE_PIN, LEDS_TORCH_MODE);
		g_is_torch_mode = 1;	
	}
	PK_DBG("FL_dim_duty %d, g_is_torch_mode %d, timeout %d\n", duty, g_is_torch_mode, g_timeOutTimeMs);	
	
    return 0;
}


int FL_init(void)
{
	PK_DBG("FL_init");

	mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_DIR_OUT);  
	mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN, GPIO_CAMERA_FLASH_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN, GPIO_DIR_OUT);  
    /*Init. to disable*/
	FL_disable();
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_is_torch_mode = 1;
    return 0;
}
	
int FL_uninit(void)
{
	PK_DBG("FL_uninit");
	FL_disable();
	g_is_torch_mode = 0;
    return 0;
}



/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{

  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	int iFlashType = (int)FLASHLIGHT_NONE;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_enable();
    		}
    		else
    		{
    			FL_disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
        case FLASHLIGHTIOC_G_FLASHTYPE:
            iFlashType = FLASHLIGHT_LED_CONSTANT;
            if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
            {
                PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
                return -EFAULT;
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


