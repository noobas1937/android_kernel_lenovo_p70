#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
	.i2c_num = 1,
	//.polling_mode =1,
	.polling_mode_ps =0,/*0 -> interrupt 1 -> polling*/
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is nwot used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x90, 0x00, 0x00, 0x00},	/*STK3x1x*/
     //add by sw2drv wangyang 2014.09.03
	.als_level	= {2,  3, 5,  6, 20, 30, 45, 80, 125, 150, 300, 500, 700, 1000, 4000},								/* als_code count = 16 - 1*/ 
	.als_value	= {0, 40,180, 240,450, 700 ,1000, 1500, 2000, 6000, 10000, 10000, 10000, 10000, 10000, 10240},		 /* lux count = 16*/
   	.state_val = 0x0,		/* disable all */
	.psctrl_val = 0x31,		/* ps_persistance=1, ps_gain=64X, PS_IT=0.391ms */
	.alsctrl_val = 0x2B,	/* als_persistance=1, als_gain=16X, ALS_IT=200ms */
	.ledctrl_val = 0xFF,	/* 100mA IRDR, 64/64 LED duty */
	.wait_val = 0x7,		/* 50 ms */
    .ps_high_thd_val = 32, //32
    .ps_low_thd_val = 9,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

