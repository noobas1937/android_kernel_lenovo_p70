#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

#include <cust_gpio_usage.h>
#if 0 //by wangyang 2014.08.14
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <cust_i2c.h>


/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  0//I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0//sophia
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct tps65132_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "tps65132_iic_probe\n");
	printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	tps65132_i2c_client  = client;		
	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
  printk( "tps65132_remove\n");
  tps65132_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


 int tps651321_write_bytess(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("tps65132 write data fail !!\n");	
	return ret ;
}
EXPORT_SYMBOL_GPL(tps651321_write_bytes);



/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

   printk( "tps65132_iic_init\n");
   i2c_register_board_info(0, &tps65132_board_info, 1);
   printk( "tps65132_iic_init2\n");
   i2c_add_driver(&tps65132_iic_driver);
   printk( "tps65132_iic_init success\n");	
   return 0;
}

static void __exit tps65132_iic_exit(void)
{
  printk( "tps65132_iic_exit\n");
  i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif
#endif 
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define GPIO_65132_ENP GPIO_LCD_BIAS_ENP_PIN //sophiarui
#define GPIO_65132_ENN GPIO_LCD_BIAS_ENN_PIN //sophiarui


#define REGFLAG_DELAY             							0XFEFF
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_OTM1284A 0x1284

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))


#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


#define _LCM_DEBUG_

#ifdef BUILD_LK
#define printk printf
#endif

#ifdef _LCM_DEBUG_
#define lcm_debug(fmt, args...) printk(fmt, ##args)
#else
#define lcm_debug(fmt, args...) do { } while (0)
#endif

#ifdef _LCM_INFO_
#define lcm_info(fmt, args...) printk(fmt, ##args)
#else
#define lcm_info(fmt, args...) do { } while (0)
#endif
#define lcm_err(fmt, args...) printk(fmt, ##args)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       

static struct LCM_setting_table {
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


	//must use 0x39 for init setting for all register.

                              {0x00, 1, {0x00}},
                                {0xff, 3, {0x12,0x84,0x01}},        //EXTC=1
                                
                                {0x00, 1, {0x80}},
                                {0xff, 2, {0x12,0x84}},            //Orise mode enable
                               
                                {0x00, 1, {0x91}},
                                {0xb0, 1, {0x92}},            
                
                //-------------------- panel setting --------------------//
                                {0x00, 1, {0x80}},
                                {0xc0, 9, {0x00,0x64,0x00,0x0f,0x11,0x00,0x64,0x0f,0x11}},
                
                                {0x00, 1, {0x90}},
                                {0xc0, 6, {0x00,0x5c,0x00,0x01,0x00,0x04}},
                                
                                {0x00, 1, {0xa4}},
                                {0xc0, 1, {0x00}},
                                
                                {0x00, 1, {0xb3}},
                                {0xc0, 2, {0x00,0x55}},
                                
                                {0x00, 1, {0x81}},
                                {0xc1, 1,{0x55}},
 
                //-------------------- BOE Power IC --------------------//
                                {0x00, 1, {0x90}},
                                {0xf5, 4, {0x02,0x11,0x02,0x15}},
                                
                                {0x00, 1, {0x90}},           //2xVPNL 1.5*=00,2*=50,3*=a0
                                {0xc5, 1, {0x50}},
                                                
                                {0x00, 1,{0x94}},                                                //Frequency
                                {0xc5, 1, 0x66},
                
                //-------------------- VGL01/02 disable --------------------//
                                {0x00, 1, {0xb2}},          //VGL01
                                {0xf5, 2, {0x00,0x00}},
                
                                {0x00, 1, {0xb4}},          //VGL01_S
                                {0xf5, 2, {0x00,0x00}},
                                                
                                {0x00, 1, {0xb6}},          //VGL02
                                {0xf5, 2, {0x00,0x00}},
                                                
                                {0x00, 1, {0xb8}},          //VGL02_S
                                {0xf5, 2, {0x00,0x00}},
                                                
                                {0x00, 1, {0x94}},          //VCL pump dis
                                {0xf5, 2, {0x00,0x00}},
                                                
                                {0x00, 1, {0xd2}},          //VCL reg. en
                                {0xf5, 2, {0x06,0x15}},
                                                
                                {0x00, 1, {0xb4}},          //VGL01/2 Pull low setting
                                {0xc5, 1, {0xcc}},          //d[7] cglo1 d[6] cglo2 => 0:pull vss, 1: pull vgl
 
                                {0x00, 1, {0xa0}},          //dcdc setting
                                {0xc4, 14, {0x05,0x10,0x06,0x02,0x05,0x15,0x10,0x05,0x10,0x07,0x02,0x05,0x15,0x10}},



                                {0x00, 1, {0xb0}},                                              //clamp voltage setting
                                {0xc4, 2, {0x00,0x00}},

                                {0x00,1,{0x91}},                                                //VGH=12V, VGL=-12V, pump ratio:VGH=6x, VGL=-5x
                                {0xc5,2,{0x19,0x52}},
                
                                {0x00,1,{0x00}},                                                //GVDD=4.87V, NGVDD=-4.87V
                                {0xd8,2,{0xbc,0xbc}},
                             
                                {0x00,1,{0xb3}},                                                //VDD_18V=1.8V, LVDSVDD=-1.6V
                                {0xc5,1,{0x84}},
                                
                
                                {0x00,1,{0xbb}},                                                //LVD voltage level setting
                                {0xc5,1,{0x8a}},
                                
                                {0x00,1,{0x82}},                                                //flash-orise add
                                {0xc4,1,{0x0a}},

				{0x00,1,{0xb2}},                                                
                                {0xc5,1,{0x40}},

				{0x00,1,{0x80}},                                                //for al890 esd
                                {0xc4,1,{0x30}},
                                
                                {0x00,1,{0x81}},                                                //for al890 esd
                                {0xc4,2,{0x82,0x0a}},
                                
                                {0x00,1,{0xc6}},                                                //debounce
                                {0xb0,1,{0x03}},
 
                                {0x00,1,{0xc2}},                                 //precharge disable
                                {0xf5,1,{0x40}},
                                
                                {0x00,1,{0xc3}},                                 //sample hold gvdd
                                {0xf5,1,{0x85}},
 
                //-------------------- Gamma Tuning--------------------//
/*     
				{0x00,1,{0x00}},
				{0xE1,20,{0x05,0x23,0x32,0x3F,0x4F,0x5B,0x5D,0x88,0x78,0x92,0x70,0x5C,0x70,0x52,0x50,0x40,0x39,0x2A,0x18,0x00}},
								                                
				{0x00,1,{0x00}},
				{0xE2,20,{0x05,0x23,0x32,0x3F,0x4F,0x5B,0x5D,0x88,0x78,0x92,0x70,0x5C,0x70,0x52,0x50,0x40,0x39,0x2A,0x18,0x00}},

                
                                {0x00,1,{0x00}},                                                //VCOMDC=-0.912
                                {0xd9,1,{0x45}},
                                */
                  //-------------------- Gamma Tuning--------------------//
				
				 {0x00,1,{0x00}},
                                        {0xE1,20,{0x05,0x10,0x17,0x22,0x2F,0x3B,0x3C,0x67,0x5A,0x74,0x8C,0x75,0x84,0x5B,0x55,0x45,0x32,0x21,0x12,0x09}},
                                                                                                                                  
                                 {0x00,1,{0x00}},
                                        {0xE2,20,{0x05,0x0F,0x17,0x22,0x2F,0x3B,0x3C,0x67,0x5A,0x74,0x8C,0x75,0x83,0x5B,0x55,0x45,0x32,0x21,0x12,0x09}},
               
				//{0x00,1,{0x00}},                                                //VCOMDC=-0.912
				//{0xd9,1,{0x45}},
                                //{0x00,1,{0x91}},                                                //VCOMDC=-0.912
                                //{0xb0,1,{0x92}},
                //-------------------- panel timing state control --------------------//
                                {0x00,1,{0x80}},                                                //panel timing state control
                                {0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0x90}},                                                // panel timing state control
                                {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xa0}},                                                //panel timing state control
                                {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xb0}},                                                //panel timing state control
                                
                                {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xc0}},                                 //panel timing state control
                                {0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xd0}},                                                //panel timing state control
                                {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00}},
                
                                {0x00,1,{0xe0}},                                                //panel timing state control
                                {0xcb,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05}},
                
                                {0x00,1,{0xf0}},                                 //panel timing state control
                                {0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},
 
                //-------------------- panel pad mapping control --------------------//
                                {0x00,1,{0x80}},                                                //panel pad mapping control
                                {0xcc,15,{0x0a,0x0c,0x0e,0x10,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0x90}},                                                //panel pad mapping control
                                {0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x2e,0x2d,0x09,0x0b,0x0d,0x0f,0x01,0x03,0x00,0x00}},
                
                                {0x00,1,{0xa0}},                                                //panel pad mapping control
                                {0xcc,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2e,0x2d}},
                
                                {0x00,1,{0xb0}},                                                //panel pad mapping control
                                {0xcc,15,{0x0F,0x0D,0x0B,0x09,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xc0}},                                 //panel pad mapping control
                                {0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x2d,0x2e,0x10,0x0E,0x0C,0x0A,0x04,0x02,0x00,0x00}},
                
                                {0x00,1,{0xd0}},                                                //panel pad mapping control
                                {0xcc,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2d,0x2e}},
                
                //-------------------- panel timing setting --------------------//
                
				{0x00,1,{0x80}},                                                //panel VST setting
				{0xce,12,{0x8d,0x03,0x00,0x8c,0x03,0x00,0x8b,0x03,0x00,0x8a,0x03,0x00}},

				{0x00,1,{0x90}},                                                //panel VEND setting
				{0xce,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

				{0x00,1,{0xa0}},                                                //panel CLKA1/2 setting
				{0xce,14,{0x38,0x0b,0x85,0x00,0x8d,0x03,0x00,0x38,0x0a,0x85,0x01,0x8d,0x03,0x00}},

				{0x00,1,{0xb0}},                                                //panel CLKA3/4 setting
				{0xce,14,{0x38,0x09,0x85,0x02,0x8d,0x03,0x00,0x38,0x08,0x85,0x03,0x8d,0x03,0x00}},

				{0x00,1,{0xc0}},                                 //panel CLKb1/2 setting
				{0xce,14,{0x38,0x07,0x85,0x04,0x8d,0x03,0x00,0x38,0x06,0x85,0x05,0x8d,0x03,0x00}},

				{0x00,1,{0xd0}},                                                //panel CLKb3/4 setting
				{0xce,14,{0x38,0x05,0x85,0x06,0x8d,0x03,0x00,0x38,0x04,0x85,0x07,0x8d,0x03,0x00}},
 
                               
                
                                {0x00,1,{0x80}},                                                //panel CLKc1/2 setting
                                {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0x90}},                                                //panel CLKc3/4 setting
                                {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xa0}},                                                //panel CLKd1/2 setting
                                {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
                                {0x00,1,{0xb0}},                                                //panel CLKd3/4 setting
                                {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
                
				{0x00,1,{0xc0}},                                 //panel ECLK setting
				{0xcf,11,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x01,0x00,0x00,0x00}}, //gate pre. ena.
                
                                {0x00,1,{0xb5}},                                                //TCON_GOA_OUT Setting
                                {0xc5,6,{0x33,0xf1,0xff,0x33,0xf1,0xff}}, //normal output with VGH/VGL
 
                                {0x00,1,{0x00}},                                                //Orise mode disable
                                {0xff,3,{0xff,0xff,0xff}}, 
				
				{0x35,1,{0x00}},   //set te
                              
                                {REGFLAG_END_OF_TABLE, 0x00, {}} 


	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_DELAY, 150, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	lcm_debug("%s %d\n", __func__,__LINE__);
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
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	lcm_debug("%s %d\n", __func__,__LINE__);
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	  	lcm_debug("%s %d\n", __func__,__LINE__);
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE;
#endif
	

		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
		params->dsi.lcm_esd_check_table[0].count = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;//4;//2;
		params->dsi.vertical_backporch					= 14;//16;//14;
		params->dsi.vertical_frontporch					= 16;//16;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;//6;//2;
		params->dsi.horizontal_backporch				= 42;//44;//44;//42;
		params->dsi.horizontal_frontporch				= 44;//44;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		params->dsi.PLL_CLOCK = 190;//dsi clock customization: should config clock value directly

}

static unsigned int lcm_compare_id(void)
{
	int data_array[4]; 
        char buffer[5]; 
        char id_high=0; 
        char id_low=0; 
        int id=0; 

	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	MDELAY(10);



	SET_RESET_PIN(1);
	MDELAY(2);
    SET_RESET_PIN(0);
	MDELAY(1);
    SET_RESET_PIN(1);
	MDELAY(10);

        data_array[0] = 0x00053700; 
        dsi_set_cmdq(data_array, 1, 1); 
        read_reg_v2(0xa1, buffer, 5); 

        id_high = buffer[2]; 
        id_low = buffer[3]; 
        id = (id_high<<8) | id_low; 

    #ifdef BUILD_LK 
                printf("%s, LK OTM1284 debug: OTM1284 id = 0x%08x\n", __func__, id); 
    #else 
                printk("%s, kernel OTM1284 horse debug: OTM1284 id = 0x%08x\n", __func__, id); 
    #endif 
	 return (LCM_ID_OTM1284A == id)?1:0;


}

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>
	
#define TPS65132_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t TPS65132_i2c;

int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    TPS65132_i2c.id = 0;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}


#else
  
//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);
//	#define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data)
//#define TPS65132_read_byte(add)  mt8193_i2c_read(add)
  
extern int tps651321_write_bytes(kal_uint8 addr, kal_uint8 value); //by wangyang 2014.08.14

#endif

static void lcm_init_power(void)
{
		unsigned char cmd = 0x0;
		unsigned char data = 0xFF;
		int ret=0;
		cmd=0x00;
		data=0x0E;
	
		mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);//sophiarui
		mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
		MDELAY(10);
		mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);//sophiarui
		mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
		MDELAY(10);
#ifdef BUILD_LK
		ret=TPS65132_write_byte(cmd,data);
		if(ret) 	
		dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write error----\n",cmd);		
		else
		dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write success----\n",cmd);			
#else
		ret=tps651321_write_bytes(cmd,data);
		if(ret<0)
		printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
		else
		printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
		
		cmd=0x01;
		data=0x0E;
#ifdef BUILD_LK
		ret=TPS65132_write_byte(cmd,data);
		if(ret) 	
			dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write error----\n",cmd);		
		else
			dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write success----\n",cmd);	 
#else
		ret=tps651321_write_bytes(cmd,data);
		if(ret<0)
		printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
		else
		printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif

}
static void lcm_resume_power(void)
{
			unsigned char cmd = 0x0;
			unsigned char data = 0xFF;
			int ret=0;
			cmd=0x00;
			data=0x0E;
		
			mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);//sophiarui
			mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
			MDELAY(10);
			mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);//sophiarui
			mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
			MDELAY(10);
#ifdef BUILD_LK
			ret=TPS65132_write_byte(cmd,data);
			if(ret) 	
			dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write error----\n",cmd);		
			else
			dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write success----\n",cmd);			
#else
			ret=tps651321_write_bytes(cmd,data);
			if(ret<0)
			printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
			else
			printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
			
			cmd=0x01;
			data=0x0E;
#ifdef BUILD_LK
			ret=TPS65132_write_byte(cmd,data);
			if(ret) 	
				dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write error----\n",cmd);		
			else
				dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write success----\n",cmd);	 
#else
			ret=tps651321_write_bytes(cmd,data);
			if(ret<0)
			printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
			else
			printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif

}
static void lcm_suspend_power(void)
{
	
	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
	MDELAY(10);
}


static void lcm_init(void)
{
#if 1	//by wangyang 2014.08.13
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0E;

	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	MDELAY(10);
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
    if(ret)    	
    dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=tps651321_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	
	cmd=0x01;
	data=0x0E;
#ifdef BUILD_LK
	ret=TPS65132_write_byte(cmd,data);
    if(ret)    	
	    dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
		dprintf(0, "[LK]otm1284a----tps6132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps651321_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]otm1284a----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
#endif 
	
	lcm_debug("%s %d\n", __func__,__LINE__);
	SET_RESET_PIN(1);
	MDELAY(2);
    SET_RESET_PIN(0);
	MDELAY(1);
    SET_RESET_PIN(1);
	MDELAY(10);

	push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);


	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	
	


}


static void lcm_suspend(void)
{
	lcm_debug("%s %d\n", __func__,__LINE__);
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);	
	MDELAY(10);	
	SET_RESET_PIN(1);
	MDELAY(10);

	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);

    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
	MDELAY(10);
    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
	//SET_GPIO_OUT(GPIO_LCM_PWR_EN,0);//Disable LCM Power
	MDELAY(10);
		
		
}



static void lcm_resume(void)
{
	lcm_debug("%s %d\n", __func__,__LINE__);
//	SET_GPIO_OUT(GPIO_LCM_PWR_EN,1);  //Enable LCM Power
	lcm_init();
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	lcm_debug("%s %d\n", __func__,__LINE__);
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
	data_array[3]= 0x00000000;
	data_array[4]= 0x00053902;
	data_array[5]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[6]= (y1_LSB);
	data_array[7]= 0x00000000;
	data_array[8]= 0x002c3909;

	dsi_set_cmdq(&data_array, 9, 0);

}



static unsigned int lcm_esd_check(void)
{
		unsigned int ret=FALSE;
  #ifndef BUILD_LK
		char  *buffer;
		int   array[4];
	
#if 1
		if(lcm_esd_test)
		{
			lcm_esd_test = FALSE;
			return TRUE;
		}
#endif
		array[0] = 0x00013700;
		dsi_set_cmdq(array, 1, 1);
	
		read_reg_v2(0x0A, buffer, 1);
		printk(" esd buffer0 =%x\n", buffer[0]);

#if 1
		if(buffer[0]==0x9c)
		{
			ret=FALSE;
		}
		else
		{			 
			ret=TRUE;
		}
#endif
 #endif
	 return ret;
	
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();

#ifndef BUILD_LK
	printk("lcm_esd_recover otm1284a\n");
#endif
	return TRUE;
}




LCM_DRIVER otm1284a_hd720_dsi_vdo_lcm_drv = 
{
    .name			= "otm1284a_dsi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
//by wangyang  2014.08.14
	 .init_power	= lcm_init_power,
     .resume_power  = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
//by wangyang end
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
	//.esd_check   = lcm_esd_check,
    //.esd_recover   = lcm_esd_recover,
};

