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


 int tps651321_write_bytes(unsigned char addr, unsigned char value)
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

#define LCM_ID1 0x83
#define LCM_ID2 0x94

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







//Updated By TIANMA 2014.08.06   TM050JDHP36-00

static void init_lcm_registers(void)
{
	 unsigned int data_array[16];


	data_array[0]=0x00043902;
	data_array[1]=0x9483ffb9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0]=0x000c3902;
	data_array[1]=0x204373ba; //data_array[1]=0xA88333ba;  \u6fa7\u70b2\u59de\u6939\u535e\u59e9\u9473\u85c9\u59cf 
    data_array[2]=0x0909B265;//0909b265
	data_array[3]=0x00001040;
	dsi_set_cmdq(data_array, 4, 1);
	
	MDELAY(1);
	//BAh,1st para=73,2nd para=43,7th para=09,8th para=40,9th para=10,10th para=00,11th para=00
	data_array[0]=0x00103902;
	data_array[1]=0x11116Cb1;
	data_array[2]=0xF1110437;//vgl=-5.55*2
	data_array[3]=0x23949F80;
	data_array[4]=0x18D2C080;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(5);
	
	data_array[0]=0x000C3902;
	data_array[1]=0x0E6400b2;
	data_array[2]=0x0823320D;
        data_array[3]=0x004D1C08;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);
	
		
	data_array[0]=0x000D3902;
	data_array[1]=0x03FF00b4;
	data_array[2]=0x03460346;
	data_array[3]=0x01680146;
	data_array[4]=0x00000068;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	
	data_array[0]=0x00043902;
	data_array[1]=0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00263902;
	data_array[1]=0x000700D3;
	data_array[2]=0x00100000;
	data_array[3]=0x00051032;
	data_array[4]=0x00103205;
	data_array[5]=0x10320000;
	data_array[6]=0x36000000;
	data_array[7]=0x37090903;   
	data_array[8]=0x00370000;
	data_array[9]=0x0A000000;
	data_array[10]=0x00000100;
	dsi_set_cmdq(data_array, 11, 1);
	MDELAY(1);
	

	
	data_array[0]=0x002D3902;
	data_array[1]=0x000302d5;
	data_array[2]=0x04070601;
	data_array[3]=0x22212005;
	data_array[4]=0x18181823;
	data_array[5]=0x18181818;
	data_array[6]=0x18181818;
	data_array[7]=0x18181818;   
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x24181818;
	data_array[11]=0x19181825;
	data_array[12]=0x00000019;
	dsi_set_cmdq(data_array, 13, 1);
	MDELAY(5);//recommond 10ms


	data_array[0]=0x002D3902;
	data_array[1]=0x070405D6;
	data_array[2]=0x03000106;
	data_array[3]=0x21222302;
	data_array[4]=0x18181820;
	data_array[5]=0x58181818;
	data_array[6]=0x18181858;
	data_array[7]=0x18181818;   
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x25181818;
	data_array[11]=0x18191924;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
	MDELAY(5);//recommond 10ms

	data_array[0]=0x002b3902;
	data_array[1]=0x0A0A02e0;
	data_array[2]=0x203F2829;//	data_array[2]=0x1A332924;
	data_array[3]=0x0F0E0C3F;//	data_array[3]=0x0D0B083C;
	data_array[4]=0x14120E17;//	data_array[4]=0x15120F17;
	data_array[5]=0x12081413;
	data_array[6]=0x0A021916;
	data_array[7]=0x3F28290A;   
	data_array[8]=0x0E0C3F20;//	data_array[8]=0x0B083C1A;
	data_array[9]=0x120E170F;//	data_array[9]=0x120F170D;
	data_array[10]=0x08141314;//	data_array[10]=0x08141315;
	data_array[11]=0x00191612;
	dsi_set_cmdq(data_array, 12, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
	data_array[1]=0x000009cc;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(5);//recommond 10ms

	data_array[0]=0x00053902;
	data_array[1]=0x40C000c7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(5);//recommond 5ms
	
	//data_array[0]=0x00033902;
    //data_array[1]=0x007F69b6;//VCOM
	//dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);

	data_array[0]=0x00033902;
        data_array[1]=0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
        data_array[1]=0x000007BC;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	data_array[0]=0x00023902;
        data_array[1]=0x0000FF51;
	dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);

	data_array[0]=0x00023902;
        data_array[1]=0x00002453;
	dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(1);

	data_array[0]=0x00023902;
        data_array[1]=0x00000055;
	dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(5);
 
    data_array[0]=0x00033902;//Enable ce
    data_array[1]=0x000152E4;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(5);//recommond 5ms

	data_array[0]=0x00023902;
	data_array[1]=0x00000035;
	dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(10);

	
	data_array[0]= 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

	data_array[0]= 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);   
 
	//data_array[0]=0x00023902;
    //data_array[1]=0x000062b6;//VCOM
	//dsi_set_cmdq(data_array, 2, 1);
	//MDELAY(10);	
}



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
	// {REGFLAG_DELAY, 10, {}},
	{REGFLAG_DELAY, 150, {}}, //add by wangyang 2014.08.15

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
	params->dsi.esd_check_enable = 1;// 1
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
	//add esd check function , set dsi.esd_check_enable = 1 ,esd check enable.
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.ssc_disable = 1;//add by wang yang for shepin 2014.11.5 
	
	params->dsi.vertical_sync_active     = 4;
	params->dsi.vertical_backporch       = 12;
	params->dsi.vertical_frontporch      = 16;
	params->dsi.vertical_active_line     = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active   = 55;
	params->dsi.horizontal_backporch     = 100;
	params->dsi.horizontal_frontporch    = 100;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;

	/*
	params->dsi.vertical_sync_active     = 4;
	params->dsi.vertical_backporch       = 12;
	params->dsi.vertical_frontporch      = 16;
	params->dsi.vertical_active_line     = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active   = 24;
	params->dsi.horizontal_backporch     = 100;
	params->dsi.horizontal_frontporch    = 50;
	params->dsi.horizontal_active_pixel  = FRAME_WIDTH;
	*/

	params->dsi.PLL_CLOCK = 229;//dsi clock customization: should config clock value directly
//24,100,50..232 =>60Hz
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id1 = 0xFF;
	unsigned int id2 = 0xFF;
	unsigned int id3 = 0xFF;
	unsigned char buffer[3];
	unsigned int data_array[16];

	lcm_debug("%s %d\n", __func__,__LINE__);
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(10);


	//array[0] = 0x00033700;// read id return three byte,manufacturer,version and id
	//dsi_set_cmdq(array, 1, 1);
	//read_reg_v2(0xDC, buffer, 3);

	//set EXTC
	data_array[0]=0x00043902;
	data_array[1]=0x9483ffb9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);

	//set MIPI
	data_array[0]=0x000c3902;
	data_array[1]=0x204373ba; //data_array[1]=0xA88333ba;  \u6fa7\u70b2\u59de\u6939\u535e\u59e9\u9473\u85c9\u59cf 
	data_array[2]=0x0909B265;
	data_array[3]=0x00001040;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);
	//pls set the same as forward BAh setting
	//set maximum return size
	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 1);

	//read id R04h
	data_array[0] = 0x00040600;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0x04, buffer, 3);

	id1 = buffer[0]; //we only need ID
	id2 = buffer[1];
	id3 = buffer[2];

#if defined(BUILD_LK)
	printf("%s,  id1 ---gyg---TM050= 0x%08x\n", __func__, id1);
	printf("%s,  id2 ---gyg---TM050= 0x%08x\n", __func__, id2);
#else
	printk("%s,  id1 ---gyg---TM050= 0x%08x\n", __func__, id1);
	printk("%s,  id2 ---gyg---TM050= 0x%08x\n", __func__, id2);
#endif
	return ((LCM_ID1== id1)&&(LCM_ID2== id2))?1:0;
		


}

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h> 
	#include <platform/mt_pmic.h>
	
#define TPS65132_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t TPS65132_i2c;

int TPS651321_write_byte(kal_uint8 addr, kal_uint8 value)
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
//	#define TPS65132_read_byte(add)  mt8193_i2c_read(add)
  

#endif

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0A;
//data=0x0A;VSP=5V,//data=0x0E;VSP=5.4V
	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
		MDELAY(10);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);//sophiarui
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
		MDELAY(10);
#ifdef BUILD_LK
	ret=TPS651321_write_byte(cmd,data);
    if(ret)    	
    dprintf(0, "[LK]TM050-----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]TM050----tps6132----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=tps651321_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]TM050-----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]TM050-----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	
	cmd=0x01;
	data=0x0A;
	//data=0x0A;VSN=-5V//data=0x0E;VSN=-5.4V
#ifdef BUILD_LK
	ret=TPS651321_write_byte(cmd,data);
    if(ret)    	
	dprintf(0, "[LK]TM050-----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]TM050----tps6132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps651321_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]TM050-----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]TM050-----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif

	
	lcm_debug("%s %d\n", __func__,__LINE__);
	SET_RESET_PIN(1);
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(20);

	init_lcm_registers();
	//push_table(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);


}


static void lcm_suspend(void)
{
	lcm_debug("%s %d\n", __func__,__LINE__);
// change by wangyang 2014.08.15 
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(40);	
	SET_RESET_PIN(0);	
	MDELAY(5);	
	SET_RESET_PIN(1);
	MDELAY(10);

	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
	MDELAY(10);

	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
	MDELAY(10);
	//SET_GPIO_OUT(GPIO_LCM_PWR_EN,0);//Disable LCM Power
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
	    char  *buffer1;
	    char  *buffer2;
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
	
		read_reg_v2(0x0D, buffer, 1);
		read_reg_v2(0x0E, buffer1, 1);
		read_reg_v2(0x0A, buffer2, 1);

		//printk(" esd buffer0 =%x\n", buffer[0]);

	#if 1
		if((buffer[0]==0) && (buffer1[0]==0) && buffer2[0] == 1)
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
	printk("lcm_esd_recover hx8394d\n");
#endif
	return TRUE;
}




LCM_DRIVER hx8394a_hd720_dsi_vdo_tianma_lcm_drv = 
{
    .name		= "hx8394a_hd720_dsi_vdo_tianma",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
	.update         = lcm_update,
#endif
	//.esd_check   = lcm_esd_check,
   // .esd_recover   = lcm_esd_recover,
};

