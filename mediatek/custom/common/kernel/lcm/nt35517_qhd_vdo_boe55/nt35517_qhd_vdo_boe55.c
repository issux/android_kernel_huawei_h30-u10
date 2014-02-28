
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/disp_drv_platform.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
    #include <linux/delay.h>
    #include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  		(540)
#define FRAME_HEIGHT 		(960)

#define REGFLAG_DELAY       		0xFE
#define REGFLAG_END_OF_TABLE    	0xFD   // END OF REGISTERS MARKER 
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//make this adjust the triple LCD ID
//ID0=1 ID1=1
const static unsigned char LCD_MODULE_ID = 0x05;
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    			(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 				(lcm_util.udelay(n))
#define MDELAY(n) 				(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  printf(fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif
const static unsigned int BL_MIN_LEVEL = 20;
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[128];
};

static struct LCM_setting_table boe_ips_init[] = {
    // Set page 0
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
	{0xB0,5,{0x00,0x0C,0x40,0x3C,0x3C}},
	{0xB1,3,{0x68,0x00,0x00}},//for esd   
	{0xB3,1,{0x00}},//v-sync disable
	{0xB6,1,{0x08}},
	{0xB7,2,{0x00,0x00}},
	{0xBA,1,{0x01}},	
    {0xBC,1,{0x00}}, //00 colume inversion;02 2dot inversion, 01 1dot inversion
	//{0xBD,5,{0x01,0x41,0x10,0x37,0x01}}, //for esd
    {0xCC,3,{0x03,0x00,0x00}},     
    
    // Set page 1
    {0xF0,5,{0x55,0xaa,0x52,0x08,0x01}},
    {0xB0,3,{0x0A,0x0A,0x0A}},
    {0xB6,3,{0x43,0x43,0x43}},
    {0xB1,3,{0x0A,0x0A,0x0A}},
    {0xB7,3,{0x22,0x22,0x22}},
    {0xB2,3,{0x03,0x03,0x03}},
    {0xB8,3,{0x33,0x33,0x33}},
    {0xB3,3,{0x0A,0x0A,0x0A}},
    {0xB9,3,{0x24,0x24,0x24}},
    {0xB4,3,{0x0A,0x0A,0x0A}},
    {0xBA,3,{0x24,0x24,0x24}},
    {0xB5,3,{0x07,0x07,0x07}},
	//Update the data for VN1 LCD
	{0xBC,3,{0x00,0x70,0x00}},
	{0xBD,3,{0x00,0x70,0x00}},
	//{0xBE,1,{0x3B}},//setting vcom
	//{0xBF,1,{0x3B}},
	//Update the gamma data for VN1 LCD
    //BOE supply gamma 2.2 parameter
	{0xD1,16,{0x00,0x2D,0x00,0x3C,0x00,0x56,0x00,0x6C,0x00,0x7C,0x00,0x9B,0x00,0xBA,0x00,0xE9}},
	{0xD2,16,{0x01,0x0F,0x01,0x4A,0x01,0x79,0x01,0xC1,0x01,0xFF,0x02,0x00,0x02,0x37,0x02,0x72}},
	{0xD3,16,{0x02,0x99,0x02,0xCE,0x02,0xF6,0x03,0x2A,0x03,0x4C,0x03,0x79,0x03,0x97,0x03,0xBD}},
	{0xD4,4,{0x03,0xF5,0x03,0xFF}},

	{0xD5,16,{0x00,0x2D,0x00,0x3C,0x00,0x56,0x00,0x6C,0x00,0x7C,0x00,0x9B,0x00,0xBA,0x00,0xE9}},
	{0xD6,16,{0x01,0x0F,0x01,0x4A,0x01,0x79,0x01,0xC1,0x01,0xFF,0x02,0x00,0x02,0x37,0x02,0x72}},
	{0xD7,16,{0x02,0x99,0x02,0xCE,0x02,0xF6,0x03,0x2A,0x03,0x4C,0x03,0x79,0x03,0x97,0x03,0xBD}},
	{0xD8,4,{0x03,0xF5,0x03,0xFF}},

	{0xD9,16,{0x00,0x2D,0x00,0x3C,0x00,0x56,0x00,0x6C,0x00,0x7C,0x00,0x9B,0x00,0xBA,0x00,0xE9}},
	{0xDD,16,{0x01,0x0F,0x01,0x4A,0x01,0x79,0x01,0xC1,0x01,0xFF,0x02,0x00,0x02,0x37,0x02,0x72}},
	{0xDE,16,{0x02,0x99,0x02,0xCE,0x02,0xF6,0x03,0x2A,0x03,0x4C,0x03,0x79,0x03,0x97,0x03,0xBD}},
	{0xDF,4,{0x03,0xF5,0x03,0xFF}},

	{0xE0,16,{0x00,0x2D,0x00,0x3C,0x00,0x56,0x00,0x6C,0x00,0x7C,0x00,0x9B,0x00,0xBA,0x00,0xE9}},
	{0xE1,16,{0x01,0x0F,0x01,0x4A,0x01,0x79,0x01,0xC1,0x01,0xFF,0x02,0x00,0x02,0x37,0x02,0x72}},
	{0xE2,16,{0x02,0x99,0x02,0xCE,0x02,0xF6,0x03,0x2A,0x03,0x4C,0x03,0x79,0x03,0x97,0x03,0xBD}},
	{0xE3,4,{0x03,0xF5,0x03,0xFF}},

	{0xE4,16,{0x00,0x2D,0x00,0x3C,0x00,0x56,0x00,0x6C,0x00,0x7C,0x00,0x9B,0x00,0xBA,0x00,0xE9}},
	{0xE5,16,{0x01,0x0F,0x01,0x4A,0x01,0x79,0x01,0xC1,0x01,0xFF,0x02,0x00,0x02,0x37,0x02,0x72}},
	{0xE6,16,{0x02,0x99,0x02,0xCE,0x02,0xF6,0x03,0x2A,0x03,0x4C,0x03,0x79,0x03,0x97,0x03,0xBD}},
	{0xE7,4,{0x03,0xF5,0x03,0xFF}},

	{0xE8,16,{0x00,0x2D,0x00,0x3C,0x00,0x56,0x00,0x6C,0x00,0x7C,0x00,0x9B,0x00,0xBA,0x00,0xE9}},
	{0xE9,16,{0x01,0x0F,0x01,0x4A,0x01,0x79,0x01,0xC1,0x01,0xFF,0x02,0x00,0x02,0x37,0x02,0x72}},
	{0xEA,16,{0x02,0x99,0x02,0xCE,0x02,0xF6,0x03,0x2A,0x03,0x4C,0x03,0x79,0x03,0x97,0x03,0xBD}},
	{0xEB,4,{0x03,0xF5,0x03,0xFF}},
    //close HS write CMD1 to avoid ESD operate IC register
    {0xF0,5,{0x55,0xAA,0x52,0x00,0x00}}, //for esd
    {0x11,1,{0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0x29,1,{0x00}},
    {REGFLAG_DELAY, 40, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

//According suplier advice , increase sleep & dispaly delay
//sleep in disbale v-sync detect, sleep out enbale v-sync detect
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Set page 0
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xB3,1,{0x00}},//v-sync disable
    
    // Sleep Out
    {0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 0, {0x00}},
    {REGFLAG_DELAY, 40, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
    // Set page 0
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xB3,1,{0x80}},//v-sync enable
    {REGFLAG_DELAY, 200, {}},
    
    // Display off sequence
    {0x28, 0, {0x00}},
    {REGFLAG_DELAY, 40, {}},

    // Sleep Mode On
    {0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


/*Optimization LCD initialization time*/
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    mdelay(table[i].count);
                else
                    msleep(table[i].count);
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
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
        memset(params, 0, sizeof(LCM_PARAMS));
    
        params->type   = LCM_TYPE_DSI;

        params->width  = FRAME_WIDTH;
        params->height = FRAME_HEIGHT;

        params->dsi.mode   = BURST_VDO_MODE; //boe wangxudong suggest
 
        // DSI
        /* Command mode setting */
        params->dsi.LANE_NUM                = LCM_TWO_LANE;
        params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


        // Video mode setting       
        params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

        params->dsi.vertical_sync_active                = 5;
        params->dsi.vertical_backporch                  = 15;
        params->dsi.vertical_frontporch                 = 7;
        params->dsi.vertical_active_line                = FRAME_HEIGHT;

        params->dsi.horizontal_sync_active              = 8;
        params->dsi.horizontal_backporch                = 40;
        params->dsi.horizontal_frontporch               = 40;
        params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
        params->dsi.PLL_CLOCK =240;
		//disable the ssc.
		params->dsi.ssc_disable = 1;
}

/******************************************************************************
Function:       lcm_id_pin_handle
Description:    operate GPIO to prevent electric leakage
Input:          none
Output:         none
Return:         none
Others:         boe id0:1;id1:1,so pull up ID0 & ID1
******************************************************************************/
static void lcm_id_pin_handle(void)
{
    unsigned int ret = 0;
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_UP);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_pull_select->UP fail\n");
    }
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_UP);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_pull_select->UP fail\n");
    }
}
static void lcm_init(void)
{
    lcm_util.set_gpio_mode(GPIO_DISP_LRSTB_PIN, GPIO_MODE_00);
    lcm_util.set_gpio_dir(GPIO_DISP_LRSTB_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    mdelay(20);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    msleep(20);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    msleep(120);

    lcm_id_pin_handle();/*Handle GPIO_DISP_ID0_PIN and GPIO_DISP_ID1_PIN*/
    push_table(boe_ips_init, sizeof(boe_ips_init) / sizeof(struct LCM_setting_table), 1);
    //when sleep out, config output high ,enable backlight drv chip
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);
#ifdef BUILD_LK
	printf("LCD nt35517_qhd_vdo_boe5.5 lcm_init\n");
#else
	printk("LCD nt35517_qhd_vdo_boe5.5 lcm_init\n");
#endif
}
static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("LCD nt35517_qhd_vdo_boe5.5 lcm_suspend\n");
#else
	printk("LCD nt35517_qhd_vdo_boe5.5 lcm_suspend\n");
#endif
    // when phone sleep , config output low, disable backlight drv chip
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ZERO);
    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("LCD nt35517_qhd_vdo_boe5.5 lcm_resume\n");
#else
	printk("LCD nt35517_qhd_vdo_boe5.5 lcm_resume\n");
#endif
    push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
     //when sleep out, config output high ,enable backlight drv chip
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);
}

static unsigned int lcm_compare_id(void)
{
    unsigned char LCD_ID_value = 0;
    //make this adjust the triple LCD ID
    LCD_ID_value = which_lcd_module_triple();
    if(LCD_MODULE_ID == LCD_ID_value)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}
#ifndef BUILD_LK
/******************************************************************************
Function:       lcm_esd_check
  Description:    check LCD work status for ESD
  Input:           none
  Output:         none
  Return:         FALSE : NO ESD happened
                       TRUE: ESD happened
  Others:         
******************************************************************************/
static unsigned int lcm_esd_check(void)
{
    unsigned char buffer[8] = {0};
    unsigned int array[4];
    
    array[0] = 0x00013700;    
    dsi_set_cmdq(array, 1,1);
    read_reg_v2(0x0A, buffer,8);
    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/
    {
        printk( "nt35517_boe lcm_esd_check buffer[0] = %d\n",buffer[0]);
        return TRUE;
    }
    else
    {
        if(buffer[3] != 0x02) //error data type is 0x02
        {
             return FALSE;
        }
        else //if lcm is normal buffer[4]=0x00,buffer[5]=0x80
        {
             if((buffer[4] != 0) || (buffer[5] != 0x80))
             {
                  printk( "nt35517_boe lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);
                  return TRUE;
             }
             else
             {
                  return FALSE;
              }
        }
    }
}

static unsigned int lcm_esd_recover(void)
{
    /*LCD work status error ,so initialize*/
    printk( "nt35517_boe lcm_esd_recover\n");
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    msleep(20);  //lcm power on , reset output high , delay 30ms ,then output low
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    msleep(20);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    msleep(120); //when reset applied during sleep out mode , after reset high-low-high , delay 120ms , then download initial code
    push_table(boe_ips_init, sizeof(boe_ips_init) / sizeof(struct LCM_setting_table), 1);
    return TRUE;
}
#endif
LCM_DRIVER nt35517_qhd_vdo_boe55_lcm_drv =
{
    .name           = "nt35517_qhd_vdo_boe55",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#ifndef BUILD_LK
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,
#endif
};
