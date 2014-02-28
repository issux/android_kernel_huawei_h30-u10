
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

//ID0=1 ID1=0
const static unsigned char LCD_MODULE_ID = 0x01;
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
static struct LCM_setting_table byd_ips_init[] = {
    // Set page 0
    {0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
    {0xB3,1,{0x00}},//v-sync disable
    {0xB1,3,{0x68,0x00,0x00}},// for esd
    {0xBC,1,{0x00}}, //00 colume inversion;02 2dot inversion, 01 1dot inversion
    
    // Set page 1
    {0xF0,5,{0x55,0xaa,0x52,0x08,0x01}},
    {0xC0,1,{0xC0}},
    {0xC2,1,{0x20}},
    {0xB0,1,{0x05}},
    {0xB6,1,{0x44}},
    
    {0xB1,1,{0x05}},
    {0xB7,1,{0x34}},
    {0xB2,1,{0x02}},
    {0xB8,1,{0x13}},
    {0xB3,1,{0x0E}},
    {0xB9,1,{0x23}},
    {0xB4,1,{0x08}},
    {0xBA,1,{0x23}},
    {0xBC,3,{0x00,0x78,0x00}},
    {0xBD,3,{0x00,0x78,0x00}},
	//{0xBE,1,{0x3B}},//setting vcom
	//{0xBF,1,{0x3B}},

    //make the gamma index close to the gamma2.2
    //gamma 2.2
    // Gamma for R+
    {0xD1,16,{0x00,0x1C,0x00,0x31,0x00,0x51,0x00,0x6B,0x00,0x81,0x00,0x9E,0x00,0xC2,0x00,0xF7}},
    // Gamma for R+
    {0xD2,16,{0x01,0x1F,0x01,0x5F,0x01,0x90,0x01,0xDF,0x02,0x21,0x02,0x23,0x02,0x5D,0x02,0x9B}},
    // Gamma for R+
    {0xD3,16,{0x02,0xC3,0x02,0xF4,0x03,0x16,0x03,0x42,0x03,0x65,0x03,0xA0,0x03,0xB8,0x03,0xC7}},
    // Gamma for R+
    {0xD4,4,{0x03,0xED,0x03,0xFF}},
    
    // Gamma for G+
    {0xD5,16,{0x00,0x1C,0x00,0x31,0x00,0x51,0x00,0x6B,0x00,0x81,0x00,0x9E,0x00,0xC2,0x00,0xF7}},
    // Gamma for G+
    {0xD6,16,{0x01,0x1F,0x01,0x5F,0x01,0x90,0x01,0xDF,0x02,0x21,0x02,0x23,0x02,0x5D,0x02,0x9B}},
    // Gamma for G+
    {0xD7,16,{0x02,0xC3,0x02,0xF4,0x03,0x16,0x03,0x42,0x03,0x65,0x03,0xA0,0x03,0xB8,0x03,0xC7}},
    // Gamma for G+
    {0xD8,4,{0x03,0xED,0x03,0xFF}},
    
   // Gamma for B+
    {0xD9,16,{0x00,0x1C,0x00,0x31,0x00,0x51,0x00,0x6B,0x00,0x81,0x00,0x9E,0x00,0xC2,0x00,0xF7}},
    // Gamma for B+
    {0xDD,16,{0x01,0x1F,0x01,0x5F,0x01,0x90,0x01,0xDF,0x02,0x21,0x02,0x23,0x02,0x5D,0x02,0x9B}},
    // Gamma for B+
    {0xDE,16,{0x02,0xC3,0x02,0xF4,0x03,0x16,0x03,0x42,0x03,0x65,0x03,0xA0,0x03,0xB8,0x03,0xC7}},
    // Gamma for B+
    {0xDF,4,{0x03,0xED,0x03,0xFF}},
    
    // Gamma for R
    {0xE0,16,{0x00,0x1C,0x00,0x31,0x00,0x51,0x00,0x6B,0x00,0x81,0x00,0x9E,0x00,0xC2,0x00,0xF7}},
    // Gamma for R
    {0xE1,16,{0x01,0x1F,0x01,0x5F,0x01,0x90,0x01,0xDF,0x02,0x21,0x02,0x23,0x02,0x5D,0x02,0x9B}},
    // Gamma for R
    {0xE2,16,{0x02,0xC3,0x02,0xF4,0x03,0x16,0x03,0x42,0x03,0x65,0x03,0xA0,0x03,0xB8,0x03,0xC7}},
    // Gamma for R
    {0xE3,4,{0x03,0xED,0x03,0xFF}},
    
    // Gamma for G
    {0xE4,16,{0x00,0x1C,0x00,0x31,0x00,0x51,0x00,0x6B,0x00,0x81,0x00,0x9E,0x00,0xC2,0x00,0xF7}},
    // Gamma for G
    {0xE5,16,{0x01,0x1F,0x01,0x5F,0x01,0x90,0x01,0xDF,0x02,0x21,0x02,0x23,0x02,0x5D,0x02,0x9B}},
    // Gamma for G
    {0xE6,16,{0x02,0xC3,0x02,0xF4,0x03,0x16,0x03,0x42,0x03,0x65,0x03,0xA0,0x03,0xB8,0x03,0xC7}},
    // Gamma for G
    {0xE7,4,{0x03,0xED,0x03,0xFF}},

    // Gamma for B
    {0xE8,16,{0x00,0x1C,0x00,0x31,0x00,0x51,0x00,0x6B,0x00,0x81,0x00,0x9E,0x00,0xC2,0x00,0xF7}},
    // Gamma for B
    {0xE9,16,{0x01,0x1F,0x01,0x5F,0x01,0x90,0x01,0xDF,0x02,0x21,0x02,0x23,0x02,0x5D,0x02,0x9B}},
    // Gamma for B
    {0xEA,16,{0x02,0xC3,0x02,0xF4,0x03,0x16,0x03,0x42,0x03,0x65,0x03,0xA0,0x03,0xB8,0x03,0xC7}},
    // Gamma for B
    {0xEB,4,{0x03,0xED,0x03,0xFF}},

    {0xFF,4,{0xAA,0x55,0x25,0x01}},

    {0x6F,1,{0x0B}},
    {0xF4,4,{0x12,0x12,0x56,0x13}},
    //Disable cmd2
    {0xF0,5,{0x55,0xAA,0x52,0x00,0x00}}, //for esd
    {0xFF,4,{0xAA,0x55,0x25,0x00}},
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

        params->dsi.vertical_sync_active                = 11;
        params->dsi.vertical_backporch                  = 3;
        params->dsi.vertical_frontporch                 = 15;
        params->dsi.vertical_active_line                = FRAME_HEIGHT;

        params->dsi.horizontal_sync_active              = 11;
        params->dsi.horizontal_backporch                = 18;//58Hz
        params->dsi.horizontal_frontporch               = 30;
        params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
        //solve 2nd lcm driver using the mipi clk setting of 1st lcm  driver
        params->dsi.PLL_CLOCK =220;
		//disable the ssc.
		params->dsi.ssc_disable = 1;
}

/******************************************************************************
Function:       lcm_id_pin_handle
Description:    operate GPIO to prevent electric leakage
Input:          none
Output:         none
Return:         none
Others:         byd id0:1;id1:0,so pull up ID0 PIN & pull down ID1 PIN
******************************************************************************/
static void lcm_id_pin_handle(void)
{
    unsigned int ret = 0;
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_UP);
    if(0 != ret)
    {
        LCD_DEBUG("ID0 mt_set_gpio_pull_select->UP fail\n");
    }
    ret = mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DOWN);
    if(0 != ret)
    {
        LCD_DEBUG("ID1 mt_set_gpio_pull_select->Down fail\n");
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
    push_table(byd_ips_init, sizeof(byd_ips_init) / sizeof(struct LCM_setting_table), 1);
    //when sleep out, config output high ,enable backlight drv chip
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ONE);
#ifdef BUILD_LK
	printf("LCD nt35517_qhd_vdo_byd5.5 lcm_init\n");
#else
	printk("LCD nt35517_qhd_vdo_byd5.5 lcm_init\n");
#endif
}
static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("LCD nt35517_qhd_vdo_byd5.5 lcm_suspend\n");
#else
	printk("LCD nt35517_qhd_vdo_byd5.5 lcm_suspend\n");
#endif
    // when phone sleep , config output low, disable backlight drv chip
    lcm_util.set_gpio_out(GPIO_LCD_DRV_EN_PIN, GPIO_OUT_ZERO);
    push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("LCD nt35517_qhd_vdo_byd5.5 lcm_resume\n");
#else
	printk("LCD nt35517_qhd_vdo_byd5.5 lcm_resume\n");
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
        printk( "nt35517_byd lcm_esd_check buffer[0] = %d\n",buffer[0]);
        return TRUE;
    }
    else
    {
        if(buffer[3] != 0x02) //error data type is 0x02
        {
             return FALSE;
        }
        else
        {
			 //if lcm is normal buffer[4]=0x00,buffer[5]=0x80
             /*buffer[4]  bit 0 : SOT Error , bit 1: SOT sync Error
                 buffer[5] bit 0 : Ecc error single-bit , bit 1: Ecc error multi-bit*/
             if(((buffer[4] & 0x03 ) != 0) || ((buffer[5] & 0x03) != 0))
             {
                  printk( "nt35517_byd lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);
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
    printk( "nt35517_byd lcm_esd_recover\n");
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    msleep(20);  //lcm power on , reset output high , delay 30ms ,then output low
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ZERO);
    msleep(20);
    lcm_util.set_gpio_out(GPIO_DISP_LRSTB_PIN, GPIO_OUT_ONE);
    msleep(120); //when reset applied during sleep out mode , after reset high-low-high , delay 120ms , then download initial code
    push_table(byd_ips_init, sizeof(byd_ips_init) / sizeof(struct LCM_setting_table), 1);
    return TRUE;
}
#endif
LCM_DRIVER nt35517_qhd_vdo_byd55_lcm_drv =
{
    .name           = "nt35517_qhd_vdo_byd55",
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
