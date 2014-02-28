/*
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
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
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "ltr559.h"
#include <mtk_kpd.h>



#define POWER_NONE_MACRO MT65XX_POWER_NONE


/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define LTR559_DEV_NAME   "LTR_559ALS"

/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)   
//(Lite-On Nelson edit)>
#define 		ALS 		0
#define 		PS 			1
//(Lite-On Nelson edit)<
/******************************************************************************
 * extern functions
*******************************************************************************/
/*extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
	extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
	extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
	extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
										 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
										 kal_bool auto_umask);*/

extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

/*----------------------------------------------------------------------------*/

static struct i2c_client *ltr559_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id ltr559_i2c_id[] = {{LTR559_DEV_NAME,0},{}};
/*the adapter id & i2c address will be available in customization*/
static struct i2c_board_info __initdata i2c_ltr559={ I2C_BOARD_INFO("LTR_559ALS", 0x23)};

//static unsigned short ltr559_force[] = {0x00, 0x46, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const ltr559_forces[] = { ltr559_force, NULL };
//static struct i2c_client_address_data ltr559_addr_data = { .forces = ltr559_forces,};
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int ltr559_i2c_remove(struct i2c_client *client);
static int ltr559_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int ltr559_i2c_resume(struct i2c_client *client);
static int ltr559_ps_enable(int gainrange);

static int ps_value;
static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

//huawei add
static int min_proximity_value = 2000;
static int pwin_value = 90;
static int pwave_value = 100;
//huawei end

//(Lite-On Nelson edit)>
uint8_t eqn_prev = 0;
uint8_t ratio_old = 0;
uint16_t ps_init_kept_data[8];
uint16_t ps_ct_avg;
uint8_t ps_grabData_stage = 0;
uint32_t ftn_init;
uint32_t ftn_final;
uint32_t ntf_final;
uint8_t lux_val_prev = 0;
//(Lite-On Nelson edit)<
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int ltr559_als_read(int gainrange);
static int ltr559_ps_read(void);
//(Lite-On Nelson edit)>
static uint16_t lux_formula(uint16_t ch0_adc, uint16_t ch1_adc, uint8_t eqtn);
static uint16_t ratioHysterisis (uint16_t ch0_adc, uint16_t ch1_adc);
static uint16_t read_adc_value(uint8_t als_ps_mode);
//(Lite-On Nelson edit)<


/*----------------------------------------------------------------------------*/


typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct ltr559_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/

struct ltr559_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
    struct mutex lock;
	/*i2c address group*/
    struct ltr559_i2c_addr  addr;

     /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;
    atomic_t    als_suspend;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL];  //f00184246 mtk bug
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;
/*add snsensor auto function*/
/*----------------------------------------------------------------------------*/
extern int hwmsen_alsps_add(struct sensor_init_info* obj);
extern struct alsps_hw *get_ltr_cust_alsps_hw(void);
static int  ltr_local_init(void);
static int  ltr_remove(void);
static int  ltr_init_flag =-1; // 0<==>OK -1 <==> fail

static struct sensor_init_info ltr_init_info = {
        .name = "ltr559",
        .init = ltr_local_init,
        .uninit = ltr_remove,
};
/*----------------------------------------------------------------------------*/
static struct PS_CALI_DATA_STRUCT ps_cali={960,959,1};
static int intr_flag_value = 0;


static struct ltr559_priv *ltr559_obj = NULL;
static struct platform_driver ltr559_alsps_driver;

/*----------------------------------------------------------------------------*/
static struct i2c_driver ltr559_i2c_driver = {	
	.probe      = ltr559_i2c_probe,
	.remove     = ltr559_i2c_remove,
	.detect     = ltr559_i2c_detect,
	.suspend    = ltr559_i2c_suspend,
	.resume     = ltr559_i2c_resume,
	.id_table   = ltr559_i2c_id,
	//.address_data = &ltr559_addr_data,
	.driver = {
		//.owner          = THIS_MODULE,
		.name           = LTR559_DEV_NAME,
	},
};


/* 
 * #########
 * ## I2C ##
 * #########
 */

// I2C Read
static int ltr559_i2c_read_reg(u8 regnum)
{
      u8 reg_value;

	
	
	reg_value = i2c_smbus_read_byte_data(ltr559_obj->client,regnum);
	if(reg_value <0)	{
	   
	   APS_ERR("i2c_smbus_read_byte_data error reg_value= %d\n",reg_value);
	   return reg_value;
	}
	
	return reg_value;
}
// I2C Write
static int ltr559_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = regnum;   
	databuf[1] = value;
	res = i2c_master_send(ltr559_obj->client, databuf, 0x2);

	if (res < 0)
		{
			APS_ERR("wirte reg send res = %d\n",res);
		   	return res;
		}
		
	else
		return 0;
}


//(Lite-On Nelson edit)>
static uint16_t lux_formula(uint16_t ch0_adc, uint16_t ch1_adc, uint8_t eqtn)
{
	uint32_t luxval = 0;
	uint32_t luxval_i = 0;
	uint32_t luxval_f = 0;
	uint16_t ch0_coeff_i = 0;
	uint16_t ch1_coeff_i = 0;
	uint16_t ch0_coeff_f = 0;
	uint16_t ch1_coeff_f = 0;
	int8_t ret; 
	uint8_t gain = 1, als_int_fac, regAddr;
	uint8_t buffer[2];
	uint16_t win_fac = 0;
	int8_t fac = 1;

#define WINFAC1		100;
#define WINFAC2		80;
#define WINFAC3		44;

	//regAddr = LTR559_ALS_CONTR;
	regAddr = LTR559_ALS_PS_STATUS;
	buffer[0] = ltr559_i2c_read_reg(regAddr);
	gain = (buffer[0] & 0x70);
	gain >>= 4;

	if (gain == 0) {			//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}
	
	//if ((buffer[0] & 0x1C) == 0x00) {				//gain 1
	//	gain = 1;
	//} else if ((buffer[0] & 0x1C) == 0x04) {		//gain 2
	//	gain = 2;
	//} else if ((buffer[0] & 0x1C) == 0x08) {		//gain 4
	//	gain = 4;
	//} else if ((buffer[0]& 0x1C) == 0x0C) {		//gain 8
	//	gain = 8;
	//} else if ((buffer[0] & 0x1C) == 0x18) {		//gain 48
	//	gain = 48;
	//} else if ((buffer[0] & 0x1C) == 0x1C) {		//gain 96
	//	gain = 96;
	//}

	regAddr = LTR559_ALS_MEAS_RATE;
	buffer[0] = ltr559_i2c_read_reg(regAddr);

	als_int_fac = buffer[0] & 0x38;
	als_int_fac >>= 3;

	if (als_int_fac == 0) {
		als_int_fac = 10;
	} else if (als_int_fac == 1) {
		als_int_fac = 5;
	} else if (als_int_fac == 2) {
		als_int_fac = 20;
	} else if (als_int_fac == 3) {
		als_int_fac = 40;
	} else if (als_int_fac == 4) {
		als_int_fac = 15;
	} else if (als_int_fac == 5) {
		als_int_fac = 25;
	} else if (als_int_fac == 6) {
		als_int_fac = 30;
	} else if (als_int_fac == 7) {
		als_int_fac = 35;
	}

	if (eqtn == 1) {
		ch0_coeff_i = 1;
		ch1_coeff_i = 1;
		ch0_coeff_f = 7743;
		ch1_coeff_f = 1059;
		fac = 1;
		win_fac = WINFAC1;
		luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));
		//luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 2) {
		ch0_coeff_i = 4;
		ch1_coeff_i = 1;
		ch0_coeff_f = 2785;
		ch1_coeff_f = 696;
		win_fac = WINFAC2;
		if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f)) {
			fac = 1;
			luxval_f = (((ch0_adc * ch0_coeff_f) - (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		} else {
			fac = -1;
			luxval_f = (((ch1_adc * ch1_coeff_f) - (ch0_adc * ch0_coeff_f)) / 100) * win_fac;
		}
		luxval_i = ((ch0_adc * ch0_coeff_i) - (ch1_adc * ch1_coeff_i)) * win_fac;
		//luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));
		//luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 3) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 5926;
		ch1_coeff_f = 1300;
		fac = 1;
		win_fac = WINFAC3;
		luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));
		//luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 4) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 0;
		ch1_coeff_f = 0;
		fac = 1;
		luxval_i = 0;
		luxval_f = 0;
		//luxval = 0;
	}

	luxval = (luxval_i  + ((fac) * luxval_f) / 100) / (gain * als_int_fac);

	return luxval;
}


static uint16_t ratioHysterisis (uint16_t ch0_adc, uint16_t ch1_adc)
{
#define	RATIO_HYSVAL	10
	int ratio;
	uint8_t buffer[2], eqn_now, regAddr;
	int8_t ret;
	uint16_t ch0_calc;
	uint32_t luxval = 0;
	int abs_ratio_now_old;

	regAddr = LTR559_ALS_CONTR;
	buffer[0] = ltr559_i2c_read_reg(regAddr);

	ch0_calc = ch0_adc;
	if ((buffer[0] & 0x20) == 0x20) {
		ch0_calc = ch0_adc - ch1_adc;
	}

	if ((ch1_adc + ch0_calc) == 0) {
		ratio = 100;
	} else {
		ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);
	}

	if (ratio < 45) {
		eqn_now = 1;
	} else if ((ratio >= 45) && (ratio < 68)) {
		eqn_now = 2;
	} else if ((ratio >= 68) && (ratio < 99)) {
		eqn_now = 3;
	} else if (ratio >= 99) {
		eqn_now = 4;
	}

	if (eqn_prev == 0) {
		luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
		ratio_old = ratio;
		eqn_prev = eqn_now;
	} else {
		if (eqn_now == eqn_prev) {
			luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
			ratio_old = ratio;
			eqn_prev = eqn_now;
		} else {
			abs_ratio_now_old = ratio - ratio_old;
			if (abs_ratio_now_old < 0) {
				abs_ratio_now_old *= (-1);
			}
			if (abs_ratio_now_old > RATIO_HYSVAL) {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
				ratio_old = ratio;
				eqn_prev = eqn_now;
			} else {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_prev);
			}
		}
	}

	return luxval;
}


static uint16_t read_adc_value(uint8_t als_ps_mode)
{
	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD  	5000
#define AGC_HYS					15

	switch (als_ps_mode) {
		case 0 :
			/* ALS */
			buffer[0] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0);
			buffer[1] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1);
			buffer[2] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0);
			buffer[3] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1);			
			
			break;

		case 1 :
			/* PS */
			buffer[0] = ltr559_i2c_read_reg(LTR559_PS_DATA_0);
			buffer[1] = ltr559_i2c_read_reg(LTR559_PS_DATA_1);
			
			break;
	}

	switch (als_ps_mode) {
		case 0 :
			/* ALS Ch0 */
		 	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
			APS_DBG("alsval_ch0_lo = %d,alsval_ch0_hi=%d,alsval_ch0=%d\n",buffer[2],buffer[3],ch0_val);
			//input_report_abs(ltr559->als_input_dev, ABS_MISC, ch0_val);
			//input_sync(ltr559->als_input_dev);

			/* ALS Ch1 */
		 	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
			APS_DBG("alsval_ch1_lo = %d,alsval_ch1_hi=%d,alsval_ch1=%d\n",buffer[0],buffer[1],ch1_val);
			//input_report_abs(ltr559->als_input_dev, ABS_MISC, ch1_val);
			//input_sync(ltr559->als_input_dev);

			//buffer[0] = ltr559_i2c_read_reg(LTR559_ALS_CONTR);
			buffer[0] = ltr559_i2c_read_reg(LTR559_ALS_PS_STATUS);

			value_temp = buffer[0];
			temp = buffer[0];
			//gain = (value_temp & 0x1C);
			//gain >>= 2;
			gain = (value_temp & 0x70);
			gain >>= 4;

			if (gain == 0) {			//gain 1
				gain = 1;
			} else if (gain == 1) {		//gain 2
				gain = 2;
			} else if (gain == 2) {		//gain 4
				gain = 4;
			} else if (gain == 3) {		//gain 8
				gain = 8;
			} else if (gain == 6) {		//gain 48
				gain = 48;
			} else if (gain == 7) {		//gain 96
				gain = 96;
			}

			//value_temp &= 0xE3;

			if ((ch0_val == 0) && (ch1_val > 50 )) {
				value = lux_val_prev;
			} else {
				if (gain == 1) {
					if ((ch0_val + ch1_val) < ((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
						value = ratioHysterisis(ch0_val, ch1_val);
						//value_temp |= ALS_GAIN_8x;
						value_temp = MODE_ALS_ON_Range4;
						gain_chg_req = 1;
					} else {
						value = ratioHysterisis(ch0_val, ch1_val);
					}
				} else if (gain == 8) {
					if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
						value = ratioHysterisis(ch0_val, ch1_val);
						//value_temp |= ALS_GAIN_1x;
						value_temp = MODE_ALS_ON_Range1;
						gain_chg_req = 1;
					} else {
						value = ratioHysterisis(ch0_val, ch1_val);
					}
				} else {
					value = ratioHysterisis(ch0_val, ch1_val);
				}

				if (gain_chg_req) {
					ret = ltr559_i2c_write_reg(LTR559_ALS_CONTR, value_temp);
					if(ret<0)
				 		APS_LOG("ltr559 als change gain failed ...ERROR\n");
				 	else
				        	APS_LOG("ltr559 als change gain ...OK\n");
				}
				/* ALS Lux Conversion */
				//value = lux_formula(ch0_val, ch1_val);
				
			}
			if ((value > 50000) || (((ch0_val + ch1_val) > 50000) && (temp & 0x80))) {
				value = 50000;
			}
			lux_val_prev = value;
			break;

		case 1 :	
			/* PS */
			ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
			APS_DBG("ps_rawdata_psval_lo = %d\n", buffer[0]);
			APS_DBG("ps_rawdata_psval_hi = %d\n", buffer[1]);
			APS_DBG("ps_rawdata = %d\n", ps_val);

			value = ps_val;

			break;		
			
	}

	return value;
}
//(Lite-On Nelson edit)<
//MauriceLee

static int8_t als_ch0ch1raw_calc_readback (uint16_t *retVal1, uint16_t *retVal2, uint16_t *retVal3)
{
	int8_t ret = 0;
	uint8_t buffer[11];
	uint16_t value1, value2, value3;

	//buffer[0] = LTR559_ALS_DATA_CH1_0;
	//ret = I2C_Read(buffer, 4);
	buffer[0] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0);
	buffer[1] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1);
	buffer[2] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0);
	buffer[3] = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1);	
	//if (ret < 0) {
		//dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		//return ret;
	//}

	value1 = ((int)buffer[2]) + ((int)buffer[3] << 8); // CH0
	value2 = ((int)buffer[0]) + ((int)buffer[1] << 8); // CH1

	value3 = read_adc_value(ALS);
	
	//printk("als_ch0ch1raw_calc_readback,0x%04X,0x%04X,0x%04X",value1,value2,rdback_val3);
	*retVal1 = value1;
	*retVal2 = value2;
	*retVal3 = value3;

	return ret;
}

static ssize_t alsch0ch1rawcalc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val1 = 0, rdback_val2 = 0, rdback_val3 = 0;
	//struct ltr559_data *ltr559 = sensor_info;

	ret = als_ch0ch1raw_calc_readback(&rdback_val1, &rdback_val2, &rdback_val3);
	if (ret < 0) {
		//dev_err(&ltr559->i2c_client->dev, "%s: ALS CH0 CH1 Calc reading readback Fail...\n", __func__);
		//return (-1);
	}
	printk("alsch0ch1rawcalc_show,0x%04X,0x%04X,0x%04X",rdback_val1,rdback_val2,rdback_val3);
	return snprintf(buf, PAGE_SIZE, "0x%04X,0x%04X,0x%04X\n", rdback_val1,rdback_val2,rdback_val3);  

	//return ret;
		
}

static DRIVER_ATTR(alsch0ch1rawcalc, 0666, alsch0ch1rawcalc_show, NULL);

//Maurice end
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	u8 dat = 0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
//(Lite-On Nelson edit)	res = ltr559_als_read(als_gainrange);
	res = read_adc_value(ALS); //(Lite-On Nelson edit)
    return snprintf(buf, PAGE_SIZE, "0x%04X\n", res);    
	
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_ps_read();
    return snprintf(buf, PAGE_SIZE, "0x%04X\n", res);     
}
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(ltr559_obj->hw)
	{
	
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr559_obj->hw->i2c_num, ltr559_obj->hw->power_id, ltr559_obj->hw->power_vol);
		
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}


	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr559_obj->als_suspend), atomic_read(&ltr559_obj->ps_suspend));

	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_status(struct device_driver *ddri, char *buf, size_t count)
{
	int status1,ret;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &status1))
	{ 
	    ret=ltr559_ps_enable(ps_gainrange);
		APS_DBG("iret= %d, ps_gainrange = %d\n", ret, ps_gainrange);
	}
	else
	{
		APS_DBG("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}


/*----------------------------------------------------------------------------*/
static ssize_t ltr559_show_reg(struct device_driver *ddri, char *buf, size_t count)
{
	int i,len=0;
	int reg[]={0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,
		0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x97,0x98,0x99,0x9a,0x9e};
	for(i=0;i<27;i++)
		{
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n", reg[i],ltr559_i2c_read_reg(reg[i]));	

	    }
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t ltr559_store_reg(struct device_driver *ddri, char *buf, size_t count)
{
	int ret,value;
	u8 reg;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%x %x ", &reg,&value))
	{ 
		APS_DBG("before write reg: %x, reg_value = %x  write value=%x\n", reg,ltr559_i2c_read_reg(reg),value);
	    ret=ltr559_i2c_write_reg(reg,value);
		APS_DBG("after write reg: %x, reg_value = %x\n", reg,ltr559_i2c_read_reg(reg));
	}
	else
	{
		APS_DBG("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, ltr559_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, ltr559_show_ps,    NULL);
//static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, ltr559_show_config,ltr559_store_config);
//static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, ltr559_show_alslv, ltr559_store_alslv);
//static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, ltr559_show_alsval,ltr559_store_alsval);
//static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO,ltr559_show_trace, ltr559_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, ltr559_show_status,  ltr559_store_status);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, ltr559_show_reg,   ltr559_store_reg);
//static DRIVER_ATTR(i2c,     S_IWUSR | S_IRUGO, ltr559_show_i2c,   ltr559_store_i2c);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *ltr559_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
   // &driver_attr_trace,        /*trace log*/
   // &driver_attr_config,
   // &driver_attr_alslv,
   // &driver_attr_alsval,
    &driver_attr_status,
   //&driver_attr_i2c,
    &driver_attr_reg,
    &driver_attr_alsch0ch1rawcalc,
};
/*----------------------------------------------------------------------------*/
static int ltr559_create_attr(struct driver_attribute *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, ltr559_attr_list[idx]))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr559_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int ltr559_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, ltr559_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/

/* 
 * ###############
 * ## PS CONFIG ##
 * ###############

 */

static int ltr559_ps_set_thres()
{
	APS_FUN();

	int res;
	u8 databuf[2];
	
		struct i2c_client *client = ltr559_obj->client;
		struct ltr559_priv *obj = ltr559_obj;
		APS_DBG("ps_cali.valid: %d\n", ps_cali.valid);
	if(1 == ps_cali.valid)
	{
		databuf[0] = LTR559_PS_THRES_LOW_0; 
		databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_LOW_1; 
		databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_0;	
		databuf[1] = (u8)(ps_cali.close & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_1;	
		databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = LTR559_PS_THRES_LOW_0; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_LOW_1; 
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low )>> 8) & 0x00FF);
		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_0;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
		databuf[0] = LTR559_PS_THRES_UP_1;	
		databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) >> 8) & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	
	}

	res = 0;
	return res;
	
	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;

}


static int ltr559_ps_enable(int gainrange)
{
	struct i2c_client *client = ltr559_obj->client;
	struct ltr559_priv *obj = ltr559_obj;
	u8 databuf[2];	
	int res;

	int error;
	int setgain;
    APS_LOG("ltr559_ps_enable() ...start!\n");

	switch (gainrange) {
		case PS_RANGE16:
			setgain = MODE_PS_ON_Gain16;
			break;

		case PS_RANGE32:
			setgain = MODE_PS_ON_Gain32;
			break;

		case PS_RANGE64:
			setgain = MODE_PS_ON_Gain64;
			break;


		default:
			setgain = MODE_PS_ON_Gain16;
			break;
	}

	APS_LOG("LTR559_PS setgain = %d!\n",setgain);
	
	
	error = ltr559_i2c_write_reg(LTR559_PS_CONTR, setgain); 
	printk("@@@@@@ltr559_ps_enable()@@@@@@@\n");
	if(error<0)
	{
	    	printk("ltr559_ps_enable() error1\n");
	    	return error;
	}
	
	mdelay(WAKEUP_DELAY);
    
	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
   error = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 8); //Liteon Maurice modify
	if(error<0)
    {
        APS_LOG("ltr559_ps_enable() error2\n");
	    return error;
	} 
	/*error = ltr559_i2c_write_reg(LTR559_PS_LED, 0x63); 
	if(error<0)
    {
        APS_LOG("ltr559_ps_enable() error3...\n");
	    return error;
	}*/


	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{		

			ltr559_ps_set_thres();
			
			databuf[0] = LTR559_INTERRUPT;	
			databuf[1] = 0x01;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return ltr559_ERR_I2C;
			}
	
			databuf[0] = LTR559_INTERRUPT_PERSIST;
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return ltr559_ERR_I2C;
			}
			mt_eint_unmask(CUST_EINT_ALS_NUM);
	
		}
	
 	APS_LOG("ltr559_ps_enable ...OK!\n");


 	
	return error;

	EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;
}

// Put PS into Standby mode
static int ltr559_ps_disable(void)
{
	int error;
	struct ltr559_priv *obj = ltr559_obj;
		
	error = ltr559_i2c_write_reg(LTR559_PS_CONTR, MODE_PS_StdBy); 
	if(error<0)
 	    APS_LOG("ltr559_ps_disable ...ERROR\n");
 	else
        APS_LOG("ltr559_ps_disable ...OK\n");

	if(0 == obj->hw->polling_mode_ps)
	{
	    cancel_work_sync(&obj->eint_work);
		mt_eint_mask(CUST_EINT_ALS_NUM);
	}
	
	return error;
}


static int ltr559_ps_read(void)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr559_i2c_read_reg(LTR559_PS_DATA_0);
	APS_DBG("ps_rawdata_psval_lo = %d\n", psval_lo);
	if (psval_lo < 0){
	    
	    APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}
	psval_hi = ltr559_i2c_read_reg(LTR559_PS_DATA_1);
    APS_DBG("ps_rawdata_psval_hi = %d\n", psval_hi);

	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	
	psdata = ((psval_hi & 7)* 256) + psval_lo;
    //psdata = ((psval_hi&0x7)<<8) + psval_lo;
    APS_DBG("ps_rawdata = %d\n", psdata);
    
	out:
	final_prox_val = psdata;
	
	return psdata;
}

/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */

static int ltr559_als_enable(int gainrange)
{
	int error;
	APS_LOG("gainrange = %d\n",gainrange);
	switch (gainrange)
	{
		case ALS_RANGE_64K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range1);
			break;

		case ALS_RANGE_32K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range2);
			break;

		case ALS_RANGE_16K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range3);
			break;
			
		case ALS_RANGE_8K:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range4);
			break;
			
		case ALS_RANGE_1300:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range5);
			break;

		case ALS_RANGE_600:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range6);
			break;
			
		default:
			error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_ON_Range1);			
			APS_ERR("proxmy sensor gainrange %d!\n", gainrange);
			break;
	}

	mdelay(WAKEUP_DELAY);

	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
 	if(error<0)
 	    APS_LOG("ltr559_als_enable ...ERROR\n");
 	else
        APS_LOG("ltr559_als_enable ...OK\n");
        
	return error;
}


// Put ALS into Standby mode
static int ltr559_als_disable(void)
{
	int error;
	error = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_StdBy); 
	if(error<0)
 	    APS_LOG("ltr559_als_disable ...ERROR\n");
 	else
        APS_LOG("ltr559_als_disable ...OK\n");
	return error;
}

static int ltr559_als_read(int gainrange)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int  luxdata_int;
	int ratio;

	alsval_ch0_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
	APS_DBG("alsval_ch0_lo = %d,alsval_ch0_hi=%d,alsval_ch0=%d\n",alsval_ch0_lo,alsval_ch0_hi,alsval_ch0);
	alsval_ch1_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	APS_DBG("alsval_ch1_lo = %d,alsval_ch1_hi=%d,alsval_ch1=%d\n",alsval_ch1_lo,alsval_ch1_hi,alsval_ch1);

    if((alsval_ch1==0)||(alsval_ch0==0))
    {
        luxdata_int = 0;
        goto err;
    }
	ratio = (alsval_ch1*100) /(alsval_ch0+alsval_ch1);
	APS_DBG("ratio = %d  gainrange = %d\n",ratio,gainrange);
	if (ratio < 45){
		luxdata_int = (((17743 * alsval_ch0)+(11059 * alsval_ch1))/gainrange)/10000;
	}
	else if ((ratio < 64) && (ratio >= 45)){
		luxdata_int = (((42785 * alsval_ch0)-(19548 * alsval_ch1))/gainrange)/10000;
	}
	else if ((ratio < 85) && (ratio >= 64)) {
		luxdata_int = (((5926 * alsval_ch0)+(1185 * alsval_ch1))/gainrange)/10000;
	}
	else {
		luxdata_int = 0;
		}
	
	APS_DBG("als_value_lux = %d\n", luxdata_int);
	return luxdata_int;

	
err:
	final_lux_val = luxdata_int;
	APS_DBG("err als_value_lux = 0x%x\n", luxdata_int);
	return luxdata_int;
}



/*----------------------------------------------------------------------------*/
int ltr559_get_addr(struct alsps_hw *hw, struct ltr559_i2c_addr *addr)
{
	/***
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	***/
	return 0;
}


/*-----------------------------------------------------------------------------*/
void ltr559_eint_func(void)
{
	APS_FUN();

	struct ltr559_priv *obj = ltr559_obj;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work);
}



/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int ltr559_setup_eint(struct i2c_client *client)
{
	APS_FUN();
	struct ltr559_priv *obj = (struct ltr559_priv *)i2c_get_clientdata(client);        

	ltr559_obj = obj;
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	//mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	//mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	//mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	//mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, ltr559_eint_func, 0);
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, ltr559_eint_func, 0);
	//mt65xx_eint_unmask(CUST_EINT_ALS_NUM); 
	mt_eint_mask(CUST_EINT_ALS_NUM);
    return 0;
}


/*----------------------------------------------------------------------------*/
static void ltr559_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LTR559")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "LTR559")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int ltr559_check_and_clear_intr(struct i2c_client *client) 
{
//***
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];	
	u8 temp;
		//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/	
		//	  return 0;
	
		buffer[0] = LTR559_ALS_PS_STATUS;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		temp = buffer[0];
		res = 1;
		intp = 0;
		intl = 0;
		if(0 != (buffer[0] & 0x02))
		{
			res = 0;
			intp = 1;
		}
		if(0 != (buffer[0] & 0x08))
		{
			res = 0;
			intl = 1;		
		}
	
		if(0 == res)
		{
			if((1 == intp) && (0 == intl))
			{
				buffer[1] = buffer[0] & 0xfD;
				
			}
			else if((0 == intp) && (1 == intl))
			{
				buffer[1] = buffer[0] & 0xf7;
			}
			else
			{
				buffer[1] = buffer[0] & 0xf5;
			}
			buffer[0] = LTR559_ALS_PS_STATUS	;
			res = i2c_master_send(client, buffer, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			else
			{
				res = 0;
			}
		}
	
		return res;
	
	EXIT_ERR:
		APS_ERR("ltr559_check_and_clear_intr fail\n");
		return 1;

}
/*----------------------------------------------------------------------------*/


static int ltr559_check_intr(struct i2c_client *client) 
{
	APS_FUN();

	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_intr fail\n");
	return 1;
}

static int ltr559_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];

	APS_FUN();
	
	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	APS_DBG("buffer[0] = %d \n",buffer[0]);
	buffer[1] = buffer[0] & 0x01;
	buffer[0] = LTR559_ALS_PS_STATUS	;

	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_and_clear_intr fail\n");
	return 1;
}




static int ltr559_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	

	struct i2c_client *client = ltr559_obj->client;

	struct ltr559_priv *obj = ltr559_obj;   
	
	mdelay(PON_DELAY);

	// Enable PS to Gain4 at startup
	init_ps_gain = PS_RANGE16;
	ps_gainrange = init_ps_gain;

	res = ltr559_ps_enable(init_ps_gain);
	if (res < 0)
		goto EXIT_ERR;


	// Enable ALS to Full Range at startup
	init_als_gain = ALS_RANGE_8K; //(Lite-On Nelson edit)
	als_gainrange = init_als_gain;

	res = ltr559_als_enable(init_als_gain);
	if (res < 0)
		goto EXIT_ERR;


	/*for interrup work mode support */
	if(0 == obj->hw->polling_mode_ps)
	{	
		APS_LOG("eint enable");
		ltr559_ps_set_thres();
		
		databuf[0] = LTR559_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

		databuf[0] = LTR559_INTERRUPT_PERSIST;
		databuf[1] = 0x00;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

	}

	if((res = ltr559_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	if((res = ltr559_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}

	res = 0;

	EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}
/*----------------------------------------------------------------------------*/


static int ltr559_get_als_value(struct ltr559_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	APS_DBG("als  = %d\n",als); 
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int ltr559_get_ps_value(struct ltr559_priv *obj, u16 ps)
{
	int val,  mask = atomic_read(&obj->ps_mask);
	int invalid = 0;

	static int val_temp = 1;
	if((ps > atomic_read(&obj->ps_thd_val_high)))
	{
		val = 0;  /*close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
			//else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
	else if((ps < atomic_read(&obj->ps_thd_val_low)))
	{
		val = 1;  /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
		val = val_temp;	
			
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*for interrup work mode support */

int ltr559_huawei_ps_read(struct ltr559_priv* obj, int rawdata )
{
    int ret;
    int pdata;
    int pthreshold_h = 0, pthreshold_l;
    u8 databuf[2];
    u8 buffer[2];
    int res;
    unsigned int is_als_enable = 1;
    //printk("pdata:%d min_proximity_value:%d\n", pdata, min_proximity_value);
    
    pdata = rawdata ;
    
    if (((pdata + pwave_value) < min_proximity_value)&& (pdata>0))
    
    {
       
        min_proximity_value = pdata + pwave_value;
        ps_cali.close = min_proximity_value + pwin_value;
        if(ps_cali.close >= 2047)
        {
            ps_cali.close = 2047;
            min_proximity_value = 2047-pwin_value;
        }
        ps_cali.far_away = min_proximity_value;

	  /*set far away*/ 
        databuf[0] = LTR559_PS_THRES_LOW_0;	
	  databuf[1] = (u8)(min_proximity_value & 0x00FF);
	  res = i2c_master_send(obj->client, databuf, 0x2);
	  if(res <= 0)
	  {
		return;
	  }
	  databuf[0] = LTR559_PS_THRES_LOW_1;	
	  databuf[1] = (u8)((min_proximity_value & 0xFF00) >> 8);
	  res = i2c_master_send(obj->client, databuf,  0x2);
        if(res <= 0)
	 {
		return;
	  }

	  /*set far away*/
        databuf[0] = LTR559_PS_THRES_UP_0;	
	  databuf[1] = (u8)((ps_cali.close) & 0x00FF);
	  res = i2c_master_send(obj->client, databuf, 0x2);
	  if(res <= 0)
	  {
  		return;
	   }
          databuf[0] = LTR559_PS_THRES_UP_1; 
	   databuf[1] = (u8)((ps_cali.close& 0xFF00) >> 8);
	   res = i2c_master_send(obj->client, databuf, 0x2);
	   if(res <= 0)
	   {
		return;
	    }

          printk("%s:min_proximity_value=%d\n", __func__, min_proximity_value);
        
          return;
        
    }
    /*read low and high threshold*/
    buffer[0] = ltr559_i2c_read_reg(LTR559_PS_THRES_LOW_0);
    buffer[1] = ltr559_i2c_read_reg(LTR559_PS_THRES_LOW_1);
    pthreshold_l=buffer[0]+(buffer[1]<<8);

    buffer[0] = ltr559_i2c_read_reg(LTR559_PS_THRES_UP_0);
    buffer[1] = ltr559_i2c_read_reg(LTR559_PS_THRES_UP_1);
    pthreshold_h=buffer[0]+(buffer[1]<<8);
    printk("%s:pdata=%d pthreshold_h=%d pthreshold_l=%d\n", __func__, pdata, pthreshold_h, pthreshold_l);

    /* if more than the value of  proximity high threshold we set*/
    if (pdata >= pthreshold_h)
    {
        printk("set PILTL:0\n");
         databuf[0] = LTR559_PS_THRES_LOW_0;	
	  databuf[1] = (u8)(min_proximity_value & 0x00FF);
	  res = i2c_master_send(obj->client, databuf, 0x2);
	  if(res <= 0)
	  {
		printk(":set ltr_LTR559_PS_THRES_LOW_0 register is error(%d)!", ret);
		return;
	  }
	  databuf[0] = LTR559_PS_THRES_LOW_1;	
	  databuf[1] = (u8)((min_proximity_value & 0xFF00) >> 8);
	  res = i2c_master_send(obj->client, databuf, 0x2);
        if(res <= 0)
	 {
		printk(":set ltr_LTR559_PS_THRES_LOW_1 register is error(%d)!", ret);
		return;
	  }

	  databuf[0] = LTR559_PS_THRES_UP_0;	
	  databuf[1] = (u8)(2047& 0x00FF);
	  res = i2c_master_send(obj->client, databuf, 0x2);
	  if(res <= 0)
	  {
		printk(":set ltr_LTR559_PS_THRES_UP_0 register is error(%d)!", ret);
		return;
	   }
          databuf[0] = LTR559_PS_THRES_UP_1; 
	   databuf[1] = (u8)((2047& 0xFF00) >> 8);
	   res = i2c_master_send(obj->client, databuf, 0x2);
	   if(res <= 0)
	   {
		printk(":set ltr_LTR559_PS_THRES_UP_1 register is error(%d)!", ret);
		return;
	    }
        
        ps_cali.far_away = min_proximity_value;
        ps_cali.close = 2047;
        
        ps_value = 0;
    }
    /* if less than the value of  proximity low threshold we set*/
    /* the condition of pdata==pthreshold_l is valid */
    else if (pdata <= pthreshold_l)
    {
        printk("set PILTL:1\n");
         databuf[0] = LTR559_PS_THRES_LOW_0;	
	  databuf[1] = (u8)(min_proximity_value & 0x00FF);
	  res = i2c_master_send(obj->client, databuf, 0x2);
	  if(res <= 0)
	  {
		printk(":set ltr_LTR559_PS_THRES_LOW_0 register is error(%d)!", ret);
		return;
	  }
	  databuf[0] = LTR559_PS_THRES_LOW_1;	
	  databuf[1] = (u8)((min_proximity_value & 0xFF00) >> 8);
	  res = i2c_master_send(obj->client, databuf, 0x2);
        if(res <= 0)
	 {
		printk(":set ltr_LTR559_PS_THRES_LOW_1 register is error(%d)!", ret);
		return;
	  }

	  /*set far away*/
         databuf[0] = LTR559_PS_THRES_UP_0;	
	  databuf[1] = (u8)((min_proximity_value + pwin_value) & 0x00FF);
	  res = i2c_master_send(obj->client, databuf, 0x2);
	  if(res <= 0)
	  {
		printk(":set ltr_LTR559_PS_THRES_UP_0 register is error(%d)!", ret);
		return;
	   }
          databuf[0] = LTR559_PS_THRES_UP_1; 
	   databuf[1] = (u8)((min_proximity_value + pwin_value) >> 8);
	   res = i2c_master_send(obj->client, databuf, 0x2);
	   if(res <= 0)
	   {
		printk(":set ltr_LTR559_PS_THRES_UP_1register is error(%d)!", ret);
		return;
	    }
        ps_cali.far_away =min_proximity_value;
        
        ps_cali.close = min_proximity_value + pwin_value;
        
        
        ps_value = 1;
    }
   
    printk("proximity send:%d\n", ps_value);
    return;
   
}
static void ltr559_eint_work(struct work_struct *work)
{
	struct ltr559_priv *obj = (struct ltr559_priv *)container_of(work, struct ltr559_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
//	u8 buffer[1];
//	u8 reg_value[1];
       int value;
	u8 databuf[2];
	int res = 0;
	APS_FUN();
	err = ltr559_check_intr(obj->client);
	if(err < 0)
	{
		APS_ERR("ltr559_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		obj->ps = ltr559_ps_read();
    	if(obj->ps < 0)
    	{
    		err = -1;
    		return;
    	}
				
		APS_DBG("ltr559_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
	     ltr559_huawei_ps_read(obj, obj->ps );
	     sensor_data.values[0] = ps_value;
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;		
/*singal interrupt function add*/
 		#if 0
		APS_DBG("intr_flag_value=%d\n",intr_flag_value);
		if(intr_flag_value){
				APS_DBG(" interrupt value ps will < ps_threshold_low");

				databuf[0] = LTR559_PS_THRES_LOW_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_LOW_1;	
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_0;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_1; 
				databuf[1] = (u8)((0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				APS_DBG(" interrupt value ps will > ps_threshold_high");
				databuf[0] = LTR559_PS_THRES_LOW_0;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_LOW_1;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_0;	
				databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = LTR559_PS_THRES_UP_1; 
				databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	ltr559_clear_intr(obj->client);
	mt_eint_unmask(CUST_EINT_ALS_NUM);      
}



/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int ltr559_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr559_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int ltr559_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/


static int ltr559_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr559_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	APS_DBG("cmd= %d\n", cmd); 
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    err = ltr559_ps_enable(ps_gainrange);
				if(err < 0)
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
			    err = ltr559_ps_disable();
				if(err < 0)
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_DBG("ALSPS_GET_PS_DATA\n"); 
		    obj->ps = ltr559_ps_read();
			if(obj->ps < 0)
			{
				goto err_out;
			}
			
			dat = ltr559_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			obj->ps = ltr559_ps_read();
			if(obj->ps < 0)
			{
				goto err_out;
			}
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
			    err = ltr559_als_enable(als_gainrange);
				if(err < 0)
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
			    err = ltr559_als_disable();
				if(err < 0)
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
//(Lite-On Nelson edit)		    obj->als = ltr559_als_read(als_gainrange);
			obj->als = read_adc_value(ALS); //(Lite-On Nelson edit)
			if(obj->als < 0)
			{
				goto err_out;
			}

			dat = ltr559_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
//(Lite-On Nelson edit)			obj->als = ltr559_als_read(als_gainrange);
			obj->als = read_adc_value(ALS); //(Lite-On Nelson edit)
			if(obj->als < 0)
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}

/*----------------------------------------------------------------------------*/
static struct file_operations ltr559_fops = {
	//.owner = THIS_MODULE,
	.open = ltr559_open,
	.release = ltr559_release,
	.unlocked_ioctl = ltr559_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice ltr559_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr559_fops,
};

static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		err = ltr559_als_disable();
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}
             /*
		atomic_set(&obj->ps_suspend, 1);
		err = ltr559_ps_disable();
		if(err < 0)
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		ltr559_power(obj->hw, 0);
		*/
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_i2c_resume(struct i2c_client *client)
{
	struct ltr559_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	//ltr559_power(obj->hw, 1);
/*	err = ltr559_devinit();
	if(err < 0)
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}*/
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(als_gainrange);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	/*
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr559_ps_enable(ps_gainrange);
	    if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}*/
	return 0;
}

static void ltr559_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1); 
	err = ltr559_als_disable();
	if(err < 0)
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
}

static void ltr559_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(als_gainrange);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
}

int ltr559_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr559_priv *obj = (struct ltr559_priv *)self;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
                		//power_key_ps = false;  
				if(value)
				{
				    err = ltr559_ps_enable(ps_gainrange);
					if(err < 0)
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
				}
				else
				{
				    err = ltr559_ps_disable();
					if(err < 0)
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor ps data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
				obj->ps = ltr559_ps_read();
    			if(obj->ps < 0)
    			{
    				err = -1;
    				break;
    			}
				sensor_data->values[0] = ltr559_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int ltr559_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ltr559_priv *obj = (struct ltr559_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
				    err = ltr559_als_enable(als_gainrange);
					if(err < 0)
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
				    err = ltr559_als_disable();
					if(err < 0)
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				APS_ERR("get sensor als data !\n");
				sensor_data = (hwm_sensor_data *)buff_out;
//(Lite-On Nelson edit)				obj->als = ltr559_als_read(als_gainrange);
				obj->als = read_adc_value(ALS); //(Lite-On Nelson edit)
                //#if defined(MTK_AAL_SUPPORT)
				sensor_data->values[0] = obj->als;
				//#else
				//sensor_data->values[0] = ltr559_get_als_value(obj, obj->als);
				//#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int ltr559_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, LTR559_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr559_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr559_obj = obj;
	ps_value = 1;
	
      printk("ltr559_i2c_probe().....enter!\n");
	obj->hw = get_ltr_cust_alsps_hw();
	ltr559_get_addr(obj->hw, &obj->addr);

	INIT_WORK(&obj->eint_work, ltr559_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	//atomic_set(&obj->als_cmd_val, 0xDF);
	//atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	APS_LOG("ltr559_devinit() start...!\n");
	ltr559_i2c_client = client;

	if(err = ltr559_devinit())
	{
		printk("ltr559_devinit().....fail!\n");
		goto exit_init_failed;
	}
	APS_LOG("ltr559_devinit() ...OK!\n");

	//printk("@@@@@@ manufacturer value:%x\n",ltr559_i2c_read_reg(0x87));

	if(err = misc_register(&ltr559_device))
	{
		APS_ERR("ltr559_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	
	/* Register sysfs attribute */
	if(err = ltr559_create_attr(&(ltr_init_info.platform_diver_addr->driver)))
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}


	obj_ps.self = ltr559_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}
	obj_ps.sensor_operate = ltr559_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = ltr559_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = ltr559_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = ltr559_early_suspend,
	obj->early_drv.resume   = ltr559_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	ltr_init_flag = 0;
	return 0;

	exit_create_attr_failed:
	misc_deregister(&ltr559_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	ltr559_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	ltr_init_flag = -1;
	return err;
}

/*----------------------------------------------------------------------------*/

static int ltr559_i2c_remove(struct i2c_client *client)
{
	int err;	
	if(err = ltr559_delete_attr(&ltr559_i2c_driver.driver))
	{
		APS_ERR("ltr559_delete_attr fail: %d\n", err);
	} 

	if(err = misc_deregister(&ltr559_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	ltr559_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
// add sensor auto funtion 
#if 0
static int ltr559_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	ltr559_power(hw, 1);
	//ltr559_force[0] = hw->i2c_num;
	//ltr559_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",ltr559_force[0],ltr559_force[1]);
	if(i2c_add_driver(&ltr559_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int ltr559_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	ltr559_power(hw, 0);    
	i2c_del_driver(&ltr559_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver ltr559_alsps_driver = {
	.probe      = ltr559_probe,
	.remove     = ltr559_remove,    
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
#endif
/*----------------------------------------------------------------------------*/
static int ltr_remove(void)
{
    struct alsps_hw *hw = get_ltr_cust_alsps_hw();
    APS_FUN();

    ltr559_power(hw, 0);    
    i2c_del_driver(&ltr559_i2c_driver);
    return 0;
}

static int  ltr_local_init(void)
{
    APS_FUN();
    struct alsps_hw *hw = get_ltr_cust_alsps_hw();

    ltr559_power(hw, 1);
    if(i2c_add_driver(&ltr559_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }
    if(-1 == ltr_init_flag)
    {
       return -1;
    }
    
    return 0;
}
static int __init ltr559_init(void)
{
	APS_FUN();
	i2c_register_board_info(2, &i2c_ltr559, 1);
	/*if(platform_driver_register(&ltr559_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}*/
	 hwmsen_alsps_add(&ltr_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit ltr559_exit(void)
{
	APS_FUN();
	//platform_driver_unregister(&ltr559_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(ltr559_init);
module_exit(ltr559_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("mtk");
MODULE_DESCRIPTION("LTR-559ALS Driver");
MODULE_LICENSE("GPL");

