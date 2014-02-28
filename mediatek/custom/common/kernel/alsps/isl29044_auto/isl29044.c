/******************************************************************************
 * isl29044.h - Linux kernel module for Intersil isl29044 ambient light sensor
 *                and proximity sensor
 *
 * Copyright 2008-2012 Intersil Inc..
 *
 * DESCRIPTION:
 *    - This is the linux driver for isl29044.
 *        Kernel version 3.0.8
 *
 * modification history
 * --------------------
 * v1.0   2010/04/06, Shouxian Chen(Simon Chen) create this file
 * v1.1   2012/06/05, Shouxian Chen(Simon Chen) modified for Android 4.0 and 
 *            linux 3.0.8
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <cust_alsps.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <asm/io.h>
#include <linux/miscdevice.h>
#include "isl29044.h"
#define    DRIVER_VERSION    "1.1"

#define ALS_EN_MSK        (1 << 0)
#define PS_EN_MSK        (1 << 1)

#define PS_POLL_TIME    70         /* unit is ms */

#define TP_COEFF        13          

#define PS_DEF_FLOOR_NOISE        80
#define PS_MAX_FLOOR_NOISE        80
#define PS_THRESHOLD_H            95
#define PS_THRESHOLD_L            45
#define LSENSOR_MAX_LEVEL C_CUST_ALS_LEVEL

/* Each client has this additional data */

#define ISL29044_DBG(x...) do {\
            printk( x);\
    } while (0)

#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               ISL29044_DBG(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    ISL29044_DBG(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    ISL29044_DBG(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    ISL29044_DBG(KERN_INFO APS_TAG fmt, ##args)
#define ALS_TAG "[ALS] "
#define PS_TAG "[PS] "
#define ALS_LOG(fmt, args...) ISL29044_DBG(KERN_INFO ALS_TAG fmt, ## args)
#define PS_LOG(fmt, args...) ISL29044_DBG(KERN_INFO PS_TAG fmt, ## args)
#define ALS_ERR(fmt, args...) ISL29044_DBG(KERN_ERR ALS_TAG "%s %d : " fmt, __func__, __LINE__, ## args)
#define PS_ERR(fmt, args...) ISL29044_DBG(KERN_ERR PS_TAG "%s %d : " fmt, __func__, __LINE__, ## args)
#define MTK_PLATFORM_ISL

static int min_proximity_value = 250;
static int pwin_value = 60;
static int pwave_value = 70;

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

static struct i2c_board_info __initdata i2c_ISl29044={ I2C_BOARD_INFO("isl29044", 0x44)}; 

struct isl29044_data_t {
       struct alsps_hw  *hw;
    struct i2c_client* client;
    u8 als_pwr_status;
    u8 ps_pwr_status;
    u8 ps_led_drv_cur;    /* led driver current, 0: 110mA, 1: 220mA */
    u8 als_range;        /* als range, 0: 125 Lux, 1: 2000Lux */
    u8 als_mode;        /* als mode, 0: Visible light, 1: IR light */
    u8 ps_lt;            /* ps low limit */
    u8 ps_ht;            /* ps high limit */
    int poll_delay;        /* poll delay set by hal */
    u8 show_als_raw;    /* show als raw data flag, used for debug */
    u8 show_ps_raw;    /* show als raw data flag, used for debug */
    struct timer_list als_timer;    /* als poll timer */
    struct timer_list ps_timer;    /* ps poll timer */
    spinlock_t als_timer_lock;
    spinlock_t ps_timer_lock;
    struct work_struct als_work;
    struct work_struct ps_work;
    struct workqueue_struct *als_wq;
    struct workqueue_struct *ps_wq;
    struct input_dev *als_input_dev;
    struct input_dev *ps_input_dev;
    int last_ps;
    u8 als_range_using;        /* the als range using now */
    u8 als_pwr_before_suspend;
    u8 ps_pwr_before_suspend;
    
    u32 als_ir_cnt;
    u32 als_ir_cnt_set;
    u16 als_ir;
    u8 als_type_now; /* 0 for als , 1 for IR */
    
    u8 ps_init;
    u8 ps_floor_noise;

       int als_value;
       int ps_value;
       ulong     enable; 
       #if defined(CONFIG_HAS_EARLYSUSPEND)
       struct early_suspend    early_drv;
       #endif
};

static int set_sensor_reg(struct isl29044_data_t *dev_dat);
static int set_sensor_power(struct isl29044_data_t *dev_dat);

/*----------------------------------------------------------------------------*/
extern int hwmsen_alsps_add(struct sensor_init_info* obj);
extern struct alsps_hw *get_isl_cust_alsps_hw(void); 
static int  isl29044_local_init(void);
static int  isl29044_remove(void);
static int  isl29044_init_flag =-1; // 0<==>OK -1 <==> fail

static struct sensor_init_info isl29044_init_info = {
        .name = "isl29044",
        .init = isl29044_local_init,
        .uninit = isl29044_remove,
};
/*----------------------------------------------------------------------------*/

/* Do not scan isl29028 automatic */
static const unsigned short normal_i2c[] = {ISL29044_ADDR, I2C_CLIENT_END };

/* data struct for isl29044 device */
struct isl29044_data_t    isl29044_data = {
    .client = NULL,
    .als_pwr_status = 0,
    .ps_pwr_status = 0,
    .als_input_dev = NULL,
    .ps_input_dev = NULL
};

static void do_als_timer(unsigned long arg)
{
    struct isl29044_data_t *dev_dat;

    dev_dat = (struct isl29044_data_t *)arg;

    /* timer handler is atomic context, so don't need sinp_lock() */
    //spin_lock(&dev_dat->als_timer);
    if(dev_dat->als_pwr_status == 0)
    {
        //spin_unlock(&dev_dat->als_timer);
        return ;
    }
    //spin_unlock(&dev_dat->als_timer);

    /* start a work queue, I cannot do i2c oepration in timer context for
       this context is atomic and i2c function maybe sleep. */
    //schedule_work(&dev_dat->als_work);
    queue_work(dev_dat->als_wq, &dev_dat->als_work);
}

static void do_ps_timer(unsigned long arg)
{
    struct isl29044_data_t *dev_dat;

    dev_dat = (struct isl29044_data_t *)arg;

    if(dev_dat->ps_pwr_status == 0)
    {
        return ;
    }
    
    /* start a work queue, I cannot do i2c oepration in timer context for
       this context is atomic and i2c function maybe sleep. */
    //schedule_work(&dev_dat->ps_work);
    queue_work(dev_dat->ps_wq, &dev_dat->ps_work);
}

static void isl_read_als(struct isl29044_data_t *dev_data)
{
    struct isl29044_data_t *dev_dat;
    int ret;
    int als_dat;
    int lux;
    u8 als_range;
    u8 reset_reg = 0;
    u8 als_ir_cnt;
    u8 als_type_now;
    u8 reg_dat[2];
    u32 als_factor_d, als_factor_n;
    int next_timer;
    int temp;
    u8 buf[2];
       int als_level;
       int i;
    
    dev_dat = dev_data;

    als_range = dev_dat->als_range_using;
    
    ret = i2c_smbus_read_i2c_block_data(dev_dat->client, 0x09, 2, buf);
    if(ret < 0) 
       {
           ALS_ERR("isl_read_als,read data error!!!!!!\n");
           goto err_rd;
       }
    als_dat = buf[0] + (buf[1] << 8);
    
    /*
    spin_lock(&dev_dat->ps_timer_lock);
    als_ir_cnt = dev_dat->als_ir_cnt;
    als_type_now = dev_dat->als_type_now;
    spin_unlock(&dev_dat->ps_timer_lock);
    */

    if(als_range)
    {
        lux = (als_dat * 2000 * TP_COEFF) / 4096;
    }
    else
    {
        lux = (als_dat * 125 * TP_COEFF) / 4096;
    }
       printk("isl_read_als lux=%d\n",lux);
    #ifndef MTK_PLATFORM_ISL
    input_report_abs(dev_dat->als_input_dev, ABS_MISC, lux);
    input_sync(dev_dat->als_input_dev);
      #endif
      
      if (lux >= 0)
     {
        
           als_level = LSENSOR_MAX_LEVEL - 1;
            
          for (i = 0; i < ARRAY_SIZE(dev_dat->hw->als_level); i++)
        {
                   if (lux < dev_dat->hw->als_level[i])
                  {
                 als_level = i;
                  break;
             }
          }
           

        dev_dat->als_value= dev_dat->hw->als_value[als_level];
      }
      else
      {
              ALS_LOG("Need to change TP_COFF %2d \n", lux);
      }
       
    
      /* change the measurement range */
       
    spin_lock(&dev_dat->als_timer_lock);
    if(dev_dat->als_range)
    {
        if(als_dat < 204)    /* About 100 Lux */
        {
            dev_dat->als_range = 0;
            reset_reg = 1;
        }
    }
    else
    {
        if(als_dat > 3768)    /* About 115 Lux */
        {
            dev_dat->als_range = 1;
            reset_reg = 1;
        }
    }
    spin_unlock(&dev_dat->als_timer_lock);
   
    if(reset_reg)
    {
        set_sensor_power(dev_dat);
        dev_dat->als_range_using = dev_dat->als_range;
    }
    
      #if 0
      /* restart timer */            
    spin_lock(&dev_dat->als_timer_lock);
    if(dev_dat->als_pwr_status == 0)
    {
        spin_unlock(&dev_dat->als_timer_lock);
        return ;
    }
    next_timer = dev_dat->poll_delay;
    if(als_ir_cnt == 0)
    {
        next_timer = next_timer - 250;
        if(next_timer < 250) next_timer = 250;
    }
    
    dev_dat->als_timer.expires = jiffies + (HZ * next_timer) / 1000;
    spin_unlock(&dev_dat->als_timer_lock);
    add_timer(&dev_dat->als_timer);    
       #endif
    return ;
       
err_rd:
    printk(KERN_ERR "Read ps sensor error, ret = %d\n", ret);
    return ;
    
err_wr:
    printk(KERN_ERR "Write ps sensor error, ret = %d\n", ret);
    return ;
}


static void do_als_work(struct work_struct *work)
{
    struct isl29044_data_t *dev_dat;
    int ret;
    int als_dat;
    u8 show_raw_dat;
    int lux;
    u8 als_range;
    u8 reset_reg = 0;
    u8 als_ir_cnt;
    u8 als_type_now;
    u8 reg_dat[2];
    u32 als_factor_d, als_factor_n;
    int next_timer;
    int temp;
    u8 buf[2];
    
    dev_dat = container_of(work, struct isl29044_data_t, als_work);

    spin_lock(&dev_dat->als_timer_lock);
    show_raw_dat = dev_dat->show_als_raw;
    //show_raw_dat = 1;//tina
    spin_unlock(&dev_dat->als_timer_lock);

    als_range = dev_dat->als_range_using;
    
    ret = i2c_smbus_read_i2c_block_data(dev_dat->client, 0x09, 2, buf);
    if(ret < 0) goto err_rd;
    als_dat = buf[0] + (buf[1] << 8);
    
    /*
    spin_lock(&dev_dat->ps_timer_lock);
    als_ir_cnt = dev_dat->als_ir_cnt;
    als_type_now = dev_dat->als_type_now;
    spin_unlock(&dev_dat->ps_timer_lock);
    */

    if(als_range)
    {
        lux = (als_dat * 2000 * TP_COEFF) / 4096;
    }
    else
    {
        lux = (als_dat * 125 * TP_COEFF) / 4096;
    }
    
    input_report_abs(dev_dat->als_input_dev, ABS_MISC, lux);
    input_sync(dev_dat->als_input_dev);
    if(show_raw_dat)
    {
        printk(KERN_INFO "now als raw data is = %d, LUX = %d\n", als_dat, lux);
    }
                
    /* change the measurement range */
    spin_lock(&dev_dat->als_timer_lock);
    if(dev_dat->als_range)
    {
        if(als_dat < 204)    /* About 100 Lux */
        {
            dev_dat->als_range = 0;
            reset_reg = 1;
        }
    }
    else
    {
        if(als_dat > 3768)    /* About 115 Lux */
        {
            dev_dat->als_range = 1;
            reset_reg = 1;
        }
    }
    spin_unlock(&dev_dat->als_timer_lock);

    if(reset_reg)
    {
        set_sensor_reg(dev_dat);
        dev_dat->als_range_using = dev_dat->als_range;
    }
    
    /* restart timer */            
    spin_lock(&dev_dat->als_timer_lock);
    if(dev_dat->als_pwr_status == 0)
    {
        spin_unlock(&dev_dat->als_timer_lock);
        return ;
    }
    next_timer = dev_dat->poll_delay;
    if(als_ir_cnt == 0)
    {
        next_timer = next_timer - 250;
        if(next_timer < 250) next_timer = 250;
    }
    
    dev_dat->als_timer.expires = jiffies + (HZ * next_timer) / 1000;
    spin_unlock(&dev_dat->als_timer_lock);
    add_timer(&dev_dat->als_timer);    
    
    return ;

err_rd:
    printk(KERN_ERR "Read ps sensor error, ret = %d\n", ret);
    return ;
    
err_wr:
    printk(KERN_ERR "Write ps sensor error, ret = %d\n", ret);
    return ;
}

static void isl_read_ps(struct isl29044_data_t *dev_data)
{
    struct isl29044_data_t *dev_dat;
    int last_ps;
    int ret;
    u8 show_raw_dat;
    u8 ps_raw_dat;
    u8 reg_dat[5];
    int i;
       int closedate;
       int far_away;

    dev_dat = dev_data;

    //spin_lock(&dev_dat->ps_timer_lock);
    //show_raw_dat = dev_dat->show_ps_raw;
    //show_raw_dat = 1;//tina
    //spin_unlock(&dev_dat->ps_timer_lock);
    
    ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
        
    if(ret < 0)
       {
             PS_ERR("isl_read_ps,read raw data err\n");
             goto err_rd;
       }
    ps_raw_dat = ret;    
    
      printk("isl_read_ps ps_raw_dat :%d\n", ps_raw_dat);
      if (((ps_raw_dat + pwave_value) < min_proximity_value)&& (ps_raw_dat>0))
     {
            min_proximity_value = ps_raw_dat + pwave_value;
            closedate = min_proximity_value + pwin_value;
        if(closedate >=255 )
        {
            closedate = 255;
            min_proximity_value = 255-pwin_value;
        }
        far_away = min_proximity_value;
        ret  = i2c_smbus_write_byte_data(dev_dat->client, 0x3, far_away);
        ret |= i2c_smbus_write_byte_data(dev_dat->client, 0x4, closedate);
        
        if (ret)
        {
            PS_ERR(":set ISL29044A_CMM_INT_LOW_THD_LOW register is error(%d)!", ret);
        }

        PS_LOG("%s:min_proximity_value=%d\n", __func__, min_proximity_value);
        
        return;
        
     }
      far_away= i2c_smbus_read_byte_data(dev_dat->client, 0x3);
    if(far_away < 0)
      {   
           PS_ERR("isl_read_ps,read far_away error\n");
           goto err_rd;
      }   
    

      closedate = i2c_smbus_read_byte_data(dev_dat->client, 0x4);
    if(closedate < 0)
      {   
           PS_ERR("isl_read_ps,read closedate error\n");
           goto err_rd;
      } 
        
    
      if(ps_raw_dat>=closedate)
      {
          dev_dat->last_ps = 0;
          dev_dat->ps_value = 0;
      }
      if(ps_raw_dat<=far_away)
     {
          dev_dat->last_ps = 1;
          dev_dat->ps_value = 1;
     }
     printk("proximity send:%d\n", dev_dat->ps_value);
      
      
       
      #if 0
    if(dev_dat->ps_init)
    {
        /* read ps raw data */
        ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
        if(ret < 0) goto err_rd;
        
        spin_lock(&dev_dat->ps_timer_lock);
        ps_raw_dat = ret;
        if(ps_raw_dat > PS_MAX_FLOOR_NOISE)
        {
            /* ps data is too larger, I think the floor noise is not right, use default */
            dev_dat->ps_floor_noise = PS_DEF_FLOOR_NOISE;
        }
        else
        {
            dev_dat->ps_floor_noise = ps_raw_dat;
        }
        dev_dat->ps_init = 0;
        spin_unlock(&dev_dat->ps_timer_lock);
        
        /* set reg ps threshold */
        reg_dat[3] = dev_dat->ps_floor_noise + dev_dat->ps_lt;
        reg_dat[4] = dev_dat->ps_floor_noise + dev_dat->ps_ht;
        for(i = 3 ; i <= 4; i++)
        {
            ret = i2c_smbus_write_byte_data(dev_dat->client, i, reg_dat[i]);
            if(ret < 0) 
            {
                printk(KERN_ERR "In set_sensor_reg() err0, ret=%d\n", ret);
                return;
            }
        }
        
        if(show_raw_dat)
        {
            printk(KERN_INFO "ps floor noise raw data = %d\n", dev_dat->ps_floor_noise);
        }
    }
    else
    {
        //ret = i2c_smbus_read_byte_data(dev_dat->client, 0x02);
        //if(ret < 0) goto err_rd;
        
        ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
        if(ret < 0) goto err_rd;
        printk(KERN_INFO "ps raw data = %d\n", ret);
        
        ps_raw_dat = ret;
        if(dev_dat->ps_floor_noise > ps_raw_dat) dev_dat->ps_floor_noise = ps_raw_dat;
        
        last_ps = dev_dat->last_ps;
        if(ps_raw_dat > (dev_dat->ps_floor_noise + dev_dat->ps_ht)) dev_dat->last_ps = 0;
        else if(ps_raw_dat < (dev_dat->ps_floor_noise + dev_dat->ps_lt)) dev_dat->last_ps = 1;
        //dev_dat->last_ps = (ret & 0x80) ? 0 : 1;

        if(show_raw_dat)
        {
            printk(KERN_INFO "ps raw data = %d\n", ret);
        }

        if(last_ps != dev_dat->last_ps)
        {
                   #ifndef MTK_PLATFORM_ISL
                    input_report_abs(dev_dat->ps_input_dev, ABS_DISTANCE, dev_dat->last_ps);
            input_sync(dev_dat->ps_input_dev);
                    #else
                    dev_dat->ps_value =dev_dat->last_ps;
                    #endif
            if(show_raw_dat)
            {
                printk(KERN_INFO "ps status changed, now = %d\n",dev_dat->last_ps);
            }
                   
        }
    }
       #endif

    /* restart timer */
       #ifndef MTK_PLATFORM_ISL
    spin_lock(&dev_dat->ps_timer_lock);
    if(dev_dat->ps_pwr_status == 0)
    {
        spin_unlock(&dev_dat->ps_timer_lock);
        return ;
    }
    dev_dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
    spin_unlock(&dev_dat->ps_timer_lock);
    add_timer(&dev_dat->ps_timer);
       #endif

    return ;

err_rd:
    printk(KERN_ERR "Read als sensor error, ret = %d\n", ret);
    return ;
}

static void isl_read_ps_func(struct work_struct *work)
{
       struct isl29044_data_t *dev_dat;
       hwm_sensor_data sensor_data;
       int ret = 0;
    dev_dat = container_of(work, struct isl29044_data_t, ps_work);
       isl_read_ps(dev_dat);
       
    sensor_data.values[0] = dev_dat->ps_value;
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
       if ((ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
    {
        PS_ERR("call hwmsen_get_interrupt_data fail = %d\n", ret);
    }
       spin_lock(&dev_dat->ps_timer_lock);
    if(dev_dat->ps_pwr_status == 0)
    {
        spin_unlock(&dev_dat->ps_timer_lock);
        return ;
    }
    dev_dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
    spin_unlock(&dev_dat->ps_timer_lock);
    add_timer(&dev_dat->ps_timer);

    return;
       
}

static void do_ps_work(struct work_struct *work)
{
    struct isl29044_data_t *dev_dat;
    int last_ps;
    int ret;
    u8 show_raw_dat;
    u8 ps_raw_dat;
    u8 reg_dat[5];
    int i;

    dev_dat = container_of(work, struct isl29044_data_t, ps_work);

    spin_lock(&dev_dat->ps_timer_lock);
    show_raw_dat = dev_dat->show_ps_raw;
    //show_raw_dat = 1;//tina
    spin_unlock(&dev_dat->ps_timer_lock);
    
    /* initial to find the ps floor noise */
    if(dev_dat->ps_init)
    {
        /* read ps raw data */
        ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
        if(ret < 0) goto err_rd;
        
        spin_lock(&dev_dat->ps_timer_lock);
        ps_raw_dat = ret;
        if(ps_raw_dat > PS_MAX_FLOOR_NOISE)
        {
            /* ps data is too larger, I think the floor noise is not right, use default */
            dev_dat->ps_floor_noise = PS_DEF_FLOOR_NOISE;
        }
        else
        {
            dev_dat->ps_floor_noise = ps_raw_dat;
        }
        dev_dat->ps_init = 0;
        spin_unlock(&dev_dat->ps_timer_lock);
        
        /* set reg ps threshold */
        reg_dat[3] = dev_dat->ps_floor_noise + dev_dat->ps_lt;
        reg_dat[4] = dev_dat->ps_floor_noise + dev_dat->ps_ht;
        for(i = 3 ; i <= 4; i++)
        {
            ret = i2c_smbus_write_byte_data(dev_dat->client, i, reg_dat[i]);
            if(ret < 0) 
            {
                printk(KERN_ERR "In set_sensor_reg() err0, ret=%d\n", ret);
                return;
            }
        }
        
        if(show_raw_dat)
        {
            printk(KERN_INFO "ps floor noise raw data = %d\n", dev_dat->ps_floor_noise);
        }
    }
    else
    {
        //ret = i2c_smbus_read_byte_data(dev_dat->client, 0x02);
        //if(ret < 0) goto err_rd;
        
        ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
        if(ret < 0) goto err_rd;
        printk(KERN_INFO "ps raw data = %d\n", ret);
        
        ps_raw_dat = ret;
        if(dev_dat->ps_floor_noise > ps_raw_dat) dev_dat->ps_floor_noise = ps_raw_dat;
        
        last_ps = dev_dat->last_ps;
        if(ps_raw_dat > (dev_dat->ps_floor_noise + dev_dat->ps_ht)) dev_dat->last_ps = 0;
        else if(ps_raw_dat < (dev_dat->ps_floor_noise + dev_dat->ps_lt)) dev_dat->last_ps = 1;
        //dev_dat->last_ps = (ret & 0x80) ? 0 : 1;

        if(show_raw_dat)
        {
            printk(KERN_INFO "ps raw data = %d\n", ret);
        }

        if(last_ps != dev_dat->last_ps)
        {
            input_report_abs(dev_dat->ps_input_dev, ABS_DISTANCE, dev_dat->last_ps);
            input_sync(dev_dat->ps_input_dev);
            if(show_raw_dat)
            {
                printk(KERN_INFO "ps status changed, now = %d\n",dev_dat->last_ps);
            }
        }
    }

    /* restart timer */
    spin_lock(&dev_dat->ps_timer_lock);
    if(dev_dat->ps_pwr_status == 0)
    {
        spin_unlock(&dev_dat->ps_timer_lock);
        return ;
    }
    dev_dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
    spin_unlock(&dev_dat->ps_timer_lock);
    add_timer(&dev_dat->ps_timer);

    return ;

err_rd:
    printk(KERN_ERR "Read als sensor error, ret = %d\n", ret);
    return ;
}

/* enable to run als */
static int set_sensor_reg(struct isl29044_data_t *dev_dat)
{
    u8 reg_dat[5]={0};
    int i, ret;
    printk(KERN_DEBUG "In set_sensor_reg()\n");
    reg_dat[2] = 0x22;
    
    spin_lock(&dev_dat->ps_timer_lock);
    if(dev_dat->ps_init == 1)
    {
        /* find new ps floor noise, set this threshold to make sure no intterrupt */
        reg_dat[3] = 0;
        reg_dat[4] = 255;
    }
    else
    {
        reg_dat[3] = dev_dat->ps_floor_noise + dev_dat->ps_lt;
        reg_dat[4] = dev_dat->ps_floor_noise + dev_dat->ps_ht;
    }
    spin_unlock(&dev_dat->ps_timer_lock);

    reg_dat[1] = 0x50;    /* set ps sleep time to 50ms */
    spin_lock(&dev_dat->als_timer_lock);
    if(dev_dat->als_pwr_status)
    {
        /* measurement als */
        reg_dat[1] |= 0x04;
    }
    spin_unlock(&dev_dat->als_timer_lock);
        
    spin_lock(&dev_dat->ps_timer_lock);
    if(dev_dat->ps_pwr_status) reg_dat[1] |= 0x80;
    spin_unlock(&dev_dat->ps_timer_lock);
    
    if(dev_dat->als_mode) reg_dat[1] |= 0x01;
    if(dev_dat->als_range) reg_dat[1] |= 0x02;
    if(dev_dat->ps_led_drv_cur) reg_dat[1] |= 0x08;

    for(i = 2 ; i <= 4; i++)
    {
        ret = i2c_smbus_write_byte_data(dev_dat->client, i, reg_dat[i]);
        if(ret < 0) 
        {
            printk(KERN_ERR "In set_sensor_reg() err0, ret=%d\n", ret);
            return ret;
        }
    }
    
    ret = i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg_dat[1]);
    if(ret < 0) 
    {
        printk(KERN_ERR, "In set_sensor_reg() err1, ret=%d\n", ret);
        return ret;
    }

    return 0;
}
static int set_sensor_power(struct isl29044_data_t *dev_dat)
{
      u8 reg_dat=0;
    int i, ret;
    printk("In set_sensor_power(),%d\n",dev_dat->als_pwr_status);
       reg_dat = 0x50;    /* set ps sleep time to 50ms */
    spin_lock(&dev_dat->als_timer_lock);
    if(dev_dat->als_pwr_status)
    {
        /* measurement als */
        reg_dat |= 0x04;
    }
    spin_unlock(&dev_dat->als_timer_lock);
      
       spin_lock(&dev_dat->ps_timer_lock);
    if(dev_dat->ps_pwr_status) reg_dat |= 0x80;
    spin_unlock(&dev_dat->ps_timer_lock);
    
    if(dev_dat->als_mode) reg_dat |= 0x01;
    if(dev_dat->als_range) reg_dat |= 0x02;
    if(dev_dat->ps_led_drv_cur) reg_dat|= 0x08;
      ret = i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg_dat);
    if(ret < 0) 
    {
        printk(KERN_ERR, "In set_sensor_power() err1, ret=%d\n", ret);
        return ret;
    }
      
      
}

/* set power status */
static int set_als_pwr_st(u8 state, struct isl29044_data_t *dat)
{
    int ret = 0;
    
    if(state)
    {
        spin_lock(&dat->als_timer_lock);
        if(dat->als_pwr_status)
        {
            spin_unlock(&dat->als_timer_lock);
            return ret;
        }
        dat->als_pwr_status = 1;
        spin_unlock(&dat->als_timer_lock);
        ret = set_sensor_power(dat);
        if(ret < 0)
        {
            printk(KERN_ERR "set light sensor reg error, ret = %d\n", ret);
            return ret;
        }
        dat->als_range_using = dat->als_range;
        /* start timer */
             #ifndef MTK_PLATFORM_ISL
        dat->als_timer.function = &do_als_timer;
        dat->als_timer.data = (unsigned long)dat;
        spin_lock(&dat->als_timer_lock);
        dat->als_timer.expires = jiffies + (HZ * dat->poll_delay) / 1000;
        spin_unlock(&dat->als_timer_lock);
        
        //dat->als_range_using = dat->als_range;
        add_timer(&dat->als_timer);
             #endif
    }
    else
    {
        spin_lock(&dat->als_timer_lock);
        if(dat->als_pwr_status == 0)
        {
            spin_unlock(&dat->als_timer_lock);
            return ret;
        }
        dat->als_pwr_status = 0;
        spin_unlock(&dat->als_timer_lock);
        ret = set_sensor_power(dat);

        /* delete timer */
             #ifndef MTK_PLATFORM_ISL
        del_timer_sync(&dat->als_timer);
             #endif
    }

    return ret;
}

static int set_ps_pwr_st(u8 state, struct isl29044_data_t *dat)
{
    int ret = 0;
    
    if(state)
    {
        spin_lock(&dat->ps_timer_lock);
        if(dat->ps_pwr_status)
        {
            spin_unlock(&dat->ps_timer_lock);
            return ret;
        }
        dat->ps_pwr_status = 1;
        dat->ps_init = 1;
        spin_unlock(&dat->ps_timer_lock);
        
        dat->last_ps = -1;
        ret = set_sensor_power(dat);
        if(ret < 0)
        {
            printk(KERN_ERR "set proximity sensor reg error, ret = %d\n", ret);
            return ret;
        }    
        
        /* start timer */
             
        dat->ps_timer.function = &do_ps_timer;
        dat->ps_timer.data = (unsigned long)dat;
        dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
        add_timer(&dat->ps_timer);
             
    }
    else
    {
        spin_lock(&dat->ps_timer_lock);
        if(dat->ps_pwr_status == 0)
        {
            spin_unlock(&dat->ps_timer_lock);
            return ret;
        }
        dat->ps_pwr_status = 0;
        spin_unlock(&dat->ps_timer_lock);

        ret = set_sensor_power(dat);
        
        /* delete timer */
             
        del_timer_sync(&dat->ps_timer);
             
    }

    return ret;
}

/* device attribute */
/* enable als attribute */
static ssize_t show_enable_als_sensor(struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;
    u8 pwr_status;

    dat = (struct isl29044_data_t *)dev->platform_data;
    spin_lock(&dat->als_timer_lock);
    pwr_status = dat->als_pwr_status;
    spin_unlock(&dat->als_timer_lock);

    return snprintf(buf, PAGE_SIZE, "%d\n", pwr_status);
}
static ssize_t store_enable_als_sensor(struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    ssize_t ret;
    unsigned long val;

    dat = (struct isl29044_data_t *)dev->platform_data;
        
    val = simple_strtoul(buf, NULL, 10);
    printk(KERN_DEBUG "set als enable, val = %d\n", val);
    ret = set_als_pwr_st(val, dat);

    if(ret == 0) ret = count;
    return ret;
}
static DEVICE_ATTR(enable_als_sensor, S_IWUGO|S_IRUGO, show_enable_als_sensor,
    store_enable_als_sensor);

/* enable ps attribute */
static ssize_t show_enable_ps_sensor(struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;
    u8 pwr_status;

    dat = (struct isl29044_data_t *)dev->platform_data;
    spin_lock(&dat->ps_timer_lock);
    pwr_status = dat->ps_pwr_status;
    spin_unlock(&dat->ps_timer_lock);

    return snprintf(buf, PAGE_SIZE, "%d\n", pwr_status);
}
static ssize_t store_enable_ps_sensor(struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    ssize_t ret;
    unsigned long val;

    dat = (struct isl29044_data_t *)dev->platform_data;
        
    val = simple_strtoul(buf, NULL, 10);
    ret = set_ps_pwr_st(val, dat);

    if(ret == 0) ret = count;
    return ret;
}
static DEVICE_ATTR(enable_ps_sensor, S_IWUGO|S_IRUGO, show_enable_ps_sensor,
    store_enable_ps_sensor);

/* ps led driver current attribute */
static ssize_t show_ps_led_drv(struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;

    dat = (struct isl29044_data_t *)dev->platform_data;
    return snprintf(buf, PAGE_SIZE, "%d\n", dat->ps_led_drv_cur);
}
static ssize_t store_ps_led_drv(struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int val;

    if(sscanf(buf, "%d", &val) != 1)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;
    if(val) dat->ps_led_drv_cur = 1;
    else dat->ps_led_drv_cur = 0;
    
    return count;
}
static DEVICE_ATTR(ps_led_driver_current, S_IWUGO|S_IRUGO, show_ps_led_drv,
    store_ps_led_drv);

/* als range attribute */
static ssize_t show_als_range(struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;
    u8 range;
    
    dat = (struct isl29044_data_t *)dev->platform_data;
    spin_lock(&dat->als_timer_lock);
    range = dat->als_range;
    spin_unlock(&dat->als_timer_lock);
    
    return snprintf(buf, PAGE_SIZE, "%d\n", range);
}
static ssize_t store_als_range(struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int val;

    if(sscanf(buf, "%d", &val) != 1)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;
    
    spin_lock(&dat->als_timer_lock);
    if(val) dat->als_range = 1;
    else dat->als_range = 0;
    spin_unlock(&dat->als_timer_lock);
    
    return count;
}
static DEVICE_ATTR(als_range, S_IWUGO|S_IRUGO, show_als_range, store_als_range);

/* als mode attribute */
static ssize_t show_als_mode(struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;

    dat = (struct isl29044_data_t *)dev->platform_data;
    return snprintf(buf, PAGE_SIZE, "%d\n", dat->als_mode);
}
static ssize_t store_als_mode(struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int val;

    if(sscanf(buf, "%d", &val) != 1)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;
    if(val) dat->als_mode = 1;
    else dat->als_mode = 0;
    
    return count;
}
static DEVICE_ATTR(als_mode, S_IWUGO|S_IRUGO, show_als_mode, store_als_mode);

/* ps limit range attribute */
static ssize_t show_ps_limit(struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;

    dat = (struct isl29044_data_t *)dev->platform_data;
    return snprintf(buf, PAGE_SIZE, "%d %d\n", dat->ps_lt, dat->ps_ht);
}
static ssize_t store_ps_limit(struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int lt, ht;

    if(sscanf(buf, "%d %d", &lt, &ht) != 2)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;
    
    if(lt > 255) dat->ps_lt = 255;
    else if(lt < 0) dat->ps_lt = 0;
    else  dat->ps_lt = lt;
    
    if(ht > 255) dat->ps_ht = 255;
    else if(ht < 0) dat->ps_ht = 0;
    else  dat->ps_ht = ht;
    
    return count;
}
static DEVICE_ATTR(ps_limit, S_IWUGO|S_IRUGO, show_ps_limit, store_ps_limit);

/* poll delay attribute */
static ssize_t show_poll_delay (struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;
    int delay;

    dat = (struct isl29044_data_t *)dev->platform_data;
    spin_lock(&dat->als_timer_lock);
    delay = dat->poll_delay;
    spin_unlock(&dat->als_timer_lock);
    
    return snprintf(buf, PAGE_SIZE, "%d\n", delay);
}
static ssize_t store_poll_delay (struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int delay;

    if(sscanf(buf, "%d", &delay) != 1)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;

    printk(KERN_DEBUG "set als poll delay, delay=%d\n", delay);
    spin_lock(&dat->als_timer_lock);
    delay = delay / 1000;
    if(delay  < 120) dat->poll_delay = 120;
    else if(delay > 1000000) dat->poll_delay = 1000000;
    else dat->poll_delay = delay;
    
    if(dat->als_pwr_status)
    {
        del_timer(&dat->als_timer);
        dat->als_timer.expires = jiffies + (HZ * dat->poll_delay) / 1000;
        add_timer(&dat->als_timer);
    }
    
    spin_unlock(&dat->als_timer_lock);
    
    return count;
}
static DEVICE_ATTR(als_poll_delay, S_IWUGO|S_IRUGO, show_poll_delay, 
    store_poll_delay);

/* show als raw data attribute */
static ssize_t show_als_show_raw (struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;
    u8 flag;

    dat = (struct isl29044_data_t *)dev->platform_data;
    spin_lock(&dat->als_timer_lock);
    flag = dat->show_als_raw;
    spin_unlock(&dat->als_timer_lock);
    
    return snprintf(buf, PAGE_SIZE, "%d\n", flag);
}
static ssize_t store_als_show_raw (struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int flag;

    if(sscanf(buf, "%d", &flag) != 1)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;

    spin_lock(&dat->als_timer_lock);
    if(flag == 0) dat->show_als_raw = 0;
    else dat->show_als_raw = 1;
    spin_unlock(&dat->als_timer_lock);
    
    return count;
}
static DEVICE_ATTR(als_show_raw, S_IWUGO|S_IRUGO, show_als_show_raw, 
    store_als_show_raw);


/* show ps raw data attribute */
static ssize_t show_ps_show_raw (struct device *dev,
              struct device_attribute *attr, char *buf)
{
    struct isl29044_data_t *dat;
    u8 flag;

    dat = (struct isl29044_data_t *)dev->platform_data;
    spin_lock(&dat->als_timer_lock);
    flag = dat->show_ps_raw;
    spin_unlock(&dat->als_timer_lock);
    
    return snprintf(buf, PAGE_SIZE, "%d\n", flag);
}
static ssize_t store_ps_show_raw (struct device *dev, 
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct isl29044_data_t *dat;
    int flag;

    if(sscanf(buf, "%d", &flag) != 1)
    {
        return -EINVAL;
    }
    
    dat = (struct isl29044_data_t *)dev->platform_data;

    spin_lock(&dat->als_timer_lock);
    if(flag == 0) dat->show_ps_raw = 0;
    else dat->show_ps_raw = 1;
    spin_unlock(&dat->als_timer_lock);
    
    return count;
}
static DEVICE_ATTR(ps_show_raw, S_IWUGO|S_IRUGO, show_ps_show_raw, 
    store_ps_show_raw);

static struct attribute *als_attr[] = {
    &dev_attr_enable_als_sensor.attr,
    &dev_attr_als_range.attr,
    &dev_attr_als_mode.attr,
    &dev_attr_als_poll_delay.attr,
    &dev_attr_als_show_raw.attr,
    NULL
};

static struct attribute_group als_attr_grp = {
    .name = "device",
    .attrs = als_attr
};

static struct attribute *ps_attr[] = {
    &dev_attr_enable_ps_sensor.attr,
    &dev_attr_ps_led_driver_current.attr,
    &dev_attr_ps_limit.attr,
    &dev_attr_ps_show_raw.attr,
    NULL
};

static struct attribute_group ps_attr_grp = {
    .name = "device",
    .attrs = ps_attr
};


/* initial and register a input device for sensor */
#if 0
static int init_input_dev(struct isl29044_data_t *dev_dat)
{
    int err;
    struct input_dev *als_dev;
    struct input_dev *ps_dev;
    
    als_dev = input_allocate_device();
    if (!als_dev)
    {
        return -ENOMEM;
    }

    ps_dev = input_allocate_device();
    if (!ps_dev)
    {
        err = -ENOMEM;    
        goto err_free_als;
    }

    als_dev->name = LIGHT_INPUT_DEV_NAME;//"light sensor";
    als_dev->id.bustype = BUS_I2C;
    als_dev->id.vendor  = 0x0001;
    als_dev->id.product = 0x0001;
    als_dev->id.version = 0x0100;
    als_dev->evbit[0] = BIT_MASK(EV_ABS);
    als_dev->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);
    als_dev->dev.platform_data = &isl29044_data;
    input_set_abs_params(als_dev, ABS_MISC, 0, 2000 * TP_COEFF, 0, 0);
    
    ps_dev->name = PROX_INPUT_DEV_NAME;//"proximity sensor";
    ps_dev->id.bustype = BUS_I2C;
    ps_dev->id.vendor  = 0x0001;
    ps_dev->id.product = 0x0002;
    ps_dev->id.version = 0x0100;
    ps_dev->evbit[0] = BIT_MASK(EV_ABS);
    ps_dev->absbit[BIT_WORD(ABS_DISTANCE)] |= BIT_MASK(ABS_DISTANCE);
    ps_dev->dev.platform_data = &isl29044_data;
    input_set_abs_params(ps_dev, ABS_DISTANCE, 0, 1, 0, 0);
    
    err = input_register_device(als_dev);
    if (err)
    {
        goto err_free_ps;
    }

    err = input_register_device(ps_dev);
    if (err)
    {
        goto err_free_ps;
    }    

#if 0
    /* register device attribute */
    err = device_create_file(&als_dev->dev, &dev_attr_enable_als_sensor);
    if (err) goto err_free_ps;

    err = device_create_file(&ps_dev->dev, &dev_attr_enable_ps_sensor);
    if (err) goto err_rm_als_en_attr;

    err = device_create_file(&ps_dev->dev, &dev_attr_ps_led_driver_current);
    if (err) goto err_rm_ps_en_attr;

    err = device_create_file(&als_dev->dev, &dev_attr_als_range);
    if (err) goto err_rm_ps_led_attr;

    err = device_create_file(&als_dev->dev, &dev_attr_als_mode);
    if (err) goto err_rm_als_range_attr;

    err = device_create_file(&ps_dev->dev, &dev_attr_ps_limit);
    if (err) goto err_rm_als_mode_attr;

//    err = device_create_file(&als_dev->dev, &dev_attr_poll_delay);
//    if (err) goto err_rm_ps_limit_attr;

    err = device_create_file(&als_dev->dev, &dev_attr_als_show_raw);
//    if (err) goto err_rm_poll_delay_attr;

    err = device_create_file(&ps_dev->dev, &dev_attr_ps_show_raw);
    if (err) goto err_rm_als_show_raw_attr;
#endif

    err = sysfs_create_group(&als_dev->dev.kobj, &als_attr_grp);
    if (err) {
        dev_err(&als_dev->dev, "isl29044: device create als file failed\n");
        goto err_free_ps;
    }

    err = sysfs_create_group(&ps_dev->dev.kobj, &ps_attr_grp);
    if (err) {
        dev_err(&ps_dev->dev, "isl29044: device create ps file failed\n");
        goto err_remove_als_attr;
    }

    dev_dat->als_input_dev = als_dev;
    dev_dat->ps_input_dev = ps_dev;
    
#ifdef CONFIG_HUAWEI_SENSORS_INPUT_INFO
    err = set_sensor_chip_info(ALS, "Intersil ISl29044 als");
    if (err) 
    {
        dev_err(&als_dev->dev, "set_sensor_chip_info error\n");
    }
    err = set_sensor_input(ALS, dev_dat->als_input_dev->dev.kobj.name);
    if (err) 
    {
        dev_err(&als_dev->dev, "%s,set_sensor_input als error: %s\n",
               __func__, dev_dat->als_input_dev->name);
        goto err_remove_ps_attr;
    }
    
    err = set_sensor_chip_info(PS, "Intersil ISl29044 ps");
    if (err) 
    {
        dev_err(&ps_dev->dev, "set_sensor_chip_info error\n");
    }
    err = set_sensor_input(PS, dev_dat->ps_input_dev->dev.kobj.name);
    if (err) 
    {
        dev_err(&ps_dev->dev, "%s,set_sensor_input ps error: %s\n",
               __func__, dev_dat->ps_input_dev->name);
        goto err_remove_ps_attr;
    }
#endif

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
    
    return 0;

#if 0
err_rm_als_show_raw_attr:
    device_remove_file(&als_dev->dev, &dev_attr_als_show_raw);
//err_rm_poll_delay_attr:
//    device_remove_file(&als_dev->dev, &dev_attr_poll_delay);
err_rm_ps_limit_attr:
    device_remove_file(&ps_dev->dev, &dev_attr_ps_limit);
err_rm_als_mode_attr:
    device_remove_file(&als_dev->dev, &dev_attr_als_mode);
err_rm_als_range_attr:
    device_remove_file(&als_dev->dev, &dev_attr_als_range);
err_rm_ps_led_attr:
    device_remove_file(&ps_dev->dev, &dev_attr_ps_led_driver_current);
err_rm_ps_en_attr:
    device_remove_file(&ps_dev->dev, &dev_attr_enable_ps_sensor);
err_rm_als_en_attr:
    device_remove_file(&als_dev->dev, &dev_attr_enable_als_sensor);
#endif

err_remove_ps_attr:
    sysfs_remove_group(&ps_dev->dev.kobj, &ps_attr_grp);
err_remove_als_attr:
    sysfs_remove_group(&als_dev->dev.kobj, &als_attr_grp);

err_free_ps:
    input_free_device(ps_dev);
err_free_als:
    input_free_device(als_dev);

    return err;
}
#endif

/* Return 0 if detection is successful, -ENODEV otherwise */
static int isl29044_i2c_detect(struct i2c_client *client,
    struct i2c_board_info *info)
{
    struct i2c_adapter *adapter = client->adapter;

    printk(KERN_DEBUG "In isl29044_detect()\n");
    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
                     | I2C_FUNC_SMBUS_READ_BYTE))
    {
        printk(KERN_WARNING "I2c adapter don't support ISL29044\n");
        return -ENODEV;
    }

    /* probe that if isl29044 is at the i2 address */
    if (i2c_smbus_xfer(adapter, client->addr, 0,I2C_SMBUS_WRITE,
        0,I2C_SMBUS_QUICK,NULL) < 0)
        return -ENODEV;

    strlcpy(info->type, "isl29044", I2C_NAME_SIZE);
    printk(KERN_INFO "%s is found at i2c device address %d\n", 
        info->type, client->addr);

    return 0;
}

int isl_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct isl29044_data_t *obj = (struct isl29044_data_t *)self;
    ALS_LOG("isl_als_operate cmd :%d\n",command);
    

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
                    ALS_LOG("isl_als_operate SENSOR_ENABLE,enable ");
                    
                    if(err = set_als_pwr_st(1,obj ))
                    {
                        APS_ERR("enable als fail: %d\n", err); 
                        return -1;
                    }
                    set_bit(CMC_BIT_ALS, &obj->enable);
                    
                }
                else
                {
                    ALS_LOG("isl_als_operate SENSOR_ENABLE,disable ");
                    if(err = set_als_pwr_st(0, obj))
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
                
                if(0 != obj->hw->polling_mode_als)
                {
                    isl_read_als(obj);
                }
                
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = obj->als_value;
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

int isl_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct isl29044_data_t *obj = (struct isl29044_data_t *)self;
    printk("isl_ps_operate cmd :%d\n",command);
    

    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                PS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
                    value = *(int *)buff_in;
                    PS_LOG("isl_ps_operate,SENSOR_DELAY=%d\n",value);
            // Do nothing
            break;

        case SENSOR_ENABLE:
            
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                PS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;                
                if(value)
                {
                    PS_LOG("isl_ps_operate SENSOR_ENABLE,enable ");
                    
                    if(err = set_ps_pwr_st(1,obj ))
                    {
                        PS_ERR("enable als fail: %d\n", err); 
                        return -1;
                    }
                    set_bit(CMC_BIT_PS, &obj->enable);
                    
                }
                else
                {
                    PS_LOG("isl_ps_operate SENSOR_ENABLE,disable ");
                    if(err = set_ps_pwr_st(0, obj))
                    {
                        PS_ERR("disable als fail: %d\n", err); 
                        return -1;
                    }
                    clear_bit(CMC_BIT_PS, &obj->enable);
                }
                
            }
            break;

        case SENSOR_GET_DATA:
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                PS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                
                if(0 != obj->hw->polling_mode_ps)
                {
                    isl_read_ps(obj);
                          } 
                
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = obj->ps_value;
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                
            }
            break;
        default:
            PS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }
    
    return err;
}

static void isl_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
    
       struct isl29044_data_t *dat= container_of(h, struct isl29044_data_t, early_drv);
       int err;
    APS_FUN();
       printk("isl_early_suspend,dat->enable=%d\n",dat->enable);
       #if 0
    spin_lock(&dat->als_timer_lock);
    dat->als_pwr_before_suspend = dat->als_pwr_status;
    spin_unlock(&dat->als_timer_lock);
       
    spin_lock(&dat->ps_timer_lock);
    dat->ps_pwr_before_suspend = dat->ps_pwr_status;
    spin_unlock(&dat->ps_timer_lock);
      #endif
      if(test_bit(CMC_BIT_PS, &dat->enable))
    {
        return;
    }
    err = set_als_pwr_st(0, dat);
    if(err < 0) return ;
    err = set_ps_pwr_st(0, dat);
    if(err < 0) return ;

    return ;
}
/*----------------------------------------------------------------------------*/
static void isl_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
    struct isl29044_data_t *dat= container_of(h, struct isl29044_data_t, early_drv);         
    int err;
      //printk(KERN_DEBUG "in isl29044_resume()\n");
      printk("isl_late_resume,dat->enable=%d\n",dat->enable);
      if(test_bit(CMC_BIT_ALS, &dat->enable))
      {
        err = set_als_pwr_st(1, dat);
       if(err < 0) return err;
      }
      
    if(test_bit(CMC_BIT_PS, &dat->enable))
      {   
        err = set_ps_pwr_st(1, dat);
        if(err < 0) return err;
      }
    
    return;
}
/******************************************************************************* 
  Function Configuration
******************************************************************************/
static int isl29044_open(struct inode *inode, struct file *file)
{
    file->private_data = &isl29044_data;

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int isl29044_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long isl29044_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    //struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct isl29044_data_t *obj =&isl29044_data;
    long err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    int ps_result;

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
                PS_LOG("isl_ps_operate SENSOR_ENABLE,enable ");
                    
                if(err = set_ps_pwr_st(1,obj ))
                {
                    APS_ERR("enable als fail: %d\n", err); 
                    return -1;
                }
                    //set_bit(CMC_BIT_ALS, &obj->enable);
                    
            }
            else
            {
                PS_LOG("isl_ps_operate SENSOR_ENABLE,disable ");
                if(err = set_ps_pwr_st(0, obj))
                {
                    APS_ERR("disable als fail: %d\n", err); 
                    return -1;
                }
                    //clear_bit(CMC_BIT_ALS, &obj->enable);
            }
                
            
        break;

        case ALSPS_GET_PS_MODE:
            enable = obj->ps_pwr_status;
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA:
                   if(0 != obj->hw->polling_mode_ps)
              {
                isl_read_ps(obj);
              }
                   
            if (copy_to_user(ptr, &obj->ps_value, sizeof(obj->ps_value)))
              {
                err = -EFAULT;
                goto err_out;
             }
            break;

        case ALSPS_GET_PS_RAW_DATA:    
            if(0 != obj->hw->polling_mode_ps)
              {
                isl_read_ps(obj);
              }
                   
            if (copy_to_user(ptr, &obj->ps_value, sizeof(obj->ps_value)))
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
                ALS_LOG("isl_als_operate SENSOR_ENABLE,enable ");
                    
                if(err = set_als_pwr_st(1,obj ))
                {
                    APS_ERR("enable als fail: %d\n", err); 
                    return -1;
                }
                    //set_bit(CMC_BIT_ALS, &obj->enable);
                    
                }
                else
                {
                    ALS_LOG("isl_als_operate SENSOR_ENABLE,disable ");
                    if(err = set_als_pwr_st(0, obj))
                    {
                        APS_ERR("disable als fail: %d\n", err); 
                        return -1;
                    }
                    //clear_bit(CMC_BIT_ALS, &obj->enable);
                }
            break;

        case ALSPS_GET_ALS_MODE:
            enable = obj->als_pwr_status;
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA: 
            if (copy_to_user(ptr, &obj->als_value, sizeof(obj->als_value)))
             {  
               err = -EFAULT;
                goto err_out;
             }           
            break;

        case ALSPS_GET_ALS_RAW_DATA:    
            if (copy_to_user(ptr, &obj->als_value, sizeof(obj->als_value)))
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
static struct file_operations isl29044_fops = {
    .owner = THIS_MODULE,
    .open = isl29044_open,
    .release = isl29044_release,
    .unlocked_ioctl = isl29044_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice isl29044_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &isl29044_fops,
};

/* isl29044 probed */
static int isl29044_i2c_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    int err, i;
    u8 reg_dat[8],chip_id=0;
    int ret = 0;
    struct hwmsen_object obj_ps, obj_als;
    printk("In isl29044_probe.....(), addr = %x\n", client->addr);
    
    /* initial device data struct */
    isl29044_data.client = client;
    isl29044_data.als_pwr_status = 0;
    isl29044_data.ps_pwr_status = 0;
    isl29044_data.ps_led_drv_cur = 1;
    isl29044_data.als_range = 1;
    isl29044_data.als_mode = 0;
    isl29044_data.ps_lt = PS_THRESHOLD_L;
    isl29044_data.ps_ht = PS_THRESHOLD_H;
    isl29044_data.poll_delay = 200;
    isl29044_data.show_als_raw = 0;
    isl29044_data.show_ps_raw = 0;
    isl29044_data.als_ir_cnt_set = 1;
    isl29044_data.ps_init = 0;
       isl29044_data.ps_value=1;
       isl29044_data.enable = 0;
    
    spin_lock_init(&isl29044_data.als_timer_lock);
    spin_lock_init(&isl29044_data.ps_timer_lock);

       #ifndef MTK_PLATFORM_ISL
    INIT_WORK(&isl29044_data.als_work, &do_als_work);
    INIT_WORK(&isl29044_data.ps_work, &do_ps_work);
    init_timer(&isl29044_data.als_timer);
    init_timer(&isl29044_data.ps_timer);
    isl29044_data.als_wq = create_workqueue("als wq");
    if(!isl29044_data.als_wq) 
    {
        err= -ENOMEM;
        printk(KERN_ERR "isl29044 probe() err 0, err = %d\n", err);
        return err;
    }

    isl29044_data.ps_wq = create_workqueue("ps wq");
    if(!isl29044_data.ps_wq) 
    {
        destroy_workqueue(isl29044_data.als_wq);
        err= -ENOMEM;
        printk(KERN_ERR "isl29044 probe() err 1, err = %d\n", err);
        return err;
    }
    #endif

      #ifdef MTK_PLATFORM_ISL 
      INIT_WORK(&isl29044_data.ps_work, &isl_read_ps_func);
      init_timer(&isl29044_data.ps_timer);
      isl29044_data.ps_wq = create_workqueue("ps wq");
      if(!isl29044_data.ps_wq) 
    {
        err= -ENOMEM;
        isl29044_init_flag = -1;
        printk(KERN_ERR "isl29044 probe() err 0, err = %d\n", err);
        return err;
    }
      #endif
      
    i2c_set_clientdata(client,&isl29044_data);
    
    isl29044_data.hw = get_isl_cust_alsps_hw();

    ret = i2c_smbus_read_byte_data(client, 0x00);
    if(ret < 0)
    {
        printk("isl29044 i2c_smbus read error ret=%d\n", ret);
        isl29044_init_flag = -1;
        return err;
    }
    else
    {
        chip_id = ret >> 3 ; // Only use 5-bit MSBs for Chip ID
        printk("isl29044 chip_id is %x ok\n", chip_id);
    }    
   
    /* initial isl29044 */
    err = set_sensor_reg(&isl29044_data);
    if(err < 0) return err;
    
    /* initial als interrupt limit to low = 0, high = 4095, so als cannot
       trigger a interrupt. We use ps interrupt only */
    reg_dat[5] = 0x00;
    reg_dat[6] = 0xf0;
    reg_dat[7] = 0xff;
    for(i = 5; i <= 7; i++)
    {
        err = i2c_smbus_write_byte_data(client, i, reg_dat[i]);
        if(err < 0) 
        {
            printk(KERN_ERR "isl29044 probe() err 2, err = %d\n", err);
            isl29044_init_flag = -1;
            return err;
        }
    }
    
    //err = sysfs_create_group(&client->dev.kobj, &apds990x_attr_group);
    /* Add input device register here */
    #if 0
    err = init_input_dev(&isl29044_data);
       
    if(err < 0)
    {
        printk(KERN_ERR "isl29044 probe() err 3, err = %d\n", err);
        destroy_workqueue(isl29044_data.als_wq);
        destroy_workqueue(isl29044_data.ps_wq);
        return err;
    }
    #endif
    
    if(err = misc_register(&isl29044_device))
    {
        APS_ERR("isl29044_i2c_probe reseter misc error\n");
        isl29044_init_flag = -1;
        return err;
    }
       
    obj_ps.self = &isl29044_data;
    obj_ps.polling = 0;
    obj_ps.sensor_operate = isl_ps_operate;
    if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("attach fail = %d\n", err);
        isl29044_init_flag = -1;
        return err;
    }
    
    obj_als.self = &isl29044_data;
    obj_als.polling = 1;
    obj_als.sensor_operate = isl_als_operate;
    if(err = hwmsen_attach(ID_LIGHT, &obj_als))
    {
        APS_ERR("attach fail = %d\n", err);
        isl29044_init_flag = -1;
        return err;
    }
       
    #if defined(CONFIG_HAS_EARLYSUSPEND)
    isl29044_data.early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    isl29044_data.early_drv.suspend  = isl_early_suspend,
    isl29044_data.early_drv.resume   = isl_late_resume,    
    register_early_suspend(&isl29044_data.early_drv);
    #endif
    if(err < 0)
    {
        printk("isl29044_i2c_probe fail..\n");
        isl29044_init_flag = -1;
    }
    else
    {
        printk("isl29044_i2c_probe sucess..\n");
        isl29044_init_flag = 0;
    }
    return err;
}

static int isl29044_i2c_remove(struct i2c_client *client)
{
    //struct input_dev *als_dev;
    //struct input_dev *ps_dev;

    printk(KERN_INFO "%s at address %d is removed\n",client->name,client->addr);

    /* clean the isl29044 data struct when isl29044 device remove */
    isl29044_data.client = NULL;
    isl29044_data.als_pwr_status = 0;
    isl29044_data.ps_pwr_status = 0;

    //als_dev = isl29044_data.als_input_dev;
    //ps_dev = isl29044_data.ps_input_dev;

#if 0
    device_remove_file(&ps_dev->dev, &dev_attr_ps_show_raw);
    device_remove_file(&als_dev->dev, &dev_attr_als_show_raw);
//    device_remove_file(&als_dev->dev, &dev_attr_poll_delay);
    device_remove_file(&ps_dev->dev, &dev_attr_ps_limit);
    device_remove_file(&als_dev->dev, &dev_attr_als_mode);
    device_remove_file(&als_dev->dev, &dev_attr_als_range);
    device_remove_file(&ps_dev->dev, &dev_attr_ps_led_driver_current);
    device_remove_file(&als_dev->dev, &dev_attr_enable_als_sensor);
    device_remove_file(&ps_dev->dev, &dev_attr_enable_ps_sensor);


    sysfs_remove_group(&als_dev->dev.kobj, &als_attr_grp);
    sysfs_remove_group(&ps_dev->dev.kobj, &ps_attr_grp);

    input_unregister_device(als_dev);
    input_unregister_device(ps_dev);
    
    destroy_workqueue(isl29044_data.ps_wq);
    destroy_workqueue(isl29044_data.als_wq);

    isl29044_data.als_input_dev = NULL;
    isl29044_data.ps_input_dev = NULL;
#endif
    return 0;
}

#ifdef CONFIG_PM    
/* if define power manager, define suspend and resume function */
static int isl29044_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct isl29044_data_t *dat;
    int ret;

    //printk(KERN_DEBUG "in isl29044_suspend()\n");
    dat = i2c_get_clientdata(client);
    
    spin_lock(&dat->als_timer_lock);
    dat->als_pwr_before_suspend = dat->als_pwr_status;
    spin_unlock(&dat->als_timer_lock);
    ret = set_als_pwr_st(0, dat);
    if(ret < 0) return ret;
    
    spin_lock(&dat->ps_timer_lock);
    dat->ps_pwr_before_suspend = dat->ps_pwr_status;
    spin_unlock(&dat->ps_timer_lock);
    ret = set_ps_pwr_st(0, dat);
    if(ret < 0) return ret;

    return 0;
}

static int isl29044_resume(struct i2c_client *client)
{
    struct isl29044_data_t *dat;
    int ret;

    //printk(KERN_DEBUG "in isl29044_resume()\n");
    dat = i2c_get_clientdata(client);

    ret = set_als_pwr_st(dat->als_pwr_before_suspend, dat);
    if(ret < 0) return ret;
    
    ret = set_ps_pwr_st(dat->ps_pwr_before_suspend, dat);
    if(ret < 0) return ret;
    
    return 0;
}
#else
#define    isl29044_suspend     NULL
#define isl29044_resume        NULL
#endif        /*ifdef CONFIG_PM end*/

static const struct i2c_device_id isl29044_id[] = {
    { "isl29044", 0 },
    { }
};

static struct i2c_driver isl29044_driver = {
    .driver = {
        .name    = "isl29044",
    },
    .probe            = isl29044_i2c_probe,
    .remove            = isl29044_i2c_remove,
    .id_table        = isl29044_id,
    .detect            = isl29044_i2c_detect,
    .address_list    = normal_i2c,
    .suspend        = NULL,
    .resume            = NULL
};

struct i2c_client *isl29044_client;
#if 0
static int ISl29044_probe(struct platform_device *pdev) 
{
      printk("ISl29044_probe,init\n");
    if(i2c_add_driver(&isl29044_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    } 
    return 0;
}

static int ISl29044_remove(struct platform_device *pdev)
{
    i2c_del_driver(&isl29044_driver);
    return 0;
}
static struct platform_driver ISl29044_alsps_driver = {
    .probe      = ISl29044_probe,
    .remove     = ISl29044_remove,    
    .driver     = {
        .name  = "als_ps",
//        .owner = THIS_MODULE,
    }
};
#endif

/*----------------------------------------------------------------------------*/
static int isl29044_remove(void)
{
    struct alsps_hw *hw = get_isl_cust_alsps_hw();
    APS_FUN();

    i2c_del_driver(&isl29044_driver);
    return 0;
}

static int  isl29044_local_init(void)
{
    APS_FUN();

    if(i2c_add_driver(&isl29044_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    } 
    printk("y00187129 isl29044 flag=%d\n", isl29044_init_flag);
    if(-1 == isl29044_init_flag)
    {
       return -1;
    }
    
    return 0;
}
/*----------------------------------------------------------------------------*/
static int __init isl29044_init(void)
{
    int ret;
    struct alsps_hw *hw = get_isl_cust_alsps_hw();
    APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);

    i2c_register_board_info(2, &i2c_ISl29044, 1);
    APS_LOG("i2c_register_board_info");
    hwmsen_alsps_add(&isl29044_init_info);
    /*
    if(platform_driver_register(&ISl29044_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
    */
      return 0;
}

static void __exit isl29044_exit(void)
{
    printk("exit isl29044 module\n");
    //i2c_del_driver(&isl29044_driver);
}

MODULE_AUTHOR("Chen Shouxian");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("isl29044 ambient light sensor driver");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl29044_init);
module_exit(isl29044_exit);
