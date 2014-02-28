#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_ltr_alsps_hw = {
    .i2c_num    = 2,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    /*Lenovo-sw chenlj2 add 2011-06-03,modify parameter below two lines*/
    .als_level  = {42,161,281,520,830,1140,1450},
    .als_value  = {10,255,320,640,1280,2600,10240},
    .ps_threshold_high = 300,
    .ps_threshold_low = 200,
    .ps_threshold = 0,
};
struct alsps_hw *get_ltr_cust_alsps_hw(void) {
    return &cust_ltr_alsps_hw;
}
