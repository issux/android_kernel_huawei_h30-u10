#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

extern int disp_bls_set_backlight(unsigned int level);

extern int mtkfb_set_backlight_level(unsigned int level);
#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{  
	if(level) {
		return level/32;
	}else {
		return level;
	}
}
//use LCD IC control backlight
unsigned int Cust_SetBacklight(int level, int div)
{
    printk("Cust_SetBacklight level=%d\n", level);
    mtkfb_set_backlight_level(level);
    return 0;
}
/*
unsigned int Cust_SetBacklight(int level, int div)
{
	kal_uint32 ret=0;
//    mtkfb_set_backlight_pwm(div);
//    mtkfb_set_backlight_level(brightness_mapping(level));

//	hwPWMsetting(MT65XX_PMIC_PWM_NUMBER PWMmode, kal_uint32 duty, kal_uint32 freqSelect);
//	hwBacklightBoostTuning(kal_uint32 MODE, kal_uint32 VRSEL, 0);
//	hwBacklightBoostTurnOn();
//	hwPWMsetting(0, , kal_uint32 freqSelect);
//	hwBacklightBoostTuning(kal_uint32 MODE, kal_uint32 VRSEL, 0);
//	hwBacklightBoostTurnOn();
//echo 15 13 > pmic_access_bank1
//echo 40 0A > pmic_access_bank1
//echo 3F 91 > pmic_access_bank1
//echo 2E 1F > pmic_access_bank1 
	printk("backlight temp solution, level=%d, div=%d\n", level, div);
	ret=pmic_bank1_config_interface(0x15, 0x13, 0xFF, 0x0);
	//backlight voltage
	ret=pmic_bank1_config_interface(0x40, 0x0A, 0xFF, 0x0);
	//bit0=1, enable boost backlight; bit2=0, CABC disable;bit5-bit4=01,PWM1;bit7=1,boost mode;
	ret=pmic_bank1_config_interface(0x3F, 0x91, 0xFF, 0x0);
	//PWM1 duty=32/32
	ret=pmic_bank1_config_interface(0x2E, 0x1F, 0xFF, 0x0);
    
    return 0;
}
*/


static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK3,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0x1, 0x1, 0, 0, 0}},
};
static struct cust_mt65xx_led cust_led_list_cabc[MT65XX_LED_TYPE_TOTAL] = {
    {"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
    {"green",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
    {"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
    {"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
    {"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
    {"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK3,{0}},
    {"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, (int)Cust_SetBacklight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list_cabc;
}

