
#ifndef __ISL29044_H__
#define __ISL29044_H__
#define ISL29044_ADDR	0x44
#define	ISL_DEVICE_NAME		"isl29044"

struct isl29044_hw_platform_data {
	int (*power_on)(struct device*);
	int (*power_off)(void);
    //int (*isl29044_gpio_config_interrupt)(void);
};
#endif

