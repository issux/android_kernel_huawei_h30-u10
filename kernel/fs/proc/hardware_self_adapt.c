/******************************************************************************

  Copyright (C), 2011-2015, Huawei Tech. Co., Ltd.

 ******************************************************************************
  File Name     : hardware_self_adapt.c
  Description   : In LK get the hardware version info from RAM
******************************************************************************/


#include <linux/hardware_self_adapt.h>
#include <linux/kernel.h>
#include <linux/module.h>


/******************************************************************************
  Function:       get_hardware_product_version
  Description:    get the hardware_product_version
  Input:          void
  Output:         None
  Return:         None
  Others:         None
******************************************************************************/
hw_product_type get_hardware_product_version( void )
{
    return hw_product_info.board_id;
}


/******************************************************************************
  Function:       get_hardware_battery_type
  Description:    Get battery id
  Input:          null
  Output:         null
  Return:         battery id
  Others:         null
******************************************************************************/
hw_battery_type get_hardware_battery_type( void )
{
    //get lowwer bytes
    return (hw_product_info.battery_id & 0x0000FFFF);
}

int get_battery_id_adc_value(void)
{
    //get higher bytes
    return (hw_product_info.battery_id & 0xFFFF0000) >> 16;
}
#ifdef CONFIG_DL_CHECK_SUPPORT
hw_dl_check_tag get_hardware_dl_check_tag( void )
{
    return hw_product_info.dl_check_tag;
}
#endif
