#include <utils/Log.h>
#include <utils/Errors.h>
#include <fcntl.h>
#include <math.h>

#include "MediaHal.h"

//#include "lens_custom_cfg.h"
//#include "msdk_lens_exp.h"
#include "camera_custom_lens.h"
//#include "lens.h"
//nclude "image_sensor.h"
#include "kd_imgsensor.h"

extern PFUNC_GETLENSDEFAULT pDummy_getDefaultData;

#if defined(SENSORDRIVE)
extern PFUNC_GETLENSDEFAULT pSensorDrive_getDefaultData;
#endif

#if defined(FM50AF)
extern PFUNC_GETLENSDEFAULT pFM50AF_getDefaultData;
extern PFUNC_GETLENSDEFAULT pFM50AF_get_S5K4E1GA_DefaultData;
extern PFUNC_GETLENSDEFAULT pFM50AF_get_OV5648_DefaultData;

//add camera imx134 vcm para for H30/G6
extern PFUNC_GETLENSDEFAULT pFM50AF_get_IMX134LITEON_DefaultData;
extern PFUNC_GETLENSDEFAULT pFM50AF_get_IMX134SUNNY_DefaultData;

#endif


MSDK_LENS_INIT_FUNCTION_STRUCT LensList[MAX_NUM_OF_SUPPORT_LENS] =
{
	{DUMMY_SENSOR_ID, DUMMY_LENS_ID, "Dummy", pDummy_getDefaultData},

#if defined(SENSORDRIVE)
    //	{DUMMY_SENSOR_ID, SENSOR_DRIVE_LENS_ID, "kd_camera_hw", pSensorDrive_getDefaultData},	

    //  for backup lens, need assign correct SensorID
    {OV3640_SENSOR_ID, SENSOR_DRIVE_LENS_ID, "kd_camera_hw", pSensorDrive_getDefaultData},
#endif

#if defined(FM50AF)
	{DUMMY_SENSOR_ID, FM50AF_LENS_ID, "FM50AF", pFM50AF_getDefaultData},

    //add camera imx134 vcm para for H30/G6
    //add liteon and sunny IMX134 module driver and para for H30
    {IMX134LITEON_SENSOR_ID, IMX134LITEON_AF_LENS_ID, "FM50AF", pFM50AF_get_IMX134LITEON_DefaultData},
    {IMX134SUNNY_SENSOR_ID, IMX134SUNNY_AF_LENS_ID, "FM50AF", pFM50AF_get_IMX134SUNNY_DefaultData},	

#endif

    //  for new added lens, need assign correct SensorID

};

UINT32 GetLensInitFuncList(PMSDK_LENS_INIT_FUNCTION_STRUCT pLensList)
{
    memcpy(pLensList, &LensList[0], sizeof(MSDK_LENS_INIT_FUNCTION_STRUCT)* MAX_NUM_OF_SUPPORT_LENS);
    return MHAL_NO_ERROR;
} // GetLensInitFuncList()






