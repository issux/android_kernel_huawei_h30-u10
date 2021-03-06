/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*
**
** Copyright 2008, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/
/* Merge the MTK patch ALPS01010717 that add the AF OTA function*/
#include "camera_custom_types.h"
#include "string.h"
#ifdef WIN32
#else
#include "camera_custom_nvram.h"
#endif
#include "flash_feature.h"
#include "flash_param.h"
#include "flash_tuning_custom.h"


#include <cutils/xlog.h>
#include <kd_camera_feature.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int getDefaultStrobeNVRam_sub(void* data, int* ret_size);

int getDefaultStrobeNVRam_main(void* data, int* ret_size)
{
	//static NVRAM_CAMERA_STROBE_STRUCT strobeNVRam;
	NVRAM_CAMERA_STROBE_STRUCT* p;
	p = (NVRAM_CAMERA_STROBE_STRUCT*)data;
    //Increase flashlight brightness
	static short engTab[]=
    {
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,
        100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,

    };
	//version
	p->u4Version = NVRAM_CAMERA_STROBE_FILE_VERSION;
	//eng tab
	memcpy(p->engTab.yTab, engTab, 256*sizeof(short));
	p->engTab.exp =20000;
	p->engTab.afe_gain = 1024;
	p->engTab.isp_gain = 1024;
	p->engTab.distance = 300; //mm
	//tuning para
	p->tuningPara[0].yTar = 230;  //285;  //188--->250
	p->tuningPara[0].antiIsoLevel = 5;  //-5---->5
	p->tuningPara[0].antiExpLevel = -5;
	p->tuningPara[0].antiStrobeLevel = -8; //-10; //fyx
	p->tuningPara[0].antiUnderLevel = 0;
	p->tuningPara[0].antiOverLevel = 0;
	p->tuningPara[0].foregroundLevel = 0;
	p->tuningPara[0].isRefAfDistance = 0;
	p->tuningPara[0].accuracyLevel = 2;  //-10--->0

	p->tuningPara[1].yTar = 230; //285;  //188--->250
	p->tuningPara[1].antiIsoLevel = 5;  //-5---->5
	p->tuningPara[1].antiExpLevel = -5;
	p->tuningPara[1].antiStrobeLevel = -8; //-10; //fyx
	p->tuningPara[1].antiUnderLevel = 0;
	p->tuningPara[1].antiOverLevel = 0;
	p->tuningPara[1].foregroundLevel = 0;
	p->tuningPara[1].isRefAfDistance = 0;
	p->tuningPara[1].accuracyLevel =2;  //-10--->0

	p->tuningPara[2].yTar = 230; //285;  //188--->250
	p->tuningPara[2].antiIsoLevel = 5;  //-5---->5
	p->tuningPara[2].antiExpLevel = -5;
	p->tuningPara[2].antiStrobeLevel = -8; //-10; //fyx
	p->tuningPara[2].antiUnderLevel = 0;
	p->tuningPara[2].antiOverLevel = 0;
	p->tuningPara[2].foregroundLevel = 0;
	p->tuningPara[2].isRefAfDistance = 0;
	p->tuningPara[2].accuracyLevel = 2;  //-10--->0

	p->tuningPara[3].yTar = 230; //285;  //188--->250
	p->tuningPara[3].antiIsoLevel = 5;  //-5---->5
	p->tuningPara[3].antiExpLevel = -5;
	p->tuningPara[3].antiStrobeLevel = -8; //-10; //fyx
	p->tuningPara[3].antiUnderLevel = 0;
	p->tuningPara[3].antiOverLevel = 0;
	p->tuningPara[3].foregroundLevel = 0;
	p->tuningPara[3].isRefAfDistance = 0;
	p->tuningPara[3].accuracyLevel = 2;  //-10--->0
	//is eng level used (or by firmware)
	p->isTorchEngUpdate =0;
	p->isNormaEnglUpdate =0;
	p->isLowBatEngUpdate =0;
	p->isBurstEngUpdate =0;
	//eng level
	memset(&p->engLevel, 0, sizeof(FLASH_ENG_LEVEL));

	*ret_size = sizeof(NVRAM_CAMERA_STROBE_STRUCT);
	return 0;
}
int getDefaultStrobeNVRam(int sensorType, void* data, int* ret_size)
{

	if(sensorType==(int)DUAL_CAMERA_SUB_SENSOR)
	{
		XLOGD("getDefaultStrobeNVRam ln=%d sensorId=%d",__LINE__, sensorType);
		return getDefaultStrobeNVRam_sub(data, ret_size);
	}
	else //DUAL_CAMERA_MAIN_SENSOR
	{
		XLOGD("getDefaultStrobeNVRam ln=%d sensorId=%d",__LINE__, sensorType);
		return getDefaultStrobeNVRam_main(data, ret_size);
	}
}
