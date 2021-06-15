/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#ifndef __VSS_CAM_SELECT_H__
#define __VSS_CAM_SELECT_H__

#include "vss.h"
void selectTwoView_new(vss_param* params, cam_param* vcam, int* camID, float* dist);
void selectTwoView(vss_param* params, cam_param* vcam, int* camID, float* dist, float*test, int *flag);


#endif