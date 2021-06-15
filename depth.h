/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#ifndef __VSS_DEPTH_H__
#define __VSS_DEPTH_H__

#include"vss.h"

void depth_pro_background(vss_view* view, vss_param* param);
void get_real_depth(vss_view* view, vss_param* param);
int  get_up_sample_depth(vss_view* view, vss_param* param);

#endif