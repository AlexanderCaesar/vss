/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#ifndef __VSS_FILTER_H__
#define __VSS_FILTER_H__

void  init_lanczos_filter();
float filtre_lanczos(unsigned char * src, float j, float i, int input_width, int input_height, int i_src, int RGB_offset);
void  median_filter(unsigned char * src, unsigned char * buf, int width, int height, int kradius);

#endif