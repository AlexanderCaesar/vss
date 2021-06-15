/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "depth.h"

//Depth map processing based on background frame
void depth_pro_background(vss_view* view, vss_param* param)
{
    for (int h = 0; h < param->source_height; h++)
    {
        for (int w = 0; w < param->source_width; w++)
        {
            if (abs(view->texture_bg[h * param->source_width * 3 + w * 3 + 0] - view->texture[h * param->source_width * 3 + w * 3 + 0]) < param->background_th     //R
                && abs(view->texture_bg[h * param->source_width * 3 + w * 3 + 1] - view->texture[h * param->source_width * 3 + w * 3 + 1]) < param->background_th  //G
                && abs(view->texture_bg[h * param->source_width * 3 + w * 3 + 2] - view->texture[h * param->source_width * 3 + w * 3 + 2]) < param->background_th) //B
            {
                view->depth[h * param->source_width + w] = view->depth_bg[h * param->source_width + w];
            }
        }
    }
}

void get_real_depth(vss_view* view, vss_param* param)
{
    //float   MIN_depth_reciprocal = 1.0 / param->MIN_depth;
    //float   MAX_depth_reciprocal = 1.0 / param->MAX_depth;
    for (int h = 0; h < param->source_height; h++)
    {
        for (int w = 0; w < param->source_width; w++)
        {
            view->z_depth[h * param->source_width + w] = 1.0 / ((float)view->depth[h * param->source_width + w] / 255.0 *(1.0 / param->min_depth - 1.0 / param->max_depth) + 1.0 / param->max_depth);
        }
    }
}

void get_texture_y(vss_view* view, vss_param* param)
{
    for (int h = 0; h < param->source_height; h++)
    {
        for (int w = 0; w < param->source_width; w++)
        {
            int r = view->texture[h * param->source_width * 3 + w * 3 + 0];
            int g = view->texture[h * param->source_width * 3 + w * 3 + 1];
            int b = view->texture[h * param->source_width * 3 + w * 3 + 2];
            float y = (float)(0.2989 * r + 0.5870 * g + 0.1140 * b);
            view->texture_y[h * param->source_width + w] = (unsigned char)y;
        }
    }
}

void depth_4x(unsigned char* depth, unsigned char* buf, unsigned char* texture_y, int width, int height, float th)
{
    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
            if ((w % 2 == 0) && (h % 2 == 0))//Even rows, Even columns
            {
                buf[h * width + w] = depth[h/2 * width / 2 + w / 2];
            }
            else if ((w % 2 != 0) && (h % 2 == 0))//Even rows, Odd columns
            {
                int depthL = depth[h/2 * width/2 + w/2];
                int depthR = depth[h / 2 * width / 2 + MIN(width/2 - 1, w/2 + 1)];
                int pixC = texture_y[h * width + w];
                int pixL = texture_y[h * width + MAX(0, w - 1)];
                int pixR = texture_y[h * width + MIN(width - 1, w + 1)];
                if (abs(pixC - pixR) < abs(pixC - pixL) * th)
                    buf[h * width + w] = depthR;
                else if (abs(pixC - pixL) < abs(pixC - pixR) * th)
                    buf[h * width + w] = depthL;
                else
                    buf[h * width + w] = MAX(depthL, depthR);
            }
            else //Odd rows, Even or Odd columns
            {
                int depthU = depth[h / 2 * width / 2 + w / 2];
                int depthD = depth[MIN(height/2 - 1, h/2 + 1) * width/2 + w/2];
                int pixC = texture_y[h * width + w];
                int pixU = texture_y[MAX(0, h-1) *width + w];
                int pixD = texture_y[MIN(height -1, h + 1) * width + w];
                if (abs(pixC - pixD) < abs(pixC - pixU) * th)
                    buf[h * width + w] = depthD;
                else if (abs(pixC - pixU) < abs(pixC - pixD) *th)
                    buf[h * width + w] = depthU;
                else
                    buf[h * width + w] = MAX(depthU, depthD);
            }
        }
    }
    memcpy(depth, buf, width*height*sizeof(unsigned char));
}

void depth_16x(unsigned char* depth, unsigned char* buf, int width, int height)
{
    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
            if ((w % 2 == 0) && (h % 2 == 0))//Even rows, Even columns
            {
                buf[h * width + w] = depth[h / 2 * width / 2 + w / 2];
            }
            else if ((w % 2 != 0) && (h % 2 == 0))//Even rows, Odd columns
            {
                int depthL = depth[h / 2 * width / 2 + w / 2];
                int depthR = depth[h / 2 * width / 2 + MIN(width / 2 - 1, w / 2 + 1)];
                buf[h * width + w] = MAX(depthL, depthR);
            }
            else //Odd rows, Even or Odd columns
            {
                int depthU = depth[h / 2 * width / 2 + w / 2];
                int depthD = depth[MIN(height / 2 - 1, h / 2 + 1) * width / 2 + w / 2];
                buf[h * width + w] = MAX(depthU, depthD);
            }
        }
    }
    memcpy(depth, buf, width*height * sizeof(unsigned char));
}
int get_up_sample_depth(vss_view* view, vss_param* param)
{
    get_texture_y(view, param);
    switch (param->depth_scale_type)
    {
       case 0:return 0;
       case 1:
           depth_4x(view->depth, view->depth_y, view->texture_y, param->source_width, param->source_height, param->depth_scale_th); 
           if (param->background_flag)
           {
               depth_4x(view->depth_bg, view->depth_y, view->texture_y, param->source_width, param->source_height, param->depth_scale_th);
           }
           return 0;
       case 2:
           depth_16x(view->depth, view->depth_y,param->source_width>>1, param->source_height>>1); 
           depth_4x(view->depth, view->depth_y, view->texture_y, param->source_width, param->source_height, param->depth_scale_th);
           if (param->background_flag)
           {
               depth_16x(view->depth_bg, view->depth_y, param->source_width >> 1, param->source_height >> 1);
               depth_4x(view->depth_bg, view->depth_y, view->texture_y, param->source_width, param->source_height, param->depth_scale_th);
           }
           return 0;
       default:
           printf("unknown scale type %d\n", param->depth_scale_type);
           return -1;
    }
}
