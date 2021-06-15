/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/
#include <stdlib.h>
#include "vss.h"
#include "filter.h"

int vss_open(vss_view* views, vss_param* param)
{
    init_lanczos_filter();
    for (int i = 0; i <= param->cam_num; i++) //The last file list is a virtual view 
    {
        views[i].texture = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
        if (!views[i].texture)
        {
            printf("malloc texture failed!\n");
            return -1;
        }

        if (param->depth_scale_type)
        {
            views[i].texture_y = (unsigned char *)malloc(param->source_width * param->source_height * sizeof(unsigned char));
            if (!views[i].texture_y)
            {
                printf("malloc texture failed!\n");
                return -1;
            }
        }
        if (param->depth_scale_type || param->depth_mf_radius)
        {
            views[i].depth_y = (unsigned char *)malloc(param->source_width * param->source_height * sizeof(unsigned char));
            if (!views[i].depth_y)
            {
                printf("malloc texture failed!\n");
                return -1;
            }
        }
        views[i].depth = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
        if (!views[i].depth)
        {
            printf("malloc depth failed!\n");
            return -1;
        }
        views[i].z_depth = (float *)malloc(param->source_width * param->source_height * 3 * sizeof(float));
        if (!views[i].z_depth)
        {
            printf("malloc z_depth failed!\n");
            return -1;
        }
        if (param->background_flag)
        {
            views[i].texture_bg = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
            if (!views[i].texture_bg)
            {
                printf("malloc texture_bg failed!\n");
                return -1;
            }

            views[i].depth_bg = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
            if (!views[i].depth_bg)
            {
                printf("malloc depth_bg failed!\n");
                return -1;
            }
        }
    }
    return 0;
}
int vss_close(vss_view* views, vss_param* param)
{
    for (int i = 0; i <= param->cam_num; i++)
    {
        if (views[i].texture)
        {
            free(views[i].texture);
            views[i].texture = NULL;
        }

        if (param->depth_scale_type)
        {
            if (views[i].texture_y)
            {
                free(views[i].texture_y);
                views[i].texture_y = NULL;
            }
        }

        if (param->depth_scale_type || param->depth_mf_radius)
        {
            if (views[i].depth_y)
            {
                free(views[i].depth_y);
                views[i].depth_y = NULL;
            }
        }

        if (views[i].depth)
        {
            free(views[i].depth);
            views[i].depth = NULL;
        }
        if (views[i].z_depth)
        {
            free(views[i].z_depth);
            views[i].z_depth = NULL;
        }
        if (param->background_flag)
        {
            if(views[i].texture_bg)
            {
                free(views[i].texture_bg);
                views[i].texture_bg = NULL;
            }

            if(views[i].depth_bg)
            {
                free(views[i].depth_bg);
                views[i].depth_bg = NULL;
            }
        }
    }
    return 0;
}