/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "file.h"

#if defined WIN32
#define PATH_SEPARATOR "\\"
#else
#define PATH_SEPARATOR "/"
#endif

int vss_files_set_path(vss_files* files, vss_filepaths* path, vss_param* param)
{
    for (int i = 0; i < param->cam_num; i++)
    {
        files[i].path = path;
    }
    files[param->cam_num].path = path; //virture view
    return 0;
}

int vss_files_open(vss_files* files, vss_param* param)
{
    for (int i = 0; i < param->cam_num; i++)
    {
        char file_name[FILE_NAME_SIZE];
        sprintf(file_name, "%s%s%d.yuv", files[i].path->texture_path, PATH_SEPARATOR, i);
        files[i].texture = fopen(file_name, "rb");
        //printf("texture name %d: %s\n", i, file_name);
        if (!files[i].texture)
        {
            //printf("not found!\n");
        }

        sprintf(file_name, "%s%s%d.yuv", files[i].path->depth_path, PATH_SEPARATOR, i);
        files[i].depth = fopen(file_name, "rb");
        //printf("texture name %d: %s\n", i, file_name);
        if (!files[i].depth)
        {
            //printf("not found!\n");
        }

        if (param->background_flag)
        {
            sprintf(file_name, "%s%s%d.yuv", files[i].path->texture_path_bg, PATH_SEPARATOR, i);
            files[i].texture_bg = fopen(file_name, "rb");
            //printf("depth name %d: %s\n", i, file_name);
            if (!files[i].texture_bg)
            {
                //printf("not found!\n");
            }

            sprintf(file_name, "%s%s%d.yuv", files[i].path->depth_path_bg, PATH_SEPARATOR, i);
            files[i].depth_bg = fopen(file_name, "rb");
            //printf("depth name %d: %s\n", i, file_name);
            if (!files[i].depth_bg)
            {
                //printf("not found!\n");
            }
        }
    }

    //virture view
    files[param->cam_num].texture = fopen(files[param->cam_num].path->textrue_output_file, "wb");
    //printf("virture texture name: %s\n", files[param->cam_num].path->textrue_output_file);
    if (!files[param->cam_num].texture)
    {
        printf("not found!\n");
    }

    files[param->cam_num].depth = fopen(files[param->cam_num].path->depth_output_file, "wb");
    //printf("virture depth name: %s\n", files[param->cam_num].path->depth_output_file);
    if (!files[param->cam_num].depth)
    {
        printf("not found!\n");
    }
    return 0;
}
int vss_files_close(vss_files* files, vss_param* param)
{
    for (int i = 0; i < param->cam_num; i++)
    {
        if (files[i].texture)
        {
            fclose(files[i].texture);
            files[i].texture = NULL;
        }

        if (files[i].depth)
        {
            fclose (files[i].depth);
            files[i].depth= NULL;
        }

        if (param->background_flag)
        {
            if (files[i].texture_bg)
            {
                fclose(files[i].texture_bg);
                files[i].texture_bg = NULL;
            }
            if (files[i].depth_bg)
            {
                fclose(files[i].depth_bg);
                files[i].depth_bg = NULL;
            }
        }
    }
    if (files[param->cam_num].texture)
    {
        fclose(files[param->cam_num].texture);
        files[param->cam_num].texture = NULL;
    }
    if (files[param->cam_num].depth)
    {
        fclose(files[param->cam_num].depth);
        files[param->cam_num].depth = NULL;
    }
    return 0;
}

//background
int vss_read_bg_files(vss_files* files, vss_view*  views, vss_param* param, int *cam_ref)
{
    for (int i = 0; i < REF_CAM_N; i++)
    {
        int ref = cam_ref[i];
        fread(views[ref].texture_bg, 1, param->source_width * param->source_height * 3, files[ref].texture_bg);
        fread(views[ref].depth_bg, 1, (param->source_width >> param->depth_scale_type) * (param->source_height >> param->depth_scale_type) * 3 / 2, files[ref].depth_bg);
    }
    return 0;
}

int vss_read_files(vss_files* files, vss_view*  views, vss_param* param, int *cam_ref)
{
    for (int i = 0; i < REF_CAM_N; i++)
    {
        int ref = cam_ref[i];
        fread(views[ref].texture, 1, param->source_width * param->source_height * 3, files[ref].texture);
        fread(views[ref].depth, 1, (param->source_width>>param->depth_scale_type) * (param->source_height >> param->depth_scale_type)*3/2 , files[ref].depth);
    }
    return 0;
}