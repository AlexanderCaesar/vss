/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#ifndef __VSS_FILE_H__
#define __VSS_FILE_H__

#include<stdio.h>
#include"vss.h"

#define FILE_NAME_SIZE 200

typedef struct vss_filepaths
{
    char          texture_path[FILE_NAME_SIZE];        //texture files
    char          texture_path_bg[FILE_NAME_SIZE];     //texture background files
    char          depth_path[FILE_NAME_SIZE];          //depth files
    char          depth_path_bg[FILE_NAME_SIZE];          //depth background files
    char          cam_params_file[FILE_NAME_SIZE];     //total camera params, and MINdepth  MAXdepth
    char          textrue_output_file[FILE_NAME_SIZE];
    char          depth_output_file[FILE_NAME_SIZE];
}vss_filepaths;


typedef struct vss_files
{
    vss_filepaths* path;
    FILE*          texture;
    FILE*          depth;
    FILE*          texture_bg;
    FILE*          depth_bg;
}vss_files;

int    vss_files_set_path(vss_files* files, vss_filepaths* path, vss_param* param);
int    vss_files_open(vss_files* files, vss_param* param);
int    vss_files_close(vss_files* files, vss_param* param);

int vss_read_bg_files(vss_files* files, vss_view*  views, vss_param* param, int *cam_ref);
int vss_read_files(vss_files* files, vss_view*  views, vss_param* param, int *cam_ref);
#endif