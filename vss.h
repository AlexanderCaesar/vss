/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#ifndef __VSS_H__
#define __VSS_H__

#include<stdio.h>

#define MAX_SFM_CAMNUM (50) // default 200   -- MAXimum support sfm image number, 2K
#define CV_PI       3.14159265358979323846
#define REF_CAM_N   2    //ref cameras or frames used to interpolate

#define I2R_OK    1
#define I2R_ERR   0 

typedef enum {

    Hole,
    Main = 128,
    Foreground_boundary = 255,
    Background_boundary = 50
} pixel_label;

typedef enum {
    Fisheye = 0,
    Pinhole = 1,
    FastPinhole = 2
} Model;


typedef struct cam_param
{
    float          krt_R[9];             //parameters Rotation matrix
    float          krt_WorldPosition[3];
    float          krt_kc[3];            //Fisheye K 
    float          krt_cc[2];
    int            width;
    int            height;
    float          lens_fov;
    float          fisheye_radius; 
}cam_param;

typedef struct
{
    int            cam_num;
    int            source_width;
    int            source_height;
    float          min_depth;  
    float          max_depth;   
    int            depth_scale_type;             // 0 full 1 1/2 downsample 2 1/4 downsample in width
    int            background_flag;
    int            background_th;                //background threshold
    float          depth_scale_th;               //depth_scale threshold
    int            depth_mf_radius;              //Median filtering radius
    int            depth_edge_th;
    int            depth_edge_radius;
    int            vir_filter_type;
    int            vir_filter_radius;
    int            vir_interpolation_type;
    int            lanczos_alpha;
    float          merge_th;
    int            hole_radius;
    int            hole_th;
    int            foreground_edge_radius;

    int            left_view;
    int            right_view;

    int            model;

   

    cam_param      krt_vcam;                     // virtual  camera

    cam_param      krt_rcam[MAX_SFM_CAMNUM];     // reference cameras

} vss_param;


typedef struct vss_view
{
    unsigned char* texture;
    unsigned char* depth;
    float*         z_depth;   //real depth

    unsigned char* texture_bg;//background frame
    unsigned char* depth_bg;  //background frame

    unsigned char* texture_y;  //The luMINance component of a texture view  for down sample type
    unsigned char* depth_y;    //The luMINance component of a depth map  for down sample type

}vss_view;

int vss_open(vss_view* view, vss_param* param);
int vss_close(vss_view* views, vss_param* param);


#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

#endif