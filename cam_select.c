/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#include <math.h>
#include "cam_select.h"

 // x^2 + y^2 + z^2
inline static float norm(float *sphere)
{
    return sqrtf(sphere[0] * sphere[0] + sphere[1] * sphere[1] + sphere[2] * sphere[2]);
}

 // New algorithm, updated in Aug, 2020.
void selectTwoView_new(vss_param* params, cam_param* vcam, int* camID, float* dist)
{
    int cam_1_number = -1;
    int cam_2_number = -1;

    float dist1 = 0, dist2 = 0;


    float MIN_dist = 1000000000000.f;

    for (int i = 0; i < params->cam_num; i++)  //select first camera
    {
        if (i == 10) continue;  //for debug, should be deleted, since for debug we use camera 1 as vitual camera  
        float pos_diff_[3] =
        {
            vcam->krt_WorldPosition[0] - params->krt_rcam[i].krt_WorldPosition[0],
            vcam->krt_WorldPosition[1] - params->krt_rcam[i].krt_WorldPosition[1],
            vcam->krt_WorldPosition[2] - params->krt_rcam[i].krt_WorldPosition[2]
        };
        dist1 = norm(pos_diff_);
        if (dist1 <= MIN_dist)
        {
            MIN_dist = dist1;
            cam_1_number = i;
        }
    }
    dist1 = MIN_dist;
    MIN_dist = 1000000000000.f;

    for (int i = 0; i < params->cam_num; i++)  //select second camera
    {
        if (i == 10) continue;  //for debug, should delete, since for debug we use camera 1 as vitual camera
        if (i == cam_1_number) continue;
        float vector_VcamRcam1[3] =
        {
            vcam->krt_WorldPosition[0] - params->krt_rcam[cam_1_number].krt_WorldPosition[0],
            vcam->krt_WorldPosition[1] - params->krt_rcam[cam_1_number].krt_WorldPosition[1],
            vcam->krt_WorldPosition[2] - params->krt_rcam[cam_1_number].krt_WorldPosition[2]
        };
        float vector_Rcam2Rcam1[3] =
        {
            params->krt_rcam[i].krt_WorldPosition[0] - params->krt_rcam[cam_1_number].krt_WorldPosition[0],
            params->krt_rcam[i].krt_WorldPosition[1] - params->krt_rcam[cam_1_number].krt_WorldPosition[1],
            params->krt_rcam[i].krt_WorldPosition[2] - params->krt_rcam[cam_1_number].krt_WorldPosition[2]
        };
        float product1 = vector_VcamRcam1[0] * vector_Rcam2Rcam1[0] + vector_VcamRcam1[1] * vector_Rcam2Rcam1[1] + vector_VcamRcam1[2] * vector_Rcam2Rcam1[2];
        if (product1 < 0) continue;
        float vector_Rcam2Vcam[3] =
        {
            params->krt_rcam[i].krt_WorldPosition[0] - vcam->krt_WorldPosition[0],
            params->krt_rcam[i].krt_WorldPosition[1] - vcam->krt_WorldPosition[1],
            params->krt_rcam[i].krt_WorldPosition[2] - vcam->krt_WorldPosition[2]
        };
        float product2 = vector_Rcam2Vcam[0] * vector_Rcam2Rcam1[0] + vector_Rcam2Vcam[1] * vector_Rcam2Rcam1[1] + vector_Rcam2Vcam[2] * vector_Rcam2Rcam1[2];
        if (product2 < 0) continue;
        dist2 = norm(vector_Rcam2Vcam);
        if (dist2 < MIN_dist)
        {
            MIN_dist = dist2;
            cam_2_number = i;
        }
        if (cam_2_number == -1 && i == params->cam_num - 1)
        {
            cam_2_number = cam_1_number;
            dist2 = dist1;
        }
    }
    dist2 = MIN_dist;

    dist[0] = dist1;

    dist[1] = dist2;

    camID[0] = cam_1_number;

    camID[1] = cam_2_number;
}

void selectTwoView(vss_param* params, cam_param* vcam, int* camID, float* dist, float*test, int *flag)
{
    int cam_1_number = -1;
    int cam_2_number = -1;

    float MIN_dist = 1000000000000.f;

    for (int i = 0; i < params->cam_num; i++)  //select first camera
    {
        //if (i == 7 || i == 8 || i==6 || i==9 ) continue;  //for debug, should be deleted, since for debug we use camera 1 as vitual camera  
        float pos_diff_[3] =
        {
            vcam->krt_WorldPosition[0] - params->krt_rcam[i].krt_WorldPosition[0],
            vcam->krt_WorldPosition[1] - params->krt_rcam[i].krt_WorldPosition[1],
            vcam->krt_WorldPosition[2] - params->krt_rcam[i].krt_WorldPosition[2]
        };
        float dist = norm(pos_diff_);
        if (dist <= MIN_dist)
        {
            MIN_dist = dist;
            cam_1_number = i;
        }
    }
    float d1 = MIN_dist;

    MIN_dist = 1000000000000.f;
    for (int i = 0; i < params->cam_num; i++)  //select second camera
    {
        if (i == cam_1_number) continue;
        float pos_diff_[3] =
        {
            vcam->krt_WorldPosition[0] - params->krt_rcam[i].krt_WorldPosition[0],
            vcam->krt_WorldPosition[1] - params->krt_rcam[i].krt_WorldPosition[1],
            vcam->krt_WorldPosition[2] - params->krt_rcam[i].krt_WorldPosition[2]
        };
        float dist = norm(pos_diff_);

        float vector_2_1[3] = {
            params->krt_rcam[i].krt_WorldPosition[0] - params->krt_rcam[cam_1_number].krt_WorldPosition[0],
            params->krt_rcam[i].krt_WorldPosition[1] - params->krt_rcam[cam_1_number].krt_WorldPosition[1],
            params->krt_rcam[i].krt_WorldPosition[2] - params->krt_rcam[cam_1_number].krt_WorldPosition[2]
        };
        float dist_2 = norm(vector_2_1);

        float vector_0_1[3] = {
            vcam->krt_WorldPosition[0] - params->krt_rcam[cam_1_number].krt_WorldPosition[0],
            vcam->krt_WorldPosition[1] - params->krt_rcam[cam_1_number].krt_WorldPosition[1],
            vcam->krt_WorldPosition[2] - params->krt_rcam[cam_1_number].krt_WorldPosition[2]
        };
        float dot_mul = vector_2_1[0] * vector_0_1[0] + vector_2_1[1] * vector_0_1[1] + vector_2_1[2] * vector_0_1[2];
        float intersection_point[3];
        intersection_point[0] = dot_mul / (dist_2 * dist_2) * vector_2_1[0] + params->krt_rcam[cam_1_number].krt_WorldPosition[0];
        intersection_point[1] = dot_mul / (dist_2 * dist_2) * vector_2_1[1] + params->krt_rcam[cam_1_number].krt_WorldPosition[1];
        intersection_point[2] = dot_mul / (dist_2 * dist_2) * vector_2_1[2] + params->krt_rcam[cam_1_number].krt_WorldPosition[2];

        float pos_diff_0[3] = {
            intersection_point[0] - params->krt_rcam[cam_1_number].krt_WorldPosition[0],
            intersection_point[1] - params->krt_rcam[cam_1_number].krt_WorldPosition[1],
            intersection_point[2] - params->krt_rcam[cam_1_number].krt_WorldPosition[2]
        };
        float dist_0 = norm(pos_diff_0);

        float pos_diff_1[3] = {
            intersection_point[0] - params->krt_rcam[i].krt_WorldPosition[0],
            intersection_point[1] - params->krt_rcam[i].krt_WorldPosition[1],
            intersection_point[2] - params->krt_rcam[i].krt_WorldPosition[2]
        };

        float dist_1 = norm(pos_diff_1);

        test[0] = dist_0; test[1] = dist_1; test[2] = dist_2;//test

        if (fabs(dist_0 + dist_1 - dist_2) < 0.00001f) {
            if (dist <= MIN_dist) {
                MIN_dist = dist;
                cam_2_number = i;
                //break;
            }

        }
        else if (i == params->cam_num - 1) {
            *flag = 1;
            cam_2_number = 0;
        }
    }

    float d2 = MIN_dist;

    camID[0] = cam_1_number;
    camID[1] = cam_2_number;
    dist[0] = d1; dist[1] = d2;

}