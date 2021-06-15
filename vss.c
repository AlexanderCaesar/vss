/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "vss.h"
#include "file.h"
#include "parser.h"


#include "depth.h"
#include "dibr.h"


int main(int argc, char** argv)
{
    vss_param params;
    vss_view  views[MAX_SFM_CAMNUM];
    vss_files files[MAX_SFM_CAMNUM];

    if (Configure(argc, argv, &params, files) < 0)  //get patameters from configure file
    {
        printf("configure failed\n");
        return -1;
    }

    vss_files* vir_file = &files[params.cam_num]; //The last file list is a virtual view file

    if (vss_files_open(files,&params)<0)
    {
        printf("vss open file failed!\n");
        return -1;
    }

    vss_view* vir_view = &views[params.cam_num]; //The last file list is a virtual view

    if (vss_open(views, &params) < 0)
    {
        printf("vss open failed!\n");
        return -1;
    }

    int cam_ref[REF_CAM_N] = { 9, 11 };
    cam_ref[0] = params.left_view;
    cam_ref[1] = params.right_view;
    vss_read_files(files, views, &params, cam_ref);

    if (params.background_flag)
    {
        vss_read_bg_files(files, views, &params, cam_ref);
    }

    clock_t dibr_start;
    dibr_start = clock();



	novel_view_t  nvs[REF_CAM_N];
    
	int i, j;

	float prefW[2] = { 1.0, 1.0 };
	float fB = 32504.0;   //80000  90000  120000
	float MAXdisp = fB / params.min_depth;
	float MINdisp = fB / params.max_depth;

	// find two nearest camera. In fact, this selecting approach cannot work well when the input cameras are not evenly placed on a circle.
	float dist[2] = { 0.f,0.f };
	int* flag = 0;

	for (int sss = 0; sss < REF_CAM_N; sss++)
	{
		clock_t start, finish;
		start = clock();

		// 我姑且认为，这个是邻域图投影在中间图的缓冲
		novel_view_t *nv = &nvs[sss];
		nv->mrange_img = malloc(params.source_width * params.source_height * sizeof(float));                 // depth  深度图
		nv->mlabel_img = malloc(params.source_width * params.source_height * sizeof(pixel_label));   // mask?? 前景边缘标记图
		nv->mnovel_view = malloc(params.source_width * params.source_height * 3 * sizeof(unsigned char));    // rgb   纹理图
		nv->mconf = malloc(params.source_width * params.source_height * sizeof(float));

		//initialize various buffer
		memset(nv->mrange_img, 0, params.source_width * params.source_height * sizeof(float));
		memset(nv->mlabel_img, 0, params.source_width * params.source_height * sizeof(pixel_label));
		memset(nv->mnovel_view, Hole, params.source_width * params.source_height * 3 * sizeof(unsigned char));
		memset(nv->mconf, 0, params.source_width * params.source_height * sizeof(float));
		//可以预先设置每个参考相机的权重比例，此时每个参考相机的权重设置为1
		nv->prefW = prefW[sss]; // 

		int re_index_input = cam_ref[sss];

        if(params.depth_scale_type)
            get_up_sample_depth(&views[re_index_input], &params);

        if (params.background_flag)
        {
            depth_pro_background(&views[re_index_input], &params);
        }

        if (params.depth_mf_radius)
        {
            median_filter(views[re_index_input].depth, views[re_index_input].depth_y, params.source_width, params.source_height, params.depth_mf_radius);
        }

        get_real_depth(&views[re_index_input], &params);

		finish = clock();
		double totaltime = (double)(finish - start) / CLOCKS_PER_SEC;

		printf("One pre time = %f\n", totaltime);

		start = clock();

		gen_novel_view(&params, re_index_input, views[re_index_input].texture, views[re_index_input].z_depth, nv, fB, sss);  

        {
            char name[128];
            sprintf(name, "warp_%d.yuv", re_index_input);
            FILE *fp = fopen(name, "wb");
            fwrite(nv->mnovel_view, 1, params.source_width * params.source_height*3,fp);
            fclose(fp);
        }

		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		printf("One warp time = %f\n", totaltime);
	}

	clock_t st, fi;
	st = clock();
	//nvs新视点的指针，ref-参考视点的个数=2
	//merge的时候会对前景边缘进行smooth_foreground，对前后左右半径内的bgr取平均
	merge_novel_views(&params, nvs, REF_CAM_N, params.source_width, params.source_height, fB, vir_view->texture,dist,flag);
	fi = clock();
	printf("merge time = %f\n", (double)(fi - st) / CLOCKS_PER_SEC);

	int select_cam = 0;  //select  1  as output depth
	for (j = 0; j < params.source_height; j++)    //convert float-point range depth into fixed-point disparity depth
		for (i = 0; i < params.source_width; i++)
		{
			nvs[select_cam].mrange_img[j*params.source_width + i] = (float)(fB / nvs[select_cam].mrange_img[j*params.source_width + i]);
			vir_view->depth[j*params.source_width + i] = (unsigned char)((nvs[select_cam].mrange_img[j*params.source_width + i] - MINdisp) * 255.f / (MAXdisp - MINdisp));

		}
    printf("total time = %f\n", (double)(clock() - dibr_start) / CLOCKS_PER_SEC);

	fwrite(vir_view->depth, 1, params.source_width * params.source_height, vir_file->depth);
	fwrite(vir_view->texture, 1, params.source_width * params.source_height * 3, vir_file->texture);

    vss_close(views,&params);
    vss_files_close(files, &params);

    return 0;
}

