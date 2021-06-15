/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "vss.h"
#include "filter.h"

#define LANCZOS_TAB_SIZE        3
#define LANCZOS_FAST_SCALE      100
#define LANCZOS_FAST_MAX_SIZE   4096
static double tbl_lanczos_coef[LANCZOS_FAST_MAX_SIZE * LANCZOS_FAST_SCALE];
#define lanczos_coef(x) tbl_lanczos_coef[(int)(fabs(x) * LANCZOS_FAST_SCALE + 0.5)]

// sinc function
static double sinc(double x)
{
    x *= CV_PI;
    if (x < 0.01 && x > -0.01) {
        double x2 = x * x;
        return 1.0f + x2 * (-1.0 / 6.0 + x2 / 120.0);
    }
    else {
        return sin(x) / x;
    }
}

void init_lanczos_filter()
{
    int i;
    for (i = 0; i < LANCZOS_FAST_MAX_SIZE*LANCZOS_FAST_SCALE; i++)
    {
        double x = (double)i / LANCZOS_FAST_SCALE;
        tbl_lanczos_coef[i] = sinc(x) * sinc(x / LANCZOS_TAB_SIZE);
    }
}

// if f < a, return a
// if f >= a, then
//      if f > z, return z
//      else, return f
static inline float clip(float f, float a, float z) {
    return (f < a) ? a : (f > z) ? z : f;
}

float filtre_lanczos(unsigned char * src, float j, float i, int input_width, int input_height, int i_src, int RGB_offset)
{
    double coef, sum = 0, res = 0;
    int m, n, idx_x, idx_y;
    float ret_val = 0;

    for (n = -LANCZOS_TAB_SIZE; n < LANCZOS_TAB_SIZE; n++)
    {
        for (m = -LANCZOS_TAB_SIZE; m < LANCZOS_TAB_SIZE; m++)
        {
            idx_x = (int)i + m + 1;
            idx_y = (int)j + n + 1;

            coef = lanczos_coef(i - idx_x) * lanczos_coef(j - idx_y);

            // when the neib. pixel is outside the boundary, using the boundary pixels
            idx_x = (idx_x < 0) ? 0 : idx_x;
            idx_y = (idx_y < 0) ? 0 : idx_y;
            idx_x = (idx_x >= input_width) ? (input_width - 1) : idx_x;
            idx_y = (idx_y >= input_height) ? (input_height - 1) : idx_y;

            //res += src[idx_x + idx_y * i_src] * coef;
            res += src[(idx_x + idx_y * i_src) * 3 + RGB_offset] * coef;
            sum += coef;
        }
    }

    if (sum != 0) {
        ret_val = (float)(res / sum + 0.5);
        ret_val = clip(ret_val, 0.0f, 255.0f);
    }

    return ret_val;
}

unsigned char find_medium(unsigned char* depth_window, int N)
{
    int length = N - 1;
    for (int i = 0; i < length; i++)
    {
        //find MAX in each cycle
        for (int j = 0; j < length - i; j++)
        {
            if (depth_window[j] > depth_window[j + 1])
            {
                //interchange positions
                unsigned char temp = depth_window[j];
                depth_window[j] = depth_window[j + 1];
                depth_window[j + 1] = temp;
            }
        }
    }
    return depth_window[N / 2];

}

#define MAX_WINDOW_SIZE 441//(10+10+1)*(10+10+1)
void median_filter(unsigned char * src, unsigned char * buf, int width, int height, int kradius)
{
    unsigned char depth_window[MAX_WINDOW_SIZE*MAX_WINDOW_SIZE];
    int window_size = (kradius + kradius + 1)*(kradius + kradius + 1);
    for (int h = 0; h < height; h++)
        for (int w = 0; w < width; w++)
        {
            unsigned char medium_value = 0;
            for (int i = -kradius; i <= kradius; i++)
            {
                for (int j = -kradius; j <= kradius; j++)
                {
                    int imy = MAX(0, MIN(h + i, height - 1));  
                    int imx = MAX(0, MIN(w + j, width - 1));  //clip
                    depth_window[(i + kradius) * (2 * kradius + 1) + (j + kradius)] = src[imy * width + imx];
                }
            }
            medium_value = find_medium(depth_window, window_size);
            buf[h * width + w] = medium_value;
        }
    memcpy(src, buf, width*height * sizeof(unsigned char));
}
