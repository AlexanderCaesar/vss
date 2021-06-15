
#ifndef __DIBR_H__
#define __DIBR_H__

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "time.h"
#include "parser.h"
#include "vss.h"
#include "filter.h"

#define IS_PINHOLE 0 // decide whether use pinhole model or not 
//#define BILATERAL_FILTER  //decide whether use bilteral filter or not. Modified in Aug, 2020
float krt_K[9];
float krt_K_inv[9];


typedef struct cuRef_data_t
{
	unsigned char  *cuTex;  //cudaTextureObject_t cuTex; 纹理数据
	float* range_img; // -- data allocated by caller 深度图

}cuRef_data_t;

typedef struct cuCam_data_t
{
	float* refR, *refinvR, *reft;
	float* refCC, *refKc, reffradius, reffov;       //reffradius  == fisheye_radius
	float* refr2t_curve;  //arr_rtheta  look-up table  
	int refr2tl;          //rtheta_len
	float* vR, *vinvR, *vt;
	float* vCC, *vKc, vfradius, vfov;
	float *vr2t_curve;
	int vr2tl;

}cuCam_data_t;


typedef struct novel_view_t
{
	float* mrange_img; // -- main/foreground disparity image  视差图
	pixel_label *mlabel_img;  // 前景边缘标记图
	unsigned char *mnovel_view;
	float* mconf;
	float prefW;

}novel_view_t;

// x^2 + y^2 + z^2
//inline float norm2(float *sphere)
//{
//	return sqrtf(sphere[0] * sphere[0] + sphere[1] * sphere[1] + sphere[2] * sphere[2]);
//}


void label_foreground_boudary_pixels(float* range_img, pixel_label* labels, const float dthresh, const float fB, const int width, const int height, const int edge_radius)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			float cval = fB / (range_img[y * width + x] + 1e-5f);

			int rx = MIN(x + 1, width - 1);
			int dy = MIN(y + 1, height - 1);

			float dval = fB / (range_img[dy * width + x] + 1e-5f);
			float rval = fB / (range_img[y * width + rx] + 1e-5f);

			float grad_x = cval - rval;
			float grad_y = cval - dval;

			if (fabs(grad_x) >= dthresh)
			{
				int sid = grad_x > 0.f ? -edge_radius : 1;
				int eid = grad_x > 0.f ? 0 : edge_radius;
				for (int i = sid; i <= eid; i++)
				{
					int imx = MAX(0, MIN(width - 1, i + x));
					labels[y * width + imx] = Foreground_boundary;
				}
			} // -- process horizontal gradients
			if (fabs(grad_y) >= dthresh)
			{
				int sid = grad_y > 0.f ? -edge_radius : 1;
				int eid = grad_y > 0.f ? 0 : edge_radius;
				for (int i = sid; i <= eid; i++)
				{
					int imy = MAX(0, MIN(height - 1, i + y));
					labels[imy * width + x] = Foreground_boundary;
				}
			} // -- process vertical gradients
		}
}

void label_background_boudary_pixels(float* range_img, pixel_label* labels, const float dthresh, const float fB, const int width, const int height, const int edge_radius)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			float cval = fB / (range_img[y * width + x] + 1e-5f);

			int rx = MIN(x + 1, width - 1);
			int dy = MIN(y + 1, height - 1);

			float dval = fB / (range_img[dy * width + x] + 1e-5f);
			float rval = fB / (range_img[y * width + rx] + 1e-5f);

			float grad_x = cval - rval;
			float grad_y = cval - dval;

			if (fabs(grad_x) >= dthresh)
			{
                int sid = grad_x > 0.f ? 1 : -edge_radius; //grad_x > 0.f On the right is the background
                int eid = grad_x > 0.f ? edge_radius : 0;
                for (int i = sid; i <= eid; i++)          //Label the background image on the left or right side respectively. Right side 1,4, left side -4, 0
				{
					int imx = MAX(0, MIN(width - 1, i + x));
					labels[y * width + imx] = Background_boundary;
				}
			} // -- process horizontal gradients
			if (fabs(grad_y) >= dthresh)
			{
				int sid = grad_y > 0.f ? 1 : -edge_radius;
				int eid = grad_y > 0.f ? edge_radius : 0;
				for (int i = sid; i <= eid; i++)
				{
					int imy = MAX(0, MIN(height - 1, i + y));
					labels[imy * width + x] = Background_boundary;
				}
			} // -- process vertical gradients
		}
}

void set_main_layer_labels(pixel_label* labels, const int width, const int height, pixel_label tarL)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{

			if ((x >= width) || (y >= height))
			{
				continue;
			}

			if (labels[y * width + x] != tarL)   //wkj 
			{
				labels[y * width + x] = Main;
			}
			//if (labels[y * width + x] == novel_params.source_heightole) { labels[y * width + x] = novel_pixel_main; }
		}
}


int label_boundary_pixels2(float* range_img, pixel_label* labels, pixel_label tarL,
	float fB, float dthresh, const int edge_radius, int width, int height)
{
	if (tarL == Foreground_boundary)
	{
		label_foreground_boudary_pixels(range_img, labels, dthresh, fB, width, height, edge_radius);
	}
	else if (tarL == Background_boundary)
	{
		label_background_boudary_pixels(range_img, labels, dthresh, fB, width, height, edge_radius);
	}

	set_main_layer_labels(labels, width, height, tarL);

	return I2R_OK;
}


//Brute-Force Search the root for five-degree polynomial equation
float  roots_one_real(float* coeff)
{
	int i;

	float root = 0;
	float MIN_diff = 100.0;
	for (i = 0; i < 100000; i = i + 20) 
	{
		float theta = ((float)i) * 0.00001f; //
		float theta3 = theta * theta * theta;
		float theta5 = theta * theta * theta3;
		float thetaD = coeff[5] * theta5 + coeff[3] * theta3 + coeff[1] * theta + coeff[0];
		if (fabs(thetaD) < MIN_diff)
		{
			MIN_diff = (float)fabs(thetaD);
			root = theta;
		}
	}
	return root;
}


//Polynomial calculation theta table
int set_r2t_array_by_kc(float* arr, float* kc, int rtheta_length, float fisheye_radius)
{
	float coeff[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };//-radius+k_1 r+k_2 r^3+k_3 r^5

	coeff[1] = kc[0];
	coeff[3] = kc[1];
	coeff[5] = kc[2];

	for (int i = 0; i < rtheta_length; i++)    // rtheta_length = r2t_len = 2*fisheye_radius 
	{
		float r = (float)(i) / (float)(rtheta_length)* fisheye_radius; 
		coeff[0] = -r;

		arr[i] = roots_one_real(coeff); // MIN_theta_val;                
	}
	return I2R_OK;
}
// bilteral filter added, modified in Aug, 2020
int set_ws_array(float* arr, float drange, int radius)
{
	const float sigs = (float)(radius * 1.2f);

	for (int i = 0; i <= radius * radius * 2; i++)
	{
		float dist = sqrtf((float)(i));
		float ws = expf(-dist / sigs);
		arr[i] = ws;
	}
	return I2R_OK;
}
// bilteral filter added, modified in Aug, 2020
int set_wd_array(float* arr, int wd_array_len, const int radius, float drange)
{
	const float sigd = (float)(radius * 3.6f);
	for (int i = 0; i < wd_array_len; i++)
	{
		float r = (float)(i) / (float)(wd_array_len)* drange;
		float wd = expf(-r / sigd);
		arr[i] = wd;
	}
	return I2R_OK;
}

#define I(_i, _j) ((_j)+3*(_i))


void matrix_mult_3x3_by_3x3(float *m, float *lhs, float *rhs)
{
	// m = rhs * lhs

    m[0] = lhs[I(0, 0)] * rhs[I(0, 0)] + lhs[I(0, 1)] * rhs[I(1, 0)] + lhs[I(0, 2)] * rhs[I(2, 0)];
    m[1] = lhs[I(0, 0)] * rhs[I(0, 1)] + lhs[I(0, 1)] * rhs[I(1, 1)] + lhs[I(0, 2)] * rhs[I(2, 1)];
    m[2] = lhs[I(0, 0)] * rhs[I(0, 2)] + lhs[I(0, 1)] * rhs[I(1, 2)] + lhs[I(0, 2)] * rhs[I(2, 2)];

    m[3] = lhs[I(1, 0)] * rhs[I(0, 0)] + lhs[I(1, 1)] * rhs[I(1, 0)] + lhs[I(1, 2)] * rhs[I(2, 0)];
    m[4] = lhs[I(1, 0)] * rhs[I(0, 1)] + lhs[I(1, 1)] * rhs[I(1, 1)] + lhs[I(1, 2)] * rhs[I(2, 1)];
    m[5] = lhs[I(1, 0)] * rhs[I(0, 2)] + lhs[I(1, 1)] * rhs[I(1, 2)] + lhs[I(1, 2)] * rhs[I(2, 2)];

    m[6] = lhs[I(2, 0)] * rhs[I(0, 0)] + lhs[I(2, 1)] * rhs[I(1, 0)] + lhs[I(2, 2)] * rhs[I(2, 0)];
    m[7] = lhs[I(2, 0)] * rhs[I(0, 1)] + lhs[I(2, 1)] * rhs[I(1, 1)] + lhs[I(2, 2)] * rhs[I(2, 1)];
    m[8] = lhs[I(2, 0)] * rhs[I(0, 2)] + lhs[I(2, 1)] * rhs[I(1, 2)] + lhs[I(2, 2)] * rhs[I(2, 2)];
}

/*Transposed matrix*/
void matrix_inverse_or_rotate_3x3(float *m, float *lhs)  // for rotate matrix, its inverse is just its rotate.
{
	int i, j;

	for (j = 0; j < 3; j++)
	{
		for (i = 0; i < 3; i++)
		{
			m[3 * j + i] = lhs[3 * i + j];
		}
	}
}

void shear_row(float mat[][3], int r1, int r2, float scalar)
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		mat[r1][i] += scalar * mat[r2][i];
	}
}
void scale_row(float mat[][3], int r, float scalar)
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		mat[r][i] *= scalar;
	}
}

void swap_rows(float** mat, int r1, int r2)
{
	float* tmp;
	tmp = mat[r1];
	mat[r1] = mat[r2];
	mat[r2] = tmp;
}

void set_identity_matrix(float** mat)
{
	int i, j;
	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				mat[i][j] = 1.0f;
			}
			else
			{
				mat[i][j] = 0.0f;
			}
		}
	}
}

int matrix_inv3x3(float* output, float* input)
{
	int i, j, r;
	float input_s[3][3];
	float output_s[3][3];
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			input_s[i][j] = input[i * 3 + j] ;
		}
	}
	float scalar;
	float shear_needed;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				output_s[i][j] = 1.0f;
			}
			else
			{
				output_s[i][j] = 0.0f;
			}
		}
	}
	for ( i = 0; i < 3 ; ++ i)
	{
		if (input_s[i][i] == 0.0)
		{
			for (r = i + 1; r < 3; ++r)
			{
				if (input_s[r][i] != 0.0)
				{
					break;
				}
			}
			if (r == 3)
			{
				return 0;
			}
			swap_rows(input_s, i, r);
			swap_rows(output_s, i, r);
		}
		scalar = 1.0 / input_s[i][i];
		scale_row(input_s, i, scalar);
		scale_row(output_s, i, scalar);
		for (j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				continue;
			}
			shear_needed = -input_s[j][i];
			shear_row(input_s, j, i, shear_needed);
			shear_row(output_s, j, i, shear_needed);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			output[i * 3 + j] = output_s[i][j] ;
		}
	}
	return 1;
}



void matrix_mult_3x3_by_3x1(float* dst, float* mat3x3, float* vec3x1)
{
	float t[3];

	t[0] = mat3x3[0] * vec3x1[0] + mat3x3[1] * vec3x1[1] + mat3x3[2] * vec3x1[2];
	t[1] = mat3x3[3] * vec3x1[0] + mat3x3[4] * vec3x1[1] + mat3x3[5] * vec3x1[2];
	t[2] = mat3x3[6] * vec3x1[0] + mat3x3[7] * vec3x1[1] + mat3x3[8] * vec3x1[2];
	memcpy(dst, t, sizeof(t));
}


void vec3x1_sub(float* dst, float* mat3x1, float* vec3x1)
{
    float t[3];
    t[0] = mat3x1[0] - vec3x1[0];
	t[1] = mat3x1[1] - vec3x1[1];
	t[2] = mat3x1[2] - vec3x1[2];
	memcpy(dst, t, sizeof(t));
}

void vec3x1_add(float* dst, float* mat3x1, float* vec3x1)
{
	float t[3];
	t[0] = mat3x1[0] + vec3x1[0];
	t[1] = mat3x1[1] + vec3x1[1];
	t[2] = mat3x1[2] + vec3x1[2];
	memcpy(dst, t, sizeof(t));
}

/* X* R + T */
void euclidean_transform_point_dp(float* newP, float* R, float* P, float *t)
{
	float Rp[3];
	matrix_mult_3x3_by_3x1(Rp, R, P);
	newP[0] = Rp[0] + t[0];
	newP[1] = Rp[1] + t[1];
	newP[2] = Rp[2] + t[2];
}

float get_theta_by_r(float* arr_rtheta, int length_rtheta, float fisheye_radius, float r)
{
	int indx = (int)(r / fisheye_radius * length_rtheta); // length_rtheta = 2*fisheye_radius
	indx = indx < length_rtheta ? indx : length_rtheta - 1;//clip
	return arr_rtheta[indx];
}

int warp_range_to_novel_view(vss_param* params, cam_param* ccam, float* range_img, pixel_label* labels, cam_param *vcam,
	float* novel_range_img, pixel_label* novel_label_img, float* r2t_curve, int r2t_len, float fB, float dthresh, const int edge_radius, int ispinhole)
{
	int width = ccam->width, height = ccam->height;
	float p3D[3] = { 0, 0, 0 };// Mat::zeros(3, 1, CV_32FC1);
	float krt_R[9];

    /*Camera coordinate conversion*/
    /*From the reference camera to the virtual camera*/
	float temp_invR[9];
	matrix_inverse_or_rotate_3x3(temp_invR, ccam->krt_R);     
	matrix_mult_3x3_by_3x3(krt_R, vcam->krt_R, temp_invR);               // krt_R = vcam->krt_R * temp_invR

    /*To calculate Translation matrix T = Rv*(ccam->krt_WorldPosition - vcam->krt_WorldPosition)*/
	float krt_t[3];
	vec3x1_sub(krt_t, ccam->krt_WorldPosition, vcam->krt_WorldPosition); // Translation matrix krt_t = ccam->krt_WorldPosition - vcam->krt_WorldPosition; 
	matrix_mult_3x3_by_3x1(krt_t, vcam->krt_R, krt_t);                   //  T = Rv*(ccam->krt_WorldPosition - vcam->krt_WorldPosition)

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			pixel_label Lab = labels[i * width + j];

			if (Main != Lab) { continue; } // project novel_pixel_main

			float xd_norm = (j - ccam->krt_cc[0]);
			float yd_norm = (i - ccam->krt_cc[1]);
			float radius = sqrtf(xd_norm * xd_norm + yd_norm * yd_norm);//需要打表
			float theta = get_theta_by_r(r2t_curve, r2t_len, ccam->fisheye_radius, radius);     //get theta by look-up table

			float phi = atan2(yd_norm, xd_norm);//需要打表
			float z_depth = range_img[i * width + j]; 

            if (z_depth <= 0.f) { continue; }

            float np3D[3];
            float r = 0.0f;
            int xp = 0;
            int yp = 0;

            if (params->model == FastPinhole)
            {
                p3D[2] = z_depth;
                p3D[0] = (j-(width>>1))*z_depth/ ccam->krt_kc[0];
                p3D[1] = (i - (height >> 1)) * z_depth / ccam->krt_kc[0];

                euclidean_transform_point_dp(np3D, krt_R, p3D, krt_t); //X*R + T;  

                float x_norm = vcam->krt_kc[0]* np3D[0] / np3D[2];   //step 3:  check: FOV, image res,fisheye_radius   
                float y_norm = vcam->krt_kc[0] * np3D[1] / np3D[2];   //map x,y,z on the plane z = 1

                r = sqrtf(x_norm * x_norm + y_norm * y_norm); //(phi, theta) under virtual camera

                xp = x_norm + (width >> 1);
                yp = y_norm + (height >> 1);
            }
            else
            {
                p3D[2] = z_depth * cos(theta);
                float r_undistort = sqrtf(z_depth * z_depth - p3D[2] * p3D[2]);
                p3D[0] = r_undistort * cos(phi);
                p3D[1] = r_undistort * sin(phi);

                
                euclidean_transform_point_dp(np3D, krt_R, p3D, krt_t); //X*R + T;     

                float x_norm = np3D[0] / np3D[2];   //step 3:  check: FOV, image res,fisheye_radius   
                float y_norm = np3D[1] / np3D[2];   //map x,y,z on the plane z = 1
                phi = atan2(y_norm, x_norm);
                r_undistort = sqrtf(x_norm * x_norm + y_norm * y_norm); //(phi, theta) under virtual camera
                theta = atan(r_undistort);

                if (np3D[2] < 0.f) { theta = CV_PI - theta; } // -- beyond 180 degree
                if (theta > vcam->lens_fov)
                {
                    continue;
                } // -- out of the lens fov
                float tp2 = theta * theta;
                float tp3 = theta * tp2;
                float tp5 = tp3 * tp2;
                r = vcam->krt_kc[0] * theta + vcam->krt_kc[1] * tp3 + vcam->krt_kc[2] * tp5;
                xp = (int)roundf(r * cos(phi) + vcam->krt_cc[0]);
                yp = (int)roundf(r * sin(phi) + vcam->krt_cc[1]);
            }		

			float newrange_pinhole;
			//if (ispinhole)
			//{
			//	float fx = cos(phi) * r_undistort * r / (theta * x_norm);
			//	float fy = sin(phi) * r_undistort * r / (theta * y_norm);
			//	krt_K[0] = fx;
			//	krt_K[1], krt_K[3], krt_K[6], krt_K[7] = 0;
			//	krt_K[2] = vcam->krt_cc[0];
			//	krt_K[4] = fy;
			//	krt_K[5] = vcam->krt_cc[1];
			//	krt_K[8] = 1;
			//	float uvcz[3] = { z_depth * j, z_depth * i, z_depth };
			//	float XYZcam[3];
			//	
			//	matrix_inv3x3(krt_K_inv, krt_K);
			//	matrix_mult_3x3_by_3x1(XYZcam, krt_K_inv, uvcz);

			//	p3D[0] = XYZcam[0];
			//	p3D[1] = XYZcam[1];
			//	p3D[2] = z_depth;

			//	euclidean_transform_point_dp(np3D, krt_R, p3D, krt_t);

			//	float v_xyz[3];
			//	matrix_mult_3x3_by_3x1(v_xyz, krt_K, np3D);
			//	newrange_pinhole = np3D[2];
			//	xp = (int)roundf(v_xyz[0] / v_xyz[2]);
			//	yp = (int)roundf(v_xyz[1] / v_xyz[2]);
			//}

			if (r < vcam->fisheye_radius && xp >= 1 && xp <= (width - 1) - 1 && yp >= 1 && yp <= (height - 1) - 1)   //because 3x3 
			{
				float newrange;
				if (ispinhole)
				{
					newrange = newrange_pinhole;
				}
				else
				{
					newrange = sqrtf(np3D[0] * np3D[0] + np3D[1] * np3D[1] + np3D[2] * np3D[2]); //step 4:					
				}

				for (int m = yp - 1; m <= yp + 1; m++)
				{
					for (int n = xp - 1; n <= xp + 1; n++)
					{
						{
							float nrange = novel_range_img[m * width + n];
							if (nrange == 0.f)
							{
								novel_range_img[m * width + n] = newrange;
								novel_label_img[m * width + n] = Main;  
							}
							else if (newrange < nrange)
							{
								novel_range_img[m * width + n] = newrange;
								novel_label_img[m * width + n] = Main;
							}
						}
					}
				} // -- project 1 pixel to 9 pixels to avoid cracks
			}
		} // -- within valid image plane
	}

	return I2R_OK;
}


int alloc_cuRef(cuRef_data_t *ppp, unsigned char* bgra, float* range_img, int width, int height)
{

	ppp->cuTex = malloc(width * height * 3 * sizeof(unsigned char));
	memcpy(ppp->cuTex, bgra, width * height * 3 * sizeof(unsigned char));

	ppp->range_img = malloc(width * height * sizeof(float));
	memcpy(ppp->range_img, range_img, width * height * sizeof(float));

	return I2R_OK;
}

int alloc_camp(cuCam_data_t* cp, cam_param* ccam, cam_param* vcam)
{
	cp->vfradius = vcam->fisheye_radius;
	cp->reffradius = ccam->fisheye_radius;
	cp->vr2tl = (int)(vcam->fisheye_radius * 2.f);
	cp->refr2tl = (int)(ccam->fisheye_radius * 2.f);
	cp->reffov = ccam->lens_fov;
	cp->vfov = vcam->lens_fov;


	cp->vr2t_curve = (float*)malloc(cp->vr2tl * sizeof(float));
	cp->refr2t_curve = (float*)malloc(cp->refr2tl * sizeof(float));

	set_r2t_array_by_kc(cp->vr2t_curve, vcam->krt_kc, cp->vr2tl, cp->vfradius);
	set_r2t_array_by_kc(cp->refr2t_curve, ccam->krt_kc, cp->refr2tl, cp->reffradius);

	// -- prepare matrix data
	float krt_R[9];// = ccam->krt_R * vcam->krt_R.inv();
	float temp_invR[9];
	matrix_inverse_or_rotate_3x3(temp_invR, vcam->krt_R);     // matrix_mult_3x3_by_3x3(krt_R, temp_invR, vcam->krt_R);   //check whether inverse  is right

	matrix_mult_3x3_by_3x3(krt_R, ccam->krt_R, temp_invR);

	float krt_invR[9];// = krt_R.inv();
	matrix_inverse_or_rotate_3x3(krt_invR, krt_R);       //matrix_mult_3x3_by_3x3(temp_invR, temp_invR, vcam->krt_R);

	float krt_WorldPosition[3];// = vcam->krt_R * (ccam->krt_WorldPosition - vcam->krt_WorldPosition);
	vec3x1_sub(krt_WorldPosition, vcam->krt_WorldPosition, ccam->krt_WorldPosition);  //Because there is a MINus symbol in the following, so we exchange the place of vcam and ccam.
	float krt_t[3];// = -krt_R * krt_WorldPosition;
	matrix_mult_3x3_by_3x1(krt_t, ccam->krt_R, krt_WorldPosition);


	cp->refR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->refR, krt_R, 9 * sizeof(float));
	cp->refinvR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->refinvR, krt_invR, 9 * sizeof(float));
	cp->reft = (float*)malloc(3 * sizeof(float));
	memcpy(cp->reft, krt_t, 3 * sizeof(float));

	float I[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };//Mat::eye(3, 3, CV_32FC1); // -- identity matrix

	cp->vR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->vR, I, 9 * sizeof(float));
	cp->vinvR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->vinvR, I, 9 * sizeof(float));
	cp->vt = (float*)malloc(3 * sizeof(float));
	memset(cp->vt, 0, 3 * sizeof(float));


	cp->refCC = (float*)malloc(2 * sizeof(float));
	cp->refCC[0] = ccam->krt_cc[0], cp->refCC[1] = ccam->krt_cc[1];

	cp->refKc = (float*)malloc(3 * sizeof(float));
	cp->refKc[0] = ccam->krt_kc[0], cp->refKc[1] = ccam->krt_kc[1], cp->refKc[2] = ccam->krt_kc[2];


	cp->vCC = (float*)malloc(2 * sizeof(float));
	cp->vCC[0] = vcam->krt_cc[0], cp->vCC[1] = vcam->krt_cc[1];

	cp->vKc = (float*)malloc(3 * sizeof(float));
	cp->vKc[0] = vcam->krt_kc[0], cp->vKc[1] = vcam->krt_kc[1], cp->vKc[2] = vcam->krt_kc[2];
	return I2R_OK;
}

int destroy_camp(cuCam_data_t* cp)
{
	free(cp->refR);
	free(cp->refinvR);
	free(cp->reft);
	free(cp->refCC);
	free(cp->refKc);
	free(cp->refr2t_curve);

	free(cp->vR);
	free(cp->vinvR);
	free(cp->vt);
	free(cp->vCC);
	free(cp->vKc);
	free(cp->vr2t_curve);
	return I2R_OK;
}

void median_filter_range_image(float* irange, float* orange, pixel_label* ilabels, pixel_label* olabels, int width, int height, int kradius, float fB, float MINdp, float dprange)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			if (ilabels[y * width + x] == Hole)
			{
				olabels[y * width + x] = Hole;
				orange[y * width + x] = 0.f;
				continue;
			}

			const int range_level = 512 + 1; // -- evenly divide the range into multiple bins, +1 for invalid holes
			//int hist[range_level];
			int hist[512 + 1];
			memset(hist, 0, range_level * sizeof(int)); // -- histogram
			float bin_len = dprange / (float)(range_level - 1);
			//int vnbr = 0;
			for (int i = -kradius; i <= kradius; i++)
			{
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = MAX(0, MIN(y + i, height - 1));  //make sure the index (x+j, y+i) is located in the image.
					int imx = MAX(0, MIN(x + j, width - 1)); // -- clamp
					if (ilabels[imy * width + imx] == Hole)
					{
						hist[0] ++;
						continue;
					} // -- invalid holes

					float ival = fB / irange[imy * width + imx]; // -- input range value
					int binID = MAX(0, MIN(range_level - 2, (int)floor((ival - MINdp) / bin_len)));
					hist[binID + 1] ++;
					//vnbr++;
				}
			}
			const int tNbr = (2 * kradius + 1) * (2 * kradius + 1);
			int mID = 0; // -- median bin ID
			int aval = 0; // -- accumulated value
			for (int id = 0; id < range_level + 1; id++)
			{
				aval += hist[id];
				if (aval > tNbr / 2)
				{
					mID = id;
					break;
				}
			} // -- select the median value by accumulating the histogram
			if (mID == 0)
			{
				orange[y * width + x] = 0.f;
				olabels[y * width + x] = Hole;
			}
			else
			{
				float oval = fB / ((mID - 1.f + 0.5f) * bin_len + MINdp);
				orange[y * width + x] = oval;
				olabels[y * width + x] = Main;
			}
		}
}

// add bilateral filter for range image. Modified in Aug, 2020
void bilateral_filter_range_image(float* irange, float* orange, pixel_label* ilabels, pixel_label* olabels, int width, int height, int kradius, float fB, float* ws_arr, float* wd_arr, int wd_arr_len, float drange)
{
	int x, y;

	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			if (ilabels[y * width + x] == Hole)
			{
				olabels[y * width + x] = Hole;
				orange[y * width + x] = 0.f;
				continue;
			}

			float range = fB / irange[y * width + x];
			float sumC = 0.f;
			float sumW = 0.f;
			const float sigs = (float)(kradius * 1.2f);
			const float sigd = (float)(kradius * 3.6f);
			for (int i = -kradius; i <= kradius; i++) {
				for (int j = -kradius; j <= kradius; j++)
				{

					int imy = MAX(0, MIN(y + i, height - 1));
					int imx = MAX(0, MIN(x + j, width - 1));

					if (ilabels[imy * width + imx] == Hole)
					{
						continue;
					}
					int dist = (i * i + j * j);
					float ws = ws_arr[dist];

					float disp = fB / irange[imy * width + imx];
					int indx = fabs(disp - range) / drange * wd_arr_len;
					indx = indx < wd_arr_len ? indx : wd_arr_len - 1;
					float wd = wd_arr[indx];
					float d = irange[imy * width + imx];

					sumC += (ws * wd * d);
					sumW += (ws * wd);

				}
			}


			if (sumW < 1e-5f) { continue; }

			orange[y * width + x] = sumC / sumW;
			olabels[y * width + x] = Main;
		}
}

int median_filtering_novel_range(float* range, pixel_label* labels, float fB, float MINdp, float dprange, int radius, int width, int height)
{
	float* orange = NULL;
	pixel_label* olabels = NULL;
	orange = (float*)malloc(width * height * sizeof(float));
	olabels = (float*)malloc(width * height * sizeof(pixel_label));
	memset(orange, 0, width * height * sizeof(float));
	memset(olabels, Hole, width * height * sizeof(pixel_label));

	median_filter_range_image(range, orange, labels, olabels, width, height, radius, fB, MINdp, dprange);

	memcpy(range, orange, width * height * sizeof(float));
	memcpy(labels, olabels, width * height * sizeof(pixel_label));

	free(orange);
	free(olabels);
	return I2R_OK;
}
// add bilateral filter for range image. Modified in Aug, 2020
int bilateral_filtering_novel_range(float* range, pixel_label* labels, float fB, int radius, int width, int height, float* ws_arr, float* wd_arr, int wd_arr_len, float drange)
{
	float* orange = NULL;
	pixel_label* olabels = NULL;
	orange = (float*)malloc(width * height * sizeof(float));
	olabels = (float*)malloc(width * height * sizeof(pixel_label));
	memset(orange, 0, width * height * sizeof(float));
	memset(olabels, Hole, width * height * sizeof(pixel_label));

	bilateral_filter_range_image(range, orange, labels, olabels, width, height, radius, fB, ws_arr, wd_arr, wd_arr_len, drange);

	memcpy(range, orange, width * height * sizeof(float));
	memcpy(labels, olabels, width * height * sizeof(pixel_label));

	free(orange);
	free(olabels);
	return I2R_OK;
}

float get_theta_by_r_dp(float* arr_rtheta, int length_rtheta, float fisheye_radius, float r)
{
	int indx = r / fisheye_radius * length_rtheta;
	indx = indx < length_rtheta ? indx : length_rtheta - 1;
	return arr_rtheta[indx];
}


void get_unit_view_vector_dp(vss_param* params, cam_param* ccam, int forward, float* rtheta_arr, int rtheta_len, float* cc, float fisheye_radius, float x, float y, float *unit_vv)
{
	float xd_norm = x - cc[0];
	float yd_norm = y - cc[1];
	float radius = sqrtf(xd_norm * xd_norm + yd_norm * yd_norm);

	float theta = get_theta_by_r_dp(rtheta_arr, rtheta_len, fisheye_radius, radius);  //resort to  look-up table
	float phi = atan2(yd_norm, xd_norm);

	float sphrad = 1.f; // -- unit vector

    if (params->model == FastPinhole)
    {
        if (forward)
        {
            unit_vv[2] = sphrad;
            unit_vv[0] = xd_norm * sphrad / ccam->krt_kc[0];
            unit_vv[1] = yd_norm * sphrad / ccam->krt_kc[0];
        }
        else
        {
            unit_vv[2] = sphrad;
            unit_vv[0] = xd_norm * sphrad / params->krt_vcam.krt_kc[0];
            unit_vv[1] = yd_norm * sphrad / params->krt_vcam.krt_kc[0];
        }
    }
    else
    {
        unit_vv[2] = sphrad * cos(theta);
        float r_undistort = sqrtf(sphrad - unit_vv[2] * unit_vv[2]);
        unit_vv[0] = r_undistort * cos(phi);
        unit_vv[1] = r_undistort * sin(phi);
    }

}


int project_to_camera_dp(vss_param* params, int forward, cam_param* ccam, float* worldP, float* R, float *t, float* kc, float* cc, float fisheye_radius, float lens_fov, int* img_reso, float *xp, float *yp, int ispinhole)
{
	float newWorldP[3];
	euclidean_transform_point_dp(newWorldP, R, worldP, t);

    float r = 0.0f;

    if (params->model == FastPinhole)
    {
        if (forward)
        {
            float x_norm = params->krt_vcam.krt_kc[0] * newWorldP[0] / newWorldP[2];   //step 3:  check: FOV, image res,fisheye_radius   
            float y_norm = params->krt_vcam.krt_kc[0] * newWorldP[1] / newWorldP[2];   //map x,y,z on the plane z = 1

            r = sqrtf(x_norm * x_norm + y_norm * y_norm); //(phi, theta) under virtual camera

            *xp = x_norm + (params->krt_vcam.width >> 1);
            *yp = y_norm + (params->krt_vcam.height >> 1);
        }
        else
        {
            float x_norm = ccam->krt_kc[0] * newWorldP[0] / newWorldP[2];   //step 3:  check: FOV, image res,fisheye_radius   
            float y_norm = ccam->krt_kc[0] * newWorldP[1] / newWorldP[2];   //map x,y,z on the plane z = 1

            r = sqrtf(x_norm * x_norm + y_norm * y_norm); //(phi, theta) under virtual camera

            *xp = x_norm + (params->krt_vcam.width >> 1);
            *yp = y_norm + (params->krt_vcam.height >> 1);
        }
    }
    else
    {
        float x_norm = newWorldP[0] / newWorldP[2];
        float y_norm = newWorldP[1] / newWorldP[2];

        float ryx = fabs(y_norm / (x_norm + 1e-10f)); // -- avoid /0 error
        float r_undistort = sqrtf(x_norm * x_norm + y_norm * y_norm);
        float theta = atan(r_undistort);

        if (newWorldP[2] < 0.f) { theta = CV_PI - theta; } // -- beyond 180 degree
        if (theta > lens_fov) { return I2R_ERR; } // -- out of the lens fov

        float tp2 = theta * theta;
        float tp3 = theta * tp2;
        float tp5 = tp3 * tp2;
        r = kc[0] * theta + kc[1] * tp3 + kc[2] * tp5;
        float fabsx = sqrtf(r * r / (1.f + ryx * ryx));
        float fabsy = ryx * fabsx;
        *xp = (x_norm > 0.f ? fabsx : -fabsx) + cc[0];
        *yp = (y_norm > 0.f ? fabsy : -fabsy) + cc[1];
    }


	//if (ispinhole)
	//{
	//	float uvz[3];
	//	matrix_mult_3x3_by_3x1(uvz, krt_K, newWorldP);
	//	uvz[0] = uvz[0] / uvz[2];
	//	uvz[1] = uvz[1] / uvz[2];
	//	*xp = uvz[0];
	//	*yp = uvz[1];
	//}

	if ((r > fisheye_radius) || (*xp > img_reso[0]) || (*xp < 0) || (*yp > img_reso[1]) || (*yp < 0))
	{
		return I2R_ERR;// I2R_ERR;
	} // -- out of the image plane
	else
	{
		return I2R_OK;
	}
}

void generate_novel_view(vss_param* params, cam_param* ccam, cuRef_data_t refData, float* novel_range_img, unsigned char* novel_view, pixel_label* novel_ls, float* novel_conf,
	pixel_label tarL, int width, int height, cuCam_data_t camp, const float prefW, float fB, int ispinhole)
{
	printf("prefW: %f\n", prefW);
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			float range = novel_range_img[y * width + x];

			if (range <= 0.f)
			{
				continue;       
			} // -- invalid range data

			float unitvv[3];
			//虚拟相机像素坐标系（x，y）到世界坐标系unitvv
			get_unit_view_vector_dp(params, ccam, 0, camp.vr2t_curve, camp.vr2tl, camp.vCC, camp.vfradius, x, y, unitvv); //step1: 虚拟相机像素坐标系到世界坐标系range image (x,y,range)  into  world (x',y',z')
			unitvv[0] *= range; unitvv[1] *= range; unitvv[2] *= range;			

			//if (ispinhole)
			//{
			//	float uvz[3] = { range * x, range * y, range };
			//	matrix_mult_3x3_by_3x1(unitvv, krt_K_inv, uvz);
			//}

			int reso[] = { width, height };
			float xpf, ypf;
			//世界坐标系unitvv 投影到参考相机坐标（u，v）即（xpf，ypf）（可能不是整像素），然后lanczos滤波从（u，v）附近取值，unitvv虚拟相机世界坐标系
			if (project_to_camera_dp(params,0, ccam, unitvv, camp.refR, camp.reft, camp.refKc, camp.refCC, camp.reffradius, camp.reffov, reso, &xpf, &ypf,ispinhole)) //step2: (x',y',z')  into  (xpf, ypf)世界坐标系到参考相机像素坐标系
			{
				//printf("xpf: %f, %f\n", xpf, ypf);
				//float4 pix = tex2D<float4>(refData.cuTex, xpf + 0.5f, ypf + 0.5f);
				unsigned char  pix[3];
				int xxx = MAX(0, MIN((int)(round(xpf)), width - 1));
				int yyy = MAX(0, MIN((int)(round(ypf)), height - 1));
				pix[0] = filtre_lanczos(refData.cuTex, ypf + 0.5f, xpf + 0.5f, width, height, width, 0);    //use subpixel interpolation
				pix[1] = filtre_lanczos(refData.cuTex, ypf + 0.5f, xpf + 0.5f, width, height, width, 1);
				pix[2] = filtre_lanczos(refData.cuTex, ypf + 0.5f, xpf + 0.5f, width, height, width, 2);

				novel_view[y * width * 3 + x * 3 + 0] = (unsigned char)(pix[0]);
				novel_view[y * width * 3 + x * 3 + 1] = (unsigned char)(pix[1]);
				novel_view[y * width * 3 + x * 3 + 2] = (unsigned char)(pix[2]);

				// -- also compute the back projection for confidence   底下都是计算conf，（xp，yp）参考相机图像坐标系坐标
				int xp = MAX(0, MIN((int)(round(xpf + 0.5f)), 1 * width - 1));
				int yp = MAX(0, MIN((int)(round(ypf + 0.5f)), 1 * height - 1));
				float bunitvv[3]; // -- unit view vector of reference camera
				//（xp，yp）投影到参考相机坐标系bunitvv
				get_unit_view_vector_dp(params, ccam, 1, camp.refr2t_curve, camp.refr2tl, camp.refCC, camp.reffradius, xpf, ypf, bunitvv);

				float brange = refData.range_img[yp  * width / 1 + xp / 1];
				if (brange <= 0.f)
				{
					continue;
				}  // -- invalid holes
				bunitvv[0] *= brange; bunitvv[1] *= brange; bunitvv[2] *= brange;
				if (ispinhole)
				{
					float uvz[3] = { brange * x, brange * y, brange };
					matrix_mult_3x3_by_3x1(bunitvv, krt_K_inv, uvz);
				}
				// -- project back    
				float pmt[] = { bunitvv[0] - camp.reft[0], bunitvv[1] - camp.reft[1], bunitvv[2] - camp.reft[2] };
				float pback[3]; // -- 3D points under ref camera's coordinate system
				//pmt是（xp，yp）的世界坐标系坐标
				matrix_mult_3x3_by_3x1(pback, camp.refinvR, pmt);

				float bxpf, bypf;
				//原始相机（u，v）即（xp，yp）反向映射回虚拟相机坐标系（bxpf，bypf）
				if (project_to_camera_dp(params,1, ccam, pback, camp.vR, camp.vt, camp.vKc, camp.vCC, camp.vfradius, camp.vfov, reso, &bxpf, &bypf,ispinhole))
				{
					float diffx = x - bxpf, diffy = y - bypf;             //wkj
					float dist = sqrtf(diffx * diffx + diffy * diffy);
					novel_conf[y * width + x] = expf(-dist / 5.f) * prefW;          //mconf   权重用到该点的深度图，深度置信度

				}
			}

		}


}


int warp_image_to_novel_view(vss_param* params, cam_param* ccam, novel_view_t* nv, cuRef_data_t *curef, cuCam_data_t* cuCam, int width, int height, float fB ,int ispinhole)
{

	generate_novel_view(params, ccam, *curef, nv->mrange_img, nv->mnovel_view, nv->mlabel_img, nv->mconf, Main, width, height, *cuCam, nv->prefW, fB, ispinhole);

	return I2R_OK;
}

int gen_novel_view(vss_param* params, int re_index_input, unsigned char* bgra, float* range_img, novel_view_t *nv, float fB,int s)
{
	int width = params->source_width, height = params->source_height;

    cam_param* ccam = &(params->krt_rcam[re_index_input]);
    float MINdepth = params->min_depth;
    float MAXdepth = params->max_depth;

	// -- allocate texture object for bgra data
	cuRef_data_t cuRef;
	alloc_cuRef(&cuRef, bgra, range_img, width, height); // rgb & depth for cuRef
	cuCam_data_t camp;
	alloc_camp(&camp, ccam, &params->krt_vcam); // camera parameter for camp

	//	--	step 0. label foreground/background pixels for later processing
	//				foreground pixels will be gaussian-smoothed after merging all the novel views
	//				background pixels will be used to perform a erosion to remove ghost coutours
	pixel_label* labels = NULL;
	labels = malloc(width * height * sizeof(pixel_label));

	memset(labels, Hole, width * height * sizeof(pixel_label));

	float MAXdisp = fB / MINdepth, MINdisp = fB / MAXdepth;
	float drange = MAXdisp - MINdisp + 1;
	const float dtr = 0.01666667f; // -- disparity threashold ratio  ???? 数据待测试 dtr=1/60
	const int edge_radius = 4; // -- the same as in the original paper   for 1080P is 4.  数据待测试

    /*label_boundary*/
	label_boundary_pixels2(cuRef.range_img, labels, Background_boundary, fB, dtr * drange, edge_radius, width, height);
	
	//	--	step 1. warp range image by forward projection,  这里可以打表
	int r2t_len = (int)(ccam->fisheye_radius * 2.f);
	float* r2t_curve;// = new float[r2t_len];
	r2t_curve = (float*)malloc(r2t_len * sizeof(float));
	memset(r2t_curve, 0, r2t_len * sizeof(float));

	set_r2t_array_by_kc(r2t_curve, ccam->krt_kc, r2t_len, ccam->fisheye_radius);
	
    clock_t st;

	st = clock();
	//warp main/foreground layer
	warp_range_to_novel_view(params, ccam, range_img, labels, &params->krt_vcam, nv->mrange_img, nv->mlabel_img, r2t_curve, r2t_len, fB, dtr * drange, edge_radius,IS_PINHOLE);
	free(r2t_curve);

	printf("warp_range_to_novel_view time = %f\n", (double)(clock()- st)/ CLOCKS_PER_SEC);
	st = clock();
	//	--	step 2. median filtering the novel range image. Updated in Aug, 2020.
#ifdef BILATERAL_FILTER
	int radius = 3;
	float* ws_array;
	ws_array = (float*)malloc((radius * radius * 2 + 1) * sizeof(float));
	memset(ws_array, 0, (radius * radius * 2 + 1) * sizeof(float));
	set_ws_array(ws_array, drange, radius);

	int wd_array_len = (int)(drange * 2.f);
	float* wd_array;
	wd_array = (float*)malloc(wd_array_len * sizeof(float));
	memset(wd_array, 0, wd_array_len * sizeof(float));
	set_wd_array(wd_array, wd_array_len, radius, drange);

	bilateral_filtering_novel_range(nv->mrange_img, nv->mlabel_img, fB, radius, width, height, ws_array, wd_array, wd_array_len, drange);
	free(ws_array);
	free(wd_array);
	printf("bilateral_filtering_novel_range time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);
#else
	median_filtering_novel_range(nv->mrange_img, nv->mlabel_img, fB, MINdisp, drange, 4, width, height);

	printf("median_filtering_novel_range time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);
#endif // BILATERAL_FILTER
	st = clock();

	//generate layered novel view image based on the novel range image
	warp_image_to_novel_view(params, ccam, nv, &cuRef, &camp, width, height, fB, IS_PINHOLE);

	printf("warp_image_to_novel_view time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);

	destroy_camp(&camp);
	free(cuRef.range_img);
	free(labels);


	return I2R_OK;
}


void merge_novel_views_mainlayer(novel_view_t* nvs, int nbr, const float fB, const float dthresh,
	int width, int height, unsigned char* mnv, pixel_label* olabel, float* orange,float* dist,int* flag)
{
	int x, y;
	float MINNum = 0;

	float dist_weight[2] = {
		expf(-dist[0] / 5.f),
		expf(-dist[1] / 5.f)
	};
	if (flag == 1) { dist_weight[0] = 1, nbr = 1; }
	float rgbt[3];
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			float bgr[3] = { 0.f, 0.f, 0.f };
			float sumW = 0.f, sumR = 0.f, MINR = 1e10f;
			int  is_hole = 1;

			//遍历两个参考视点
			for (int i = 0; i < nbr; i++)
			{	//如果一个纹理图该像素位置为空洞，则跳过该纹理图
				if (nvs[i].mlabel_img[y * width + x] != Main)////非深度图后景边缘区域的处理
				{
					continue;                           
				}
				
				float range = nvs[i].mrange_img[y * width + x];
				float conf = nvs[i].mconf[y * width + x] * dist_weight[i];
				sumW += conf;
				sumR += range;
				MINR = range < MINR ? range : MINR;
				//权重乘以该参考视点的
				for (int chan = 0; chan < 3; chan++)
				{
					bgr[chan] += (conf * nvs[i].mnovel_view[y * width * 3 + x * 3 + chan]);
				}

			}
			//ori
			if (sumW < 1e-5f) { continue; }

			for (int chan = 0; chan < 3; chan++)
			{
				mnv[y * width * 3 + x * 3 + chan] = (unsigned char)(MAX(0.f, MIN(255.f, bgr[chan] / sumW)));
			}

			olabel[y * width + x] = Main;
			orange[y * width + x] = MINR;// sumR / float(sumN);

		}
}


void bilateral_hole_filling(unsigned char* bgr, float* irange, float* orange, pixel_label* ilabels, pixel_label* olabels, int width, int height, int kradius, float fB, float sigd)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{

			if (ilabels[y * width + x] != Hole)
			{
				olabels[y * width + x] = ilabels[y * width + x];
				orange[y * width + x] = irange[y * width + x];

				continue;
			}

			float MINdisp = 1e10f;
			int MINdist = 0;
			// -- move along 8 directions to find the smallest disparity
			for (int movx = x + 1; movx < width; movx++)
			{
				if (ilabels[y * width + movx] != Hole)
				{
					float disp = fB / irange[y * width + movx];
					if (disp < MINdisp)
					{
						MINdisp = disp; MINdist = abs(movx - x);
					}
					break;
				}
			} // -- move horizontally

			for (int movx = x - 1; movx >= 0; movx--)
			{
				if (ilabels[y * width + movx] != Hole)
				{
					float disp = fB / irange[y * width + movx];
					if (disp < MINdisp)
					{
						MINdisp = disp; MINdist = abs(movx - x);
					}
					break;
				}
			} // -- move horizontally

			for (int movy = y + 1; movy < height; movy++)
			{
				if (ilabels[movy * width + x] != Hole)
				{
					float disp = fB / irange[movy * width + x];
					if (disp < MINdisp)
					{
						MINdisp = disp; MINdist = abs(movy - y);
					}
					break;
				}
			} // -- move vertically

			for (int movy = y - 1; movy >= 0; movy--)
			{
				if (ilabels[movy * width + x] != Hole)
				{
					float disp = fB / irange[movy * width + x];
					if (disp < MINdisp)
					{
						MINdisp = disp; MINdist = abs(movy - y);
					}
					break;
				}
			} // -- move vertically

			if (MINdisp > 1e9f) { continue; } // -- some kind of exception ???

			float sumC[] = { 0.f, 0.f, 0.f };
			float sumW = 0.f;
			const float sigs = (float)(kradius * 1.2f);
			for (int i = -kradius; i <= kradius; i++) {
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = MAX(0, MIN(y + i, height - 1));
					int imx = MAX(0, MIN(x + j, width - 1));
					if (ilabels[imy * width + imx] == Hole)
					{
						continue;
					}
					float dist = sqrtf((float)(i * i + j * j));
					float ws = expf(-dist / sigs);

					float disp = fB / irange[imy * width + imx];
					float wd = expf(-fabs(disp - MINdisp) / sigd);

					for (int c = 0; c < 3; c++)
					{
						float d = bgr[imy * width * 3 + imx * 3 + c];
						sumC[c] += (ws * wd * d);
					}
					sumW += (ws * wd);
				}
			} // -- disparity based joint bilateral filtering

			//if (sumW < 1e-5f) { return; }
			if (sumW < 1e-5f) { continue; }
			for (int c = 0; c < 3; c++)
			{
				bgr[y * width * 3 + x * 3 + c] = sumC[c] / sumW;
			}
			orange[y * width + x] = fB / MINdisp;
			olabels[y * width + x] = Foreground_boundary; // -- set to forground edge so that it will be smoothed later

		}
}

int fill_holes_with_background_color(unsigned char* img, pixel_label* labels, pixel_label* tlabels, float* range, float* trange,
	float fB, int width, int height, float sigd)

{
	int kradius = 30;     //30

	int base_param = 20;    //wkj  fixed parameter

	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			//如果该像素点是空洞，则取空洞附近kradius个像素点的深度值的加权和作为空洞点深度值
			if (labels[y * width + x] == Hole)
			{
				float sumC[] = { 0.f, 0.f, 0.f };
				float sumW = 0.f;
				const float sigs = (float)(kradius * 1.2f);
				for (int i = -kradius; i <= kradius; i++)
				{
					for (int j = -kradius; j <= kradius; j++)
					{
						int imy = MAX(0, MIN(y + i, height - 1));
						int imx = MAX(0, MIN(x + j, width - 1));
						if (labels[imy * width + imx] == Hole)
						{
							continue;
						}
						float dist = sqrtf((float)(i * i + j * j));
						float ws = expf(-dist / sigs);

						float disp = fB / range[imy * width + imx];
						float wd = expf(-fabs(disp - base_param) / sigd);
						//float wd = 1.0;

						for (int c = 0; c < 3; c++)
						{
							float d = img[imy * width * 3 + imx * 3 + c];
							sumC[c] += (ws * wd * d);
						}
						sumW += (ws * wd);
					}
				} // -- disparity based joint bilateral filtering   根据视差图双边滤波
				for (int c = 0; c < 3; c++)
				{
					img[y * width * 3 + x * 3 + c] = sumC[c] / sumW;
				}

			}
		}

	return I2R_OK;
}


int fill_holes_with_background_color__(unsigned char* img, pixel_label* labels, pixel_label* tlabels, float* range, float* trange,
	float fB, int width, int height, float sigd)
{

	bilateral_hole_filling(img, range, trange, labels, tlabels, width, height, 15, fB, sigd);
	memset(labels, Hole, width * height * sizeof(pixel_label));
	memset(range, 0, width * height * sizeof(float));

	bilateral_hole_filling(img, trange, range, tlabels, labels, width, height, 15, fB, sigd);
	memset(tlabels, Hole, width * height * sizeof(pixel_label));
	memset(trange, 0, width * height * sizeof(float));

	bilateral_hole_filling(img, range, trange, labels, tlabels, width, height, 15, fB, sigd);
	memset(labels, Hole, width * height * sizeof(pixel_label));
	memset(range, 0, width * height * sizeof(float));

	bilateral_hole_filling(img, trange, range, tlabels, labels, width, height, 15, fB, sigd);


	return I2R_OK;
}


void expand_label_pixels(pixel_label* ilabels, pixel_label* olabels, pixel_label tarL, int width, int height, int kradius)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{

			int nearFB = 0;
			for (int i = -kradius; i <= kradius; i++)
			{
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = MAX(0, MIN(y + i, height - 1));
					int imx = MAX(0, MIN(x + j, width - 1));
					if (ilabels[imy * width + imx] == tarL)
					{
						nearFB = 1;
						break;
					}
				}
			} // -- disparity based joint bilateral filtering
			if (nearFB)
			{
				olabels[y * width + x] = tarL;
			}
			else
			{
				olabels[y * width + x] = ilabels[y * width + x];
			}
		}

}


void smooth_foreground(unsigned char* bgr, unsigned char* obgr, pixel_label* labels, int width, int height, int kradius, float sig)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			if (labels[y * width + x] != Foreground_boundary)
			{
				for (int c = 0; c < 3; c++)
				{
					obgr[y * width * 3 + x * 3 + c] = bgr[y * width * 3 + x * 3 + c];
				}

				continue;
			}

			float sumC[] = { 0.f, 0.f, 0.f };
			float sumW = 0.f;
			for (int i = -kradius; i <= kradius; i++)
			{
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = MAX(0, MIN(y + i, height - 1));
					int imx = MAX(0, MIN(x + j, width - 1));
					float dist = sqrtf((float)(i * i + j * j));
					float w = expf(-dist / sig);
					for (int c = 0; c < 3; c++)
					{
						float d = bgr[imy * width * 3 + imx * 3 + c];
						sumC[c] += (w * d);
					}
					sumW += w;
				}
			}
			sumW += 1e-5f; // -- avoid /0 error
			for (int c = 0; c < 3; c++)
			{
				obgr[y * width * 3 + x * 3 + c] = sumC[c] / sumW;
			}
		}
}

int merge_novel_views(vss_param* params, novel_view_t* nvs, int nbr, int width, int height, const float fB,
	 unsigned char* mnv, float*dist,int flag)
{
	unsigned char* mnvdata = NULL, *onvdata = NULL;
	float* mrange = NULL, *orange = NULL;
	pixel_label *mlabel = NULL, *expanded_label = NULL;
    float MINdepth = params->min_depth;
    float MAXdepth = params->max_depth;

	mnvdata = (unsigned char*)malloc(width * height * 3 * sizeof(unsigned char));
	onvdata = (unsigned char*)malloc(width * height * 3 * sizeof(unsigned char));
	mrange = (float*)malloc(width * height * sizeof(float));
	orange = (float*)malloc(width * height * sizeof(float));
	mlabel = (pixel_label*)malloc(width * height * sizeof(pixel_label));
	expanded_label = (pixel_label*)malloc(width * height * sizeof(pixel_label));

	float MAXdisp = fB / MINdepth, MINdisp = fB / MAXdepth;
	float drange = MAXdisp - MINdisp + 1;
	const float dtr = 0.01666667f; // -- disparity threashold ratio


	memset(mnvdata, 0, width * height * 3 * sizeof(unsigned char));
	memset(mlabel, Hole, width * height * sizeof(pixel_label));//初始化全是空洞点
	memset(mrange, 0, width * height * sizeof(float));
	//根据深度值对纹理图进行融合，权重加权
	merge_novel_views_mainlayer(nvs, nbr, fB, dtr * drange, width, height, mnvdata, mlabel, mrange,dist,flag);

	//add


	//memcpy(mnvdata, buf_color, width * height * 3 * sizeof(unsigned char));
	// -- post processing: filling holes/smoothing edges
	memset(expanded_label, Hole, width * height * sizeof(pixel_label));
	memset(orange, 0, width * height * sizeof(float));

	//填补深度空洞
	// -- fill the holes   (1) merge color,  generate mlable and mrange
	//mnvdata 混合后的纹理图merge novel view，填补纹理图空洞
	//expanded_label和orange都没用到
	fill_holes_with_background_color__(mnvdata, mlabel, expanded_label, mrange, orange, fB, width, height, dtr * drange);

	//标记前景边缘
	//(2)lable process
	const int edge_radius = 1;
	memset(mlabel, Hole, width * height * sizeof(pixel_label));
	label_boundary_pixels2(mrange, mlabel, Foreground_boundary, fB, dtr * drange, edge_radius, width, height);
	memset(expanded_label, Hole, width * height * sizeof(pixel_label));
	expand_label_pixels(mlabel, expanded_label, Foreground_boundary, width, height, 2); // -- expand foreground boundary
	////对纹理图的前景边缘进行滤波
	////(3) smooth
	memset(onvdata, 0, width * height * 3 * sizeof(unsigned char));
	smooth_foreground(mnvdata, onvdata, expanded_label, width, height, 2, 2.f); // -- smooth the foreground edges

	//// -- copy back to CPU
	memcpy(mnv, mnvdata, width * height * 3 * sizeof(unsigned char));    //copy intermedia result before smooth
	memcpy(mnv, onvdata, width * height * 3 * sizeof(unsigned char));  //copy final result after smooth


	free(mnvdata);
	free(onvdata);
	free(mrange);
	free(orange);
	free(mlabel);
	free(expanded_label);

	return I2R_OK;
}
#endif