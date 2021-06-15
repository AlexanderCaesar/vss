/*****************************************************************************
 * Copyright (C) 2021-2031 yangang cai
 *
 * Authors: caiyangang <caiyangang@pku.edu.cn>
 *****************************************************************************/
#include<stdio.h>
#include<stdlib.h>
#include "string.h"
#include "vss.h"
#include "file.h"
#include "parser.h"


#if defined WIN32
#include <io.h>
#define strcasecmp strcmpi
#else
#include <unistd.h>
#endif

#define MAX_ITEMS_TO_PARSE  10000

#define SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT (MAX_SFM_CAMNUM * (9 + 2 + 3 + 3 + 2 + 2) + 1)

#pragma warning(disable:4996)

enum MappingType
{
    MappingType_INT = 0,
    MappingType_STRING = 1,
    MappingType_FLOAT = 2,
    MappingType_UNKONWN = 3
};

typedef struct {
    char*  TokenName;
    void*  Place;
    int    Type;
    double Default;
    int    param_limits; //! 0: no limits, 1: both MIN and MAX, 2: only MIN (i.e. no negatives), 3: specialcase for QPs since MIN needs bitdepth_qp_scale
    double MIN_limit;
    double MAX_limit;  //we do not use param_limits, MIN_limit and MAX_limit.//wkj
} Mapping;

void no_mem_exit(char *where)
{
    printf("Could not allocate memory: %s", where);
}

char *GetConfigFileContent(char *Filename)
{
    long FileSize;
    FILE *f;
    char *buf;

    if (NULL == (f = fopen(Filename, "r")))
    {
        printf("Cannot open configuration file %s.", Filename);
        return NULL;
    }

    if (0 != fseek(f, 0, SEEK_END))
    {
        printf("Cannot fseek in configuration file %s.", Filename);
        return NULL;
    }

    FileSize = ftell(f);
    if (FileSize < 0 || FileSize > 60000)
    {
        printf("Unreasonable Filesize %ld reported by ftell for configuration file %s.", FileSize, Filename);
        return NULL;
    }
    if (0 != fseek(f, 0, SEEK_SET))
    {
        printf("Cannot fseek in configuration file %s.", Filename);
        return NULL;
    }

    if ((buf = malloc(FileSize + 1)) == NULL) no_mem_exit("GetConfigFileContent: buf");

    // Note that ftell() gives us the file size as the file system sees it.  The actual file size,
    // as reported by fread() below will be often smaller due to CR/LF to CR conversion and/or
    // control characters after the dos EOF marker in the file.

    FileSize = fread(buf, 1, FileSize, f);
    buf[FileSize] = '\0';

    fclose(f);
    return buf;
}

static vss_param     configinput;
vss_filepaths        file_path;

Mapping Map[] = {


    { "Color_Path",        &file_path.texture_path,        MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "Color_Path_Background",  &file_path.texture_path_bg,        MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "Depth_Path",        &file_path.depth_path,        MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "Depth_Path_Background", &file_path.depth_path_bg,        MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "Cam_Params_File",   &file_path.cam_params_file,   MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "Color_Output_File", &file_path.textrue_output_file, MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "Depth_Output_File", &file_path.depth_output_file, MappingType_STRING, 0.0,     0, 0.0, 0.0 },
    { "CamNum",         &configinput.cam_num, MappingType_INT, 30, 0, 0.0, 0.0 },
    { "SourceWidth",    &configinput.source_width,  MappingType_INT, 1920,         0, 0.0, 0.0 },
    { "SourceHeight",   &configinput.source_height, MappingType_INT, 1080,         0, 0.0, 0.0 },
    { "Vcam_krt_R_0", &configinput.krt_vcam.krt_R[0], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_1", &configinput.krt_vcam.krt_R[1], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_2", &configinput.krt_vcam.krt_R[2], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_3", &configinput.krt_vcam.krt_R[3], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_4", &configinput.krt_vcam.krt_R[4], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_5", &configinput.krt_vcam.krt_R[5], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_6", &configinput.krt_vcam.krt_R[6], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_7", &configinput.krt_vcam.krt_R[7], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_R_8", &configinput.krt_vcam.krt_R[8], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },

    { "Vcam_krt_WorldPosition_0", &configinput.krt_vcam.krt_WorldPosition[0], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_WorldPosition_1", &configinput.krt_vcam.krt_WorldPosition[1], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_WorldPosition_2", &configinput.krt_vcam.krt_WorldPosition[2], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },

    { "Vcam_krt_kc_0", &configinput.krt_vcam.krt_kc[0], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_kc_1", &configinput.krt_vcam.krt_kc[1], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_kc_2", &configinput.krt_vcam.krt_kc[2], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },

    { "Vcam_krt_cc_0", &configinput.krt_vcam.krt_cc[0], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "Vcam_krt_cc_1", &configinput.krt_vcam.krt_cc[1], MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "lens_fov", &configinput.krt_vcam.lens_fov, MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },
    { "fisheye_radius", &configinput.krt_vcam.fisheye_radius, MappingType_FLOAT, 0.0, 0, 0.0, 0.0 },

    { "Vcam_src_width", &configinput.krt_vcam.width, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Vcam_src_height", &configinput.krt_vcam.height, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Depth_scale_type", &configinput.depth_scale_type, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Background_flag", &configinput.background_flag, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Background_th", &configinput.background_th, MappingType_INT, 9, 0, 0.0, 0.0 },
    { "Depth_scale_th", &configinput.depth_scale_th, MappingType_FLOAT, 0.5, 0, 0.0, 0.0 },
    { "Depth_mf_radius", &configinput.depth_mf_radius, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Depth_mf_radius", &configinput.depth_mf_radius, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Depth_mf_radius", &configinput.depth_mf_radius, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Left_view", &configinput.left_view , MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Right_view", &configinput.right_view, MappingType_INT, 0, 0, 0.0, 0.0 },
    { "Model", &configinput.model, MappingType_INT, 0, 0, 0.0, 0.0 },
    { NULL, NULL, MappingType_UNKONWN, 0.0, 0, 0.0, 0.0 }
};


static int ParameterNameToMapIndex(char *s)
{
    int i = 0;

    while (Map[i].TokenName != NULL)
        if (0 == strcasecmp(Map[i].TokenName, s))
            return i;
        else
            i++;
    return -1;
};


void ParseContent(char *buf, int bufsize, vss_param* params)
{
    char *items[MAX_ITEMS_TO_PARSE];
    int MapIdx;
    int item = 0;
    int InString = 0, InItem = 0;
    char *p = buf;
    char *bufend = &buf[bufsize];
    int IntContent;
    float FloatContent;
    int i;

    // Stage one: Generate an argc/argv-type list in items[], without comments and whitespace.
    // This is context insensitive and could be done most easily with lex(1).

    while (p < bufend)
    {
        switch (*p)
        {
        case 13:
            p++;
            break;
        case '#':                 // Found comment
            *p = '\0';              // Replace '#' with '\0' in case of comment immediately following integer or string
            while (*p != '\n' && p < bufend)  // Skip till EOL or EOF, whichever comes first
                p++;
            InString = 0;
            InItem = 0;
            break;
        case '\n':
            InItem = 0;
            InString = 0;
            *p++ = '\0';
            break;
        case ' ':
        case '\t':              // Skip whitespace, leave state unchanged
            if (InString)
                p++;
            else
            {                     // TerMINate non-strings once whitespace is found
                *p++ = '\0';
                InItem = 0;
            }
            break;

        case '"':               // Begin/End of String
            *p++ = '\0';
            if (!InString)
            {
                items[item++] = p;
                InItem = ~InItem;
            }
            else
                InItem = 0;
            InString = ~InString; // Toggle
            break;

        default:
            if (!InItem)
            {
                items[item++] = p;
                InItem = ~InItem;
            }
            p++;
        }
    }

    item--;

    for (i = 0; i < item; i += 3)
    {
        MapIdx = ParameterNameToMapIndex(items[i]);
        if(MapIdx < 0 )
        {
            printf("unkonwn type %s\n", items[i]);
            continue;
        }
        if (strcasecmp("=", items[i + 1]))
        {
            //error(errortext, 300);
        }
        // Now interpret the Value, context sensitive...

        switch (Map[MapIdx].Type)
        {
        case MappingType_INT:           // Numerical
            if (1 != sscanf(items[i + 2], "%d", &IntContent))
            {
                //error(errortext, 300);
            }
            *(int *)(Map[MapIdx].Place) = IntContent;
            break;
        case MappingType_STRING:
            strncpy((char *)Map[MapIdx].Place, items[i + 2], FILE_NAME_SIZE);
            break;
        case MappingType_FLOAT:           // Numerical double
            if (1 != sscanf(items[i + 2], "%f", &FloatContent))
            {
                //error(errortext, 300);
            }
            *(float *)(Map[MapIdx].Place) = FloatContent;
            break;
        default:
            printf("Unknown value type in the map definition of configfile.h");
        }
    }
}

int set_sfm_parameters(float* save_data, vss_param* params)
{
    // -- this function init sfm solver by loaded data

    for (int i = 0; i < MAX_SFM_CAMNUM; i++)
    {
        memcpy((float*)params->krt_rcam[i].krt_R, save_data, 9 * sizeof(float)); // -- copy rotation matrix
        save_data += 9;
        memcpy(params->krt_rcam[i].krt_cc, save_data, 2 * sizeof(float));
        save_data += 2;

        memcpy(params->krt_rcam[i].krt_WorldPosition, save_data, 3 * sizeof(float)); // -- copy position
        save_data += 3;

        memcpy(params->krt_rcam[i].krt_kc, save_data, 3 * sizeof(float)); // -- copy kc
        save_data += 3;

        if (i < params->cam_num)
        {
            int* int_save_data = (int*)save_data;
            if ((params->krt_rcam[i].width != int_save_data[0]) || (params->krt_rcam[i].height != int_save_data[1]))
            {
                printf("error: sfm calibration image resolution(%d x %d) is not compatible with the loaded "
                    "image(%d x %d) for camera %d;\n", int_save_data[0], int_save_data[1], params->krt_rcam[i].width, params->krt_rcam[i].height, i);
            }

        }
        save_data += 2;
        params->krt_rcam[i].fisheye_radius = save_data[0];
        params->krt_rcam[i].lens_fov = save_data[1];
        save_data += 2;
    }
    int* int_save_data = (int*)save_data;
    save_data++;
    save_data -= SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT;

    // -- printf sfm data
    for (int m = 0; m < params->cam_num; m++)
    {
        params->krt_rcam[m].lens_fov = (float)params->krt_vcam.lens_fov;//60.60 * CV_PI / 360;  72.06* CV_PI / 360
    }
    return I2R_OK;
}

int init_with_sfm_file(char* filename, vss_param* params)
{
    if (!filename) { printf("error: can not open sfm file %s\n", filename); return I2R_ERR; }

    // -- sizeof(int) == sizeof(float)
    float load_data_array[SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT];
    float *load_data = &load_data_array[0];
    FILE* file = fopen(filename, "rb"); // 某个sfm文件
    if (file) {
        fread(&(params->min_depth), sizeof(float), 1, file);   //MINdepth and MAXdepth are placed at first two float space.
        fread(&(params->max_depth), sizeof(float), 1, file);
        size_t read_count = fread(load_data, sizeof(float), SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT, file);

        fclose(file);
        if (read_count != SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT) {
            printf("error: sfm calibration file size not compatible\n");
            return I2R_ERR;
        }
    }
    else { printf("error: can not open sfm calibration file\n"); return I2R_ERR; }

    set_sfm_parameters(load_data, params); // -- set calibration data with the loaded data

    return I2R_OK;
}

void printf_params(vss_param* params)
{
    printf("********************************************************\n");
    printf("CamNum           :    %d\n", params->cam_num);
    printf("SourceWidth      :    %d\n",params->source_width);
    printf("SourceHeight     :    %d\n", params->source_height);
    printf("Left_view        :    %d\n", params->left_view);
    printf("Right_view       :    %d\n", params->right_view);
    printf("Model            :    %d\n", params->model);
    printf("vcamWorldPosition:    (%f,%f,%f)\n", params->krt_vcam.krt_WorldPosition[0], params->krt_vcam.krt_WorldPosition[1], params->krt_vcam.krt_WorldPosition[2]);
    printf("lens_fov         :    %f\n", params->krt_vcam.lens_fov);
    printf("fisheye_radius   :    %f\n", params->krt_vcam.fisheye_radius);
    printf("Depth_scale_type :    %d\n", params->depth_scale_type);
    printf("Background_flag  :    %d\n", params->background_flag);
    printf("Background_th    :    %d\n", params->background_th);
    printf("Depth_scale_th   :    %f\n", params->depth_scale_th);
    printf("Depth_mf_radius  :    %d\n", params->depth_mf_radius);
    printf("MIN_depth        :    %f\n", params->min_depth);
    printf("MAX_depth        :    %f\n", params->max_depth);
}

void printf_file_path(vss_filepaths* path)
{
    printf("Color_Path       :    %s\n", path->texture_path);
    printf("Color_Background :    %s\n", path->texture_path_bg);
    printf("Depth_Path       :    %s\n", path->depth_path);
    printf("Depth_Background :    %s\n", path->depth_path_bg);
}

int Configure(int ac, char *av[], vss_param* params,vss_files* files)
{
    char*  content;
    char*  filename = "";

    int    IntContent;
    float  FloatContent;

    memset(params, 0, sizeof(vss_param));

    if (ac <= 2)
    {
        printf("not enough params\n");
        return -1;
    }

    for (int i = 1; i < ac; i += 2)
    {
        if (0 == strncmp(av[i], "-d", 2))
        {
            filename = av[i+1];
            content = GetConfigFileContent(filename);
            ParseContent(content, strlen(content), params);
            continue;
        }
        int map_idx = ParameterNameToMapIndex(av[i]+1);

        if (map_idx < 0)
        {
            printf("unkonwn type %s\n", av[i]);
            continue;
        }

        switch (Map[map_idx].Type)
        {
        case MappingType_INT:           // Numerical
            if (1 != sscanf(av[i + 1], "%d", &IntContent))
            {
                //error(errortext, 300);
            }
            *(int *)(Map[map_idx].Place) = IntContent;
            printf(".");
            break;
        case MappingType_STRING:
            strncpy((char *)Map[map_idx].Place, av[i + 1], FILE_NAME_SIZE);
            printf(".");
            break;
        case MappingType_FLOAT:           // Numerical double
            if (1 != sscanf(av[i + 1], "%f", &FloatContent))
            {
                //error(errortext, 300);
            }
            *(float *)(Map[map_idx].Place) = FloatContent;
            printf(".");
            break;
        default:
            printf("Unknown value type in the map definition of configfile.h");
        }

    }

    free(content);

    for (int i = 0; i < MAX_SFM_CAMNUM; i++)
    {
        configinput.krt_rcam[i].width = configinput.source_width;
        configinput.krt_rcam[i].height = configinput.source_height;
    }

    memcpy(params, &configinput, sizeof(vss_param));

    init_with_sfm_file(file_path.cam_params_file, params);
    vss_files_set_path(files, &file_path, params);
    printf_params(params);
    printf_file_path(&file_path);
   
    return 0;
}
