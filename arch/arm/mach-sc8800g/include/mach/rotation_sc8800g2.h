#ifndef _ROTATION_SC8800G2_H_
#define _ROTATION_SC8800G2_H_

typedef enum
{
    ROTATION_YUV422 = 0,
    ROTATION_YUV420,
    ROTATION_YUV400,
    ROTATION_RGB888,
    ROTATION_RGB666,
    ROTATION_RGB565,
    ROTATION_RGB555,
    ROTATION_MAX
} ROTATION_DATA_FORMAT_E;

typedef enum
{
    ROTATION_90 = 0,
    ROTATION_270,
    ROTATION_180,
    ROTATION_MIRROR,
    ROTATION_DIR_MAX
} ROTATION_DIR_E;

typedef struct _rotation_size_tag
{
    uint16_t w;
    uint16_t h;
} ROTATION_SIZE_T;

typedef struct _ROTATION_data_addr_tag
{
    uint32_t y_addr;
    uint32_t uv_addr;
    uint32_t v_addr;
} ROTATION_DATA_ADDR_T;
typedef struct _rotation_tag
{
    ROTATION_SIZE_T         img_size;
    ROTATION_DATA_FORMAT_E   data_format;
    ROTATION_DIR_E         rotation_dir; 
    ROTATION_DATA_ADDR_T    src_addr;
    ROTATION_DATA_ADDR_T    dst_addr;     
}ROTATION_PARAM_T, *ROTATION_PARAM_T_PTR;

int rotation_start(ROTATION_PARAM_T* param_ptr);

#define SC8800G_ROTATION_IOCTL_MAGIC 'm'
#define SC8800G_ROTATION_DONE _IOW(SC8800G_ROTATION_IOCTL_MAGIC, 1, unsigned int)

#endif //_ROTATION_SC8800G2_H_
