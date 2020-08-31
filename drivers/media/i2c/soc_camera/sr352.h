#ifndef SR352_H_
#define SR352_H_

#include <linux/types.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

/* The token to indicate array termination */
#define SR352_TERM			0xff
#define SR352_OUT_WIDTH_DEF		640
#define SR352_OUT_HEIGHT_DEF		480
#define SR352_WIN_WIDTH_MAX		2048
#define SR352_WIN_HEIGHT_MAX		1536
#define SR352_WIN_WIDTH_MIN		8
#define SR352_WIN_HEIGHT_MIN		8

enum sr352_res_support {
	//From Enter preview resolution
	SR352_FMT_VGA = 0,
	SR352_FMT_720X480,
	SR352_FMT_800X480,
	SR352_FMT_704X576,
	SR352_FMT_1024X576,
	SR352_FMT_1024X640,
	SR352_FMT_1024X768,
	//special recording & preview
	SR352_FMT_176X144,
	SR352_FMT_320X240,
	SR352_FMT_352X288,
	SR352_FMT_800X600,
	SR352_FMT_HD,
	//special capture
	// SR352_FMT_VGA, as it defined into preview
	// SR352_FMT_800X480, as it defined into preview
	SR352_FMT_960X720,
	SR352_FMT_1280X720,
	SR352_FMT_1280X960,
	SR352_FMT_1600X960,
	SR352_FMT_1600X1200,
	SR352_FMT_2048X1152,
	SR352_FMT_2048X1536,
	SR352_FMT_END,
};

struct sr352_format {
	enum v4l2_mbus_pixelcode	code;
	unsigned int fmt_support;
	struct sr352_regval	*regs;
	struct sr352_regval	*def_set;
};

struct sr352 {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler	hdl;
	struct v4l2_rect rect;
	u32 pixfmt;
	int frame_rate;
	struct i2c_client *client;
	struct soc_camera_device icd;
};

/* sr352 has only one fixed colorspace per pixelcode */
struct sr352_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct sr352_resolution_table {
	int width;
	int height;
	enum sr352_res_support res_code;
};

struct sr352_resolution_table sr352_resolutions[] = {
	{ 640,  480, SR352_FMT_VGA},	
	{ 720,  480, SR352_FMT_720X480},	
	{ 800,  480, SR352_FMT_800X480},
	{ 704,  576, SR352_FMT_704X576},
	{ 1024,  640, SR352_FMT_1024X640},
	{ 1024,  768, SR352_FMT_1024X768,},
	/**/
	{ 176,  144, SR352_FMT_176X144},	
	{ 320,  240, SR352_FMT_320X240},	
	{ 352,  288, SR352_FMT_352X288},	
	{ 800,  600, SR352_FMT_800X600},	
	{ 1280,  720, SR352_FMT_HD},
	/**/
	{ 960,  720, SR352_FMT_960X720},  
	{ 1280,  960, SR352_FMT_1280X960}, 
	{ 1600,  960, SR352_FMT_1600X960}, 
	{ 1600,  1200,  SR352_FMT_1600X1200},
	{ 2048,  1152, SR352_FMT_2048X1152},
	{ 2048,  1536, SR352_FMT_2048X1536},
};

/*
 * this define for normal preview and camcorder different setting
 * but same resolution
 */
#define NORMAL_STATUS 0
#define VIDEO_TO_NORMAL 1
#define NORMAL_TO_VIDEO 2
#define VIDEO_TO_CALL 3
#define CAPTURE_TO_NORMAL 4
#define NORMAL_TO_CAPTURE 5
#define V4L2_CID_PRIVATE_SR352_SENSOR_MODE \
	(V4L2_CID_CAMERA_CLASS_BASE + 0x1001)
#endif
