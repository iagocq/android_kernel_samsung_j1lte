#ifndef SR130PC20M_H_
#define SR130PC20M_H_

#include <linux/types.h>
#include <media/v4l2-common.h>
#include <media/soc_camera.h>

/* The token to indicate array termination */
#define SR130PC20M_TERM			0xff
#define SR130PC20M_OUT_WIDTH_DEF		640
#define SR130PC20M_OUT_HEIGHT_DEF		480
#define SR130PC20M_WIN_WIDTH_MAX		1600
#define SR130PC20M_WIN_HEIGHT_MAX		1200
#define SR130PC20M_WIN_WIDTH_MIN		8
#define SR130PC20M_WIN_HEIGHT_MIN		8


enum sr130pc20m_res_support {
	SR130PC20M_FMT_VGA = 0,
	SR130PC20M_FMT_1_3M,
	SR130PC20M_FMT_176X144,
	SR130PC20M_FMT_352X288,
	SR130PC20M_FMT_320X240,
	SR130PC20M_FMT_528X432,
	SR130PC20M_FMT_END,
};

struct sr130pc20m_format {
	enum v4l2_mbus_pixelcode	code;
	unsigned int fmt_support;
	struct sr130pc20m_regval	*regs;
	struct sr130pc20m_regval	*def_set;
};

struct sr130pc20m {
	struct v4l2_subdev subdev;
	struct v4l2_ctrl_handler	hdl;
	struct v4l2_rect rect;
	u32 pixfmt;
	int frame_rate;
	struct i2c_client *client;
	struct soc_camera_device icd;
};

/* sr130pc20m has only one fixed colorspace per pixelcode */
struct sr130pc20m_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct sr130pc20m_resolution_table {
	int width;
	int height;
	enum sr130pc20m_res_support res_code;
};


struct sr130pc20m_resolution_table sr130pc20m_resolutions[] = {
	{ 640,  480, SR130PC20M_FMT_VGA},	/* VGA */
	{1280,  960, SR130PC20M_FMT_1_3M},	/* 2M */
	{176,  144, SR130PC20M_FMT_176X144},	/* 176x144 */
	{352,  288, SR130PC20M_FMT_352X288},	/* 352x288 */
	{320,  240, SR130PC20M_FMT_320X240},	/* 320x240 */
	{528,  432, SR130PC20M_FMT_528X432},	/* 528x432 */
};

/*this define for normal preview and camcorder different setting
 * but same resolution*/
#define NORMAL_STATUS 0
#define VIDEO_TO_NORMAL 1
#define NORMAL_TO_VIDEO 2
#define VIDEO_TO_CALL 3
#define V4L2_CID_PRIVATE_SR130PC20M_VIDEO_MODE \
	(V4L2_CID_CAMERA_CLASS_BASE + 0x1001)
#endif
