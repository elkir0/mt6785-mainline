// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6785 CAMSV0 raw-frame capture driver (V4L2 video node).
 *
 * CAMSV0 is the "sensor viewfinder" DMA pipe of the MT6785 camera system:
 * CAM_MUX 2 -> TG (timing generator) -> PAK (packer) -> IMGO DMA into
 * physically contiguous memory.  With the measured FMT_SEL = 0x102 /
 * PAK = 1 configuration (docs/hardware/CAMERA.md "Frame layout and
 * first photo") each line is a continuous little-endian 10-bit bit
 * stream (pixel k in bits 10k..10k+9), bytes per line = width * 10 / 8.
 *
 * PIXEL FORMAT MISMATCH: no V4L2 fourcc documents this exact bit
 * layout.  The node exposes the closest packed 10-bit Bayer format,
 * V4L2_PIX_FMT_S*10P (same bytes-per-line and frame size), whose
 * documented layout (4 pixels per 5 bytes, MSBs first) is DIFFERENT.
 * Userspace must decode with tools/camera/raw2rgb.py --layout le10;
 * see docs/research/camera/T8-V4L2.md section 3.
 *
 * The register sequence below is the one validated on the Lenovo Yoga
 * Tab 11 by tools/camera/ov-cap.py (setup_camsv0() with --layout le10)
 * and the begonia bring-up (src/begonia-ftp/camera/NOTES.md), with FBC
 * re-armed per frame (FBC_EN then RCNT_INC, not RCNT_INC alone).
 *
 * IOMMU: the experiments run CAMSV DMA with the larb6/larb7 SMI MMU
 * bypassed and a plain CMA physical address; this driver keeps that
 * model (vb2_dma_contig + a one-time MMU_EN clear on both LARBs while
 * streaming).  Proper IOMMU support (mapping larb ports into the M4U) is
 * future work; see docs/research/camera/T8-V4L2.md.
 *
 * Structure modeled on the BayLibre MT8365 ISP3.0 camsv driver
 * (mtk_camsv_hw.c / mtk_camsv_video.c, "Add Mediatek ISP3.0" v8 series)
 * with the MT6785 register layout (single 0x1000 window, TOP +0x510,
 * TG +0x220, IMGO +0x20, FBC 0x110/0x114).
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "mt6785-cam.h"

/* ---- CAMSV0 registers (window 0x1a050000, from ov-cap.py) ---- */

#define CAMSV_IMGO_BASE_ADDR		0x020
#define CAMSV_IMGO_XSIZE		0x030
#define CAMSV_IMGO_YSIZE		0x034
#define CAMSV_IMGO_STRIDE		0x038
#define CAMSV_IMGO_CON2			0x040
#define CAMSV_IMGO_CON3			0x044
#define CAMSV_FBC_IMGO_CTL1		0x110
#define CAMSV_FBC_IMGO_CTL2		0x114
#define CAMSV_TG_SEN_MODE		0x230
#define CAMSV_TG_VF_CON			0x234
#define CAMSV_TG_SEN_GRAB_PXL		0x238
#define CAMSV_TG_SEN_GRAB_LIN		0x23c
#define CAMSV_TG_FRMSIZE			0x268
#define CAMSV_DMA_EN			0x400
#define CAMSV_MODULE_EN			0x510
#define CAMSV_FMT_SEL			0x514
#define CAMSV_INT_EN			0x518
#define CAMSV_INT_STATUS		0x51c
#define CAMSV_SW_CTL			0x520
#define CAMSV_CLK_EN			0x530
#define CAMSV_PAK				0x54c

#define CAMSV_FBC_EN			BIT(15)
#define CAMSV_FBC_RCNT_INC		BIT(0)
#define CAMSV_IRQ_PASS1_DON		BIT(10)
#define CAMSV_IRQ_TG_ERR		BIT(4)
#define CAMSV_IRQ_IMGO_ERR		BIT(16)

/* SMI LARB MMU bypass (same registers as tools/camera/larb-bypass.py). */
#define SMI_LARB_NONSEC_CON(port)	(0x380 + 4 * (port))
#define LARB6_NPORTS			31
#define LARB7_NPORTS			5

#define MT6785_CAMSV_AUTOSUSPEND_MS	2000

/* Single-plane capture, min buffers vb2 requires before streaming. */
#define MT6785_CAMSV_MIN_BUFFERS		2

/**
 * struct mt6785_camsv_buffer - vb2 buffer with its DMA address
 * @vb: vb2 buffer (with v4l2 buffer header)
 * @list: entry in the driver's capture queue
 * @dma_addr: physical (bypassed-MMU) address of the plane
 */
struct mt6785_camsv_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
};

/**
 * struct mt6785_camsv - CAMSV0 capture device
 * @dev: platform device
 * @base: mapped CAMSV0 registers
 * @larb6, @larb7: mapped SMI LARB registers for the MMU bypass
 * @clks: bulk clocks (cam, camsys gates, top seninf muxes, APMIXED 26 MHz)
 * @num_clks: number of entries in @clks
 * @sd: CAMSV subdev (sink from SENINF, source to the video node)
 * @pads: subdev pads: 0 sink (from seninf), 1 source (to video node)
 * @vdev: registered /dev/videoN capture node
 * @vbq: vb2 queue of @vdev
 * @vdev_lock: serializes format ioctls against the queue
 * @format: current V4L2 pixel format
 * @seninf_pad: remote SENINF pad, resolved when streaming starts
 * @pipeline: media pipeline handle
 * @buf_list_lock: protects @buf_list
 * @buf_list: queued capture buffers
 * @sequence: completed frame counter
 * @streaming: streaming state
 */
struct mt6785_camsv {
	struct device *dev;
	void __iomem *base;
	void __iomem *larb6;
	void __iomem *larb7;

	struct clk_bulk_data *clks;
	unsigned int num_clks;

	struct v4l2_subdev sd;
	struct media_pad pads[2];
	struct media_pad vdev_pad;
	struct video_device vdev;
	struct vb2_queue vbq;
	struct mutex vdev_lock;
	struct v4l2_pix_format format;

	struct media_pad *seninf_pad;
	struct media_pipeline pipeline;

	spinlock_t buf_list_lock;
	struct list_head buf_list;
	unsigned int sequence;
	bool streaming;
};

static inline struct mt6785_camsv *sd_to_camsv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt6785_camsv, sd);
}

static inline struct mt6785_camsv *vb2q_to_camsv(struct vb2_queue *vq)
{
	return vb2_get_drv_priv(vq);
}

static inline struct mt6785_camsv *vdev_to_camsv(struct video_device *vdev)
{
	return video_get_drvdata(vdev);
}

static inline struct mt6785_camsv_buffer *to_camsv_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct mt6785_camsv_buffer, vb);
}

static inline u32 camsv_rd(struct mt6785_camsv *c, u32 off)
{
	return readl(c->base + off);
}

static inline void camsv_wr(struct mt6785_camsv *c, u32 off, u32 val)
{
	writel(val, c->base + off);
}

/* --------------------------------------------------------------------------
 * Formats: 10-bit Bayer.  The sensor's media bus code picks the Bayer
 * pattern; the fourcc kept is the closest packed 10-bit format
 * (V4L2_PIX_FMT_S*10P, same geometry) — the actual byte layout is the
 * continuous LE 10-bit bitstream described in the file header (decode
 * with tools/camera/raw2rgb.py --layout le10, NOT a S*10P decoder).
 */

struct mt6785_camsv_format {
	u32 code;		/* media bus code */
	u32 fourcc;		/* V4L2 pixel format */
};

static const struct mt6785_camsv_format camsv_formats[] = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10P },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10P },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10P },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10P },
};

static const struct mt6785_camsv_format *camsv_format_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++)
		if (camsv_formats[i].code == code)
			return &camsv_formats[i];

	return NULL;
}

static const struct mt6785_camsv_format *camsv_format_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++)
		if (camsv_formats[i].fourcc == fourcc)
			return &camsv_formats[i];

	return NULL;
}

static unsigned int camsv_bpl(unsigned int width)
{
	return ALIGN(DIV_ROUND_UP(width * MT6785_CAM_BPP, 8), 2);
}

/* --------------------------------------------------------------------------
 * Hardware sequence (ov-cap.py setup_camsv0(), lines 339-368)
 */

/* SW reset: 4 -> 0 -> 1 (wait == 3) -> 0, ov-cap.py lines 343-346. */
static int camsv_reset(struct mt6785_camsv *c)
{
	u32 val;
	int ret;

	camsv_wr(c, CAMSV_SW_CTL, 4);
	camsv_wr(c, CAMSV_SW_CTL, 0);
	camsv_wr(c, CAMSV_SW_CTL, 1);
	ret = readl_poll_timeout(c->base + CAMSV_SW_CTL, val, val == 3,
				 1000, 100000);
	if (ret)
		dev_err(c->dev, "CAMSV0 SW reset timeout (SW_CTL 0x%08x)\n", val);
	camsv_wr(c, CAMSV_SW_CTL, 0);

	return ret;
}

/*
 * ov-cap.py setup_camsv0() register block with the measured le10 layout
 * (--layout le10, docs/hardware/CAMERA.md "Frame layout and first
 * photo"): FMT_SEL = 0x102 and PAK = 1 with the line sized for 10 bits
 * give complete frames as a continuous little-endian 10-bit bit stream
 * per line (pixel k in bits 10k..10k+9).  The begonia values
 * (FMT_SEL = 0x101 / PAK = 2) write 12-bit containers and truncate each
 * line to 2/3 of the pixels on this board.
 *
 * Split in two so the sensor can be started between the reset and the
 * TG enable (the --stream-first order validated on this tablet):
 * camsv_hw_config() runs before the sensor streams, camsv_hw_enable()
 * arms the TG on an already-running receiver.
 */
static void camsv_hw_config(struct mt6785_camsv *c, dma_addr_t dma,
			    u32 w, u32 h, u32 bpl)
{
	camsv_wr(c, CAMSV_TG_SEN_MODE, 0);
	camsv_wr(c, CAMSV_TG_VF_CON, 0);
	camsv_wr(c, CAMSV_MODULE_EN, 0x40000000);	/* DB only */
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, 0);

	camsv_reset(c);

	camsv_wr(c, CAMSV_TG_SEN_GRAB_PXL, w << 16);
	camsv_wr(c, CAMSV_TG_SEN_GRAB_LIN, h << 16);
	camsv_wr(c, CAMSV_IMGO_BASE_ADDR, lower_32_bits(dma));
	camsv_wr(c, CAMSV_IMGO_XSIZE, bpl - 1);
	camsv_wr(c, CAMSV_IMGO_YSIZE, h - 1);
	/* Plain BPL: bit 24 packs two lines into one and halves the frame. */
	camsv_wr(c, CAMSV_IMGO_STRIDE, bpl);
	camsv_wr(c, CAMSV_IMGO_CON2, 0x20002);
	camsv_wr(c, CAMSV_IMGO_CON3, 0x20002);
	camsv_wr(c, CAMSV_FMT_SEL, 0x102);		/* measured le10 value */
	camsv_wr(c, CAMSV_PAK, 1);			/* measured le10 value */
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_EN);
	camsv_wr(c, CAMSV_CLK_EN, 0x8005);		/* TG | PAK | DMA */
	camsv_wr(c, CAMSV_MODULE_EN, 0x40000015);	/* DB|IMGO|PAK|TG */
	camsv_wr(c, CAMSV_INT_EN, 0xffffffff);
	camsv_wr(c, CAMSV_DMA_EN, 1);
}

static void camsv_hw_enable(struct mt6785_camsv *c)
{
	camsv_wr(c, CAMSV_TG_VF_CON, 1);		/* VFDATA_EN */
	camsv_wr(c, CAMSV_TG_SEN_MODE, 0x10001);	/* 1-pixel + CMOS_EN */
}

static void camsv_hw_stop(struct mt6785_camsv *c)
{
	camsv_wr(c, CAMSV_TG_SEN_MODE, 0);
	camsv_wr(c, CAMSV_TG_VF_CON, 0);
	camsv_wr(c, CAMSV_DMA_EN, 0);
	camsv_wr(c, CAMSV_MODULE_EN, 0x40000000);
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, 0);
	camsv_wr(c, CAMSV_INT_EN, 0);
}

/*
 * SMI LARB MMU bypass: same writes as tools/camera/larb-bypass.py
 * (SMI_LARB_NONSEC_CON = 0x380 + 4*port, clear MMU_EN bit 0) so the DMA
 * addresses of CMA buffers reach DRAM untranslated.  larb6's registers
 * are mapped through its DT node; larb7 has no DT node in this tree
 * yet, so its window comes from the CAMSV node's second reg tuple.
 * The LARB APB clocks (camsys CLK_CAM_LARB6/7) must be on: they are
 * part of this driver's clock list.
 *
 * FIXME: proper IOMMU support (mapping the camera LARB ports into the
 * M4U instead of bypassing it) is future work; see
 * docs/research/camera/T8-V4L2.md.
 */
static void camsv_larb_bypass(struct mt6785_camsv *c, bool enable)
{
	void __iomem *larbs[2] = { c->larb6, c->larb7 };
	unsigned int nports[2] = { LARB6_NPORTS, LARB7_NPORTS };
	unsigned int i, p;

	for (i = 0; i < 2; i++) {
		if (!larbs[i])
			continue;
		for (p = 0; p < nports[i]; p++)
			writel(enable ? 0 : 1, larbs[i] + SMI_LARB_NONSEC_CON(p));
	}
}

/*
 * Frame-done handling: complete the head queued buffer, hand the next
 * one to the DMA and pulse FBC RCNT_INC to release the slot (ov-cap.py
 * line 463; begonia NOTES.md FBC semantics).  Without a queued buffer
 * the hardware drops the frame until buf_queue runs again.
 */
static void camsv_buffer_done(struct mt6785_camsv *c)
{
	struct mt6785_camsv_buffer *done, *next;
	unsigned long flags;

	spin_lock_irqsave(&c->buf_list_lock, flags);
	done = list_first_entry_or_null(&c->buf_list,
					struct mt6785_camsv_buffer, list);
	if (done)
		list_del(&done->list);
	next = list_first_entry_or_null(&c->buf_list,
					struct mt6785_camsv_buffer, list);
	spin_unlock_irqrestore(&c->buf_list_lock, flags);

	if (!done) {
		if (next)
			camsv_wr(c, CAMSV_IMGO_BASE_ADDR,
				 lower_32_bits(next->dma_addr));
		return;
	}

	c->sequence++;
	done->vb.sequence = c->sequence;
	done->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&done->vb.vb2_buf, VB2_BUF_STATE_DONE);

	if (next)
		camsv_wr(c, CAMSV_IMGO_BASE_ADDR,
			 lower_32_bits(next->dma_addr));

	/*
	 * Re-arm FBC for the next frame.  Measured on this tablet
	 * (ov-cap.py arm_one_frame()): the full FBC_EN -> RCNT_INC
	 * sequence, not RCNT_INC alone — writing RCNT_INC on a running
	 * FBC does not latch another frame and the frame counter stalls.
	 */
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_EN);
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_EN | CAMSV_FBC_RCNT_INC);
}

static irqreturn_t camsv_irq(int irq, void *data)
{
	struct mt6785_camsv *c = data;
	u32 status = camsv_rd(c, CAMSV_INT_STATUS);

	if (!status)
		return IRQ_NONE;

	if (status & CAMSV_IRQ_IMGO_ERR)
		dev_err_ratelimited(c->dev,
				    "IMGO DMA error (INT_STATUS 0x%08x); is the LARB MMU bypass active?\n",
				    status);
	if (status & CAMSV_IRQ_TG_ERR)
		dev_dbg(c->dev, "TG error in frame (INT_STATUS 0x%08x)\n",
			status);

	if (status & CAMSV_IRQ_PASS1_DON)
		camsv_buffer_done(c);

	return IRQ_HANDLED;
}

/* --------------------------------------------------------------------------
 * vb2 queue operations
 */

static int camsv_vb2_queue_setup(struct vb2_queue *vq,
				 unsigned int *num_buffers,
				 unsigned int *num_planes,
				 unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct mt6785_camsv *c = vb2q_to_camsv(vq);
	unsigned int size = c->format.sizeimage;

	if (*num_planes == 0) {
		*num_planes = 1;
		sizes[0] = size;
	} else if (*num_planes != 1 || sizes[0] < size) {
		return -EINVAL;
	}

	alloc_devs[0] = c->dev;

	return 0;
}

static int camsv_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct mt6785_camsv *c = vb2q_to_camsv(vb->vb2_queue);
	struct mt6785_camsv_buffer *buf = to_camsv_buffer(vb);

	if (vb2_plane_size(vb, 0) < c->format.sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, c->format.sizeimage);
	buf->vb.field = V4L2_FIELD_NONE;
	buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	return 0;
}

static void camsv_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct mt6785_camsv *c = vb2q_to_camsv(vb->vb2_queue);
	struct mt6785_camsv_buffer *buf = to_camsv_buffer(vb);
	unsigned long flags;
	bool was_empty;

	spin_lock_irqsave(&c->buf_list_lock, flags);
	was_empty = list_empty(&c->buf_list);
	list_add_tail(&buf->list, &c->buf_list);
	spin_unlock_irqrestore(&c->buf_list_lock, flags);

	/*
	 * If the queue ran dry while streaming (a frame was dropped in
	 * camsv_buffer_done), point the DMA at the new buffer.
	 */
	if (c->streaming && was_empty)
		camsv_wr(c, CAMSV_IMGO_BASE_ADDR,
			 lower_32_bits(buf->dma_addr));
}

/* Return every queued buffer to vb2 (start failure / stop). */
static void camsv_return_all_buffers(struct mt6785_camsv *c,
				     enum vb2_buffer_state state)
{
	struct mt6785_camsv_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&c->buf_list_lock, flags);
	list_for_each_entry_safe(buf, tmp, &c->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&c->buf_list_lock, flags);
}

static int camsv_vb2_start_streaming(struct vb2_queue *vq,
				     unsigned int count)
{
	struct mt6785_camsv *c = vb2q_to_camsv(vq);
	struct v4l2_subdev *seninf_sd;
	struct mt6785_camsv_buffer *buf;
	int ret;

	if (!c->seninf_pad) {
		c->seninf_pad = media_pad_remote_pad_first(&c->pads[0]);
		if (!c->seninf_pad) {
			dev_err(c->dev,
				"no SENINF connected to sink pad 0; the media link is missing (notifier bound callback)\n");
			return -ENOLINK;
		}
	}

	seninf_sd = media_entity_to_v4l2_subdev(c->seninf_pad->entity);

	ret = video_device_pipeline_start(&c->vdev, &c->pipeline);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(c->dev);
	if (ret < 0)
		goto err_pipeline;

	/*
	 * LARB MMU bypass before any DMA (the LARB APB clocks are on from
	 * this driver's clock list, the CAM domain from runtime PM above).
	 */
	camsv_larb_bypass(c, true);

	/*
	 * The validated --stream-first order (ov-cap.py option and begonia
	 * NOTES.md "where we stopped"): configure CAMSV fully (reset,
	 * window, DMA, FBC, module enable) with the TG still off, then
	 * start SENINF + PHY + sensor, then arm the TG.  Arming the TG
	 * before the sensor streams was the failure mode that left IMGO
	 * stuck in IMGO_ERR on this tablet.
	 */
	buf = list_first_entry(&c->buf_list, struct mt6785_camsv_buffer, list);
	camsv_hw_config(c, buf->dma_addr, c->format.width, c->format.height,
			c->format.bytesperline);

	c->sequence = 0;

	ret = v4l2_subdev_call(seninf_sd, video, s_stream, 1);
	if (ret) {
		dev_err(c->dev, "failed to stream on SENINF: %d\n", ret);
		goto err_stop;
	}

	camsv_hw_enable(c);

	c->streaming = true;

	return 0;

err_stop:
	camsv_hw_stop(c);
	camsv_larb_bypass(c, false);
	pm_runtime_mark_last_busy(c->dev);
	pm_runtime_put_autosuspend(c->dev);
err_pipeline:
	/*
	 * Every buffer vb2 handed us must go back before returning an
	 * error, or vb2_start_streaming() WARNs (seen on the t8b boot).
	 */
	camsv_return_all_buffers(c, VB2_BUF_STATE_QUEUED);
	video_device_pipeline_stop(&c->vdev);

	return ret;
}

static void camsv_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct mt6785_camsv *c = vb2q_to_camsv(vq);
	struct v4l2_subdev *seninf_sd;

	if (c->seninf_pad) {
		seninf_sd = media_entity_to_v4l2_subdev(c->seninf_pad->entity);
		v4l2_subdev_call(seninf_sd, video, s_stream, 0);
	}

	c->streaming = false;
	camsv_hw_stop(c);
	camsv_larb_bypass(c, false);

	camsv_return_all_buffers(c, VB2_BUF_STATE_ERROR);

	video_device_pipeline_stop(&c->vdev);

	pm_runtime_mark_last_busy(c->dev);
	pm_runtime_put_autosuspend(c->dev);
}

static const struct vb2_ops camsv_vb2_ops = {
	.queue_setup = camsv_vb2_queue_setup,
	.buf_prepare = camsv_vb2_buf_prepare,
	.buf_queue = camsv_vb2_buf_queue,
	.start_streaming = camsv_vb2_start_streaming,
	.stop_streaming = camsv_vb2_stop_streaming,
};

/* --------------------------------------------------------------------------
 * V4L2 subdev ops (sink format from SENINF)
 */

/*
 * Default format on both pads: the validated capture geometry
 * (640x480 SBGGR10), matching the seninf default and the ov8856
 * 24 MHz mode so link validation passes on a fresh boot.
 */
static const struct v4l2_mbus_framefmt camsv_default_fmt = {
	.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	.width = 640,
	.height = 480,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
};

static int camsv_subdev_init_state(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_state_get_format(state, 0, 0);
	*fmt = camsv_default_fmt;
	fmt = v4l2_subdev_state_get_format(state, 1, 0);
	*fmt = camsv_default_fmt;

	return 0;
}

static int camsv_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(camsv_formats))
		return -EINVAL;

	code->code = camsv_formats[code->index].code;

	return 0;
}

static int camsv_subdev_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *sinkfmt;

	if (fmt->pad != 0)
		return -EINVAL;		/* sink only, pass-through */

	if (!camsv_format_by_code(fmt->format.code))
		fmt->format.code = camsv_default_fmt.code;

	fmt->format.field = V4L2_FIELD_NONE;
	sinkfmt = v4l2_subdev_state_get_format(state, 0, 0);
	*sinkfmt = fmt->format;

	/* Propagate to the source pad. */
	*v4l2_subdev_state_get_format(state, 1, 0) = *sinkfmt;

	return 0;
}

/*
 * Wrap the default link validation: the core only dev_dbg()s the
 * mismatch, which never reaches the log on a distro kernel — promote
 * it to dev_err() with the pads and both formats named, so a broken
 * pipeline is visible in dmesg instead of a bare -EPIPE from
 * STREAMON.
 */
static int camsv_link_validate(struct v4l2_subdev *sd,
			       struct media_link *link,
			       struct v4l2_subdev_format *source_fmt,
			       struct v4l2_subdev_format *sink_fmt)
{
	int ret;

	ret = v4l2_subdev_link_validate_default(sd, link, source_fmt,
						sink_fmt);
	if (ret)
		dev_err(sd->dev,
			"link %s:%u -> %s:%u format mismatch: source %ux%u code 0x%x field %u, sink %ux%u code 0x%x field %u\n",
			link->source->entity->name, link->source->index,
			link->sink->entity->name, link->sink->index,
			source_fmt->format.width, source_fmt->format.height,
			source_fmt->format.code, source_fmt->format.field,
			sink_fmt->format.width, sink_fmt->format.height,
			sink_fmt->format.code, sink_fmt->format.field);

	return ret;
}

static const struct v4l2_subdev_pad_ops camsv_subdev_pad_ops = {
	.enum_mbus_code = camsv_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = camsv_subdev_set_fmt,
	.link_validate = camsv_link_validate,
};

static const struct v4l2_subdev_ops camsv_subdev_ops = {
	.pad = &camsv_subdev_pad_ops,
};

/*
 * Entity ops are needed or media_pipeline_start() skips validating the
 * enabled link whose sink is this entity (seninf:3 -> camsv:0) — a
 * format mismatch there would go unnoticed.
 */
static const struct media_entity_operations camsv_media_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

/* --------------------------------------------------------------------------
 * Video node ioctls (single-plane)
 */

static int camsv_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strscpy(cap->driver, "mtk-mt6785-camsv", sizeof(cap->driver));
	strscpy(cap->card, "MT6785 CAMSV0", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 "mtk-mt6785-camsv.0");

	return 0;
}

static int camsv_enum_fmt(struct file *file, void *fh,
			  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(camsv_formats))
		return -EINVAL;

	f->pixelformat = camsv_formats[f->index].fourcc;

	return 0;
}

static void camsv_try_fmt(const struct mt6785_camsv *c,
			  struct v4l2_pix_format *pix)
{
	const struct mt6785_camsv_format *fmt;

	fmt = camsv_format_by_fourcc(pix->pixelformat);
	if (!fmt)
		pix->pixelformat = camsv_formats[0].fourcc;

	pix->width = clamp(pix->width, 16U, 4624U);
	pix->height = clamp(pix->height, 16U, 4032U);
	pix->bytesperline = camsv_bpl(pix->width);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->priv = 0;
}

static int camsv_try_fmt_ioctl(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	camsv_try_fmt(vdev_to_camsv(video_devdata(file)), &f->fmt.pix);

	return 0;
}

static int camsv_s_fmt_ioctl(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct mt6785_camsv *c = vdev_to_camsv(video_devdata(file));

	if (vb2_is_busy(&c->vbq))
		return -EBUSY;

	camsv_try_fmt(c, &f->fmt.pix);
	c->format = f->fmt.pix;

	return 0;
}

static int camsv_g_fmt_ioctl(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	f->fmt.pix = vdev_to_camsv(video_devdata(file))->format;

	return 0;
}

static int camsv_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *sizes)
{
	if (sizes->index)
		return -EINVAL;

	sizes->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	sizes->stepwise.min_width = 16;
	sizes->stepwise.max_width = 4624;
	sizes->stepwise.min_height = 16;
	sizes->stepwise.max_height = 4032;
	sizes->stepwise.step_width = 1;
	sizes->stepwise.step_height = 1;

	return 0;
}

static const struct v4l2_ioctl_ops camsv_ioctl_ops = {
	.vidioc_querycap = camsv_querycap,
	.vidioc_enum_fmt_vid_cap = camsv_enum_fmt,
	.vidioc_try_fmt_vid_cap = camsv_try_fmt_ioctl,
	.vidioc_s_fmt_vid_cap = camsv_s_fmt_ioctl,
	.vidioc_g_fmt_vid_cap = camsv_g_fmt_ioctl,
	.vidioc_enum_framesizes = camsv_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations camsv_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};


/*
 * Video node registration: runs from the subdev's .registered internal
 * op, i.e. once the SENINF notifier has bound this subdev and sd.v4l2_dev
 * points at the pipeline's v4l2_device.
 */
static int camsv_video_register(struct mt6785_camsv *c)
{
	struct video_device *vdev = &c->vdev;
	int ret;

	mutex_init(&c->vdev_lock);

	c->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	c->vbq.io_modes = VB2_MMAP | VB2_DMABUF;
	c->vbq.drv_priv = c;
	c->vbq.ops = &camsv_vb2_ops;
	c->vbq.mem_ops = &vb2_dma_contig_memops;
	c->vbq.buf_struct_size = sizeof(struct mt6785_camsv_buffer);
	c->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
				 V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	c->vbq.min_queued_buffers = MT6785_CAMSV_MIN_BUFFERS;
	c->vbq.lock = &c->vdev_lock;
	c->vbq.dev = c->dev;

	ret = vb2_queue_init(&c->vbq);
	if (ret) {
		dev_err(c->dev, "failed to init vb2 queue: %d\n", ret);
		goto err_mutex;
	}

	vdev->v4l2_dev = c->sd.v4l2_dev;
	vdev->queue = &c->vbq;
	vdev->lock = &c->vdev_lock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->ioctl_ops = &camsv_ioctl_ops;
	vdev->fops = &camsv_fops;
	vdev->release = video_device_release_empty;
	vdev->vfl_dir = VFL_DIR_RX;
	strscpy(vdev->name, "mtk-mt6785-camsv video", sizeof(vdev->name));
	video_set_drvdata(vdev, c);

	/*
	 * The video entity needs one SINK pad before the subdev->video
	 * link is created (media_create_pad_link() validates both pad
	 * flags and indices; without media_entity_pads_init() num_pads
	 * is 0 and the link fails with -EINVAL, seen on first boot).
	 */
	c->vdev_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, &c->vdev_pad);
	if (ret) {
		dev_err(c->dev, "failed to init video entity pads: %d\n", ret);
		goto err_vb2;
	}

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(c->dev, "failed to register video device: %d\n", ret);
		goto err_vb2;
	}

	/* Subdev source pad -> video node sink. */
	ret = media_create_pad_link(&c->sd.entity, 1, &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		dev_err(c->dev, "failed to link video node: %d\n", ret);
		goto err_vdev;
	}

	dev_info(c->dev, "MT6785 CAMSV0 registered as /dev/video%d (10-bit LE bitstream)\n",
		 vdev->num);

	return 0;

err_vdev:
	video_unregister_device(vdev);
err_vb2:
	vb2_queue_release(&c->vbq);
err_mutex:
	mutex_destroy(&c->vdev_lock);

	return ret;
}

static void camsv_video_unregister(struct mt6785_camsv *c)
{
	media_entity_cleanup(&c->vdev.entity);
	video_unregister_device(&c->vdev);
	vb2_queue_release(&c->vbq);
	mutex_destroy(&c->vdev_lock);
}

static int camsv_subdev_registered(struct v4l2_subdev *sd)
{
	struct mt6785_camsv *c = sd_to_camsv(sd);

	return camsv_video_register(c);
}

static void camsv_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct mt6785_camsv *c = sd_to_camsv(sd);

	camsv_video_unregister(c);
}

static const struct v4l2_subdev_internal_ops camsv_subdev_internal_ops = {
	.init_state = camsv_subdev_init_state,
	.registered = camsv_subdev_registered,
	.unregistered = camsv_subdev_unregistered,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 */

static const char *const camsv_clk_names[] = {
	"cam-main", "cam-gals", "cam-tg", "cam-seninf", "cam-sv0",
	"cam-larb6", "cam-larb7",
	"top-seninf0", "top-seninf1", "top-seninf2",
	"mipic0", "mipic1",
};

static int camsv_runtime_resume(struct device *dev)
{
	struct mt6785_camsv *c = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(c->num_clks, c->clks);
}

static int camsv_runtime_suspend(struct device *dev)
{
	struct mt6785_camsv *c = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(c->num_clks, c->clks);

	return 0;
}

static const struct dev_pm_ops camsv_pm_ops = {
	SET_RUNTIME_PM_OPS(camsv_runtime_suspend, camsv_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static int camsv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt6785_camsv *c;
	struct device_node *larb_np;
	struct resource *res;
	unsigned int i;
	int irq;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->dev = dev;
	dev_set_drvdata(dev, c);
	spin_lock_init(&c->buf_list_lock);
	INIT_LIST_HEAD(&c->buf_list);

	c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(c->base))
		return PTR_ERR(c->base);

	/*
	 * larb6 comes from the DT phandle (mt6785.dtsi has the node); larb7
	 * has no DT node in this tree yet, so its window comes from this
	 * node's second reg tuple.  Both are needed for the DMA MMU bypass:
	 * failing to map either means capture would write nothing.
	 */
	larb_np = of_parse_phandle(dev->of_node, "mediatek,larb6", 0);
	if (!larb_np)
		return dev_err_probe(dev, -ENODEV,
				     "mediatek,larb6 phandle missing\n");
	c->larb6 = of_iomap(larb_np, 0);
	of_node_put(larb_np);
	if (!c->larb6)
		return dev_err_probe(dev, -ENOMEM, "failed to map larb6\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "larb7");
	if (!res)
		return dev_err_probe(dev, -ENODEV,
				     "larb7 reg tuple missing\n");
	c->larb7 = devm_ioremap_resource(dev, res);
	if (IS_ERR(c->larb7))
		return dev_err_probe(dev, PTR_ERR(c->larb7),
				     "failed to map larb7\n");

	c->num_clks = ARRAY_SIZE(camsv_clk_names);
	c->clks = devm_kcalloc(dev, c->num_clks, sizeof(*c->clks), GFP_KERNEL);
	if (!c->clks)
		return -ENOMEM;
	for (i = 0; i < c->num_clks; i++)
		c->clks[i].id = camsv_clk_names[i];

	ret = devm_clk_bulk_get(dev, c->num_clks, c->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, camsv_irq, 0, dev_name(dev), c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq %d\n", irq);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set 32-bit DMA mask\n");

	/* Default format until userspace sets one. */
	c->format.pixelformat = camsv_formats[0].fourcc;
	c->format.width = camsv_default_fmt.width;
	c->format.height = camsv_default_fmt.height;
	c->format.bytesperline = camsv_bpl(c->format.width);
	c->format.sizeimage = c->format.bytesperline * c->format.height;
	c->format.field = V4L2_FIELD_NONE;
	c->format.colorspace = V4L2_COLORSPACE_SRGB;

	/* Subdev: sink pad 0 from SENINF, source pad 1 to the video node. */
	c->pads[0].flags = MEDIA_PAD_FL_SINK;
	c->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	v4l2_subdev_init(&c->sd, &camsv_subdev_ops);
	c->sd.internal_ops = &camsv_subdev_internal_ops;
	c->sd.dev = dev;
	c->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	c->sd.entity.ops = &camsv_media_entity_ops;
	c->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(c->sd.name, "mtk-mt6785-camsv", sizeof(c->sd.name));
	v4l2_set_subdevdata(&c->sd, c);

	ret = media_entity_pads_init(&c->sd.entity, 2, c->pads);
	if (ret)
		return ret;
	/*
	 * Async registration: when the SENINF notifier binds this subdev,
	 * .registered creates the video node under the pipeline's
	 * v4l2_device.
	 */
	ret = v4l2_async_register_subdev(&c->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pads;
	}

	ret = v4l2_subdev_init_finalize(&c->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev state\n");
		goto err_async;
	}

	pm_runtime_set_autosuspend_delay(dev, MT6785_CAMSV_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	devm_pm_runtime_enable(dev);
	pm_runtime_set_suspended(dev);

	dev_dbg(dev, "MT6785 CAMSV0 probe complete (video node on bind)\n");

	return 0;

err_async:
	v4l2_async_unregister_subdev(&c->sd);
err_pads:
	media_entity_cleanup(&c->sd.entity);
	return ret;
}

static void camsv_remove(struct platform_device *pdev)
{
	struct mt6785_camsv *c = dev_get_drvdata(&pdev->dev);

	v4l2_async_unregister_subdev(&c->sd);	/* runs .unregistered */
	v4l2_subdev_cleanup(&c->sd);
	media_entity_cleanup(&c->sd.entity);

	iounmap(c->larb6);
}

static const struct of_device_id camsv_of_match[] = {
	{ .compatible = "mediatek,mt6785-camsv" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, camsv_of_match);

static struct platform_driver mt6785_camsv_driver = {
	.probe = camsv_probe,
	.remove = camsv_remove,
	.driver = {
		.name = "mtk-mt6785-camsv",
		.pm = &camsv_pm_ops,
		.of_match_table = camsv_of_match,
	},
};

module_platform_driver(mt6785_camsv_driver);

MODULE_DESCRIPTION("MediaTek MT6785 CAMSV0 raw capture driver");
MODULE_LICENSE("GPL");
