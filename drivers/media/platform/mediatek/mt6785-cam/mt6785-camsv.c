// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6785 CAMSV0 raw-frame capture driver (V4L2 video node).
 *
 * CAMSV0 is the "sensor viewfinder" DMA pipe of the MT6785 camera system:
 * CAM_MUX 2 -> TG (timing generator) -> PAK (packer) -> IMGO DMA into
 * physically contiguous memory.  With the measured FMT_SEL = 0x102 /
 * PAK = 5 configuration (docs/hardware/CAMERA.md "Standard 16-bit
 * layout and autofocus actuator") each 10-bit sample lands in its own
 * little-endian 16-bit word, value in the low 10 bits, two bytes per
 * pixel: exactly V4L2's unpacked 10-bit Bayer formats
 * (V4L2_PIX_FMT_SBGGR10 & co), which libcamera's software ISP reads
 * without any custom decoder.  The node advertises only those four
 * fourccs, bytesperline = width * 2, sizeimage = bytesperline * height,
 * colorspace raw.
 *
 * LAYOUT OVERRIDE (experiments only): the fmt_sel / pak /
 * bytes_per_px_x8 module parameters (0 = the defaults above) let the
 * packer be swept at runtime (tools/camera/layout-sweep.sh); e.g.
 * PAK 1 with a 10-bit line is the continuous little-endian 10-bit bit
 * stream found first ("le10", no standard fourcc — decode with
 * tools/camera/raw2rgb.py --layout le10).  Using any override logs a
 * warning at STREAMON: the DMA output then no longer matches the
 * advertised fourcc.
 *
 * The register sequence below is the one validated on the Lenovo Yoga
 * Tab 11 by tools/camera/ov-cap.py (setup_camsv0()) and the begonia
 * bring-up (src/begonia-ftp/camera/NOTES.md), except the FBC: its CTL1
 * low field is a compare value against the count of frames written,
 * kept ahead of that count at every frame end so the DMA writes every
 * frame (measured on the tablet, see fbc_margin).
 *
 * Frames (rotate=1, default): IMGO_BASE_ADDR is programmed for the
 * next frame at each PASS1_DON, a scratch buffer takes the frames
 * nobody queued a buffer for, and markers at the start of the first
 * and last lines prove that each frame landed in the buffer programmed
 * for it.  rotate=0 keeps a bounce buffer + copy path as the fallback.
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
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
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
#include <linux/seq_file.h>
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

/*
 * INT_EN / INT_STATUS bits.  INT_EN reads back 0x803b04ff after a write
 * of 0xffffffff (begonia camsv/frames/d1-run.txt), i.e. bits 0-7, 10,
 * 16, 17, 19-21 and 31 exist; the names follow the ISP4/5 CAMSV layout
 * and the begonia notes (bit 7 = SOF, bit 10 = PASS1_DON, bit 16 =
 * IMGO_ERR, bit 19 set when the FBC drops frames).
 *
 * Bit 31 is INT_WCLR_EN: it switches INT_STATUS from read-to-clear to
 * write-1-to-clear.  The t8f driver wrote INT_EN = 0xffffffff, which set
 * it, while its handler only read the status: the level interrupt never
 * dropped and the handler ran ~575000 times a second, each run seeing
 * PASS1_DON again.  INT_EN now gets the vendor HAL value (0x001304b1,
 * bit 31 clear) and the handler also writes the status back, which
 * clears it in either mode.
 */
#define CAMSV_IRQ_VS1			BIT(0)
#define CAMSV_IRQ_TG_ERR		BIT(4)
#define CAMSV_IRQ_TG_GBERR		BIT(5)
#define CAMSV_IRQ_SOF			BIT(7)
#define CAMSV_IRQ_PASS1_DON		BIT(10)
#define CAMSV_IRQ_IMGO_ERR		BIT(16)
#define CAMSV_IRQ_IMGO_OVERR		BIT(17)
#define CAMSV_IRQ_IMGO_DROP		BIT(19)
#define CAMSV_INT_WCLR_EN		BIT(31)
#define CAMSV_INT_EN_MASK		0x001304b1	/* vendor HAL value */

/*
 * More interrupts than this in one second is a storm (a real stream
 * raises a handful per frame): the handler then masks INT_EN and says
 * so, instead of eating a CPU.
 */
#define CAMSV_IRQ_STORM_LIMIT		10000

/*
 * FBC (measured on the tablet with /dev/mem pokes while streaming,
 * t8g-t8i; the begonia "RCNT_INC" reading of bit 0 was wrong):
 *
 * - FBC_CTL2[7:0] counts the frames the DMA has written (7 bits, wraps
 *   through 0; nothing ever decrements it); FBC_CTL2[31:24] counts the
 *   frames dropped;
 * - the low 6-bit field of FBC_CTL1 (NUM) is a compare value: the DMA
 *   stops writing (drops every frame) when the written count EQUALS
 *   NUM, and always when NUM is 0.  Any other NUM lets frames flow;
 *   a frame in progress when NUM is set to 0 is still completed.
 *
 * So the FBC never needs a release: the driver keeps NUM ahead of the
 * count.  At STREAMON and at every PASS1_DON it writes
 * NUM = (count + fbc_margin) & 0x3f, or 1 if that is 0, which is never
 * 0 and never equal to the count; the DMA would only stall if fbc_margin
 * frames went by without a PASS1_DON being serviced.  Only the low
 * field changes, FBC_EN stays set.  The bounce path holds the DMA with
 * NUM = 0 while it copies.
 */
/*
 * Measured packer values for the standard unpacked 10-bit layout
 * (CAMERA.md "Standard 16-bit layout and autofocus actuator"): the
 * layout-sweep found FMT_SEL 0x102 with PAK 5 (PAK 1 plus bit 2) and
 * two bytes per pixel writing each sample in a little-endian 16-bit
 * word, value in the low bits — what V4L2_PIX_FMT_S*10 documents and
 * libcamera's software ISP debayer reads.  The fmt_sel/pak/
 * bytes_per_px_x8 module parameters below override these at STREAMON
 * for further experiments (tools/camera/layout-sweep.sh; PAK 1 with a
 * 10-bit line is the earlier "le10" bitstream layout, kept reachable
 * for tools/camera/raw2rgb.py --layout le10).
 */
#define CAMSV_FMT_SEL_U16		0x102
#define CAMSV_PAK_U16			5
#define CAMSV_BPL_X8_U16		16	/* bytes per pixel * 8 */

#define CAMSV_FBC_NUM_MASK		GENMASK(5, 0)
#define CAMSV_FBC2_WCNT			GENMASK(6, 0)	/* frames written */
#define CAMSV_FBC2_DROP			GENMASK(31, 24)	/* frames dropped */
#define CAMSV_FBC_MARGIN_MIN		2
#define CAMSV_FBC_MARGIN_MAX		32

static unsigned int fbc_margin = 8;
module_param(fbc_margin, uint, 0644);
MODULE_PARM_DESC(fbc_margin,
		 "Frames the FBC compare value is kept ahead of the written-frame count (2-32, default 8; read at STREAMON)");

/*
 * Packer layout override knobs, experiments only (read at STREAMON):
 * fmt_sel overrides the CAMSV FMT_SEL register (0x514), pak the PAK
 * register (0x54c) and bytes_per_px_x8 the bytes-per-line computation.
 * 0 (the default of each) keeps the standard unpacked layout: FMT_SEL
 * 0x102, PAK 5, two bytes per pixel.  Any non-default value makes the
 * DMA output diverge from the advertised fourcc — STREAMON logs a
 * warning naming every override in use.  See
 * docs/research/camera/T13-LIBCAMERA.md section 2 and
 * tools/camera/layout-sweep.sh.
 */
static unsigned int fmt_sel;
module_param(fmt_sel, uint, 0644);
MODULE_PARM_DESC(fmt_sel,
		 "FMT_SEL register value written at STREAMON (0 = driver default 0x102, the standard unpacked 10-bit layout)");

static unsigned int pak = CAMSV_PAK_U16;
module_param(pak, uint, 0644);
MODULE_PARM_DESC(pak,
		 "PAK register value written at STREAMON (default 5, the standard unpacked 10-bit layout; written as-is because 0 is a real PAK mode to sweep)");

static unsigned int bytes_per_px_x8 = CAMSV_BPL_X8_U16;
module_param(bytes_per_px_x8, uint, 0644);
MODULE_PARM_DESC(bytes_per_px_x8,
		 "Bytes per pixel * 8 for the stride/BPL computation (default 16 = two bytes per pixel, the standard unpacked 10-bit layout)");

/*
 * Capture path (read at STREAMON):
 *
 * rotate=1 (default, zero copy): at each PASS1_DON the hard interrupt
 * programs IMGO_BASE_ADDR with the next queued vb2 buffer, in vertical
 * blanking, so the next frame lands in it; when no buffer is queued it
 * parks the DMA on a driver-owned scratch buffer, so the free-running
 * DMA never writes a buffer userspace holds.  Whether the address takes
 * effect for the very next frame is checked per frame (markers below);
 * a buffer that did not get its frame is recycled, never delivered.
 *
 * rotate=0 (bounce): the DMA always writes the scratch buffer; at
 * PASS1_DON the hard interrupt holds the FBC (compare value 0), the thread
 * copies the frame into the next vb2 buffer, then releases the hold.
 * Costs a copy and every other frame; kept as the fallback if the
 * per-frame address does not work on some mode.
 */
static bool rotate = true;
module_param(rotate, bool, 0644);
MODULE_PARM_DESC(rotate,
		 "1 (default): each frame written straight into its vb2 buffer (IMGO_BASE_ADDR per frame); 0: one bounce buffer + copy");

/*
 * Frame markers: before a buffer is programmed for a frame, the driver
 * writes a 16-byte pattern at the start of its first line and at the
 * start of its last line.  At the frame's PASS1_DON both must have been
 * overwritten.  Both intact while the hardware counted a written frame
 * (FBC_CTL2[6:0]): the frame went to another address ("missed").  Head
 * intact, last line written: torn (the DMA started mid-frame).  Head
 * written, last line intact: short.  These are the only per-address
 * evidence, so they stay on in both modes (buffers without a kernel
 * mapping are judged by FBC_CTL2[7:0] alone).
 */
#define CAMSV_MARK_LEN			16

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
 * @vaddr: kernel mapping of an MMAP plane (coherent), for the markers;
 *	NULL for imported buffers
 */
struct mt6785_camsv_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
	void *vaddr;
};

/**
 * struct mt6785_camsv_stats - IRQ path counters, reset at stream-on and
 *	shown in the debugfs status file
 * @irqs: hard interrupts handled
 * @bits: per INT_STATUS bit, how many interrupts carried it
 * @pass1: PASS1_DON events (the frame index: sequence numbers use it)
 * @hw_frames: frames the DMA wrote (sum of FBC_CTL2[7:0] advances)
 * @hw_idle: PASS1_DON events with FBC_CTL2[7:0] unchanged (no frame
 *	written: FBC held or dropping)
 * @hw_multi: PASS1_DON events where FBC_CTL2[7:0] advanced by 2 or more
 * @drops: FBC_CTL2[31:24] advances seen at PASS1_DON
 * @stalls: PASS1_DON events that found the written count equal to the
 *	compare value (DMA stopped; it should stay 0)
 * @frames: frames delivered to vb2 as DONE
 * @missed: written frames whose target's markers were both intact
 * @torn: frames whose head marker survived (DMA started mid-frame)
 * @short_frames: frames whose last-line marker survived
 * @disagree: markers overwritten while FBC_CTL2[7:0] did not advance
 * @parked: written frames that went to the scratch buffer (rotate: no
 *	vb2 buffer queued) or that no vb2 buffer took (bounce)
 * @recycled: buffers put back in the queue (frame not in them)
 * @copy_races: bounce: a frame written while a copy was running
 * @errors: buffers returned with an error
 * @storms: times the storm guard masked the interrupt
 * @last_status: last INT_STATUS seen by the handler
 * @last_fbc2: FBC_IMGO_CTL2 read at the last PASS1_DON
 * @last_base: IMGO_BASE_ADDR programmed at the last PASS1_DON
 */
struct mt6785_camsv_stats {
	unsigned int irqs;
	unsigned int bits[32];
	unsigned int pass1;
	unsigned int hw_frames;
	unsigned int hw_idle;
	unsigned int hw_multi;
	unsigned int drops;
	unsigned int stalls;
	unsigned int frames;
	unsigned int missed;
	unsigned int torn;
	unsigned int short_frames;
	unsigned int disagree;
	unsigned int parked;
	unsigned int recycled;
	unsigned int copy_races;
	unsigned int errors;
	unsigned int storms;
	u32 last_status;
	u32 last_fbc2;
	u32 last_base;
};

/**
 * struct mt6785_camsv - CAMSV0 capture device
 * @dev: platform device
 * @base: mapped CAMSV0 registers
 * @larb6: mapped SMI LARB6 registers for the MMU bypass
 * @larb7: mapped SMI LARB7 registers for the MMU bypass
 * @clks: bulk clocks (cam, camsys gates, top seninf muxes, APMIXED 26 MHz)
 * @num_clks: number of entries in @clks
 * @sd: CAMSV subdev (sink from SENINF, source to the video node)
 * @pads: subdev pads: 0 sink (from seninf), 1 source (to video node)
 * @vdev: registered /dev/videoN capture node
 * @vdev_pad: sink pad of the video node entity (from the subdev source)
 * @vbq: vb2 queue of @vdev
 * @vdev_lock: serializes format ioctls against the queue
 * @format: current V4L2 pixel format
 * @seninf_pad: remote SENINF pad, resolved when streaming starts
 * @pipeline: media pipeline handle
 * @buf_list_lock: protects @buf_list
 * @buf_list: queued capture buffers not yet handed to the hardware
 * @streaming: streaming state
 * @rotate: zero-copy mode for this stream (module parameter)
 * @fbc_margin: FBC compare value lead for this stream (module parameter)
 * @fmt_sel: FMT_SEL register value for this stream (module parameter, 0 = the standard unpacked value)
 * @pak: PAK register value for this stream (module parameter; non-default = override)
 * @bytes_per_px_x8: bytes-per-pixel * 8 for this stream (module parameter; 0 = 10-bit fallback, 16 = standard)
 * @fbc_cur_num: FBC_CTL1 compare value last written (0: held)
 * @cur: rotate: the vb2 buffer programmed for the frame in progress
 *	(NULL: the scratch buffer); driver-owned until judged
 * @scratch: scratch buffer (rotate: parking target; bounce: the DMA
 *	target), cacheable, synced by hand
 * @scratch_dma: its DMA (physical) address
 * @scratch_size: its size
 * @last_wcnt: FBC_CTL2[7:0] at the previous PASS1_DON
 * @gated: bounce: FBC held while the thread copies a frame
 * @gate_seq: bounce: sequence number of the frame being copied
 * @gate_wcnt: bounce: FBC_CTL2[7:0] when the hold was set
 * @irq: CAMSV0 interrupt, synchronized on stream-off
 * @storm_start: start (jiffies) of the storm guard's one-second window
 * @storm_count: interrupts in that window
 * @stats: IRQ path counters
 * @dbg_status: debugfs status dump dentry
 *
 * @cur, @last_wcnt and the gate fields belong to the interrupt handlers
 * while streaming, and to start/stop while the interrupt is quiet
 * (before TG enable, after synchronize_irq()).
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
	bool streaming;

	bool rotate;
	unsigned int fbc_margin;
	u32 fmt_sel;
	u32 pak;
	u32 bytes_per_px_x8;
	u32 fbc_cur_num;
	struct mt6785_camsv_buffer *cur;
	void *scratch;
	dma_addr_t scratch_dma;
	size_t scratch_size;
	u8 last_wcnt;
	bool gated;
	unsigned int gate_seq;
	u8 gate_wcnt;

	int irq;
	unsigned long storm_start;
	unsigned int storm_count;
	struct mt6785_camsv_stats stats;

	struct dentry *dbg_status;
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
 * Formats: 10-bit Bayer, unpacked.  The sensor's media bus code picks the
 * Bayer pattern; the packer (FMT_SEL 0x102 / PAK 5) writes each sample in
 * a little-endian 16-bit word, value in the low bits, so the V4L2
 * unpacked 10-bit fourccs (V4L2_PIX_FMT_S*10) describe the memory layout
 * exactly.  bytesperline = width * 2, sizeimage = bytesperline * height,
 * colorspace raw.
 */

struct mt6785_camsv_format {
	u32 code;		/* media bus code */
	u32 fourcc;		/* V4L2 pixel format */
};

static const struct mt6785_camsv_format camsv_formats[] = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10 },
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

/*
 * Bytes per line for the current override setting @bpp_x8 (the
 * per-stream bytes_per_px_x8 at STREAMON, or the live module parameter
 * from S_FMT / queue_setup): 0 falls back to the sensor sample width
 * (10 bits per pixel, the pre-sweep behaviour); any other value scales
 * it (bytes per line = width * bpp_x8 / 8).
 */
static unsigned int camsv_bpl_stream(unsigned int width, unsigned int bpp_x8)
{
	if (!bpp_x8 || bpp_x8 == MT6785_CAM_BPP)
		return camsv_bpl(width);

	return ALIGN(DIV_ROUND_UP(width * bpp_x8, 8), 2);
}

/*
 * Same, with the live module parameter (S_FMT / queue_setup, before the
 * per-stream copy exists).
 */
static unsigned int camsv_bpl_sweep(unsigned int width)
{
	return camsv_bpl_stream(width, READ_ONCE(bytes_per_px_x8));
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
 * ov-cap.py setup_camsv0() register block with the standard unpacked
 * layout (docs/hardware/CAMERA.md "Standard 16-bit layout and autofocus
 * actuator"): FMT_SEL = 0x102 and PAK = 5 (PAK 1 plus bit 2) with the
 * line sized for two bytes per pixel write each sample in a
 * little-endian 16-bit word, value in the low bits.  The begonia values
 * (FMT_SEL = 0x101 / PAK = 2) write 12-bit containers and truncate each
 * line to 2/3 of the pixels on this board; PAK 1 with a 10-bit line is
 * the earlier continuous little-endian 10-bit bitstream ("le10").
 *
 * Split in two so the sensor can be started between the reset and the
 * TG enable (the --stream-first order validated on this tablet):
 * camsv_hw_config() runs before the sensor streams, camsv_hw_enable()
 * arms the TG on an already-running receiver.
 */
/*
 * Keep the DMA writing: set the FBC compare value fbc_margin frames
 * ahead of the written count @cnt, never 0 (always holds) and never
 * equal to @cnt (holds when equal).  A count of 64..127 can never equal
 * the 6-bit field, so any value works there too.
 */
static void camsv_fbc_run(struct mt6785_camsv *c, u8 cnt)
{
	u32 num = (cnt + c->fbc_margin) & CAMSV_FBC_NUM_MASK;

	if (!num)
		num = 1;
	c->fbc_cur_num = num;
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_EN | num);
}

/* Stop the DMA after the frame in progress (compare value 0). */
static void camsv_fbc_hold(struct mt6785_camsv *c)
{
	c->fbc_cur_num = 0;
	camsv_wr(c, CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_EN);
}

static u8 camsv_fbc_count(struct mt6785_camsv *c)
{
	return FIELD_GET(CAMSV_FBC2_WCNT, camsv_rd(c, CAMSV_FBC_IMGO_CTL2));
}

/* IMGO DMA window. */
static void camsv_imgo_window(struct mt6785_camsv *c, dma_addr_t dma,
			      u32 h, u32 bpl)
{
	camsv_wr(c, CAMSV_IMGO_BASE_ADDR, lower_32_bits(dma));
	camsv_wr(c, CAMSV_IMGO_XSIZE, bpl - 1);
	camsv_wr(c, CAMSV_IMGO_YSIZE, h - 1);
	/* Plain BPL: bit 24 packs two lines into one and halves the frame. */
	camsv_wr(c, CAMSV_IMGO_STRIDE, bpl);
	camsv_wr(c, CAMSV_IMGO_CON2, 0x20002);
	camsv_wr(c, CAMSV_IMGO_CON3, 0x20002);
}

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
	camsv_imgo_window(c, dma, h, bpl);
	camsv_wr(c, CAMSV_FMT_SEL, c->fmt_sel ?: CAMSV_FMT_SEL_U16);
	camsv_wr(c, CAMSV_PAK, c->pak);
	/* FBC_EN with the compare value ahead of the written count. */
	c->last_wcnt = camsv_fbc_count(c);
	camsv_fbc_run(c, c->last_wcnt);
	camsv_wr(c, CAMSV_CLK_EN, 0x8005);		/* TG | PAK | DMA */
	camsv_wr(c, CAMSV_MODULE_EN, 0x40000015);	/* DB|IMGO|PAK|TG */
	/*
	 * NOT 0xffffffff: bit 31 (INT_WCLR_EN) makes the status
	 * write-1-to-clear (the t8f interrupt storm).  Then drop whatever
	 * is pending: the read clears it in read-clear mode, the write in
	 * write-clear mode.
	 */
	camsv_wr(c, CAMSV_INT_EN, CAMSV_INT_EN_MASK);
	camsv_wr(c, CAMSV_INT_STATUS, camsv_rd(c, CAMSV_INT_STATUS));
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

static const u8 camsv_mark[CAMSV_MARK_LEN] = {
	0x4d, 0x54, 0x36, 0x37, 0x38, 0x35, 0xc3, 0x5a,
	0xa5, 0x3c, 0x0f, 0xf0, 0x96, 0x69, 0xe1, 0x1e,
};

/* Offset of the second marker: the start of the last line. */
static size_t camsv_mark_tail(struct mt6785_camsv *c)
{
	return (size_t)(c->format.height - 1) * c->format.bytesperline;
}

/* Scratch buffer cache maintenance for the marker areas. */
static void camsv_scratch_sync(struct mt6785_camsv *c, bool for_device)
{
	size_t tail = camsv_mark_tail(c);

	if (for_device) {
		dma_sync_single_range_for_device(c->dev, c->scratch_dma, 0,
						 CAMSV_MARK_LEN,
						 DMA_BIDIRECTIONAL);
		dma_sync_single_range_for_device(c->dev, c->scratch_dma, tail,
						 CAMSV_MARK_LEN,
						 DMA_BIDIRECTIONAL);
	} else {
		dma_sync_single_range_for_cpu(c->dev, c->scratch_dma, 0,
					      CAMSV_MARK_LEN,
					      DMA_BIDIRECTIONAL);
		dma_sync_single_range_for_cpu(c->dev, c->scratch_dma, tail,
					      CAMSV_MARK_LEN,
					      DMA_BIDIRECTIONAL);
	}
}

/*
 * Write the markers into @va.  vb2 MMAP buffers are coherent (no
 * maintenance); the scratch buffer is cacheable (@scratch: clean them
 * to memory).  The writel() of the following IMGO_BASE_ADDR / FBC write
 * orders these stores before the DMA is pointed at the buffer.
 */
static void camsv_mark_set(struct mt6785_camsv *c, void *va, bool scratch)
{
	memcpy(va, camsv_mark, CAMSV_MARK_LEN);
	memcpy(va + camsv_mark_tail(c), camsv_mark, CAMSV_MARK_LEN);
	if (scratch)
		camsv_scratch_sync(c, true);
}

enum camsv_frame_state {
	CAMSV_FRAME_WRITTEN,
	CAMSV_FRAME_TORN,
	CAMSV_FRAME_SHORT,
	CAMSV_FRAME_UNTOUCHED,
};

/* Which markers of @va did the DMA overwrite? */
static enum camsv_frame_state camsv_mark_state(struct mt6785_camsv *c,
					       void *va, bool scratch)
{
	bool head, tail;

	if (scratch)
		camsv_scratch_sync(c, false);
	head = memcmp(va, camsv_mark, CAMSV_MARK_LEN);
	tail = memcmp(va + camsv_mark_tail(c), camsv_mark, CAMSV_MARK_LEN);

	if (head && tail)
		return CAMSV_FRAME_WRITTEN;
	if (tail)
		return CAMSV_FRAME_TORN;
	if (head)
		return CAMSV_FRAME_SHORT;
	return CAMSV_FRAME_UNTOUCHED;
}

static void camsv_complete(struct mt6785_camsv *c,
			   struct mt6785_camsv_buffer *buf, unsigned int seq)
{
	buf->vb.sequence = seq;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	c->stats.frames++;
}

/* Put a buffer that did not get its frame back at the queue head. */
static void camsv_recycle(struct mt6785_camsv *c,
			  struct mt6785_camsv_buffer *buf)
{
	unsigned long flags;

	spin_lock_irqsave(&c->buf_list_lock, flags);
	list_add(&buf->list, &c->buf_list);
	spin_unlock_irqrestore(&c->buf_list_lock, flags);
	c->stats.recycled++;
}

static struct mt6785_camsv_buffer *camsv_pop(struct mt6785_camsv *c)
{
	struct mt6785_camsv_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&c->buf_list_lock, flags);
	buf = list_first_entry_or_null(&c->buf_list,
				       struct mt6785_camsv_buffer, list);
	if (buf)
		list_del(&buf->list);
	spin_unlock_irqrestore(&c->buf_list_lock, flags);

	return buf;
}

/*
 * Rotate mode, hard interrupt, frame @seq just ended (@wrote: the FBC
 * counted a written frame).  First point the DMA at the next frame's
 * buffer (we are in vertical blanking: the address must be in place
 * before the next frame starts), then judge the buffer that was
 * programmed for the frame that ended.
 */
static void camsv_rotate_frame_end(struct mt6785_camsv *c, unsigned int seq,
				   bool wrote, u8 wcnt)
{
	struct mt6785_camsv_buffer *done = c->cur, *next;
	enum camsv_frame_state st;
	dma_addr_t dma;

	next = camsv_pop(c);
	if (next) {
		if (next->vaddr)
			camsv_mark_set(c, next->vaddr, false);
		dma = next->dma_addr;
	} else {
		dma = c->scratch_dma;		/* park: no buffer queued */
	}
	camsv_wr(c, CAMSV_IMGO_BASE_ADDR, lower_32_bits(dma));
	c->stats.last_base = lower_32_bits(dma);
	c->cur = next;
	/* Keep the FBC compare value ahead of the written count. */
	camsv_fbc_run(c, wcnt);

	if (!done) {
		if (wrote)
			c->stats.parked++;
		return;
	}

	if (!done->vaddr) {
		/* Imported buffer, no markers: trust the FBC counter. */
		if (wrote)
			camsv_complete(c, done, seq);
		else
			camsv_recycle(c, done);
		return;
	}

	st = camsv_mark_state(c, done->vaddr, false);
	switch (st) {
	case CAMSV_FRAME_WRITTEN:
		if (!wrote)
			c->stats.disagree++;
		camsv_complete(c, done, seq);
		return;
	case CAMSV_FRAME_TORN:
		c->stats.torn++;
		break;
	case CAMSV_FRAME_SHORT:
		c->stats.short_frames++;
		break;
	case CAMSV_FRAME_UNTOUCHED:
		if (wrote)
			c->stats.missed++;	/* frame went elsewhere */
		break;
	}
	camsv_recycle(c, done);
}

/*
 * Bounce mode, hard interrupt: a frame was written into the scratch
 * buffer.  Hold the FBC (compare value 0: the DMA stops after the frame
 * in progress, i.e. before the next one, since we are in vertical
 * blanking) and let the thread copy it.  Returns true to wake it.
 */
static bool camsv_bounce_frame_end(struct mt6785_camsv *c, unsigned int seq,
				   bool wrote, u8 wcnt)
{
	if (c->gated) {
		if (wrote)
			c->stats.copy_races++;
		return false;
	}
	if (!wrote) {
		camsv_fbc_run(c, wcnt);		/* restart a stalled FBC */
		return false;
	}

	camsv_fbc_hold(c);
	c->gated = true;
	c->gate_seq = seq;
	c->gate_wcnt = c->last_wcnt;

	return true;
}

/*
 * Threaded half (bounce mode only): check the scratch markers, copy the
 * frame into the next queued vb2 buffer, re-mark, release the hold.
 * May sleep (the copy of a 10 MB frame).
 */
static irqreturn_t camsv_irq_thread(int irq, void *data)
{
	struct mt6785_camsv *c = data;
	struct mt6785_camsv_buffer *buf;
	u8 wcnt;

	if (!READ_ONCE(c->streaming) || !READ_ONCE(c->gated))
		return IRQ_HANDLED;

	switch (camsv_mark_state(c, c->scratch, true)) {
	case CAMSV_FRAME_WRITTEN:
		buf = camsv_pop(c);
		if (!buf) {
			c->stats.parked++;
			break;
		}
		if (!buf->vaddr) {
			c->stats.errors++;
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			break;
		}
		dma_sync_single_for_cpu(c->dev, c->scratch_dma,
					c->format.sizeimage, DMA_BIDIRECTIONAL);
		memcpy(buf->vaddr, c->scratch, c->format.sizeimage);
		/* Another frame written during the copy: it may be mixed. */
		wcnt = camsv_fbc_count(c);
		if (wcnt != c->gate_wcnt) {
			c->stats.copy_races++;
			camsv_recycle(c, buf);
			break;
		}
		camsv_complete(c, buf, c->gate_seq);
		break;
	case CAMSV_FRAME_TORN:
		c->stats.torn++;
		break;
	case CAMSV_FRAME_SHORT:
		c->stats.short_frames++;
		break;
	case CAMSV_FRAME_UNTOUCHED:
		c->stats.missed++;
		break;
	}

	camsv_mark_set(c, c->scratch, true);
	WRITE_ONCE(c->gated, false);
	camsv_fbc_run(c, camsv_fbc_count(c));

	return IRQ_HANDLED;
}

/*
 * Hard half: read and clear INT_STATUS, count, guard against a storm;
 * at PASS1_DON read the FBC written-frame counter and run the frame end
 * of the current mode (rotate: entirely here, bounce: wake the thread).
 */
static irqreturn_t camsv_irq(int irq, void *data)
{
	struct mt6785_camsv *c = data;
	u32 status = camsv_rd(c, CAMSV_INT_STATUS);
	unsigned long bits = status;
	unsigned int bit, seq;
	u8 wcnt, delta;
	u32 fbc2;

	if (!status)
		return IRQ_NONE;

	/* Read-clear with INT_WCLR_EN off; the write clears it otherwise. */
	camsv_wr(c, CAMSV_INT_STATUS, status);

	c->stats.irqs++;
	c->stats.last_status = status;
	for_each_set_bit(bit, &bits, 32)
		c->stats.bits[bit]++;

	if (time_after(jiffies, c->storm_start + HZ)) {
		c->storm_start = jiffies;
		c->storm_count = 0;
	}
	if (++c->storm_count > CAMSV_IRQ_STORM_LIMIT) {
		camsv_wr(c, CAMSV_INT_EN, 0);
		c->stats.storms++;
		dev_err_ratelimited(c->dev,
				    "interrupt storm, INT_STATUS 0x%08x does not clear: CAMSV interrupts masked until the next stream-on\n",
				    status);
		return IRQ_HANDLED;
	}

	if (status & CAMSV_IRQ_IMGO_ERR)
		dev_err_ratelimited(c->dev,
				    "IMGO DMA error (INT_STATUS 0x%08x); is the LARB MMU bypass active?\n",
				    status);
	if (status & CAMSV_IRQ_TG_ERR)
		dev_dbg_ratelimited(c->dev, "TG error in frame (INT_STATUS 0x%08x)\n",
				    status);

	if (!(status & CAMSV_IRQ_PASS1_DON))
		return IRQ_HANDLED;

	fbc2 = camsv_rd(c, CAMSV_FBC_IMGO_CTL2);
	wcnt = FIELD_GET(CAMSV_FBC2_WCNT, fbc2);
	delta = (wcnt - c->last_wcnt) & CAMSV_FBC2_WCNT;	/* 7-bit wrap */
	c->last_wcnt = wcnt;
	if (c->fbc_cur_num && wcnt == c->fbc_cur_num)
		c->stats.stalls++;	/* the DMA had stopped */
	c->stats.hw_frames += delta;
	if (!delta)
		c->stats.hw_idle++;
	else if (delta > 1)
		c->stats.hw_multi++;
	if ((fbc2 ^ c->stats.last_fbc2) & CAMSV_FBC2_DROP)
		c->stats.drops++;
	c->stats.last_fbc2 = fbc2;
	seq = c->stats.pass1++;

	if (!READ_ONCE(c->streaming))
		return IRQ_HANDLED;

	if (c->rotate) {
		camsv_rotate_frame_end(c, seq, delta, wcnt);
		return IRQ_HANDLED;
	}

	return camsv_bounce_frame_end(c, seq, delta, wcnt) ? IRQ_WAKE_THREAD :
							    IRQ_HANDLED;
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
	/*
	 * Markers need a CPU mapping the interrupt handler can use: only
	 * MMAP planes (coherent, mapped at allocation).  Imported buffers
	 * are judged by the FBC counter alone.
	 */
	buf->vaddr = vb->memory == VB2_MEMORY_MMAP ? vb2_plane_vaddr(vb, 0) :
						     NULL;

	return 0;
}

static void camsv_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct mt6785_camsv *c = vb2q_to_camsv(vb->vb2_queue);
	struct mt6785_camsv_buffer *buf = to_camsv_buffer(vb);
	unsigned long flags;

	/*
	 * The interrupt handler takes buffers from the list at each frame
	 * end (rotate: programs the next one; bounce: copies into it).
	 */
	spin_lock_irqsave(&c->buf_list_lock, flags);
	list_add_tail(&buf->list, &c->buf_list);
	spin_unlock_irqrestore(&c->buf_list_lock, flags);
}

/* Return the in-flight and every queued buffer to vb2 (stop / failure). */
static void camsv_return_all_buffers(struct mt6785_camsv *c,
				     enum vb2_buffer_state state)
{
	struct mt6785_camsv_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&c->buf_list_lock, flags);
	if (c->cur) {
		vb2_buffer_done(&c->cur->vb.vb2_buf, state);
		c->cur = NULL;
	}
	list_for_each_entry_safe(buf, tmp, &c->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&c->buf_list_lock, flags);
}

static void camsv_free_scratch(struct mt6785_camsv *c)
{
	if (!c->scratch)
		return;

	dma_free_noncoherent(c->dev, c->scratch_size, c->scratch,
			     c->scratch_dma, DMA_BIDIRECTIONAL);
	c->scratch = NULL;
}

static int camsv_vb2_start_streaming(struct vb2_queue *vq,
				     unsigned int count)
{
	struct mt6785_camsv *c = vb2q_to_camsv(vq);
	struct v4l2_subdev *seninf_sd;
	dma_addr_t target;
	int ret;

	if (!c->seninf_pad) {
		c->seninf_pad = media_pad_remote_pad_first(&c->pads[0]);
		if (!c->seninf_pad) {
			dev_err(c->dev,
				"no SENINF connected to sink pad 0; the media link is missing (notifier bound callback)\n");
			ret = -ENOLINK;
			goto err_buffers;
		}
	}

	seninf_sd = media_entity_to_v4l2_subdev(c->seninf_pad->entity);

	/* Per-stream state; the interrupt is quiet (TG off). */
	c->rotate = rotate;
	c->fbc_margin = clamp_val(fbc_margin, CAMSV_FBC_MARGIN_MIN,
				  CAMSV_FBC_MARGIN_MAX);
	c->fmt_sel = fmt_sel;
	c->pak = pak;
	c->bytes_per_px_x8 = bytes_per_px_x8;
	if (c->bytes_per_px_x8 == MT6785_CAM_BPP)
		c->bytes_per_px_x8 = 0;		/* same as the 10-bit fallback */
	c->cur = NULL;
	c->gated = false;
	c->storm_start = jiffies;
	c->storm_count = 0;
	memset(&c->stats, 0, sizeof(c->stats));

	/*
	 * Layout override check: any of the three parameters off its
	 * standard-unpacked default makes the DMA output diverge from the
	 * advertised V4L2_PIX_FMT_S*10, so say so — loudly enough to be
	 * seen in a sweep log (the driver is otherwise quiet when it
	 * works).
	 */
	if (c->fmt_sel && c->fmt_sel != CAMSV_FMT_SEL_U16)
		dev_warn(c->dev,
			 "fmt_sel=0x%x overrides the standard unpacked layout (FMT_SEL 0x%03x): the DMA output no longer matches the advertised fourcc\n",
			 c->fmt_sel, CAMSV_FMT_SEL_U16);
	if (c->pak != CAMSV_PAK_U16)
		dev_warn(c->dev,
			 "pak=%u overrides the standard unpacked layout (PAK %u): the DMA output no longer matches the advertised fourcc\n",
			 c->pak, CAMSV_PAK_U16);
	if (c->bytes_per_px_x8 && c->bytes_per_px_x8 != CAMSV_BPL_X8_U16)
		dev_warn(c->dev,
			 "bytes_per_px_x8=%u overrides the standard unpacked layout (%u/8 bytes per pixel): the DMA output no longer matches the advertised fourcc\n",
			 c->bytes_per_px_x8, CAMSV_BPL_X8_U16);
	if (!c->bytes_per_px_x8)
		dev_warn(c->dev,
			 "bytes_per_px_x8=0 falls back to a 10-bit line: the DMA output no longer matches the advertised fourcc\n");

	/*
	 * Layout override: the bytes-per-line follows bytes_per_px_x8, so
	 * buffers queued for a different BPL must be re-derived.  S_FMT has
	 * already run with the current parameter value; recompute here in
	 * case the parameter changed since (vb2 is not streaming yet).
	 * bytes_per_px_x8 = 0 keeps the standard width * 2.
	 */
	c->format.bytesperline = camsv_bpl_stream(c->format.width,
						  c->bytes_per_px_x8);
	c->format.sizeimage = c->format.bytesperline * c->format.height;

	/*
	 * Scratch buffer: the parking target (rotate) or the DMA target
	 * (bounce).  Cacheable, below 4 GB (32-bit mask) and physically
	 * contiguous (dma-direct + CMA, no IOMMU on this device).
	 */
	c->scratch_size = PAGE_ALIGN(c->format.sizeimage);
	c->scratch = dma_alloc_noncoherent(c->dev, c->scratch_size,
					   &c->scratch_dma, DMA_BIDIRECTIONAL,
					   GFP_KERNEL);
	if (!c->scratch) {
		dev_err(c->dev, "cannot allocate a %zu-byte scratch buffer\n",
			c->scratch_size);
		ret = -ENOMEM;
		goto err_buffers;
	}
	camsv_mark_set(c, c->scratch, true);

	if (c->rotate) {
		/* The first frame's buffer, programmed before the TG runs. */
		c->cur = camsv_pop(c);
		if (!c->cur) {
			ret = -EINVAL;
			goto err_scratch;
		}
		if (c->cur->vaddr)
			camsv_mark_set(c, c->cur->vaddr, false);
		target = c->cur->dma_addr;
	} else {
		target = c->scratch_dma;
	}
	c->stats.last_base = lower_32_bits(target);

	ret = video_device_pipeline_start(&c->vdev, &c->pipeline);
	if (ret)
		goto err_scratch;

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
	 * start SENINF + PHY + sensor, then start the TG.  Starting the TG
	 * before the sensor streams was the failure mode that left IMGO
	 * stuck in IMGO_ERR on this tablet.  The FBC compare value starts
	 * fbc_margin frames ahead of the written count (camsv_fbc_run()).
	 */
	camsv_hw_config(c, target, c->format.width, c->format.height,
			c->format.bytesperline);

	ret = v4l2_subdev_call(seninf_sd, video, s_stream, 1);
	if (ret) {
		dev_err(c->dev, "failed to stream on SENINF: %d\n", ret);
		goto err_stop;
	}

	WRITE_ONCE(c->streaming, true);
	camsv_hw_enable(c);

	dev_info(c->dev,
		 "streaming %ux%u, FMT_SEL 0x%03x, PAK %u, bpl %u (%u/8 bytes/px), %s, FBC count %u compare %u, first target 0x%08x\n",
		 c->format.width, c->format.height,
		 c->fmt_sel ?: CAMSV_FMT_SEL_U16, c->pak,
		 c->format.bytesperline,
		 c->bytes_per_px_x8 ?: MT6785_CAM_BPP,
		 c->rotate ? "zero copy" : "bounce buffer", c->last_wcnt,
		 c->fbc_cur_num, lower_32_bits(target));

	return 0;

err_stop:
	camsv_hw_stop(c);
	synchronize_irq(c->irq);
	camsv_larb_bypass(c, false);
	pm_runtime_mark_last_busy(c->dev);
	pm_runtime_put_autosuspend(c->dev);
err_pipeline:
	video_device_pipeline_stop(&c->vdev);
err_scratch:
	camsv_free_scratch(c);
err_buffers:
	/*
	 * Every buffer vb2 handed us must go back before returning an
	 * error, or vb2_start_streaming() WARNs (seen on the t8b boot).
	 */
	camsv_return_all_buffers(c, VB2_BUF_STATE_QUEUED);

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

	WRITE_ONCE(c->streaming, false);
	camsv_hw_stop(c);
	/* Neither half of the interrupt handler may still be running. */
	synchronize_irq(c->irq);
	camsv_larb_bypass(c, false);

	camsv_return_all_buffers(c, VB2_BUF_STATE_ERROR);
	camsv_free_scratch(c);
	c->gated = false;

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

/* Defined with the video node ioctls (walks the media graph). */
static int camsv_sensor_frame_size(struct mt6785_camsv *c, u32 code,
				   u32 index, u32 *width, u32 *height);

/*
 * Default format on both pads: the validated capture geometry
 * (640x480 SBGGR10), matching the seninf default and the ov8856
 * 24 MHz mode so link validation passes on a fresh boot.  Raw Bayer on
 * the bus: colorspace raw, no transfer function.
 */
static const struct v4l2_mbus_framefmt camsv_default_fmt = {
	.code = MEDIA_BUS_FMT_SBGGR10_1X10,
	.width = 640,
	.height = 480,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
	.xfer_func = V4L2_XFER_FUNC_NONE,
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
 * Frame sizes on either subdev pad: the geometry is pass-through, so
 * the sizes are the active sensor's (through the enabled seninf link),
 * same as the video node's ENUM_FRAMESIZES.  Without a reachable
 * sensor the caller gets -ENODEV (userspace then relies on set_fmt
 * clamping).
 */
static int camsv_subdev_enum_frame_size(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  struct v4l2_subdev_frame_size_enum *fse)
{
	struct mt6785_camsv *c = sd_to_camsv(sd);
	int ret;

	if (fse->pad > 1)
		return -EINVAL;

	if (!camsv_format_by_code(fse->code))
		return -EINVAL;

	ret = camsv_sensor_frame_size(c, fse->code, fse->index,
				      &fse->min_width, &fse->min_height);
	if (ret)
		return ret;

	fse->max_width = fse->min_width;
	fse->max_height = fse->min_height;

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
	.enum_frame_size = camsv_subdev_enum_frame_size,
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
	const struct mt6785_camsv_format *fmt;
	u32 index = f->index;
	unsigned int i;

	/*
	 * The node has V4L2_CAP_IO_MC, so the core passes userspace's
	 * mbus_code filter through: enumerate only the fourcc paired with
	 * that media bus code (all codes without a filter).
	 */
	for (i = 0; i < ARRAY_SIZE(camsv_formats); i++) {
		fmt = &camsv_formats[i];

		if (f->mbus_code && fmt->code != f->mbus_code)
			continue;
		if (index--)
			continue;

		f->pixelformat = fmt->fourcc;
		return 0;
	}

	return -EINVAL;
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
	pix->bytesperline = camsv_bpl_sweep(pix->width);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
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
	const struct mt6785_camsv_format *fmt;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *padfmt;

	if (vb2_is_busy(&c->vbq))
		return -EBUSY;

	camsv_try_fmt(c, &f->fmt.pix);
	c->format = f->fmt.pix;

	/*
	 * Keep the subdev pads in step with the node: link validation
	 * between the seninf source and this sink compares bus codes, and
	 * userspace picks the Bayer order through the fourcc.  Without
	 * this, selecting a pgAA node format leaves pad 0 at BGGR and
	 * STREAMON fails with EPIPE.
	 */
	fmt = camsv_format_by_fourcc(c->format.pixelformat);
	if (WARN_ON(!fmt))
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(&c->sd);
	padfmt = v4l2_subdev_state_get_format(state, 0, 0);
	padfmt->width = c->format.width;
	padfmt->height = c->format.height;
	padfmt->code = fmt->code;
	*v4l2_subdev_state_get_format(state, 1, 0) = *padfmt;
	v4l2_subdev_unlock_state(state);

	return 0;
}

static int camsv_g_fmt_ioctl(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	f->fmt.pix = vdev_to_camsv(video_devdata(file))->format;

	return 0;
}

/*
 * Frame sizes of the sensor behind the enabled seninf link, enumerated
 * from its subdev (V4L2_SUBDEV_FORMAT_ACTIVE, NULL state: the in-kernel
 * call path used by e.g. stm32-dcmi).  Returns -ENODEV when no sensor
 * is reachable; the caller then falls back to the continuous range.
 */
static int camsv_sensor_frame_size(struct mt6785_camsv *c, u32 code,
				     u32 index, u32 *width, u32 *height)
{
	struct v4l2_subdev_frame_size_enum fse = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.code = code,
		.index = index,
	};
	struct v4l2_subdev *seninf_sd, *sensor_sd;
	struct media_pad *sensor_pad = NULL;
	struct media_link *link;
	int ret;

	if (!c->seninf_pad)
		c->seninf_pad = media_pad_remote_pad_first(&c->pads[0]);
	if (!c->seninf_pad)
		return -ENODEV;

	/*
	 * The sensor behind the SENINF's ENABLED sink link (the selected
	 * CSI port) — not media_pad_remote_pad_first() on the seninf source
	 * pad, which returns this CAMSV (the first enabled source link).
	 * The entity test keeps the seninf->camsv forward link (its sink is
	 * the camsv) out; the pad index test keeps pad 3 (the source)
	 * out, leaving only the CSI port sinks.
	 */
	seninf_sd = media_entity_to_v4l2_subdev(c->seninf_pad->entity);
	list_for_each_entry(link, &seninf_sd->entity.links, list) {
		if ((link->flags & MEDIA_LNK_FL_ENABLED) &&
		    link->sink->entity == &seninf_sd->entity &&
		    link->sink->index < MT6785_CAM_PAD_SRC) {
			sensor_pad = link->source;
			break;
		}
	}
	if (!sensor_pad)
		return -ENODEV;

	sensor_sd = media_entity_to_v4l2_subdev(sensor_pad->entity);
	ret = v4l2_subdev_call(sensor_sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	*width = fse.max_width;
	*height = fse.max_height;

	return 0;
}

static int camsv_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *sizes)
{
	struct mt6785_camsv *c = vdev_to_camsv(video_devdata(file));
	const struct mt6785_camsv_format *fmt;
	u32 width, height;

	fmt = camsv_format_by_fourcc(sizes->pixel_format);
	if (!fmt)
		return -EINVAL;

	/*
	 * Discrete sizes from the active sensor when it can be reached
	 * (its enum_frame_size needs a bus code it emits); otherwise the
	 * continuous DMA window range — the geometry is pass-through, so
	 * any size the sensor is then set to by hand still streams.
	 */
	if (!camsv_sensor_frame_size(c, fmt->code, sizes->index,
				     &width, &height)) {
		sizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		sizes->discrete.width = width;
		sizes->discrete.height = height;

		return 0;
	}

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

	dev_info(c->dev, "MT6785 CAMSV0 registered as /dev/video%d (unpacked 10-bit Bayer, 2 bytes/px)\n",
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

/* --------------------------------------------------------------------------
 * Debugfs: the CAMSV side of the status dump (mirrors ov-cap.py's
 * "CAMSV0:" line).  The SENINF side lives in the seninf driver; both
 * are one-line reads usable without /dev/mem.
 */

static int camsv_status_show(struct seq_file *m, void *v)
{
	struct mt6785_camsv *c = m->private;
	struct mt6785_camsv_stats st = c->stats;	/* racy snapshot */
	u32 int_en, frmsize, fbc1, fbc2, base;
	unsigned int bit;
	int ret;

	/* Registers are only readable with the CAM domain and clocks on. */
	ret = pm_runtime_resume_and_get(c->dev);
	if (ret < 0) {
		seq_printf(m, "power domain unavailable: %d\n", ret);
		return 0;
	}

	/* Not INT_STATUS: reading it clears it under the handler's feet. */
	int_en = camsv_rd(c, CAMSV_INT_EN);
	frmsize = camsv_rd(c, CAMSV_TG_FRMSIZE);
	fbc1 = camsv_rd(c, CAMSV_FBC_IMGO_CTL1);
	fbc2 = camsv_rd(c, CAMSV_FBC_IMGO_CTL2);
	base = camsv_rd(c, CAMSV_IMGO_BASE_ADDR);

	pm_runtime_mark_last_busy(c->dev);
	pm_runtime_put_autosuspend(c->dev);

	seq_printf(m, "mode        %s, FBC margin %u, streaming %d\n",
		   c->rotate ? "zero copy (IMGO_BASE_ADDR per frame)" :
			       "bounce buffer + copy",
		   c->fbc_margin, READ_ONCE(c->streaming));
	seq_printf(m, "layout      FMT_SEL 0x%03x, PAK %u, bpl %u (%u/8 bytes/px)%s\n",
		   c->fmt_sel ?: CAMSV_FMT_SEL_U16, c->pak,
		   c->format.bytesperline,
		   c->bytes_per_px_x8 ?: MT6785_CAM_BPP,
		   (c->fmt_sel && c->fmt_sel != CAMSV_FMT_SEL_U16) ||
		   c->pak != CAMSV_PAK_U16 ||
		   (c->bytes_per_px_x8 &&
		    c->bytes_per_px_x8 != CAMSV_BPL_X8_U16) ||
		   !c->bytes_per_px_x8 ? "  [OVERRIDDEN]" : "");
	seq_printf(m, "INT_EN      0x%08x  (write-clear bit31 %d; 0 = masked)\n",
		   int_en, !!(int_en & CAMSV_INT_WCLR_EN));
	seq_printf(m, "TG_FRMSIZE  0x%08x  (%u px x %u lines)\n",
		   frmsize, frmsize >> 16, frmsize & 0xffff);
	seq_printf(m, "FBC_CTL1    0x%08x  (compare %lu)  FBC_CTL2 0x%08x  (written count %lu, [23:16] %lu, dropped %lu)%s\n",
		   fbc1, FIELD_GET(CAMSV_FBC_NUM_MASK, fbc1), fbc2,
		   FIELD_GET(CAMSV_FBC2_WCNT, fbc2),
		   FIELD_GET(GENMASK(23, 16), fbc2),
		   FIELD_GET(CAMSV_FBC2_DROP, fbc2),
		   (fbc1 & CAMSV_FBC_EN) &&
		   (!FIELD_GET(CAMSV_FBC_NUM_MASK, fbc1) ||
		    FIELD_GET(CAMSV_FBC_NUM_MASK, fbc1) ==
		    FIELD_GET(CAMSV_FBC2_WCNT, fbc2)) ? "  STOPPED" : "");
	seq_printf(m, "IMGO_BASE   0x%08x  (last programmed 0x%08x, scratch 0x%08x)\n",
		   base, st.last_base, lower_32_bits(c->scratch_dma));
	seq_printf(m, "pass1_don %u  hw_written %u  hw_idle %u  hw_multi %u  drops %u  stalls %u\n",
		   st.pass1, st.hw_frames, st.hw_idle, st.hw_multi, st.drops,
		   st.stalls);
	seq_printf(m, "delivered %u  missed %u  torn %u  short %u  disagree %u  parked %u  recycled %u  copy_races %u  errors %u\n",
		   st.frames, st.missed, st.torn, st.short_frames, st.disagree,
		   st.parked, st.recycled, st.copy_races, st.errors);
	seq_printf(m, "irqs %u  storms %u  last INT_STATUS 0x%08x  FBC_CTL2 at last PASS1_DON 0x%08x\n",
		   st.irqs, st.storms, st.last_status, st.last_fbc2);
	seq_puts(m, "INT_STATUS bit counts:");
	for (bit = 0; bit < 32; bit++)
		if (st.bits[bit])
			seq_printf(m, " b%u=%u", bit, st.bits[bit]);
	seq_puts(m, "\n");

	return 0;
}

static int camsv_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, camsv_status_show, inode->i_private);
}

static const struct file_operations camsv_status_fops = {
	.owner = THIS_MODULE,
	.open = camsv_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
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
	c->irq = irq;

	/*
	 * Hard half clears and counts, the thread checks, copies and arms
	 * (it may sleep).  Not IRQF_ONESHOT: the hard half clears the
	 * source, and frame ends must keep being counted while the thread
	 * copies a large frame.
	 */
	ret = devm_request_threaded_irq(dev, irq, camsv_irq, camsv_irq_thread,
					0, dev_name(dev), c);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq %d\n", irq);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set 32-bit DMA mask\n");

	/* Default format until userspace sets one. */
	c->format.pixelformat = camsv_formats[0].fourcc;
	c->format.width = camsv_default_fmt.width;
	c->format.height = camsv_default_fmt.height;
	c->format.bytesperline = camsv_bpl_sweep(c->format.width);
	c->format.sizeimage = c->format.bytesperline * c->format.height;
	c->format.field = V4L2_FIELD_NONE;
	c->format.colorspace = V4L2_COLORSPACE_RAW;
	c->format.xfer_func = V4L2_XFER_FUNC_NONE;

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
	 * The active state must exist before the subdev can be bound: the
	 * SENINF notifier may bind it (and register the subdev nodes)
	 * from inside v4l2_async_register_subdev() below.
	 */
	ret = v4l2_subdev_init_finalize(&c->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev state\n");
		goto err_pads;
	}

	/*
	 * Async registration: when the SENINF notifier binds this subdev,
	 * .registered creates the video node under the pipeline's
	 * v4l2_device.
	 */
	ret = v4l2_async_register_subdev(&c->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_state;
	}

	pm_runtime_set_autosuspend_delay(dev, MT6785_CAMSV_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	devm_pm_runtime_enable(dev);
	pm_runtime_set_suspended(dev);

	/* Status dump (see camsv_status_show); registers need the CAM domain. */
	c->dbg_status = debugfs_create_file("mt6785-camsv-status", 0444,
					    NULL, c, &camsv_status_fops);

	dev_dbg(dev, "MT6785 CAMSV0 probe complete (video node on bind)\n");

	return 0;

err_state:
	v4l2_subdev_cleanup(&c->sd);
err_pads:
	media_entity_cleanup(&c->sd.entity);
	return ret;
}

static void camsv_remove(struct platform_device *pdev)
{
	struct mt6785_camsv *c = dev_get_drvdata(&pdev->dev);

	debugfs_remove(c->dbg_status);

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
