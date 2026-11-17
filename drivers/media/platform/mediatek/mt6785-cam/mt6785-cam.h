/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared definitions of the MT6785 camera driver pair (SENINF + CAMSV0).
 *
 * The two drivers form one media pipeline:
 *
 *   sensor subdev -> mtk seninf subdev -> mtk camsv subdev -> /dev/videoN
 *   (ov8856)        (CSI port sink pads)  (sink from seninf,
 *                                          source to the video node)
 */
#ifndef MT6785_CAM_H
#define MT6785_CAM_H

/* MIPI RX CSI ports of the MT6785 (ANA window 0x11c80000, stride 0x4000). */
#define MT6785_CAM_CSI_PORTS		3

/*
 * Media pads of the SENINF subdev: pads 0..2 are the CSI port sinks,
 * pad 3 is the single source towards CAMSV0.
 */
#define MT6785_CAM_PAD_SRC		3
#define MT6785_CAM_NUM_PADS		4

/*
 * CAMSV0 is hardwired to SENINF mux 2 / cam_mux 2 (vendor HAL
 * convertTgEnum2CamMux: tg - 14), so the source pad always routes through
 * TOP_MUX 2 and CAM_MUX 2 (docs/hardware/CAMERA.md, begonia NOTES.md).
 */
#define MT6785_CAM_MUX			2

/* Media bus codes / pixel formats: 10-bit Bayer, MIPI packed (5 bytes/4 px). */
#define MT6785_CAM_BPP			10

#endif /* MT6785_CAM_H */
