/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Himax HX83102 TDDI touchscreen and pen controller driver
 *
 * The HX83102 is a touch and display driver integrated device. Its touch
 * MCU has no firmware flash: the host uploads the touch firmware into the
 * chip SRAM over SPI after every power-up, resume and exception, then
 * verifies it with the chip's hardware CRC engine. Fingers are reported
 * on one input device and the active pen on a second one.
 *
 * Ported from the Himax Android driver sample code for the HX83102
 * chipset, version 2.0.0.68_LC_00, as built by the stock Lenovo kernel
 * (HX_ZERO_FLASH, HX_PROTOCOL_B_3PA, HX_RESUME_SEND_CMD,
 * HX_EXCP_RECOVERY and HX_RST_PIN_FUNC enabled):
 *
 * Copyright (C) 2019 Himax Corporation.
 * Copyright (C) 2026 The Lenovomarchy authors
 *
 * Every constant below names the vendor function it comes from.
 */
#ifndef _HIMAX_HX83102_H
#define _HIMAX_HX83102_H

#include <drm/drm_panel.h>

#include <linux/bitops.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct device;

/*
 * SPI command framing: every register access is a command byte, a
 * register byte and the payload (vendor himax_bus_read() and
 * himax_bus_write() in himax_platform.c).
 */
#define HX83102_SPI_CMD_READ			0xF3
#define HX83102_SPI_CMD_WRITE			0xF2
/* Retry count of the vendor himax_spi_read()/himax_spi_write(). */
#define HX83102_SPI_XFER_RETRIES		10

/*
 * AHB bridge registers in the SPI command space (vendor
 * hx83102_register_read() in himax_ic_HX83102.c and
 * himax_mcu_flash_write_burst_lenth() in himax_ic_incell_core.c). Chip
 * registers behind the bridge are reached by writing the target address
 * to HX83102_AHB_ADDR_BYTE_0, then moving data through the window.
 */
#define HX83102_AHB_ADDR_BYTE_0			0x00
#define HX83102_AHB_ADDR_RDATA_BYTE_0		0x08
#define HX83102_AHB_ADDR_ACCESS_DIR		0x0C
#define HX83102_AHB_ADDR_INCR4			0x0D
#define HX83102_AHB_ADDR_CONTI			0x13
/* Vendor hx83102_burst_enable(). */
#define HX83102_AHB_CMD_CONTI			0x31
#define HX83102_AHB_CMD_INCR4			0x10
#define HX83102_AHB_CMD_DIR_READ		0x00
/* Vendor himax_mcu_interface_on(): poll count for burst mode check. */
#define HX83102_INTERFACE_ON_TRIES		10

/*
 * Safe-mode password, written to the SPI command space (not through the
 * AHB bridge) to halt the touch MCU before an SRAM upload (vendor
 * hx83102e_sense_off() in himax_ic_HX83102.c).
 */
#define HX83102_REG_SPI_PSW_LB			0x31
#define HX83102_REG_SPI_PSW_UB			0x32
#define HX83102_PSW_ENTER_SAFE_LB		0x27
#define HX83102_PSW_ENTER_SAFE_UB		0x95

/*
 * Event stack readout (vendor hx83102e_read_event_stack(): a plain SPI
 * read of the event register, no AHB bridge involved).
 */
#define HX83102_REG_EVENT_STACK			0x30
/* Vendor HIMAX_TOUCH_DATA_SIZE: bytes the firmware pushes per frame. */
#define HX83102_EVENT_STACK_SIZE		128

/* Chip registers behind the AHB bridge. */
#define HX83102_REG_IC_ID			0x900000D0	/* hx83102_chip_detect() */
/* Chip ID read at HX83102_REG_IC_ID: 0x83 0x10 0x2E (vendor retries 5x). */
#define HX83102_CHIP_ID_0			0x83
#define HX83102_CHIP_ID_1			0x10
#define HX83102_CHIP_ID_83102E			0x2E
#define HX83102_CHIP_ID_TRIES			5
/* Vendor himax_mcu_sense_off()/hx83102e_sense_on(): 0xA5 stops the FW. */
#define HX83102_REG_FW_ISR			0x9000005C
#define HX83102_FW_ISR_STOP			0xA5
/* Vendor hx83102e_sense_off(): central state machine status. */
#define HX83102_REG_CS_CENTRAL_STATE		0x900000A8
#define HX83102_CENTRAL_STATE_SAFE		0x0C
/*
 * Vendor hx83102e_sense_off(): six password attempts, with a reset
 * pulse (line low 20 ms, high 50 ms) between attempts.
 */
#define HX83102_SENSE_OFF_TRIES			6
#define HX83102_SENSE_OFF_RETRY_DELAY_US	5000
#define HX83102_SENSE_OFF_RESET_LOW_MS		20
#define HX83102_SENSE_OFF_RESET_HIGH_MS		50
/* Vendor hx83102e_sense_off(): E variant clears the TCON reset. */
#define HX83102_REG_TCON_RST			0x80020020
/* Vendor himax_mcu_firmware_update_0f(): MCU system reset. */
#define HX83102_REG_SYSTEM_RESET			0x90000018
#define HX83102_SYSTEM_RESET_DATA		0x00000055
/*
 * Vendor himax_hx83102d_reload_to_active(): tell the freshly reset MCU
 * to start running the uploaded image; 0x000000EC written until
 * 0x000001EC reads back.
 */
#define HX83102_REG_FW_RELOAD_ACTIVATE		0x90000048
#define HX83102_FW_RELOAD_ACTIVATE_CMD		0x000000EC
#define HX83102_FW_RELOAD_ACTIVATE_ACK		0x000001EC
#define HX83102_REG_RETRY_TIMES			5	/* HIMAX_REG_RETRY_TIMES */
/*
 * Vendor hx83102ab_power_on_init(): the FW writes 0x000072C0 here once
 * it has finished reloading, polled 30 times every 10 ms.
 */
#define HX83102_REG_FW_2ND_RELOAD_DONE		0x100072C0
#define HX83102_FW_2ND_RELOAD_DONE_DATA		0x000072C0
#define HX83102_FW_2ND_RELOAD_POLLS		30
#define HX83102_FW_2ND_RELOAD_POLL_US		10000
/* Vendor hx83102ab_power_on_init(): clear the raw-data output select. */
#define HX83102_REG_RAW_OUT_SEL			0x100072EC	/* hx83102e reg_re_init */
/* Vendor himax_mcu_assign_sorting_mode(), cleared in power_on_init(). */
#define HX83102_REG_SORTING_MODE_EN		0x10007F04
/* Vendor himax_mcu_firmware_update_0f(): "N frame" back to default 1. */
#define HX83102_REG_SET_FRAME			0x10007294
#define HX83102_SET_FRAME_DATA			0x00000001
/* Vendor himax_mcu_read_FW_ver(): informational version registers. */
#define HX83102_REG_FW_VER			0x10007004
#define HX83102_REG_CFG_VER			0x10007084
/*
 * Vendor himax_mcu_reload_disable() as overridden for zero flash by
 * hx_dis_rload_0f(): always writes the disable value (there is no flash
 * to reload from).
 */
#define HX83102_REG_FLASH_RELOAD			0x10007F00
#define HX83102_FLASH_RELOAD_DIS_DATA		0x00009AA9

/*
 * Code overlay handshake (vendor himax_zf_part_info() with
 * HX_CODE_OVERLAY): after the main upload the host clears the
 * handshaking register, brings the firmware up, waits for the
 * algorithm-overlay request (0x11 in all four bytes), uploads the
 * overlay partition and replies (0x22), then waits for the firmware to
 * signal that it finished reloading.
 */
#define HX83102_REG_HANDSHAKE			0x10007FFC
#define HX83102_HANDSHAKE_CLEAR_DATA		0xA55A5AA5
#define HX83102_HANDSHAKE_OVL_REQUEST		0x11111111
#define HX83102_HANDSHAKE_OVL_REPLY		0x22222222
#define HX83102_OVL_REQUEST_TRIES		60

/*
 * Hardware CRC engine (vendor himax_mcu_check_CRC() in
 * himax_ic_incell_core.c): start address and length in words, the
 * length prefixed with 0x99, are written to the engine and the CRC32C
 * result is read back. The engine doubles as the SRAM reload engine.
 */
#define HX83102_REG_CRC_ADDR_FROM		0x80050020
#define HX83102_REG_CRC_CMD_BEAT			0x80050028
#define HX83102_CRC_CMD_BEAT_PREFIX		0x99
#define HX83102_REG_CRC_STATUS			0x80050000
#define HX83102_CRC_STATUS_BUSY			BIT(0)
#define HX83102_REG_CRC_RESULT			0x80050018
#define HX83102_CRC_POLL_TRIES			100
#define HX83102_CRC_POLL_US			1000
/*
 * Software CRC32C the config block is verified against (poly from
 * himax_mcu_Calculate_CRC_with_AP(), init 0xFFFFFFFF, no final XOR, fed
 * with big-endian words).
 */
#define HX83102_CRC32C_POLY			0x82F63B78

/*
 * Reset timing. himax_mcu_pin_reset() with HX_ZERO_FLASH uses 5 ms per
 * phase; the power-on sequence of himax_gpio_power_config() toggles the
 * line with 20 ms phases.
 */
#define HX83102_RESET_ASSERT_US			5000
#define HX83102_RESET_DEASSERT_US		5000
#define HX83102_POWER_ON_PHASE_MS		20

/*
 * Zero-flash firmware image layout (vendor himax_zf_part_info() and
 * hx83102e_chip_init() in himax_ic_incell_core.c / himax_ic_HX83102.c).
 * The image carries a partition table at a fixed offset; each entry is
 * 16 bytes: 4 SRAM address bytes (little endian, used verbatim as the
 * AHB write address), a big-endian size, a big-endian offset into the
 * firmware file, and a marker pair. Partition 0 is the firmware code;
 * the remaining partitions are configurations that share one 32 KiB
 * DSRAM window and are uploaded and verified as a single block. A
 * partition marked 0x88 0x77 is a code overlay, uploaded after the
 * main firmware through the handshake above.
 */
#define HX83102_ZF_TABLE_OFFSET			0x10400	/* hx83102e_chip_init() */
#define HX83102_ZF_TABLE_COUNT_OFF		12
#define HX83102_ZF_TABLE_MAX_PARTS		16
#define HX83102_ZF_PART_SIZE_OFF			4
#define HX83102_ZF_PART_FWADDR_OFF		8
#define HX83102_ZF_PART_MARK_OFF			14
/* Marker pair of an overlay partition: buf[14] = 0x88, buf[15] = 0x77. */
#define HX83102_ZF_PART_MARK_OVERLAY_0		0x88
#define HX83102_ZF_PART_MARK_OVERLAY_1		0x77
#define HX83102_ZF_DSRAM_SIZE			(32 * 1024)	/* DSRAM_SIZE */
/* himax_zf_part_info() rounds the config block size up to 16 bytes. */
#define HX83102_ZF_CFG_ALIGN			16
/* himax_zf_part_info() rounds config addresses down to 4 bytes. */
#define HX83102_ZF_CFG_ADDR_ALIGN		4
/*
 * SRAM write chunk. The vendor writes up to HX_MAX_WRITE_SZ - 4 bytes
 * per transfer (himax_mcu_write_sram_0f()); this driver uses the same
 * addressing with smaller chunks so each transfer stays well within
 * common SPI controller limits.
 */
#define HX83102_ZF_WRITE_CHUNK			4096
/* Vendor himax_sram_write_crc_check(): write plus CRC verify attempts. */
#define HX83102_ZF_CRC_RETRIES			3
/* Vendor himax_mcu_firmware_update_0f(): whole upload attempts. */
#define HX83102_ZF_UPDATE_RETRIES		2
/* Vendor himax_mcu_register_write(): pause between SRAM chunks. */
#define HX83102_ZF_CHUNK_GAP_US			100

/*
 * Event packet layout with 10 fingers and pen support (vendor
 * himax_report_data_init() and himax_parse_report_points() in
 * himax_common.c): ten 4-byte finger coordinates, a 12-byte touch
 * major/padding area, then the finger count byte, three state bytes and
 * the 12-byte pen block. The byte sum of the packet must be a multiple
 * of 0x100.
 */
#define HX83102_MAX_TOUCHES			10	/* FIX_HX_MAX_PT */
#define HX83102_EVENT_FINGER_SIZE		4
#define HX83102_EVENT_FINGER_COORD_SIZE		(HX83102_MAX_TOUCHES * \
						 HX83102_EVENT_FINGER_SIZE)
#define HX83102_EVENT_MAJOR_SIZE		12
#define HX83102_EVENT_COUNT_OFF			(HX83102_EVENT_FINGER_COORD_SIZE + \
						 HX83102_EVENT_MAJOR_SIZE)
#define HX83102_EVENT_COUNT_MASK		GENMASK(3, 0)
/*
 * Vendor himax_parse_report_data(): finger count byte 0xFF means "all
 * fingers up", like a count of zero.
 */
#define HX83102_EVENT_COUNT_ALL_LEAVE		0xFF
/*
 * Vendor himax_checksum_cal(): the high nibble of the count byte
 * selects the raw-data output; frames not addressed to the host are
 * dropped and the rest of the event stack flushed.
 */
#define HX83102_EVENT_RAWSEL_MASK		GENMASK(7, 4)
#define HX83102_EVENT_RAWSEL_HOST		0x0F
#define HX83102_EVENT_RAWSEL_NONE		0x00
#define HX83102_EVENT_PEN_SIZE			12	/* PEN_INFO_SZ */
#define HX83102_EVENT_TOUCH_INFO_SIZE		(HX83102_EVENT_COUNT_OFF + 4 + \
						 HX83102_EVENT_PEN_SIZE)
#define HX83102_EVENT_EXCP_SCAN_SIZE		(HX83102_EVENT_TOUCH_INFO_SIZE - \
						 HX83102_EVENT_PEN_SIZE)

/* Pen block offsets (vendor himax_parse_report_points()). */
#define HX83102_PEN_X_OFF			0
#define HX83102_PEN_Y_OFF			2
#define HX83102_PEN_PRESSURE_OFF		4
#define HX83102_PEN_TILT_X_OFF			6
#define HX83102_PEN_HOVER_OFF			7
#define HX83102_PEN_BTN1_OFF			8
#define HX83102_PEN_BTN2_OFF			9
#define HX83102_PEN_TILT_Y_OFF			10
#define HX83102_PEN_TILT_RANGE			60	/* himax_input_register() */
#define HX83102_PEN_PRESSURE_MAX		4095	/* himax_input_register() */
#define HX83102_PEN_DISTANCE_MAX		1	/* himax_input_register() */

/*
 * Exception event patterns (vendor himax_ts_event_check() in
 * himax_common.c): a failed-checksum frame whose finger area is filled
 * with one of these bytes is the firmware signalling a watchdog or ESD
 * event. 0xEB/0xEC/0xEE trigger a full firmware re-upload; 0xED, seen
 * after a reset, only asks the MCU to start the image again.
 */
#define HX83102_EXCP_EB				0xEB
#define HX83102_EXCP_EC				0xEC
#define HX83102_EXCP_EE				0xEE
#define HX83102_EXCP_ED				0xED
/* Rate limit of the recovery re-upload, against exception storms. */
#define HX83102_EXCP_COOLDOWN_MS		1000

#define HX83102_FW_NAME_DEFAULT			"himax/hx83102p.bin"
#define HX83102_NAME				"himax-hx83102"

/** enum hx83102_event_result - what the packet decoder recognised */
enum hx83102_event_result {
	HX83102_EVENT_FRAME,		/**< decoded, report fingers and pen */
	HX83102_EVENT_DROP,		/**< bad frame, ignore */
	HX83102_EVENT_FLUSH,		/**< not addressed to host, flush stack */
	HX83102_EVENT_EXCP,		/**< 0xEB/EC/EE pattern, re-upload */
	HX83102_EVENT_RELOAD,		/**< 0xED pattern, restart the image */
};

/** struct hx83102_finger_event - one decoded finger slot */
struct hx83102_finger_event {
	u16 x;
	u16 y;
	u8 major;
	bool active;
};

/** struct hx83102_pen_event - decoded pen block */
struct hx83102_pen_event {
	u16 x;
	u16 y;
	u16 pressure;
	s16 tilt_x;
	s16 tilt_y;
	u8 hover;
	u8 btn1;
	u8 btn2;
	bool active;
};

/** struct hx83102_touch_event - decoded event packet */
struct hx83102_touch_event {
	/** @leave: no finger and no pen: release everything */
	bool leave;
	/** @fingers: per-slot finger data */
	struct hx83102_finger_event fingers[HX83102_MAX_TOUCHES];
	/** @pen: pen data */
	struct hx83102_pen_event pen;
};

/** struct hx83102_fw_layout - parsed zero-flash firmware layout */
struct hx83102_fw_layout {
	/** @code_sram_addr: destination of partition 0 (the code) */
	u32 code_sram_addr;
	/** @code_fw_off: offset of the code in the firmware file */
	u32 code_fw_off;
	/** @code_size: size of the code block */
	u32 code_size;
	/** @cfg_image: assembled configuration block, cfg_size bytes */
	u8 *cfg_image;
	/** @cfg_sram_addr: SRAM base of the configuration block */
	u32 cfg_sram_addr;
	/** @cfg_size: size of the configuration block */
	u32 cfg_size;
	/** @cfg_crc: CRC32C of the configuration block */
	u32 cfg_crc;
	/** @has_overlay: the image carries a code overlay partition */
	bool has_overlay;
	/** @overlay_sram_addr: destination of the overlay partition */
	u32 overlay_sram_addr;
	/** @overlay_fw_off: offset of the overlay in the firmware file */
	u32 overlay_fw_off;
	/** @overlay_size: size of the overlay partition */
	u32 overlay_size;
};

/** struct hx83102_transport_ops - register access of the bus transport */
struct hx83102_transport_ops {
	/** @read_reg: read @len bytes at SPI register @reg */
	int (*read_reg)(struct device *dev, u8 reg, void *data, size_t len);
	/** @write_reg: write @len bytes at SPI register @reg */
	int (*write_reg)(struct device *dev, u8 reg, const void *data,
			 size_t len);
};

/**
 * struct hx83102_data - driver state of one HX83102
 *
 * @dev: underlying device, owned by the SPI transport.
 * @ops: register access provided by the transport.
 * @input: finger input device (MT protocol B).
 * @pen_input: pen input device.
 * @fw_lock: serialises chip power sequencing (start, stop, recovery).
 * @panel_follower: descriptor when power-sequenced with a DRM panel.
 * @panel_follower_prepare_work: start work scheduled by the follower.
 * @recovery_work: rate-limited firmware re-upload after an exception.
 * @recovery_jiffies: when the last recovery was queued.
 * @reset_gpio: XRES line of the TDDI chip.
 * @prop: touchscreen properties (swaps, inversions), shared by both
 *	input devices.
 * @fw_name: firmware file name from DT or the default.
 * @fw: cached firmware, kept for the chip's lifetime so resume and
 *	recovery never touch the file system.
 * @layout: parsed partition layout of @fw.
 * @irq: touch interrupt, falling edge.
 * @width: maximum valid X coordinate.
 * @height: maximum valid Y coordinate.
 * @started: firmware uploaded and interrupt enabled.
 * @chip_verified: chip ID has been read and matched once.
 * @pen_down: pen was reported in range in the last frame.
 * @prepare_work_finished: follower prepare work completed successfully.
 * @event: scratch buffer of one event stack frame.
 * @xfer_buf: address plus payload bounce buffer for SRAM writes.
 */
struct hx83102_data {
	struct device *dev;
	const struct hx83102_transport_ops *ops;
	struct input_dev *input;
	struct input_dev *pen_input;
	/* serialises chip power sequencing (start, stop, recovery) */
	struct mutex fw_lock;
	struct drm_panel_follower panel_follower;
	struct work_struct panel_follower_prepare_work;
	struct work_struct recovery_work;
	unsigned long recovery_jiffies;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;
	const char *fw_name;
	const struct firmware *fw;
	struct hx83102_fw_layout layout;
	int irq;
	u32 width;
	u32 height;
	bool started;
	bool chip_verified;
	bool pen_down;
	bool prepare_work_finished;
	u8 event[HX83102_EVENT_STACK_SIZE];
	u8 xfer_buf[HX83102_ZF_WRITE_CHUNK + 4];
};

extern const struct dev_pm_ops hx83102_pm_ops;

int hx83102_probe(struct device *dev, int irq,
		  const struct hx83102_transport_ops *ops);
void hx83102_remove(struct device *dev);

#if IS_ENABLED(CONFIG_KUNIT)
/* Symbols shared with the KUnit test module. */
u32 hx83102_crc32c(const u8 *data, size_t len);
int hx83102_parse_fw(const struct firmware *fw,
		     struct hx83102_fw_layout *layout);
int hx83102_parse_event(const u8 *buf, u32 max_x, u32 max_y,
			struct hx83102_touch_event *event);
#endif

#endif /* _HIMAX_HX83102_H */
