// SPDX-License-Identifier: GPL-2.0-only
/*
 * Himax HX83102 TDDI touchscreen and pen controller driver
 *
 * Copyright (C) 2026 The Lenovomarchy authors
 *
 * Ported from the Himax Android driver sample code for the HX83102
 * chipset, version 2.0.0.68_LC_00, as built by the stock Lenovo kernel
 * (see himax_hx83102.h for the vendor configuration):
 *
 * Copyright (C) 2019 Himax Corporation.
 *
 * The touch MCU of this TDDI chip has no firmware flash. After reset,
 * resume and exceptions the host uploads the firmware into the chip
 * SRAM over SPI in partitions, verifies each with the hardware CRC
 * engine and then starts the image. Fingers are reported as MT protocol
 * B slots; the active pen (pressure, tilt, barrel buttons) is reported
 * on a second input device.
 */

#include <kunit/visibility.h>

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kconfig.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "himax_hx83102.h"

/* ---------------------------------------------------------------------------
 * Register access
 * ---------------------------------------------------------------------------
 */

static int hx83102_bus_read(struct hx83102_data *ts, u8 reg, void *data,
			    size_t len)
{
	return ts->ops->read_reg(ts->dev, reg, data, len);
}

static int hx83102_bus_write(struct hx83102_data *ts, u8 reg,
			     const void *data, size_t len)
{
	return ts->ops->write_reg(ts->dev, reg, data, len);
}

/* Vendor hx83102_burst_enable(): select continuous burst mode. */
static int hx83102_burst_enable(struct hx83102_data *ts, bool burst)
{
	u8 val;
	int ret;

	val = HX83102_AHB_CMD_CONTI;
	ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_CONTI, &val, 1);
	if (ret)
		return ret;

	val = HX83102_AHB_CMD_INCR4 | (burst ? 1 : 0);
	return hx83102_bus_write(ts, HX83102_AHB_ADDR_INCR4, &val, 1);
}

/*
 * Vendor himax_mcu_interface_on(): knock the bridge awake and confirm
 * burst mode sticks (needed right after a reset).
 */
static int hx83102_interface_on(struct hx83102_data *ts)
{
	u8 conti;
	u8 incr4;
	u8 dummy[4];
	int try;
	int ret;

	/* Dummy read to wake the interface up. */
	ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_RDATA_BYTE_0, dummy,
			       sizeof(dummy));
	if (ret)
		return ret;

	for (try = 0; try < HX83102_INTERFACE_ON_TRIES; try++) {
		ret = hx83102_burst_enable(ts, false);
		if (ret)
			return ret;

		ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_CONTI, &conti, 1);
		if (ret)
			return ret;

		ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_INCR4, &incr4, 1);
		if (ret)
			return ret;

		if (conti == HX83102_AHB_CMD_CONTI &&
		    incr4 == HX83102_AHB_CMD_INCR4)
			return 0;

		usleep_range(1000, 1100);
	}

	return -ETIMEDOUT;
}

/* Vendor hx83102_register_read(): move len bytes out of a chip register. */
static int hx83102_ahb_read(struct hx83102_data *ts, u32 addr, void *data,
			    size_t len)
{
	u8 addr_le[4];
	u8 val;
	int ret;

	put_unaligned_le32(addr, addr_le);

	ret = hx83102_burst_enable(ts, len > 4);
	if (ret)
		return ret;

	ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_BYTE_0, addr_le,
				sizeof(addr_le));
	if (ret)
		return ret;

	val = HX83102_AHB_CMD_DIR_READ;
	ret = hx83102_bus_write(ts, HX83102_AHB_ADDR_ACCESS_DIR, &val, 1);
	if (ret)
		return ret;

	ret = hx83102_bus_read(ts, HX83102_AHB_ADDR_RDATA_BYTE_0, data, len);
	if (ret)
		return ret;

	if (len > 4)
		return hx83102_burst_enable(ts, false);

	return 0;
}

/*
 * Vendor himax_mcu_flash_write_burst_lenth(): write a chip register.
 * The transfer carries the 4 address bytes followed by the payload, so
 * unlike the read path it needs no separate address phase. Uses the
 * pre-allocated bounce buffer, as chunks reach 4 KiB.
 */
static int hx83102_ahb_write(struct hx83102_data *ts, u32 addr,
			     const void *data, size_t len)
{
	int ret;

	if (len > HX83102_ZF_WRITE_CHUNK)
		return -EINVAL;

	put_unaligned_le32(addr, ts->xfer_buf);
	memcpy(ts->xfer_buf + 4, data, len);

	ret = hx83102_burst_enable(ts, len > 4);
	if (ret)
		return ret;

	return hx83102_bus_write(ts, HX83102_AHB_ADDR_BYTE_0, ts->xfer_buf,
				 len + 4);
}

static int hx83102_ahb_write_u32(struct hx83102_data *ts, u32 addr, u32 val)
{
	__le32 le = cpu_to_le32(val);

	return hx83102_ahb_write(ts, addr, &le, sizeof(le));
}

static int hx83102_ahb_read_u32(struct hx83102_data *ts, u32 addr, u32 *val)
{
	__le32 le;
	int ret;

	ret = hx83102_ahb_read(ts, addr, &le, sizeof(le));
	if (!ret)
		*val = le32_to_cpu(le);

	return ret;
}

/* ---------------------------------------------------------------------------
 * Chip operations
 * ---------------------------------------------------------------------------
 */

/* Vendor himax_mcu_pin_reset(): XRES asserted then released. */
static void hx83102_pin_reset(struct hx83102_data *ts)
{
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(HX83102_RESET_ASSERT_US, HX83102_RESET_ASSERT_US + 100);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	usleep_range(HX83102_RESET_DEASSERT_US,
		     HX83102_RESET_DEASSERT_US + 100);
}

/*
 * Vendor hx83102e_sense_off() retry reset: line low 20 ms, high 50 ms,
 * slower than the short pin reset above.
 */
static void hx83102_pin_reset_slow(struct hx83102_data *ts)
{
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(HX83102_SENSE_OFF_RESET_LOW_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(HX83102_SENSE_OFF_RESET_HIGH_MS);
}

/*
 * Vendor himax_hx83102d_reload_to_active(): poke the MCU into running
 * the image sitting in SRAM, retrying until the chip acknowledges.
 */
static int hx83102_reload_to_active(struct hx83102_data *ts)
{
	u32 ack;
	int retry;
	int ret;

	for (retry = 0; retry < HX83102_REG_RETRY_TIMES; retry++) {
		ret = hx83102_ahb_write_u32(ts, HX83102_REG_FW_RELOAD_ACTIVATE,
					    HX83102_FW_RELOAD_ACTIVATE_CMD);
		if (ret)
			return ret;

		usleep_range(1000, 1100);

		ret = hx83102_ahb_read_u32(ts, HX83102_REG_FW_RELOAD_ACTIVATE,
					   &ack);
		if (ret)
			return ret;

		if (ack == HX83102_FW_RELOAD_ACTIVATE_ACK)
			return 0;
	}

	return -ETIMEDOUT;
}

/*
 * Vendor himax_mcu_sense_off() / hx83102e_sense_off(): stop the touch
 * firmware, then enter safe mode with the SPI password so the SRAM
 * becomes writable. The chip is pin-reset between password attempts.
 */
static int hx83102_sense_off(struct hx83102_data *ts)
{
	u8 psw;
	u32 state;
	int try;
	int ret;

	/* Ask the firmware to stop (vendor first loop, run once). */
	ret = hx83102_ahb_write_u32(ts, HX83102_REG_FW_ISR,
				    HX83102_FW_ISR_STOP);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	for (try = 0; try < HX83102_SENSE_OFF_TRIES; try++) {
		psw = HX83102_PSW_ENTER_SAFE_LB;
		ret = hx83102_bus_write(ts, HX83102_REG_SPI_PSW_LB, &psw, 1);
		if (ret)
			return ret;

		psw = HX83102_PSW_ENTER_SAFE_UB;
		ret = hx83102_bus_write(ts, HX83102_REG_SPI_PSW_UB, &psw, 1);
		if (ret)
			return ret;

		ret = hx83102_ahb_read_u32(ts, HX83102_REG_CS_CENTRAL_STATE,
					   &state);
		if (ret)
			return ret;

		if ((u8)state == HX83102_CENTRAL_STATE_SAFE) {
			/*
			 * Clear the TCON reset so the display side keeps
			 * running while the touch MCU is in safe mode
			 * (vendor hx83102e_sense_off()).
			 */
			return hx83102_ahb_write_u32(ts,
						     HX83102_REG_TCON_RST, 0);
		}

		usleep_range(HX83102_SENSE_OFF_RETRY_DELAY_US,
			     HX83102_SENSE_OFF_RETRY_DELAY_US + 100);
		hx83102_pin_reset_slow(ts);
	}

	return -ETIMEDOUT;
}

/*
 * Vendor himax_mcu_sense_on() as the HX83102E overrides define it
 * (hx83102e_sense_on() with HX_RST_PIN_FUNC): bring the interface back,
 * clear the FW ISR flag, pin-reset out of safe mode and start the image.
 */
static int hx83102_sense_on(struct hx83102_data *ts)
{
	int ret;

	ret = hx83102_interface_on(ts);
	if (ret)
		return ret;

	ret = hx83102_ahb_write_u32(ts, HX83102_REG_FW_ISR, 0);
	if (ret)
		return ret;

	usleep_range(10000, 11000);

	hx83102_pin_reset(ts);

	return hx83102_reload_to_active(ts);
}

/*
 * Vendor himax_mcu_check_CRC(): run the hardware CRC engine over a SRAM
 * range and return its CRC32C.
 */
static int hx83102_hw_crc(struct hx83102_data *ts, u32 addr, size_t len,
			  u32 *crc)
{
	u32 beat;
	u8 status;
	int try;
	int ret;

	if (len % 4)
		return -EINVAL;

	ret = hx83102_ahb_write_u32(ts, HX83102_REG_CRC_ADDR_FROM, addr);
	if (ret)
		return ret;

	/*
	 * The beat register takes the word count in the low half word and
	 * the 0x99 prefix in byte 2 (vendor writes 00 99 len_hi len_lo).
	 */
	beat = HX83102_CRC_CMD_BEAT_PREFIX << 16 | (len / 4);
	ret = hx83102_ahb_write_u32(ts, HX83102_REG_CRC_CMD_BEAT, beat);
	if (ret)
		return ret;

	for (try = 0; try < HX83102_CRC_POLL_TRIES; try++) {
		ret = hx83102_ahb_read(ts, HX83102_REG_CRC_STATUS, &status, 1);
		if (ret)
			return ret;

		if (!(status & HX83102_CRC_STATUS_BUSY))
			return hx83102_ahb_read_u32(ts, HX83102_REG_CRC_RESULT,
						    crc);

		usleep_range(HX83102_CRC_POLL_US, HX83102_CRC_POLL_US + 100);
	}

	return -ETIMEDOUT;
}

/*
 * Software counterpart of the hardware CRC, computed exactly the vendor
 * way in himax_mcu_Calculate_CRC_with_AP(): the register starts at
 * 0xFFFFFFFF, is XORed with each 32-bit word of the data (assembled
 * little endian, byte 0 in the lowest bits) and then shifted right 32
 * times, XORing the polynomial whenever the lowest bit is set. No final
 * XOR.
 */
VISIBLE_IF_KUNIT u32 hx83102_crc32c(const u8 *data, size_t len)
{
	u32 crc = 0xFFFFFFFF;
	size_t i;

	for (i = 0; i + 4 <= len; i += 4) {
		u32 word = get_unaligned_le32(&data[i]);
		int bit;

		crc ^= word;
		for (bit = 0; bit < 32; bit++) {
			if (crc & BIT(0))
				crc = (crc >> 1) ^ HX83102_CRC32C_POLY;
			else
				crc >>= 1;
		}
	}

	return crc;
}
EXPORT_SYMBOL_IF_KUNIT(hx83102_crc32c);

/*
 * Vendor himax_mcu_write_sram_0f(): copy a firmware block into SRAM
 * through the AHB write window, splitting it into bounded chunks.
 */
static int hx83102_sram_write(struct hx83102_data *ts, u32 addr,
			      const u8 *data, size_t len)
{
	size_t written = 0;

	while (written < len) {
		size_t chunk = min(len - written,
				   (size_t)HX83102_ZF_WRITE_CHUNK);
		int ret;

		ret = hx83102_ahb_write(ts, addr + written, data + written,
					chunk);
		if (ret)
			return ret;

		written += chunk;
		usleep_range(HX83102_ZF_CHUNK_GAP_US,
			     HX83102_ZF_CHUNK_GAP_US + 10);
	}

	return 0;
}

/*
 * Vendor himax_sram_write_crc_check(): upload a firmware block and
 * verify it with the hardware CRC engine, which returns zero for a
 * correct image, retrying a bounded number of times.
 */
static int hx83102_upload_block(struct hx83102_data *ts, u32 sram_addr,
				const u8 *data, u32 size)
{
	u32 crc;
	int try;
	int ret;

	for (try = 0; try < HX83102_ZF_CRC_RETRIES; try++) {
		ret = hx83102_sram_write(ts, sram_addr, data, size);
		if (ret)
			return ret;

		ret = hx83102_hw_crc(ts, sram_addr, size, &crc);
		if (ret)
			return ret;

		if (crc == 0)
			return 0;
	}

	return -EIO;
}

/*
 * Vendor himax_zf_part_info(), config part: write the assembled
 * configuration block and require the hardware CRC over the whole block
 * to equal the software CRC of the same bytes.
 */
static int hx83102_upload_config(struct hx83102_data *ts,
				 const struct hx83102_fw_layout *layout)
{
	u32 crc = 0;
	int try;
	int ret;

	for (try = 0; try < HX83102_ZF_CRC_RETRIES; try++) {
		ret = hx83102_sram_write(ts, layout->cfg_sram_addr,
					 layout->cfg_image,
					 layout->cfg_size);
		if (ret)
			return ret;

		ret = hx83102_hw_crc(ts, layout->cfg_sram_addr,
				     layout->cfg_size, &crc);
		if (ret)
			return ret;

		if (crc == layout->cfg_crc)
			return 0;
	}

	return -EIO;
}

/* Vendor hx83102ab_power_on_init(): poll until the FW finished reloading. */
static int hx83102_wait_fw_reload_done(struct hx83102_data *ts)
{
	u32 done;
	int try;
	int ret;

	for (try = 0; try < HX83102_FW_2ND_RELOAD_POLLS; try++) {
		ret = hx83102_ahb_read_u32(ts, HX83102_REG_FW_2ND_RELOAD_DONE,
					   &done);
		if (ret)
			return ret;

		if (done == HX83102_FW_2ND_RELOAD_DONE_DATA)
			return 0;

		usleep_range(HX83102_FW_2ND_RELOAD_POLL_US,
			     HX83102_FW_2ND_RELOAD_POLL_US + 1000);
	}

	return -ETIMEDOUT;
}

/*
 * Vendor himax_zf_part_info(), overlay part (HX_CODE_OVERLAY): after
 * the main upload, bring the firmware up and let it ask for its code
 * overlay through a handshaking register; upload the overlay, reply and
 * wait until the firmware signals that it finished reloading.
 */
static int hx83102_overlay_handshake(struct hx83102_data *ts,
				     const struct hx83102_fw_layout *layout)
{
	u32 handshake;
	int try;
	int ret;

	/* Clear the handshaking register and confirm the write. */
	for (try = 0; try < HX83102_REG_RETRY_TIMES; try++) {
		ret = hx83102_ahb_write_u32(ts, HX83102_REG_HANDSHAKE,
					    HX83102_HANDSHAKE_CLEAR_DATA);
		if (ret)
			return ret;

		usleep_range(1000, 1100);

		ret = hx83102_ahb_read_u32(ts, HX83102_REG_HANDSHAKE,
					   &handshake);
		if (ret)
			return ret;

		if (handshake == HX83102_HANDSHAKE_CLEAR_DATA)
			break;
	}

	if (try == HX83102_REG_RETRY_TIMES)
		return -ETIMEDOUT;

	/*
	 * Flash-reload control: the zero-flash override of the vendor
	 * reload_disable() (hx_dis_rload_0f()) ignores its argument and
	 * always writes the disable value, as there is no flash to reload
	 * from.
	 */
	ret = hx83102_ahb_write_u32(ts, HX83102_REG_FLASH_RELOAD,
				    HX83102_FLASH_RELOAD_DIS_DATA);
	if (ret)
		return ret;

	ret = hx83102_ahb_write_u32(ts, HX83102_REG_RAW_OUT_SEL, 0);
	if (ret)
		return ret;

	ret = hx83102_ahb_write_u32(ts, HX83102_REG_SORTING_MODE_EN, 0);
	if (ret)
		return ret;

	ret = hx83102_ahb_write_u32(ts, HX83102_REG_FW_2ND_RELOAD_DONE, 0);
	if (ret)
		return ret;

	/* Start the firmware and wait for its overlay request. */
	ret = hx83102_sense_on(ts);
	if (ret)
		return ret;

	for (try = 0; try < HX83102_OVL_REQUEST_TRIES; try++) {
		ret = hx83102_ahb_read_u32(ts, HX83102_REG_HANDSHAKE,
					   &handshake);
		if (ret)
			return ret;

		if (handshake == HX83102_HANDSHAKE_OVL_REQUEST)
			break;

		usleep_range(1000, 1100);
	}

	if (try == HX83102_OVL_REQUEST_TRIES)
		return -ETIMEDOUT;

	/* Upload the overlay and CRC-verify it like the code block. */
	ret = hx83102_upload_block(ts, layout->overlay_sram_addr,
				   ts->fw->data + layout->overlay_fw_off,
				   layout->overlay_size);
	if (ret)
		return ret;

	/* Reply and wait for the firmware to accept it. */
	for (try = 0; try < HX83102_REG_RETRY_TIMES; try++) {
		ret = hx83102_ahb_write_u32(ts, HX83102_REG_HANDSHAKE,
					    HX83102_HANDSHAKE_OVL_REPLY);
		if (ret)
			return ret;

		usleep_range(1000, 1100);

		ret = hx83102_ahb_read_u32(ts, HX83102_REG_HANDSHAKE,
					   &handshake);
		if (ret)
			return ret;

		if (handshake == HX83102_HANDSHAKE_OVL_REPLY)
			break;
	}

	if (try == HX83102_REG_RETRY_TIMES)
		return -ETIMEDOUT;

	return hx83102_wait_fw_reload_done(ts);
}

/*
 * Vendor himax_mcu_firmware_update_0f(): reset the MCU, halt it in safe
 * mode, upload and CRC-verify the code partition and the configuration
 * block, then run the overlay handshake when the image carries one.
 * The whole sequence is attempted twice (vendor count < 2).
 */
static int hx83102_fw_upload(struct hx83102_data *ts)
{
	const struct hx83102_fw_layout *layout = &ts->layout;
	int attempt;
	int ret;

	for (attempt = 0; attempt < HX83102_ZF_UPDATE_RETRIES; attempt++) {
		/*
		 * Vendor himax_mcu_firmware_update_0f(): system reset
		 * first on every attempt, then safe mode.
		 */
		ret = hx83102_ahb_write_u32(ts, HX83102_REG_SYSTEM_RESET,
					    HX83102_SYSTEM_RESET_DATA);
		if (ret)
			return ret;

		ret = hx83102_sense_off(ts);
		if (ret)
			continue;

		ret = hx83102_upload_block(ts, layout->code_sram_addr,
					   ts->fw->data + layout->code_fw_off,
					   layout->code_size);
		if (ret)
			continue;

		if (layout->cfg_size) {
			ret = hx83102_upload_config(ts, layout);
			if (ret)
				continue;
		}

		if (layout->has_overlay) {
			ret = hx83102_overlay_handshake(ts, layout);
		} else {
			/*
			 * No overlay: vendor power_on_init() clears the
			 * init registers and brings the firmware up.
			 */
			ret = hx83102_ahb_write_u32(ts,
						    HX83102_REG_FLASH_RELOAD,
						    HX83102_FLASH_RELOAD_DIS_DATA);
			if (!ret)
				ret = hx83102_ahb_write_u32(ts,
							    HX83102_REG_RAW_OUT_SEL,
							    0);
			if (!ret)
				ret = hx83102_ahb_write_u32(ts,
							    HX83102_REG_SORTING_MODE_EN,
							    0);
			if (!ret)
				ret = hx83102_ahb_write_u32(ts,
							    HX83102_REG_FW_2ND_RELOAD_DONE,
							    0);
			if (!ret)
				ret = hx83102_sense_on(ts);
			if (!ret)
				ret = hx83102_wait_fw_reload_done(ts);
		}

		if (!ret)
			return 0;
	}

	dev_err(ts->dev, "firmware upload failed: %d\n", ret);
	return ret ?: -EIO;
}

/* ---------------------------------------------------------------------------
 * Firmware layout parsing
 * ---------------------------------------------------------------------------
 */

static bool hx83102_part_is_overlay(const u8 *part)
{
	return part[HX83102_ZF_PART_MARK_OFF] ==
	       HX83102_ZF_PART_MARK_OVERLAY_0 &&
	       part[HX83102_ZF_PART_MARK_OFF + 1] ==
	       HX83102_ZF_PART_MARK_OVERLAY_1;
}

/*
 * Vendor himax_zf_part_info(): parse the zero-flash partition table.
 * Partition 0 is the firmware code. The remaining partitions are
 * configurations living in one contiguous 32 KiB DSRAM window: they are
 * gathered into a single image (destinations rounded down to 4 bytes as
 * the vendor does) whose span is rounded up to 16 bytes, uploaded in one
 * piece and verified by one hardware CRC run against a software CRC of
 * the same bytes. A partition marked 0x88 0x77 is a code overlay that
 * the firmware requests at run time (see hx83102_overlay_handshake()).
 */
VISIBLE_IF_KUNIT int hx83102_parse_fw(const struct firmware *fw,
				      struct hx83102_fw_layout *layout)
{
	const u8 *table = fw->data + HX83102_ZF_TABLE_OFFSET;
	u32 dsram_base = U32_MAX;
	u32 dsram_end = 0;
	int num_parts;
	int i;

	memset(layout, 0, sizeof(*layout));

	if (fw->size < HX83102_ZF_TABLE_OFFSET +
		       HX83102_ZF_TABLE_MAX_PARTS *
		       (HX83102_ZF_PART_MARK_OFF + 2))
		return -EINVAL;

	num_parts = table[HX83102_ZF_TABLE_COUNT_OFF];
	if (num_parts < 1 || num_parts > HX83102_ZF_TABLE_MAX_PARTS)
		return -EINVAL;

	/* Partition 0: the firmware code. */
	layout->code_sram_addr = get_unaligned_le32(table);
	layout->code_size = get_unaligned_le32(table +
					       HX83102_ZF_PART_SIZE_OFF);
	layout->code_fw_off = get_unaligned_le32(table +
						 HX83102_ZF_PART_FWADDR_OFF);

	if (!layout->code_size ||
	    layout->code_fw_off > fw->size ||
	    layout->code_size > fw->size - layout->code_fw_off)
		return -EINVAL;

	for (i = 1; i < num_parts; i++) {
		const u8 *part = table + i * 16;
		u32 sram_addr = get_unaligned_le32(part);
		u32 size = get_unaligned_le32(part +
					      HX83102_ZF_PART_SIZE_OFF);
		u32 fw_off = get_unaligned_le32(part +
						HX83102_ZF_PART_FWADDR_OFF);

		if (hx83102_part_is_overlay(part)) {
			if (!layout->has_overlay) {
				layout->overlay_sram_addr = sram_addr;
				layout->overlay_size = size;
				layout->overlay_fw_off = fw_off;
				layout->has_overlay = true;
			}
			continue;
		}

		if (!size)
			continue;

		if (fw_off > fw->size || size > fw->size - fw_off)
			return -EINVAL;

		/* Vendor rounds configuration destinations down to 4. */
		sram_addr = round_down(sram_addr, HX83102_ZF_CFG_ADDR_ALIGN);
		dsram_base = min(dsram_base, sram_addr);
		dsram_end = max(dsram_end, sram_addr + size);
	}

	if (layout->has_overlay &&
	    (layout->overlay_fw_off > fw->size ||
	     layout->overlay_size > fw->size - layout->overlay_fw_off))
		return -EINVAL;

	if (dsram_end) {
		layout->cfg_sram_addr = dsram_base;
		layout->cfg_size = round_up(dsram_end - dsram_base,
					    HX83102_ZF_CFG_ALIGN);

		if (layout->cfg_size > HX83102_ZF_DSRAM_SIZE)
			return -EINVAL;

		layout->cfg_image = kmalloc(layout->cfg_size, GFP_KERNEL);
		if (!layout->cfg_image)
			return -ENOMEM;

		memset(layout->cfg_image, 0, layout->cfg_size);

		for (i = 1; i < num_parts; i++) {
			const u8 *part = table + i * 16;
			u32 sram_addr = get_unaligned_le32(part);
			u32 size = get_unaligned_le32(part +
						      HX83102_ZF_PART_SIZE_OFF);
			u32 fw_off = get_unaligned_le32(part +
							HX83102_ZF_PART_FWADDR_OFF);

			if (hx83102_part_is_overlay(part) || !size)
				continue;

			sram_addr = round_down(sram_addr,
					       HX83102_ZF_CFG_ADDR_ALIGN);
			memcpy(layout->cfg_image + sram_addr - dsram_base,
			       fw->data + fw_off, size);
		}

		layout->cfg_crc = hx83102_crc32c(layout->cfg_image,
						 layout->cfg_size);
	}

	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(hx83102_parse_fw);

/* ---------------------------------------------------------------------------
 * Event decoding
 * ---------------------------------------------------------------------------
 */

/*
 * Vendor himax_checksum_cal() and himax_ts_event_check(): decode one
 * event packet. Frames whose byte sum is not a multiple of 0x100 (or
 * that are all zeroes) are dropped unless they carry an exception
 * pattern, in which case the caller must recover the chip. Frames whose
 * raw-data selector nibble is not the host's are flushed from the event
 * stack. Slot activity is decided purely by coordinate range, exactly
 * like the vendor parser.
 */
VISIBLE_IF_KUNIT int hx83102_parse_event(const u8 *buf, u32 max_x, u32 max_y,
					 struct hx83102_touch_event *event)
{
	const u8 *pen = buf + HX83102_EVENT_TOUCH_INFO_SIZE -
			HX83102_EVENT_PEN_SIZE;
	unsigned int sum = 0;
	unsigned int zero = 0;
	bool all_eb = true;
	bool all_ec = true;
	bool all_ee = true;
	bool all_ed = true;
	u8 rawsel;
	u8 count;
	int i;

	memset(event, 0, sizeof(*event));

	for (i = 0; i < HX83102_EVENT_TOUCH_INFO_SIZE; i++) {
		sum += buf[i];
		if (!buf[i])
			zero++;
	}

	if (sum % 0x100 || zero == HX83102_EVENT_TOUCH_INFO_SIZE) {
		/* Exception scan (himax_ts_event_check()). */
		for (i = 0; i < HX83102_EVENT_EXCP_SCAN_SIZE; i++) {
			all_eb &= buf[i] == HX83102_EXCP_EB;
			all_ec &= buf[i] == HX83102_EXCP_EC;
			all_ee &= buf[i] == HX83102_EXCP_EE;
			all_ed &= buf[i] == HX83102_EXCP_ED;
		}

		if (all_eb || all_ec || all_ee)
			return HX83102_EVENT_EXCP;
		if (all_ed)
			return HX83102_EVENT_RELOAD;

		return HX83102_EVENT_DROP;
	}

	rawsel = (buf[HX83102_EVENT_COUNT_OFF] & HX83102_EVENT_RAWSEL_MASK) >> 4;
	if (rawsel != HX83102_EVENT_RAWSEL_HOST &&
	    rawsel != HX83102_EVENT_RAWSEL_NONE)
		return HX83102_EVENT_FLUSH;

	/*
	 * A count byte of 0xFF means no finger at all: the vendor zeroes
	 * the point count first and himax_parse_report_points() skips the
	 * finger slots, releasing them. Compare the full byte, the mask
	 * would cut it to 0x0F.
	 */
	count = buf[HX83102_EVENT_COUNT_OFF] & HX83102_EVENT_COUNT_MASK;
	if (buf[HX83102_EVENT_COUNT_OFF] == HX83102_EVENT_COUNT_ALL_LEAVE)
		count = 0;

	/*
	 * The vendor's point count starts from the pen (in range adds
	 * one); a count byte of 0xFF counts as zero. With no point at
	 * all, himax_parse_report_points() returns before touching the
	 * finger slots: everything is released and the untouched slots
	 * (which read as coordinates at the origin, in range) are not
	 * reported.
	 */
	if (get_unaligned_be16(&pen[HX83102_PEN_X_OFF]) <= max_x &&
	    get_unaligned_be16(&pen[HX83102_PEN_Y_OFF]) <= max_y) {
		if (count) {
			/*
			 * Fingers present: decode them (range decides
			 * activity, an untouched slot reads as a touch at
			 * the origin).
			 */
			for (i = 0; i < HX83102_MAX_TOUCHES; i++) {
				int off = i * HX83102_EVENT_FINGER_SIZE;
				u16 x = get_unaligned_be16(&buf[off]);
				u16 y = get_unaligned_be16(&buf[off + 2]);
				u8 major = buf[HX83102_EVENT_FINGER_COORD_SIZE + i];

				if (x <= max_x && y <= max_y) {
					event->fingers[i].x = x;
					event->fingers[i].y = y;
					event->fingers[i].major = major;
					event->fingers[i].active = true;
				}
			}
		}

		event->pen.x = get_unaligned_be16(&pen[HX83102_PEN_X_OFF]);
		event->pen.y = get_unaligned_be16(&pen[HX83102_PEN_Y_OFF]);
		event->pen.pressure =
			get_unaligned_be16(pen + HX83102_PEN_PRESSURE_OFF);
		event->pen.tilt_x = (s8)pen[HX83102_PEN_TILT_X_OFF];
		event->pen.tilt_y = (s8)pen[HX83102_PEN_TILT_Y_OFF];
		event->pen.hover = pen[HX83102_PEN_HOVER_OFF];
		event->pen.btn1 = pen[HX83102_PEN_BTN1_OFF];
		event->pen.btn2 = pen[HX83102_PEN_BTN2_OFF];
		event->pen.active = true;

		return HX83102_EVENT_FRAME;
	}

	/* No pen: fingers only decode when the count is non-zero. */
	if (!count) {
		event->leave = true;
		return HX83102_EVENT_FRAME;
	}

	for (i = 0; i < HX83102_MAX_TOUCHES; i++) {
		u16 x = get_unaligned_be16(&buf[i * HX83102_EVENT_FINGER_SIZE]);
		u16 y = get_unaligned_be16(&buf[i * HX83102_EVENT_FINGER_SIZE + 2]);
		u8 major = buf[HX83102_EVENT_FINGER_COORD_SIZE + i];

		if (x <= max_x && y <= max_y) {
			event->fingers[i].x = x;
			event->fingers[i].y = y;
			event->fingers[i].major = major;
			event->fingers[i].active = true;
		}
	}

	return HX83102_EVENT_FRAME;
}
EXPORT_SYMBOL_IF_KUNIT(hx83102_parse_event);

static void hx83102_report_event(struct hx83102_data *ts,
				 struct hx83102_touch_event *event)
{
	struct input_dev *input = ts->input;
	bool finger_active = false;
	int i;

	for (i = 0; i < HX83102_MAX_TOUCHES; i++) {
		struct hx83102_finger_event *finger = &event->fingers[i];

		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER,
					   finger->active);
		if (finger->active) {
			touchscreen_report_pos(input, &ts->prop, finger->x,
					       finger->y, true);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR,
					 finger->major);
			finger_active = true;
		}
	}

	input_mt_sync_frame(input);
	input_report_key(input, BTN_TOUCH, finger_active);
	input_sync(input);

	if (ts->pen_input) {
		struct input_dev *pen = ts->pen_input;
		struct hx83102_pen_event *pev = &event->pen;
		bool pen_down = pev->active && !pev->hover && pev->pressure;

		input_report_key(pen, BTN_TOOL_PEN, pev->active);
		input_report_key(pen, BTN_TOOL_RUBBER, false);
		input_report_key(pen, BTN_TOUCH, pen_down);
		input_report_key(pen, BTN_STYLUS,
				 pev->active && pev->btn1);
		input_report_key(pen, BTN_STYLUS2,
				 pev->active && pev->btn2);

		if (pev->active) {
			touchscreen_report_pos(pen, &ts->prop, pev->x, pev->y,
					       true);
			input_report_abs(pen, ABS_PRESSURE,
					 pen_down ? pev->pressure : 0);
			input_report_abs(pen, ABS_TILT_X, pev->tilt_x);
			input_report_abs(pen, ABS_TILT_Y, pev->tilt_y);
		} else {
			/* Pen-up frames zero pressure and tilt (vendor
			 * himax_finger_report(), pen-up branch).
			 */
			input_report_abs(pen, ABS_PRESSURE, 0);
			input_report_abs(pen, ABS_TILT_X, 0);
			input_report_abs(pen, ABS_TILT_Y, 0);
		}

		input_sync(pen);
		ts->pen_down = pev->active;
	}
}

/* Release every finger and the pen (vendor himax_report_all_leave_event()). */
static void hx83102_report_release(struct hx83102_data *ts)
{
	struct input_dev *input = ts->input;
	int i;

	for (i = 0; i < HX83102_MAX_TOUCHES; i++) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
	}

	input_mt_sync_frame(input);
	input_report_key(input, BTN_TOUCH, false);
	input_sync(input);

	if (ts->pen_input && ts->pen_down) {
		struct input_dev *pen = ts->pen_input;

		input_report_key(pen, BTN_TOUCH, false);
		input_report_key(pen, BTN_STYLUS, false);
		input_report_key(pen, BTN_STYLUS2, false);
		input_report_abs(pen, ABS_PRESSURE, 0);
		input_report_abs(pen, ABS_TILT_X, 0);
		input_report_abs(pen, ABS_TILT_Y, 0);
		input_report_key(pen, BTN_TOOL_RUBBER, false);
		input_report_key(pen, BTN_TOOL_PEN, false);
		input_sync(pen);
		ts->pen_down = false;
	}
}

/* ---------------------------------------------------------------------------
 * Interrupt handling and recovery
 * ---------------------------------------------------------------------------
 */

/*
 * Vendor himax_checksum_cal(): a frame addressed to a different
 * raw-data consumer leaves the rest of the event stack unread; flush it
 * so the next interrupt starts on a frame boundary.
 */
static void hx83102_flush_event_stack(struct hx83102_data *ts)
{
	u8 buf[HX83102_EVENT_STACK_SIZE - HX83102_EVENT_TOUCH_INFO_SIZE];

	hx83102_bus_read(ts, HX83102_REG_EVENT_STACK, buf, sizeof(buf));
}

/* Vendor himax_excp_hw_reset(): re-upload the firmware, rate limited. */
static void hx83102_schedule_recovery(struct hx83102_data *ts)
{
	if (time_before(jiffies, ts->recovery_jiffies +
			msecs_to_jiffies(HX83102_EXCP_COOLDOWN_MS)))
		return;

	ts->recovery_jiffies = jiffies;
	schedule_work(&ts->recovery_work);
}

static irqreturn_t hx83102_irq(int irq, void *dev_id)
{
	struct hx83102_data *ts = dev_id;
	struct hx83102_touch_event event;
	int result;

	result = hx83102_bus_read(ts, HX83102_REG_EVENT_STACK, ts->event,
				  HX83102_EVENT_TOUCH_INFO_SIZE);
	if (result) {
		hx83102_schedule_recovery(ts);
		return IRQ_HANDLED;
	}

	result = hx83102_parse_event(ts->event, ts->width, ts->height, &event);
	switch (result) {
	case HX83102_EVENT_FRAME:
		if (event.leave)
			hx83102_report_release(ts);
		else
			hx83102_report_event(ts, &event);
		break;
	case HX83102_EVENT_FLUSH:
		hx83102_flush_event_stack(ts);
		break;
	case HX83102_EVENT_EXCP:
		dev_err_ratelimited(ts->dev,
				    "exception event, re-uploading firmware\n");
		hx83102_report_release(ts);
		hx83102_schedule_recovery(ts);
		break;
	case HX83102_EVENT_RELOAD:
		hx83102_reload_to_active(ts);
		break;
	case HX83102_EVENT_DROP:
	default:
		break;
	}

	return IRQ_HANDLED;
}

static void hx83102_recover(struct work_struct *work)
{
	struct hx83102_data *ts = container_of(work, struct hx83102_data,
					       recovery_work);
	int ret;

	guard(mutex)(&ts->fw_lock);

	if (!ts->started)
		return;

	disable_irq(ts->irq);

	ret = hx83102_fw_upload(ts);
	if (ret) {
		dev_err(ts->dev, "recovery failed: %d\n", ret);
	} else {
		ret = hx83102_ahb_write_u32(ts, HX83102_REG_SET_FRAME,
					    HX83102_SET_FRAME_DATA);
		if (ret)
			dev_err(ts->dev, "recovery tail failed: %d\n", ret);
	}

	enable_irq(ts->irq);
}

/* ---------------------------------------------------------------------------
 * Start / stop
 * ---------------------------------------------------------------------------
 */

static int hx83102_chip_verify(struct hx83102_data *ts)
{
	u8 id[4];
	int try;
	int ret;

	/*
	 * Vendor hx83102_chip_detect(): pin reset, safe mode, then read
	 * the ID register, retrying a few times.
	 */
	hx83102_pin_reset(ts);

	ret = hx83102_sense_off(ts);
	if (ret)
		return ret;

	for (try = 0; try < HX83102_CHIP_ID_TRIES; try++) {
		ret = hx83102_ahb_read(ts, HX83102_REG_IC_ID, id, sizeof(id));
		if (ret)
			return ret;

		/* Vendor hx83102_chip_detect(): ID in bytes 3, 2, 1. */
		if (id[3] == HX83102_CHIP_ID_0 &&
		    id[2] == HX83102_CHIP_ID_1 &&
		    id[1] == HX83102_CHIP_ID_83102E)
			return 0;
	}

	return dev_err_probe(ts->dev, -ENODEV,
			     "unexpected chip ID %02x %02x %02x\n",
			     id[3], id[2], id[1]);
}

static int hx83102_fw_cache(struct hx83102_data *ts)
{
	int ret;

	if (ts->fw)
		return 0;

	ret = request_firmware(&ts->fw, ts->fw_name, ts->dev);
	if (ret)
		return ret;

	ret = hx83102_parse_fw(ts->fw, &ts->layout);
	if (ret) {
		release_firmware(ts->fw);
		ts->fw = NULL;
	}

	return ret;
}

static int hx83102_start(struct hx83102_data *ts)
{
	int ret;

	guard(mutex)(&ts->fw_lock);

	if (ts->started)
		return 0;

	ret = hx83102_fw_cache(ts);
	if (ret)
		return ret;

	/* Vendor himax_gpio_power_config(): power-on reset toggle. */
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(HX83102_POWER_ON_PHASE_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(HX83102_POWER_ON_PHASE_MS);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(HX83102_POWER_ON_PHASE_MS);

	if (!ts->chip_verified) {
		ret = hx83102_chip_verify(ts);
		if (ret)
			return ret;

		ts->chip_verified = true;
	}

	ret = hx83102_fw_upload(ts);
	if (ret)
		return ret;

	/* Vendor tail of himax_mcu_firmware_update_0f(): N frame = 1. */
	ret = hx83102_ahb_write_u32(ts, HX83102_REG_SET_FRAME,
				    HX83102_SET_FRAME_DATA);
	if (ret)
		return ret;

	enable_irq(ts->irq);
	ts->started = true;
	dev_dbg(ts->dev, "firmware %s running\n", ts->fw_name);

	return 0;
}

static void hx83102_stop(struct hx83102_data *ts)
{
	guard(mutex)(&ts->fw_lock);

	if (!ts->started)
		return;

	disable_irq(ts->irq);
	hx83102_report_release(ts);
	/*
	 * The XRES line is shared with the display side of the TDDI: leave
	 * it deasserted instead of holding the chip in reset.
	 */
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	ts->started = false;
}

/* ---------------------------------------------------------------------------
 * Power management and panel follower
 * ---------------------------------------------------------------------------
 */

static void hx83102_panel_prepare_work(struct work_struct *work)
{
	struct hx83102_data *ts =
		container_of(work, struct hx83102_data,
			     panel_follower_prepare_work);
	int ret;

	ret = hx83102_start(ts);
	if (ret) {
		dev_err(ts->dev, "failed to start the touch controller: %d\n",
			ret);
		return;
	}

	WRITE_ONCE(ts->prepare_work_finished, true);
	/* Pairs with the read barrier in the suspend path. */
	smp_wmb();
}

static int hx83102_input_stop(struct hx83102_data *ts)
{
	guard(mutex)(&ts->input->mutex);

	if (input_device_enabled(ts->input))
		hx83102_stop(ts);

	return 0;
}

static int hx83102_suspend(struct device *dev)
{
	struct hx83102_data *ts = dev_get_drvdata(dev);

	if (drm_is_panel_follower(dev))
		return 0;

	cancel_work_sync(&ts->panel_follower_prepare_work);

	/* Pairs with the write barrier in the prepare work. */
	smp_rmb();
	if (!READ_ONCE(ts->prepare_work_finished))
		return 0;

	return hx83102_input_stop(ts);
}

static int hx83102_resume(struct device *dev)
{
	struct hx83102_data *ts = dev_get_drvdata(dev);

	if (drm_is_panel_follower(dev))
		return 0;

	/* Vendor HX_RESUME_SEND_CMD: the upload repeats on every resume. */
	WRITE_ONCE(ts->prepare_work_finished, false);
	schedule_work(&ts->panel_follower_prepare_work);

	return 0;
}

static int hx83102_panel_prepared(struct drm_panel_follower *follower)
{
	struct hx83102_data *ts = container_of(follower, struct hx83102_data,
					       panel_follower);

	WRITE_ONCE(ts->prepare_work_finished, false);
	schedule_work(&ts->panel_follower_prepare_work);

	return 0;
}

static int hx83102_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct hx83102_data *ts = container_of(follower, struct hx83102_data,
					       panel_follower);

	cancel_work_sync(&ts->panel_follower_prepare_work);

	/* Pairs with the write barrier in the prepare work. */
	smp_rmb();
	if (!READ_ONCE(ts->prepare_work_finished))
		return 0;

	return hx83102_input_stop(ts);
}

static const struct drm_panel_follower_funcs hx83102_panel_follower_funcs = {
	.panel_prepared = hx83102_panel_prepared,
	.panel_unpreparing = hx83102_panel_unpreparing,
};

EXPORT_GPL_SIMPLE_DEV_PM_OPS(hx83102_pm_ops, hx83102_suspend,
			     hx83102_resume);

/* ---------------------------------------------------------------------------
 * Probe and remove
 * ---------------------------------------------------------------------------
 */

static int hx83102_setup_input_devices(struct hx83102_data *ts)
{
	struct input_dev *input;
	struct input_dev *pen;
	int ret;

	input = devm_input_allocate_device(ts->dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = HX83102_NAME;
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x1AF0;
	input->id.product = 0x3102;
	input->id.version = 0x0100;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->width, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->height, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	/* The DT properties may have overridden the resolution. */
	ts->width = ts->prop.max_x;
	ts->height = ts->prop.max_y;
	input_abs_set_max(input, ABS_MT_POSITION_X, ts->width);
	input_abs_set_max(input, ABS_MT_POSITION_Y, ts->height);

	ret = input_mt_init_slots(input, HX83102_MAX_TOUCHES,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	input_set_drvdata(input, ts);

	pen = devm_input_allocate_device(ts->dev);
	if (!pen)
		return -ENOMEM;

	ts->pen_input = pen;
	pen->name = "himax-hx83102-pen";
	pen->id.bustype = BUS_SPI;
	pen->id.vendor = 0x1AF0;
	pen->id.product = 0x3102;
	pen->id.version = 0x0100;

	input_set_abs_params(pen, ABS_X, 0, ts->width, 0, 0);
	input_set_abs_params(pen, ABS_Y, 0, ts->height, 0, 0);
	input_set_abs_params(pen, ABS_PRESSURE, 0, HX83102_PEN_PRESSURE_MAX,
			     0, 0);
	input_set_abs_params(pen, ABS_TILT_X, -HX83102_PEN_TILT_RANGE,
			     HX83102_PEN_TILT_RANGE, 0, 0);
	input_set_abs_params(pen, ABS_TILT_Y, -HX83102_PEN_TILT_RANGE,
			     HX83102_PEN_TILT_RANGE, 0, 0);
	input_set_capability(pen, EV_ABS, ABS_DISTANCE);
	input_set_capability(pen, EV_KEY, BTN_TOUCH);
	input_set_capability(pen, EV_KEY, BTN_TOOL_PEN);
	input_set_capability(pen, EV_KEY, BTN_TOOL_RUBBER);
	input_set_capability(pen, EV_KEY, BTN_STYLUS);
	input_set_capability(pen, EV_KEY, BTN_STYLUS2);
	__set_bit(INPUT_PROP_DIRECT, pen->propbit);
	input_set_drvdata(pen, ts);

	return 0;
}

int hx83102_probe(struct device *dev, int irq,
		  const struct hx83102_transport_ops *ops)
{
	struct hx83102_data *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->dev = dev;
	ts->ops = ops;
	ts->irq = irq;
	dev_set_drvdata(dev, ts);

	mutex_init(&ts->fw_lock);
	INIT_WORK(&ts->recovery_work, hx83102_recover);
	INIT_WORK(&ts->panel_follower_prepare_work,
		  hx83102_panel_prepare_work);

	ret = device_property_read_string(dev, "firmware-name", &ts->fw_name);
	if (ret)
		ts->fw_name = HX83102_FW_NAME_DEFAULT;

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to request reset GPIO\n");

	/* Placeholder resolution until DT properties are parsed. */
	ts->width = HX83102_PEN_PRESSURE_MAX;
	ts->height = HX83102_PEN_PRESSURE_MAX;

	ret = hx83102_setup_input_devices(ts);
	if (ret)
		return ret;

	if (!ts->width || !ts->height)
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen-size-x/y not set\n");

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register input device\n");

	ret = input_register_device(ts->pen_input);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register pen input device\n");

	ret = devm_request_threaded_irq(dev, irq, NULL, hx83102_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN |
					IRQF_TRIGGER_FALLING,
					HX83102_NAME, ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	if (drm_is_panel_follower(dev)) {
		ts->panel_follower.funcs = &hx83102_panel_follower_funcs;
		ret = devm_drm_panel_add_follower(dev, &ts->panel_follower);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to add panel follower\n");
	} else {
		ret = hx83102_start(ts);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to start controller\n");
		WRITE_ONCE(ts->prepare_work_finished, true);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(hx83102_probe);

void hx83102_remove(struct device *dev)
{
	struct hx83102_data *ts = dev_get_drvdata(dev);

	hx83102_stop(ts);
	cancel_work_sync(&ts->recovery_work);
	cancel_work_sync(&ts->panel_follower_prepare_work);

	kfree(ts->layout.cfg_image);
	release_firmware(ts->fw);
	ts->fw = NULL;
}
EXPORT_SYMBOL_GPL(hx83102_remove);

MODULE_DESCRIPTION("Himax HX83102 TDDI touchscreen and pen controller driver");
MODULE_LICENSE("GPL");
