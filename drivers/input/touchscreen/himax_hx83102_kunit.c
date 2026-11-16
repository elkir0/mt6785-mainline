// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the Himax HX83102 event and firmware layout parsers
 *
 * Copyright (C) 2026 The Lenovomarchy authors
 *
 * The test packets are built the way the vendor driver decodes them in
 * himax_parse_report_points() and himax_checksum_cal() in
 * himax_common.c: ten fingers of 4 big-endian bytes, a 12-byte touch
 * major area, the finger count byte, three state bytes and the 12-byte
 * pen block, with the byte sum of the whole packet a multiple of 0x100.
 */

#include <kunit/test.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "himax_hx83102.h"

MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");

/* Packet size the parser consumes (touch information part). */
#define PACKET_SIZE		HX83102_EVENT_TOUCH_INFO_SIZE
/* Pen block position inside a packet. */
#define PEN_OFF			(PACKET_SIZE - HX83102_EVENT_PEN_SIZE)
/* Finger count byte position. */
#define COUNT_OFF		HX83102_EVENT_COUNT_OFF
/* First state byte, used as the checksum slack. */
#define STATE_OFF		(COUNT_OFF + 1)

/* Test panel resolution (the unit's 1200x2000 panel). */
#define TEST_MAX_X		1199
#define TEST_MAX_Y		1999

/*
 * Build a valid packet: fix up the byte sum (mod 0x100) through the
 * first state byte, which the vendor driver ignores.
 */
static void fix_checksum(u8 *buf)
{
	unsigned int sum = 0;
	int i;

	for (i = 0; i < PACKET_SIZE; i++)
		sum += buf[i];

	buf[STATE_OFF] += (0x100 - (sum % 0x100)) & 0xFF;
}

static void set_finger(u8 *buf, int slot, u16 x, u16 y)
{
	put_unaligned_be16(x, &buf[slot * HX83102_EVENT_FINGER_SIZE]);
	put_unaligned_be16(y,
			   &buf[slot * HX83102_EVENT_FINGER_SIZE + 2]);
}

static void set_pen(u8 *buf, u16 x, u16 y, u16 pressure)
{
	put_unaligned_be16(x, &buf[PEN_OFF + HX83102_PEN_X_OFF]);
	put_unaligned_be16(y, &buf[PEN_OFF + HX83102_PEN_Y_OFF]);
	put_unaligned_be16(pressure,
			   &buf[PEN_OFF + HX83102_PEN_PRESSURE_OFF]);
}

static void crc32c_test_known(struct kunit *test)
{
	u8 buf[16];
	u32 crc;
	int i;

	/* Reference values from the vendor bit-level algorithm
	 * (himax_mcu_Calculate_CRC_with_AP()).
	 */
	for (i = 0; i < 16; i++)
		buf[i] = i;
	crc = hx83102_crc32c(buf, sizeof(buf));
	KUNIT_EXPECT_EQ(test, crc, 0x2636f714);

	memset(buf, 0xEB, sizeof(buf));
	crc = hx83102_crc32c(buf, sizeof(buf));
	KUNIT_EXPECT_EQ(test, crc, 0x71a775cf);
}

static void crc32c_test_all_zero(struct kunit *test)
{
	u8 buf[16] = {};

	/* Vendor algorithm over 16 zero bytes. */
	KUNIT_EXPECT_EQ(test, hx83102_crc32c(buf, sizeof(buf)),
			0xbd8f6515);
}

static void parse_event_finger_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/* Finger 0 at (100, 200), major 33; finger 3 at (0, 0), major 1. */
	set_finger(buf, 0, 100, 200);
	set_finger(buf, 3, 0, 0);
	buf[HX83102_EVENT_FINGER_COORD_SIZE] = 33;
	buf[HX83102_EVENT_FINGER_COORD_SIZE + 3] = 1;
	/* Untouched pen block reads as (0, 0): push it out of range. */
	set_pen(buf, 0xFFFF, 0xFFFF, 0);
	buf[COUNT_OFF] = 0xF2;	/* 2 fingers, raw selector 0x0F (host) */
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FRAME);

	KUNIT_EXPECT_TRUE(test, event.fingers[0].active);
	KUNIT_EXPECT_EQ(test, event.fingers[0].x, 100);
	KUNIT_EXPECT_EQ(test, event.fingers[0].y, 200);
	KUNIT_EXPECT_EQ(test, event.fingers[0].major, 33);

	KUNIT_EXPECT_TRUE(test, event.fingers[3].active);
	KUNIT_EXPECT_EQ(test, event.fingers[3].x, 0);
	KUNIT_EXPECT_EQ(test, event.fingers[3].y, 0);
	KUNIT_EXPECT_EQ(test, event.fingers[3].major, 1);

	/* An untouched slot reads as coordinates (0, 0), which the vendor
	 * parser (and this one) accepts as a touch at the origin.
	 */
	KUNIT_EXPECT_TRUE(test, event.fingers[1].active);
	KUNIT_EXPECT_FALSE(test, event.pen.active);
	KUNIT_EXPECT_FALSE(test, event.leave);
}

static void parse_event_out_of_range_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/* Finger 0 beyond max_x: inactive; finger 2 valid. */
	set_finger(buf, 0, 1200, 10);
	set_finger(buf, 2, 5, 6);
	buf[HX83102_EVENT_FINGER_COORD_SIZE + 2] = 20;
	set_pen(buf, 0xFFFF, 0xFFFF, 0);
	buf[COUNT_OFF] = 0xF2;
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FRAME);

	KUNIT_EXPECT_FALSE(test, event.fingers[0].active);
	KUNIT_EXPECT_TRUE(test, event.fingers[2].active);
	KUNIT_EXPECT_EQ(test, event.fingers[2].x, 5);
	KUNIT_EXPECT_EQ(test, event.fingers[2].y, 6);
	KUNIT_EXPECT_FALSE(test, event.leave);
}

static void parse_event_pen_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/* Pen in range and down at (600, 700), pressure 2048,
	 * tilt (-20, 15), button 1 held, button 2 up.
	 */
	set_pen(buf, 600, 700, 2048);
	buf[PEN_OFF + HX83102_PEN_TILT_X_OFF] = (u8)(s8)-20;
	buf[PEN_OFF + HX83102_PEN_TILT_Y_OFF] = 15;
	buf[PEN_OFF + HX83102_PEN_HOVER_OFF] = 0;
	buf[PEN_OFF + HX83102_PEN_BTN1_OFF] = 1;
	buf[PEN_OFF + HX83102_PEN_BTN2_OFF] = 0;
	buf[COUNT_OFF] = 0xF0;
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FRAME);

	KUNIT_EXPECT_TRUE(test, event.pen.active);
	KUNIT_EXPECT_EQ(test, event.pen.x, 600);
	KUNIT_EXPECT_EQ(test, event.pen.y, 700);
	KUNIT_EXPECT_EQ(test, event.pen.pressure, 2048);
	KUNIT_EXPECT_EQ(test, event.pen.tilt_x, -20);
	KUNIT_EXPECT_EQ(test, event.pen.tilt_y, 15);
	KUNIT_EXPECT_EQ(test, event.pen.hover, 0);
	KUNIT_EXPECT_EQ(test, event.pen.btn1, 1);
	KUNIT_EXPECT_EQ(test, event.pen.btn2, 0);
	KUNIT_EXPECT_FALSE(test, event.leave);
	/* Count 0xF0: no finger slots decoded. */
	KUNIT_EXPECT_FALSE(test, event.fingers[0].active);
}

static void parse_event_pen_out_of_range_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/* Pen coordinates out of range but a finger down: the pen block
	 * must be ignored while the finger reports normally.
	 */
	set_pen(buf, 4000, 4000, 100);
	set_finger(buf, 1, 42, 43);
	buf[COUNT_OFF] = 0xF1;
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FRAME);

	KUNIT_EXPECT_FALSE(test, event.pen.active);
	KUNIT_EXPECT_TRUE(test, event.fingers[1].active);
	KUNIT_EXPECT_FALSE(test, event.leave);
}

static void parse_event_all_leave_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/*
	 * All fingers up: count byte 0xFF (himax_parse_report_data()).
	 * The pen block must be out of range, or the vendor flow would
	 * report a pen sitting at the origin.
	 */
	buf[COUNT_OFF] = 0xFF;
	set_pen(buf, 0xFFFF, 0xFFFF, 0);
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FRAME);
	KUNIT_EXPECT_TRUE(test, event.leave);
	KUNIT_EXPECT_FALSE(test, event.pen.active);
	/* The untouched slots are not decoded on a leave frame. */
	KUNIT_EXPECT_FALSE(test, event.fingers[0].active);
}

static void parse_event_count_zero_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/*
	 * Count zero with nothing in range also means all contacts left.
	 * The vendor accepts raw-data selector 0 like 0x0F. The pen and
	 * the untouched finger slots must read as out of range: put the
	 * pen at an impossible coordinate (a zeroed pen block would sit
	 * at the origin, in range).
	 */
	buf[COUNT_OFF] = 0x00;
	buf[STATE_OFF + 2] = 0x01;
	set_pen(buf, 0xFFFF, 0xFFFF, 0);
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FRAME);
	KUNIT_EXPECT_TRUE(test, event.leave);
	KUNIT_EXPECT_FALSE(test, event.pen.active);
}

static void parse_event_bad_checksum_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	set_finger(buf, 0, 100, 100);
	buf[COUNT_OFF] = 0xF1;
	/* Deliberately no fix_checksum(): sum not a multiple of 0x100. */

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_DROP);
}

static void parse_event_exception_eb_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/* Vendor himax_ts_event_check(): the finger area all 0xEB. */
	memset(buf, HX83102_EXCP_EB, HX83102_EVENT_EXCP_SCAN_SIZE);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_EXCP);
}

static void parse_event_exception_ec_ee_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	memset(buf, HX83102_EXCP_EC, HX83102_EVENT_EXCP_SCAN_SIZE);
	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_EXCP);

	memset(buf, HX83102_EXCP_EE, HX83102_EVENT_EXCP_SCAN_SIZE);
	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_EXCP);
}

static void parse_event_reload_ed_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	memset(buf, HX83102_EXCP_ED, HX83102_EVENT_EXCP_SCAN_SIZE);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_RELOAD);
}

static void parse_event_flush_test(struct kunit *test)
{
	struct hx83102_touch_event event;
	u8 buf[PACKET_SIZE] = {};

	/* Raw selector nibble not 0x0F: the frame belongs to a different
	 * raw-data consumer and the host must flush the rest of the stack.
	 */
	buf[COUNT_OFF] = 0x21;
	fix_checksum(buf);

	KUNIT_EXPECT_EQ(test,
			hx83102_parse_event(buf, TEST_MAX_X, TEST_MAX_Y,
					    &event),
			HX83102_EVENT_FLUSH);
}

/*
 * Build a synthetic zero-flash firmware image: a partition table at
 * HX83102_ZF_TABLE_OFFSET with @num_parts entries and, for every
 * partition, a payload at file offset 0x11000 + running offset. Every
 * payload byte carries its partition index so tests can tell them
 * apart.
 */
static struct firmware *build_test_fw(struct kunit *test, int num_parts,
				      const u32 *sram_addrs, const u32 *sizes)
{
	struct firmware *fw;
	size_t img_size = HX83102_ZF_TABLE_OFFSET +
			  HX83102_ZF_TABLE_MAX_PARTS * 16;
	size_t off = 0x11000;
	u8 *table;
	int i;

	for (i = 0; i < num_parts; i++)
		img_size = max(img_size, off + (size_t)i * 0x1000 + sizes[i]);

	fw = kunit_kzalloc(test, sizeof(*fw), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fw);
	fw->data = kunit_kzalloc(test, img_size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, fw->data);
	fw->size = img_size;

	table = (u8 *)fw->data + HX83102_ZF_TABLE_OFFSET;

	for (i = 0; i < num_parts; i++) {
		u8 *part = table + i * 16;
		u32 fw_off = off + (u32)(i * 0x1000);
		u32 j;

		/* SRAM address little endian (AHB write address). */
		part[0] = sram_addrs[i];
		part[1] = sram_addrs[i] >> 8;
		part[2] = sram_addrs[i] >> 16;
		part[3] = sram_addrs[i] >> 24;
		/* Size and file offset big endian. */
		put_unaligned_be32(sizes[i], part + HX83102_ZF_PART_SIZE_OFF);
		put_unaligned_be32(fw_off, part + HX83102_ZF_PART_FWADDR_OFF);
		/* Marker pair zero: regular partition. */

		for (j = 0; j < sizes[i]; j++)
			((u8 *)fw->data)[fw_off + j] = 0xA0 | i;
	}

	table[HX83102_ZF_TABLE_COUNT_OFF] = num_parts;

	return fw;
}

static void parse_fw_layout_test(struct kunit *test)
{
	const u32 sram_addrs[] = { 0x20000000, 0x10007000, 0x10007100 };
	const u32 sizes[] = { 0x1000, 0x100, 0x80 };
	struct hx83102_fw_layout layout;
	struct firmware *fw;
	u8 cfg[384];
	u32 expect_crc;
	int ret;

	fw = build_test_fw(test, 3, sram_addrs, sizes);

	ret = hx83102_parse_fw(fw, &layout);
	KUNIT_EXPECT_EQ(test, ret, 0);

	/* Code partition (index 0). */
	KUNIT_EXPECT_EQ(test, layout.code_sram_addr, 0x20000000);
	KUNIT_EXPECT_EQ(test, layout.code_size, 0x1000);
	KUNIT_EXPECT_EQ(test, layout.code_fw_off, 0x11000);

	/* Config block: partitions 1 and 2 gathered, rounded to 16 bytes. */
	KUNIT_EXPECT_EQ(test, layout.cfg_sram_addr, 0x10007000);
	KUNIT_EXPECT_EQ(test, layout.cfg_size, 0x180);
	KUNIT_EXPECT_FALSE(test, layout.has_overlay);

	memset(cfg, 0, sizeof(cfg));
	memcpy(cfg, fw->data + 0x11000 + 0x1000, 0x100);
	memcpy(cfg + 0x100, fw->data + 0x11000 + 0x2000, 0x80);
	expect_crc = hx83102_crc32c(cfg, sizeof(cfg));

	KUNIT_EXPECT_EQ(test, layout.cfg_crc, expect_crc);
	KUNIT_EXPECT_MEMEQ(test, layout.cfg_image, cfg, sizeof(cfg));
}

static void parse_fw_align_test(struct kunit *test)
{
	/* The second config partition starts at an unaligned address:
	 * the vendor rounds destinations down to 4 bytes.
	 */
	const u32 sram_addrs[] = { 0x20000000, 0x10007000, 0x10007003 };
	const u32 sizes[] = { 0x1000, 0x100, 0x40 };
	struct hx83102_fw_layout layout;
	struct firmware *fw;

	fw = build_test_fw(test, 3, sram_addrs, sizes);
	KUNIT_ASSERT_EQ(test, hx83102_parse_fw(fw, &layout), 0);

	/*
	 * 0x10007003 rounds down to 0x10007000, colliding with the first
	 * partition: its 0x40 bytes overwrite the start of the block, and
	 * the span stays 0x100 (the first partition still reaches
	 * 0x10007100).
	 */
	KUNIT_EXPECT_EQ(test, layout.cfg_sram_addr, 0x10007000);
	KUNIT_EXPECT_EQ(test, layout.cfg_size, 0x100);
}

static void parse_fw_overlay_test(struct kunit *test)
{
	const u32 sram_addrs[] = { 0x20000000, 0x10007000, 0x30000000 };
	const u32 sizes[] = { 0x1000, 0x100, 0x800 };
	struct hx83102_fw_layout layout;
	struct firmware *fw;
	u8 *table;

	fw = build_test_fw(test, 3, sram_addrs, sizes);

	/* Mark partition 2 as a code overlay (marker 0x88 0x77): it must
	 * not contribute to the config block but be staged for the
	 * handshake (himax_zf_part_info()).
	 */
	table = (u8 *)fw->data + HX83102_ZF_TABLE_OFFSET;
	table[2 * 16 + HX83102_ZF_PART_MARK_OFF] =
		HX83102_ZF_PART_MARK_OVERLAY_0;
	table[2 * 16 + HX83102_ZF_PART_MARK_OFF + 1] =
		HX83102_ZF_PART_MARK_OVERLAY_1;

	KUNIT_ASSERT_EQ(test, hx83102_parse_fw(fw, &layout), 0);

	KUNIT_EXPECT_TRUE(test, layout.has_overlay);
	KUNIT_EXPECT_EQ(test, layout.overlay_sram_addr, 0x30000000);
	KUNIT_EXPECT_EQ(test, layout.overlay_size, 0x800);
	KUNIT_EXPECT_EQ(test, layout.cfg_size, 0x100);
	KUNIT_EXPECT_EQ(test, layout.cfg_sram_addr, 0x10007000);
}

static void parse_fw_too_small_test(struct kunit *test)
{
	struct hx83102_fw_layout layout;
	u8 data[0x100] = {};
	struct firmware fw = {
		.size = sizeof(data),
		.data = data,
	};

	KUNIT_EXPECT_EQ(test, hx83102_parse_fw(&fw, &layout), -EINVAL);
}

static void parse_fw_bad_part_count_test(struct kunit *test)
{
	struct hx83102_fw_layout layout;
	struct firmware *fw;
	const u32 sram_addrs[] = { 0x20000000 };
	const u32 sizes[] = { 0x1000 };

	fw = build_test_fw(test, 1, sram_addrs, sizes);
	/* Partition count 0: rejected (himax_zf_part_info()). */
	((u8 *)fw->data)[HX83102_ZF_TABLE_OFFSET +
			 HX83102_ZF_TABLE_COUNT_OFF] = 0;

	KUNIT_EXPECT_EQ(test, hx83102_parse_fw(fw, &layout), -EINVAL);
}

static void parse_fw_partition_overrun_test(struct kunit *test)
{
	const u32 sram_addrs[] = { 0x20000000, 0x10007000 };
	const u32 sizes[] = { 0x1000, 0x100000 };
	struct hx83102_fw_layout layout;
	struct firmware *fw;

	/* Config partition larger than the file: rejected. */
	fw = build_test_fw(test, 2, sram_addrs, sizes);
	KUNIT_EXPECT_EQ(test, hx83102_parse_fw(fw, &layout), -EINVAL);
}

static void parse_fw_dsram_too_big_test(struct kunit *test)
{
	const u32 sram_addrs[] = { 0x20000000, 0x10007000 };
	const u32 sizes[] = { 0x1000, 40 * 1024 };
	struct hx83102_fw_layout layout;
	struct firmware *fw;

	/* Config block beyond the 32 KiB DSRAM window: rejected. */
	fw = build_test_fw(test, 2, sram_addrs, sizes);
	KUNIT_EXPECT_EQ(test, hx83102_parse_fw(fw, &layout), -EINVAL);
}

static struct kunit_case hx83102_test_cases[] = {
	/* CRC32C (vendor himax_mcu_Calculate_CRC_with_AP()). */
	KUNIT_CASE(crc32c_test_known),
	KUNIT_CASE(crc32c_test_all_zero),
	/* Event parser (himax_parse_report_points() and friends). */
	KUNIT_CASE(parse_event_finger_test),
	KUNIT_CASE(parse_event_out_of_range_test),
	KUNIT_CASE(parse_event_pen_test),
	KUNIT_CASE(parse_event_pen_out_of_range_test),
	KUNIT_CASE(parse_event_all_leave_test),
	KUNIT_CASE(parse_event_count_zero_test),
	KUNIT_CASE(parse_event_bad_checksum_test),
	KUNIT_CASE(parse_event_exception_eb_test),
	KUNIT_CASE(parse_event_exception_ec_ee_test),
	KUNIT_CASE(parse_event_reload_ed_test),
	KUNIT_CASE(parse_event_flush_test),
	/* Firmware layout parser (himax_zf_part_info()). */
	KUNIT_CASE(parse_fw_layout_test),
	KUNIT_CASE(parse_fw_align_test),
	KUNIT_CASE(parse_fw_overlay_test),
	KUNIT_CASE(parse_fw_too_small_test),
	KUNIT_CASE(parse_fw_bad_part_count_test),
	KUNIT_CASE(parse_fw_partition_overrun_test),
	KUNIT_CASE(parse_fw_dsram_too_big_test),
	{ /* sentinel */ }
};

static struct kunit_suite hx83102_test_suite = {
	.name = "himax_hx83102",
	.test_cases = hx83102_test_cases,
};

kunit_test_suite(hx83102_test_suite);

MODULE_DESCRIPTION("KUnit tests for the Himax HX83102 parsers");
MODULE_LICENSE("GPL");
