// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI transport for the Himax HX83102 TDDI touchscreen and pen driver
 *
 * Copyright (C) 2026 The Lenovomarchy authors
 *
 * Framing follows the vendor himax_spi_read()/himax_spi_write() in
 * himax_platform.c, Copyright (C) 2019 Himax Corporation: a read is a
 * 3-byte command prefix (0xF3, register, padding) in one transfer, then
 * a full-duplex transfer that clocks the answer in; a write is the
 * 2-byte command prefix (0xF2, register) followed by the payload.
 */

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>

#include "himax_hx83102.h"

/* Vendor himax_spi_read(): read command prefix, 0xF3 reg 0x00. */
#define HX83102_SPI_READ_PREFIX_LEN	3
/* Vendor himax_spi_write(): write command prefix, 0xF2 reg. */
#define HX83102_SPI_WRITE_PREFIX_LEN	2

static int hx83102_spi_read_reg(struct device *dev, u8 reg, void *data,
				size_t len)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 prefix[HX83102_SPI_READ_PREFIX_LEN] = {
		HX83102_SPI_CMD_READ,
		reg,
		0x00,
	};
	struct spi_transfer xfers[2] = {};
	struct spi_message msg;
	int ret;
	int try;

	/* Command prefix first (vendor himax_spi_read(), xfer[0]). */
	xfers[0].tx_buf = prefix;
	xfers[0].len = sizeof(prefix);

	/* Then the data, read back over a full-duplex transfer (xfer[1]). */
	xfers[1].tx_buf = data;
	xfers[1].rx_buf = data;
	xfers[1].len = len;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	for (try = 0; try < HX83102_SPI_XFER_RETRIES; try++) {
		ret = spi_sync(spi, &msg);
		if (!ret)
			return 0;
	}

	return ret;
}

static int hx83102_spi_write_reg(struct device *dev, u8 reg,
				 const void *data, size_t len)
{
	struct spi_device *spi = to_spi_device(dev);
	u8 prefix[HX83102_SPI_WRITE_PREFIX_LEN] = {
		HX83102_SPI_CMD_WRITE,
		reg,
	};
	struct spi_transfer xfers[2] = {};
	struct spi_message msg;
	int ret;
	int try;

	xfers[0].tx_buf = prefix;
	xfers[0].len = sizeof(prefix);

	xfers[1].tx_buf = data;
	xfers[1].len = len;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);

	for (try = 0; try < HX83102_SPI_XFER_RETRIES; try++) {
		ret = spi_sync(spi, &msg);
		if (!ret)
			return 0;
	}

	return ret;
}

static const struct hx83102_transport_ops hx83102_spi_ops = {
	.read_reg = hx83102_spi_read_reg,
	.write_reg = hx83102_spi_write_reg,
};

static const struct of_device_id hx83102_spi_of_match[] = {
	{ .compatible = "himax,hx83102p-ts" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hx83102_spi_of_match);

static const struct spi_device_id hx83102_spi_id[] = {
	{ "hx83102p-ts", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, hx83102_spi_id);

static int hx83102_spi_probe(struct spi_device *spi)
{
	int ret;

	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX)
		return dev_err_probe(&spi->dev, -EINVAL,
				     "controller does not support full duplex\n");

	/* Vendor himax_common_probe_spi(): CPOL=CPHA=1, 8 bits. */
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	return hx83102_probe(&spi->dev, spi->irq, &hx83102_spi_ops);
}

static void hx83102_spi_remove(struct spi_device *spi)
{
	hx83102_remove(&spi->dev);
}

static struct spi_driver hx83102_spi_driver = {
	.driver = {
		.name = HX83102_NAME,
		.pm = pm_sleep_ptr(&hx83102_pm_ops),
		.of_match_table = hx83102_spi_of_match,
	},
	.probe = hx83102_spi_probe,
	.remove = hx83102_spi_remove,
	.id_table = hx83102_spi_id,
};
module_spi_driver(hx83102_spi_driver);

MODULE_DESCRIPTION("SPI transport for the Himax HX83102 touchscreen driver");
MODULE_LICENSE("GPL");
