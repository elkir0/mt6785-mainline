// SPDX-License-Identifier: GPL-2.0-only
//
// Synaptics AS33970 ("Tahiti") voice processor
//
// The chip has no firmware of its own: after reset its ROM waits on I2C for
// a second-stage boot loader, which then takes the system image. Both come
// from one EFLC image file, loaded in the background after probe. Once the
// firmware runs, the processed microphone signal leaves on one I2S output.

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cache.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define AS33970_DEFAULT_FW		"tahiti/partition_boot2flash.img"

/*
 * I2C bursts are split as the vendor's loader splits them: at most 120 data
 * bytes per write and 16 bytes per read. The chip hands out each 32-bit word
 * slightly late (hence the slow bus) and longer reads have not been tried.
 */
#define AS33970_WRITE_MAX		120
#define AS33970_READ_MAX		16

/* EFLC image: header, boot loader, then the system image in 4 KiB blocks */
#define AS33970_IMG_MAGIC		0x434c4645	/* "EFLC" */
#define AS33970_IMG_HDR_SIZE		0x48
#define AS33970_IMG_BL_OFFSET		0x08
#define AS33970_IMG_BL_SIZE		0x0c
#define AS33970_IMG_SYS_OFFSET		0x26	/* u16, in 4 KiB units */
#define AS33970_SYS_HDR_SIZE		0x28
#define AS33970_SYS_BLOCKS		0x26	/* u16, in the system header */
#define AS33970_BLOCK_SIZE		4096

/*
 * Stage 1: the ROM shows a status letter in word 0. The host writes a chunk
 * at 0x10, its header (index, checksum, word count) at 0x04, then 'R' in
 * word 0; the ROM answers 'S' for the next chunk or 'T' for a checksum
 * error. 'D' ends the transfer, 'A' aborts it.
 */
#define AS33970_ROM_PROMPT		'C'	/* waiting for a loader */
#define AS33970_ROM_SEND		'S'
#define AS33970_ROM_CHUNK		'R'
#define AS33970_ROM_DONE		'D'
#define AS33970_ROM_RETRY		'T'
#define AS33970_ROM_ABORT		'A'
#define AS33970_ROM_OK			'O'
#define AS33970_ROM_FAIL		'F'
#define AS33970_LOADER_READY		0x80	/* second-stage loader running */
#define AS33970_ROM_CHUNK_SUB		0x10
#define AS33970_ROM_RETRIES		3
#define AS33970_ROM_IDLE_POLLS		200

/*
 * Stage 2: a message is {cmd, seq, le16 length, le32 Fletcher checksum,
 * arguments}, followed by its data. The loader sets bit 7 of byte 0 when it
 * has answered, and bit 6 on error with a signed 6-bit code in bits 5:0.
 */
#define AS33970_MSG_SIZE		12
#define AS33970_MSG_PING		0x00
#define AS33970_MSG_READ_STATUS		0x01
#define AS33970_MSG_WRITE_STATUS	0x02
#define AS33970_MSG_WRITE_BLOCK		0x08
#define AS33970_MSG_REBOOT		0x0e
#define AS33970_MSG_CMD			GENMASK(5, 0)
#define AS33970_MSG_DONE		BIT(7)
#define AS33970_MSG_ERROR		BIT(6)
#define AS33970_MSG_CODE		GENMASK(5, 0)
#define AS33970_PING_REPLY		0x1c
#define AS33970_FLETCHER_WORDS		359

/* Written by the firmware at sub-address 4 once it runs */
#define AS33970_READY_MAGIC		0x8badd00d

/*
 * Run-time commands. Word 0: signed word count (bits 15:0), command id
 * (bits 30:16) and the reply flag (bit 31), set by the firmware when done;
 * the count is then the number of reply words at sub-address 8, or a
 * negative error code. Word 1 is the module id, the data follows.
 */
#define AS33970_CMD_SUB			0x00
#define AS33970_CMD_ARGS_SUB		0x04
#define AS33970_CMD_REPLY_SUB		0x08
#define AS33970_CMD_COUNT		GENMASK(15, 0)
#define AS33970_CMD_ID			GENMASK(30, 16)
#define AS33970_CMD_REPLY		BIT(31)
#define AS33970_CMD_GET(id)		((id) | 0x100)
/* Data words of a command; also the count a "get" command announces */
#define AS33970_CMD_WORDS		13

#define AS33970_ID(a, b, c, d)		(((u32)(a) - 0x20) << 8 | \
					 ((u32)(b) - 0x20) << 14 | \
					 ((u32)(c) - 0x20) << 20 | \
					 ((u32)(d) - 0x20) << 26)
#define AS33970_ID_ARM			AS33970_ID('M', 'C', 'U', ' ')

#define SYS_CMD_VERSION			1
#define SYS_CMD_EXEC_FILE		9
#define SYS_CMD_PARAMETER_VALUE		14
#define SYS_CMD_EVENT_PARAM		15
#define SYS_CMD_PM_CMD			29

#define PAR_INDEX_I2S_RX_WIDTH		14
#define PAR_INDEX_I2S_RX_NUM_OF_BITS	18
#define PAR_INDEX_I2S_TX_WIDTH		19
#define PAR_INDEX_I2S_TX_NUM_OF_BITS	23
#define EVENT_PAR_RATE_MAIN_INPUT	32
#define EVENT_PAR_RATE_HOST_RECORD	38
#define EVENT_PAR_USB_RECORD_STATE	43
#define EVENT_PAR_USB_PLAYBACK_STATE	50
#define EVENT_USB_RECORD_STARTSTOP	BIT(1)
#define EVENT_USB_PLAYBACK_STARTSTOP	BIT(5)

/* I2S sample width codes */
#define AS33970_WIDTH_16		1
#define AS33970_WIDTH_24		2

#define AS33970_MODE_RAW		1

#define AS33970_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | \
					 SNDRV_PCM_FMTBIT_S24_LE | \
					 SNDRV_PCM_FMTBIT_S32_LE)

enum as33970_fw_state {
	AS33970_FW_NONE,
	AS33970_FW_LOADING,
	AS33970_FW_READY,
	AS33970_FW_FAILED,
};

struct as33970_priv {
	struct device *dev;
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	const char *fw_name;
	struct work_struct fw_work;
	struct completion fw_done;
	/* Protects the fields below and serialises run-time commands */
	struct mutex lock;
	enum as33970_fw_state fw_state;
	unsigned int mode;
	bool recording;
	unsigned int rate;
	unsigned int width;
	unsigned int frame_size;
	unsigned int bclk_ratio;
	/* I2C buffers, on their own cache lines */
	u8 tx[2 + AS33970_WRITE_MAX] __aligned(ARCH_DMA_MINALIGN);
	u8 rx[AS33970_READ_MAX];
};

struct as33970_image {
	const u8 *bl;
	size_t bl_size;
	const u8 *sys;
	size_t sys_size;	/* bytes present in the file */
	unsigned int sys_blocks;
};

struct as33970_msg {
	u8 buf[AS33970_MSG_SIZE];	/* bytes 8..11 persist across messages */
	u8 seq;
};

static const char * const as33970_mode_texts[] = { "Voice", "Raw", "Dump" };

static const u32 as33970_mode_ids[] = {
	AS33970_ID('Z', 'F', 'V', '0'),
	AS33970_ID('Z', 'F', 'V', '3'),
	AS33970_ID('Z', 'F', 'V', '2'),
};

static int as33970_write(struct as33970_priv *priv, unsigned int sub,
			 const void *data, size_t len)
{
	struct i2c_msg msg = {
		.addr = priv->client->addr,
		.buf = priv->tx,
	};
	size_t off, n;
	int ret;

	for (off = 0; off < len; off += n) {
		n = min_t(size_t, len - off, AS33970_WRITE_MAX);
		put_unaligned_be16(sub + off, priv->tx);
		memcpy(priv->tx + 2, data + off, n);
		msg.len = n + 2;
		ret = i2c_transfer(priv->client->adapter, &msg, 1);
		if (ret < 0)
			return ret;
		if (ret != 1)
			return -EIO;
	}

	return 0;
}

static int as33970_read(struct as33970_priv *priv, unsigned int sub,
			void *data, size_t len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = priv->client->addr,
			.len = 2,
			.buf = priv->tx,
		}, {
			.addr = priv->client->addr,
			.flags = I2C_M_RD,
			.buf = priv->rx,
		},
	};
	size_t off, n;
	int ret;

	for (off = 0; off < len; off += n) {
		n = min_t(size_t, len - off, AS33970_READ_MAX);
		put_unaligned_be16(sub + off, priv->tx);
		msgs[1].len = n;
		ret = i2c_transfer(priv->client->adapter, msgs, ARRAY_SIZE(msgs));
		if (ret < 0)
			return ret;
		if (ret != ARRAY_SIZE(msgs))
			return -EIO;
		memcpy(data + off, priv->rx, n);
	}

	return 0;
}

static int as33970_write_u32(struct as33970_priv *priv, unsigned int sub,
			     u32 val)
{
	__le32 word = cpu_to_le32(val);

	return as33970_write(priv, sub, &word, sizeof(word));
}

static int as33970_read_u32(struct as33970_priv *priv, unsigned int sub,
			    u32 *val)
{
	__le32 word;
	int ret;

	ret = as33970_read(priv, sub, &word, sizeof(word));
	if (ret)
		return ret;

	*val = le32_to_cpu(word);
	return 0;
}

static int as33970_parse_image(struct as33970_priv *priv,
			       const struct firmware *fw,
			       struct as33970_image *img)
{
	const u8 *data = fw->data;
	size_t size = fw->size;
	size_t bl_offset, sys_offset;

	if (size < AS33970_IMG_HDR_SIZE ||
	    get_unaligned_le32(data) != AS33970_IMG_MAGIC) {
		dev_err(priv->dev, "%s is not an EFLC image\n", priv->fw_name);
		return -EINVAL;
	}

	bl_offset = get_unaligned_le32(data + AS33970_IMG_BL_OFFSET);
	img->bl_size = get_unaligned_le32(data + AS33970_IMG_BL_SIZE);
	if (!img->bl_size || img->bl_size % 4 || bl_offset > size ||
	    img->bl_size > size - bl_offset) {
		dev_err(priv->dev, "%s: bad boot loader range\n", priv->fw_name);
		return -EINVAL;
	}
	img->bl = data + bl_offset;

	sys_offset = (size_t)get_unaligned_le16(data + AS33970_IMG_SYS_OFFSET) << 12;
	if (sys_offset > size || size - sys_offset < AS33970_SYS_HDR_SIZE) {
		dev_err(priv->dev, "%s: bad system image offset\n",
			priv->fw_name);
		return -EINVAL;
	}
	img->sys = data + sys_offset;

	/* Only the last block may be cut short; it is padded with 0xff. */
	img->sys_blocks = get_unaligned_le16(img->sys + AS33970_SYS_BLOCKS);
	if (!img->sys_blocks ||
	    (size_t)(img->sys_blocks - 1) * AS33970_BLOCK_SIZE >= size - sys_offset) {
		dev_err(priv->dev, "%s: bad system block count %u\n",
			priv->fw_name, img->sys_blocks);
		return -EINVAL;
	}
	img->sys_size = min_t(size_t, size - sys_offset,
			      (size_t)img->sys_blocks * AS33970_BLOCK_SIZE);

	return 0;
}

static int as33970_rom_wait_prompt(struct as33970_priv *priv)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	u32 st;
	int ret;

	/*
	 * Right after reset the ROM does not acknowledge its address for a
	 * while: I2C errors only mean "not ready yet".
	 */
	for (;;) {
		ret = as33970_read_u32(priv, 0, &st);
		if (!ret && st == AS33970_ROM_PROMPT)
			return 0;
		if (time_after_eq(jiffies, timeout))
			break;
		usleep_range(1000, 2000);
	}

	if (ret)
		dev_warn(priv->dev, "no answer from the ROM after reset: %d\n",
			 ret);
	else
		dev_warn(priv->dev, "no ROM prompt after reset (status 0x%08x)\n",
			 st);
	return -ETIMEDOUT;
}

static int as33970_rom_send_chunk(struct as33970_priv *priv,
				  const struct as33970_image *img,
				  unsigned int idx)
{
	size_t off = (size_t)idx * AS33970_BLOCK_SIZE;
	const u8 *chunk = img->bl + off;
	size_t len = min_t(size_t, img->bl_size - off, AS33970_BLOCK_SIZE);
	u32 nwords = len / 4;
	u32 sum = AS33970_ROM_CHUNK + idx + nwords;
	__le32 hdr[3];
	size_t i;
	int ret;

	/* The ROM accepts a chunk whose words and header sum to 0xffffffff. */
	for (i = 0; i < len; i += 4)
		sum += get_unaligned_le32(chunk + i);

	hdr[0] = cpu_to_le32(idx);
	hdr[1] = cpu_to_le32(~sum);
	hdr[2] = cpu_to_le32(nwords);

	ret = as33970_write(priv, AS33970_ROM_CHUNK_SUB, chunk, len);
	if (ret)
		return ret;
	ret = as33970_write(priv, 4, hdr, sizeof(hdr));
	if (ret)
		return ret;
	return as33970_write_u32(priv, 0, AS33970_ROM_CHUNK);
}

static int as33970_stage1(struct as33970_priv *priv,
			  const struct as33970_image *img)
{
	unsigned int nchunks = DIV_ROUND_UP(img->bl_size, AS33970_BLOCK_SIZE);
	unsigned int next = 0, last = 0, retries = 0, idle = 0;
	unsigned long timeout;
	u32 st;
	int ret;

	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	msleep(20);
	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	ret = as33970_rom_wait_prompt(priv);
	if (ret)
		return ret;

	ret = as33970_write_u32(priv, 0, 0);
	if (ret)
		return ret;

	timeout = jiffies + msecs_to_jiffies(10000);
	for (;;) {
		if (time_after_eq(jiffies, timeout)) {
			dev_warn(priv->dev, "stage 1: timeout after chunk %u of %u\n",
				 next, nchunks);
			return -ETIMEDOUT;
		}

		ret = as33970_read_u32(priv, 0, &st);
		if (ret)
			return ret;

		if (st == 0 || st == AS33970_ROM_PROMPT) {
			if (++idle > AS33970_ROM_IDLE_POLLS) {
				dev_warn(priv->dev, "stage 1: stuck at status 0x%08x\n",
					 st);
				return -ETIMEDOUT;
			}
			usleep_range(1000, 2000);
			continue;
		}
		idle = 0;

		switch (st) {
		case AS33970_ROM_SEND:
		case AS33970_LOADER_READY:
			if (next >= nchunks)
				return as33970_write_u32(priv, 0, AS33970_ROM_DONE);
			ret = as33970_rom_send_chunk(priv, img, next);
			if (ret)
				return ret;
			last = next++;
			retries = 0;
			break;
		case AS33970_ROM_CHUNK:
		case AS33970_ROM_DONE:
			usleep_range(1000, 2000);
			break;
		case AS33970_ROM_RETRY:
			if (!next) {
				dev_warn(priv->dev, "stage 1: retry asked before any chunk\n");
				return -EIO;
			}
			if (++retries > AS33970_ROM_RETRIES) {
				dev_warn(priv->dev, "stage 1: chunk %u rejected %u times\n",
					 last, retries);
				as33970_write_u32(priv, 0, AS33970_ROM_ABORT);
				return -EIO;
			}
			ret = as33970_rom_send_chunk(priv, img, last);
			if (ret)
				return ret;
			break;
		case AS33970_ROM_OK:
			return 0;
		case AS33970_ROM_FAIL:
			dev_warn(priv->dev, "stage 1: the ROM reported a failure\n");
			return -EIO;
		default:
			dev_warn(priv->dev, "stage 1: unexpected status 0x%08x\n",
				 st);
			return -EIO;
		}
	}
}

static int as33970_wait_loader(struct as33970_priv *priv)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(2000);
	u32 st;
	int ret;

	do {
		ret = as33970_read_u32(priv, 0, &st);
		if (!ret && st == AS33970_LOADER_READY)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, timeout));

	if (ret)
		dev_warn(priv->dev, "no answer from the boot loader: %d\n", ret);
	else
		dev_warn(priv->dev, "boot loader not started (status 0x%08x)\n",
			 st);
	return -ETIMEDOUT;
}

static u32 as33970_fletcher(const u8 *buf, size_t len)
{
	size_t n = len / 2, k;
	u32 s1 = 0xffff, s2 = 0xffff;

	while (n) {
		k = min_t(size_t, n, AS33970_FLETCHER_WORDS);
		n -= k;
		while (k--) {
			s1 += get_unaligned_le16(buf);
			s2 += s1;
			buf += 2;
		}
		s1 = (s1 & 0xffff) + (s1 >> 16);
		s2 = (s2 & 0xffff) + (s2 >> 16);
	}

	return (((s2 & 0xffff) + (s2 >> 16)) << 16) |
	       ((s1 & 0xffff) + (s1 >> 16));
}

static const char *as33970_msg_error(int code)
{
	switch (code) {
	case -1:
		return "message error";
	case -2:
	case -5:
		return "unsupported flash";
	default:
		return "image error";
	}
}

static int as33970_send_msg(struct as33970_priv *priv, struct as33970_msg *m,
			    u8 cmd, u16 len, const u8 *data, size_t data_len,
			    u8 *reply, size_t reply_len)
{
	unsigned long timeout;
	u8 hdr[4];
	int code, ret;

	m->buf[0] = cmd & AS33970_MSG_CMD;
	m->buf[1] = m->seq;
	put_unaligned_le16(len, &m->buf[2]);
	put_unaligned_le32(0, &m->buf[4]);
	put_unaligned_le32(as33970_fletcher(m->buf, len), &m->buf[4]);

	if (data_len) {
		ret = as33970_write(priv, len, data, data_len);
		if (ret)
			return ret;
	}
	ret = as33970_write(priv, 4, &m->buf[4], len - 4);
	if (ret)
		return ret;
	ret = as33970_write(priv, 0, m->buf, 4);
	if (ret)
		return ret;

	timeout = jiffies + msecs_to_jiffies(5000);
	for (;;) {
		ret = as33970_read(priv, 0, hdr, sizeof(hdr));
		if (ret)
			return ret;
		if (hdr[0] & AS33970_MSG_DONE)
			break;
		if (time_after_eq(jiffies, timeout)) {
			dev_warn(priv->dev, "stage 2: no reply to message 0x%02x\n",
				 cmd);
			return -ETIMEDOUT;
		}
		usleep_range(1000, 2000);
	}

	m->seq++;
	if (hdr[0] & AS33970_MSG_ERROR) {
		code = sign_extend32(FIELD_GET(AS33970_MSG_CODE, hdr[0]), 5);
		dev_warn(priv->dev, "stage 2: message 0x%02x failed: %d (%s)\n",
			 cmd, code, as33970_msg_error(code));
		return -EIO;
	}

	return as33970_read(priv, 0, reply, reply_len);
}

/* Trailing 0xffffffff words are not sent, unless the block is all 0xff. */
static size_t as33970_trim_block(const u8 *blk)
{
	size_t end = AS33970_BLOCK_SIZE;

	while (end && get_unaligned_le32(blk + end - 4) == 0xffffffff)
		end -= 4;

	return end ?: AS33970_BLOCK_SIZE;
}

static int as33970_stage2(struct as33970_priv *priv,
			  const struct as33970_image *img)
{
	struct as33970_msg m = { };
	u8 reply[AS33970_PING_REPLY];
	size_t off, n, len;
	unsigned int i;
	u32 status;
	int ret;

	u8 *blk __free(kfree) = kmalloc(AS33970_BLOCK_SIZE, GFP_KERNEL);
	if (!blk)
		return -ENOMEM;

	ret = as33970_send_msg(priv, &m, AS33970_MSG_PING, 8, NULL, 0,
			       reply, AS33970_PING_REPLY);
	if (ret)
		return ret;

	put_unaligned_le32(0, &m.buf[8]);
	ret = as33970_send_msg(priv, &m, AS33970_MSG_WRITE_STATUS, 12, NULL, 0,
			       reply, 12);
	if (ret)
		return ret;

	ret = as33970_send_msg(priv, &m, AS33970_MSG_READ_STATUS, 8, NULL, 0,
			       reply, 12);
	if (ret)
		return ret;

	for (i = 0; i < img->sys_blocks; i++) {
		off = (size_t)i * AS33970_BLOCK_SIZE;
		n = min_t(size_t, img->sys_size - off, AS33970_BLOCK_SIZE);
		memcpy(blk, img->sys + off, n);
		memset(blk + n, 0xff, AS33970_BLOCK_SIZE - n);
		len = as33970_trim_block(blk);

		put_unaligned_le16(i, &m.buf[8]);
		put_unaligned_le16(len, &m.buf[10]);
		ret = as33970_send_msg(priv, &m, AS33970_MSG_WRITE_BLOCK, 12,
				       blk, len, reply, 12);
		if (ret)
			return ret;
	}

	/* The reboot message carries the last block's arguments, as sent. */
	ret = as33970_send_msg(priv, &m, AS33970_MSG_REBOOT, 12, NULL, 0,
			       reply, 12);
	if (ret)
		return ret;

	status = get_unaligned_le32(&reply[8]);
	if (status) {
		dev_warn(priv->dev, "stage 2: reboot failed (status 0x%08x)\n",
			 status);
		return -EIO;
	}

	return 0;
}

static int as33970_wait_ready(struct as33970_priv *priv)
{
	unsigned long timeout;
	u32 val;
	int ret;

	msleep(1000);
	timeout = jiffies + msecs_to_jiffies(5000);
	for (;;) {
		ret = as33970_read_u32(priv, 4, &val);
		if (ret)
			return ret;
		if (val == AS33970_READY_MAGIC)
			return 0;
		if (time_after_eq(jiffies, timeout)) {
			dev_warn(priv->dev, "firmware not ready (0x%08x)\n", val);
			return -ETIMEDOUT;
		}
		msleep(50);
	}
}

static int as33970_sendcmd(struct as33970_priv *priv, u32 cmd_id,
			   u32 module_id, const u32 *data, int count,
			   u32 *reply, int reply_max)
{
	__le32 words[2 + AS33970_CMD_WORDS];
	unsigned long timeout;
	u32 word0;
	int i, n, ret;

	lockdep_assert_held(&priv->lock);

	if (WARN_ON(count < 0 || count > AS33970_CMD_WORDS))
		return -EINVAL;

	n = (cmd_id & AS33970_CMD_GET(0)) ? AS33970_CMD_WORDS : count;
	words[0] = cpu_to_le32(FIELD_PREP(AS33970_CMD_COUNT, n) |
			       FIELD_PREP(AS33970_CMD_ID, cmd_id));
	words[1] = cpu_to_le32(module_id);
	for (i = 0; i < count; i++)
		words[2 + i] = cpu_to_le32(data[i]);

	ret = as33970_write(priv, AS33970_CMD_ARGS_SUB, &words[1],
			    (count + 1) * sizeof(words[0]));
	if (ret)
		return ret;
	ret = as33970_write(priv, AS33970_CMD_SUB, &words[0], sizeof(words[0]));
	if (ret)
		return ret;

	timeout = jiffies + msecs_to_jiffies(2000);
	do {
		usleep_range(5000, 6000);
		if (as33970_read_u32(priv, AS33970_CMD_SUB, &word0))
			word0 = 0;
		if (word0 & AS33970_CMD_REPLY)
			break;
	} while (time_before(jiffies, timeout));

	if (!(word0 & AS33970_CMD_REPLY)) {
		dev_err(priv->dev, "command 0x%x: no reply\n", cmd_id);
		return -ETIMEDOUT;
	}

	n = (s16)FIELD_GET(AS33970_CMD_COUNT, word0);
	if (n < 0) {
		dev_err(priv->dev, "command 0x%x to module 0x%08x failed: %d\n",
			cmd_id, module_id, n);
		return -EIO;
	}

	n = min3(n, reply ? reply_max : 0, AS33970_CMD_WORDS);
	if (n <= 0)
		return 0;

	ret = as33970_read(priv, AS33970_CMD_REPLY_SUB, &words[2],
			   n * sizeof(words[0]));
	if (ret)
		return ret;
	for (i = 0; i < n; i++)
		reply[i] = le32_to_cpu(words[2 + i]);

	return n;
}

static int as33970_set_mode(struct as33970_priv *priv, u32 mode_id)
{
	static const u32 idle[] = { 1, 0, 1 };
	int ret, try;

	for (try = 0; try < 3; try++) {
		ret = as33970_sendcmd(priv, SYS_CMD_PM_CMD, AS33970_ID_ARM,
				      idle, ARRAY_SIZE(idle), NULL, 0);
		if (ret < 0)
			continue;

		msleep(try ? 500 : 50);

		ret = as33970_sendcmd(priv, SYS_CMD_EXEC_FILE, AS33970_ID_ARM,
				      &mode_id, 1, NULL, 0);
		if (ret >= 0)
			return 0;
	}

	return ret;
}

static int as33970_set_param(struct as33970_priv *priv, u32 index, u32 value)
{
	const u32 data[] = { index, value };

	return as33970_sendcmd(priv, SYS_CMD_PARAMETER_VALUE, AS33970_ID_ARM,
			       data, ARRAY_SIZE(data), NULL, 0);
}

static int as33970_send_event(struct as33970_priv *priv, const u32 *data,
			      int count)
{
	return as33970_sendcmd(priv, SYS_CMD_EVENT_PARAM, AS33970_ID_ARM,
			       data, count, NULL, 0);
}

static int as33970_playback_event(struct as33970_priv *priv, bool start)
{
	const u32 data[] = {
		EVENT_USB_PLAYBACK_STARTSTOP, EVENT_PAR_USB_PLAYBACK_STATE, start,
	};

	return as33970_send_event(priv, data, ARRAY_SIZE(data));
}

static int as33970_start_record(struct as33970_priv *priv)
{
	const u32 event[] = {
		EVENT_USB_RECORD_STARTSTOP,
		EVENT_PAR_USB_RECORD_STATE, 1,
		EVENT_PAR_RATE_HOST_RECORD, priv->rate,
		PAR_INDEX_I2S_RX_WIDTH, priv->width,
		PAR_INDEX_I2S_RX_NUM_OF_BITS, priv->frame_size,
	};
	int ret;

	ret = as33970_set_param(priv, EVENT_PAR_RATE_MAIN_INPUT, priv->rate);
	if (ret < 0)
		return ret;
	ret = as33970_set_param(priv, PAR_INDEX_I2S_TX_WIDTH, priv->width);
	if (ret < 0)
		return ret;
	ret = as33970_set_param(priv, PAR_INDEX_I2S_TX_NUM_OF_BITS,
				priv->frame_size);
	if (ret < 0)
		return ret;
	ret = as33970_send_event(priv, event, ARRAY_SIZE(event));
	if (ret < 0)
		return ret;

	return as33970_playback_event(priv, true);
}

static void as33970_stop_record(struct as33970_priv *priv)
{
	static const u32 event[] = {
		EVENT_USB_RECORD_STARTSTOP, EVENT_PAR_USB_RECORD_STATE, 0,
	};
	int ret;

	ret = as33970_send_event(priv, event, ARRAY_SIZE(event));
	if (ret < 0)
		dev_err(priv->dev, "cannot stop recording: %d\n", ret);

	ret = as33970_playback_event(priv, false);
	if (ret < 0)
		dev_err(priv->dev, "cannot send the playback stop event: %d\n",
			ret);
}

static int as33970_boot(struct as33970_priv *priv,
			const struct as33970_image *img, const char **step)
{
	u32 ver[4];
	int ret;

	*step = "stage 1";
	ret = as33970_stage1(priv, img);
	if (ret)
		return ret;

	*step = "boot loader start";
	ret = as33970_wait_loader(priv);
	if (ret)
		return ret;

	*step = "stage 2";
	ret = as33970_stage2(priv, img);
	if (ret)
		return ret;

	*step = "firmware start";
	ret = as33970_wait_ready(priv);
	if (ret)
		return ret;

	guard(mutex)(&priv->lock);

	*step = "version query";
	ret = as33970_sendcmd(priv, AS33970_CMD_GET(SYS_CMD_VERSION),
			      AS33970_ID_ARM, NULL, 0, ver, ARRAY_SIZE(ver));
	if (ret < 0)
		return ret;
	if (ret < ARRAY_SIZE(ver))
		return -EIO;

	*step = "mode selection";
	ret = as33970_set_mode(priv, as33970_mode_ids[priv->mode]);
	if (ret < 0)
		return ret;

	priv->fw_state = AS33970_FW_READY;
	dev_info(priv->dev, "firmware %u.%u.%u.%u\n",
		 ver[0], ver[1], ver[2], ver[3]);

	return 0;
}

static int as33970_load(struct as33970_priv *priv, const struct firmware *fw)
{
	struct as33970_image img;
	const char *step;
	int ret;

	ret = as33970_parse_image(priv, fw, &img);
	if (ret)
		return ret;

	ret = as33970_boot(priv, &img, &step);
	if (!ret)
		return 0;

	dev_warn(priv->dev, "firmware load failed at %s (%d), retrying\n",
		 step, ret);
	ret = as33970_boot(priv, &img, &step);
	if (ret)
		dev_err(priv->dev, "firmware load failed at %s: %d\n", step, ret);

	return ret;
}

static void as33970_fw_work(struct work_struct *work)
{
	struct as33970_priv *priv = container_of(work, struct as33970_priv,
						 fw_work);
	const struct firmware *fw;
	int ret;

	scoped_guard(mutex, &priv->lock)
		priv->fw_state = AS33970_FW_LOADING;

	ret = request_firmware(&fw, priv->fw_name, priv->dev);
	if (ret) {
		dev_err(priv->dev, "cannot load %s: %d\n", priv->fw_name, ret);
	} else {
		ret = as33970_load(priv, fw);
		release_firmware(fw);
	}

	if (ret) {
		scoped_guard(mutex, &priv->lock)
			priv->fw_state = AS33970_FW_FAILED;
	}
	complete_all(&priv->fw_done);
}

static int as33970_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;

	return 0;
}

static int as33970_set_bclk_ratio(struct snd_soc_dai *dai, unsigned int ratio)
{
	struct as33970_priv *priv = snd_soc_component_get_drvdata(dai->component);

	priv->bclk_ratio = ratio;
	return 0;
}

static int as33970_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct as33970_priv *priv = snd_soc_component_get_drvdata(dai->component);
	int frame_size;

	switch (params_width(params)) {
	case 16:
		priv->width = AS33970_WIDTH_16;
		break;
	case 24:
	case 32:
		priv->width = AS33970_WIDTH_24;
		break;
	default:
		return -EINVAL;
	}

	if (priv->bclk_ratio) {
		frame_size = priv->bclk_ratio;
	} else {
		/* Two slots of the sample's size on the wire (S24_LE: 32) */
		frame_size = 2 * params_physical_width(params);
		if (frame_size <= 0)
			return -EINVAL;
	}

	priv->rate = params_rate(params);
	priv->frame_size = frame_size;

	return 0;
}

static int as33970_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	struct as33970_priv *priv = snd_soc_component_get_drvdata(dai->component);
	long left;

	left = wait_for_completion_interruptible_timeout(&priv->fw_done,
							 secs_to_jiffies(30));
	if (left < 0)
		return left;

	guard(mutex)(&priv->lock);

	return priv->fw_state == AS33970_FW_READY ? 0 : -ENODEV;
}

static const struct snd_soc_dai_ops as33970_dai_ops = {
	.set_fmt = as33970_set_fmt,
	.set_bclk_ratio = as33970_set_bclk_ratio,
	.hw_params = as33970_hw_params,
	.prepare = as33970_prepare,
};

static struct snd_soc_dai_driver as33970_dai = {
	.name = "as33970-aif",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = AS33970_FORMATS,
	},
	.ops = &as33970_dai_ops,
};

static int as33970_mode_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct as33970_priv *priv = snd_soc_component_get_drvdata(component);

	guard(mutex)(&priv->lock);
	ucontrol->value.enumerated.item[0] = priv->mode;

	return 0;
}

static int as33970_mode_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct as33970_priv *priv = snd_soc_component_get_drvdata(component);
	unsigned int mode = ucontrol->value.enumerated.item[0];
	int ret;

	if (mode >= ARRAY_SIZE(as33970_mode_ids))
		return -EINVAL;

	guard(mutex)(&priv->lock);

	if (priv->recording)
		return -EBUSY;
	if (mode == priv->mode)
		return 0;

	if (priv->fw_state == AS33970_FW_READY) {
		ret = as33970_set_mode(priv, as33970_mode_ids[mode]);
		if (ret < 0)
			return ret;
	}

	priv->mode = mode;
	return 1;
}

static SOC_ENUM_SINGLE_EXT_DECL(as33970_mode_enum, as33970_mode_texts);

static const struct snd_kcontrol_new as33970_controls[] = {
	SOC_ENUM_EXT("Capture Mode", as33970_mode_enum,
		     as33970_mode_get, as33970_mode_put),
};

/*
 * The firmware refuses the sample rate (error -1061) while the I2S clocks
 * are stopped: recording starts here, once DAPM has powered the CPU side's
 * clock supplies, not in the DAI's prepare().
 */
static int as33970_aif_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct as33970_priv *priv = snd_soc_component_get_drvdata(component);
	int ret;

	guard(mutex)(&priv->lock);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		if (priv->fw_state != AS33970_FW_READY)
			return -ENODEV;
		if (priv->recording)
			return 0;
		ret = as33970_start_record(priv);
		if (ret < 0)
			return ret;
		priv->recording = true;
		break;
	case SND_SOC_DAPM_PRE_PMD:
		if (priv->recording) {
			as33970_stop_record(priv);
			priv->recording = false;
		}
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget as33970_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_AIF_OUT_E("AIF", "Capture", 0, SND_SOC_NOPM, 0, 0,
			       as33970_aif_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
};

static const struct snd_soc_dapm_route as33970_dapm_routes[] = {
	{ "AIF", NULL, "MIC" },
};

static const struct snd_soc_component_driver as33970_component = {
	.controls = as33970_controls,
	.num_controls = ARRAY_SIZE(as33970_controls),
	.dapm_widgets = as33970_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(as33970_dapm_widgets),
	.dapm_routes = as33970_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(as33970_dapm_routes),
};

static int as33970_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct as33970_priv *priv;
	struct gpio_desc *enable;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->client = client;
	priv->mode = AS33970_MODE_RAW;
	i2c_set_clientdata(client, priv);

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;
	init_completion(&priv->fw_done);
	INIT_WORK(&priv->fw_work, as33970_fw_work);

	/* Held in reset until the firmware load starts */
	priv->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "cannot get the reset GPIO\n");

	enable = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(enable))
		return dev_err_probe(dev, PTR_ERR(enable),
				     "cannot get the enable GPIO\n");

	priv->fw_name = AS33970_DEFAULT_FW;
	device_property_read_string(dev, "firmware-name", &priv->fw_name);

	ret = snd_soc_register_component(dev, &as33970_component,
					 &as33970_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register the component\n");

	queue_work(system_long_wq, &priv->fw_work);

	return 0;
}

static void as33970_i2c_remove(struct i2c_client *client)
{
	struct as33970_priv *priv = i2c_get_clientdata(client);

	snd_soc_unregister_component(&client->dev);
	cancel_work_sync(&priv->fw_work);
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
}

static const struct of_device_id as33970_of_match[] = {
	{ .compatible = "synaptics,as33970" },
	{ }
};
MODULE_DEVICE_TABLE(of, as33970_of_match);

static const struct i2c_device_id as33970_i2c_id[] = {
	{ "as33970" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, as33970_i2c_id);

static struct i2c_driver as33970_i2c_driver = {
	.driver = {
		.name = "as33970",
		.of_match_table = as33970_of_match,
	},
	.probe = as33970_i2c_probe,
	.remove = as33970_i2c_remove,
	.id_table = as33970_i2c_id,
};
module_i2c_driver(as33970_i2c_driver);

MODULE_DESCRIPTION("ASoC Synaptics AS33970 voice processor driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(AS33970_DEFAULT_FW);
