// SPDX-License-Identifier: GPL-2.0
/*
 * yt11-cam-probe: M0 power-up probe for Lenovo Yoga Tab 11 (YT-J706F) cameras.
 *
 * Powers one camera slot (cam0 or cam1) at a time with its shared regulators,
 * per-slot reset GPIO, and optional 24 MHz CAMTG master clock so userspace can
 * identify sensors over I2C (/dev/i2c-N).
 *
 * debugfs interface under /sys/kernel/debug/yt11-cam/:
 *   power   write "<slot> on" | "<slot> off"   (slot = 0 or 1)
 *   status  read current rails, GPIOs, MCLK pin mux, and CAMTG clock rates
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#define NSLOT	2
#define NTG	5

static int stage = 1;
module_param(stage, int, 0444);
MODULE_PARM_DESC(stage, "0: regulators/GPIOs only (no MCLK); 1: +24 MHz CAMTG MCLK (default)");

static int camtg[NSLOT] = { 0, 0 };
static unsigned int camtg_argc;
module_param_array(camtg, int, &camtg_argc, 0644);
MODULE_PARM_DESC(camtg, "CAMTG clock index (1..5 for CLK_TOP_CAMTG1..5) per slot [cam0,cam1]; single value applies to active slot; 0 = enable all 5 (default)");

struct yt11_cam {
	struct device *dev;
	struct pinctrl *pc;
	struct pinctrl_state *mclk_off[NSLOT];
	struct pinctrl_state *mclk_on[NSLOT];
	struct regulator *vcamio;
	struct gpio_desc *vcama;
	struct gpio_desc *vcamd;
	struct gpio_desc *vcamaf;
	struct gpio_desc *rst[NSLOT];
	struct clk *tg[NTG];
	struct clk *src;
	struct mutex lock;
	struct dentry *dbg;
	u32 clk_mask;
	int cur;		/* active slot (0 or 1), or -1 if off */
	int mclk_slot;		/* slot whose MCLK pin is muxed, or -1 */
	bool io_on;
	bool a_on;
	bool d_on;
	bool af_on;
	bool rst_on[NSLOT];
};

static int slot_camtg_idx(int slot)
{
	if (camtg_argc == 1)
		return camtg[0];
	return camtg[slot];
}

static int clocks_on(struct yt11_cam *c, int slot)
{
	int idx = slot_camtg_idx(slot);
	int start, end, i, ret;

	if (idx < 0 || idx > NTG)
		return -EINVAL;

	if (idx == 0) {
		start = 0;
		end = NTG - 1;
	} else {
		start = idx - 1;
		end = idx - 1;
	}

	for (i = start; i <= end; i++) {
		if (c->src) {
			ret = clk_set_parent(c->tg[i], c->src);
			if (ret)
				dev_warn(c->dev, "camtg%d set_parent: %d\n", i + 1, ret);
		}
		ret = clk_prepare_enable(c->tg[i]);
		if (ret) {
			dev_err(c->dev, "camtg%d enable: %d\n", i + 1, ret);
			while (--i >= start) {
				if (c->clk_mask & BIT(i)) {
					clk_disable_unprepare(c->tg[i]);
					c->clk_mask &= ~BIT(i);
				}
			}
			return ret;
		}
		c->clk_mask |= BIT(i);
	}
	return 0;
}

static void clocks_off(struct yt11_cam *c)
{
	int i;

	for (i = 0; i < NTG; i++) {
		if (c->clk_mask & BIT(i)) {
			clk_disable_unprepare(c->tg[i]);
			c->clk_mask &= ~BIT(i);
		}
	}
}

static void power_down_locked(struct yt11_cam *c)
{
	int i;

	for (i = 0; i < NSLOT; i++) {
		if (c->rst_on[i]) {
			gpiod_set_value_cansleep(c->rst[i], 0);
			c->rst_on[i] = false;
		}
	}
	msleep(1);

	if (c->mclk_slot >= 0) {
		pinctrl_select_state(c->pc, c->mclk_off[c->mclk_slot]);
		c->mclk_slot = -1;
	}
	clocks_off(c);

	if (c->af_on && c->vcamaf) {
		gpiod_set_value_cansleep(c->vcamaf, 0);
		c->af_on = false;
	}
	if (c->d_on) {
		gpiod_set_value_cansleep(c->vcamd, 0);
		c->d_on = false;
		msleep(1);
	}
	if (c->a_on) {
		gpiod_set_value_cansleep(c->vcama, 0);
		c->a_on = false;
		msleep(1);
	}
	if (c->io_on) {
		regulator_disable(c->vcamio);
		c->io_on = false;
	}
	c->cur = -1;
}

static int slot_on(struct yt11_cam *c, int slot)
{
	int ret;

	if (c->cur >= 0)
		return -EBUSY;

	if (stage >= 1) {
		int idx = slot_camtg_idx(slot);

		if (idx < 0 || idx > NTG)
			return -EINVAL;
	}

	gpiod_set_value_cansleep(c->rst[0], 0);
	gpiod_set_value_cansleep(c->rst[1], 0);
	ret = pinctrl_select_state(c->pc, c->mclk_off[1 - slot]);
	if (ret)
		dev_warn(c->dev, "slot%d mclk_off: %d\n", 1 - slot, ret);

	ret = regulator_set_voltage(c->vcamio, 1800000, 1800000);
	if (ret)
		dev_warn(c->dev, "vcamio set_voltage: %d\n", ret);
	ret = regulator_enable(c->vcamio);
	if (ret) {
		dev_err(c->dev, "vcamio enable: %d\n", ret);
		return ret;
	}
	c->io_on = true;
	msleep(1);

	gpiod_set_value_cansleep(c->vcama, 1);
	c->a_on = true;
	msleep(1);

	gpiod_set_value_cansleep(c->vcamd, 1);
	c->d_on = true;
	msleep(1);

	if (slot == 0 && c->vcamaf) {
		gpiod_set_value_cansleep(c->vcamaf, 1);
		c->af_on = true;
		msleep(1);
	}

	if (stage >= 1) {
		ret = clocks_on(c, slot);
		if (ret)
			goto err_off;
		ret = pinctrl_select_state(c->pc, c->mclk_on[slot]);
		if (ret) {
			dev_err(c->dev, "slot%d mclk_on: %d\n", slot, ret);
			goto err_off;
		}
		c->mclk_slot = slot;
	}

	msleep(5);
	gpiod_set_value_cansleep(c->rst[slot], 1);
	c->rst_on[slot] = true;
	msleep(10);

	c->cur = slot;
	dev_info(c->dev, "slot %d on (stage=%d camtg=%d mask=0x%x)\n",
		 slot, stage, slot_camtg_idx(slot), c->clk_mask);
	return 0;

err_off:
	power_down_locked(c);
	return ret;
}

static int slot_off(struct yt11_cam *c, int slot)
{
	if (c->cur < 0)
		return 0;
	if (c->cur != slot)
		return -EINVAL;

	power_down_locked(c);
	dev_info(c->dev, "slot %d off\n", slot);
	return 0;
}

static ssize_t power_write(struct file *f, const char __user *ubuf,
			   size_t len, loff_t *off)
{
	struct yt11_cam *c = file_inode(f)->i_private;
	char buf[32], cmd[8] = "";
	int slot, n, ret;

	if (!len || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	n = sscanf(buf, "%d %7s", &slot, cmd);
	if (n != 2 || slot < 0 || slot >= NSLOT)
		return -EINVAL;

	mutex_lock(&c->lock);
	if (!strcmp(cmd, "on"))
		ret = slot_on(c, slot);
	else if (!strcmp(cmd, "off"))
		ret = slot_off(c, slot);
	else
		ret = -EINVAL;
	mutex_unlock(&c->lock);

	return ret ? ret : len;
}

static int status_show(struct seq_file *m, void *v)
{
	struct yt11_cam *c = m->private;
	int i;

	mutex_lock(&c->lock);
	seq_printf(m, "stage=%d slot=%d camtg_param=%d,%d\n",
		   stage, c->cur, camtg[0], camtg[1]);
	seq_printf(m, "vcamio=%d vcama=%d vcamd=%d vcamaf=%d rst0=%d rst1=%d mclk_slot=%d\n",
		   c->io_on, c->a_on, c->d_on, c->af_on,
		   c->rst_on[0], c->rst_on[1], c->mclk_slot);
	seq_printf(m, "clk_mask=0x%x rates=", c->clk_mask);
	for (i = 0; i < NTG; i++) {
		unsigned long rate = (c->clk_mask & BIT(i)) ? clk_get_rate(c->tg[i]) : 0;

		seq_printf(m, "%s%lu", i ? "," : "", rate);
	}
	seq_putc(m, '\n');
	mutex_unlock(&c->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(status);

static const struct file_operations power_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = power_write,
	.llseek = noop_llseek,
};

static int yt11_cam_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct yt11_cam *c;
	char name[24];
	int i, ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->dev = dev;
	c->cur = -1;
	c->mclk_slot = -1;
	mutex_init(&c->lock);

	c->pc = devm_pinctrl_get(dev);
	if (IS_ERR(c->pc))
		return dev_err_probe(dev, PTR_ERR(c->pc), "pinctrl\n");

	for (i = 0; i < NSLOT; i++) {
		snprintf(name, sizeof(name), "cam%d_mclk_off", i);
		c->mclk_off[i] = pinctrl_lookup_state(c->pc, name);
		if (IS_ERR(c->mclk_off[i]))
			return dev_err_probe(dev, PTR_ERR(c->mclk_off[i]), "%s\n", name);

		snprintf(name, sizeof(name), "cam%d_mclk_on", i);
		c->mclk_on[i] = pinctrl_lookup_state(c->pc, name);
		if (IS_ERR(c->mclk_on[i]))
			return dev_err_probe(dev, PTR_ERR(c->mclk_on[i]), "%s\n", name);

		ret = pinctrl_select_state(c->pc, c->mclk_off[i]);
		if (ret)
			return dev_err_probe(dev, ret, "select %s\n", name);
	}

	c->vcamio = devm_regulator_get(dev, "vcamio");
	if (IS_ERR(c->vcamio))
		return dev_err_probe(dev, PTR_ERR(c->vcamio), "vcamio\n");

	c->vcama = devm_gpiod_get(dev, "vcama", GPIOD_OUT_LOW);
	if (IS_ERR(c->vcama))
		return dev_err_probe(dev, PTR_ERR(c->vcama), "vcama\n");

	c->vcamd = devm_gpiod_get(dev, "vcamd", GPIOD_OUT_LOW);
	if (IS_ERR(c->vcamd))
		return dev_err_probe(dev, PTR_ERR(c->vcamd), "vcamd\n");

	c->vcamaf = devm_gpiod_get_optional(dev, "vcamaf", GPIOD_OUT_LOW);
	if (IS_ERR(c->vcamaf))
		return dev_err_probe(dev, PTR_ERR(c->vcamaf), "vcamaf\n");

	c->rst[0] = devm_gpiod_get(dev, "cam0-reset", GPIOD_OUT_LOW);
	if (IS_ERR(c->rst[0]))
		return dev_err_probe(dev, PTR_ERR(c->rst[0]), "cam0-reset\n");

	c->rst[1] = devm_gpiod_get(dev, "cam1-reset", GPIOD_OUT_LOW);
	if (IS_ERR(c->rst[1]))
		return dev_err_probe(dev, PTR_ERR(c->rst[1]), "cam1-reset\n");

	for (i = 0; i < NTG; i++) {
		snprintf(name, sizeof(name), "camtg%d", i + 1);
		c->tg[i] = devm_clk_get(dev, name);
		if (IS_ERR(c->tg[i]))
			return dev_err_probe(dev, PTR_ERR(c->tg[i]), "%s\n", name);
	}

	c->src = devm_clk_get_optional(dev, "src24");
	if (IS_ERR(c->src))
		return dev_err_probe(dev, PTR_ERR(c->src), "src24\n");

	platform_set_drvdata(pdev, c);

	c->dbg = debugfs_create_dir("yt11-cam", NULL);
	debugfs_create_file("power", 0200, c->dbg, c, &power_fops);
	debugfs_create_file("status", 0444, c->dbg, c, &status_fops);

	dev_info(dev, "ready (stage=%d): echo '<0|1> on|off' > /sys/kernel/debug/yt11-cam/power\n",
		 stage);
	return 0;
}

static void yt11_cam_remove(struct platform_device *pdev)
{
	struct yt11_cam *c = platform_get_drvdata(pdev);
	int i;

	debugfs_remove_recursive(c->dbg);

	mutex_lock(&c->lock);
	power_down_locked(c);
	for (i = 0; i < NSLOT; i++) {
		gpiod_set_value_cansleep(c->rst[i], 0);
		pinctrl_select_state(c->pc, c->mclk_off[i]);
	}
	if (c->vcamaf)
		gpiod_set_value_cansleep(c->vcamaf, 0);
	gpiod_set_value_cansleep(c->vcamd, 0);
	gpiod_set_value_cansleep(c->vcama, 0);
	mutex_unlock(&c->lock);
}

static const struct of_device_id yt11_cam_of_match[] = {
	{ .compatible = "lenovo,yt-j706f-cam-probe" },
	{ }
};
MODULE_DEVICE_TABLE(of, yt11_cam_of_match);

static struct platform_driver yt11_cam_driver = {
	.probe = yt11_cam_probe,
	.remove = yt11_cam_remove,
	.driver = {
		.name = "yt11-cam-probe",
		.of_match_table = yt11_cam_of_match,
	},
};
module_platform_driver(yt11_cam_driver);

MODULE_DESCRIPTION("Lenovo Yoga Tab 11 (YT-J706F) camera power-up probe (M0)");
MODULE_LICENSE("GPL");
