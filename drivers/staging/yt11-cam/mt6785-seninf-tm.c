// SPDX-License-Identifier: GPL-2.0
/*
 * mt6785-seninf-tm: MT6785 SENINF test-model bring-up module.
 *
 * Ported from begonia's seninf/seninf-tm.c. Drives the SoC SENINF0 test-pattern
 * generator through MUX0 and CAM_MUX0 (without a sensor or DMA) so clocks,
 * the CAM power domain, and the SENINF/MUX/CAM_MUX pipeline can be verified.
 *
 * MT6785 SENINF matches upstream MediaTek ISP7 phy_2_0, except the test-model
 * TG block sits at +0x600 instead of +0xf00 (verified on begonia HAL):
 *   SENINF_TOP     +0x000
 *   SENINF_CAMMUX  +0x400   (EN +0x10, CHK_EN +0x18, IRQ +0xa8, MUX0_CHK +0x108)
 *   SENINF_CTRL(n) +0x200 + 0x1000*n  (CTRL +0x0, TESTMDL_CTRL +0x20)
 *   SENINF_TG(n)   +0x600 + 0x1000*n  (TM_CTL +0x8, TM_SIZE +0xc, TM_CLK +0x10, TM_DUM +0x18)
 *   SENINF_CSI2(n) +0xa00 + 0x1000*n
 *   SENINF_MUX(n)  +0xd00 + 0x1000*n  (CTRL0 +0x0, CTRL1 +0x4, IRQ +0x18, SIZE +0x30,
 *                                      PIX_CNT +0xa4, MON_CTRL +0xa8, H_VALID +0xb0, V_VALID +0xb8)
 *
 * debugfs interface under /sys/kernel/debug/seninf-tm/:
 *   power   write "1"/"0" to attach/power CAM domain and enable SENINF clocks
 *   start   write anything while powered to start the SENINF0 -> MUX0 -> CAM_MUX0 test model
 *   dump    read SENINF_TOP / SENINF0 / TG0 / MUX0 / CAM_MUX status registers
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>

#define SENINF_TOP		0x000
#define SENINF_CTRL(n)		(0x200 + 0x1000 * (n))
#define SENINF_CAMMUX		0x400
#define SENINF_TG(n)		(0x600 + 0x1000 * (n))
#define SENINF_MUX(n)		(0xd00 + 0x1000 * (n))

static unsigned int tm_size = 0x10001f00;
module_param(tm_size, uint, 0644);
MODULE_PARM_DESC(tm_size, "SENINF TG0 TM_SIZE register value (default 0x10001f00 = 4096x7936)");

static unsigned int tm_ctl = 0x00300161;
module_param(tm_ctl, uint, 0644);
MODULE_PARM_DESC(tm_ctl, "SENINF TG0 TM_CTL register value (default 0x00300161 = enable + pattern 0x16)");

static unsigned int tm_clk = 7;
module_param(tm_clk, uint, 0644);
MODULE_PARM_DESC(tm_clk, "SENINF TG0 TM_CLK divider/rate register value (default 7)");

enum stm_clk_idx {
	CLK_CAM_MAIN,
	CLK_CAM_GALS,
	CLK_CAM_TG,
	CLK_CAM_SENINF,
	CLK_TOP_SENINF0,
	CLK_TOP_SENINF1,
	CLK_TOP_SENINF2,
	CLK_STM_NR,
};

static const char *const clk_names[CLK_STM_NR] = {
	[CLK_CAM_MAIN]		= "cam-main",
	[CLK_CAM_GALS]		= "cam-gals",
	[CLK_CAM_TG]		= "cam-tg",
	[CLK_CAM_SENINF]	= "cam-seninf",
	[CLK_TOP_SENINF0]	= "top-seninf0",
	[CLK_TOP_SENINF1]	= "top-seninf1",
	[CLK_TOP_SENINF2]	= "top-seninf2",
};

struct stm {
	struct device *dev;
	void __iomem *base;
	struct clk_bulk_data clks[CLK_STM_NR];
	struct device *pd;
	bool on;
	struct mutex lock;
	struct dentry *dbg;
};

static u32 stm_rd(struct stm *s, u32 off)
{
	return readl(s->base + off);
}

static void stm_wr(struct stm *s, u32 off, u32 val)
{
	writel(val, s->base + off);
}

static int stm_power(struct stm *s, bool on)
{
	int ret;

	if (on == s->on)
		return 0;

	if (on) {
		s->pd = dev_pm_domain_attach_by_name(s->dev, "cam");
		if (IS_ERR_OR_NULL(s->pd)) {
			ret = s->pd ? PTR_ERR(s->pd) : -ENODEV;
			s->pd = NULL;
			dev_err(s->dev, "attaching cam power domain failed: %d\n", ret);
			return ret;
		}
		ret = pm_runtime_resume_and_get(s->pd);
		if (ret < 0) {
			dev_err(s->dev, "resuming cam power domain failed: %d\n", ret);
			goto detach;
		}
		ret = clk_bulk_prepare_enable(CLK_STM_NR, s->clks);
		if (ret) {
			dev_err(s->dev, "enabling clocks failed: %d\n", ret);
			goto put;
		}
		s->on = true;
		dev_info(s->dev, "power on: SENINF_TOP_CTRL=0x%08x, top-seninf0=%lu Hz\n",
			 stm_rd(s, SENINF_TOP),
			 clk_get_rate(s->clks[CLK_TOP_SENINF0].clk));
		return 0;
	}

	clk_bulk_disable_unprepare(CLK_STM_NR, s->clks);
	ret = 0;
put:
	pm_runtime_put_sync(s->pd);
detach:
	dev_pm_domain_detach(s->pd, true);
	s->pd = NULL;
	s->on = false;
	return ret;
}

/* Vendor setTestModel sequence for SENINF0 -> MUX0 -> CAM_MUX0, 1-pixel mode */
static void stm_start(struct stm *s)
{
	stm_wr(s, SENINF_CAMMUX + 0x10, stm_rd(s, SENINF_CAMMUX + 0x10) | 0x1f);
	stm_wr(s, SENINF_CAMMUX + 0x18, stm_rd(s, SENINF_CAMMUX + 0x18) | 0x1);
	stm_wr(s, SENINF_MUX(0) + 0x04, 0x001f0001);
	stm_wr(s, SENINF_MUX(0) + 0x00, 0x1);
	stm_wr(s, SENINF_MUX(0) + 0xa8, 0x1);
	stm_wr(s, SENINF_CTRL(0) + 0x20, 0x1);
	stm_wr(s, SENINF_CTRL(0) + 0x00, 0x1);
	stm_wr(s, SENINF_TG(0) + 0x0c, tm_size);
	stm_wr(s, SENINF_TG(0) + 0x10, tm_clk);
	stm_wr(s, SENINF_TG(0) + 0x08, tm_ctl);
	dev_info(s->dev, "test model started: size=0x%08x clk=%u ctl=0x%08x\n",
		 tm_size, tm_clk, tm_ctl);
}

static int dump_show(struct seq_file *m, void *v)
{
	struct stm *s = m->private;
	int i;

	mutex_lock(&s->lock);
	if (!s->on) {
		seq_puts(m, "power is off (echo 1 > power)\n");
		goto out;
	}

	seq_printf(m, "TOP: ctrl 0x%08x mux0 0x%08x mux1 0x%08x\n",
		   stm_rd(s, SENINF_TOP),
		   stm_rd(s, SENINF_TOP + 0x10),
		   stm_rd(s, SENINF_TOP + 0x14));
	seq_printf(m, "SENINF0: ctrl 0x%08x testmdl 0x%08x | TG: ctl 0x%08x size 0x%08x clk 0x%08x dum 0x%08x\n",
		   stm_rd(s, SENINF_CTRL(0)),
		   stm_rd(s, SENINF_CTRL(0) + 0x20),
		   stm_rd(s, SENINF_TG(0) + 0x08),
		   stm_rd(s, SENINF_TG(0) + 0x0c),
		   stm_rd(s, SENINF_TG(0) + 0x10),
		   stm_rd(s, SENINF_TG(0) + 0x18));

	for (i = 0; i < 2; i++) {
		u32 mx = SENINF_MUX(0);

		seq_printf(m, "MUX0 #%d: ctrl0 0x%08x ctrl1 0x%08x irq 0x%08x img_size 0x%08x size 0x%08x "
			   "err_size 0x%08x fifo 0x%08x pix_cnt 0x%08x mon H %u V %u (blank %u/%u)\n",
			   i, stm_rd(s, mx), stm_rd(s, mx + 0x04), stm_rd(s, mx + 0x18),
			   stm_rd(s, mx + 0x20), stm_rd(s, mx + 0x30), stm_rd(s, mx + 0x34),
			   stm_rd(s, mx + 0x40), stm_rd(s, mx + 0xa4), stm_rd(s, mx + 0xb0),
			   stm_rd(s, mx + 0xb8), stm_rd(s, mx + 0xb4), stm_rd(s, mx + 0xbc));
		if (i == 0)
			msleep(100);
	}

	seq_printf(m, "CAM_MUX: ctrl0 0x%08x en 0x%08x chk_en 0x%08x irq 0x%08x | mux0 chk_res 0x%08x err 0x%08x\n",
		   stm_rd(s, SENINF_CAMMUX),
		   stm_rd(s, SENINF_CAMMUX + 0x10),
		   stm_rd(s, SENINF_CAMMUX + 0x18),
		   stm_rd(s, SENINF_CAMMUX + 0xa8),
		   stm_rd(s, SENINF_CAMMUX + 0x108),
		   stm_rd(s, SENINF_CAMMUX + 0x10c));
out:
	mutex_unlock(&s->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dump);

static ssize_t power_write(struct file *f, const char __user *ub, size_t len, loff_t *off)
{
	struct stm *s = file_inode(f)->i_private;
	bool on;
	int ret;

	ret = kstrtobool_from_user(ub, len, &on);
	if (ret)
		return ret;

	mutex_lock(&s->lock);
	ret = stm_power(s, on);
	mutex_unlock(&s->lock);
	return ret ? ret : len;
}

static ssize_t start_write(struct file *f, const char __user *ub, size_t len, loff_t *off)
{
	struct stm *s = file_inode(f)->i_private;
	int ret = 0;

	mutex_lock(&s->lock);
	if (s->on)
		stm_start(s);
	else
		ret = -EIO;
	mutex_unlock(&s->lock);
	return ret ? ret : len;
}

static const struct file_operations power_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = power_write,
	.llseek = noop_llseek,
};

static const struct file_operations start_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = start_write,
	.llseek = noop_llseek,
};

static int stm_probe(struct platform_device *pdev)
{
	struct stm *s;
	int i, ret;

	s = devm_kzalloc(&pdev->dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = &pdev->dev;
	mutex_init(&s->lock);

	s->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(s->base))
		return PTR_ERR(s->base);

	for (i = 0; i < CLK_STM_NR; i++)
		s->clks[i].id = clk_names[i];

	ret = devm_clk_bulk_get(s->dev, CLK_STM_NR, s->clks);
	if (ret)
		return dev_err_probe(s->dev, ret, "clocks\n");

	platform_set_drvdata(pdev, s);

	s->dbg = debugfs_create_dir("seninf-tm", NULL);
	debugfs_create_file("power", 0200, s->dbg, s, &power_fops);
	debugfs_create_file("start", 0200, s->dbg, s, &start_fops);
	debugfs_create_file("dump", 0444, s->dbg, s, &dump_fops);

	dev_info(s->dev, "ready: echo 1 > /sys/kernel/debug/seninf-tm/power; echo > start; cat dump\n");
	return 0;
}

static void stm_remove(struct platform_device *pdev)
{
	struct stm *s = platform_get_drvdata(pdev);

	debugfs_remove_recursive(s->dbg);

	mutex_lock(&s->lock);
	if (s->on) {
		stm_wr(s, SENINF_TG(0) + 0x08, 0);
		stm_wr(s, SENINF_CTRL(0), 0);
		stm_wr(s, SENINF_MUX(0), 0);
		stm_power(s, false);
	}
	mutex_unlock(&s->lock);
}

static const struct of_device_id stm_of[] = {
	{ .compatible = "mediatek,mt6785-seninf-tm" },
	{ }
};
MODULE_DEVICE_TABLE(of, stm_of);

static struct platform_driver stm_drv = {
	.probe = stm_probe,
	.remove = stm_remove,
	.driver = {
		.name = "mt6785-seninf-tm",
		.of_match_table = stm_of,
	},
};
module_platform_driver(stm_drv);

MODULE_DESCRIPTION("MT6785 SENINF test-model bring-up driver");
MODULE_LICENSE("GPL");
