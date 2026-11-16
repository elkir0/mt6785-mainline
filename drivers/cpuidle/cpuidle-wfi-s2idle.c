// SPDX-License-Identifier: GPL-2.0-only
/*
 * WFI-only cpuidle driver with a suspend-to-idle (s2idle) state.
 *
 * On platforms whose firmware hangs on PSCI CPU_SUSPEND idle states, no
 * cpuidle driver is registered and s2idle cannot freeze the tick and
 * timekeeping. This driver provides a WFI runtime state and an s2idle state
 * so s2idle can freeze the tick while CPUs still only execute WFI.
 */

#define pr_fmt(fmt) "wfi_s2idle: " fmt

#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/printk.h>

#include <asm/proc-fns.h>

static __cpuidle int wfi_s2idle_enter(struct cpuidle_device *dev,
				      struct cpuidle_driver *drv, int index)
{
	cpu_do_idle();

	return index;
}

static struct cpuidle_driver wfi_s2idle_driver = {
	.name = "wfi_s2idle",
	.owner = THIS_MODULE,
	.states = {
		{
			.enter			= wfi_s2idle_enter,
			.exit_latency		= 1,
			.target_residency	= 1,
			.power_usage		= UINT_MAX,
			.name			= "WFI",
			.desc			= "ARM WFI",
		},
		{
			.enter			= wfi_s2idle_enter,
			.enter_s2idle		= wfi_s2idle_enter,
			.exit_latency		= 1,
			.target_residency	= UINT_MAX / 2,
			.power_usage		= UINT_MAX,
			.name			= "s2idle",
			.desc			= "WFI, tick frozen (s2idle only)",
		},
	},
	.state_count = 2,
};

static int __init wfi_s2idle_init(void)
{
	int ret;

	if (!of_machine_is_compatible("mediatek,mt6785"))
		return 0;

	if (cpuidle_get_cpu_driver(per_cpu(cpuidle_devices, 0)) ||
	    cpuidle_get_driver())
		return 0;

	ret = cpuidle_register(&wfi_s2idle_driver, NULL);
	if (ret) {
		pr_err("failed to register cpuidle driver: %d\n", ret);
		return ret;
	}

	pr_info("WFI cpuidle driver with an s2idle state\n");
	return 0;
}
late_initcall(wfi_s2idle_init);
