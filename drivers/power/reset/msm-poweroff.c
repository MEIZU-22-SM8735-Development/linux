// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/regmap.h>

/*
 * Releasing PS_HOLD is how a Qualcomm board turns itself off: the PMIC holds
 * the rails up while the SoC drives this line, and drops them when it goes low.
 * What the PMIC does next -- stay off, or start again -- is decided by its PON
 * block, and on PMICs whose PS_HOLD sequencing lives in the PBS (PMK8550 among
 * them) the reset type has to be programmed first.  Qualcomm's own driver says
 * why: without a configured type the board "hangs on performing reset such as
 * hard reset", and a warm reset that was not fully configured is treated as a
 * shutdown instead.
 *
 * So a restart that is meant to look like a power cycle is two steps: program
 * the PBS for a hard reset while we can still sleep, then let go of PS_HOLD.
 * That is what makes the panel come back afterwards -- only a real drop of the
 * rails resets it (see AGENTS.md rule 24).
 */
#define PON_PBS_PS_HOLD_SW_CTL		0x52
#define PON_PBS_PS_HOLD_RESET_CTL2	0x53
#define PON_PBS_S2_RESET_EN		BIT(7)
#define PON_PS_HOLD_TYPE_HARD_RESET	0x07

/* Give the PMIC time to run its own reset sequence before falling back. */
#define PON_PS_HOLD_FALLBACK_MS		200

struct qcom_pshold {
	struct device *dev;
	void __iomem *base;
	struct regmap *pon_regmap;
	u32 pbs_base;
};

static int qcom_pshold_pon_set_type(struct qcom_pshold *ps, u8 type)
{
	unsigned int ctl2 = ps->pbs_base + PON_PBS_PS_HOLD_RESET_CTL2;
	unsigned int sw = ps->pbs_base + PON_PBS_PS_HOLD_SW_CTL;
	int ret;

	/*
	 * The PBS latches the type on the rising edge of S2_RESET_EN, so the
	 * order is: clear the latch, set the type, re-arm it.  The delays are
	 * the ones in Qualcomm's implementation.
	 */
	ret = regmap_update_bits(ps->pon_regmap, ctl2, PON_PBS_S2_RESET_EN, 0);
	if (ret)
		return ret;
	udelay(100);

	ret = regmap_write(ps->pon_regmap, sw, type);
	if (ret)
		return ret;
	udelay(300);

	ret = regmap_update_bits(ps->pon_regmap, ctl2, PON_PBS_S2_RESET_EN,
				 PON_PBS_S2_RESET_EN);
	if (ret)
		return ret;
	udelay(100);

	return 0;
}

/*
 * Stage one: still able to sleep.  Failure is only logged -- the reset below
 * still runs, and whatever the boot firmware left in the PBS is then what
 * happens.
 */
static int qcom_pshold_prepare(struct sys_off_data *data)
{
	struct qcom_pshold *ps = data->cb_data;

	if (qcom_pshold_pon_set_type(ps, PON_PS_HOLD_TYPE_HARD_RESET))
		dev_err(ps->dev,
			"PON PBS hard-reset type not set; PS_HOLD may only power off\n");

	return NOTIFY_DONE;
}

/* Stage two: interrupts are off and every other CPU is stopped. */
static int qcom_pshold_restart(struct sys_off_data *data)
{
	struct qcom_pshold *ps = data->cb_data;

	writel(0, ps->base);
	mdelay(PON_PS_HOLD_FALLBACK_MS);

	pr_emerg("pshold: PS_HOLD released but the board is still here, falling back\n");
	return NOTIFY_DONE;
}

static int msm_restart_probe(struct platform_device *pdev)
{
	struct qcom_pshold *ps;
	struct device_node *pon;
	int ret;

	ps = devm_kzalloc(&pdev->dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;
	ps->dev = &pdev->dev;

	ps->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ps->base))
		return PTR_ERR(ps->base);

	/*
	 * The PON is where the PBS lives, and its parent PMIC owns the SPMI
	 * regmap.  Without both this driver keeps its historical behaviour:
	 * clear PS_HOLD and let the board power off.
	 */
	pon = of_parse_phandle(pdev->dev.of_node, "qcom,pon", 0);
	if (pon) {
		struct platform_device *pon_pdev;
		u32 pbs[2];

		if (!of_property_read_u32_array(pon, "reg", pbs, 2))
			ps->pbs_base = pbs[1];

		/*
		 * The PON is a platform device whose parent is the SPMI PMIC
		 * device, and that parent owns the regmap.  Looking the parent
		 * up with of_find_device_by_node() does not work: it searches
		 * the platform bus only, and an SPMI device is not on it --
		 * which is why this driver sat in the deferred-probe list
		 * forever on the first attempt.
		 */
		pon_pdev = of_find_device_by_node(pon);
		if (pon_pdev) {
			struct device *pmic = pon_pdev->dev.parent;

			if (pmic)
				ps->pon_regmap = dev_get_regmap(pmic, NULL);
			put_device(&pon_pdev->dev);
		}
		of_node_put(pon);

		/*
		 * Defer rather than silently fall back to the power-off-only
		 * path: releasing PS_HOLD without a configured PBS type is the
		 * case Qualcomm's driver warns "hangs on performing reset".
		 */
		if (!ps->pon_regmap) {
			dev_info(&pdev->dev, "PMIC regmap not there yet; deferring\n");
			return -EPROBE_DEFER;
		}
	}

	if (!ps->pon_regmap) {
		devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART,
					      128, qcom_pshold_restart, ps);
		devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF,
					      SYS_OFF_PRIO_DEFAULT,
					      qcom_pshold_restart, ps);
		return 0;
	}

	/*
	 * Priority has to beat PSCI's 129, or that handler warm-resets the SoC
	 * before this ever runs.
	 *
	 * POWER_OFF is deliberately not registered: the PBS type is sticky, and
	 * programming it for a hard reset would turn a "poweroff" into a reboot.
	 */
	ret = devm_register_sys_off_handler(&pdev->dev,
			SYS_OFF_MODE_RESTART_PREPARE, SYS_OFF_PRIO_DEFAULT,
			qcom_pshold_prepare, ps);
	if (ret)
		return ret;

	return devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART,
					     SYS_OFF_PRIO_HIGH,
					     qcom_pshold_restart, ps);
}

static const struct of_device_id of_msm_restart_match[] = {
	{ .compatible = "qcom,pshold", },
	{},
};
MODULE_DEVICE_TABLE(of, of_msm_restart_match);

static struct platform_driver msm_restart_driver = {
	.probe = msm_restart_probe,
	.driver = {
		.name = "msm-restart",
		.of_match_table = of_match_ptr(of_msm_restart_match),
	},
};
builtin_platform_driver(msm_restart_driver);
