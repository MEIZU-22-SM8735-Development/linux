// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU hotplug cooling device: one per CPU, state 1 offlines it.
 *
 * This is the hard step of the vendor's emergency ladder.  Its weaker 118 C
 * step (qcom,cpu-pause) cannot be ported -- it only hides the CPU from the
 * WALT scheduler, which is not in this tree, so a copy would do nothing.
 *
 * Two things are not optional.  The offline runs from a work item, because the
 * governor calls in with the zone locked, and remove_cpu() sleeps.  And a
 * hotplug callback refuses to let the CPU return while the trip is still
 * crossed, or `echo 1 > online` would undo the mitigation.
 *
 * CPU0 is skipped: the kernel does not allow offlining it.
 */

#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

/* In service, or out of service.  There is nothing in between. */
#define CPU_HOTPLUG_LEVEL	1

struct cpu_hotplug_cdev {
	struct list_head		node;
	struct thermal_cooling_device	*cdev;
	struct device			*dev;
	struct work_struct		work;
	int				cpu;
	bool				requested;
	bool				offlined;
	char				name[THERMAL_NAME_LENGTH];
};

static DEFINE_MUTEX(cpu_hotplug_lock);
static LIST_HEAD(cpu_hotplug_list);
static enum cpuhp_state cpu_hotplug_hp_state;

/*
 * Called with the CPU hotplug lock held, on the CPU that is coming online.
 * Returning NOTIFY_BAD aborts the online transition, which is how a manual
 * `echo 1 > .../online` is rejected while the zone is still above its trip.
 */
static int cpu_hotplug_refuse_online(unsigned int cpu)
{
	struct cpu_hotplug_cdev *hc;
	int ret = 0;

	mutex_lock(&cpu_hotplug_lock);
	list_for_each_entry(hc, &cpu_hotplug_list, node) {
		if (hc->cpu != cpu || !hc->requested)
			continue;
		dev_dbg(hc->dev, "cpu%d: cooling still active, refusing to come online\n",
			cpu);
		ret = NOTIFY_BAD;
		break;
	}
	mutex_unlock(&cpu_hotplug_lock);

	return ret;
}

static int cpu_hotplug_get_max_state(struct thermal_cooling_device *cdev,
				     unsigned long *state)
{
	*state = CPU_HOTPLUG_LEVEL;

	return 0;
}

static int cpu_hotplug_get_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long *state)
{
	struct cpu_hotplug_cdev *hc = cdev->devdata;

	mutex_lock(&cpu_hotplug_lock);
	/*
	 * Report what the CPU is really doing, not what was asked for: an
	 * offline that failed (busy CPU, for instance) must be visible.
	 */
	*state = hc->offlined ? CPU_HOTPLUG_LEVEL : 0;
	mutex_unlock(&cpu_hotplug_lock);

	return 0;
}

static void cpu_hotplug_work(struct work_struct *work)
{
	struct cpu_hotplug_cdev *hc =
		container_of(work, struct cpu_hotplug_cdev, work);
	bool want;
	int ret;

	mutex_lock(&cpu_hotplug_lock);
	want = hc->requested;
	if (want == hc->offlined) {
		mutex_unlock(&cpu_hotplug_lock);
		return;
	}
	mutex_unlock(&cpu_hotplug_lock);

	if (want)
		ret = remove_cpu(hc->cpu);
	else
		ret = add_cpu(hc->cpu);
	if (ret) {
		dev_warn(hc->dev, "cpu%d: %s failed: %d\n", hc->cpu,
			 want ? "offline" : "online", ret);
		return;
	}

	mutex_lock(&cpu_hotplug_lock);
	hc->offlined = want;
	mutex_unlock(&cpu_hotplug_lock);

	dev_info(hc->dev, "cpu%d: %s\n", hc->cpu,
		 want ? "offlined for cooling" : "back online");
}

static int cpu_hotplug_set_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long state)
{
	struct cpu_hotplug_cdev *hc = cdev->devdata;
	bool want;

	if (state > CPU_HOTPLUG_LEVEL)
		return -EINVAL;

	want = state == CPU_HOTPLUG_LEVEL;

	mutex_lock(&cpu_hotplug_lock);
	if (hc->requested == want) {
		mutex_unlock(&cpu_hotplug_lock);
		return 0;
	}
	hc->requested = want;
	mutex_unlock(&cpu_hotplug_lock);

	queue_work(system_highpri_wq, &hc->work);

	return 0;
}

static const struct thermal_cooling_device_ops cpu_hotplug_ops = {
	.get_max_state	= cpu_hotplug_get_max_state,
	.get_cur_state	= cpu_hotplug_get_cur_state,
	.set_cur_state	= cpu_hotplug_set_cur_state,
};

static int cpu_hotplug_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cpu_hotplug_cdev *hc;
	const char *alias;
	int ret;

	for_each_available_child_of_node_scoped(np, child) {
		struct device_node *cpu_np;
		int cpu;

		cpu_np = of_parse_phandle(child, "qcom,cpu", 0);
		if (!cpu_np) {
			ret = dev_err_probe(dev, -EINVAL,
					    "%pOF: missing qcom,cpu\n", child);
			goto err;
		}

		cpu = of_cpu_node_to_id(cpu_np);
		of_node_put(cpu_np);
		if (cpu < 0) {
			dev_err(dev, "%pOF: not a CPU node\n", child);
			continue;
		}
		if (cpu == 0) {
			dev_warn(dev, "%pOF: cpu0 cannot be offlined, skipping\n",
				 child);
			continue;
		}

		hc = devm_kzalloc(dev, sizeof(*hc), GFP_KERNEL);
		if (!hc) {
			ret = -ENOMEM;
			goto err;
		}

		if (of_property_read_string(child, "qcom,cdev-alias", &alias))
			snprintf(hc->name, sizeof(hc->name), "cpu%d-hotplug", cpu);
		else
			strscpy(hc->name, alias, sizeof(hc->name));

		hc->dev = dev;
		hc->cpu = cpu;
		INIT_WORK(&hc->work, cpu_hotplug_work);

		hc->cdev = devm_thermal_of_child_cooling_device_register(dev,
									 child, hc->name,
									 hc, &cpu_hotplug_ops);
		if (IS_ERR(hc->cdev)) {
			ret = dev_err_probe(dev, PTR_ERR(hc->cdev),
					    "failed to register %s\n", hc->name);
			goto err;
		}

		mutex_lock(&cpu_hotplug_lock);
		list_add(&hc->node, &cpu_hotplug_list);
		mutex_unlock(&cpu_hotplug_lock);
	}

	/*
	 * CPUHP_AP_ONLINE_DYN puts us late enough that the CPU registers and
	 * caches are already up, and early enough to veto the transition.
	 */
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
					"thermal/cpu-hotplug:online",
					cpu_hotplug_refuse_online, NULL);
	if (ret < 0) {
		dev_err(dev, "failed to register hotplug state: %d\n", ret);
		goto err;
	}
	cpu_hotplug_hp_state = ret;

	return 0;

err:
	while (!list_empty(&cpu_hotplug_list)) {
		struct cpu_hotplug_cdev *pos =
			list_first_entry(&cpu_hotplug_list,
					 struct cpu_hotplug_cdev, node);

		list_del(&pos->node);
	}
	if (cpu_hotplug_hp_state) {
		cpuhp_remove_state_nocalls(cpu_hotplug_hp_state);
		cpu_hotplug_hp_state = 0;
	}

	return ret;
}

static void cpu_hotplug_remove(struct platform_device *pdev)
{
	struct cpu_hotplug_cdev *hc, *tmp;
	LIST_HEAD(gone);

	if (cpu_hotplug_hp_state) {
		cpuhp_remove_state_nocalls(cpu_hotplug_hp_state);
		cpu_hotplug_hp_state = 0;
	}

	/*
	 * Unlink under the lock, then do the rest outside it: add_cpu() runs
	 * the hotplug callbacks, and ours takes cpu_hotplug_lock.
	 */
	mutex_lock(&cpu_hotplug_lock);
	list_splice_init(&cpu_hotplug_list, &gone);
	mutex_unlock(&cpu_hotplug_lock);

	list_for_each_entry_safe(hc, tmp, &gone, node) {
		cancel_work_sync(&hc->work);
		list_del(&hc->node);
		if (!hc->offlined)
			continue;
		hc->offlined = false;
		hc->requested = false;
		if (add_cpu(hc->cpu))
			dev_err(hc->dev, "cpu%d: online failed\n", hc->cpu);
	}
}

static const struct of_device_id cpu_hotplug_match[] = {
	{ .compatible = "qcom,cpu-hotplug" },
	{}
};
MODULE_DEVICE_TABLE(of, cpu_hotplug_match);

static struct platform_driver cpu_hotplug_driver = {
	.probe		= cpu_hotplug_probe,
	.remove		= cpu_hotplug_remove,
	.driver		= {
		.name		= "cpu-hotplug-cdev",
		.of_match_table	= cpu_hotplug_match,
	},
};
module_platform_driver(cpu_hotplug_driver);

MODULE_DESCRIPTION("CPU hotplug thermal cooling device");
MODULE_LICENSE("GPL");
