// SPDX-License-Identifier: GPL-2.0-only
/*
 * M2582: one boot-time switch per bring-up subsystem.
 *
 * This board is adapted one peripheral at a time, and the failure mode of a
 * half-adapted peripheral is not a failed probe: it is a wrong serial-engine
 * protocol (touch, Bluetooth), a firmware the kernel cannot reach (ADSP), or a
 * bus transaction that never completes (the GPU GMU, the TCSR clock
 * controller).  A machine that does not finish booting cannot be bisected from
 * the inside, and reflashing a kernel per subsystem to find that out is slow
 * enough that nobody does it properly.
 *
 * So each device node that belongs to one of these subsystems carries
 *
 *	m2582,feature = "<name>";
 *
 * and this runs before the device tree is populated, turning the ones that are
 * not wanted off:
 *
 *	m2582.features=adsp,touch	only these are probed
 *	m2582.features=			none of them are (a safe boot)
 *	m2582.disable=wifi,bt		all but these
 *
 * "off" is implemented by writing status = "disabled", which is the device
 * tree's own mechanism: of_platform_default_populate() then simply never
 * creates the device, so no driver's probe runs and no register is touched.
 * A feature whose node the board already leaves disabled stays disabled either
 * way -- the gate only ever turns things off, never on.
 *
 * The decision is logged one line per feature and can be read back from
 * /proc/m2582_features, because the interesting case (a subsystem that is
 * silently absent) looks exactly like a subsystem that was gated off.
 *
 * This is bring-up scaffolding, not a binding: once every subsystem is
 * adapted, the property, this file and its Kconfig symbol all go away.
 */
#ifdef M2582_FEATURES_HOST_TEST
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define __init
#else
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/meizu-m2582.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/string_choices.h>
#endif

#define M2582_FEATURE_PROP	"m2582,feature"
#define M2582_MAX_FEATURES	32

static const char *m2582_features;
static const char *m2582_disable;

/*
 * Comma-separated list membership, exact word.  The list is the command line's
 * own string, which is never modified, so this walks it instead of tokenising.
 */
static bool m2582_list_has(const char *list, const char *name)
{
	size_t len = strlen(name);
	const char *p = list;

	if (!list)
		return false;

	while (*p) {
		const char *end = p;

		while (*end && *end != ',')
			end++;

		if ((size_t)(end - p) == len && !strncmp(p, name, len))
			return true;

		p = *end ? end + 1 : end;
	}

	return false;
}

/*
 * An allowlist wins over a denylist when both are given: "only these" is the
 * stricter request, and a boot that silently probes something the caller asked
 * to be left alone is worse than one that probes too little.  With neither
 * given, everything is on -- the gate must not change how the board boots
 * unless it was asked to.
 */
static bool __init m2582_feature_on(const char *name)
{
	if (m2582_features)
		return m2582_list_has(m2582_features, name) ||
		       m2582_list_has(m2582_features, "all");

	if (m2582_disable)
		return !m2582_list_has(m2582_disable, name) &&
		       !m2582_list_has(m2582_disable, "all");

	return true;
}

/*
 * The opt-in helper: a deny list is used to keep board bring-up quiet, and
 * letting it enable a new behaviour as a side effect of "everything except
 * wifi" is how a feature ends up on an image that was not meant to test it.
 * Kept outside the host-test guard so that this polarity is exercised by
 * tools/meizu22/test_m2582_features_gate.c as well.
 */
bool m2582_feature_opt_in(const char *name)
{
	if (!m2582_features)
		return false;

	return m2582_list_has(m2582_features, name) ||
	       m2582_list_has(m2582_features, "all");
}

/*
 * The mirror image, for a behaviour that is not new: it is what the factory
 * stack does and the board needs it, so it is on unless this image says
 * otherwise.  Only the deny list can switch one off, so "m2582.features=touch"
 * cannot silently drop a fix.  The runtime test for the reverse polarity lives
 * in tools/meizu22/test_m2582_features_gate.c with the opt-in one.
 */
bool m2582_feature_opt_out(const char *name)
{
	if (!name)
		return false;

	if (!m2582_disable)
		return false;

	/*
	 * Deliberately no "all" wildcard: m2582.disable=all is the bring-up
	 * "gate every device tree node off" request, and it must not also change
	 * how the panel is driven -- a safe boot that cannot light the screen is
	 * not a safe boot.
	 */
	return m2582_list_has(m2582_disable, name);
}

#ifndef M2582_FEATURES_HOST_TEST

/*
 * The command line lives in __initdata and is gone long before a panel is
 * prepared or a GPU is powered up, so the opt-in list is copied once, while
 * init memory is still valid.
 */
#define M2582_FEATURE_LIST_MAX	512
static char m2582_features_copy[M2582_FEATURE_LIST_MAX] __ro_after_init;
static char m2582_disable_copy[M2582_FEATURE_LIST_MAX] __ro_after_init;

EXPORT_SYMBOL_GPL(m2582_feature_opt_in);
EXPORT_SYMBOL_GPL(m2582_feature_opt_out);

static int __init m2582_features_setup(char *str)
{
	strscpy(m2582_features_copy, str, sizeof(m2582_features_copy));
	m2582_features = m2582_features_copy;
	return 1;
}
__setup("m2582.features=", m2582_features_setup);

static int __init m2582_disable_setup(char *str)
{
	strscpy(m2582_disable_copy, str, sizeof(m2582_disable_copy));
	m2582_disable = m2582_disable_copy;
	return 1;
}
__setup("m2582.disable=", m2582_disable_setup);

struct m2582_feature_state {
	const char *name;	/* points into the device tree's own property */
	bool on;
};

static struct m2582_feature_state m2582_states[M2582_MAX_FEATURES];
static int m2582_state_count;

/*
 * Replace the node's status with "disabled".  of_update_property() takes
 * ownership of the new property; the one it replaces is moved to the node's
 * dead-properties list rather than freed, so nothing here has to keep a pointer
 * to it.  This runs before anything can have taken a reference to the old one.
 */
static int __init m2582_gate_disable(struct device_node *np)
{
	struct property *prop;
	char *name, *value;

	name = kstrdup("status", GFP_KERNEL);
	value = kstrdup("disabled", GFP_KERNEL);
	prop = kzalloc_obj(*prop);
	if (!name || !value || !prop) {
		kfree(name);
		kfree(value);
		kfree(prop);
		return -ENOMEM;
	}

	prop->name = name;
	prop->value = value;
	prop->length = strlen(value) + 1;

	if (of_update_property(np, prop)) {
		kfree(prop->name);
		kfree(prop->value);
		kfree(prop);
		return -EINVAL;
	}

	return 0;
}

static void __init m2582_gate_node(struct device_node *np)
{
	const char *feature;
	bool on;

	if (of_property_read_string(np, M2582_FEATURE_PROP, &feature))
		return;

	on = m2582_feature_on(feature);

	if (m2582_state_count < M2582_MAX_FEATURES) {
		m2582_states[m2582_state_count].name = feature;
		m2582_states[m2582_state_count].on = on;
		m2582_state_count++;
	}

	if (on) {
		pr_info("m2582: feature %s on (%pOFn)\n", feature, np);
		return;
	}

	if (m2582_gate_disable(np)) {
		pr_err("m2582: feature %s could not be turned off (%pOFn)\n",
		       feature, np);
		return;
	}

	pr_info("m2582: feature %s off (%pOFn)\n", feature, np);
}

static void __init m2582_gate_walk(struct device_node *np)
{
	struct device_node *child;

	for_each_child_of_node(np, child) {
		m2582_gate_node(child);
		m2582_gate_walk(child);
	}
}

static int m2582_features_show(struct seq_file *m, void *v)
{
	int i, j;

	seq_printf(m, "allow=%s deny=%s\n",
		   m2582_features ?: "-", m2582_disable ?: "-");

	/*
	 * One line per feature, not per node: a subsystem owns several nodes
	 * (the ADSP is four), and every node of one name gets the same answer,
	 * so repeating it says nothing and makes the file hard to read -- and
	 * hard to grep, which is how the on-device checker reads it.
	 */
	for (i = 0; i < m2582_state_count; i++) {
		for (j = 0; j < i; j++)
			if (!strcmp(m2582_states[j].name, m2582_states[i].name))
				break;
		if (j != i)
			continue;

		seq_printf(m, "%-12s %s\n", m2582_states[i].name,
			   str_on_off(m2582_states[i].on));
	}

	return 0;
}

/*
 * Before arch_initcall_sync(of_platform_default_populate_init), which is what
 * creates the platform devices from the tree: a node disabled here is a device
 * that never exists, rather than one that probes and fails.
 */
static int __init m2582_features_init(void)
{
	if (!m2582_features && !m2582_disable)
		return 0;

	m2582_gate_walk(of_root);
	proc_create_single("m2582_features", 0444, NULL, m2582_features_show);

	return 0;
}
postcore_initcall(m2582_features_init);

#endif /* !M2582_FEATURES_HOST_TEST */
