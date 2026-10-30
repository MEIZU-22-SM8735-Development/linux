/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_MEIZU_M2582_H
#define _LINUX_MEIZU_M2582_H

#include <linux/types.h>

/*
 * Board bring-up switches, from the kernel command line.  One name per
 * behaviour, so an image can carry several and still be tested one variable at
 * a time:
 *
 *	ltpo		panel adaptive refresh (idle floor below 120 Hz)
 *	gpuifpc		GMU idle power collapse for the Adreno on this board
 *	gpubw		GPU DDR/AB bandwidth vote from the board's own table
 *
 * Opt-in: an absent list, or a list without the name, leaves the behaviour at
 * the driver's shipped default (the configuration last verified on the device).
 *
 * A behaviour that is not an experiment but the factory's own sequence, and
 * without which the board does not work, uses the mirror-image helper instead:
 *
 *	pps0a		panel DSC picture parameter set, sent after the power-on set
 *			(m2582.disable=pps0a to A/B)
 *	updsc		DSC full-frame topology bits (on by default;
 *			m2582.disable=updsc to A/B)
 *
 * Everything else is opt-in, including diagnostic arms that look like they
 * should be opt-out (flbo1, flbo15, dtsnap): they are experiments, and an
 * experiment that is on unless you say otherwise is how a board ends up on a
 * configuration nobody chose.
 *
 * m2582_feature_opt_out() is satisfied by m2582.disable=<name> and by nothing
 * else -- in particular not by m2582.features=, and not by m2582.features=all,
 * so an "only these features" list can never switch such a behaviour off by
 * accident.
 */
#if IS_ENABLED(CONFIG_ARM64_MEIZU_M2582_FEATURES)
bool m2582_feature_opt_in(const char *name);
bool m2582_feature_opt_out(const char *name);
#else
static inline bool m2582_feature_opt_in(const char *name)
{
	return false;
}

static inline bool m2582_feature_opt_out(const char *name)
{
	return false;
}
#endif

#endif /* _LINUX_MEIZU_M2582_H */
