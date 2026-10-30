// SPDX-License-Identifier: GPL-2.0-only
/*
 * M2582: freeze the machine at a chosen moment so its log can be photographed.
 *
 * Why this exists
 * ---------------
 * On this board the usual ways to see a boot that dies do not work:
 *
 *   - pstore/ramoops records nothing.  A deliberate panic leaves
 *     /sys/fs/pstore empty on the next boot, and a control test with
 *     /dev/pmsg0 reached the same conclusion: that region is wiped between
 *     boots.
 *   - The cross-reboot marker in the same area does not survive either, for
 *     the same reason.
 *   - The USB gadget console only exists once userspace runs, which is much
 *     later than the failures that matter (a driver probe at 1-2 s).
 *   - The panel console does print, but a reset takes the panel with it: the
 *     last lines are visible for milliseconds and then the screen is black.
 *
 * What is left is the panel plus a camera -- if the machine can be made to
 * *stop* instead of resetting.  That is all this file does: it prints where it
 * stopped, gives the console a moment to reach the panel, stops the other CPUs
 * and spins.  The screen then holds the log for as long as anyone needs.
 *
 * Usage
 * -----
 *	meizu.halt_ms=1500		freeze at boot time 1500 ms -- the same clock
 *				dmesg timestamps use, so the value can be read
 *				straight off the log
 *	meizu.halt_at=bt_enable		freeze at the named checkpoint
 *
 * and in the code being investigated:
 *
 *	meizu_m2582_halt("bt_enable");
 *
 * With no cmdline option nothing is ever frozen, so this can stay enabled in a
 * normal image.
 */

#define pr_fmt(fmt) "m2582-halt: " fmt

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched/clock.h>
#include <linux/smp.h>
#include <linux/string.h>
#include <linux/timer.h>

static const char *halt_at __ro_after_init;
static unsigned int halt_ms __ro_after_init;
static bool halted;

static int __init m2582_halt_setup_at(char *arg)
{
	if (arg && *arg)
		halt_at = arg;
	return 0;
}
early_param("meizu.halt_at", m2582_halt_setup_at);

static int __init m2582_halt_setup_ms(char *arg)
{
	if (arg && *arg)
		halt_ms = simple_strtoul(arg, NULL, 0);
	return 0;
}
early_param("meizu.halt_ms", m2582_halt_setup_ms);

static void m2582_freeze(const char *where)
{
	if (halted)
		return;
	halted = true;

	/*
	 * pr_emerg rather than pr_info: this has to be visible even when the
	 * console is quiet by policy, and it is the one line the photograph is
	 * actually for.
	 */
	pr_emerg("HALTED at %s (meizu.halt_at/meizu.halt_ms) -- frozen for a photo\n",
		 where);

	/*
	 * The panel console writes straight into the framebuffer, but give it
	 * a moment anyway: the picture is worthless if the last line has not
	 * landed yet.
	 */
	mdelay(300);

	/*
	 * Spin with interrupts on and the other CPUs running.  Stopping them
	 * (or disabling interrupts) also stops the kernel's watchdog service,
	 * and then the PMIC resets the machine within about a second -- which
	 * is exactly what the first version of this did: the freeze looked
	 * identical to a reset, black screen included, and sent the search in
	 * the wrong direction.
	 *
	 * Spinning here is enough to stop the boot on its own: initcalls run
	 * one after another on a single thread, so the call site -- usually a
	 * driver probe -- never returns and nothing later happens.
	 */
	for (;;)
		mdelay(1000);
}

/**
 * meizu_m2582_halt() - freeze the machine if this checkpoint was selected
 * @where: checkpoint name, matched against meizu.halt_at=
 *
 * Safe to call from any context, including a driver probe.  Deliberately not
 * a notifier or a hook: the point is that a normal call site names itself.
 */
void meizu_m2582_halt(const char *where)
{
	if (!halt_at || strcmp(halt_at, where))
		return;

	m2582_freeze(where);
}
EXPORT_SYMBOL_GPL(meizu_m2582_halt);

static struct timer_list m2582_halt_timer;

static void m2582_halt_timeout(struct timer_list *unused)
{
	m2582_freeze("meizu.halt_ms");
}

static int __init m2582_halt_init(void)
{
	unsigned int now_ms, delay_ms;

	if (!halt_ms)
		return 0;

	/*
	 * halt_ms is an absolute boot time, on the same clock the dmesg
	 * timestamps come from.  Measuring from "now" instead would have made
	 * the number depend on when this initcall happens to run, which is
	 * exactly the confusion it is supposed to remove: a log line that says
	 * [0.9 s] should be reachable with halt_ms=900.
	 */
	now_ms = div_u64(local_clock(), NSEC_PER_MSEC);
	delay_ms = halt_ms > now_ms ? halt_ms - now_ms : 1;

	pr_info("freezing at %u ms (in %u ms)\n", halt_ms, delay_ms);
	timer_setup(&m2582_halt_timer, m2582_halt_timeout, 0);
	m2582_halt_timer.expires = jiffies + msecs_to_jiffies(delay_ms);
	add_timer(&m2582_halt_timer);

	return 0;
}
core_initcall(m2582_halt_init);
