// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Red Hat
 * Copyright (c) 2014-2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Author: Rob Clark <robdclark@gmail.com>
 */

#define pr_fmt(fmt)	"[drm:%s:%d] " fmt, __func__, __LINE__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/dma-buf.h>
#include <linux/io.h>
#include <linux/of_irq.h>
#include <linux/pm_opp.h>

#include <drm/drm_crtc.h>
#include <drm/drm_file.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>
#include <drm/drm_writeback.h>

#include <linux/soc/qcom/ubwc.h>

#include "msm_drv.h"
#include "msm_mmu.h"
#include "msm_gem.h"
#include "disp/msm_disp_snapshot.h"

#include "dpu_core_irq.h"
#include "dpu_crtc.h"
#include "dpu_encoder.h"
#include "dpu_formats.h"
#include "dpu_hw_vbif.h"
#include "dpu_kms.h"
#include "dpu_hw_dspp.h"
#include <linux/meizu-m2582.h>
#include "dpu_hw_reg_dma.h"
#include "dpu_plane.h"
#include "dpu_vbif.h"
#include "dpu_writeback.h"

#define CREATE_TRACE_POINTS
#include "dpu_trace.h"

/*
 * To enable overall DRM driver logging
 * # echo 0x2 > /sys/module/drm/parameters/debug
 *
 * To enable DRM driver h/w logging
 * # echo <mask> > /sys/kernel/debug/dri/0/debug/hw_log_mask
 *
 * See dpu_hw_mdss.h for h/w logging mask definitions (search for DPU_DBG_MASK_)
 */
#define DPU_DEBUGFS_DIR "msm_dpu"
#define DPU_DEBUGFS_HWMASKNAME "hw_log_mask"

bool dpu_use_virtual_planes = true;
module_param(dpu_use_virtual_planes, bool, 0);

static int dpu_kms_hw_init(struct msm_kms *kms);
static void _dpu_kms_mmu_destroy(struct dpu_kms *dpu_kms);

#ifdef CONFIG_DEBUG_FS
static int _dpu_danger_signal_status(struct seq_file *s,
		bool danger_status)
{
	struct dpu_danger_safe_status status;
	struct dpu_kms *kms = s->private;
	int i;

	if (!kms->hw_mdp) {
		DPU_ERROR("invalid arg(s)\n");
		return 0;
	}

	memset(&status, 0, sizeof(struct dpu_danger_safe_status));

	pm_runtime_get_sync(&kms->pdev->dev);
	if (danger_status) {
		seq_puts(s, "\nDanger signal status:\n");
		if (kms->hw_mdp->ops.get_danger_status)
			kms->hw_mdp->ops.get_danger_status(kms->hw_mdp,
					&status);
	} else {
		seq_puts(s, "\nSafe signal status:\n");
		if (kms->hw_mdp->ops.get_safe_status)
			kms->hw_mdp->ops.get_safe_status(kms->hw_mdp,
					&status);
	}
	pm_runtime_put_sync(&kms->pdev->dev);

	seq_printf(s, "MDP     :  0x%x\n", status.mdp);

	for (i = SSPP_VIG0; i < SSPP_MAX; i++)
		seq_printf(s, "SSPP%d   :  0x%x  \n", i - SSPP_VIG0,
				status.sspp[i]);
	seq_puts(s, "\n");

	return 0;
}

static int dpu_debugfs_danger_stats_show(struct seq_file *s, void *v)
{
	return _dpu_danger_signal_status(s, true);
}
DEFINE_SHOW_ATTRIBUTE(dpu_debugfs_danger_stats);

static int dpu_debugfs_safe_stats_show(struct seq_file *s, void *v)
{
	return _dpu_danger_signal_status(s, false);
}
DEFINE_SHOW_ATTRIBUTE(dpu_debugfs_safe_stats);

static ssize_t _dpu_plane_danger_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	struct dpu_kms *kms = file->private_data;
	int len;
	char buf[40];

	len = scnprintf(buf, sizeof(buf), "%d\n", !kms->has_danger_ctrl);

	return simple_read_from_buffer(buff, count, ppos, buf, len);
}

static void _dpu_plane_set_danger_state(struct dpu_kms *kms, bool enable)
{
	struct drm_plane *plane;

	drm_for_each_plane(plane, kms->dev) {
		if (plane->fb && plane->state) {
			dpu_plane_danger_signal_ctrl(plane, enable);
			DPU_DEBUG("plane:%d img:%dx%d ",
				plane->base.id, plane->fb->width,
				plane->fb->height);
			DPU_DEBUG("src[%d,%d,%d,%d] dst[%d,%d,%d,%d]\n",
				plane->state->src_x >> 16,
				plane->state->src_y >> 16,
				plane->state->src_w >> 16,
				plane->state->src_h >> 16,
				plane->state->crtc_x, plane->state->crtc_y,
				plane->state->crtc_w, plane->state->crtc_h);
		} else {
			DPU_DEBUG("Inactive plane:%d\n", plane->base.id);
		}
	}
}

static ssize_t _dpu_plane_danger_write(struct file *file,
		    const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct dpu_kms *kms = file->private_data;
	int disable_panic;
	int ret;

	ret = kstrtouint_from_user(user_buf, count, 0, &disable_panic);
	if (ret)
		return ret;

	if (disable_panic) {
		/* Disable panic signal for all active pipes */
		DPU_DEBUG("Disabling danger:\n");
		_dpu_plane_set_danger_state(kms, false);
		kms->has_danger_ctrl = false;
	} else {
		/* Enable panic signal for all active pipes */
		DPU_DEBUG("Enabling danger:\n");
		kms->has_danger_ctrl = true;
		_dpu_plane_set_danger_state(kms, true);
	}

	return count;
}

static const struct file_operations dpu_plane_danger_enable = {
	.open = simple_open,
	.read = _dpu_plane_danger_read,
	.write = _dpu_plane_danger_write,
};

static void dpu_debugfs_danger_init(struct dpu_kms *dpu_kms,
		struct dentry *parent)
{
	struct dentry *entry = debugfs_create_dir("danger", parent);

	debugfs_create_file("danger_status", 0600, entry,
			dpu_kms, &dpu_debugfs_danger_stats_fops);
	debugfs_create_file("safe_status", 0600, entry,
			dpu_kms, &dpu_debugfs_safe_stats_fops);
	debugfs_create_file("disable_danger", 0600, entry,
			dpu_kms, &dpu_plane_danger_enable);

}

/*
 * Companion structure for dpu_debugfs_create_regset32.
 */
struct dpu_debugfs_regset32 {
	uint32_t offset;
	uint32_t blk_len;
	struct dpu_kms *dpu_kms;
};

static int dpu_regset32_show(struct seq_file *s, void *data)
{
	struct dpu_debugfs_regset32 *regset = s->private;
	struct dpu_kms *dpu_kms = regset->dpu_kms;
	void __iomem *base;
	uint32_t i, addr;

	if (!dpu_kms->mmio)
		return 0;

	base = dpu_kms->mmio + regset->offset;

	/* insert padding spaces, if needed */
	if (regset->offset & 0xF) {
		seq_printf(s, "[%x]", regset->offset & ~0xF);
		for (i = 0; i < (regset->offset & 0xF); i += 4)
			seq_puts(s, "         ");
	}

	pm_runtime_get_sync(&dpu_kms->pdev->dev);

	/* main register output */
	for (i = 0; i < regset->blk_len; i += 4) {
		addr = regset->offset + i;
		if ((addr & 0xF) == 0x0)
			seq_printf(s, i ? "\n[%x]" : "[%x]", addr);
		seq_printf(s, " %08x", readl_relaxed(base + i));
	}
	seq_puts(s, "\n");
	pm_runtime_put_sync(&dpu_kms->pdev->dev);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dpu_regset32);

/**
 * dpu_debugfs_create_regset32 - Create register read back file for debugfs
 *
 * This function is almost identical to the standard debugfs_create_regset32()
 * function, with the main difference being that a list of register
 * names/offsets do not need to be provided. The 'read' function simply outputs
 * sequential register values over a specified range.
 *
 * @name:   File name within debugfs
 * @mode:   File mode within debugfs
 * @parent: Parent directory entry within debugfs, can be NULL
 * @offset: sub-block offset
 * @length: sub-block length, in bytes
 * @dpu_kms: pointer to dpu kms structure
 */
void dpu_debugfs_create_regset32(const char *name, umode_t mode,
		void *parent,
		uint32_t offset, uint32_t length, struct dpu_kms *dpu_kms)
{
	struct dpu_debugfs_regset32 *regset;

	if (WARN_ON(!name || !dpu_kms || !length))
		return;

	regset = devm_kzalloc(&dpu_kms->pdev->dev, sizeof(*regset), GFP_KERNEL);
	if (!regset)
		return;

	/* make sure offset is a multiple of 4 */
	regset->offset = round_down(offset, 4);
	regset->blk_len = length;
	regset->dpu_kms = dpu_kms;

	debugfs_create_file(name, mode, parent, regset, &dpu_regset32_fops);
}

static void dpu_debugfs_sspp_init(struct dpu_kms *dpu_kms, struct dentry *debugfs_root)
{
	struct dentry *entry = debugfs_create_dir("sspp", debugfs_root);
	int i;

	if (IS_ERR(entry))
		return;

	for (i = SSPP_NONE; i < SSPP_MAX; i++) {
		struct dpu_hw_sspp *hw = dpu_rm_get_sspp(&dpu_kms->rm, i);

		if (!hw)
			continue;

		_dpu_hw_sspp_init_debugfs(hw, dpu_kms, entry);
	}
}

/* M2582 register poke probe; see the definition at the end of the file. */

/*
 * M2582: force the DPU core clock, bypassing dpu_core_perf's "only update when
 * the computed rate changed" logic.  The factory runs this exact panel mode with
 * disp_cc_mdss_mdp_clk at 85.714286 MHz, while this board sits at 660 MHz -- the
 * highest entry in dispcc's table -- because dpu_core_perf never asks for
 * anything lower after the first commit.  Seven times the core clock is worth
 * one experiment: write the rate the factory uses and see whether the fetch
 * still loses every other 256 B.
 */
static ssize_t m2582_coreclk_write(struct file *f, const char __user *ubuf,
				   size_t len, loff_t *off)
{
	struct dpu_kms *dpu_kms = f->private_data;
	char buf[32];
	unsigned long rate;
	int ret;

	if (!dpu_kms || len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	if (kstrtoul(buf, 0, &rate))
		return -EINVAL;

	if (pm_runtime_get_sync(&dpu_kms->pdev->dev) < 0) {
		pm_runtime_put_noidle(&dpu_kms->pdev->dev);
		return -EIO;
	}
	ret = dev_pm_opp_set_rate(&dpu_kms->pdev->dev, rate);
	pm_runtime_put(&dpu_kms->pdev->dev);
	pr_info("m2582-coreclk: request %lu -> %d\n", rate, ret);
	return len;
}

static const struct file_operations m2582_coreclk_fops = {
	.open = simple_open,
	.write = m2582_coreclk_write,
	.llseek = noop_llseek,
};

static void m2582_poke_init(struct dentry *entry, struct dpu_kms *dpu_kms);

static int dpu_kms_debugfs_init(struct msm_kms *kms, struct drm_minor *minor)
{
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);
	void *p = dpu_hw_util_get_log_mask_ptr();
	struct dentry *entry;

	if (!p)
		return -EINVAL;

	/* Only create a set of debugfs for the primary node, ignore render nodes */
	if (minor->type != DRM_MINOR_PRIMARY)
		return 0;

	entry = debugfs_create_dir("debug", minor->debugfs_root);

	debugfs_create_x32(DPU_DEBUGFS_HWMASKNAME, 0600, entry, p);

	dpu_debugfs_danger_init(dpu_kms, entry);
	dpu_debugfs_vbif_init(dpu_kms, entry);
	dpu_debugfs_core_irq_init(dpu_kms, entry);
	dpu_debugfs_sspp_init(dpu_kms, entry);

	/*
	 * M2582: read-only window on the command-mode handshake, so the tear
	 * check can be compared with the state this board was measured working
	 * in.  INTF_1 is at 0x35000: 0x35284 TEAR_CHECK_EN, 0x35288
	 * SYNC_CONFIG_VSYNC, which has to read 0x0018003a (BIT(19) counter
	 * enable, BIT(20) hw_vsync_mode, 0x3a = 58).  0x1501c is CTL_START; a
	 * value that stays 1 means the start request was never consumed, which
	 * is what makes the ping-pong never report done.
	 */
	dpu_debugfs_create_regset32("m2582_intf_tear", 0400, entry,
				    0x35280, 16, dpu_kms);
	/*
	 * M2582: INTF_TEAR_INT_COUNT_VAL on its own (0x298, one word).  It is the
	 * only free 12.78 MHz clock in the display block, and the rate
	 * measurement in artifacts/meizu22/dsi-wire-byte-measurement.md times a
	 * single test-pattern frame with it: 5.89 ms means the link runs at
	 * 550 MB/s and 11.78 ms means it runs at 275.  Reading it out of the
	 * 0x300-byte m2582_intf window costs milliseconds, which is the same
	 * order as the difference being measured, hence this one-word file.
	 */
	dpu_debugfs_create_regset32("m2582_intf_count", 0400, entry,
				    0x35298, 4, dpu_kms);
	dpu_debugfs_create_regset32("m2582_ctl", 0400, entry,
				    0x15000, 0x40, dpu_kms);
	/*
	 * M2582: the two halves of the DSC-merged command-mode output, side by
	 * side, so a per-half difference shows up as a differing word in the same
	 * position.  pingpong_0/1 are at 0x69000/0x6a000 (catalog), both merging
	 * through merge_3d_0 at 0x4e000, and the two DSC encoders share the DCE at
	 * 0x80000 with encoder sub-blocks at +0x100 and +0x200 (sm8750_dsc_sblk_0/1).
	 * A half that is one line out of step must leave a trace in one of these.
	 */
	dpu_debugfs_create_regset32("m2582_pp0", 0400, entry, 0x69000, 0x100, dpu_kms);
	dpu_debugfs_create_regset32("m2582_pp1", 0400, entry, 0x6a000, 0x100, dpu_kms);
	dpu_debugfs_create_regset32("m2582_merge3d", 0400, entry, 0x4e000, 0x20, dpu_kms);
	dpu_debugfs_create_regset32("m2582_dsc0", 0400, entry, 0x80100, 0x100, dpu_kms);
	dpu_debugfs_create_regset32("m2582_dsc1", 0400, entry, 0x80200, 0x100, dpu_kms);
	/*
	 * M2582: the DCE common block, which none of the probes above cover --
	 * 0x80000 is the DSC_CMN_MAIN_CNF shared by both encoders.  It has never
	 * been read on this board and the two factory sources disagree: their
	 * evtlog says dsc_common_mode = 3 (_dce_dsc_setup_single payload
	 * "42 4b0 a6e 3 ..."), which makes CMN_MAIN_CNF 0x83, while the single
	 * raw factory read of this word is 0x00000080.  Mainline writes
	 * BIT(0)|BIT(1) (split panel / multiplex) plus 1 << 7 (one active slice
	 * per encoder), i.e. 0x83.  So this is either a real difference in the
	 * block that stitches the two halves together, or bits 0/1 are
	 * self-clearing here; one read settles which.
	 */
	dpu_debugfs_create_regset32("m2582_dce0", 0400, entry, 0x80000, 0x80, dpu_kms);
	/*
	 * M2582: the two DSC *wrapper* sub-blocks (+0xf00 and +0xf80), which is
	 * where the half-to-ping-pong binding (DSC_CTL[2:0]) and the merge shape
	 * (DSC_CFG bits 0/12/13) live.  They are the only registers that say how
	 * the two encoders' output is stitched into the single interface, and
	 * they have never been read back on this board.
	 */
	dpu_debugfs_create_regset32("m2582_dscctl", 0400, entry, 0x80f00, 0x100, dpu_kms);
	/* Whole INTF_1 window: MUX, CONFIG/CONFIG2, TIMING_ENGINE_EN, line count. */
	dpu_debugfs_create_regset32("m2582_intf", 0400, entry, 0x35000, 0x300, dpu_kms);
	/*
	 * M2582: the MDP top block's split-router registers.  0x2f4 SPLIT_DISPLAY_EN
	 * and 0x2f8/0x3f0 UPPER/LOWER_PIPE_CTRL decide which ping-pong feeds which
	 * interface, and 0x330/0x334 PPB0_CNTL/CONFIG, 0x338/0x33c PPB1_CNTL/CONFIG
	 * are the ping-pong split.  Mainline only writes them from the video-mode
	 * split-display path, so in command mode this board keeps the bootloader's
	 * values; the factory clears the PPB pair unconditionally for a
	 * single-interface panel.  One window covers 0x2f0..0x3ff.
	 */
	dpu_debugfs_create_regset32("m2582_mdptop", 0400, entry, 0x2f0, 0x110, dpu_kms);
	/*
	 * M2582: the two layer mixers, side by side.  CTL_LAYER_0/1 (0x15000,
	 * 0x15004) carry the per-mixer rectangle -- stage, format, multirect and
	 * the source position -- which is the only place a one-row vertical
	 * difference between the two halves could be expressed in software.  The
	 * LM blocks themselves are here because they hold the blend/op-mode words
	 * for the same two mixers.
	 */
	dpu_debugfs_create_regset32("m2582_lm0", 0400, entry, 0x44000, 0x40, dpu_kms);
	dpu_debugfs_create_regset32("m2582_lm1", 0400, entry, 0x45000, 0x40, dpu_kms);
	m2582_poke_init(entry, dpu_kms);
	dpu_reg_dma_debugfs_init(dpu_kms, entry);

	return dpu_core_perf_debugfs_init(dpu_kms, entry);
}
#endif

/* Global/shared object state funcs */

/*
 * This is a helper that returns the private state currently in operation.
 * Note that this would return the "old_state" if called in the atomic check
 * path, and the "new_state" after the atomic swap has been done.
 */
struct dpu_global_state *
dpu_kms_get_existing_global_state(struct dpu_kms *dpu_kms)
{
	return to_dpu_global_state(dpu_kms->global_state.state);
}

/*
 * This acquires the modeset lock set aside for global state, creates
 * a new duplicated private object state.
 */
struct dpu_global_state *dpu_kms_get_global_state(struct drm_atomic_commit *s)
{
	struct msm_drm_private *priv = s->dev->dev_private;
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	struct drm_private_state *priv_state;

	priv_state = drm_atomic_get_private_obj_state(s,
						&dpu_kms->global_state);
	if (IS_ERR(priv_state))
		return ERR_CAST(priv_state);

	return to_dpu_global_state(priv_state);
}

static struct drm_private_state *
dpu_kms_global_duplicate_state(struct drm_private_obj *obj)
{
	struct dpu_global_state *state;

	state = kmemdup(obj->state, sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_private_obj_duplicate_state(obj, &state->base);

	return &state->base;
}

static void dpu_kms_global_destroy_state(struct drm_private_obj *obj,
				      struct drm_private_state *state)
{
	struct dpu_global_state *dpu_state = to_dpu_global_state(state);

	kfree(dpu_state);
}

static struct drm_private_state *
dpu_kms_global_create_state(struct drm_private_obj *obj)
{
	struct drm_device *dev = obj->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	struct dpu_global_state *dpu_state;

	dpu_state = kzalloc_obj(*dpu_state);
	if (!dpu_state)
		return ERR_PTR(-ENOMEM);

	__drm_atomic_helper_private_obj_create_state(obj, &dpu_state->base);
	dpu_state->rm = &dpu_kms->rm;

	return &dpu_state->base;
}

static void dpu_kms_global_print_state(struct drm_printer *p,
				       const struct drm_private_state *state)
{
	const struct dpu_global_state *global_state = to_dpu_global_state(state);

	dpu_rm_print_state(p, global_state);
}

static const struct drm_private_state_funcs dpu_kms_global_state_funcs = {
	.atomic_create_state = dpu_kms_global_create_state,
	.atomic_duplicate_state = dpu_kms_global_duplicate_state,
	.atomic_destroy_state = dpu_kms_global_destroy_state,
	.atomic_print_state = dpu_kms_global_print_state,
};

static void dpu_kms_global_obj_fini(struct dpu_kms *dpu_kms)
{
	drm_atomic_private_obj_fini(&dpu_kms->global_state);
}

static int dpu_kms_parse_data_bus_icc_path(struct dpu_kms *dpu_kms)
{
	struct icc_path *path0;
	struct icc_path *path1;
	struct device *dpu_dev = &dpu_kms->pdev->dev;

	path0 = msm_icc_get(dpu_dev, "mdp0-mem");
	path1 = msm_icc_get(dpu_dev, "mdp1-mem");

	if (IS_ERR_OR_NULL(path0))
		return PTR_ERR_OR_ZERO(path0);

	dpu_kms->path[0] = path0;
	dpu_kms->num_paths = 1;

	if (!IS_ERR_OR_NULL(path1)) {
		dpu_kms->path[1] = path1;
		dpu_kms->num_paths++;
	}
	return 0;
}

static int dpu_kms_enable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	return dpu_crtc_vblank(crtc, true);
}

static void dpu_kms_disable_vblank(struct msm_kms *kms, struct drm_crtc *crtc)
{
	dpu_crtc_vblank(crtc, false);
}

static void dpu_kms_enable_commit(struct msm_kms *kms)
{
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);
	pm_runtime_get_sync(&dpu_kms->pdev->dev);
}

static void dpu_kms_disable_commit(struct msm_kms *kms)
{
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);
	pm_runtime_put_sync(&dpu_kms->pdev->dev);
}

static int dpu_kms_check_mode_changed(struct msm_kms *kms, struct drm_atomic_commit *state)
{
	struct drm_crtc_state *new_crtc_state;
	struct drm_crtc_state *old_crtc_state;
	struct drm_crtc *crtc;
	int i;

	for_each_oldnew_crtc_in_state(state, crtc, old_crtc_state, new_crtc_state, i)
		dpu_crtc_check_mode_changed(old_crtc_state, new_crtc_state);

	return 0;
}

static void dpu_kms_flush_commit(struct msm_kms *kms, unsigned crtc_mask)
{
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);
	struct drm_crtc *crtc;

	for_each_crtc_mask(dpu_kms->dev, crtc, crtc_mask) {
		if (!crtc->state->active)
			continue;

		trace_dpu_kms_commit(DRMID(crtc));
		dpu_crtc_commit_kickoff(crtc);
	}
}

static void dpu_kms_complete_commit(struct msm_kms *kms, unsigned crtc_mask)
{
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);
	struct drm_crtc *crtc;

	DPU_ATRACE_BEGIN("kms_complete_commit");

	for_each_crtc_mask(dpu_kms->dev, crtc, crtc_mask)
		dpu_crtc_complete_commit(crtc);

	DPU_ATRACE_END("kms_complete_commit");
}

static void dpu_kms_wait_for_commit_done(struct msm_kms *kms,
		struct drm_crtc *crtc)
{
	struct drm_encoder *encoder;
	struct drm_device *dev;
	int ret;

	if (!kms || !crtc || !crtc->state) {
		DPU_ERROR("invalid params\n");
		return;
	}

	dev = crtc->dev;

	if (!crtc->state->enable) {
		DPU_DEBUG("[crtc:%d] not enable\n", crtc->base.id);
		return;
	}

	if (!drm_atomic_crtc_effectively_active(crtc->state)) {
		DPU_DEBUG("[crtc:%d] not active\n", crtc->base.id);
		return;
	}

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		if (encoder->crtc != crtc)
			continue;
		/*
		 * Wait for post-flush if necessary to delay before
		 * plane_cleanup. For example, wait for vsync in case of video
		 * mode panels. This may be a no-op for command mode panels.
		 */
		trace_dpu_kms_wait_for_commit_done(DRMID(crtc));
		ret = dpu_encoder_wait_for_commit_done(encoder);
		if (ret && ret != -EWOULDBLOCK) {
			DPU_ERROR("wait for commit done returned %d\n", ret);
			break;
		}
	}
}

static void dpu_kms_wait_flush(struct msm_kms *kms, unsigned crtc_mask)
{
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);
	struct drm_crtc *crtc;

	for_each_crtc_mask(dpu_kms->dev, crtc, crtc_mask)
		dpu_kms_wait_for_commit_done(kms, crtc);
}

static const char *dpu_vsync_sources[] = {
	[DPU_VSYNC_SOURCE_GPIO_0] = "mdp_vsync_p",
	[DPU_VSYNC_SOURCE_GPIO_1] = "mdp_vsync_s",
	[DPU_VSYNC_SOURCE_GPIO_2] = "mdp_vsync_e",
	[DPU_VSYNC_SOURCE_INTF_0] = "mdp_intf0",
	[DPU_VSYNC_SOURCE_INTF_1] = "mdp_intf1",
	[DPU_VSYNC_SOURCE_INTF_2] = "mdp_intf2",
	[DPU_VSYNC_SOURCE_INTF_3] = "mdp_intf3",
	[DPU_VSYNC_SOURCE_WD_TIMER_0] = "timer0",
	[DPU_VSYNC_SOURCE_WD_TIMER_1] = "timer1",
	[DPU_VSYNC_SOURCE_WD_TIMER_2] = "timer2",
	[DPU_VSYNC_SOURCE_WD_TIMER_3] = "timer3",
	[DPU_VSYNC_SOURCE_WD_TIMER_4] = "timer4",
};

static int dpu_kms_dsi_set_te_source(struct msm_display_info *info,
				     struct msm_dsi *dsi)
{
	const char *te_source = msm_dsi_get_te_source(dsi);
	int i;

	if (!te_source) {
		info->vsync_source = DPU_VSYNC_SOURCE_GPIO_0;
		return 0;
	}

	/* we can not use match_string since dpu_vsync_sources is a sparse array */
	for (i = 0; i < ARRAY_SIZE(dpu_vsync_sources); i++) {
		if (dpu_vsync_sources[i] &&
		    !strcmp(dpu_vsync_sources[i], te_source)) {
			info->vsync_source = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int _dpu_kms_initialize_dsi(struct drm_device *dev,
				    struct msm_drm_private *priv,
				    struct dpu_kms *dpu_kms)
{
	struct drm_encoder *encoder = NULL;
	struct msm_display_info info;
	int i, rc = 0;

	if (!(priv->kms->dsi[0] || priv->kms->dsi[1]))
		return rc;

	/*
	 * We support following confiurations:
	 * - Single DSI host (dsi0 or dsi1)
	 * - Two independent DSI hosts
	 * - Bonded DSI0 and DSI1 hosts
	 *
	 * TODO: Support swapping DSI0 and DSI1 in the bonded setup.
	 */
	for (i = 0; i < ARRAY_SIZE(priv->kms->dsi); i++) {
		int other = (i + 1) % 2;

		if (!priv->kms->dsi[i])
			continue;

		if (msm_dsi_is_bonded_dsi(priv->kms->dsi[i]) &&
		    !msm_dsi_is_master_dsi(priv->kms->dsi[i]))
			continue;

		memset(&info, 0, sizeof(info));
		info.intf_type = INTF_DSI;

		info.h_tile_instance[info.num_of_h_tiles++] = i;
		if (msm_dsi_is_bonded_dsi(priv->kms->dsi[i]))
			info.h_tile_instance[info.num_of_h_tiles++] = other;

		info.is_cmd_mode = msm_dsi_is_cmd_mode(priv->kms->dsi[i]);

		rc = dpu_kms_dsi_set_te_source(&info, priv->kms->dsi[i]);
		if (rc) {
			DPU_ERROR("failed to identify TE source for dsi display\n");
			return rc;
		}

		encoder = dpu_encoder_init(dev, DRM_MODE_ENCODER_DSI, &info);
		if (IS_ERR(encoder)) {
			DPU_ERROR("encoder init failed for dsi display\n");
			return PTR_ERR(encoder);
		}

		rc = msm_dsi_modeset_init(priv->kms->dsi[i], dev, encoder);
		if (rc) {
			DPU_ERROR("modeset_init failed for dsi[%d], rc = %d\n",
				i, rc);
			break;
		}

		if (msm_dsi_is_bonded_dsi(priv->kms->dsi[i]) && priv->kms->dsi[other]) {
			rc = msm_dsi_modeset_init(priv->kms->dsi[other], dev, encoder);
			if (rc) {
				DPU_ERROR("modeset_init failed for dsi[%d], rc = %d\n",
					other, rc);
				break;
			}
		}
	}

	return rc;
}

static int _dpu_kms_initialize_displayport(struct drm_device *dev,
					    struct msm_drm_private *priv,
					    struct dpu_kms *dpu_kms)
{
	struct drm_encoder *encoder = NULL;
	struct msm_display_info info;
	bool yuv_supported;
	int rc;
	int i;

	for (i = 0; i < ARRAY_SIZE(priv->kms->dp); i++) {
		if (!priv->kms->dp[i])
			continue;

		memset(&info, 0, sizeof(info));
		info.num_of_h_tiles = 1;
		info.h_tile_instance[0] = i;
		info.intf_type = INTF_DP;

		encoder = dpu_encoder_init(dev, DRM_MODE_ENCODER_TMDS, &info);
		if (IS_ERR(encoder)) {
			DPU_ERROR("encoder init failed for dsi display\n");
			return PTR_ERR(encoder);
		}

		yuv_supported = !!dpu_kms->catalog->cdm;
		rc = msm_dp_modeset_init(priv->kms->dp[i], dev, encoder, yuv_supported);
		if (rc) {
			DPU_ERROR("modeset_init failed for DP, rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int _dpu_kms_initialize_hdmi(struct drm_device *dev,
				    struct msm_drm_private *priv,
				    struct dpu_kms *dpu_kms)
{
	struct drm_encoder *encoder = NULL;
	struct msm_display_info info;
	int rc;

	if (!priv->kms->hdmi)
		return 0;

	memset(&info, 0, sizeof(info));
	info.num_of_h_tiles = 1;
	info.h_tile_instance[0] = 0;
	info.intf_type = INTF_HDMI;

	encoder = dpu_encoder_init(dev, DRM_MODE_ENCODER_TMDS, &info);
	if (IS_ERR(encoder)) {
		DPU_ERROR("encoder init failed for HDMI display\n");
		return PTR_ERR(encoder);
	}

	rc = msm_hdmi_modeset_init(priv->kms->hdmi, dev, encoder);
	if (rc) {
		DPU_ERROR("modeset_init failed for DP, rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int _dpu_kms_initialize_writeback(struct drm_device *dev,
		struct msm_drm_private *priv, struct dpu_kms *dpu_kms,
		const u32 *wb_formats, int n_formats)
{
	struct drm_encoder *encoder = NULL;
	struct msm_display_info info;
	const enum dpu_wb wb_idx = WB_2;
	u32 maxlinewidth;
	int rc;

	memset(&info, 0, sizeof(info));

	info.num_of_h_tiles = 1;
	/* use only WB idx 2 instance for DPU */
	info.h_tile_instance[0] = wb_idx;
	info.intf_type = INTF_WB;

	maxlinewidth = dpu_rm_get_wb(&dpu_kms->rm, info.h_tile_instance[0])->caps->maxlinewidth;

	encoder = dpu_encoder_init(dev, DRM_MODE_ENCODER_VIRTUAL, &info);
	if (IS_ERR(encoder)) {
		DPU_ERROR("encoder init failed for dsi display\n");
		return PTR_ERR(encoder);
	}

	rc = dpu_writeback_init(dev, encoder, wb_formats, n_formats, maxlinewidth);
	if (rc) {
		DPU_ERROR("dpu_writeback_init, rc = %d\n", rc);
		return rc;
	}

	return 0;
}

/**
 * _dpu_kms_setup_displays - create encoders, bridges and connectors
 *                           for underlying displays
 * @dev:        Pointer to drm device structure
 * @priv:       Pointer to private drm device data
 * @dpu_kms:    Pointer to dpu kms structure
 * Returns:     Zero on success
 */
static int _dpu_kms_setup_displays(struct drm_device *dev,
				    struct msm_drm_private *priv,
				    struct dpu_kms *dpu_kms)
{
	int rc = 0;
	int i;

	rc = _dpu_kms_initialize_dsi(dev, priv, dpu_kms);
	if (rc) {
		DPU_ERROR("initialize_dsi failed, rc = %d\n", rc);
		return rc;
	}

	rc = _dpu_kms_initialize_displayport(dev, priv, dpu_kms);
	if (rc) {
		DPU_ERROR("initialize_DP failed, rc = %d\n", rc);
		return rc;
	}

	rc = _dpu_kms_initialize_hdmi(dev, priv, dpu_kms);
	if (rc) {
		DPU_ERROR("initialize HDMI failed, rc = %d\n", rc);
		return rc;
	}

	/* Since WB isn't a driver check the catalog before initializing */
	if (dpu_kms->catalog->wb_count) {
		for (i = 0; i < dpu_kms->catalog->wb_count; i++) {
			if (dpu_kms->catalog->wb[i].id == WB_2) {
				rc = _dpu_kms_initialize_writeback(dev, priv, dpu_kms,
						dpu_kms->catalog->wb[i].format_list,
						dpu_kms->catalog->wb[i].num_formats);
				if (rc) {
					DPU_ERROR("initialize_WB failed, rc = %d\n", rc);
					return rc;
				}
			}
		}
	}

	return rc;
}

#define MAX_PLANES 20
static int _dpu_kms_drm_obj_init(struct dpu_kms *dpu_kms)
{
	struct drm_device *dev;
	struct drm_plane *primary_planes[MAX_PLANES], *plane;
	struct drm_plane *cursor_planes[MAX_PLANES] = { NULL };
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	unsigned int num_encoders;

	struct msm_drm_private *priv;
	const struct dpu_mdss_cfg *catalog;

	int primary_planes_idx = 0, cursor_planes_idx = 0, i, ret;
	int max_crtc_count;
	dev = dpu_kms->dev;
	priv = dev->dev_private;
	catalog = dpu_kms->catalog;
	/* Diagnostic mode: bind DRM planes directly to their catalog SSPPs. */
	if (m2582_feature_opt_in("physicalplanes"))
		dpu_use_virtual_planes = false;

	/*
	 * Create encoder and query display drivers to create
	 * bridges and connectors
	 */
	ret = _dpu_kms_setup_displays(dev, priv, dpu_kms);
	if (ret)
		return ret;

	num_encoders = 0;
	drm_for_each_encoder(encoder, dev) {
		num_encoders++;
		if (catalog->cwb_count > 0)
			encoder->possible_clones = dpu_encoder_get_clones(encoder);
	}

	max_crtc_count = min(catalog->mixer_count, num_encoders);

	/* Create the planes, keeping track of one primary/cursor per crtc */
	for (i = 0; i < catalog->sspp_count; i++) {
		enum drm_plane_type type;

		if ((catalog->sspp[i].features & BIT(DPU_SSPP_CURSOR))
			&& cursor_planes_idx < max_crtc_count)
			type = DRM_PLANE_TYPE_CURSOR;
		else if (primary_planes_idx < max_crtc_count)
			type = DRM_PLANE_TYPE_PRIMARY;
		else
			type = DRM_PLANE_TYPE_OVERLAY;

		DPU_DEBUG("Create plane type %d with features %lx (cur %lx)\n",
			  type, catalog->sspp[i].features,
			  catalog->sspp[i].features & BIT(DPU_SSPP_CURSOR));

		if (dpu_use_virtual_planes)
			plane = dpu_plane_init_virtual(dev, type, (1UL << max_crtc_count) - 1);
		else
			plane = dpu_plane_init(dev, catalog->sspp[i].id, type,
					       (1UL << max_crtc_count) - 1);
		if (IS_ERR(plane)) {
			DPU_ERROR("dpu_plane_init failed\n");
			ret = PTR_ERR(plane);
			return ret;
		}

		if (type == DRM_PLANE_TYPE_CURSOR)
			cursor_planes[cursor_planes_idx++] = plane;
		else if (type == DRM_PLANE_TYPE_PRIMARY)
			primary_planes[primary_planes_idx++] = plane;
	}

	max_crtc_count = min(max_crtc_count, primary_planes_idx);

	/* Create one CRTC per encoder */
	for (i = 0; i < max_crtc_count; i++) {
		crtc = dpu_crtc_init(dev, primary_planes[i], cursor_planes[i]);
		if (IS_ERR(crtc)) {
			ret = PTR_ERR(crtc);
			return ret;
		}
	}

	/* All CRTCs are compatible with all encoders */
	drm_for_each_encoder(encoder, dev)
		encoder->possible_crtcs = (1 << dev->mode_config.num_crtc) - 1;

	return 0;
}

static void _dpu_kms_hw_destroy(struct dpu_kms *dpu_kms)
{
	dpu_kms->hw_intr = NULL;

	/* needs the iommu domain, so before the mmu goes away */
	dpu_reg_dma_destroy(dpu_kms);

	/* safe to call these more than once during shutdown */
	_dpu_kms_mmu_destroy(dpu_kms);

	dpu_kms->hw_vbif = NULL;

	dpu_kms_global_obj_fini(dpu_kms);

	dpu_kms->catalog = NULL;

	dpu_kms->hw_mdp = NULL;
}

static void dpu_kms_destroy(struct msm_kms *kms)
{
	struct dpu_kms *dpu_kms;

	if (!kms) {
		DPU_ERROR("invalid kms\n");
		return;
	}

	dpu_kms = to_dpu_kms(kms);

	_dpu_kms_hw_destroy(dpu_kms);

	msm_kms_destroy(&dpu_kms->base);

	if (dpu_kms->rpm_enabled)
		pm_runtime_disable(&dpu_kms->pdev->dev);
}

static int dpu_irq_postinstall(struct msm_kms *kms)
{
	struct msm_drm_private *priv;
	struct dpu_kms *dpu_kms = to_dpu_kms(kms);

	if (!dpu_kms || !dpu_kms->dev)
		return -EINVAL;

	priv = dpu_kms->dev->dev_private;
	if (!priv)
		return -EINVAL;

	return 0;
}

static void dpu_kms_mdp_snapshot(struct msm_disp_state *disp_state, struct msm_kms *kms)
{
	int i;
	struct dpu_kms *dpu_kms;
	const struct dpu_mdss_cfg *cat;
	void __iomem *base;

	dpu_kms = to_dpu_kms(kms);

	cat = dpu_kms->catalog;

	pm_runtime_get_sync(&dpu_kms->pdev->dev);

	/* dump CTL sub-blocks HW regs info */
	for (i = 0; i < cat->ctl_count; i++)
		msm_disp_snapshot_add_block(disp_state, cat->ctl[i].len,
				dpu_kms->mmio + cat->ctl[i].base, "%s",
				cat->ctl[i].name);

	/* dump DSPP sub-blocks HW regs info */
	for (i = 0; i < cat->dspp_count; i++) {
		base = dpu_kms->mmio + cat->dspp[i].base;
		msm_disp_snapshot_add_block(disp_state, cat->dspp[i].len, base,
					    "%s", cat->dspp[i].name);

		if (cat->dspp[i].sblk && cat->dspp[i].sblk->pcc.len > 0)
			msm_disp_snapshot_add_block(disp_state, cat->dspp[i].sblk->pcc.len,
						    base + cat->dspp[i].sblk->pcc.base, "%s_%s",
						    cat->dspp[i].name,
						    cat->dspp[i].sblk->pcc.name);
	}

	/* dump INTF sub-blocks HW regs info */
	for (i = 0; i < cat->intf_count; i++)
		msm_disp_snapshot_add_block(disp_state, cat->intf[i].len,
				dpu_kms->mmio + cat->intf[i].base, "%s",
				cat->intf[i].name);

	/* dump PP sub-blocks HW regs info */
	for (i = 0; i < cat->pingpong_count; i++) {
		base = dpu_kms->mmio + cat->pingpong[i].base;
		msm_disp_snapshot_add_block(disp_state, cat->pingpong[i].len, base,
					    "%s", cat->pingpong[i].name);

		/* TE2 sub-block has length of 0, so will not print it */

		if (cat->pingpong[i].sblk && cat->pingpong[i].sblk->dither.len > 0)
			msm_disp_snapshot_add_block(disp_state, cat->pingpong[i].sblk->dither.len,
						    base + cat->pingpong[i].sblk->dither.base,
						    "%s_%s", cat->pingpong[i].name,
						    cat->pingpong[i].sblk->dither.name);
	}

	/* dump SSPP sub-blocks HW regs info */
	for (i = 0; i < cat->sspp_count; i++) {
		base = dpu_kms->mmio + cat->sspp[i].base;
		msm_disp_snapshot_add_block(disp_state, cat->sspp[i].len, base,
					    "%s", cat->sspp[i].name);

		if (cat->sspp[i].sblk && cat->sspp[i].sblk->scaler_blk.len > 0)
			msm_disp_snapshot_add_block(disp_state, cat->sspp[i].sblk->scaler_blk.len,
						    base + cat->sspp[i].sblk->scaler_blk.base,
						    "%s_%s", cat->sspp[i].name,
						    cat->sspp[i].sblk->scaler_blk.name);

		if (cat->sspp[i].sblk && cat->sspp[i].sblk->csc_blk.len > 0)
			msm_disp_snapshot_add_block(disp_state, cat->sspp[i].sblk->csc_blk.len,
						    base + cat->sspp[i].sblk->csc_blk.base,
						    "%s_%s", cat->sspp[i].name,
						    cat->sspp[i].sblk->csc_blk.name);
	}

	/* dump LM sub-blocks HW regs info */
	for (i = 0; i < cat->mixer_count; i++)
		msm_disp_snapshot_add_block(disp_state, cat->mixer[i].len,
				dpu_kms->mmio + cat->mixer[i].base,
				"%s", cat->mixer[i].name);

	/* dump WB sub-blocks HW regs info */
	for (i = 0; i < cat->wb_count; i++)
		msm_disp_snapshot_add_block(disp_state, cat->wb[i].len,
				dpu_kms->mmio + cat->wb[i].base, "%s",
				cat->wb[i].name);

	if (dpu_kms->catalog->mdss_ver->core_major_ver >= 8) {
		msm_disp_snapshot_add_block(disp_state, MDP_PERIPH_TOP0,
				dpu_kms->mmio + cat->mdp[0].base, "top");
		msm_disp_snapshot_add_block(disp_state, cat->mdp[0].len - MDP_PERIPH_TOP0_END,
				dpu_kms->mmio + cat->mdp[0].base + MDP_PERIPH_TOP0_END, "top_2");
	} else {
		msm_disp_snapshot_add_block(disp_state, cat->mdp[0].len,
				dpu_kms->mmio + cat->mdp[0].base, "top");
	}

	/* dump CWB sub-blocks HW regs info */
	for (i = 0; i < cat->cwb_count; i++)
		msm_disp_snapshot_add_block(disp_state, cat->cwb[i].len,
					    dpu_kms->mmio + cat->cwb[i].base, cat->cwb[i].name);

	/* dump DSC sub-blocks HW regs info */
	for (i = 0; i < cat->dsc_count; i++) {
		base = dpu_kms->mmio + cat->dsc[i].base;
		msm_disp_snapshot_add_block(disp_state, cat->dsc[i].len, base,
					    "%s", cat->dsc[i].name);

		if (cat->mdss_ver->core_major_ver >= 7) {
			struct dpu_dsc_blk enc = cat->dsc[i].sblk->enc;
			struct dpu_dsc_blk ctl = cat->dsc[i].sblk->ctl;

			msm_disp_snapshot_add_block(disp_state, enc.len, base + enc.base, "%s_%s",
						    cat->dsc[i].name, enc.name);
			msm_disp_snapshot_add_block(disp_state, ctl.len, base + ctl.base, "%s_%s",
						    cat->dsc[i].name, ctl.name);
		}
	}

	if (cat->cdm)
		msm_disp_snapshot_add_block(disp_state, cat->cdm->len,
					    dpu_kms->mmio + cat->cdm->base,
					    "%s", cat->cdm->name);

	const struct dpu_vbif_cfg *vbif = dpu_kms->catalog->vbif;

	msm_disp_snapshot_add_block(disp_state, vbif->len,
				    dpu_kms->vbif,
				    "vbif");

	pm_runtime_put_sync(&dpu_kms->pdev->dev);
}

static const struct msm_kms_funcs kms_funcs = {
	.hw_init         = dpu_kms_hw_init,
	.irq_preinstall  = dpu_core_irq_preinstall,
	.irq_postinstall = dpu_irq_postinstall,
	.irq_uninstall   = dpu_core_irq_uninstall,
	.irq             = dpu_core_irq,
	.enable_commit   = dpu_kms_enable_commit,
	.disable_commit  = dpu_kms_disable_commit,
	.check_mode_changed = dpu_kms_check_mode_changed,
	.flush_commit    = dpu_kms_flush_commit,
	.wait_flush      = dpu_kms_wait_flush,
	.complete_commit = dpu_kms_complete_commit,
	.enable_vblank   = dpu_kms_enable_vblank,
	.disable_vblank  = dpu_kms_disable_vblank,
	.destroy         = dpu_kms_destroy,
	.snapshot        = dpu_kms_mdp_snapshot,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init    = dpu_kms_debugfs_init,
#endif
};

static void _dpu_kms_mmu_destroy(struct dpu_kms *dpu_kms)
{
	struct msm_mmu *mmu;

	if (!dpu_kms->base.vm)
		return;

	mmu = to_msm_vm(dpu_kms->base.vm)->mmu;

	mmu->funcs->detach(mmu);
	drm_gpuvm_put(dpu_kms->base.vm);

	dpu_kms->base.vm = NULL;
}

static int _dpu_kms_mmu_init(struct dpu_kms *dpu_kms)
{
	struct drm_gpuvm *vm;

	vm = msm_kms_init_vm(dpu_kms->dev, dpu_kms->dev->dev->parent);
	if (IS_ERR(vm))
		return PTR_ERR(vm);

	dpu_kms->base.vm = vm;

	return 0;
}

/**
 * dpu_kms_get_clk_rate() - get the clock rate
 * @dpu_kms:  pointer to dpu_kms structure
 * @clock_name: clock name to get the rate
 *
 * Return: current clock rate
 */
unsigned long dpu_kms_get_clk_rate(struct dpu_kms *dpu_kms, char *clock_name)
{
	struct clk *clk;

	clk = msm_clk_bulk_get_clock(dpu_kms->clocks, dpu_kms->num_clocks, clock_name);
	if (!clk)
		return 0;

	return clk_get_rate(clk);
}

#define	DPU_PERF_DEFAULT_MAX_CORE_CLK_RATE	412500000

static int dpu_kms_hw_init(struct msm_kms *kms)
{
	struct dpu_kms *dpu_kms;
	struct drm_device *dev;
	int rc = -EINVAL;
	unsigned long max_core_clk_rate;
	u32 core_rev;

	if (!kms) {
		DPU_ERROR("invalid kms\n");
		return rc;
	}

	dpu_kms = to_dpu_kms(kms);
	dev = dpu_kms->dev;

	dev->mode_config.cursor_width = 512;
	dev->mode_config.cursor_height = 512;

	drm_atomic_private_obj_init(dpu_kms->dev, &dpu_kms->global_state,
				    &dpu_kms_global_state_funcs);

	atomic_set(&dpu_kms->bandwidth_ref, 0);

	/*
	 * M2582: dump the display blocks exactly as the bootloader left them.
	 *
	 * The factory splash is a linear scanout and it looks correct on this
	 * panel, so these words are a known-good linear reference -- the one
	 * thing every earlier comparison lacked, because Android's own buffers
	 * are all UBWC.  Boot with m2582.features=mdssnoreset,bldump so the
	 * MDSS core reset does not wipe them first; the dispcc clocks stay up
	 * through clk_ignore_unused, so the reads below are safe without
	 * resuming the DPU.
	 */
	if (m2582_feature_opt_in("bldump")) {
		u32 off;

		for (off = 0; off < 0x300; off += 0x10)
			pr_info("m2582-bldump:sspp0 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x4000 + off),
				readl_relaxed(dpu_kms->mmio + 0x4000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x4000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x4000 + off + 12));
		for (off = 0; off < 0x300; off += 0x10)
			pr_info("m2582-bldump:dma0 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x24000 + off),
				readl_relaxed(dpu_kms->mmio + 0x24000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x24000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x24000 + off + 12));
		for (off = 0; off < 0x40; off += 0x10)
			pr_info("m2582-bldump:ctl0 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x15000 + off),
				readl_relaxed(dpu_kms->mmio + 0x15000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x15000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x15000 + off + 12));
		for (off = 0; off < 0x40; off += 0x10)
			pr_info("m2582-bldump:lm0 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x44000 + off),
				readl_relaxed(dpu_kms->mmio + 0x44000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x44000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x44000 + off + 12));
		for (off = 0; off < 0x40; off += 0x10)
			pr_info("m2582-bldump:lm1 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x45000 + off),
				readl_relaxed(dpu_kms->mmio + 0x45000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x45000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x45000 + off + 12));
		for (off = 0; off < 0x40; off += 0x10)
			pr_info("m2582-bldump:ctl1 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x16000 + off),
				readl_relaxed(dpu_kms->mmio + 0x16000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x16000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x16000 + off + 12));
		/* INTF1: the mode registers that say command vs video. */
		for (off = 0; off < 0x60; off += 0x10)
			pr_info("m2582-bldump:intf1 %03x %08x %08x %08x %08x\n", off,
				readl_relaxed(dpu_kms->mmio + 0x35000 + off),
				readl_relaxed(dpu_kms->mmio + 0x35000 + off + 4),
				readl_relaxed(dpu_kms->mmio + 0x35000 + off + 8),
				readl_relaxed(dpu_kms->mmio + 0x35000 + off + 12));
		pr_info("m2582-bldump: done\n");
	}

	rc = pm_runtime_resume_and_get(&dpu_kms->pdev->dev);
	if (rc < 0)
		goto error;

	core_rev = readl_relaxed(dpu_kms->mmio + 0x0);

	pr_info("dpu hardware revision:0x%x\n", core_rev);

	dpu_kms->catalog = of_device_get_match_data(dev->dev);
	if (!dpu_kms->catalog) {
		DPU_ERROR("device config not known!\n");
		rc = -EINVAL;
		goto err_pm_put;
	}

	/*
	 * Now we need to read the HW catalog and initialize resources such as
	 * clocks, regulators, GDSC/MMAGIC, ioremap the register ranges etc
	 */
	rc = _dpu_kms_mmu_init(dpu_kms);
	if (rc) {
		DPU_ERROR("dpu_kms_mmu_init failed: %d\n", rc);
		goto err_pm_put;
	}

	dpu_kms->mdss = qcom_ubwc_config_get_data();
	if (IS_ERR(dpu_kms->mdss)) {
		rc = PTR_ERR(dpu_kms->mdss);
		DPU_ERROR("failed to get UBWC config data: %d\n", rc);
		goto err_pm_put;
	}

	if (!dpu_kms->mdss) {
		rc = -EINVAL;
		DPU_ERROR("NULL MDSS data\n");
		goto err_pm_put;
	}

	rc = dpu_rm_init(dev, &dpu_kms->rm, dpu_kms->catalog, dpu_kms->mdss, dpu_kms->mmio);
	if (rc) {
		DPU_ERROR("rm init failed: %d\n", rc);
		goto err_pm_put;
	}

	dpu_kms->hw_mdp = dpu_hw_mdptop_init(dev,
					     dpu_kms->catalog->mdp,
					     dpu_kms->mmio,
					     dpu_kms->catalog->mdss_ver);
	if (IS_ERR(dpu_kms->hw_mdp)) {
		rc = PTR_ERR(dpu_kms->hw_mdp);
		DPU_ERROR("failed to get hw_mdp: %d\n", rc);
		dpu_kms->hw_mdp = NULL;
		goto err_pm_put;
	}

	/*
	 * M2582: bring up the regDMA engine's op buffers.  Optional: boards
	 * without a "reg_dma" window, or a failed allocation, simply leave the
	 * colour blocks on the direct-register path.
	 */
	if (dpu_kms->reg_dma && m2582_feature_opt_in("regdmatest")) {
		dpu_kms->regdma = dpu_reg_dma_init(dpu_kms);
		if (IS_ERR(dpu_kms->regdma)) {
			DPU_ERROR("regdma init failed, colour blocks stay direct\n");
			dpu_kms->regdma = NULL;
		}
	}

	struct dpu_hw_vbif *hw;
	const struct dpu_vbif_cfg *vbif = dpu_kms->catalog->vbif;

	hw = dpu_hw_vbif_init(dev, vbif, dpu_kms->vbif);
	if (IS_ERR(hw)) {
		rc = PTR_ERR(hw);
		DPU_ERROR("failed to init vbif: %d\n", rc);
		goto err_pm_put;
	}

	dpu_kms->hw_vbif = hw;

	/*
	 * M2582: program every XIN's real-time QoS remap and read
	 * outstanding-transaction limit the way the factory's device tree does.
	 * Upstream applies these only from the writeback encoder, which leaves
	 * this board's command-mode panel reading its frames out with the reset
	 * priorities.  The values are in tuna_vbif
	 * (catalog/dpu_12_3_tuna.h).
	 */
	if (hw->ops.set_qos_remap) {
		struct dpu_vbif_set_qos_params qos = { .is_rt = true };
		int xin;

		for (xin = 0; xin < MAX_XIN_COUNT; xin++) {
			qos.xin_id = xin;
			qos.num = xin;
			dpu_vbif_set_qos_remap(dpu_kms, &qos);
		}
	}

	if (hw->ops.set_limit_conf && hw->ops.set_halt_ctrl) {
		struct dpu_vbif_set_ot_params ot = { .rd = true };
		int xin;

		for (xin = 0; xin < MAX_XIN_COUNT; xin++) {
			ot.xin_id = xin;
			ot.num = xin;
			ot.width = 1200;
			ot.height = 2670;
			ot.frame_rate = 120;
			dpu_vbif_set_ot_limit(dpu_kms, &ot);
		}
	}

	/* TODO: use the same max_freq as in dpu_kms_hw_init */
	max_core_clk_rate = dpu_kms_get_clk_rate(dpu_kms, "core");
	if (!max_core_clk_rate) {
		DPU_DEBUG("max core clk rate not determined, using default\n");
		max_core_clk_rate = DPU_PERF_DEFAULT_MAX_CORE_CLK_RATE;
	}

	rc = dpu_core_perf_init(&dpu_kms->perf, dpu_kms->catalog->perf, max_core_clk_rate);
	if (rc) {
		DPU_ERROR("failed to init perf %d\n", rc);
		goto err_pm_put;
	}

	/*
	 * We need to program DP <-> PHY relationship only for SC8180X since it
	 * has fewer DP controllers than DP PHYs.
	 * If any other platform requires the same kind of programming, or if
	 * the INTF <->DP relationship isn't static anymore, this needs to be
	 * configured through the DT.
	 */
	if (of_device_is_compatible(dpu_kms->pdev->dev.of_node, "qcom,sc8180x-dpu"))
		dpu_kms->hw_mdp->ops.dp_phy_intf_sel(dpu_kms->hw_mdp, (unsigned int[]){ 1, 2, });

	dpu_kms->hw_intr = dpu_hw_intr_init(dev, dpu_kms->mmio, dpu_kms->catalog);
	if (IS_ERR(dpu_kms->hw_intr)) {
		rc = PTR_ERR(dpu_kms->hw_intr);
		DPU_ERROR("hw_intr init failed: %d\n", rc);
		dpu_kms->hw_intr = NULL;
		goto err_pm_put;
	}

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = DPU_MAX_IMG_WIDTH;
	dev->mode_config.max_height = DPU_MAX_IMG_HEIGHT;

	dev->max_vblank_count = 0xffffffff;
	/* Disable vblank irqs aggressively for power-saving */
	dev->vblank_disable_immediate = true;

	/*
	 * _dpu_kms_drm_obj_init should create the DRM related objects
	 * i.e. CRTCs, planes, encoders, connectors and so forth
	 */
	rc = _dpu_kms_drm_obj_init(dpu_kms);
	if (rc) {
		DPU_ERROR("modeset init failed: %d\n", rc);
		goto err_pm_put;
	}

		dpu_vbif_init_memtypes(dpu_kms);

	pm_runtime_put_sync(&dpu_kms->pdev->dev);

	return 0;

err_pm_put:
	pm_runtime_put_sync(&dpu_kms->pdev->dev);
error:
	_dpu_kms_hw_destroy(dpu_kms);

	return rc;
}

static int dpu_kms_init(struct drm_device *ddev)
{
	struct msm_drm_private *priv = ddev->dev_private;
	struct device *dev = ddev->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	struct dev_pm_opp *opp;
	int ret = 0;
	unsigned long max_freq = ULONG_MAX;

	opp = dev_pm_opp_find_freq_floor(dev, &max_freq);
	if (!IS_ERR(opp))
		dev_pm_opp_put(opp);

	dev_pm_opp_set_rate(dev, max_freq);

	ret = msm_kms_init(&dpu_kms->base, &kms_funcs);
	if (ret) {
		DPU_ERROR("failed to init kms, ret=%d\n", ret);
		return ret;
	}
	dpu_kms->dev = ddev;

	pm_runtime_enable(&pdev->dev);
	dpu_kms->rpm_enabled = true;

	return 0;
}

static int dpu_kms_mmap_mdp5(struct dpu_kms *dpu_kms)
{
	struct platform_device *pdev = dpu_kms->pdev;
	struct platform_device *mdss_dev;
	int ret;

	if (!dev_is_platform(dpu_kms->pdev->dev.parent))
		return -EINVAL;

	mdss_dev = to_platform_device(dpu_kms->pdev->dev.parent);

	dpu_kms->mmio = msm_ioremap(pdev, "mdp_phys");
	if (IS_ERR(dpu_kms->mmio)) {
		ret = PTR_ERR(dpu_kms->mmio);
		DPU_ERROR("mdp register memory map failed: %d\n", ret);
		dpu_kms->mmio = NULL;
		return ret;
	}
	DRM_DEBUG("mapped dpu address space @%p\n", dpu_kms->mmio);

	dpu_kms->vbif = msm_ioremap_mdss(mdss_dev, dpu_kms->pdev, "vbif_phys");
	if (IS_ERR(dpu_kms->vbif)) {
		ret = PTR_ERR(dpu_kms->vbif);
		DPU_ERROR("vbif register memory map failed: %d\n", ret);
		dpu_kms->vbif = NULL;
		return ret;
	}

	return 0;
}

static int dpu_kms_mmap_dpu(struct dpu_kms *dpu_kms)
{
	struct platform_device *pdev = dpu_kms->pdev;
	int ret;

	dpu_kms->mmio = msm_ioremap(pdev, "mdp");
	if (IS_ERR(dpu_kms->mmio)) {
		ret = PTR_ERR(dpu_kms->mmio);
		DPU_ERROR("mdp register memory map failed: %d\n", ret);
		dpu_kms->mmio = NULL;
		return ret;
	}
	DRM_DEBUG("mapped dpu address space @%p\n", dpu_kms->mmio);

	/*
	 * Optional: boards that describe the regDMA window get it mapped, and
	 * the colour blocks then have a way to reach the datapath.
	 */
	dpu_kms->reg_dma = msm_ioremap(pdev, "reg_dma");
	if (IS_ERR(dpu_kms->reg_dma)) {
		dpu_kms->reg_dma = NULL;
		DRM_DEBUG("no reg_dma register window\n");
	}

	dpu_kms->vbif = msm_ioremap(pdev, "vbif");
	if (IS_ERR(dpu_kms->vbif)) {
		ret = PTR_ERR(dpu_kms->vbif);
		DPU_ERROR("vbif register memory map failed: %d\n", ret);
		dpu_kms->vbif = NULL;
		return ret;
	}

	return 0;
}

static int dpu_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dpu_kms *dpu_kms;
	int irq;
	int ret = 0;

	if (!msm_disp_drv_should_bind(&pdev->dev, true))
		return -ENODEV;

	dpu_kms = devm_kzalloc(dev, sizeof(*dpu_kms), GFP_KERNEL);
	if (!dpu_kms)
		return -ENOMEM;

	dpu_kms->pdev = pdev;

	ret = devm_pm_opp_set_clkname(dev, "core");
	if (ret)
		return ret;
	/* OPP table is optional */
	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "invalid OPP table in device tree\n");

	ret = devm_clk_bulk_get_all(&pdev->dev, &dpu_kms->clocks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to parse clocks\n");

	dpu_kms->num_clocks = ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get irq\n");

	dpu_kms->base.irq = irq;

	if (of_device_is_compatible(dpu_kms->pdev->dev.of_node, "qcom,mdp5"))
		ret = dpu_kms_mmap_mdp5(dpu_kms);
	else
		ret = dpu_kms_mmap_dpu(dpu_kms);
	if (ret)
		return ret;

	ret = dpu_kms_parse_data_bus_icc_path(dpu_kms);
	if (ret)
		return ret;

	return msm_drv_probe(&pdev->dev, dpu_kms_init, &dpu_kms->base);
}

static void dpu_dev_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &msm_drm_ops);
}

static int __maybe_unused dpu_runtime_suspend(struct device *dev)
{
	int i;
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);

	clk_bulk_disable_unprepare(dpu_kms->num_clocks, dpu_kms->clocks);

	for (i = 0; i < dpu_kms->num_paths; i++)
		icc_set_bw(dpu_kms->path[i], 0, 0);

	return 0;
}

static int __maybe_unused dpu_runtime_resume(struct device *dev)
{
	int rc = -1;
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_drm_private *priv = platform_get_drvdata(pdev);
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	struct drm_encoder *encoder;
	struct drm_device *ddev;

	ddev = dpu_kms->dev;

	rc = clk_bulk_prepare_enable(dpu_kms->num_clocks, dpu_kms->clocks);
	if (rc) {
		DPU_ERROR("clock enable failed rc:%d\n", rc);
		return rc;
	}

	dpu_vbif_init_memtypes(dpu_kms);

	drm_for_each_encoder(encoder, ddev)
		dpu_encoder_virt_runtime_resume(encoder);

	return rc;
}

static const struct dev_pm_ops dpu_pm_ops = {
	SET_RUNTIME_PM_OPS(dpu_runtime_suspend, dpu_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	.prepare = msm_kms_pm_prepare,
	.complete = msm_kms_pm_complete,
};

static const struct of_device_id dpu_dt_match[] = {
	{ .compatible = "qcom,tuna-dpu", .data = &dpu_tuna_cfg, },
	{ .compatible = "qcom,eliza-dpu", .data = &dpu_eliza_cfg, },
	{ .compatible = "qcom,glymur-dpu", .data = &dpu_glymur_cfg, },
	{ .compatible = "qcom,kaanapali-dpu", .data = &dpu_kaanapali_cfg, },
	{ .compatible = "qcom,milos-dpu", .data = &dpu_milos_cfg, },
	{ .compatible = "qcom,msm8917-mdp5", .data = &dpu_msm8917_cfg, },
	{ .compatible = "qcom,msm8937-mdp5", .data = &dpu_msm8937_cfg, },
	{ .compatible = "qcom,msm8953-mdp5", .data = &dpu_msm8953_cfg, },
	{ .compatible = "qcom,msm8996-mdp5", .data = &dpu_msm8996_cfg, },
	{ .compatible = "qcom,msm8998-dpu", .data = &dpu_msm8998_cfg, },
	{ .compatible = "qcom,qcm2290-dpu", .data = &dpu_qcm2290_cfg, },
	{ .compatible = "qcom,sa8775p-dpu", .data = &dpu_sa8775p_cfg, },
	{ .compatible = "qcom,sar2130p-dpu", .data = &dpu_sar2130p_cfg, },
	{ .compatible = "qcom,sdm630-mdp5", .data = &dpu_sdm630_cfg, },
	{ .compatible = "qcom,sdm660-mdp5", .data = &dpu_sdm660_cfg, },
	{ .compatible = "qcom,sdm670-dpu", .data = &dpu_sdm670_cfg, },
	{ .compatible = "qcom,sdm845-dpu", .data = &dpu_sdm845_cfg, },
	{ .compatible = "qcom,sc7180-dpu", .data = &dpu_sc7180_cfg, },
	{ .compatible = "qcom,sc7280-dpu", .data = &dpu_sc7280_cfg, },
	{ .compatible = "qcom,sc8180x-dpu", .data = &dpu_sc8180x_cfg, },
	{ .compatible = "qcom,sc8280xp-dpu", .data = &dpu_sc8280xp_cfg, },
	{ .compatible = "qcom,sm6115-dpu", .data = &dpu_sm6115_cfg, },
	{ .compatible = "qcom,sm6125-dpu", .data = &dpu_sm6125_cfg, },
	{ .compatible = "qcom,sm6150-dpu", .data = &dpu_sm6150_cfg, },
	{ .compatible = "qcom,sm6350-dpu", .data = &dpu_sm6350_cfg, },
	{ .compatible = "qcom,sm6375-dpu", .data = &dpu_sm6375_cfg, },
	{ .compatible = "qcom,sm7150-dpu", .data = &dpu_sm7150_cfg, },
	{ .compatible = "qcom,sm8150-dpu", .data = &dpu_sm8150_cfg, },
	{ .compatible = "qcom,sm8250-dpu", .data = &dpu_sm8250_cfg, },
	{ .compatible = "qcom,sm8350-dpu", .data = &dpu_sm8350_cfg, },
	{ .compatible = "qcom,sm8450-dpu", .data = &dpu_sm8450_cfg, },
	{ .compatible = "qcom,sm8550-dpu", .data = &dpu_sm8550_cfg, },
	{ .compatible = "qcom,sm8650-dpu", .data = &dpu_sm8650_cfg, },
	{ .compatible = "qcom,sm8750-dpu", .data = &dpu_sm8750_cfg, },
	{ .compatible = "qcom,x1e80100-dpu", .data = &dpu_x1e80100_cfg, },
	{}
};
MODULE_DEVICE_TABLE(of, dpu_dt_match);

/*
 * M2582 probe: one-word write into the DPU register file, so a single live
 * hypothesis can be tested by writing a register and looking at the panel
 * instead of rebuilding and reflashing the board.  Reads come from the
 * regset32 windows above, which sample at cat time.
 *
 * Write "<hex offset> <hex value>" to m2582-poke.  The offset is relative to
 * the DPU register base and is rejected outside the mapped window and off a
 * word boundary, which is the whole safety story: this never reaches a block
 * the driver has not already mapped (see the devmem rule in AGENTS.md §5.2).
 *
 * This is a probe, not a feature: it goes away once the row geometry is fixed.
 */
static ssize_t m2582_poke_write(struct file *f, const char __user *ubuf,
				size_t len, loff_t *off)
{
	struct dpu_kms *dpu_kms = f->private_data;
	char buf[64];
	unsigned int reg, val;
	int n;

	if (!dpu_kms || len == 0 || len >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	n = sscanf(buf, "%x %x", &reg, &val);
	if (n < 1)
		return -EINVAL;

	/* the mdp window is 0x92800 long; nothing past it is mapped */
	if ((reg & 3) || reg >= 0x92800)
		return -EINVAL;

	/*
	 * One word reads back with "<offset>" alone.  The blocks this driver has
	 * no regset for (the DSPP sub-blocks -- SPR and demura -- and anything
	 * else the catalogue does not describe) can only be inspected this way,
	 * and knowing their power-on state is the difference between "we wrote
	 * it" and "it was already like that".
	 *
	 * The bounds check only proves the offset is inside the mapped window:
	 * the window also contains reset and clock-force registers, so a write
	 * here can still take the display down.  It is a bring-up tool.
	 */
	/* touching MMIO while the DPU is runtime suspended hangs the bus */
	if (pm_runtime_get_sync(&dpu_kms->pdev->dev) < 0) {
		pm_runtime_put_noidle(&dpu_kms->pdev->dev);
		return -EIO;
	}

	if (n == 1) {
		pr_info("m2582-poke: %05x = %08x\n", reg,
			readl_relaxed(dpu_kms->mmio + reg));
	} else {
		writel_relaxed(val, dpu_kms->mmio + reg);
		/* the DPU latches on the next commit; make it visible first */
		wmb();
	}

	pm_runtime_put(&dpu_kms->pdev->dev);

	return len;
}

static const struct file_operations m2582_poke_fops = {
	.open = simple_open,
	.write = m2582_poke_write,
	.llseek = noop_llseek,
};

/*
 * M2582: write the SPR control word that dpu_setup_dspp_spr() will use on the
 * next commit that programs the block.  Lets one boot enumerate candidate
 * words (the factory builds this from userspace and the panel never got a
 * working one from us).
 */
static ssize_t m2582_spr_write(struct file *f, const char __user *ubuf,
			       size_t len, loff_t *off)
{
	struct dpu_kms *dpu_kms = f->private_data;
	char buf[32];
	u32 cfg;

	if (!dpu_kms || len == 0 || len >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	if (sscanf(buf, "%x", &cfg) != 1)
		return -EINVAL;

	m2582_spr_override_cfg(cfg);
	pr_info("m2582-spr: control word override %08x\n", cfg);

	return len;
}

static const struct file_operations m2582_spr_fops = {
	.open = simple_open,
	.write = m2582_spr_write,
	.llseek = noop_llseek,
};

/* M2582: read-only full dump of the VBIF register window (0x0..0x2000).
 * Lets one diff the active VBIF against the factory vbif_rt.txt. */
static int m2582_vbifdump_show(struct seq_file *s, void *unused)
{
	struct dpu_kms *dpu_kms = s->private;
	u32 off;

	if (!dpu_kms || !dpu_kms->vbif)
		return -ENODEV;

	/* touching MMIO while the DPU is runtime suspended hangs the bus */
	if (pm_runtime_get_sync(&dpu_kms->pdev->dev) < 0) {
		pm_runtime_put_noidle(&dpu_kms->pdev->dev);
		return -EIO;
	}

	for (off = 0; off < 0x2000; off += 16) {
		seq_printf(s, "0x%08x: %08x %08x %08x %08x\n", off,
			   readl_relaxed(dpu_kms->vbif + off + 0x0),
			   readl_relaxed(dpu_kms->vbif + off + 0x4),
			   readl_relaxed(dpu_kms->vbif + off + 0x8),
			   readl_relaxed(dpu_kms->vbif + off + 0xc));
	}
	/* tail words inside the 0x2008 window */
	seq_printf(s, "0x00002000: %08x %08x\n",
		   readl_relaxed(dpu_kms->vbif + 0x2000),
		   readl_relaxed(dpu_kms->vbif + 0x2004));

	pm_runtime_put(&dpu_kms->pdev->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(m2582_vbifdump);

/* M2582: poke a single VBIF window register (like m2582-poke but on the
 * 0xaeb0000 window). Read-only diagnostic; the window is 0x2008 long. */
static ssize_t m2582_vbifpoke_write(struct file *f, const char __user *ubuf,
				    size_t len, loff_t *off)
{
	struct dpu_kms *dpu_kms = f->private_data;
	char buf[64];
	unsigned int reg, val;
	int n;

	if (!dpu_kms || !dpu_kms->vbif || len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	n = sscanf(buf, "%x %x", &reg, &val);
	if (n < 1)
		return -EINVAL;
	if ((reg & 3) || reg >= 0x2008)
		return -EINVAL;

	if (pm_runtime_get_sync(&dpu_kms->pdev->dev) < 0) {
		pm_runtime_put_noidle(&dpu_kms->pdev->dev);
		return -EIO;
	}
	if (n == 1)
		pr_info("m2582-vbifpoke: %05x = %08x\n", reg,
			readl_relaxed(dpu_kms->vbif + reg));
	else
		writel_relaxed(val, dpu_kms->vbif + reg);
	pm_runtime_put(&dpu_kms->pdev->dev);

	return len;
}

static const struct file_operations m2582_vbifpoke_fops = {
	.open = simple_open,
	.write = m2582_vbifpoke_write,
	.llseek = noop_llseek,
};

/* Factory VBIF debug bus, temporary opt-in signal discovery probe.
 * Samples are opaque: they are not AXI transaction counts.
 */
static DEFINE_MUTEX(m2582_vbif_bus_lock);
static u32 m2582_vbif_bus_full;
static u32 m2582_vbif_bus_dwell_us = 35000;
static u32 m2582_vbif_bus_target = 0xffffffff;
static bool m2582_vbif_bus_trace;

struct m2582_vbif_sample {
	u32 value;
	u32 repeats;
	u64 ns;
};

static int m2582_vbif_bus_show(struct seq_file *s, void *unused)
{
	static const u32 regs[] = { 0x210, 0x214, 0x218, 0x21c, 0x220 };
	static const u8 blocks[][8] = {
		{ 0, 1, 4, 5, 12, 13, 0xff, 0xff },
		{ 0, 1, 4, 5, 12, 13, 16, 17 },
	};
	struct dpu_kms *kms = s->private;
	u32 saved[ARRAY_SIZE(regs)], clkon;
	int bus, b, id, i, ret;
	u32 full = READ_ONCE(m2582_vbif_bus_full);
	u32 dwell = clamp_t(u32, READ_ONCE(m2582_vbif_bus_dwell_us), 20, 35000);
	u32 target = READ_ONCE(m2582_vbif_bus_target);
	struct m2582_vbif_sample *trace = NULL;
	bool tracing = READ_ONCE(m2582_vbif_bus_trace);

	if (!kms || !kms->vbif)
		return -ENODEV;
	if (tracing) {
		if (target == 0xffffffff)
			return -EINVAL;
		trace = kvcalloc(4096, sizeof(*trace), GFP_KERNEL);
		if (!trace)
			return -ENOMEM;
	}
	ret = pm_runtime_resume_and_get(&kms->pdev->dev);
	if (ret < 0) {
		kvfree(trace);
		return ret;
	}
	mutex_lock(&m2582_vbif_bus_lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		saved[i] = readl_relaxed(kms->vbif + regs[i]);
	/* Refuse to disturb another owner of either selector. */
	if (saved[0] || saved[1] || saved[3]) {
		seq_puts(s, "debug bus busy; no changes made\n");
		goto out;
	}
	clkon = readl_relaxed(kms->vbif + 0x4);
	writel_relaxed(clkon | BIT(1), kms->vbif + 0x4);
	wmb();
	for (bus = 0; bus < 2; bus++) {
		u32 ctrl = bus ? 0x21c : 0x214;

		writel_relaxed(0, kms->vbif + (bus ? 0x214 : 0x21c));
		writel_relaxed(1, kms->vbif + 0x210);
		wmb();
		for (b = 0; b < (full ? (bus ? 18 : 15) : ARRAY_SIZE(blocks[bus])); b++) {
			u32 block = full ? b : blocks[bus][b];

			if (block == 0xff)
				continue;
			if (target != 0xffffffff &&
			    (((target >> 24) & 0xff) != bus + 1 ||
			     ((target >> 16) & 0xff) != block))
				continue;
			writel_relaxed(BIT(block), kms->vbif + ctrl);
			for (id = 0; id < (full ? 512 : (bus ? 16 : 12)); id++) {
				u32 both = ~0U, either = 0, first, last, count = 0;
				u32 test = id;
				u64 deadline;
				u64 began;
				u32 stored = 0, changes = 0, previous = 0;

				if (target != 0xffffffff && (target & 0xffff) != id)
					continue;
				writel_relaxed(test, kms->vbif + ctrl + 4);
				wmb();
				first = readl_relaxed(kms->vbif + 0x230);
				last = first;
				began = ktime_get_ns();
				deadline = began + (u64)dwell * 1000;
				do {
					last = readl_relaxed(kms->vbif + 0x230);
					if (trace) {
						if (!count || last != previous) {
							changes++;
							if (stored < 4096) {
								trace[stored].value = last;
								trace[stored].repeats = 1;
								trace[stored++].ns = ktime_get_ns() - began;
							}
						} else if (changes <= 4096) {
							trace[stored - 1].repeats++;
						}
						previous = last;
					}
					both &= last;
					either |= last;
					count++;
					if (!trace)
						udelay(5);
					if (!(count & 127))
						cond_resched();
				} while (ktime_get_ns() < deadline);
				seq_printf(s, "%d %u %d %08x %08x %08x %08x %u\n",
					   bus + 1, block, test,
					   first, last, both, either, count);
				if (trace) {
					u32 j;

					seq_printf(s, "trace changes=%u stored=%u\n", changes, stored);
					for (j = 0; j < stored; j++)
						seq_printf(s, "sample %llu %08x %u\n",
							   trace[j].ns, trace[j].value,
							   trace[j].repeats);
				}
			}
		}
		writel_relaxed(0, kms->vbif + ctrl);
		writel_relaxed(0, kms->vbif + ctrl + 4);
		wmb();
	}
	/* Restore selectors before the output enable. */
	for (i = 1; i < ARRAY_SIZE(regs); i++)
		writel_relaxed(saved[i], kms->vbif + regs[i]);
	writel_relaxed(saved[0], kms->vbif + regs[0]);
	wmb();
	writel_relaxed(clkon, kms->vbif + 0x4);
	wmb();
out:
	mutex_unlock(&m2582_vbif_bus_lock);
	pm_runtime_put(&kms->pdev->dev);
	kvfree(trace);
	return 0;
}
static int m2582_vbif_bus_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, m2582_vbif_bus_show, inode->i_private, 2 * 1024 * 1024);
}

static const struct file_operations m2582_vbif_bus_fops = {
	.owner = THIS_MODULE,
	.open = m2582_vbif_bus_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* Temporary SSPP signal discovery, using the M2582 factory debug-bus layout. */
static u32 m2582_sde_bus_target = U32_MAX;
static u32 m2582_sde_bus_dwell_us = 35000;
static bool m2582_sde_bus_c00_route;

static u32 m2582_sde_test_select(u32 block, u32 test)
{
	return (block << 4) | ((test & 7) << 1) |
		((test >> 3) << 24) | BIT(0);
}

static int m2582_sde_bus_show(struct seq_file *s, void *unused)
{
	static const u32 regs[] = { 0x188, 0x298, 0x194, 0x348, 0x384 };
	struct dpu_kms *kms = s->private;
	u32 target = READ_ONCE(m2582_sde_bus_target);
	u32 route = READ_ONCE(m2582_sde_bus_c00_route) ? 0x384 : 0x348;
	u32 dwell = clamp_t(u32, READ_ONCE(m2582_sde_bus_dwell_us), 20, 35000);
	u32 saved[ARRAY_SIZE(regs)];
	u32 selector, block, test, i;
	int ret;

	if (!kms || !kms->mmio)
		return -ENODEV;
	if (target != U32_MAX &&
	    ((target >> 24) > 2 || ((target >> 16) & 0xff) >= 74 ||
	     (target & 0xffff) >= 32 ||
	     ((target >> 24) == 2 && ((target >> 16) & 0xff) >= 63)))
		return -EINVAL;
	ret = pm_runtime_resume_and_get(&kms->pdev->dev);
	if (ret < 0)
		return ret;
	mutex_lock(&m2582_vbif_bus_lock);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		saved[i] = readl_relaxed(kms->mmio + regs[i]);
		if (saved[i] & BIT(0)) {
			seq_puts(s, "debug bus busy; no changes made\n");
			goto out;
		}
	}
	for (selector = 0; selector < (target == U32_MAX ? 2 : 3); selector++) {
		if (target != U32_MAX && selector != target >> 24)
			continue;
		for (block = 0; block < (selector == 2 ? 63 : 74); block++) {
			if (target != U32_MAX && block != ((target >> 16) & 0xff))
				continue;
			/* Factory limited SSPP table; targeted reads allow all valid IDs. */
			if (target == U32_MAX && !(block < 9 ||
			    (block >= 20 && block < 54) ||
			    (block >= 60 && block < 64) || block >= 70))
				continue;
			for (test = 0; test < (target == U32_MAX ? 8 : 32); test++) {
				u32 first, last, both = U32_MAX, either = 0, count = 0;
				u64 deadline;

				if (target != U32_MAX && test != (target & 0xffff))
					continue;
				writel_relaxed(m2582_sde_test_select(block, test),
					       kms->mmio + regs[selector]);
				/* 348 matches the shipped callback; 384 is the calibrated C00 route. */
				writel_relaxed(0x7001, kms->mmio + route);
				wmb();
				first = readl_relaxed(kms->mmio + 0x388);
				last = first;
				deadline = ktime_get_ns() + (u64)dwell * 1000;
				do {
					last = readl_relaxed(kms->mmio + 0x388);
					both &= last;
					either |= last;
					count++;
					udelay(5);
					if (!(count & 127))
						cond_resched();
				} while (ktime_get_ns() < deadline);
				seq_printf(s, "%u %u %u %08x %08x %08x %08x %u\n",
					   selector, block, test, first, last, both, either, count);
			}
		}
		writel_relaxed(saved[selector], kms->mmio + regs[selector]);
		wmb();
	}
	for (i = 0; i < ARRAY_SIZE(regs); i++)
		writel_relaxed(saved[i], kms->mmio + regs[i]);
	wmb();
out:
	mutex_unlock(&m2582_vbif_bus_lock);
	pm_runtime_put(&kms->pdev->dev);
	return 0;
}
static int m2582_sde_bus_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, m2582_sde_bus_show, inode->i_private, SZ_128K);
}

static const struct file_operations m2582_sde_bus_fops = {
	.owner = THIS_MODULE,
	.open = m2582_sde_bus_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * M2582: fill the bootloader splash region (0xfc800000) with white, so that
 * with m2582.features=...,splashsrc the DPU fetches a known pattern from the
 * exact physical address the bootloader scanned out cleanly.  Used to decide
 * whether the address/region (vs the fetch config) is what separates the
 * bootloader's clean splash from our broken fetch.
 */
static ssize_t m2582_splash_write(struct file *f, const char __user *ubuf,
				  size_t len, loff_t *off)
{
	const size_t bytes = 2670u * 4864u; /* 1200x2670 XR24 @ pitch 4864 */
	void *va;
	u32 *p;
	size_t i;

	va = memremap(0xfc800000, bytes, MEMREMAP_WB);
	if (!va)
		return -ENOMEM;

	for (p = va, i = 0; i < bytes / 4; i++)
		p[i] = 0xffffff;

	memunmap(va);
	pr_info("m2582-splash: filled 0xfc800000 + %zu bytes with white\n",
		bytes);

	return len;
}

static const struct file_operations m2582_splash_fops = {
	.open = simple_open,
	.write = m2582_splash_write,
	.llseek = noop_llseek,
};

static void m2582_poke_init(struct dentry *entry, struct dpu_kms *dpu_kms)
{
	debugfs_create_file("m2582-poke", 0600, entry, dpu_kms,
			    &m2582_poke_fops);
	debugfs_create_file("m2582_spr", 0200, entry, dpu_kms,
			    &m2582_spr_fops);
	debugfs_create_file("m2582_vbifdump", 0400, entry, dpu_kms,
			    &m2582_vbifdump_fops);
	debugfs_create_file("m2582_vbifpoke", 0600, entry, dpu_kms,
			    &m2582_vbifpoke_fops);
	if (m2582_feature_opt_in("vbifbus")) {
		debugfs_create_file("m2582_sde_bus", 0400, entry, dpu_kms,
				    &m2582_sde_bus_fops);
		debugfs_create_u32("m2582_sde_bus_target", 0600, entry,
				   &m2582_sde_bus_target);
		debugfs_create_bool("m2582_sde_bus_c00_route", 0600, entry,
				    &m2582_sde_bus_c00_route);
		debugfs_create_u32("m2582_sde_bus_dwell_us", 0600, entry,
				   &m2582_sde_bus_dwell_us);
		debugfs_create_file("m2582_vbif_bus", 0400, entry, dpu_kms,
				    &m2582_vbif_bus_fops);
		debugfs_create_u32("m2582_vbif_bus_full", 0600, entry,
				   &m2582_vbif_bus_full);
		debugfs_create_u32("m2582_vbif_bus_dwell_us", 0600, entry,
				   &m2582_vbif_bus_dwell_us);
		debugfs_create_u32("m2582_vbif_bus_target", 0600, entry,
				   &m2582_vbif_bus_target);
		debugfs_create_bool("m2582_vbif_bus_trace", 0600, entry,
				    &m2582_vbif_bus_trace);
	}
	debugfs_create_file("m2582_coreclk", 0200, entry, dpu_kms,
			    &m2582_coreclk_fops);
	debugfs_create_file("m2582_splash", 0200, entry, dpu_kms,
			    &m2582_splash_fops);
}

static struct platform_driver dpu_driver = {
	.probe = dpu_dev_probe,
	.remove = dpu_dev_remove,
	.shutdown = msm_kms_shutdown,
	.driver = {
		.name = "msm_dpu",
		.of_match_table = dpu_dt_match,
		.pm = &dpu_pm_ops,
	},
};

void __init msm_dpu_register(void)
{
	platform_driver_register(&dpu_driver);
}

void __exit msm_dpu_unregister(void)
{
	platform_driver_unregister(&dpu_driver);
}
