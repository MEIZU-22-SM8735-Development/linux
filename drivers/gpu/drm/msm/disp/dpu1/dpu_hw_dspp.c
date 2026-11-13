// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */

#include <drm/drm_managed.h>

#include "dpu_hwio.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_lm.h"
#include "dpu_hw_dspp.h"
#include <linux/meizu-m2582.h>

#include "dpu_kms.h"


/* DSPP_PCC */
#define PCC_EN BIT(0)
#define PCC_DIS 0
#define PCC_RED_R_OFF 0x10
#define PCC_RED_G_OFF 0x1C
#define PCC_RED_B_OFF 0x28
#define PCC_GREEN_R_OFF 0x14
#define PCC_GREEN_G_OFF 0x20
#define PCC_GREEN_B_OFF 0x2C
#define PCC_BLUE_R_OFF 0x18
#define PCC_BLUE_G_OFF 0x24
#define PCC_BLUE_B_OFF 0x30

/* DSPP_GC */
#define GC_EN BIT(0)
#define GC_DIS 0
#define GC_8B_ROUND_EN BIT(1)
#define GC_LUT_SWAP_OFF 0x1c
#define GC_C0_OFF 0x4
#define GC_C1_OFF 0xc
#define GC_C2_OFF 0x14
#define GC_C0_INDEX_OFF 0x8
#define GC_C1_INDEX_OFF 0x10
#define GC_C2_INDEX_OFF 0x18

static void dpu_setup_dspp_pcc(struct dpu_hw_dspp *ctx,
		struct dpu_hw_pcc_cfg *cfg)
{

	u32 base;

	if (!ctx) {
		DRM_ERROR("invalid ctx %p\n", ctx);
		return;
	}

	base = ctx->cap->sblk->pcc.base;

	if (!base) {
		DRM_ERROR("invalid ctx %p pcc base 0x%x\n", ctx, base);
		return;
	}

	if (!cfg) {
		DRM_DEBUG_DRIVER("disable pcc feature\n");
		DPU_REG_WRITE(&ctx->hw, base, PCC_DIS);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_R_OFF, cfg->r.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_G_OFF, cfg->r.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_RED_B_OFF, cfg->r.b);

	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_R_OFF, cfg->g.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_G_OFF, cfg->g.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_GREEN_B_OFF, cfg->g.b);

	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_R_OFF, cfg->b.r);
	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_G_OFF, cfg->b.g);
	DPU_REG_WRITE(&ctx->hw, base + PCC_BLUE_B_OFF, cfg->b.b);

	DPU_REG_WRITE(&ctx->hw, base, PCC_EN);
}

static void dpu_setup_dspp_gc(struct dpu_hw_dspp *ctx,
		struct dpu_hw_gc_lut *gc_lut)
{
	int i = 0;
	u32 base, reg;

	if (!ctx) {
		DRM_ERROR("invalid ctx\n");
		return;
	}

	base = ctx->cap->sblk->gc.base;

	if (!base) {
		DRM_ERROR("invalid ctx %p gc base\n", ctx);
		return;
	}

	if (!gc_lut) {
		DRM_DEBUG_DRIVER("disable gc feature\n");
		DPU_REG_WRITE(&ctx->hw, base, GC_DIS);
		return;
	}

	DPU_REG_WRITE(&ctx->hw, base + GC_C0_INDEX_OFF, 0);
	DPU_REG_WRITE(&ctx->hw, base + GC_C1_INDEX_OFF, 0);
	DPU_REG_WRITE(&ctx->hw, base + GC_C2_INDEX_OFF, 0);

	for (i = 0; i < PGC_TBL_LEN; i++) {
		DPU_REG_WRITE(&ctx->hw, base + GC_C0_OFF, gc_lut->c0[i]);
		DPU_REG_WRITE(&ctx->hw, base + GC_C1_OFF, gc_lut->c1[i]);
		DPU_REG_WRITE(&ctx->hw, base + GC_C2_OFF, gc_lut->c2[i]);
	}

	DPU_REG_WRITE(&ctx->hw, base + GC_LUT_SWAP_OFF, BIT(0));

	reg = GC_EN | ((gc_lut->flags & PGC_8B_ROUND) ? GC_8B_ROUND_EN : 0);
	DPU_REG_WRITE(&ctx->hw, base, reg);
}

/**
 * dpu_hw_dspp_init() - Initializes the DSPP hw driver object.
 * should be called once before accessing every DSPP.
 * @dev:  Corresponding device for devres management
 * @cfg:  DSPP catalog entry for which driver object is required
 * @addr: Mapped register io address of MDP
 * Return: pointer to structure or ERR_PTR
 */

/*
 * M2582: DSPP sub-pixel rendering (SPR).
 *
 * The panel is a pentile panel -- the factory panel node says so in as many
 * words (qcom,spr-pack-type = "pentile", qcom,spr-pentile-pack-type =
 * "BG-RG Type B") -- and its DDIC therefore expects the subpixels to have been
 * rearranged for it.  The factory stack does that in this block; the parameters
 * are not in any kernel image or device tree, they are handed down by userspace
 * through SDE's regDMA path, so they were read back from a running factory
 * image instead (artifacts/meizu22/android-spr-1101/spr0.txt).
 *
 * Without the block the panel is fed a plain RGB raster and full-width
 * horizontal lines come out as a zigzag.  That the raster is the cause is the
 * working hypothesis this code exists to test, not an established fact: on the
 * factory image this block is left bypassed (control word reads zero) and the
 * picture is fine, so the DDIC may also be doing the packing itself.
 *
 * The block sits at DSPP + 0x15400 (qcom,sde-dspp-spr-off) and is 0x200 bytes;
 * the three instances of the factory image hold byte-identical values.  Only
 * these registers are non-zero there, so this is the whole configuration.
 */
/*
 * qcom,sde-dspp-spr-off = <0x15400 0x14400 0x13400>: the block walks down one
 * page per instance, so DSPP1's SPR is not at DSPP0's offset (using a single
 * fixed offset silently programs the neighbour's block, or nothing at all).
 */
static u32 m2582_spr_off_id(u32 id)
{
	return 0x15400 - (id - DSPP_0) * 0x1000;
}

static u32 m2582_spr_off(const struct dpu_hw_dspp *ctx)
{
	/* DSPP ids are one based, so DSPP_0 is the 0x15400 instance */
	return m2582_spr_off_id(ctx->idx);
}

/*
 * qcom,spr-pack-type = "pentile", qcom,spr-pentile-pack-type = "BG-RG Type B"
 * on this panel; the numbers are the vendor's enum order
 * (msm_display_spr_pack_type / msm_display_spr_pentile_pack_type_mode).
 */
#define M2582_SPR_TYPE_PENTILE	1
#define M2582_SPR_PENTILE_MODE	6

/*
 * M2582: the SPR control word is a guess -- the factory builds it in
 * userspace from pack type/mode plus a dozen other fields, and the panel never
 * got a working one from us.  The debugfs knob m2582_spr writes this override
 * so a boot can enumerate candidate words instead of costing a reflash each.
 */
static u32 m2582_spr_cfg_override;
static bool m2582_spr_cfg_valid;

void m2582_spr_override_cfg(u32 cfg)
{
	m2582_spr_cfg_override = cfg;
	m2582_spr_cfg_valid = true;
}

static const struct {
	u32 off;
	u32 val;
} m2582_spr_cfg[] = {
	{ 0x054, 0x01062080 },
	{ 0x060, 0x0000000f },
	{ 0x070, 0x15555555 },
	{ 0x090, 0x00000008 },
	{ 0x098, 0x0000e401 },
	{ 0x0a0, 0x00800000 },
	{ 0x0a4, 0x01800100 },
	{ 0x0a8, 0x02800200 },
	{ 0x0ac, 0x03800300 },
	{ 0x0b0, 0x04800400 },
	{ 0x0b4, 0x05800500 },
	{ 0x0b8, 0x06800600 },
	{ 0x0bc, 0x07800700 },
	{ 0x0c0, 0x08800800 },
	{ 0x0c4, 0x09800900 },
	{ 0x0c8, 0x0a800a00 },
	{ 0x0cc, 0x0b800b00 },
	{ 0x0d0, 0x0c800c00 },
	{ 0x0d4, 0x0d800d00 },
	{ 0x0d8, 0x0e800e00 },
	{ 0x0dc, 0x0f800f00 },
	{ 0x0e4, 0x000000c0 },
	{ 0x0e8, 0x00005d7f },
	{ 0x0ec, 0x000091b3 },
	{ 0x0f0, 0x00006e4c },
	{ 0x0f4, 0x0000a280 },
};

static void dpu_setup_dspp_spr(struct dpu_hw_dspp *ctx)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(m2582_spr_cfg); i++)
		DPU_REG_WRITE(&ctx->hw, m2582_spr_off(ctx) + m2582_spr_cfg[i].off,
			      m2582_spr_cfg[i].val);

	/*
	 * The table above only holds the parameter block; the block stays
	 * bypassed until its control register says otherwise.  The factory
	 * stack writes this word from userspace (drm_msm_spr_init_cfg), so the
	 * field positions come from the driver that consumes that payload:
	 *   bit 0      enable
	 *   bits 16-19 pack pattern (the panel's pentile mode)
	 *   bits 20-22 pack type (1 = pentile)
	 * and +0x8 holds the number of mixers the block serves (see below).
	 */
	{
		/*
		 * The factory writes this word with a masked modify whose mask
		 * (~0xFDFFFFFF) preserves bit 25; that bit is the UDC target
		 * (SPR_UDC_TARGET), and since this driver programmes no UDC it
		 * stays zero, so the word can be built as a constant instead of
		 * reading the block back (reads of this page have reset this
		 * board before).
		 */
		u32 cfg = m2582_spr_cfg_valid ? m2582_spr_cfg_override :
			  (BIT(0) |
			   ((M2582_SPR_PENTILE_MODE & 0xf) << 16) |
			   ((M2582_SPR_TYPE_PENTILE & 0x7) << 20));

		/*
		 * +0x08 is the mixer count in the factory's encoding
		 * (1 -> 0, 2 -> 1, 4 -> 3); one DSPP serves one mixer here.
		 */
		DPU_REG_WRITE(&ctx->hw, m2582_spr_off(ctx) + 0x04, cfg);
		DPU_REG_WRITE(&ctx->hw, m2582_spr_off(ctx) + 0x08, 0);
	}

	/*
	 * m2582.features=sprtest: force the colour-correction block to pass only
	 * green, so the panel itself says whether the DSPP is in the mixer's
	 * output path at all.  Driving it from the CTM property did not work --
	 * a later commit put the block back into pass-through and the panel had
	 * nothing to show -- so the write happens here, at the same point in the
	 * commit as the SPR registers, where it cannot be undone by a commit
	 * that carries no colour state.
	 */

}

struct dpu_hw_dspp *dpu_hw_dspp_init(struct drm_device *dev,
				     const struct dpu_dspp_cfg *cfg,
				     void __iomem *addr)
{
	struct dpu_hw_dspp *c;

	if (!addr)
		return ERR_PTR(-EINVAL);

	c = drmm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	/*
	 * M2582: the bootloader drives this panel before the kernel does, and
	 * its SPR settings are the ones this panel was calibrated with.  Read
	 * the control word once, before anything of ours can overwrite it, so
	 * the value can be adopted instead of guessed.
	 */


	c->hw.blk_addr = addr + cfg->base;
	c->hw.log_mask = DPU_DBG_MASK_DSPP;

	/* Assign ops */
	c->idx = cfg->id;
	c->cap = cfg;
	if (c->cap->sblk->pcc.base)
		c->ops.setup_pcc = dpu_setup_dspp_pcc;
	if (c->cap->sblk->gc.base)
		c->ops.setup_gc = dpu_setup_dspp_gc;
	/* opt-in: the panel is pentile, see the comment above the table */


	return c;
}
