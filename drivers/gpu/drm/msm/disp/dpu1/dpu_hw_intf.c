// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
 */

#include "dpu_hwio.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_intf.h"
#include "dpu_kms.h"
#include "dpu_trace.h"

#include <linux/iopoll.h>
#include <linux/meizu-m2582.h>

#include <drm/drm_managed.h>

#define INTF_TIMING_ENGINE_EN           0x000
#define INTF_CONFIG                     0x004
#define INTF_HSYNC_CTL                  0x008
#define INTF_VSYNC_PERIOD_F0            0x00C
#define INTF_VSYNC_PERIOD_F1            0x010
#define INTF_VSYNC_PULSE_WIDTH_F0       0x014
#define INTF_VSYNC_PULSE_WIDTH_F1       0x018
#define INTF_DISPLAY_V_START_F0         0x01C
#define INTF_DISPLAY_V_START_F1         0x020
#define INTF_DISPLAY_V_END_F0           0x024
#define INTF_DISPLAY_V_END_F1           0x028
#define INTF_ACTIVE_V_START_F0          0x02C
#define INTF_ACTIVE_V_START_F1          0x030
#define INTF_ACTIVE_V_END_F0            0x034
#define INTF_ACTIVE_V_END_F1            0x038
#define INTF_DISPLAY_HCTL               0x03C
#define INTF_ACTIVE_HCTL                0x040
#define INTF_BORDER_COLOR               0x044
#define INTF_UNDERFLOW_COLOR            0x048
#define INTF_HSYNC_SKEW                 0x04C
#define INTF_POLARITY_CTL               0x050
#define INTF_TEST_CTL                   0x054
#define INTF_TP_COLOR0                  0x058
#define INTF_TP_COLOR1                  0x05C
#define INTF_CONFIG2                    0x060
#define INTF_DISPLAY_DATA_HCTL          0x064
#define INTF_ACTIVE_DATA_HCTL           0x068

#define INTF_DSI_CMD_MODE_TRIGGER_EN    0x084
#define INTF_PANEL_FORMAT               0x090

#define INTF_FRAME_LINE_COUNT_EN        0x0A8
#define INTF_FRAME_COUNT                0x0AC
#define INTF_LINE_COUNT                 0x0B0

#define INTF_DEFLICKER_CONFIG           0x0F0
#define INTF_DEFLICKER_STRNG_COEFF      0x0F4
#define INTF_DEFLICKER_WEAK_COEFF       0x0F8

#define INTF_TPG_ENABLE                 0x100
#define INTF_TPG_MAIN_CONTROL           0x104
#define INTF_TPG_VIDEO_CONFIG           0x108
#define INTF_TPG_COMPONENT_LIMITS       0x10C
#define INTF_TPG_RECTANGLE              0x110
#define INTF_TPG_INITIAL_VALUE          0x114
#define INTF_TPG_BLK_WHITE_PATTERN_FRAMES 0x118
#define INTF_TPG_RGB_MAPPING            0x11C
#define INTF_PROG_FETCH_START           0x170
#define INTF_PROG_ROT_START             0x174

#define INTF_VSYNC_TIMESTAMP_CTRL       0x210
#define INTF_MISR_CTRL                  0x180
#define INTF_MISR_SIGNATURE             0x184

#define INTF_WD_TIMER_0_CTL		0x230
#define INTF_WD_TIMER_0_CTL2		0x234
#define INTF_WD_TIMER_0_LOAD_VALUE	0x238

#define INTF_MUX                        0x25C
#define INTF_STATUS                     0x26C
#define INTF_AVR_CONTROL                0x270
#define INTF_AVR_MODE                   0x274
#define INTF_AVR_TRIGGER                0x278
#define INTF_AVR_VTOTAL                 0x27C
#define INTF_TEAR_MDP_VSYNC_SEL         0x280
#define INTF_TEAR_TEAR_CHECK_EN         0x284
#define INTF_TEAR_SYNC_CONFIG_VSYNC     0x288
#define INTF_TEAR_SYNC_CONFIG_HEIGHT    0x28C
#define INTF_TEAR_SYNC_WRCOUNT          0x290
#define INTF_TEAR_VSYNC_INIT_VAL        0x294
#define INTF_TEAR_INT_COUNT_VAL         0x298
#define INTF_TEAR_SYNC_THRESH           0x29C
#define INTF_TEAR_START_POS             0x2A0
#define INTF_TEAR_RD_PTR_IRQ            0x2A4
#define INTF_TEAR_WR_PTR_IRQ            0x2A8
#define INTF_TEAR_OUT_LINE_COUNT        0x2AC
#define INTF_TEAR_LINE_COUNT            0x2B0
#define INTF_TEAR_AUTOREFRESH_CONFIG    0x2B4
/*
 * Tear-check registers and bits added in DPU 10.0: the tear-detect control,
 * the upper halves of the sync threshold and write count, and the
 * TE_SINGLE_UPDATE enable bit. Earlier cores do not have them.
 */
#define INTF_TEAR_TEAR_DETECT_CTRL      0x2B8
#define INTF_TEAR_SYNC_THRESH_EXT       0x2E0
#define INTF_TEAR_SYNC_WRCOUNT_EXT      0x2E4
/* TE_SINGLE_UPDATE, only meaningful on cores that carry the feature. */
#define INTF_TEAR_TE_SINGLE_UPDATE      BIT(3)
/* VSYNC_TS_SRC_EN: dp_intf timestamp source select, set from DPU 10.0 on. */
#define INTF_VSYNC_TS_SRC_EN            BIT(5)

#define INTF_CFG_ACTIVE_H_EN	BIT(29)
#define INTF_CFG_ACTIVE_V_EN	BIT(30)

#define INTF_CFG2_DATABUS_WIDEN	BIT(0)
#define INTF_CFG2_DATA_HCTL_EN	BIT(4)
#define INTF_CFG2_DCE_DATA_COMPRESS     BIT(12)
/*
 * INTF_CONFIG2 bits the vendor sets for DSI outputs of DPU 7.0+ cores:
 * BIT(24) is SDE_INTF_PERIPHERAL_FLUSH, BIT(28) is SDE_INTF_PROG_DYNREF.
 * Upstream programs neither.
 */
#define INTF_CFG2_PERIPHERAL_FLUSH      BIT(24)
#define INTF_CFG2_PROG_DYNREF           BIT(28)

/*
 * INTF_TIMING_ENGINE_ALIGN_CTRL has no upstream name.  The factory writes it,
 * together with CONFIG2 BIT(23), whenever its setup_timing_engine is called for
 * a wide-bus output; the value is 0x46 when compression is on and 0x451 when it
 * is off (msm_drm.ko 0x101bd4 branch / 0x101bec OR-of-BIT(23) / 0x101c00 value
 * selected at 0x1019f8).  Mainline never writes it, so this board keeps the
 * bootloader's 0x00000001.
 */
#define INTF_TIMING_ENGINE_ALIGN_CTRL   0x06C
#define INTF_CFG2_ALIGN_CTRL_EN         BIT(23)


static void dpu_hw_intf_setup_timing_engine(struct dpu_hw_intf *intf,
		const struct dpu_hw_intf_timing_params *p,
		const struct msm_format *fmt)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;
	u32 hsync_period, vsync_period;
	u32 display_v_start, display_v_end;
	u32 hsync_start_x, hsync_end_x;
	u32 hsync_data_start_x, hsync_data_end_x;
	u32 active_h_start, active_h_end;
	u32 active_v_start, active_v_end;
	u32 active_hctl, display_hctl, hsync_ctl;
	u32 polarity_ctl, den_polarity;
	u32 panel_format;
	u32 intf_cfg, intf_cfg2 = 0;
	u32 display_data_hctl = 0, active_data_hctl = 0;
	u32 data_width;
	bool dp_intf = false;

	/* read interface_cfg */
	intf_cfg = DPU_REG_READ(c, INTF_CONFIG);

	if (intf->cap->type == INTF_DP)
		dp_intf = true;

	hsync_period = p->hsync_pulse_width + p->h_back_porch + p->width +
	p->h_front_porch;
	vsync_period = p->vsync_pulse_width + p->v_back_porch + p->height +
	p->v_front_porch;

	display_v_start = ((p->vsync_pulse_width + p->v_back_porch) *
	hsync_period) + p->hsync_skew;
	display_v_end = ((vsync_period - p->v_front_porch) * hsync_period) +
	p->hsync_skew - 1;

	hsync_start_x = p->h_back_porch + p->hsync_pulse_width;
	hsync_end_x = hsync_period - p->h_front_porch - 1;

	if (p->width != p->xres) { /* border fill added */
		active_h_start = hsync_start_x;
		active_h_end = active_h_start + p->xres - 1;
	} else {
		active_h_start = 0;
		active_h_end = 0;
	}

	if (p->height != p->yres) { /* border fill added */
		active_v_start = display_v_start;
		active_v_end = active_v_start + (p->yres * hsync_period) - 1;
	} else {
		active_v_start = 0;
		active_v_end = 0;
	}

	if (active_h_end) {
		active_hctl = (active_h_end << 16) | active_h_start;
		intf_cfg |= INTF_CFG_ACTIVE_H_EN;
	} else {
		active_hctl = 0;
	}

	if (active_v_end)
		intf_cfg |= INTF_CFG_ACTIVE_V_EN;

	hsync_ctl = (hsync_period << 16) | p->hsync_pulse_width;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	if (p->wide_bus_en)
		intf_cfg2 |= INTF_CFG2_DATABUS_WIDEN;

	/*
	 * Must be part of intf_cfg2 before the INTF_CONFIG2 write below.
	 * Upstream gates these two on core version >= 7; the factory gates them
	 * on capability bits that its INTF_CONFIG2 = 0x110 shows clear for this
	 * panel.  BIT(24) is the peripheral-flush handshake to the DSI, BIT(28)
	 * programmable dynamic refresh -- both matter for an ADFR panel, so the
	 * difference is real.  m2582.features=nopfdyn clears them; programmed at
	 * enable time, so a runtime switch needs a real mode switch.
	 */
	if (!dp_intf && intf->mdss_ver->core_major_ver >= 7 &&
	    !m2582_feature_opt_in("nopfdyn"))
		intf_cfg2 |= INTF_CFG2_PERIPHERAL_FLUSH | INTF_CFG2_PROG_DYNREF;

	data_width = p->width;

	/*
	 * If widebus is disabled:
	 * For uncompressed stream, the data is valid for the entire active
	 * window period.
	 * For compressed stream, data is valid for a shorter time period
	 * inside the active window depending on the compression ratio.
	 *
	 * If widebus is enabled:
	 * For uncompressed stream, data is valid for only half the active
	 * window, since the data rate is doubled in this mode.
	 * For compressed stream, data validity window needs to be adjusted for
	 * compression ratio and then further halved.
	 *
	 * For the compression mode in DP case, the p->width is already
	 * adjusted in drm_mode_to_intf_timing_params().
	 */
	if (p->compression_en && !dp_intf) {
		if (p->wide_bus_en)
			data_width = DIV_ROUND_UP(p->dce_bytes_per_line, 6);
		else
			data_width = DIV_ROUND_UP(p->dce_bytes_per_line, 3);
	} else if (p->wide_bus_en && !dp_intf) {
		data_width = p->width >> 1;
	}

	/* TODO: handle DSC+DP case, we only handle DSC+DSI case so far */
	if (p->compression_en && !dp_intf &&
	    intf->mdss_ver->core_major_ver >= 7)
		intf_cfg2 |= INTF_CFG2_DCE_DATA_COMPRESS;

	hsync_data_start_x = hsync_start_x;
	hsync_data_end_x =  hsync_start_x + data_width - 1;

	display_data_hctl = (hsync_data_end_x << 16) | hsync_data_start_x;

	if (dp_intf) {
		/* DP timing adjustment */
		display_v_start += p->hsync_pulse_width + p->h_back_porch;
		display_v_end   -= p->h_front_porch;

		active_h_start = hsync_start_x;
		active_h_end = active_h_start + p->xres - 1;
		active_v_start = display_v_start;
		active_v_end = active_v_start + (p->yres * hsync_period) - 1;

		active_hctl = (active_h_end << 16) | active_h_start;
		display_hctl = active_hctl;

		intf_cfg |= INTF_CFG_ACTIVE_H_EN | INTF_CFG_ACTIVE_V_EN;
	}

	den_polarity = 0;
	polarity_ctl = (den_polarity << 2) | /*  DEN Polarity  */
		(p->vsync_polarity << 1) | /* VSYNC Polarity */
		(p->hsync_polarity << 0);  /* HSYNC Polarity */

	if (!MSM_FORMAT_IS_YUV(fmt))
		panel_format = (fmt->bpc_g_y |
				(fmt->bpc_b_cb << 2) |
				(fmt->bpc_r_cr << 4) |
				(0x21 << 8));
	else
		/* Interface treats all the pixel data in RGB888 format */
		panel_format = (BPC8 |
				(BPC8 << 2) |
				(BPC8 << 4) |
				(0x21 << 8));

	DPU_REG_WRITE(c, INTF_HSYNC_CTL, hsync_ctl);
	DPU_REG_WRITE(c, INTF_VSYNC_PERIOD_F0, vsync_period * hsync_period);
	DPU_REG_WRITE(c, INTF_VSYNC_PULSE_WIDTH_F0,
			p->vsync_pulse_width * hsync_period);
	DPU_REG_WRITE(c, INTF_DISPLAY_HCTL, display_hctl);
	DPU_REG_WRITE(c, INTF_DISPLAY_V_START_F0, display_v_start);
	DPU_REG_WRITE(c, INTF_DISPLAY_V_END_F0, display_v_end);
	DPU_REG_WRITE(c, INTF_ACTIVE_HCTL,  active_hctl);
	DPU_REG_WRITE(c, INTF_ACTIVE_V_START_F0, active_v_start);
	DPU_REG_WRITE(c, INTF_ACTIVE_V_END_F0, active_v_end);
	DPU_REG_WRITE(c, INTF_BORDER_COLOR, p->border_clr);
	DPU_REG_WRITE(c, INTF_UNDERFLOW_COLOR, p->underflow_clr);
	DPU_REG_WRITE(c, INTF_HSYNC_SKEW, p->hsync_skew);
	DPU_REG_WRITE(c, INTF_POLARITY_CTL, polarity_ctl);
	DPU_REG_WRITE(c, INTF_FRAME_LINE_COUNT_EN, 0x3);
	DPU_REG_WRITE(c, INTF_CONFIG, intf_cfg);
	DPU_REG_WRITE(c, INTF_PANEL_FORMAT, panel_format);
	if (intf->mdss_ver->core_major_ver >= 5) {
		/*
		 * DATA_HCTL_EN controls data timing which can be different from
		 * video timing. It is recommended to enable it for all cases, except
		 * if compression is enabled in 1 pixel per clock mode
		 */
		if (!(p->compression_en && !p->wide_bus_en))
			intf_cfg2 |= INTF_CFG2_DATA_HCTL_EN;

		/*
		 * m2582.features=intfalign: reproduce the factory's alignment
		 * programming for a compressed wide-bus output.  Off by default so
		 * that the two writes stay a single variable against the baseline.
		 */
		if (p->wide_bus_en && m2582_feature_opt_in("intfalign"))
			intf_cfg2 |= INTF_CFG2_ALIGN_CTRL_EN;

		DPU_REG_WRITE(c, INTF_CONFIG2, intf_cfg2);

		if (p->wide_bus_en && m2582_feature_opt_in("intfalign"))
			DPU_REG_WRITE(c, INTF_TIMING_ENGINE_ALIGN_CTRL,
				      p->compression_en ? 0x46 : 0x451);
		DPU_REG_WRITE(c, INTF_DISPLAY_DATA_HCTL, display_data_hctl);
		DPU_REG_WRITE(c, INTF_ACTIVE_DATA_HCTL, active_data_hctl);
	}
}

static void dpu_hw_intf_enable_timing_engine(
		struct dpu_hw_intf *intf,
		u8 enable)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;
	/*
	 * Note: Display interface select is handled in top block hw layer.
	 *
	 * 0x1801011 is what keeps the tear check running.  With a bare 1 --
	 * the value the vendor writes in sde_hw_intf_enable_timing_engine() --
	 * it never latches a frame, so every command-mode kickoff times out in
	 * wait_for_idle.  The disable path must stay 0.
	 */
	DPU_REG_WRITE(c, INTF_TIMING_ENGINE_EN, enable ? 0x1801011 : 0);

	/*
	 * The timestamp block has to be enabled together with the timing
	 * engine. The vendor gates this on interface capability bits that
	 * struct dpu_intf_cfg does not model, so the value they select for
	 * this interface is written directly.
	 */
	if (enable)
		DPU_REG_WRITE(c, INTF_VSYNC_TIMESTAMP_CTRL, 0x11);
}

static void dpu_hw_intf_setup_prg_fetch(
		struct dpu_hw_intf *intf,
		const struct dpu_hw_intf_prog_fetch *fetch)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;
	int fetch_enable;

	/*
	 * Fetch should always be outside the active lines. If the fetching
	 * is programmed within active region, hardware behavior is unknown.
	 */

	fetch_enable = DPU_REG_READ(c, INTF_CONFIG);
	if (fetch->enable) {
		fetch_enable |= BIT(31);
		DPU_REG_WRITE(c, INTF_PROG_FETCH_START,
				fetch->fetch_start);
	} else {
		fetch_enable &= ~BIT(31);
	}

	DPU_REG_WRITE(c, INTF_CONFIG, fetch_enable);
}

static void dpu_hw_intf_bind_pingpong_blk(
		struct dpu_hw_intf *intf,
		const enum dpu_pingpong pp)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;
	u32 mux_cfg;

	mux_cfg = DPU_REG_READ(c, INTF_MUX);
	mux_cfg &= ~0xf;

	if (pp)
		mux_cfg |= (pp - PINGPONG_0) & 0x7;
	else
		mux_cfg |= 0xf;

	DPU_REG_WRITE(c, INTF_MUX, mux_cfg);
}

static void dpu_hw_intf_get_status(
		struct dpu_hw_intf *intf,
		struct dpu_hw_intf_status *s)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;

	if (intf->mdss_ver->core_major_ver >= 5)
		s->is_en = DPU_REG_READ(c, INTF_STATUS) & BIT(0);
	else
		s->is_en = DPU_REG_READ(c, INTF_TIMING_ENGINE_EN);

	s->is_prog_fetch_en = !!(DPU_REG_READ(c, INTF_CONFIG) & BIT(31));
	if (s->is_en) {
		s->frame_count = DPU_REG_READ(c, INTF_FRAME_COUNT);
		s->line_count = DPU_REG_READ(c, INTF_LINE_COUNT);
	} else {
		s->line_count = 0;
		s->frame_count = 0;
	}
}

static u32 dpu_hw_intf_get_line_count(struct dpu_hw_intf *intf)
{
	struct dpu_hw_blk_reg_map *c;

	if (!intf)
		return 0;

	c = &intf->hw;

	return DPU_REG_READ(c, INTF_LINE_COUNT);
}

static void dpu_hw_intf_setup_misr(struct dpu_hw_intf *intf)
{
	dpu_hw_setup_misr(&intf->hw, INTF_MISR_CTRL, 0x1);
}

static int dpu_hw_intf_collect_misr(struct dpu_hw_intf *intf, u32 *misr_value)
{
	return dpu_hw_collect_misr(&intf->hw, INTF_MISR_CTRL, INTF_MISR_SIGNATURE, misr_value);
}

static int dpu_hw_intf_enable_te(struct dpu_hw_intf *intf,
		struct dpu_hw_tear_check *te)
{
	struct dpu_hw_blk_reg_map *c;
	int cfg;
	/*
	 * DPU 10.0+ interfaces carry the 32-bit tear-check registers and
	 * TE_SINGLE_UPDATE; upstream has neither flag.
	 */
	bool te_32bit;
	bool te_single_update;
	u32 val;

	if (!intf)
		return -EINVAL;

	te_32bit = intf->mdss_ver->core_major_ver >= 10;
	te_single_update = intf->mdss_ver->core_major_ver >= 10;

	c = &intf->hw;

	/*
	 * These tear-check registers are single-buffered: they only latch while
	 * VSYNC_COUNTER_EN (BIT(19)) is clear. Program the config without it
	 * first, then set it in the second write once the rest is in place.
	 */
	cfg = 0;
	if (te->hw_vsync_mode)
		cfg |= BIT(20);

	cfg |= te->vsync_count;

	DPU_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_VSYNC, cfg);
	wmb(); /* vsync counter disabled while the single-buffer regs are written */

	DPU_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_HEIGHT, te->sync_cfg_height);
	DPU_REG_WRITE(c, INTF_TEAR_VSYNC_INIT_VAL, te->vsync_init_val);
	DPU_REG_WRITE(c, INTF_TEAR_RD_PTR_IRQ, te->rd_ptr_irq);
	DPU_REG_WRITE(c, INTF_TEAR_WR_PTR_IRQ, te->wr_ptr_irq);
	DPU_REG_WRITE(c, INTF_TEAR_START_POS, te->start_pos);
	/*
	 * Selects when the incoming TE pulse is sampled. Left at its reset
	 * value the interface never latches a tear event.
	 */
	DPU_REG_WRITE(c, INTF_TEAR_TEAR_DETECT_CTRL,
			te->vsync_init_val + 0x14);
	/*
	 * Upper halves of the sync threshold and of start_pos + threshold + 1;
	 * without them only the low 16 bits of each are programmed.
	 */
	if (te_32bit)
		DPU_REG_WRITE(c, INTF_TEAR_SYNC_THRESH_EXT,
			      ((te->sync_threshold_continue & 0xffff0000) |
			       (te->sync_threshold_start >> 16)));
	DPU_REG_WRITE(c, INTF_TEAR_SYNC_THRESH,
			((te->sync_threshold_continue << 16) |
			 te->sync_threshold_start));

	cfg |= BIT(19); /* VSYNC_COUNTER_EN */
	DPU_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_VSYNC, cfg);
	wmb(); /* ensure vsync_counter_en is written */

	if (te_32bit)
		DPU_REG_WRITE(c, INTF_TEAR_SYNC_WRCOUNT_EXT,
			      ((te->start_pos + te->sync_threshold_start + 1) >> 16));
	DPU_REG_WRITE(c, INTF_TEAR_SYNC_WRCOUNT,
			(te->start_pos + te->sync_threshold_start + 1));

	/*
	 * DPU 10.0+ interfaces need TE_SINGLE_UPDATE set alongside the enable
	 * bit; upstream writes a bare 1.
	 */
	val = 1;
	if (te_single_update)
		val |= INTF_TEAR_TE_SINGLE_UPDATE;
	DPU_REG_WRITE(c, INTF_TEAR_TEAR_CHECK_EN, val);

	/*
	 * The ping-pong split block lives in the MDP top block, not in the
	 * interface, hence the raw base offset.  The factory clears
	 * PPB0/1_CONFIG and PPB0/1_CNTL from sde_hw_setup_pp_split() for a
	 * single-interface panel; that this board is single-interface cannot be
	 * established offline, and writing zero where the factory enables would
	 * mis-route the two halves, so it stays opt-in.
	 */
	if (m2582_feature_opt_in("ppbsplit")) {
		void __iomem *mdp = c->blk_addr - intf->cap->base;

		writel_relaxed(0, mdp + 0x334);	/* MDP_PPB0_CONFIG */
		writel_relaxed(0, mdp + 0x330);	/* MDP_PPB0_CNTL   */
		writel_relaxed(0, mdp + 0x33c);	/* MDP_PPB1_CONFIG */
		writel_relaxed(0, mdp + 0x338);	/* MDP_PPB1_CNTL   */
		wmb();
	}

	/*
	 * The factory also sets the timestamp source from inside its tear-check
	 * setup, i.e. on every command-mode enable (sde_hw_intf.c: it writes
	 * BIT(0) | VSYNC_TS_SRC_EN = 0x21).  Mainline only writes this register
	 * from the video timing path, so a command-mode interface keeps the
	 * bootloader's 0x30 on this board -- BIT(4) set, BIT(0) clear.
	 * "novendorts" reverts to that for A/B.
	 */
	if (!m2582_feature_opt_in("novendorts"))
		DPU_REG_WRITE(c, INTF_VSYNC_TIMESTAMP_CTRL,
			      BIT(0) | INTF_VSYNC_TS_SRC_EN);

	/*
	 * m2582.features=dsppzero: this driver never reserves a DSPP, so the
	 * whole block keeps whatever the bootloader left in it.  Clear the
	 * enable words of the colour stages that reshape pixels (IGC, PCC, GC,
	 * MEMCOLOR, SIXZONE, 3D GAMUT) on both DSPPs; none is a register this
	 * driver writes.
	 */
	if (!m2582_feature_opt_in("nodsppzero")) {
		void __iomem *mdp = c->blk_addr - intf->cap->base;
		/*
		 * Every colour block the factory DT declares inside the DSPP, at
		 * its base word (qcom,sde-dspp-* in the runtime DT).  IGC is the
		 * first stage in the chain and maps every component through a
		 * 256-entry table, so a stale IGC enable is the one way a block
		 * nobody on the host owns can still warp every pixel that
		 * reaches the DSC encoder.  None of these blocks is used by this
		 * driver, so clearing their base word cannot collide with a
		 * register we write.
		 */
		static const u32 offs[] = {
			0x800,	/* HIST/HSIC; not an op-mode word, write is inert */
			0x82c,	/* dither;     not an op-mode word, write is inert */
			0x880,	/* MEMCOLOR;   PWL table area, not an op-mode word */
			0x900,	/* SIXZONE op mode, sixzone.base + 0 */
			0xa00,	/* VLUT data window, not an op-mode word */
			0x1000,	/* GAMUT op mode, gamut.base + 0 (vendor reads it
				 * back as op_mode) */
			0x1264,	/* IGC op mode, igc.base + 0x4 -- NOT +0; the
				 * vendor's _dspp_igcv4_off() writes 0 there */
			0x1700,	/* PCC op mode, pcc.base + 0 */
			0x17c0,	/* GC op mode,  gc.base + 0 */
		};
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(offs); i++) {
			writel_relaxed(0, mdp + 0x54000 + offs[i]);	/* DSPP0 */
			writel_relaxed(0, mdp + 0x56000 + offs[i]);	/* DSPP1 */
		}
		/* make the clears land before the pipe starts fetching */
		wmb();
	}

	/*
	 * m2582.features=gcinv2: diagnostic plumbing check.  The factory's
	 * gamma template uses channel bases 0x1808/0x1a08/0x1c08 (0x200 stride,
	 * 512 entries), not the 0x04/0x0c/0x14 window dpu_hw_dspp.c writes.
	 * An inverted ramp written here must invert the picture if the block and
	 * the layout are right.  The two words in front of the first channel are
	 * the enable/swap attempt; every address stays inside the gap between
	 * DSPP0 (0x54000) and DSPP1 (0x56000).
	 */
	if (m2582_feature_opt_in("gcinv2")) {
		void __iomem *mdp = c->blk_addr - intf->cap->base;
		static const u32 dspp[2] = { 0x54000, 0x56000 };
		static const u32 chan[3] = { 0x1808, 0x1a08, 0x1c08 };
		unsigned int d, c, i;

		for (d = 0; d < ARRAY_SIZE(dspp); d++) {
			void __iomem *base = mdp + dspp[d];

			for (c = 0; c < 3; c++)
				for (i = 0; i < 512; i++)
					writel_relaxed(1023 - i * 2, base + chan[c] + i * 4);
			writel_relaxed(1, base + 0x1804);
			writel_relaxed(1, base + 0x1800);
		}
		/* LUT and enable must be visible before the next frame */
		wmb();
	}


	return 0;
}

static void dpu_hw_intf_setup_autorefresh_config(struct dpu_hw_intf *intf,
		u32 frame_count, bool enable)
{
	struct dpu_hw_blk_reg_map *c;
	u32 refresh_cfg;

	c = &intf->hw;
	refresh_cfg = DPU_REG_READ(c, INTF_TEAR_AUTOREFRESH_CONFIG);
	if (enable)
		refresh_cfg = BIT(31) | frame_count;
	else
		refresh_cfg &= ~BIT(31);

	DPU_REG_WRITE(c, INTF_TEAR_AUTOREFRESH_CONFIG, refresh_cfg);
}

/*
 * dpu_hw_intf_get_autorefresh_config - Get autorefresh config from HW
 * @intf:        DPU intf structure
 * @frame_count: Used to return the current frame count from hw
 *
 * Returns: True if autorefresh enabled, false if disabled.
 */
static bool dpu_hw_intf_get_autorefresh_config(struct dpu_hw_intf *intf,
		u32 *frame_count)
{
	u32 val = DPU_REG_READ(&intf->hw, INTF_TEAR_AUTOREFRESH_CONFIG);

	if (frame_count != NULL)
		*frame_count = val & 0xffff;
	return !!((val & BIT(31)) >> 31);
}

static int dpu_hw_intf_disable_te(struct dpu_hw_intf *intf)
{
	struct dpu_hw_blk_reg_map *c;

	if (!intf)
		return -EINVAL;

	c = &intf->hw;
	DPU_REG_WRITE(c, INTF_TEAR_TEAR_CHECK_EN, 0);
	return 0;
}

static int dpu_hw_intf_connect_external_te(struct dpu_hw_intf *intf,
		bool enable_external_te)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;
	u32 cfg;
	int orig;

	if (!intf)
		return -EINVAL;

	c = &intf->hw;
	cfg = DPU_REG_READ(c, INTF_TEAR_SYNC_CONFIG_VSYNC);
	orig = (bool)(cfg & BIT(20));
	if (enable_external_te)
		cfg |= BIT(20);
	else
		cfg &= ~BIT(20);
	DPU_REG_WRITE(c, INTF_TEAR_SYNC_CONFIG_VSYNC, cfg);
	trace_dpu_intf_connect_ext_te(intf->idx - INTF_0, cfg);

	return orig;
}

static int dpu_hw_intf_get_vsync_info(struct dpu_hw_intf *intf,
		struct dpu_hw_pp_vsync_info *info)
{
	struct dpu_hw_blk_reg_map *c = &intf->hw;
	u32 val;

	if (!intf || !info)
		return -EINVAL;

	c = &intf->hw;

	val = DPU_REG_READ(c, INTF_TEAR_VSYNC_INIT_VAL);
	info->rd_ptr_init_val = val & 0xffff;

	val = DPU_REG_READ(c, INTF_TEAR_INT_COUNT_VAL);
	info->rd_ptr_frame_count = (val & 0xffff0000) >> 16;
	info->rd_ptr_line_count = val & 0xffff;

	val = DPU_REG_READ(c, INTF_TEAR_LINE_COUNT);
	info->wr_ptr_line_count = val & 0xffff;

	val = DPU_REG_READ(c, INTF_FRAME_COUNT);
	info->intf_frame_count = val;

	return 0;
}

static void dpu_hw_intf_vsync_sel(struct dpu_hw_intf *intf,
				  struct dpu_vsync_source_cfg *cfg)
{
	struct dpu_hw_blk_reg_map *c;

	if (!intf)
		return;

	c = &intf->hw;

	DPU_REG_WRITE(c, INTF_TEAR_MDP_VSYNC_SEL, (cfg->vsync_source & 0xf));
}

static void dpu_hw_intf_vsync_sel_v8(struct dpu_hw_intf *intf,
				  struct dpu_vsync_source_cfg *cfg)
{
	struct dpu_hw_blk_reg_map *c;

	if (!intf)
		return;

	c = &intf->hw;

	if (cfg->vsync_source >= DPU_VSYNC_SOURCE_WD_TIMER_4 &&
	    cfg->vsync_source <= DPU_VSYNC_SOURCE_WD_TIMER_1) {
		pr_warn_once("DPU 8.x supports only GPIOs and timer0 as TE sources\n");
		return;
	}

	if (cfg->vsync_source == DPU_VSYNC_SOURCE_WD_TIMER_0) {
		u32 reg;

		DPU_REG_WRITE(c, INTF_WD_TIMER_0_LOAD_VALUE,
			      CALCULATE_WD_LOAD_VALUE(cfg->frame_rate));

		DPU_REG_WRITE(c, INTF_WD_TIMER_0_CTL, BIT(0)); /* clear timer */

		reg  = BIT(8);		/* enable heartbeat timer */
		reg |= BIT(0);		/* enable WD timer */
		reg |= BIT(1);		/* select default 16 clock ticks */
		DPU_REG_WRITE(c, INTF_WD_TIMER_0_CTL2, reg);

		/* make sure that timers are enabled/disabled for vsync state */
		wmb();
	}

	dpu_hw_intf_vsync_sel(intf, cfg);
}

static void dpu_hw_intf_disable_autorefresh(struct dpu_hw_intf *intf,
					    uint32_t encoder_id, u16 vdisplay)
{
	struct dpu_hw_pp_vsync_info info;
	int trial = 0;

	/* If autorefresh is already disabled, we have nothing to do */
	if (!dpu_hw_intf_get_autorefresh_config(intf, NULL))
		return;

	/*
	 * If autorefresh is enabled, disable it and make sure it is safe to
	 * proceed with current frame commit/push. Sequence followed is,
	 * 1. Disable TE
	 * 2. Disable autorefresh config
	 * 4. Poll for frame transfer ongoing to be false
	 * 5. Enable TE back
	 */

	dpu_hw_intf_connect_external_te(intf, false);
	dpu_hw_intf_setup_autorefresh_config(intf, 0, false);

	do {
		udelay(DPU_ENC_MAX_POLL_TIMEOUT_US);
		if ((trial * DPU_ENC_MAX_POLL_TIMEOUT_US)
				> (KICKOFF_TIMEOUT_MS * USEC_PER_MSEC)) {
			DPU_ERROR("enc%d intf%d disable autorefresh failed\n",
				  encoder_id, intf->idx - INTF_0);
			break;
		}

		trial++;

		dpu_hw_intf_get_vsync_info(intf, &info);
	} while (info.wr_ptr_line_count > 0 &&
		 info.wr_ptr_line_count < vdisplay);

	dpu_hw_intf_connect_external_te(intf, true);

	DPU_DEBUG("enc%d intf%d disabled autorefresh\n",
		  encoder_id, intf->idx - INTF_0);

}

static void dpu_hw_intf_program_intf_cmd_cfg(struct dpu_hw_intf *intf,
					     struct dpu_hw_intf_cmd_mode_cfg *cmd_mode_cfg)
{
	u32 intf_cfg2 = DPU_REG_READ(&intf->hw, INTF_CONFIG2);

	if (cmd_mode_cfg->data_compress)
		intf_cfg2 |= INTF_CFG2_DCE_DATA_COMPRESS;

	if (cmd_mode_cfg->wide_bus_en)
		intf_cfg2 |= INTF_CFG2_DATABUS_WIDEN;

	/*
	 * The factory RMWs BIT(12) (compressed input) and BIT(0) (wide bus) but
	 * never sets DATA_HCTL_EN (BIT(4)) nor programs the two DATA_HCTL
	 * registers.  We reset MDSS and do not call the video path that sets
	 * those bounds, so BIT(4) here could enable a window whose bounds are
	 * never written.  m2582.features=vendorintf suppresses only this
	 * addition; it cannot restore the pre-reset bootloader contents.
	 */
	if (!m2582_feature_opt_in("vendorintf"))
		intf_cfg2 |= INTF_CFG2_DATA_HCTL_EN;

	/* Isolate DATA_HCTL_EN without changing wide bus or compression. */
	if (m2582_feature_opt_in("nohctl4"))
		intf_cfg2 &= ~INTF_CFG2_DATA_HCTL_EN;

	/*
	 * m2582.features=intf110 is a diagnostic full-register value, not a
	 * factory-matched setting.  The often-cited factory readback 0x110 came
	 * from vendor offset 0x35060, which the factory DT identifies as DP0;
	 * DSI0 is at vendor offset 0x36000.  This arm also clears BIT(0) and
	 * BIT(12), so its worse picture cannot isolate DATA_HCTL_EN.
	 */
	if (m2582_feature_opt_in("intf110"))
		intf_cfg2 = 0x110;

	DPU_REG_WRITE(&intf->hw, INTF_CONFIG2, intf_cfg2);
}

/**
 * dpu_hw_intf_init() - Initializes the INTF driver for the passed
 * interface catalog entry.
 * @dev:  Corresponding device for devres management
 * @cfg:  interface catalog entry for which driver object is required
 * @addr: mapped register io address of MDP
 * @mdss_rev: dpu core's major and minor versions
 */
struct dpu_hw_intf *dpu_hw_intf_init(struct drm_device *dev,
				     const struct dpu_intf_cfg *cfg,
				     void __iomem *addr,
				     const struct dpu_mdss_version *mdss_rev)
{
	struct dpu_hw_intf *c;

	if (cfg->type == INTF_NONE) {
		DPU_DEBUG("Skip intf %d with type NONE\n", cfg->id - INTF_0);
		return NULL;
	}

	c = drmm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	c->hw.blk_addr = addr + cfg->base;
	c->hw.log_mask = DPU_DBG_MASK_INTF;

	/*
	 * Assign ops
	 */
	c->idx = cfg->id;
	c->cap = cfg;

	c->mdss_ver = mdss_rev;

	c->ops.setup_timing_gen = dpu_hw_intf_setup_timing_engine;
	c->ops.setup_prg_fetch  = dpu_hw_intf_setup_prg_fetch;
	c->ops.get_status = dpu_hw_intf_get_status;
	c->ops.enable_timing = dpu_hw_intf_enable_timing_engine;
	c->ops.get_line_count = dpu_hw_intf_get_line_count;
	c->ops.setup_misr = dpu_hw_intf_setup_misr;
	c->ops.collect_misr = dpu_hw_intf_collect_misr;

	if (mdss_rev->core_major_ver >= 5)
		c->ops.bind_pingpong_blk = dpu_hw_intf_bind_pingpong_blk;

	/* INTF TE is only for DSI interfaces */
	if (mdss_rev->core_major_ver >= 5 && cfg->type == INTF_DSI) {
		WARN_ON(!cfg->intr_tear_rd_ptr);

		c->ops.enable_tearcheck = dpu_hw_intf_enable_te;
		c->ops.disable_tearcheck = dpu_hw_intf_disable_te;
		c->ops.connect_external_te = dpu_hw_intf_connect_external_te;
		if (mdss_rev->core_major_ver >= 8)
			c->ops.vsync_sel = dpu_hw_intf_vsync_sel_v8;
		else
			c->ops.vsync_sel = dpu_hw_intf_vsync_sel;
		c->ops.disable_autorefresh = dpu_hw_intf_disable_autorefresh;
	}

	/* Technically, INTF_CONFIG2 is present for DPU 5.0+, but
	 * we can configure it for DPU 7.0+ since the wide bus and DSC flags
	 * would not be set for DPU < 7.0 anyways
	 */
	if (mdss_rev->core_major_ver >= 7)
		c->ops.program_intf_cmd_cfg = dpu_hw_intf_program_intf_cmd_cfg;

	return c;
}
