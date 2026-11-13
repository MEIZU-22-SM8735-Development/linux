// SPDX-License-Identifier: GPL-2.0-only
/* Meizu M2582 VTDR6130A command-mode panel. Commands extracted from OEM DT. */
#include <linux/backlight.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/meizu-m2582.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

struct m2582_cmd {
	u8 type, delay_ms, len;
	/*
	 * Factory burst grouping: set on the last command of a burst. Every
	 * other packet of the burst is sent with
	 * MIPI_DSI_MSG_BATCH_COMMAND, so the transport accumulates the group and
	 * kicks off once on the marked packet.  The last element of a set is
	 * always treated as a burst end.
	 */
	u8 end;
	/* set on the three writes that only the factory bootloader sends */
	u8 abl_tail;
	u8 data[95];
};

/* A command and its payload, sized for the largest entry in the tables. */
#define M2582_CMD_BUF	(sizeof(((struct m2582_cmd *)0)->data))

/*
 * First-line budget of the factory's model for this panel, and it depends on
 * the advertised DSC version.
 *
 * The panel this board actually selects is the Visionox vtdr6130a node (the
 * factory bootargs name it), and that node carries
 * qcom,mdss-dsc-version = <0x12> with a 0x70 payload whose first byte is
 * 0x12 -- DSC 1.2.  2026-10-28 concluded 1.1 from a different node in the
 * dtbo (a CSOT NT37801 with the same 1200x2670 / 600x30 geometry but no
 * version property); docs/notes/display-dscver-offline-1104.md retracted
 * that.  So the two arms are:
 *
 *   1.1 (the retracted reading): flbo 12, PPS byte 0x11
 *   1.2 (what the factory node says): flbo 13, PPS byte 0x12
 *
 * 13 for a 30-row slice is mainline's own 1.2 derivation
 * (drm_dsc_populate_dsc_params) and matches the factory node's payload.
 * m2582_verify_pps() below uses this to notice a model that drifted, so it
 * has to follow the same switch dsi_host.c uses for dsc_version_minor.
 *
 * 1.2 is the default, because that is what the factory runs (encoder
 * DSC_MAIN_CONF[31:28] and the node's 0x70 payload both say 1.2).  The 1.1
 * arm is the deviation from the broken-supply era, kept behind an explicit
 * opt-out for a controlled A/B only.
 */
static u32 m2582_dsc_flbo(void)
{
	return m2582_feature_opt_in("dsc11") ? 12 : 13;
}

/*
 * Power-on sequence.  The first 102 packets are qcom,mdss-dsi-on-command of the
 * factory panel node, entry for entry; the last three before Sleep Out are the
 * bootloader's own tail (its <DSIInitSequence> packets 102-104, see
 * artifacts/meizu22/analysis/abl-dsi-init.md), which this driver re-sends
 * because prepare() runs the whole sequence from reset instead of inheriting a
 * panel the bootloader already brought up.  The factory kernel sends 104
 * packets, not 107.  It ends with Sleep Out (0x11, 121 ms) and Display On
 * (0x29).
 */
/* not const: m2582_patch_pps_version() rewrites the 0x70 payload's
 * version byte once, at the first prepare(). */
static struct m2582_cmd m2582_on_cmds[] = {
	{ .type = 0x39, .delay_ms = 21, .len = 3,
	  .data = { 0xf0, 0xaa, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x01 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xd0, 0x05 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xcf, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xb0, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xb2, 0x13 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x15 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 10, .end = 1,
	  .data = { 0xb4, 0x45, 0x21, 0x00, 0x55, 0x11, 0x00, 0x50, 0x10, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x03, 0x01 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x35, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x53, 0x20 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x5e, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x59, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x6c, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x71, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x6f, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x75, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x72, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x03, 0x01 } },
	/*
	 * The factory's own DSC-parameter snapshot for this panel, verbatim from
	 * its device-tree node: byte 1 of the payload is the DSC version and the
	 * factory packet says 0x12, which matches qcom,mdss-dsc-version = <0x12>
	 * in that node.  The rest of the payload is the placeholder model the
	 * panel tool generated (first-line budget 1, nfl 71); the real model goes
	 * out later in the 0x0a PPS.  0x11 is only put here by the dsc11 opt-out,
	 * in m2582_patch_pps_version().
	 */
	{ .type = 0x39, .delay_ms = 0, .len = 95,
	  .data = { 0x70, 0x12, 0x00, 0x00, 0xab, 0x30, 0x80, 0x0a,
		    0x6e, 0x04, 0xb0, 0x00, 0x1e, 0x02, 0x58, 0x02,
		    0x58, 0x02, 0x00, 0x01, 0x19, 0x00, 0x20, 0x05,
		    0xd0, 0x00, 0x08, 0x00, 0x01, 0x00, 0x47, 0x03,
		    0x0d, 0x18, 0x00, 0x10, 0xf0, 0x07, 0x10, 0x20,
		    0x00, 0x06, 0x0f, 0x0f, 0x33, 0x0e, 0x1c, 0x2a,
		    0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79,
		    0x7b, 0x7d, 0x7e, 0x02, 0x02, 0x22, 0x00, 0x2a,
		    0x40, 0x2a, 0xbe, 0x3a, 0xfc, 0x3a, 0xfa, 0x3a,
		    0xf8, 0x3b, 0x38, 0x3b, 0x78, 0x3b, 0xb6, 0x4b,
		    0xb6, 0x4b, 0xf4, 0x4b, 0xf4, 0x6c, 0x34, 0x84,
		    0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xff, 0x5a, 0x80 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0a } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf9, 0x9e, 0x8f } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0f } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xf9, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xff, 0x5a, 0x81 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0x65, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xf3, 0x24 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x05 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xf3, 0xa0 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xff, 0x5a, 0x81 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xf8, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x1a } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xf8, 0x0f } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0a } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xf8, 0xfb } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xff, 0x5a, 0x81 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5, .end = 1,
	  .data = { 0xfb, 0xd3, 0xd3, 0xd3, 0xd3 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xff, 0x5a, 0x82 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xf8, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd1, 0x00, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd1, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd2, 0x00, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd2, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd5, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x03, 0x03, 0x03, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd7, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x03, 0x03, 0x03, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5, .end = 1,
	  .data = { 0xd7, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd3, 0x00, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd3, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd4, 0x00, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd4, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd6, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x02, 0x02, 0x02, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd8, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x02, 0x02, 0x02, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5, .end = 1,
	  .data = { 0xd8, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x17 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xb2, 0x16 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xb3, 0x2a } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xb5, 0x0d } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .end = 1,
	  .data = { 0xb3, 0xa0 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x12 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3, .end = 1,
	  .data = { 0xd6, 0x00, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd0, 0x00, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x12 } },
	{ .type = 0x39, .delay_ms = 0, .len = 4, .end = 1,
	  .data = { 0xd1, 0x06, 0x00, 0x00 } },
	/*
	 * The three cold-wake writes.  They are NOT the "entire difference" from
	 * the factory: the factory kernel's on-command list is a re-init that runs
	 * after DisplayDxe has already brought the panel up, while this driver is
	 * the cold initialiser.  DisplayDxe's own panel file
	 * (Panel_meizu_m2582_vtdr6130a_fhd_plus_dsc_cmd.xml, 107 commands) ends
	 * with exactly these three and then sleep-out/display-on, so they belong
	 * in the cold sequence and are sent by default.
	 *
	 * f0 aa 10 selects vendor page 0xAA10, 65 04 selects index 4, cf 4b writes
	 * 0x4B there -- an undocumented register, on a page the kernel's list only
	 * touches at index 1 (cf 09). Enable only with m2582.features=abltail.
	 */
	{ .type = 0x39, .delay_ms = 0, .len = 3, .abl_tail = 1,
	  .data = { 0xf0, 0xaa, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2, .abl_tail = 1,
	  .data = { 0x65, 0x04 } },
	{ .type = 0x39, .delay_ms = 20, .len = 2, .end = 1, .abl_tail = 1,
	  .data = { 0xcf, 0x4b } },
	{ .type = 0x05, .delay_ms = 121, .len = 1, .end = 1,
	  .data = { 0x11 } },
	{ .type = 0x05, .delay_ms = 0, .len = 1, .end = 1,
	  .data = { 0x29 } },
};

/*
 * Adaptive refresh, transcribed byte for byte from the factory panel node.  It
 * exposes six mode sets, three per frame-rate family, byte-identical between
 * timing@0 and timing@1: the family is selected by one byte, 6c 02 for 120 Hz
 * and 6c 03 for 90 Hz, in the power-on, timing-switch and adfr-on sets.  Each
 * set is self-contained (120 Hz writes d1/d2/d5/d7, 90 Hz d3/d4/d6/d8); d1 00 77
 * and d3 00 59 are the family ceilings, d2/d4 00 0X the floor codes.
 */
/* timing@0 timing-switch-command: ADFR off, family 120 (6c 02), 71 00. */
static const struct m2582_cmd m2582_timing_switch_120_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 2,
	  .data = { 0xd0, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x6c, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 2,
	  .data = { 0x71, 0x00 } },
};

/* timing@1 timing-switch-command: identical to timing@0 except 6c 03. */
static const struct m2582_cmd m2582_timing_switch_90_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 2,
	  .data = { 0xd0, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x6c, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 2,
	  .data = { 0x71, 0x00 } },
};

/* timing@0 adfr-on-command: page 0x10, cf 09, family 120, 71 00, 75 00, ADFR on (d0 11). */
static const struct m2582_cmd m2582_adfr_on_120_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xcf, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x6c, 0x02 } },
	{ .type = 0x39, .delay_ms = 9, .len = 2,
	  .data = { 0x71, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x75, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 2,
	  .data = { 0xd0, 0x11 } },
};

/* timing@1 adfr-on-command: same as timing@0 except 6c 03 and 75 02. */
static const struct m2582_cmd m2582_adfr_on_90_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xcf, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x6c, 0x03 } },
	{ .type = 0x39, .delay_ms = 9, .len = 2,
	  .data = { 0x71, 0x00 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x75, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 2,
	  .data = { 0xd0, 0x11 } },
};

/* adfr-120HZ-30HZ: d1 00 77 = 119 Hz ceiling, d2 00 03 = 30 Hz floor. */
static const struct m2582_cmd m2582_mf_120_30_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd1, 0x00, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd1, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd2, 0x00, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd2, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd5, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x03, 0x03, 0x03, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd7, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x03, 0x03, 0x03, 0x03 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 5,
	  .data = { 0xd7, 0x00, 0x55, 0xaa, 0xff } },
};

/* adfr-120HZ-20HZ: same as 120HZ-30HZ with the floor code 04. */
static const struct m2582_cmd m2582_mf_120_20_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd1, 0x00, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd1, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd2, 0x00, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd2, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd5, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x04, 0x04, 0x04, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd7, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x04, 0x04, 0x04, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 5,
	  .data = { 0xd7, 0x00, 0x55, 0xaa, 0xff } },
};

/* adfr-120HZ-10HZ: same as 120HZ-30HZ with the floor code 05. */
static const struct m2582_cmd m2582_mf_120_10_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd1, 0x00, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd1, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd2, 0x00, 0x05 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd2, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd5, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x05, 0x05, 0x05, 0x05 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd5, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd7, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x08 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x77, 0x77, 0x77, 0x77 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x0c } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd7, 0x05, 0x05, 0x05, 0x05 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 5,
	  .data = { 0xd7, 0x00, 0x55, 0xaa, 0xff } },
};

/* adfr-90HZ-30HZ: d3 00 59 = 89 Hz ceiling, d4 00 02 = 30 Hz floor. */
static const struct m2582_cmd m2582_mf_90_30_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd3, 0x00, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd3, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd4, 0x00, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd4, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd6, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x02, 0x02, 0x02, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd8, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x02, 0x02, 0x02, 0x02 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 5,
	  .data = { 0xd8, 0x00, 0x55, 0xaa, 0xff } },
};

/* adfr-90HZ-18HZ: same as 90HZ-30HZ with the floor code 04. */
static const struct m2582_cmd m2582_mf_90_18_cmds[] = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd3, 0x00, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd3, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd4, 0x00, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd4, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd6, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x04, 0x04, 0x04, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd8, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x04, 0x04, 0x04, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .end = 1, .len = 5,
	  .data = { 0xd8, 0x00, 0x55, 0xaa, 0xff } },
};

/* A frame-rate family and a floor, with the set that programs both. */
struct m2582_rate {
	const char *name;
	unsigned int family;
	unsigned int min_hz;
	const struct m2582_cmd *cmds;
	size_t n_cmds;
};

/*
 * adfr-90HZ-10HZ-mode-command, transcribed from the factory's running device
 * tree: the sixth of its six mode sets.  It is byte for byte the 90/18 set
 * above (same md5), so the controller cannot ask for it as a distinct step and
 * it is deliberately not in m2582_rates[]: requesting "10 Hz" in the 90 family
 * would send 18 Hz bytes while the driver reported 10.  Kept as the
 * vendor's own table until a measurement tells the two apart.
 */
static const struct m2582_cmd m2582_mf_90_10_cmds[] __maybe_unused = {
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xf0, 0xaa, 0x1b } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd3, 0x00, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd3, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 3,
	  .data = { 0xd4, 0x00, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x09 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0xd4, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd6, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x04, 0x04, 0x04, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd6, 0x00, 0x55, 0xaa, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 9,
	  .data = { 0xd8, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff, 0x1f, 0xff } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x10 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x59, 0x59, 0x59, 0x59 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x14 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x04, 0x04, 0x04, 0x04 } },
	{ .type = 0x39, .delay_ms = 0, .len = 2,
	  .data = { 0x65, 0x18 } },
	{ .type = 0x39, .delay_ms = 0, .len = 5,
	  .data = { 0xd8, 0x00, 0x55, 0xaa, 0xff } },
};

/*
 * The vendor's six mode sets, indexed by family and floor, minus the 90/10
 * entry that is byte-identical to 90/18 (see above): five distinct requests.
 */
static const struct m2582_rate m2582_rates[] = {
	{ "120-30", 120, 30, m2582_mf_120_30_cmds, ARRAY_SIZE(m2582_mf_120_30_cmds) },
	{ "120-20", 120, 20, m2582_mf_120_20_cmds, ARRAY_SIZE(m2582_mf_120_20_cmds) },
	{ "120-10", 120, 10, m2582_mf_120_10_cmds, ARRAY_SIZE(m2582_mf_120_10_cmds) },
	{ "90-30",   90, 30, m2582_mf_90_30_cmds,  ARRAY_SIZE(m2582_mf_90_30_cmds) },
	{ "90-18",   90, 18, m2582_mf_90_18_cmds,  ARRAY_SIZE(m2582_mf_90_18_cmds) },
};

/*
 * The factory's off-command set is two packets in two groups: each of 0x28 and
 * 0x10 is released to LP-11 after it, with the 15 ms and 106 ms waits in
 * between.  Both packets therefore carry the "last of the group" flag.
 */
static const struct m2582_cmd m2582_off_cmds[] = {
	{ .type = 0x5, .delay_ms = 15, .len = 2, .end = 1,
	  .data = { 0x28, 0x00 } },
	{ .type = 0x5, .delay_ms = 106, .len = 2, .end = 1,
	  .data = { 0x10, 0x00 } },
};

struct m2582_panel {
	struct drm_panel panel;
	struct device *dev;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
	struct regulator_bulk_data *supplies;
	struct drm_dsc_config dsc;
	/* Serialises command sets between the modeset path and the backlight path. */
	struct mutex lock;
	/* Frame-rate family (120 or 90) and floor, as last programmed. */
	unsigned int family;
	unsigned int min_hz;
	/*
	 * The family the driver is being asked for.  It is what the
	 * initialisation tail programs, so a rate requested at runtime survives
	 * the next unprepare()/prepare() cycle: unprepare() puts the DDIC back
	 * to the power-on family, which is only correct while nobody has asked
	 * for another one.
	 */
	unsigned int req_hz;
	/*
	 * The connector this panel is attached to, remembered from get_modes()
	 * so prepare() can read which mode the commit selected.  The atomic
	 * state swap happens before commit_tail, so by the time prepare() runs
	 * connector->state->mode is the new mode.
	 */
	struct drm_connector *connector;
	/*
	 * Rails are up: power-on happens in prepare(), so unprepare() must not
	 * guess.  ADFR must not be reprogrammed either once the DDIC is going
	 * away, so the floor policy stops at the same moment.
	 */
	bool powered;
	/* <debugfs>/m2582-panel/dcs, raw access to the DDIC's own command set. */
	struct dentry *debugfs;

	/*
	 * Adaptive refresh (m2582.features=ltpo).  The DDIC drops from its
	 * timing family's ceiling to a floor when the image stops changing, and
	 * the host only chooses that floor: the vendor picks it from the
	 * backlight level, not from activity (its 1 Hz/AOD state is armed only
	 * on the encoder-off path, never while the screen is on).
	 *
	 * The floor table is the vendor's, from
	 * csot_vtdr6130a_adfr_update_panel_register: 120 Hz family 30 Hz below
	 * backlight 180, 20 Hz below 700, 10 Hz above; 90 Hz family 30/18/10
	 * with thresholds 350 and 870.
	 */
	bool ltpo_enabled;
	/* ADFR was last programmed on; cleared whenever it is turned off. */
	bool adfr_on;
	/* An A/B arm deliberately changed the model: skip the verify warning. */
	bool dsc_override;
	/*
	 * First-line budget to pin after the DSI host has derived the model:
	 * dsi_populate_dsc_params() re-derives it on every dsi_host_enable(),
	 * and m2582_prepare() re-applies the pin before the PPS is packed.
	 */
	unsigned int dsc_flbo_pin;
	/*
	 * Pin first_line_bpg_offset WITHOUT recomputing the rest of the model,
	 * i.e. the inconsistent model the old flbo override produced
	 * (m2582.features=flbo15 only).
	 */
	unsigned int dsc_flbo_raw;
};

static const struct regulator_bulk_data m2582_supplies[] = {
	{ .supply = "vddio" },
	{ .supply = "vci" },
	{ .supply = "vdd" },
};

/*
 * The panel decodes what the PPS sent in prepare() describes and nothing
 * re-sends that PPS later, while the encoder's copy is reprogrammed on every
 * mode set.  These are the validated defaults for this panel: DSC 1.2,
 * 10 bpc in, 8 bpp on the wire, two 600 x 30 slices of 600 bytes each,
 * first-line budget 13 (DSC 1.2; 12 is the 1.1 opt-out arm).  A mismatch does
 * not fail the modeset, which is why it has
 * to be reported: it is exactly the state that shows up as garbage on screen.
 */
static void m2582_verify_pps(struct m2582_panel *ctx)
{
	struct drm_dsc_config *dsc = &ctx->dsc;

	if (ctx->dsc_override)
		return;
	if (dsc->dsc_version_major == 1 &&
	    dsc->dsc_version_minor == (m2582_feature_opt_in("dsc11") ? 1 : 2) &&
	    dsc->bits_per_component == 10 &&
	    dsc->bits_per_pixel == (8 << 4) &&
	    dsc->line_buf_depth == 11 &&
	    dsc->slice_width == 600 && dsc->slice_height == 30 &&
	    dsc->slice_count == 2 && dsc->slice_chunk_size == 600 &&
	    dsc->first_line_bpg_offset == m2582_dsc_flbo())
		return;

	dev_warn_once(ctx->dev,
		      "DSC model differs from validated defaults: v%u.%u bpc %u bpp %u linebuf %u slice %ux%u count %u chunk %u flbo %u\n",
		      dsc->dsc_version_major, dsc->dsc_version_minor,
		      dsc->bits_per_component, dsc->bits_per_pixel >> 4,
		      dsc->line_buf_depth, dsc->slice_width, dsc->slice_height,
		      dsc->slice_count, dsc->slice_chunk_size,
		      dsc->first_line_bpg_offset);
}

/*
 * Re-apply a pinned first-line budget and let the standard derivation rebuild
 * the rest of the rate-control model from it.  Called from prepare(), i.e.
 * after dsi_timing_setup()->dsi_populate_dsc_params() has run for this modeset
 * and before any PPS is transmitted.
 */
static void m2582_dsc_apply_pin(struct m2582_panel *ctx)
{
	struct drm_dsc_config *dsc = &ctx->dsc;
	u32 bpp, ubpg;

	/*
	 * m2582.features=lbuf9: shrink the decoder's line buffer one step
	 * (11 -> 9) with everything else at the validated 10 bpc 8 bpp model.
	 * The 8 bpc arm changed both this and bpc, and the "missing sub-row at
	 * the bottom of alternate 64-column strips" disappeared there; this arm
	 * separates the two, and unlike bpc8 it should still show correct colour.
	 * The rate-control model is recomputed from the new buffer size, because
	 * the PPS carries it.
	 */
	/*
	 * m2582.features=cvt0: stop the encoder from converting RGB to YUV before
	 * DSC and tell the decoder the same.  convert_rgb is the field that says
	 * how a source pixel is split into components, which is the same layer as
	 * bpc; bpc 8 moved the per-64-column sub-row gap but also broke colour
	 * (DSC and DDIC disagreed), so this arm changes the component model
	 * instead, where both ends can stay consistent.
	 */



	/*
	 * m2582.features=dtsnap pins the whole rate-control model to the values
	 * in the vendor device tree's 0x70 PPS snapshot, which are self-consistent
	 * but differ from what the driver derives: flbo 1 / nfl 71 / dec_delay
	 * 281 / scale_inc 1488 instead of 13 / 919 / 581 / 746.  The panel is
	 * handed that snapshot by the on-command list on every prepare(), so if
	 * the DDIC latches it and ignores the PPS we send afterwards, the decoder
	 * runs on this model while the encoder runs on the derived one -- a
	 * mismatch that would corrupt exactly one slice per affected row and
	 * leave flat content untouched.
	 */

	if (ctx->dsc_flbo_raw) {
		dsc->first_line_bpg_offset = ctx->dsc_flbo_raw;
		return;
	}
	if (!ctx->dsc_flbo_pin)
		return;

	bpp = dsc->bits_per_pixel >> 4;
	ubpg = (3 * bpp + 2) * 3;
	dsc->first_line_bpg_offset = min(ctx->dsc_flbo_pin, ubpg - 3 * bpp);
	if (drm_dsc_compute_rc_parameters(dsc))
		dev_warn(ctx->dev, "pinned flbo %u left the model inconsistent\n",
			 dsc->first_line_bpg_offset);
}

/*
 * Byte 1 of the 0x70 command's payload is the DSC version the DDIC is told to
 * expect, and it has to agree with what the encoder advertises
 * (dsi_host.c:dsc_version_minor) or the panel rejects the PPS outright.
 * The factory node for this panel (vtdr6130a) carries 0x12; the 0x11 in the
 * table below came from the 2026-10-28 reading of a different dtbo node that
 * docs/notes/display-dscver-offline-1104.md retracted.  Patched once per boot
 * rather than duplicated as a second table.
 */
static void m2582_patch_pps_version(void)
{
	unsigned int i;
	u8 v = m2582_feature_opt_in("dsc11") ? 0x11 : 0x12;

	for (i = 0; i < ARRAY_SIZE(m2582_on_cmds); i++)
		if (m2582_on_cmds[i].len >= 2 && m2582_on_cmds[i].data[0] == 0x70)
			m2582_on_cmds[i].data[1] = v;
}

static void m2582_pack_pps(struct drm_dsc_picture_parameter_set *pps,
			   const struct drm_dsc_config *dsc)
{
	drm_dsc_pps_payload_pack(pps, dsc);

	/*
	 * Stock sde_dsc_create_pps_buf_cmd() retains PPS87 in PPS88 for 1.2
	 * RGB (msm_drm.ko 0x1232dc). Match its complete payload, including this
	 * reserved-bit quirk; the stock model is validated with memory DMA.
	 */
	if (dsc->dsc_version_minor == 2 && !dsc->native_422 && !dsc->native_420)
		pps->native_422_420 =
			((dsc->rc_range_params[14].range_max_qp & 3) << 6) |
			(dsc->rc_range_params[14].range_bpg_offset & 0x3f);
}

static int m2582_send(struct m2582_panel *ctx, const struct m2582_cmd *cmds, size_t count)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	unsigned int i;

	for (i = 0; i < count; i++) {
		const struct m2582_cmd *cmd = &cmds[i];

		/*
		 * These three writes belong to DisplayDxe, not the running factory
		 * kernel's 104-packet ON set. Keep them only as an explicit fallback.
		 */
		if (cmd->abl_tail && !m2582_feature_opt_in("abltail"))
			continue;
		/*
		 * One DSI transfer per command in LP mode (the factory marks every set
		 * dsi_lp_mode): the framing a cold panel is known to accept.
		 */
		struct mipi_dsi_msg msg = {
			.channel = dsi->channel,
			.type = cmd->type,
			.flags = MIPI_DSI_MSG_USE_LPM,
			.tx_buf = cmd->data,
			.tx_len = cmd->type == MIPI_DSI_DCS_SHORT_WRITE ? 1 : cmd->len,
		};

		ssize_t ret;

		/*
		 * The factory sends the power-on set as 25 bursts, marking every
		 * packet but the last of a burst MIPI_DSI_MSG_BATCH_COMMAND; the
		 * transport then accumulates the group and kicks off once.  Only
		 * enabled together with the factory command path, because the FIFO
		 * path has a 1 KB limit and grouping there is untested.
		 */
		/*
		 * MIPI_DSI_MSG_BATCH_COMMAND means "this group continues": every
		 * packet except the one that ends the burst carries it.  The last
		 * element of a set is therefore never batched, which also keeps a
		 * table with a missing .end mark from leaving a half-open burst
		 * behind (that failure is silent: the packets accumulate and the
		 * next transfer prepends them).
		 */
		if (!cmd->end && i != count - 1 &&
		    !m2582_feature_opt_in("cmdsingle"))
			/* msm's private flag (dsi_host.c: BIT(6)): the group
			 * continues, accumulate it and kick off once. */
			msg.flags |= BIT(6);

		if (!dsi->host->ops->transfer)
			return -EOPNOTSUPP;
		ret = dsi->host->ops->transfer(dsi->host, &msg);
		if (ret < 0)
			return ret;
		/*
		 * msm's dsi_cmds2buf_tx() returns the size of its DMA buffer rather
		 * than the payload length, so only a negative return is a real
		 * failure here.
		 */
		if (cmd->delay_ms)
			msleep(cmd->delay_ms);
	}
	return 0;
}

/*
 * The factory sends the DSC picture parameter set a second time, as a MIPI
 * "Picture Parameter Set" packet (data type 0x0a) whose payload is the
 * 128-byte PPS itself; dsi_display_enable() sends it right after
 * dsi_panel_enable(), i.e. after the whole power-on set.  This is the factory's
 * own enable sequence and the default; m2582.features=nopps0a drops it and
 */
static int m2582_send_pps_packet(struct m2582_panel *ctx)
{
	struct drm_dsc_picture_parameter_set pps;
	int ret;
	struct mipi_dsi_msg msg = {
		.channel = ctx->dsi->channel,
		.type = MIPI_DSI_PICTURE_PARAMETER_SET,
		.flags = 0,
		.tx_buf = &pps,
		.tx_len = sizeof(pps),
	};

	/*
	 * High-speed mode, because that is what the factory uses for this packet
	 * only: dsi_dce_prepare_pps_header() hardcodes
	 * 0x0A 0x01 0x00 0x00 <delay> 0x00 0x80 and
	 * dsi_panel_create_cmd_packets() only does `msg.flags |= data[3]`.  Byte 3
	 * is a literal 0, so MIPI_DSI_MSG_USE_LPM is not set here, while the same
	 * panel's init command sets are low power (the DT -state properties say
	 * dsi_lp_mode, and m2582_send() matches).  A DDIC fed the model only in LP
	 * keeps decoding with the 0x70 snapshot while the encoder runs the
	 * generated one -- a per-slice mismatch that starts on the first line.
	 */

	if (!ctx->dsi->host->ops->transfer)
		return -EOPNOTSUPP;


	m2582_pack_pps(&pps, &ctx->dsc);
	/* warns if the model being sent is not the factory's 600x30 8bpp one */
	m2582_verify_pps(ctx);

	ret = ctx->dsi->host->ops->transfer(ctx->dsi->host, &msg);
	return ret;
}

static int m2582_set_min_fps(struct m2582_panel *ctx, unsigned int family,
			     unsigned int min_hz)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(m2582_rates); i++) {
		if (m2582_rates[i].family == family &&
		    m2582_rates[i].min_hz == min_hz) {
			int ret = m2582_send(ctx, m2582_rates[i].cmds,
					     m2582_rates[i].n_cmds);

			return ret;
		}
	}
	return -EINVAL;
}

/*
 * ADFR off: the DDIC holds its family's ceiling instead of dropping to the
 * floor when the image stops changing.  This is the vtdr6130 adfr-off-command
 * (page 0x1b, d0 00); the power-on table ends with it, which is why a panel
 * that has just been prepared scans at its ceiling.
 */
static int m2582_adfr_off(struct m2582_panel *ctx)
{
	/*
	 * Three bytes, exactly as the verified baseline sent it.  The vendor's
	 * adfr-off cmd_set carries two (d0 00); changing a byte sequence that
	 * every prepare() of the working image used needs its own experiment,
	 * so the extra zero stays until that experiment exists.
	 */
	static const struct m2582_cmd cmds[] = {
		{ .type = 0x39, .delay_ms = 0, .len = 3,
		  .data = { 0xf0, 0xaa, 0x1b } },
		{ .type = 0x39, .delay_ms = 0, .len = 3, .end = 1,
		  .data = { 0xd0, 0x00, 0x00 } },
	};
	int ret;

	ret = m2582_send(ctx, cmds, ARRAY_SIZE(cmds));
	if (!ret)
		ctx->adfr_on = false;

	return ret;
}

/*
 * The vendor rewrites the "75 xx" packet of the adfr-on set on every send
 * (csot_vtdr6130a_create_minfps_cmd_packets insists it is the set's fifth
 * packet): 1 for the 120 Hz family and 3 for 90 in normal operation, 0/2 only
 * for the doze-time 1 Hz state, which is armed on the encoder-off path and
 * never while the screen is on.  The DT tables carry the 0/2 form, so the byte
 * is patched here instead of being transcribed.
 */
static int m2582_adfr_on(struct m2582_panel *ctx, unsigned int family)
{
	const struct m2582_cmd *cmds;
	size_t i, n;
	u8 v;
	int ret;

	if (family == 90) {
		cmds = m2582_adfr_on_90_cmds;
		n = ARRAY_SIZE(m2582_adfr_on_90_cmds);
		v = 3;
	} else {
		cmds = m2582_adfr_on_120_cmds;
		n = ARRAY_SIZE(m2582_adfr_on_120_cmds);
		v = 1;
	}

	for (i = 0; i < n; i++) {
		struct m2582_cmd cmd = cmds[i];

		/*
		 * Only adaptive refresh has a reason to choose the value: the
		 * non-adaptive path turns ADFR off right after this set, so it
		 * keeps the device tree's byte and stays bit-identical to the
		 * configuration that was verified before this switch existed.
		 */
		if (ctx->ltpo_enabled && cmd.data[0] == 0x75 && cmd.len > 1)
			cmd.data[1] = v;

		ret = m2582_send(ctx, &cmd, 1);
		if (ret) {
			ctx->adfr_on = false;
			return ret;
		}
	}

	ctx->adfr_on = true;

	return 0;
}

/*
 * Move the panel to a frame-rate family and set its floor: the target timing
 * node's timing-switch-command (ADFR off, then the new timing), its
 * adfr-on-command, then the family's mode set.
 *
 * The old tail sent d0 00 after the mode set.  That is the adfr-off command,
 * not a page exit, so it turned ADFR back off immediately and the floor never
 * took effect: the panel's own TE pin stayed at its ceiling and the DPU's
 * vblank count could not tell the difference.
 * The tail is kept only when adaptive refresh is not requested, because a
 * panel dropping to 20 Hz while the host still waits for a 120 Hz TE would add
 * latency to every frame.
 */
static int m2582_set_rate(struct m2582_panel *ctx, unsigned int family,
			  unsigned int min_hz)
{
	bool changed = family != ctx->family;
	int ret;

	if (changed) {
		/*
		 * The timing-switch set starts with d0 00, so ADFR is off from
		 * the first packet: clear the flag before the send, or a partial
		 * failure leaves it true and the next apply() skips the re-arm.
		 */
		ctx->adfr_on = false;
		if (family == 90)
			ret = m2582_send(ctx, m2582_timing_switch_90_cmds,
					 ARRAY_SIZE(m2582_timing_switch_90_cmds));
		else
			ret = m2582_send(ctx, m2582_timing_switch_120_cmds,
					 ARRAY_SIZE(m2582_timing_switch_120_cmds));
		if (ret)
			return ret;

		ctx->family = family;
	}

	/*
	 * The adfr-on command carries cf 09, the family code (6c 02/6c 03),
	 * 71 00 and the 75 byte m2582_adfr_on() patches to the runtime value.
	 * Adaptive refresh needs it even when the family did not change:
	 * prepare() runs on a DDIC that the power-on table just left with ADFR
	 * off.
	 */
	if (changed || ctx->ltpo_enabled) {
		ret = m2582_adfr_on(ctx, family);
		if (ret)
			return ret;
	}

	ret = m2582_set_min_fps(ctx, family, min_hz);
	if (ret)
		return ret;

	ctx->min_hz = min_hz;

	if (ctx->ltpo_enabled)
		return 0;

	return m2582_adfr_off(ctx);
}

/*
 * The floor the vendor's policy picks for the current backlight level.  The
 * vendor reads the panel's bl_level at every update and never uses activity
 * for this decision; its thresholds are 180/700 for the 120 Hz family and
 * 350/870 for the 90 Hz family (csot_vtdr6130a_adfr_update_panel_register).
 */
static unsigned int m2582_ltpo_floor(struct m2582_panel *ctx, unsigned int family)
{
	unsigned int bl;

	bl = backlight_get_brightness(ctx->panel.backlight);

	/*
	 * The factory tree's 90/10 command set is byte for byte the 90/18 one
	 * (md5 37f562acbfdc), so the controller cannot ask for two different
	 * steps there.  The vendor's threshold still selects cmd_set 50 above
	 * 870, but until a measurement tells them apart the lower step is
	 * reported as 18 Hz, the first label the factory gives it.
	 */
	if (family == 90)
		return bl < 350 ? 30 : 18;

	return bl < 180 ? 30 : (bl < 700 ? 20 : 10);
}

/*
 * Program the family's floor if it changed.  Caller holds ctx->lock and the
 * panel is powered.  ADFR itself is already on: m2582_set_rate() leaves it on
 * whenever adaptive refresh is enabled, and the DDIC drops to the floor by
 * itself once the image stops changing.
 */
static int m2582_ltpo_apply(struct m2582_panel *ctx)
{
	unsigned int hz = m2582_ltpo_floor(ctx, ctx->family);
	int ret;

	if (!ctx->ltpo_enabled || !ctx->powered)
		return 0;

	/* ADFR off always needs the re-arm, even when the floor is unchanged. */
	if (ctx->adfr_on && hz == ctx->min_hz)
		return 0;

	/*
	 * A failed modeset can leave ADFR off (timing-switch turns it off and
	 * the adfr-on that follows may not have gone out).  Re-arm it here,
	 * otherwise every later floor update would be a no-op and the panel
	 * would stay at 120 Hz while the driver believes adaptive refresh is on.
	 */
	if (!ctx->adfr_on) {
		ret = m2582_adfr_on(ctx, ctx->family);
		if (ret)
			return ret;
	}

	ret = m2582_set_min_fps(ctx, ctx->family, hz);
	if (!ret)
		ctx->min_hz = hz;
	else
		dev_err_ratelimited(ctx->dev, "ltpo floor %u Hz: %d\n",
				    hz, ret);

	return ret;
}

#define M2582_BL_OS_MAX		8191	/* max_os_bl */
#define M2582_BL_DBV_MIN	10	/* factory minimum_bl */

/*
 * The panel's brightness register is 13 bits (the factory's bl-max-level is
 * 0x1fff, i.e. DBV 10..8191), but this project drives nothing past DBV 4095 --
 * the highest value the factory calibration is measured for -- and never
 * touches HBM, because the panel's thermal state cannot be judged here.
 */
static const unsigned int m2582_dbv_max = 4095;

/*
 * The factory brightness curve, from the vendor calibration
 * (backlight_calib_vtdr6130_amoled_cmd_mode_dsi_visionox_panel_with_DSC.xml):
 * max_os_bl 8191 is the framework scale, minimum_bl 1200 the lowest DBV the
 * panel is calibrated for, max_bl 4095 its own maximum.  The (linearized_bl,
 * panel_bl) pairs both end at 4095, so nothing here exceeds the panel rating.
 * HBM is not implemented: the calibration says hbm_mode_supported="0".
 */
static const u16 m2582_bl_linearized[52] = {
	   0,   80,  160,  240,  321,  401,  481,  562,  642,  722,  802,  883,  963,
	1043, 1124, 1204, 1284, 1365, 1445, 1525, 1605, 1686, 1766, 1846, 1927, 2007,
	2087, 2167, 2248, 2328, 2408, 2489, 2569, 2649, 2730, 2810, 2890, 2970, 3051,
	3131, 3211, 3292, 3372, 3452, 3532, 3613, 3693, 3773, 3854, 3934, 4014, 4095,
};

static const u16 m2582_bl_panel[52] = {
	   0,  286,  425,  538,  627,  700,  768,  836,  910,  984, 1042, 1094, 1143,
	1190, 1234, 1276, 1315, 1353, 1389, 1424, 1457, 1489, 1521, 1552, 1583, 1613,
	1641, 1670, 1698, 1725, 1752, 1779, 1804, 1830, 1855, 1879, 1903, 1927, 1950,
	1973, 1995, 2026, 2065, 2200, 2445, 2700, 2951, 3170, 3405, 3633, 3863, 4095,
};

/*
 * OS level -> DBV through the factory curve.  Below the first table point the
 * panel is not calibrated to display anything, so anything short of 0 becomes
 * minimum_bl; 0 stays 0, which the factory driver does too (a zero request must
 * blank the panel, not light it at the floor).
 */
static u16 m2582_bl_to_dbv(unsigned int level)
{
	unsigned int x, i, span, num;
	u32 dbv;

	if (level == 0)
		return 0;
	if (level > M2582_BL_OS_MAX)
		level = M2582_BL_OS_MAX;

	x = DIV_ROUND_CLOSEST(level * m2582_dbv_max, M2582_BL_OS_MAX);

	if (x <= m2582_bl_linearized[0])
		dbv = m2582_bl_panel[0];
	else if (x >= m2582_bl_linearized[ARRAY_SIZE(m2582_bl_linearized) - 1])
		dbv = m2582_bl_panel[ARRAY_SIZE(m2582_bl_panel) - 1];
	else {
		for (i = 1; i < ARRAY_SIZE(m2582_bl_linearized); i++) {
			if (x <= m2582_bl_linearized[i])
				break;
		}
		span = m2582_bl_linearized[i] - m2582_bl_linearized[i - 1];
		num = x - m2582_bl_linearized[i - 1];
		dbv = m2582_bl_panel[i - 1] +
		      DIV_ROUND_CLOSEST(num * (m2582_bl_panel[i] -
					       m2582_bl_panel[i - 1]), span);
	}

	return clamp_val(dbv, M2582_BL_DBV_MIN, m2582_dbv_max);
}

static int __m2582_backlight_update(struct m2582_panel *ctx, u16 dbv)
{
	int ret;

	/*
	 * Stock Meizu vendor driver (msm_drm.ko dsi_panel_update_backlight)
	 * sends DCS 0x51 with 16-bit big-endian DBV.
	 * No manufacturer page0 writes or DCS 0x53 writes are sent.
	 */
	ret = mipi_dsi_dcs_set_display_brightness_large(ctx->dsi, dbv);
	if (ret)
		dev_err(ctx->dev, "backlight: DCS 0x51 failed: %d\n", ret);

	return ret;
}

static int m2582_backlight_update(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct m2582_panel *ctx = mipi_dsi_get_drvdata(dsi);
	unsigned int level = backlight_get_brightness(bl);
	u16 dbv = m2582_bl_to_dbv(level);
	int ret;

	mutex_lock(&ctx->lock);
	ret = __m2582_backlight_update(ctx, dbv);
	if (!ret)
		m2582_ltpo_apply(ctx);
	mutex_unlock(&ctx->lock);

	return ret;
}

/*
 * The factory node carries qcom,mdss-dsi-lp11-init and its driver implements it
 * by powering the panel on *after* the DSI host is up, so the DDIC comes out of
 * reset with the lanes already in LP-11; mainline expresses the same order with
 * panel.prepare_prev_first.  A DDIC released from reset with the link down comes
 * up unable to hear the commands that follow, so the reset pulse and the whole
 * command set stay together here, in the factory's order.
 */

/*
 * The 90 Hz member of the same panel mode.  The vertical blanking is stretched
 * by 120/90 so it is a real 90 Hz frame at the panel's line rate; that also
 * keeps it distinct from the 120 Hz mode, since DRM collapses two modes with
 * identical timings and the second one then cannot be selected.  The panel
 * scans at whichever rate its timing-switch set selected and the SoC follows
 * TE, so only the mode name matters -- prepare() reads it for the rate.
 */
static const struct drm_display_mode m2582_mode_90 = {
	/* nominal refresh, like m2582_mode below: 90 * 1244 * 3648 is the same
	 * pixel product as 120 * 1244 * 2736, so one clock value covers both. */
	.clock = 408430,
	.hdisplay = 1200,
	.hsync_start = 1220,
	.hsync_end = 1224,
	.htotal = 1244,
	.vdisplay = 2670,
	.vsync_start = 3604,
	.vsync_end = 3607,
	.vtotal = 3648,
	.width_mm = 65,
	.height_mm = 148,
};

static const struct drm_display_mode m2582_mode = {
	/*
	 * .clock is the panel's nominal refresh rate, not the DSI link clock.
	 *
	 * drm_mode_vrefresh() divides .clock by htotal*vtotal, and that is the
	 * only number userspace sees as "the refresh rate".  msm_dsi used to
	 * take the link rate from the same field, which is why both of this
	 * panel's modes reported ~184 Hz / ~138 Hz for years: the field had to
	 * carry the synthetic 627.126 MHz that produced the right DSI byte clock
	 * (137.625 MHz), and 627.126 MHz over 1244x2736 is 184.26 Hz, not 120.
	 *
	 * The factory declares the two things separately --
	 * qcom,mdss-dsi-panel-clockrate = 0x419fed40 for the PHY and
	 * qcom,mdss-dsi-panel-framerate = 120 for the panel -- so they are split
	 * here the same way: this field now carries the rate, and dsi_host.c
	 * takes the link clock from the DT property above.
	 *
	 * 120 * 1244 * 2736 == 90 * 1244 * 3648 == 408430080, so one clock value
	 * describes both modes exactly.
	 */
	.clock = 408430,
	.hdisplay = 1200,
	.hsync_start = 1220,
	.hsync_end = 1224,
	.htotal = 1244,
	.vdisplay = 2670,
	.vsync_start = 2703,
	.vsync_end = 2705,
	.vtotal = 2736,
	.width_mm = 65,
	.height_mm = 148,
};

static int m2582_prepare(struct drm_panel *panel)
{
	struct m2582_panel *ctx = container_of(panel, struct m2582_panel, panel);
	int i, ret, rate_ret, pps_ret;

	/*
	 * Prepare runs in the DRM bridge chain's pre_enable, after the DSI host's
	 * pre_enable (msm_dsi_host_power_on) brought up the PHY, PLL and link
	 * clocks with the lanes in LP-11 -- the order prepare_prev_first sets --
	 * and before the DPU timing engine/encoder runs.  That is DisplayDxe's
	 * and vendor dsi_display_prepare()'s window: rails, reset pulse, all 107
	 * commands, TE, ADFR and backlight, with no active frames.
	 */
	mutex_lock(&ctx->lock);

	for (i = 0; i < ARRAY_SIZE(m2582_supplies); i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret) {
			dev_err(ctx->dev, "supply %d failed: %d\n", i, ret);
			while (i--)
				regulator_disable(ctx->supplies[i].consumer);
			mutex_unlock(&ctx->lock);
			return ret;
		}
		msleep(i == 2 ? 20 : 2);
	}
	ctx->powered = true;


	/*
	 * The factory's reset sequence, from the panel node's
	 * qcom,mdss-dsi-reset-sequence <1 5 0 5 1 26>: 5/5/26 ms, active low.
	 *
	 * m2582.features=panelnoreset drops just the panel reset pulse.
	 * The legacy noreset also skips the MDSS reset; use panelnoreset to
	 * distinguish DDIC state inheritance from DPU state inheritance.
	 * m2582.features=panelskip drops
	 * the pulse *and* the whole power-on table + PPS below, i.e. the DDIC is
	 * left exactly as the bootloader (DisplayDxe) programmed it.  The vendor
	 * stack never resets this panel on a normal boot: Android inherits the
	 * bootloader's DDIC state through the continuous-splash handoff, while
	 * this driver starts from a hard reset and replays the table.  Anything
	 * the bootloader writes that is not part of the 107-packet table (the
	 * 0x70 PPS snapshot is part of it -- the bootloader's own XML ends with
	 * the same three packets) therefore survives on Android and is lost here.
	 */
	if (!m2582_feature_opt_in("panelskip") &&
	    !m2582_feature_opt_in("noreset") &&
	    !m2582_feature_opt_in("panelnoreset")) {
		gpiod_set_value_cansleep(ctx->reset, 0);
		msleep(5);
		gpiod_set_value_cansleep(ctx->reset, 1);
		msleep(5);
		gpiod_set_value_cansleep(ctx->reset, 0);
		msleep(26);
	}

	/*
	 * m2582.features=flbo15 keeps first_line_bpg_offset at 15 while nfl,
	 * dec_delay and scale_increment_interval stay derived from 13, i.e. an
	 * encoder whose own rate-control model is not self-consistent -- the
	 * state the old table override produced.
	 */


	/*
	 * m2582.features=flbo15full pins the same budget but re-derives the
	 * dependent fields from it (nfl 1060, dec_delay 631, scale_increment
	 * 689), so the model the encoder runs and the one the PPS describes
	 * stay equal.
	 */


	/*
	 * Block prediction off (m2582.features=nobp).  There is only one
	 * drm_dsc_config on this board -- this ctx->dsc, handed to the DSI host
	 * as dsi->dsc and from there to the DPU encoder -- so this assignment
	 * and the one dsi_populate_dsc_params() makes under the same flag touch
	 * the same field.  The encoder's registers only follow at the next
	 * commit, because dsc_config() runs only when the encoder is dirty.
	 */


	/* the DSI host has re-derived the model for this modeset by now */
	m2582_dsc_apply_pin(ctx);

	m2582_patch_pps_version();

	/* m2582.features=panelskip: keep the bootloader's DDIC programming */
	ret = m2582_feature_opt_in("panelskip") ? 0 :
	      m2582_send(ctx, m2582_on_cmds, ARRAY_SIZE(m2582_on_cmds));

	/*
	 * The generated PPS, where the factory sends it: after the whole power-on
	 * set and before anything else (dsi_display_enable() calls
	 * dsi_panel_update_pps() immediately after dsi_panel_enable() returns).
	 * A negative return is the only failure the transport can report: it
	 * returns the size of its DMA buffer rather than the payload length
	 * (dsi_cmds2buf_tx), so a "0" is no evidence of delivery either.
	 */
	pps_ret = 0;
	if (!ret) {
		pps_ret = m2582_send_pps_packet(ctx);
		if (pps_ret < 0)
			dev_err(ctx->dev, "early PPS packet failed: %d\n",
				pps_ret);
	}

	if (!ret) {
		/*
		 * Settle after the power-on set, documented with the cold-wake
		 * trio: without it the DDIC does not execute the tail 11/29 (bad
		 * boots read back 0x0c instead of 0x9c).
		 */
		msleep(300);

		/*
		 * The rate a modeset asks for is the one its mode carries: the
		 * 90 Hz mode and the 120 Hz mode share every timing except the
		 * vertical totals, so those identify it.  prepare() runs inside
		 * the modeset, i.e. after the SoC has stopped pushing frames --
		 * the window the timing switch needs.
		 */
		if (ctx->connector && ctx->connector->state &&
		    ctx->connector->state->crtc) {
			const struct drm_display_mode *sel =
				&ctx->connector->state->crtc->mode;

			if (!strcmp(sel->name, "1200x2670-90"))
				ctx->req_hz = 90;
			else
				ctx->req_hz = 120;
		}

		/*
		 * The ON set already enables TE and selects the 120 Hz family.
		 * Match the factory enable sequence; apply a timing/floor change
		 * only when the requested mode or adaptive-refresh policy needs it.
		 */
		if (!ctx->ltpo_enabled && ctx->req_hz == 120)
			goto out_unlock;


		/*
		 * Without adaptive refresh the floor stays at 30 Hz and
		 * set_rate() also turns ADFR off; with it, the floor is the one
		 * the vendor's brightness policy picks for the current level.  A
		 * failure leaves the panel usable but unprogrammed, so report it
		 * and let the next backlight update re-arm ADFR instead of
		 * failing the whole modeset.
		 */
		rate_ret = m2582_set_rate(ctx, ctx->req_hz,
					  ctx->ltpo_enabled ?
					  m2582_ltpo_floor(ctx, ctx->req_hz) : 30);
		if (rate_ret)
			dev_err(ctx->dev, "rate setup failed: %d\n",
				rate_ret);
	}

	/*
	 * m2582.features=dcsprobe: ask the DDIC a question before the SoC starts
	 * pushing frames.  Nothing else in this driver reads from the panel, so
	 * this is the window where a read cannot collide with the command
	 * stream -- the same window the factory's own reg_read health check
	 * runs in.
	 */




	/*
	 * The power-on path stops where the factory bootloader's does: no page exit,
	 * no brightness, no DCS read and no adaptive-refresh traffic after the set.
	 * Extra traffic from the manufacturer page state the DDIC is left in has
	 * been measured to leave the panel unable to put frames on screen.
	 */
out_unlock:
	mutex_unlock(&ctx->lock);

	return ret;
}

static int m2582_enable(struct drm_panel *panel)
{
	return 0;
}

static int m2582_unprepare(struct drm_panel *panel)
{
	struct m2582_panel *ctx = container_of(panel, struct m2582_panel, panel);
	int i;

	mutex_lock(&ctx->lock);
	if (ctx->powered) {
		m2582_send(ctx, m2582_off_cmds, ARRAY_SIZE(m2582_off_cmds));
		gpiod_set_value_cansleep(ctx->reset, 1);
		for (i = ARRAY_SIZE(m2582_supplies) - 1; i >= 0; i--) {
			msleep(3);
			regulator_disable(ctx->supplies[i].consumer);
		}
		ctx->powered = false;
	}
	/*
	 * Back to the power-on family, so the next prepare() sends only a mode
	 * set.  req_hz deliberately survives: it is the rate that prepare()'s
	 * tail must program, so a rate asked for at runtime is not undone by the
	 * next disable/enable cycle.
	 */
	ctx->family = 120;
	ctx->min_hz = 30;
	mutex_unlock(&ctx->lock);
	return 0;
}

static int m2582_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct m2582_panel *ctx = container_of(panel, struct m2582_panel, panel);
	struct drm_display_mode *mode;
	unsigned int n = 0;

	ctx->connector = connector;

	mode = drm_mode_duplicate(connector->dev, &m2582_mode);
	if (!mode)
		return -ENOMEM;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	strscpy(mode->name, "1200x2670-120", sizeof(mode->name));
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);
	n++;

	mode = drm_mode_duplicate(connector->dev, &m2582_mode_90);
	if (!mode)
		return n;
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER;
	strscpy(mode->name, "1200x2670-90", sizeof(mode->name));
	drm_mode_probed_add(connector, mode);
	n++;

	return n;
}

static const struct drm_panel_funcs m2582_panel_funcs = {
	.prepare = m2582_prepare,
	.enable = m2582_enable,
	.unprepare = m2582_unprepare,
	.get_modes = m2582_get_modes,
};

static const struct backlight_ops m2582_bl_ops = { .update_status = m2582_backlight_update };

/*
 * m2582: raw access to the DDIC's own command set.
 *
 * The panel has no host-visible address space; everything goes through DCS, and
 * the factory pages (0xaa10, 0xaa12, ...) are an index/data pair on top of it:
 * 0xf0 0xaa <page> selects the page, 0x65 <index> the register, and 0xcf carries
 * the byte -- the power-on table writes exactly that way.  Reading the pair back
 * is what turns "the sequence was sent" into "the DDIC holds this value", so a
 * claim about the panel's configuration can be checked instead of assumed.
 *
 *   w <b0> <b1> ...        one DCS write: 1-2 bytes short, 3+ long
 *   r <cmd> <len>          DCS read, len 1..10
 *   g <page> <idx> <len>   read len bytes at <page>/<idx>
 *   s <page> <idx> <byte>  write one byte at <page>/<idx>
 *   d <page> <idx> <count> read count consecutive indices, 16 per line
 */
#define M2582_DCS_MAX	256
/* one DCS read answers a single payload, i.e. ten bytes */
#define M2582_DCS_RD	10

static int m2582_dcs_page_get(struct m2582_panel *ctx, u8 page, u8 idx,
			      u8 *buf, size_t len)
{
	u8 sel[3] = { 0xf0, 0xaa, page };
	u8 at[2] = { 0x65, idx };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, sel, sizeof(sel));
	if (ret < 0)
		return ret;
	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, at, sizeof(at));
	if (ret < 0)
		return ret;

	return mipi_dsi_dcs_read(ctx->dsi, 0xcf, buf, len);
}

static int m2582_dcs_page_set(struct m2582_panel *ctx, u8 page, u8 idx, u8 val)
{
	u8 sel[3] = { 0xf0, 0xaa, page };
	u8 at[2] = { 0x65, idx };
	u8 wr[2] = { 0xcf, val };
	int ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, sel, sizeof(sel));
	if (ret < 0)
		return ret;
	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, at, sizeof(at));
	if (ret < 0)
		return ret;

	return mipi_dsi_dcs_write_buffer(ctx->dsi, wr, sizeof(wr));
}

static ssize_t m2582_dcs_write(struct file *f, const char __user *ubuf,
			       size_t len, loff_t *off)
{
	struct m2582_panel *ctx = file_inode(f)->i_private;
	char buf[M2582_DCS_MAX + 32];
	char *p, *cur, *tok;
	unsigned int page, idx, arg, i;
	u8 data[M2582_DCS_MAX], rx[M2582_DCS_RD];
	int ret = 0, n = 0;

	if (!ctx->powered)
		return -ENODEV;
	if (len == 0 || len >= sizeof(buf) - 1)
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	p = strim(buf);
	if (!*p)
		return -EINVAL;
	cur = p + 1;

	mutex_lock(&ctx->lock);
	switch (*p) {
	case 'w':
		while ((tok = strsep(&cur, " \t")) && n < M2582_DCS_MAX) {
			u8 byte;

			if (!*tok)
				continue;
			if (kstrtou8(tok, 16, &byte)) {
				ret = -EINVAL;
				break;
			}
			data[n++] = byte;
		}
		if (!ret && n) {
			ret = mipi_dsi_dcs_write_buffer(ctx->dsi, data, n);
			if (ret < 0)
				pr_err("m2582-panel: dcs w failed %d\n", ret);
			else
				pr_info("m2582-panel: dcs w %d -> %*ph\n", n, n, data);
		}
		break;
	case 'r':
		if (sscanf(cur, "%x %u", &page, &arg) != 2 || !arg ||
		    arg > M2582_DCS_RD) {
			ret = -EINVAL;
			break;
		}
		ret = mipi_dsi_dcs_read(ctx->dsi, page, rx, arg);
		if (ret < 0)
			pr_err("m2582-panel: dcs r %02x failed %d\n", page, ret);
		else
			pr_info("m2582-panel: dcs r %02x -> %*ph\n", page, ret, rx);
		break;
	case 'g':
		if (sscanf(cur, "%x %x %u", &page, &idx, &arg) != 3 || !arg ||
		    arg > M2582_DCS_RD) {
			ret = -EINVAL;
			break;
		}
		ret = m2582_dcs_page_get(ctx, page, idx, rx, arg);
		if (ret < 0)
			pr_err("m2582-panel: page %02x[%02x] read failed %d\n",
			       page, idx, ret);
		else
			pr_info("m2582-panel: page %02x[%02x] -> %*ph\n",
				page, idx, ret, rx);
		break;
	case 's':
		if (sscanf(cur, "%x %x %x", &page, &idx, &arg) != 3) {
			ret = -EINVAL;
			break;
		}
		ret = m2582_dcs_page_set(ctx, page, idx, arg);
		if (ret < 0)
			pr_err("m2582-panel: page %02x[%02x] write failed %d\n",
			       page, idx, ret);
		else
			pr_info("m2582-panel: page %02x[%02x] = %02x\n",
				page, idx, arg);
		break;
	case 'd':
		if (sscanf(cur, "%x %x %u", &page, &idx, &arg) != 3 || !arg ||
		    arg > M2582_DCS_MAX || idx + arg > M2582_DCS_MAX) {
			ret = -EINVAL;
			break;
		}
		for (i = 0; i < arg; i++) {
			ret = m2582_dcs_page_get(ctx, page, idx + i, &data[i], 1);
			if (ret < 0) {
				pr_err("m2582-panel: page %02x[%02x] read failed %d\n",
				       page, idx + i, ret);
				break;
			}
		}
		if (ret < 0)
			break;
		for (i = 0; i < arg; i += 16) {
			unsigned int chunk = min_t(unsigned int, 16, arg - i);

			pr_info("m2582-panel: page %02x[%02x] %*ph\n",
				page, idx + i, chunk, &data[i]);
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&ctx->lock);

	return ret < 0 ? ret : len;
}

static const struct file_operations m2582_dcs_fops = {
	.write = m2582_dcs_write,
	.llseek = noop_llseek,
};

static int m2582_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct m2582_panel *ctx;
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		/*
		 * The scale the framework uses, i.e. the calibration's max_os_bl
		 * (and the factory node's qcom,mdss-brightness-max-level).  It is
		 * NOT a DBV range: m2582_bl_to_dbv() maps it onto the panel's own
		 * 1200..4095 and nothing here may exceed that.
		 */
		.max_brightness = M2582_BL_OS_MAX,
		/* Come up at safe factory default level (320 / 0x140) to protect AMOLED. */
		.brightness = 320,
	};
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct m2582_panel, panel, &m2582_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(m2582_supplies), m2582_supplies, &ctx->supplies);
	if (ret)
		return ret;
	/*
	 * GPIOD_OUT_LOW, not GPIOD_OUT_HIGH: the DT line is GPIO_ACTIVE_LOW, so
	 * GPIOD_OUT_HIGH would drive it physically low and hold the panel in
	 * reset from probe onwards.
	 */
	ctx->req_hz = 120;
	ctx->min_hz = 30;
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "Get reset GPIO\n");
	ctx->dev = dev;
	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/*
	 * Command mode, as the factory node is: its label is
	 * qcom,mdss_dsi_meizu_amoled_m2582_vtdr6130a_dsc_cmd and its panel-type is
	 * "dsi_cmd_mode".
	 */
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
		MIPI_DSI_CLOCK_NON_CONTINUOUS;
	/*
	 * The geometry is the factory's: bit-per-component = 10,
	 * bit-per-pixel = 8, slice 600 x 30.  The 8 bpp is not a derived value --
	 * it is what the factory stack transmits, and the DSI controller
	 * registers of a running Android confirm it: DSI_CMD_MDP_STREAM0_CTRL
	 * carries a word count of 601 (600 payload bytes plus the
	 * write_memory_start command) and COMMAND_COMPRESSION_MODE_CTRL2 a
	 * stream0 slice width of 0x258, while STREAM0_TOTAL reports an H_TOTAL of
	 * 200 = 1200 bytes of compressed line over a 48-bit databus.  There is no
	 * 10 bpp anywhere in the factory register set.
	 */
	ctx->dsc.dsc_version_major = 1;
	/* DSC 1.2, like the factory: the vendor encoder's own
	 * DSC_MAIN_CONF[31:28] reads 1.2 and this panel's 0x70 payload starts
	 * with 0x12.  The PPS this driver sends must carry the same version the
	 * encoder advertises, or the two models disagree by construction.
	 * m2582.features=dsc11 is the opt-out for a controlled A/B.  Both arms
	 * assign: the struct is zeroed, so the default is otherwise DSC 1.0. */
	ctx->dsc.dsc_version_minor = m2582_feature_opt_in("dsc11") ? 1 : 2;
	/* The factory geometry, byte for byte: 1200x2670, 10 bpc in, 8 bpp on the
	 * wire, two 600x30 slices of 600 bytes each.  See m2582_verify_pps().
	 */
	ctx->dsc.slice_height = 30;
	ctx->dsc.slice_count = 2;
	ctx->dsc.slice_width = 1200 / ctx->dsc.slice_count;
	ctx->dsc.bits_per_component = 10;
	ctx->dsc.bits_per_pixel = 8 << 4;

	/*
	 * The factory panel node sets qcom,mdss-dsc-block-prediction-enable, so
	 * block prediction is on: sde_dsc_populate_dsc_config() leaves the field
	 * alone and the vendor's parser is what fills it in from the DT.
	 */
	ctx->dsc.block_pred_enable = true;
	ctx->dsc.first_line_bpg_offset = 0;
	/*
	 * No first-line pin by default: the model dsi_populate_dsc_params()
	 * derives is the factory's rule (budget 13 at a 30-row slice, and
	 * nfl/dec/scale_increment from the shared formula).  Pinning a larger
	 * budget hides an encoder/decoder mismatch at the cost of coarser
	 * quantisation on lines 2..30, so it stays an opt-in arm
	 * (m2582.features=flbo15full).
	 */
	ctx->dsc_flbo_pin = 0;
	dsi->dsc = &ctx->dsc;
	/*
	 * The power-on sequence must travel over a live DSI link, so the panel has
	 * to be prepared after the DSI host is powered on; prepare_prev_first
	 * flips the bridge walk to that order.
	 */
	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = devm_backlight_device_register(dev, dev_name(dev), dev, dsi, &m2582_bl_ops, &props);
	if (IS_ERR(ctx->panel.backlight))
		return PTR_ERR(ctx->panel.backlight);

	/*
	 * Power-on state, and the state prepare() returns to: the family it is
	 * already in, so that the first m2582_set_rate() sends only a mode set.
	 */
	mutex_init(&ctx->lock);
	ctx->family = 120;
	ctx->min_hz = 30;

	/*
	 * Adaptive refresh is opt-in (m2582.features=ltpo).  The floor comes
	 * from the backlight level, which is where the vendor takes it from;
	 * 30 Hz at the dim end keeps the panel out of the family's lowest step
	 * where the vendor also refuses to go.
	 */
	ctx->ltpo_enabled = m2582_feature_opt_in("ltpo");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		debugfs_remove_recursive(ctx->debugfs);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	return 0;
}

static void m2582_remove(struct mipi_dsi_device *dsi)
{
	struct m2582_panel *ctx = mipi_dsi_get_drvdata(dsi);

	debugfs_remove_recursive(ctx->debugfs);
	ctx->debugfs = NULL;
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id m2582_of_match[] = {
	{ .compatible = "meizu,m2582-vtdr6130a" },
	{ }
};
MODULE_DEVICE_TABLE(of, m2582_of_match);
static struct mipi_dsi_driver m2582_driver = {
	.probe = m2582_probe,
	.remove = m2582_remove,
	.driver = { .name = "panel-meizu-m2582", .of_match_table = m2582_of_match },
};
module_mipi_dsi_driver(m2582_driver);
MODULE_DESCRIPTION("Meizu M2582 VTDR6130A command-mode AMOLED panel");
MODULE_LICENSE("GPL");
