// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

/*
 * MDP regDMA ("LUTDMA") engine, version 3.0.
 *
 * On this generation the DSPP colour blocks (PCC, IGC, GC, 3D gamut and the
 * sub-pixel renderer) are not programmed by writing their registers: the
 * registers a CPU write reaches are shadows that never enter the pixel
 * datapath.  Instead a script of "ops" is placed in memory and handed to this
 * engine, which executes it between frames.
 *
 * The engine has its own register window, mapped by dpu_kms as "reg_dma".
 * Per CTL there is a queue: writing (iova, cmd) enqueues one descriptor, and
 * writing the LUTDMA trigger bit of the CTL block starts execution.
 */

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/uaccess.h>

#include "dpu_kms.h"
#include "dpu_hw_reg_dma.h"
#include "dpu_hw_ctl.h"

/* iovas handed to the engine must be 256 byte aligned */
#define REG_DMA_GUARD_BYTES	(BIT(8) - 1)

/* op encoding, from the vendor's v3 encoder */
#define REG_DMA_DECODE_SEL	0x18180114
#define REG_DMA_SINGLE_WRITE	BIT(28)
#define REG_DMA_AUTO_INC_WRITE	BIT(30)
#define REG_DMA_OFF_MASK	(BIT(21) - 1)
#define REG_DMA_MAX_DWORDS	(BIT(14) - 1)

/* registers inside the regDMA window, relative to the DB block base */
#define REG_DMA_OPMODE		0x4
#define REG_DMA_QUEUE_STRIDE	0x1000
/*
 * The hardware numbers its CTLs from one (vendor CTL_0 = 1), and CTL n owns
 * the queue window at 0x1000 * n, so mainline ctl[0] lands on 0x1000.
 */
#define REG_DMA_REG_STATUS	0x44
#define REG_DMA_REG_CLEAR	0x48
#define REG_DMA_ERR_STATUS	0x170
#define REG_DMA_ERR_CLEAR	0x1b0
#define REG_DMA_ERR_MASK	(BIT(0) | BIT(1) | BIT(2) | BIT(16) | \
				 BIT(17) | BIT(18) | BIT(19))
#define REG_DMA_DONE		BIT(3)

/* DB (immediate) queue is started by a write to the CTL block */
#define REG_DMA_CMD_WRITE	BIT(22)
#define REG_DMA_CMD_LAST	BIT(24)
#define REG_DMA_CTL_TRIGGER	0xd4
#define REG_DMA_CTL_QUEUE0	BIT(0)

static void dpu_reg_dma_buf_free(struct dpu_kms *dpu_kms,
				 struct dpu_reg_dma_buf *buf)
{
	if (buf->bo) {
		msm_gem_put_vaddr(buf->bo);
		msm_gem_unpin_iova(buf->bo, dpu_kms->base.vm);
		drm_gem_object_put(buf->bo);
	}
	buf->bo = NULL;
	buf->vaddr = NULL;
	buf->iova = 0;
	buf->index = 0;
}

static int dpu_reg_dma_buf_alloc(struct dpu_kms *dpu_kms,
				 struct dpu_reg_dma_buf *buf, u32 size)
{
	struct drm_gem_object *bo;
	void *vaddr;
	u64 iova = 0, aligned;
	u32 off;
	int rc;

	bo = msm_gem_new(dpu_kms->dev, size + REG_DMA_GUARD_BYTES, MSM_BO_WC);
	if (IS_ERR(bo))
		return PTR_ERR(bo);

	msm_gem_object_set_name(bo, "dpu-regdma");

	/*
	 * M2582: the vendor's own descriptors sit at the top of the 32 bit iova
	 * space (0xfff...), so allocate there first and only fall back to the
	 * default allocator if that range is not part of the display's vm.
	 */
	rc = msm_gem_get_and_pin_iova_range(bo, dpu_kms->base.vm, &iova,
					    0xfff00000ULL, 0xffffffffULL);
	if (rc)
		rc = msm_gem_get_and_pin_iova(bo, dpu_kms->base.vm, &iova);
	if (rc) {
		drm_gem_object_put(bo);
		return rc;
	}

	vaddr = msm_gem_get_vaddr(bo);
	if (IS_ERR(vaddr)) {
		msm_gem_unpin_iova(bo, dpu_kms->base.vm);
		drm_gem_object_put(bo);
		return PTR_ERR(vaddr);
	}

	aligned = (iova + REG_DMA_GUARD_BYTES) & ~(u64)REG_DMA_GUARD_BYTES;
	off = aligned - iova;

	buf->bo = bo;
	buf->iova = aligned;
	buf->vaddr = (u8 *)vaddr + off;
	buf->size = size;
	buf->index = 0;

	return 0;
}

struct dpu_reg_dma *dpu_reg_dma_init(struct dpu_kms *dpu_kms)
{
	struct dpu_reg_dma *rdma;
	int rc;

	rdma = kzalloc(sizeof(*rdma), GFP_KERNEL);
	if (!rdma)
		return ERR_PTR(-ENOMEM);

	/* enough for the largest script we push (SPR init is the big one) */
	rc = dpu_reg_dma_buf_alloc(dpu_kms, &rdma->script, 0x1000);
	if (rc) {
		DPU_ERROR("failed to allocate regdma script buffer: %d\n", rc);
		goto fail;
	}

	rc = dpu_reg_dma_buf_alloc(dpu_kms, &rdma->last, 0x80);
	if (rc) {
		DPU_ERROR("failed to allocate regdma end-of-script: %d\n", rc);
		dpu_reg_dma_buf_free(dpu_kms, &rdma->script);
		goto fail;
	}

	pr_info("m2582-regdma: script iova %llx, end-of-script iova %llx\n",
		rdma->script.iova, rdma->last.iova);

	return rdma;

fail:
	kfree(rdma);
	return ERR_PTR(rc);
}

void dpu_reg_dma_destroy(struct dpu_kms *dpu_kms)
{
	struct dpu_reg_dma *rdma = dpu_kms->regdma;

	if (!rdma)
		return;

	dpu_reg_dma_buf_free(dpu_kms, &rdma->script);
	dpu_reg_dma_buf_free(dpu_kms, &rdma->last);
	kfree(rdma);
	dpu_kms->regdma = NULL;
}

static void dpu_reg_dma_emit(struct dpu_reg_dma *rdma, u32 word0, u32 word1)
{
	struct dpu_reg_dma_buf *buf = &rdma->script;
	u32 *dst = (u32 *)((u8 *)buf->vaddr + buf->index);

	dst[0] = word0;
	dst[1] = word1;
	buf->index += sizeof(u32) * 2;
}

/**
 * dpu_reg_dma_begin() - start a new script for the given decoder selection
 *
 * Every script has to start with a decode-select op, which tells the engine
 * which hardware block the following offsets are relative to.
 */
void dpu_reg_dma_begin(struct dpu_reg_dma *rdma, u32 decode_sel)
{
	rdma->script.index = 0;
	dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL, decode_sel);
}

int dpu_reg_dma_write_reg(struct dpu_reg_dma *rdma, u32 blk_off, u32 val)
{
	struct dpu_reg_dma_buf *buf = &rdma->script;

	if (!buf->vaddr)
		return -EINVAL;

	if (blk_off & ~REG_DMA_OFF_MASK)
		return -EINVAL;

	if (buf->index + 2 * sizeof(u32) > buf->size)
		return -ENOSPC;

	dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | blk_off, val);

	return 0;
}

/**
 * dpu_reg_dma_write_block() - write a table of consecutive registers
 *
 * The data is appended right after the two header words and is read by the
 * engine from the same buffer.
 */
int dpu_reg_dma_write_block(struct dpu_reg_dma *rdma, u32 blk_off,
			    const u32 *data, u32 len)
{
	struct dpu_reg_dma_buf *buf = &rdma->script;
	u32 *dst;

	if (!buf->vaddr)
		return -EINVAL;

	if (blk_off & ~REG_DMA_OFF_MASK)
		return -EINVAL;

	if (!len || len > REG_DMA_MAX_DWORDS)
		return -EINVAL;

	if (buf->index + (2 + len) * sizeof(u32) > buf->size)
		return -ENOSPC;

	dpu_reg_dma_emit(rdma, REG_DMA_AUTO_INC_WRITE | blk_off, len);
	dst = (u32 *)((u8 *)buf->vaddr + buf->index);
	memcpy(dst, data, len * sizeof(u32));
	buf->index += len * sizeof(u32);

	return 0;
}

/**
 * dpu_reg_dma_commit() - hand the script to the engine and let it run
 * @db_base: base of the DB block inside the regDMA window
 * @ctl_idx: zero-based index into the catalogue's CTL array (NOT ctl->idx,
 *           which is one based); the queue window is 0x1000 * (ctl_idx + 1)
 * @trigger_at_mdp: start the queue from the MDP block instead of the CTL block
 *
 * A queue descriptor is only executed when the trigger is written after the
 * last descriptor of the batch, so the end-of-script marker goes into its own
 * descriptor exactly like the vendor driver does.
 */
int dpu_reg_dma_commit(struct dpu_kms *dpu_kms, u32 ctl_idx, u32 ctl_base,
		       u32 db_base, u32 queue, int trigger_at_mdp)
{
	struct dpu_reg_dma *rdma = dpu_kms->regdma;
	struct dpu_reg_dma_buf *script, *last;
	void __iomem *base = dpu_kms->reg_dma;
	u32 err, status;
	int rc;

	if (!queue)
		queue = REG_DMA_QUEUE_STRIDE * (ctl_idx + 1);

	if (!rdma || !base)
		return -EINVAL;

	script = &rdma->script;
	last = &rdma->last;

	if (!script->index)
		return -EINVAL;

	/* end of script */
	last->index = 0;
	{
		u32 *dst = last->vaddr;

		dst[0] = REG_DMA_DECODE_SEL;
		dst[1] = 0;
		last->index = sizeof(u32) * 2;
	}

	wmb();

	pr_info("m2582-regdma: words %08x %08x %08x %08x\n",
		((u32 *)script->vaddr)[0], ((u32 *)script->vaddr)[1],
		((u32 *)script->vaddr)[2], ((u32 *)script->vaddr)[3]);

	writel_relaxed(BIT(0), base + db_base + REG_DMA_OPMODE);

	err = readl_relaxed(base + db_base + REG_DMA_ERR_STATUS);
	if (err) {
		pr_info("m2582-regdma: engine error status %08x, clearing\n", err);
		writel_relaxed(REG_DMA_ERR_MASK, base + db_base + REG_DMA_ERR_CLEAR);
	}

	writel_relaxed(lower_32_bits(script->iova), base + queue);
	writel_relaxed(REG_DMA_CMD_WRITE | (script->index / sizeof(u32)),
		       base + queue + 4);
	writel_relaxed(lower_32_bits(last->iova), base + queue);
	writel_relaxed(REG_DMA_CMD_WRITE | REG_DMA_CMD_LAST |
		       (last->index / sizeof(u32)), base + queue + 4);
	wmb();

	pr_info("m2582-regdma: pre  addr=%08x cmd=%08x opmode=%08x\n",
		readl_relaxed(base + queue), readl_relaxed(base + queue + 4),
		readl_relaxed(base + db_base + REG_DMA_OPMODE));

	writel_relaxed(REG_DMA_DONE, base + queue + REG_DMA_REG_CLEAR);

	if (trigger_at_mdp == 1)
		writel_relaxed(REG_DMA_CTL_QUEUE0, dpu_kms->mmio + 0x19c);
	else if (trigger_at_mdp == 2)
		writel_relaxed(REG_DMA_CTL_QUEUE0, dpu_kms->mmio + 0x119c);
	else
		writel_relaxed(REG_DMA_CTL_QUEUE0,
			       dpu_kms->mmio + ctl_base + REG_DMA_CTL_TRIGGER);

	pr_info("m2582-regdma: q=%03x addr=%08x cmd=%08x iova=%llx/%llx\n",
		queue, readl_relaxed(base + queue),
		readl_relaxed(base + queue + 4), script->iova, last->iova);

	rc = readl_poll_timeout(base + queue + REG_DMA_REG_STATUS,
				status, status & REG_DMA_DONE, 10, 20000);
	pr_info("m2582-regdma: post st=%08x err=%08x\n", status,
		readl_relaxed(base + db_base + REG_DMA_ERR_STATUS));
	if (rc)
		pr_info("m2582-regdma: done bit never came, status %08x\n",
			status);

	return rc;
}

/**
 * dpu_reg_dma_kick() - enqueue the script without starting the engine
 *
 * The vendor enqueues one descriptor per feature during the commit setup and
 * only writes the trigger immediately before the CTL flush, so the engine
 * starts with the frame.
 */
int dpu_reg_dma_kick(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl)
{
	struct dpu_reg_dma *rdma = dpu_kms->regdma;
	struct dpu_reg_dma_buf *script;
	void __iomem *base = dpu_kms->reg_dma;
	/* ctl->idx is one based (CTL_0 = 1), and CTL n owns 0x1000 * n */
	u32 window = REG_DMA_QUEUE_STRIDE * ctl->idx;

	if (!rdma || !base || !rdma->script.index)
		return -EINVAL;

	script = &rdma->script;

	wmb();
	writel_relaxed(BIT(0), base + REG_DMA_OPMODE);
	writel_relaxed(lower_32_bits(script->iova), base + window);
	writel_relaxed(REG_DMA_CMD_WRITE | (script->index / sizeof(u32)),
		       base + window + 4);
	wmb();

	return 0;
}

int dpu_reg_dma_flush(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl)
{
	struct dpu_reg_dma *rdma = dpu_kms->regdma;
	struct dpu_reg_dma_buf *last;
	void __iomem *base = dpu_kms->reg_dma;
	u32 window = REG_DMA_QUEUE_STRIDE * ctl->idx;
	u32 status = 0;
	int rc;

	if (!rdma || !base)
		return -EINVAL;

	last = &rdma->last;
	{
		u32 *dst = last->vaddr;

		dst[0] = REG_DMA_DECODE_SEL;
		dst[1] = 0;
		last->index = sizeof(u32) * 2;
	}
	wmb();

	writel_relaxed(lower_32_bits(last->iova), base + window);
	writel_relaxed(REG_DMA_CMD_WRITE | REG_DMA_CMD_LAST |
		       (last->index / sizeof(u32)), base + window + 4);
	wmb();

	writel_relaxed(REG_DMA_DONE, base + window + REG_DMA_REG_CLEAR);
	writel_relaxed(REG_DMA_CTL_QUEUE0,
		       dpu_kms->mmio + ctl->caps->base + REG_DMA_CTL_TRIGGER);

	rc = readl_poll_timeout(base + window + REG_DMA_REG_STATUS, status,
				status & REG_DMA_DONE, 10, 2000);
	pr_info_once("m2582-regdma: flush window %x rc %d st %08x err %08x\n",
		     window, rc, status, readl_relaxed(base + REG_DMA_ERR_STATUS));
	if (rc)
		pr_info("m2582-regdma: flush timeout st=%08x err=%08x\n",
			status, readl_relaxed(base + REG_DMA_ERR_STATUS));

	return rc;
}

#ifdef CONFIG_DEBUG_FS
/*
 * Self test: clear the DSPP PCC registers, let the engine write them back,
 * and read them over the CPU.  If the read-back shows the values the engine
 * wrote, the engine, its clock, its memory path and its trigger all work.
 *
 *	echo "<db_base> <trigger: 0=ctl 1=mdp> <lutdma_clk: 0|1>" > m2582_regdma
 */
static ssize_t dpu_reg_dma_test_write(struct file *file,
				      const char __user *ubuf, size_t len,
				      loff_t *off)
{
	struct dpu_kms *dpu_kms = file->private_data;
	struct dpu_reg_dma *rdma = dpu_kms->regdma;
	u32 db = 0, trigger = 0, clk = 1, queue = 0, ctl_idx = 0, ctl_base;
	u32 kind = 0;
	u32 before_c, before_g, after_c, after_g;
	char buf[64];
	int n, rc;

	if (len == 0 || len >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	if (buf[0] == 'w' || buf[0] == 'r' || buf[0] == 's') {
		u32 off = 0, val = 0, i, n_words = 1;

		if (!dpu_kms->reg_dma)
			return -ENODEV;

		if (buf[0] == 'w') {
			if (sscanf(buf + 1, "%x %x", &off, &val) != 2)
				return -EINVAL;
			if (off & 3 || off > 0x6ffc)
				return -EINVAL;
			writel_relaxed(val, dpu_kms->reg_dma + off);
			readl_relaxed(dpu_kms->reg_dma + off);
			pr_info("m2582-regdma: w %04x = %08x, reads %08x\n",
				off, val, readl_relaxed(dpu_kms->reg_dma + off));
			return len;
		}

		if (buf[0] == 's') {
			u32 n = 1, i;

			if (sscanf(buf + 1, "%x %x", &off, &n) < 1)
				return -EINVAL;
			if (n > 32)
				n = 32;
			for (i = 0; i < n; i++) {
				u32 o = off + i * 4, old, back;

				if (o > 0x6ffc)
					break;
				old = readl_relaxed(dpu_kms->reg_dma + o);
				if (old == 0x1000)
					continue;
				writel_relaxed(0x1000, dpu_kms->reg_dma + o);
				back = readl_relaxed(dpu_kms->reg_dma + o);
				writel_relaxed(old, dpu_kms->reg_dma + o);
				if (back == 0x1000)
					pr_info("m2582-regdma: rw %04x (old %08x)\n",
						o, old);
			}
			return len;
		}

		if (sscanf(buf + 1, "%x %x", &off, &n_words) < 1)
			return -EINVAL;
		if (n_words > 16)
			n_words = 16;
		for (i = 0; i < n_words; i++) {
			u32 o = off + i * 4;
			pr_info("m2582-regdma: %04x = %08x\n", o,
				readl_relaxed(dpu_kms->reg_dma + o));
		}
		return len;
	}

	n = sscanf(buf, "%x %u %u %x %u %u", &db, &trigger, &clk, &queue,
		   &ctl_idx, &kind);
	if (n < 1)
		return -EINVAL;

	if (ctl_idx >= dpu_kms->catalog->ctl_count)
		return -EINVAL;
	ctl_base = dpu_kms->catalog->ctl[ctl_idx].base;
	if (n < 4)
		queue = REG_DMA_QUEUE_STRIDE * (ctl_idx + 1);

	if (!rdma || !dpu_kms->reg_dma) {
		pr_info("m2582-regdma: engine not mapped\n");
		return -ENODEV;
	}

	if (clk)
		dpu_kms->hw_mdp->ops.setup_clk_force_ctrl(dpu_kms->hw_mdp,
							  DPU_CLK_CTRL_REG_DMA,
							  true);

	before_c = readl_relaxed(dpu_kms->mmio + 0x55700);
	before_g = readl_relaxed(dpu_kms->mmio + 0x55720);

	/* make sure a successful engine write is not mistaken for our own */
	writel_relaxed(0, dpu_kms->mmio + 0x55700);

	dpu_reg_dma_begin(rdma, DPU_REG_DMA_SEL_DSPP0);
	if (kind == 0) {
		dpu_reg_dma_write_reg(rdma, 0x1700, 1);
		dpu_reg_dma_write_reg(rdma, 0x1720, 0x8000);
	} else if (kind == 2) {
		dpu_reg_dma_write_reg(rdma, 0x1000, 0x12345678);
	} else if (kind == 3) {
		/* decode select followed by a single NO-OP */
		rdma->script.index = 0;
		dpu_reg_dma_begin(rdma, DPU_REG_DMA_SEL_DSPP0);
	} else if (kind == 4) {
		/* nothing but NO-OPs: does the engine read our buffer at all? */
		dpu_reg_dma_begin(rdma, 0);
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, 0, 0);
		dpu_reg_dma_emit(rdma, 0, 0);
	} else if (kind == 5) {
		/* exactly the vendor's end-of-script payload */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL, 0);
	} else if (kind == 6) {
		/* decode select for DSPP0, no writes */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL,
				 DPU_REG_DMA_SEL_DSPP0);
	} else if (kind == 7) {
		/* single write with no decode select */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | 0x1700, 1);
	} else if (kind == 8) {
		/* decode select with bit 8 instead of bit 17 */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL, BIT(8));
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | 0x1700, 1);
	} else if (kind == 10) {
		/* write op with BIT(27) (the vendor's "absolute range" bit) */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL,
				 DPU_REG_DMA_SEL_DSPP0);
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | BIT(27) | 0x1700,
				 1);
	} else if (kind == 11) {
		/* write to a small block offset */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL,
				 DPU_REG_DMA_SEL_DSPP0);
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | 0x100, 1);
	} else if (kind == 12) {
		/* write to block offset 0 */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL,
				 DPU_REG_DMA_SEL_DSPP0);
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE, 1);
	} else if (kind == 13) {
		/* address and value words swapped */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL,
				 DPU_REG_DMA_SEL_DSPP0);
		dpu_reg_dma_emit(rdma, 1, REG_DMA_SINGLE_WRITE | 0x1700);
	} else if (kind == 14) {
		/* absolute write: vendor numbering (mdp base 0xae00000) */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, DPU_REG_DMA_SEL_DSPP0, 0);
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | BIT(27) | 0x56700,
				 1);
	} else if (kind == 15) {
		/* absolute write: mainline numbering */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, DPU_REG_DMA_SEL_DSPP0, 0);
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | BIT(27) | 0x55700,
				 1);
	} else if (kind == 9) {
		/* decode select + a single write, nothing else */
		rdma->script.index = 0;
		dpu_reg_dma_emit(rdma, REG_DMA_DECODE_SEL,
				 DPU_REG_DMA_SEL_DSPP0);
		dpu_reg_dma_emit(rdma, REG_DMA_SINGLE_WRITE | 0x1700, 1);
	}

	rc = dpu_reg_dma_commit(dpu_kms, ctl_idx, ctl_base, db, queue, trigger);

	after_c = readl_relaxed(dpu_kms->mmio + 0x55700);
	after_g = readl_relaxed(dpu_kms->mmio + 0x55720);

	pr_info("m2582-regdma: db=%03x q=%03x trig=%s clk=%u rc=%d before=%08x/%08x after=%08x/%08x\n",
		db, queue, trigger ? "mdp" : "ctl", clk, rc, before_c, before_g,
		after_c, after_g);

	return len;
}

void dpu_reg_dma_debugfs_init(struct dpu_kms *dpu_kms, struct dentry *dir)
{
}
#else
void dpu_reg_dma_debugfs_init(struct dpu_kms *dpu_kms, struct dentry *dir)
{
}
#endif
