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

#include <linux/iopoll.h>

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
	if (rc)
		DPU_ERROR("regdma flush timed out, status %#x\n", status);

	return rc;
}
