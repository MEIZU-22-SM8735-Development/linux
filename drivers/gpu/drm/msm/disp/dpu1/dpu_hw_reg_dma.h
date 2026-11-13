/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#ifndef __DPU_HW_REG_DMA_H__
#define __DPU_HW_REG_DMA_H__

#include <linux/types.h>

struct dentry;
struct dpu_kms;
struct drm_gem_object;

/*
 * The regDMA engine executes a script ("op" stream) that is handed to it as a
 * descriptor: the engine reads (address, length) from its queue register and
 * then walks the words in that buffer.  One buffer per feature is enough.
 */
struct dpu_reg_dma_buf {
	struct drm_gem_object *bo;
	void *vaddr;
	u64 iova;
	u32 size;
	u32 index;
};

struct dpu_reg_dma {
	struct dpu_reg_dma_buf script;
	struct dpu_reg_dma_buf last;
};

/* block select bits of the decode-select op */
#define DPU_REG_DMA_SEL_DSPP0		BIT(17)
#define DPU_REG_DMA_SEL_DSPP1		BIT(18)
#define DPU_REG_DMA_SEL_DSPP2		BIT(19)

#if IS_ENABLED(CONFIG_DRM_MSM_DPU)
struct dpu_reg_dma *dpu_reg_dma_init(struct dpu_kms *dpu_kms);
void dpu_reg_dma_destroy(struct dpu_kms *dpu_kms);

void dpu_reg_dma_begin(struct dpu_reg_dma *rdma, u32 decode_sel);
int dpu_reg_dma_write_reg(struct dpu_reg_dma *rdma, u32 blk_off, u32 val);
int dpu_reg_dma_write_block(struct dpu_reg_dma *rdma, u32 blk_off,
			    const u32 *data, u32 len);

int dpu_reg_dma_commit(struct dpu_kms *dpu_kms, u32 ctl_idx, u32 ctl_base,
		       u32 db_base, u32 queue, int trigger_at_mdp);
int dpu_reg_dma_kick(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl);
int dpu_reg_dma_flush(struct dpu_kms *dpu_kms, struct dpu_hw_ctl *ctl);

void dpu_reg_dma_debugfs_init(struct dpu_kms *dpu_kms, struct dentry *dir);
#else
static inline struct dpu_reg_dma *dpu_reg_dma_init(struct dpu_kms *dpu_kms)
{
	return NULL;
}

static inline void dpu_reg_dma_destroy(struct dpu_kms *dpu_kms) { }
static inline void dpu_reg_dma_debugfs_init(struct dpu_kms *kms,
					    struct dentry *dir) { }
#endif

#endif /* __DPU_HW_REG_DMA_H__ */
