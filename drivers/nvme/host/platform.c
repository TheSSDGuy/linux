// SPDX-License-Identifier: GPL-2.0
/*
 * NVM Express driver for controllers on the interconnect of a SoC (platform
 * devices): the NVMe registers in a memory-mapped window, one wired interrupt
 * line per vector, DMA as a bus manager. Everything NVMe defines (registers,
 * doorbells, queues, commands, controller reset) is as on PCIe; only what PCIe
 * provided (BAR, MSI-X, function reset) is replaced.
 *
 * Based on the Apple ANS NVMe driver (apple.c, The Asahi Linux Contributors),
 * itself based on the PCI NVMe driver (pci.c, Intel Corporation) and on the
 * NVMe over Fabrics RDMA host code (rdma.c, HGST, a Western Digital Company).
 *
 * Copyright (C) 2026 Fondazione Chips-IT
 */

#include <linux/async.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/io.h>
#include <linux/mempool.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/once.h>
#include <linux/platform_device.h>

#include "nvme.h"

/* I/O queue depth, capped by CAP.MQES */
#define NVME_PLAT_IO_QUEUE_DEPTH	1024
/* I/O queues: at most one per CPU and this many */
#define NVME_PLAT_MAX_IO_QUEUES		16

/*
 * Requests are limited so that the scatterlist and the PRP list pointers of a
 * command fit in one page (as in apple.c).
 */
#define NVME_MAX_KB_SZ	4096
#define NVME_MAX_SEGS	127

struct nvme_plat_dev;

struct nvme_plat_queue {
	struct nvme_plat_dev *dev;
	struct nvme_command *sqes;
	struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	u32 __iomem *sq_db;
	u32 __iomem *cq_db;
	spinlock_t sq_lock;	/* submission queue tail and doorbell */
	spinlock_t cq_lock;	/* completion queue head, phase and doorbell */
	u16 qid;
	u16 depth;
	u16 vector;		/* interrupt line (index in the interrupts property) */
	u16 sq_tail;
	u16 cq_head;
	u8 cq_phase;
	bool enabled;
};

/* Per-request data: the command and its data mapping */
struct nvme_plat_iod {
	struct nvme_request req;
	struct nvme_command cmd;
	struct nvme_plat_queue *q;
	int npages;		/* in the PRP list; 0: small pool in use */
	int nents;		/* in the scatterlist */
	dma_addr_t first_dma;
	unsigned int dma_len;	/* length of a single DMA segment mapping */
	struct scatterlist *sg;
};

struct nvme_plat_dev {
	struct device *dev;
	void __iomem *bar;
	u32 db_stride;		/* in 32-bit words */

	int nr_irqs;
	int *irqs;

	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	mempool_t *iod_mempool;

	struct nvme_ctrl ctrl;
	struct work_struct remove_work;

	/* queues[0] is the admin queue, queues[1..nr_io_queues] the I/O queues */
	struct nvme_plat_queue *queues;
	unsigned int max_io_queues;	/* queues allocated */
	unsigned int nr_io_queues;	/* queues in use */
	u16 io_depth;

	struct blk_mq_tag_set admin_tagset;
	struct blk_mq_tag_set tagset;
};

static_assert(sizeof(struct nvme_command) == 64);

static inline struct nvme_plat_dev *to_nvme_plat_dev(struct nvme_ctrl *ctrl)
{
	return container_of(ctrl, struct nvme_plat_dev, ctrl);
}

/*
 * Command submission
 */

static void nvme_plat_submit_cmd(struct nvme_plat_queue *q, struct nvme_command *cmd)
{
	unsigned long flags;

	spin_lock_irqsave(&q->sq_lock, flags);
	memcpy(&q->sqes[q->sq_tail], cmd, sizeof(*cmd));
	if (++q->sq_tail == q->depth)
		q->sq_tail = 0;
	/* the entry is in memory before the controller sees the doorbell */
	writel(q->sq_tail, q->sq_db);
	spin_unlock_irqrestore(&q->sq_lock, flags);
}

/*
 * Data mapping with PRPs (from apple.c, itself from pci.c). Will slightly
 * overestimate the number of pages needed, which only wastes a little memory
 * for the lifetime of the I/O.
 */
static inline size_t nvme_plat_iod_alloc_size(void)
{
	const unsigned int nprps = DIV_ROUND_UP(
		NVME_MAX_KB_SZ + NVME_CTRL_PAGE_SIZE, NVME_CTRL_PAGE_SIZE);
	const int npages = DIV_ROUND_UP(8 * nprps, PAGE_SIZE - 8);

	return sizeof(__le64 *) * npages +
	       sizeof(struct scatterlist) * NVME_MAX_SEGS;
}

static void **nvme_plat_iod_list(struct request *req)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);

	return (void **)(iod->sg + blk_rq_nr_phys_segments(req));
}

static void nvme_plat_free_prps(struct nvme_plat_dev *pdev, struct request *req)
{
	const int last_prp = NVME_CTRL_PAGE_SIZE / sizeof(__le64) - 1;
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	dma_addr_t dma_addr = iod->first_dma;
	int i;

	for (i = 0; i < iod->npages; i++) {
		__le64 *prp_list = nvme_plat_iod_list(req)[i];
		dma_addr_t next_dma_addr = le64_to_cpu(prp_list[last_prp]);

		dma_pool_free(pdev->prp_page_pool, prp_list, dma_addr);
		dma_addr = next_dma_addr;
	}
}

static void nvme_plat_unmap_data(struct nvme_plat_dev *pdev, struct request *req)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);

	if (iod->dma_len) {
		dma_unmap_page(pdev->dev, iod->first_dma, iod->dma_len, rq_dma_dir(req));
		return;
	}

	WARN_ON_ONCE(!iod->nents);

	dma_unmap_sg(pdev->dev, iod->sg, iod->nents, rq_dma_dir(req));
	if (iod->npages == 0)
		dma_pool_free(pdev->prp_small_pool, nvme_plat_iod_list(req)[0],
			      iod->first_dma);
	else
		nvme_plat_free_prps(pdev, req);
	mempool_free(iod->sg, pdev->iod_mempool);
}

static void nvme_plat_print_sgl(struct scatterlist *sgl, int nents)
{
	struct scatterlist *sg;
	int i;

	for_each_sg(sgl, sg, nents, i) {
		dma_addr_t phys = sg_phys(sg);

		pr_warn("sg[%d] phys_addr:%pad offset:%d length:%d dma_address:%pad dma_length:%d\n",
			i, &phys, sg->offset, sg->length, &sg_dma_address(sg),
			sg_dma_len(sg));
	}
}

static blk_status_t nvme_plat_setup_prps(struct nvme_plat_dev *pdev,
					 struct request *req,
					 struct nvme_rw_command *cmnd)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	struct dma_pool *pool;
	int length = blk_rq_payload_bytes(req);
	struct scatterlist *sg = iod->sg;
	int dma_len = sg_dma_len(sg);
	u64 dma_addr = sg_dma_address(sg);
	int offset = dma_addr & (NVME_CTRL_PAGE_SIZE - 1);
	__le64 *prp_list;
	void **list = nvme_plat_iod_list(req);
	dma_addr_t prp_dma;
	int nprps, i;

	length -= (NVME_CTRL_PAGE_SIZE - offset);
	if (length <= 0) {
		iod->first_dma = 0;
		goto done;
	}

	dma_len -= (NVME_CTRL_PAGE_SIZE - offset);
	if (dma_len) {
		dma_addr += (NVME_CTRL_PAGE_SIZE - offset);
	} else {
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}

	if (length <= NVME_CTRL_PAGE_SIZE) {
		iod->first_dma = dma_addr;
		goto done;
	}

	nprps = DIV_ROUND_UP(length, NVME_CTRL_PAGE_SIZE);
	if (nprps <= (256 / 8)) {
		pool = pdev->prp_small_pool;
		iod->npages = 0;
	} else {
		pool = pdev->prp_page_pool;
		iod->npages = 1;
	}

	prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
	if (!prp_list) {
		iod->first_dma = dma_addr;
		iod->npages = -1;
		return BLK_STS_RESOURCE;
	}
	list[0] = prp_list;
	iod->first_dma = prp_dma;
	i = 0;
	for (;;) {
		if (i == NVME_CTRL_PAGE_SIZE >> 3) {
			__le64 *old_prp_list = prp_list;

			prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
			if (!prp_list)
				goto free_prps;
			list[iod->npages++] = prp_list;
			prp_list[0] = old_prp_list[i - 1];
			old_prp_list[i - 1] = cpu_to_le64(prp_dma);
			i = 1;
		}
		prp_list[i++] = cpu_to_le64(dma_addr);
		dma_len -= NVME_CTRL_PAGE_SIZE;
		dma_addr += NVME_CTRL_PAGE_SIZE;
		length -= NVME_CTRL_PAGE_SIZE;
		if (length <= 0)
			break;
		if (dma_len > 0)
			continue;
		if (unlikely(dma_len < 0))
			goto bad_sgl;
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}
done:
	cmnd->dptr.prp1 = cpu_to_le64(sg_dma_address(iod->sg));
	cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma);
	return BLK_STS_OK;
free_prps:
	nvme_plat_free_prps(pdev, req);
	return BLK_STS_RESOURCE;
bad_sgl:
	WARN(DO_ONCE(nvme_plat_print_sgl, iod->sg, iod->nents),
	     "Invalid SGL for payload:%d nents:%d\n", blk_rq_payload_bytes(req),
	     iod->nents);
	return BLK_STS_IOERR;
}

static blk_status_t nvme_plat_setup_prp_simple(struct nvme_plat_dev *pdev,
					       struct request *req,
					       struct nvme_rw_command *cmnd,
					       struct bio_vec *bv)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	unsigned int offset = bv->bv_offset & (NVME_CTRL_PAGE_SIZE - 1);
	unsigned int first_prp_len = NVME_CTRL_PAGE_SIZE - offset;

	iod->first_dma = dma_map_bvec(pdev->dev, bv, rq_dma_dir(req), 0);
	if (dma_mapping_error(pdev->dev, iod->first_dma))
		return BLK_STS_RESOURCE;
	iod->dma_len = bv->bv_len;

	cmnd->dptr.prp1 = cpu_to_le64(iod->first_dma);
	if (bv->bv_len > first_prp_len)
		cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma + first_prp_len);
	return BLK_STS_OK;
}

static blk_status_t nvme_plat_map_data(struct nvme_plat_dev *pdev,
				       struct request *req,
				       struct nvme_command *cmnd)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret = BLK_STS_RESOURCE;
	int nr_mapped;

	if (blk_rq_nr_phys_segments(req) == 1) {
		struct bio_vec bv = req_bvec(req);

		if (bv.bv_offset + bv.bv_len <= NVME_CTRL_PAGE_SIZE * 2)
			return nvme_plat_setup_prp_simple(pdev, req, &cmnd->rw, &bv);
	}

	iod->dma_len = 0;
	iod->sg = mempool_alloc(pdev->iod_mempool, GFP_ATOMIC);
	if (!iod->sg)
		return BLK_STS_RESOURCE;
	sg_init_table(iod->sg, blk_rq_nr_phys_segments(req));
	iod->nents = blk_rq_map_sg(req, iod->sg);
	if (!iod->nents)
		goto out_free_sg;

	nr_mapped = dma_map_sg_attrs(pdev->dev, iod->sg, iod->nents,
				     rq_dma_dir(req), DMA_ATTR_NO_WARN);
	if (!nr_mapped)
		goto out_free_sg;

	ret = nvme_plat_setup_prps(pdev, req, &cmnd->rw);
	if (ret != BLK_STS_OK)
		goto out_unmap_sg;
	return BLK_STS_OK;

out_unmap_sg:
	dma_unmap_sg(pdev->dev, iod->sg, iod->nents, rq_dma_dir(req));
out_free_sg:
	mempool_free(iod->sg, pdev->iod_mempool);
	return ret;
}

static __always_inline void nvme_plat_unmap_rq(struct request *req)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);

	if (blk_rq_nr_phys_segments(req))
		nvme_plat_unmap_data(iod->q->dev, req);
}

static void nvme_plat_complete_rq(struct request *req)
{
	nvme_plat_unmap_rq(req);
	nvme_complete_rq(req);
}

static void nvme_plat_complete_batch(struct io_comp_batch *iob)
{
	nvme_complete_batch(iob, nvme_plat_unmap_rq);
}

/*
 * Completions
 */

static inline bool nvme_plat_cqe_pending(struct nvme_plat_queue *q)
{
	struct nvme_completion *hcqe = &q->cqes[q->cq_head];

	return (le16_to_cpu(READ_ONCE(hcqe->status)) & 1) == q->cq_phase;
}

static inline struct blk_mq_tags *nvme_plat_queue_tags(struct nvme_plat_queue *q)
{
	struct nvme_plat_dev *pdev = q->dev;

	if (!q->qid)
		return pdev->admin_tagset.tags[0];
	return pdev->tagset.tags[q->qid - 1];
}

static inline void nvme_plat_handle_cqe(struct nvme_plat_queue *q,
					struct io_comp_batch *iob, u16 idx)
{
	struct nvme_completion *cqe = &q->cqes[idx];
	__u16 command_id = READ_ONCE(cqe->command_id);
	struct request *req;

	/* The asynchronous event request has no request of its own */
	if (unlikely(nvme_is_aen_req(q->qid, command_id))) {
		nvme_complete_async_event(&q->dev->ctrl, cqe->status, &cqe->result);
		return;
	}

	req = nvme_find_rq(nvme_plat_queue_tags(q), command_id);
	if (unlikely(!req)) {
		dev_warn(q->dev->dev, "invalid id %d completed on queue %d\n",
			 command_id, q->qid);
		return;
	}

	if (!nvme_try_complete_req(req, cqe->status, cqe->result) &&
	    !blk_mq_add_to_batch(req, iob,
				 nvme_req(req)->status != NVME_SC_SUCCESS,
				 nvme_plat_complete_batch))
		nvme_plat_complete_rq(req);
}

static inline void nvme_plat_update_cq_head(struct nvme_plat_queue *q)
{
	u32 tmp = q->cq_head + 1;

	if (tmp == q->depth) {
		q->cq_head = 0;
		q->cq_phase ^= 1;
	} else {
		q->cq_head = tmp;
	}
}

/* Called with cq_lock held */
static bool nvme_plat_poll_cq(struct nvme_plat_queue *q, struct io_comp_batch *iob)
{
	bool found = false;

	while (nvme_plat_cqe_pending(q)) {
		found = true;
		/*
		 * load-load control dependency between phase and the rest of
		 * the cqe requires a full read memory barrier
		 */
		dma_rmb();
		nvme_plat_handle_cqe(q, iob, q->cq_head);
		nvme_plat_update_cq_head(q);
	}

	/*
	 * The head doorbell also drops the interrupt line of the queue's vector:
	 * the line stays up while the queue holds entries not consumed
	 */
	if (found)
		writel(q->cq_head, q->cq_db);

	return found;
}

static bool nvme_plat_handle_cq(struct nvme_plat_queue *q, bool force)
{
	DEFINE_IO_COMP_BATCH(iob);
	unsigned long flags;
	bool found;

	if (!READ_ONCE(q->enabled) && !force)
		return false;

	spin_lock_irqsave(&q->cq_lock, flags);
	found = nvme_plat_poll_cq(q, &iob);
	spin_unlock_irqrestore(&q->cq_lock, flags);

	if (!rq_list_empty(&iob.req_list))
		nvme_plat_complete_batch(&iob);

	return found;
}

/* One handler per interrupt line: it serves every queue of its vector */
static irqreturn_t nvme_plat_irq(int irq, void *data)
{
	struct nvme_plat_dev *pdev = data;
	bool handled = false;
	unsigned int i;
	int vector;

	for (vector = 0; vector < pdev->nr_irqs; vector++)
		if (pdev->irqs[vector] == irq)
			break;

	for (i = 0; i <= pdev->nr_io_queues; i++) {
		struct nvme_plat_queue *q = &pdev->queues[i];

		if (q->vector == vector && nvme_plat_handle_cq(q, false))
			handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * blk-mq
 */

static blk_status_t nvme_plat_queue_rq(struct blk_mq_hw_ctx *hctx,
				       const struct blk_mq_queue_data *bd)
{
	struct nvme_ns *ns = hctx->queue->queuedata;
	struct nvme_plat_queue *q = hctx->driver_data;
	struct nvme_plat_dev *pdev = q->dev;
	struct request *req = bd->rq;
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_command *cmnd = &iod->cmd;
	blk_status_t ret;

	iod->q = q;
	iod->npages = -1;
	iod->nents = 0;
	iod->dma_len = 0;

	/*
	 * We should not need to do this, but we're still using this to
	 * ensure we can drain requests on a dying queue.
	 */
	if (unlikely(!READ_ONCE(q->enabled)))
		return BLK_STS_IOERR;

	if (!nvme_check_ready(&pdev->ctrl, req, true))
		return nvme_fail_nonready_command(&pdev->ctrl, req);

	ret = nvme_setup_cmd(ns, req);
	if (ret)
		return ret;

	if (blk_rq_nr_phys_segments(req)) {
		ret = nvme_plat_map_data(pdev, req, cmnd);
		if (ret)
			goto out_free_cmd;
	}

	nvme_start_request(req);
	nvme_plat_submit_cmd(q, cmnd);
	return BLK_STS_OK;

out_free_cmd:
	nvme_cleanup_cmd(req);
	return ret;
}

static int nvme_plat_admin_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				     unsigned int hctx_idx)
{
	struct nvme_plat_dev *pdev = to_nvme_plat_dev(data);

	WARN_ON(hctx_idx != 0);
	hctx->driver_data = &pdev->queues[0];
	return 0;
}

static int nvme_plat_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
			       unsigned int hctx_idx)
{
	struct nvme_plat_dev *pdev = to_nvme_plat_dev(data);

	hctx->driver_data = &pdev->queues[hctx_idx + 1];
	return 0;
}

static int nvme_plat_init_request(struct blk_mq_tag_set *set,
				  struct request *req, unsigned int hctx_idx,
				  int numa_node)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_request *nreq = nvme_req(req);

	nreq->ctrl = set->driver_data;
	nreq->cmd = &iod->cmd;
	return 0;
}

static void nvme_plat_disable(struct nvme_plat_dev *pdev, bool shutdown);

static enum blk_eh_timer_return nvme_plat_timeout(struct request *req)
{
	struct nvme_plat_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_plat_queue *q = iod->q;
	struct nvme_plat_dev *pdev = q->dev;
	u32 csts = readl(pdev->bar + NVME_REG_CSTS);

	if (nvme_ctrl_state(&pdev->ctrl) != NVME_CTRL_LIVE) {
		/*
		 * From rdma.c: when resetting, connecting or deleting, complete
		 * at once, the request may block the teardown or setup sequence;
		 * the error recovery cancels all the others.
		 */
		dev_warn(pdev->dev, "I/O %d (queue %d) timeout while not live\n",
			 req->tag, q->qid);
		if (blk_mq_request_started(req) && !blk_mq_request_completed(req)) {
			nvme_req(req)->status = NVME_SC_HOST_ABORTED_CMD;
			nvme_req(req)->flags |= NVME_REQ_CANCELLED;
			blk_mq_complete_request(req);
		}
		return BLK_EH_DONE;
	}

	/* check if we just missed an interrupt */
	if (!(csts & NVME_CSTS_CFS)) {
		nvme_plat_handle_cq(q, false);
		if (blk_mq_request_completed(req)) {
			dev_warn(pdev->dev, "I/O %d (queue %d) timeout: completion polled\n",
				 req->tag, q->qid);
			return BLK_EH_DONE;
		}
	}

	dev_warn(pdev->dev, "I/O %d (queue %d) timeout: resetting controller\n",
		 req->tag, q->qid);
	nvme_req(req)->flags |= NVME_REQ_CANCELLED;
	nvme_plat_disable(pdev, false);
	nvme_reset_ctrl(&pdev->ctrl);
	return BLK_EH_DONE;
}

static const struct blk_mq_ops nvme_plat_mq_admin_ops = {
	.queue_rq	= nvme_plat_queue_rq,
	.complete	= nvme_plat_complete_rq,
	.init_hctx	= nvme_plat_admin_init_hctx,
	.init_request	= nvme_plat_init_request,
	.timeout	= nvme_plat_timeout,
};

static const struct blk_mq_ops nvme_plat_mq_ops = {
	.queue_rq	= nvme_plat_queue_rq,
	.complete	= nvme_plat_complete_rq,
	.init_hctx	= nvme_plat_init_hctx,
	.init_request	= nvme_plat_init_request,
	.timeout	= nvme_plat_timeout,
};

/*
 * Queues
 */

static int nvme_plat_alloc_queue(struct nvme_plat_dev *pdev, u16 qid, u16 depth)
{
	struct nvme_plat_queue *q = &pdev->queues[qid];

	q->cqes = dmam_alloc_coherent(pdev->dev, depth * sizeof(struct nvme_completion),
				      &q->cq_dma_addr, GFP_KERNEL);
	if (!q->cqes)
		return -ENOMEM;

	q->sqes = dmam_alloc_coherent(pdev->dev, depth * sizeof(struct nvme_command),
				      &q->sq_dma_addr, GFP_KERNEL);
	if (!q->sqes)
		return -ENOMEM;

	q->dev = pdev;
	q->qid = qid;
	q->depth = depth;
	q->sq_db = pdev->bar + NVME_REG_DBS + qid * 2 * pdev->db_stride * 4;
	q->cq_db = q->sq_db + pdev->db_stride;
	spin_lock_init(&q->sq_lock);
	spin_lock_init(&q->cq_lock);
	return 0;
}

/* The admin queue on vector 0; the I/O queues spread over the other lines */
static u16 nvme_plat_queue_vector(struct nvme_plat_dev *pdev, u16 qid)
{
	if (!qid || pdev->nr_irqs == 1)
		return 0;
	return (qid - 1) % (pdev->nr_irqs - 1) + 1;
}

static void nvme_plat_init_queue(struct nvme_plat_queue *q)
{
	q->sq_tail = 0;
	q->cq_head = 0;
	q->cq_phase = 1;
	q->vector = nvme_plat_queue_vector(q->dev, q->qid);
	memset(q->cqes, 0, q->depth * sizeof(struct nvme_completion));
	WRITE_ONCE(q->enabled, true);
	wmb(); /* ensure the first interrupt sees the initialization */
}

static int nvme_plat_create_cq(struct nvme_plat_dev *pdev, struct nvme_plat_queue *q)
{
	struct nvme_command c = {};

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(q->cq_dma_addr);
	c.create_cq.cqid = cpu_to_le16(q->qid);
	c.create_cq.qsize = cpu_to_le16(q->depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED);
	c.create_cq.irq_vector = cpu_to_le16(nvme_plat_queue_vector(pdev, q->qid));

	return nvme_submit_sync_cmd(pdev->ctrl.admin_q, &c, NULL, 0);
}

static int nvme_plat_create_sq(struct nvme_plat_dev *pdev, struct nvme_plat_queue *q)
{
	struct nvme_command c = {};

	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(q->sq_dma_addr);
	c.create_sq.sqid = cpu_to_le16(q->qid);
	c.create_sq.qsize = cpu_to_le16(q->depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(NVME_QUEUE_PHYS_CONTIG);
	c.create_sq.cqid = cpu_to_le16(q->qid);

	return nvme_submit_sync_cmd(pdev->ctrl.admin_q, &c, NULL, 0);
}

static int nvme_plat_delete_queue(struct nvme_plat_dev *pdev, u8 opcode, u16 qid)
{
	struct nvme_command c = {};

	c.delete_queue.opcode = opcode;
	c.delete_queue.qid = cpu_to_le16(qid);

	return nvme_submit_sync_cmd(pdev->ctrl.admin_q, &c, NULL, 0);
}

static int nvme_plat_setup_io_queues(struct nvme_plat_dev *pdev)
{
	int nr_io_queues = pdev->max_io_queues;
	unsigned int i;
	int ret;

	ret = nvme_set_queue_count(&pdev->ctrl, &nr_io_queues);
	if (ret)
		return ret;
	if (nr_io_queues == 0) {
		dev_warn(pdev->dev, "no I/O queue, admin queue only\n");
		pdev->nr_io_queues = 0;
		pdev->ctrl.queue_count = 1;
		return 0;
	}

	pdev->nr_io_queues = min_t(unsigned int, nr_io_queues, pdev->max_io_queues);
	pdev->ctrl.queue_count = pdev->nr_io_queues + 1;

	for (i = 1; i <= pdev->nr_io_queues; i++) {
		struct nvme_plat_queue *q = &pdev->queues[i];

		ret = nvme_plat_create_cq(pdev, q);
		if (ret)
			return ret;
		ret = nvme_plat_create_sq(pdev, q);
		if (ret) {
			nvme_plat_delete_queue(pdev, nvme_admin_delete_cq, q->qid);
			return ret;
		}
		nvme_plat_init_queue(q);
	}

	dev_info(pdev->dev, "%u I/O queues, %d interrupt lines\n",
		 pdev->nr_io_queues, pdev->nr_irqs);
	return 0;
}

static void nvme_plat_delete_io_queues(struct nvme_plat_dev *pdev)
{
	unsigned int i;

	for (i = pdev->nr_io_queues; i >= 1; i--) {
		if (!READ_ONCE(pdev->queues[i].enabled))
			continue;
		nvme_plat_delete_queue(pdev, nvme_admin_delete_sq, i);
		nvme_plat_delete_queue(pdev, nvme_admin_delete_cq, i);
	}
}

/*
 * Controller
 */

static void nvme_plat_disable(struct nvme_plat_dev *pdev, bool shutdown)
{
	enum nvme_ctrl_state state = nvme_ctrl_state(&pdev->ctrl);
	u32 csts = readl(pdev->bar + NVME_REG_CSTS);
	bool dead = false, freeze = false;
	unsigned int i;

	if (!(csts & NVME_CSTS_RDY) || (csts & NVME_CSTS_CFS))
		dead = true;

	if (state == NVME_CTRL_LIVE || state == NVME_CTRL_RESETTING) {
		freeze = true;
		nvme_start_freeze(&pdev->ctrl);
	}

	/*
	 * Give the controller a chance to complete all entered requests if
	 * doing a safe shutdown.
	 */
	if (!dead && shutdown && freeze)
		nvme_wait_freeze_timeout(&pdev->ctrl);

	nvme_quiesce_io_queues(&pdev->ctrl);

	if (!dead) {
		nvme_plat_delete_io_queues(pdev);
		nvme_disable_ctrl(&pdev->ctrl, shutdown);
	}

	for (i = 0; i <= pdev->nr_io_queues; i++)
		WRITE_ONCE(pdev->queues[i].enabled, false);
	mb(); /* ensure that queue_rq sees that enabled is cleared */
	nvme_quiesce_admin_queue(&pdev->ctrl);

	/* last chance to complete any requests before nvme_cancel_request */
	for (i = 0; i <= pdev->nr_io_queues; i++)
		nvme_plat_handle_cq(&pdev->queues[i], true);

	if (pdev->ctrl.tagset)
		nvme_cancel_tagset(&pdev->ctrl);
	nvme_cancel_admin_tagset(&pdev->ctrl);

	/*
	 * The driver will not be starting up queues again if shutting down so
	 * must flush all entered requests to their failed completion to avoid
	 * deadlocking blk-mq hot-cpu notifier.
	 */
	if (shutdown) {
		nvme_unquiesce_io_queues(&pdev->ctrl);
		nvme_unquiesce_admin_queue(&pdev->ctrl);
	}
}

static void nvme_plat_reset_work(struct work_struct *work)
{
	struct nvme_plat_dev *pdev = container_of(work, struct nvme_plat_dev, ctrl.reset_work);
	enum nvme_ctrl_state state = nvme_ctrl_state(&pdev->ctrl);
	u32 aqa;
	int ret;

	if (state != NVME_CTRL_RESETTING) {
		dev_warn(pdev->dev, "ctrl state %d is not RESETTING\n", state);
		ret = -ENODEV;
		goto out;
	}

	if (pdev->ctrl.ctrl_config & NVME_CC_ENABLE)
		nvme_plat_disable(pdev, false);

	/* Limit the size of a command so that its iod->sg fits in a page */
	pdev->ctrl.max_hw_sectors = min_t(u32, NVME_MAX_KB_SZ << 1,
					  dma_max_mapping_size(pdev->dev) >> 9);
	pdev->ctrl.max_segments = NVME_MAX_SEGS;
	dma_set_max_seg_size(pdev->dev, 0xffffffff);

	/* The admin queue */
	ret = nvme_disable_ctrl(&pdev->ctrl, false);
	if (ret)
		goto out;
	aqa = NVME_AQ_DEPTH - 1;
	aqa |= aqa << 16;
	writel(aqa, pdev->bar + NVME_REG_AQA);
	lo_hi_writeq(pdev->queues[0].sq_dma_addr, pdev->bar + NVME_REG_ASQ);
	lo_hi_writeq(pdev->queues[0].cq_dma_addr, pdev->bar + NVME_REG_ACQ);

	ret = nvme_enable_ctrl(&pdev->ctrl);
	if (ret)
		goto out;

	nvme_plat_init_queue(&pdev->queues[0]);
	nvme_unquiesce_admin_queue(&pdev->ctrl);

	if (!nvme_change_ctrl_state(&pdev->ctrl, NVME_CTRL_CONNECTING)) {
		dev_warn(pdev->ctrl.device, "failed to mark controller CONNECTING\n");
		ret = -ENODEV;
		goto out;
	}

	ret = nvme_init_ctrl_finish(&pdev->ctrl, false);
	if (ret)
		goto out;

	/* The I/O queues */
	ret = nvme_plat_setup_io_queues(pdev);
	if (ret)
		goto out;

	if (pdev->nr_io_queues) {
		if (!pdev->ctrl.tagset) {
			ret = nvme_alloc_io_tag_set(&pdev->ctrl, &pdev->tagset,
						    &nvme_plat_mq_ops, 1,
						    sizeof(struct nvme_plat_iod));
			if (ret)
				goto out;
		} else {
			nvme_unquiesce_io_queues(&pdev->ctrl);
			nvme_wait_freeze(&pdev->ctrl);
			blk_mq_update_nr_hw_queues(pdev->ctrl.tagset, pdev->nr_io_queues);
			nvme_unfreeze(&pdev->ctrl);
		}
	}

	if (!nvme_change_ctrl_state(&pdev->ctrl, NVME_CTRL_LIVE)) {
		dev_warn(pdev->ctrl.device, "failed to mark controller live state\n");
		ret = -ENODEV;
		goto out;
	}

	nvme_start_ctrl(&pdev->ctrl);
	return;

out:
	dev_warn(pdev->ctrl.device, "Reset failure status: %d\n", ret);
	nvme_change_ctrl_state(&pdev->ctrl, NVME_CTRL_DELETING);
	nvme_get_ctrl(&pdev->ctrl);
	nvme_plat_disable(pdev, false);
	nvme_mark_namespaces_dead(&pdev->ctrl);
	if (!queue_work(nvme_wq, &pdev->remove_work))
		nvme_put_ctrl(&pdev->ctrl);
}

static void nvme_plat_remove_dead_ctrl_work(struct work_struct *work)
{
	struct nvme_plat_dev *pdev = container_of(work, struct nvme_plat_dev, remove_work);

	nvme_put_ctrl(&pdev->ctrl);
	device_release_driver(pdev->dev);
}

/* 64-bit registers in two 32-bit halves, low first (NVMe platform binding) */
static int nvme_plat_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	*val = readl(to_nvme_plat_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_plat_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	writel(val, to_nvme_plat_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_plat_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	*val = lo_hi_readq(to_nvme_plat_dev(ctrl)->bar + off);
	return 0;
}

static void nvme_plat_submit_async_event(struct nvme_ctrl *ctrl)
{
	struct nvme_plat_dev *pdev = to_nvme_plat_dev(ctrl);
	struct nvme_command c = {};

	c.common.opcode = nvme_admin_async_event;
	c.common.command_id = NVME_AQ_BLK_MQ_DEPTH;
	nvme_plat_submit_cmd(&pdev->queues[0], &c);
}

static int nvme_plat_get_address(struct nvme_ctrl *ctrl, char *buf, int size)
{
	return snprintf(buf, size, "%s\n", dev_name(to_nvme_plat_dev(ctrl)->dev));
}

static void nvme_plat_free_ctrl(struct nvme_ctrl *ctrl)
{
	struct nvme_plat_dev *pdev = to_nvme_plat_dev(ctrl);

	if (ctrl->tagset)
		nvme_remove_io_tag_set(ctrl);
	put_device(pdev->dev);
}

static const struct nvme_ctrl_ops nvme_plat_ctrl_ops = {
	.name			= "nvme-platform",
	.module			= THIS_MODULE,
	.flags			= 0,
	.reg_read32		= nvme_plat_reg_read32,
	.reg_write32		= nvme_plat_reg_write32,
	.reg_read64		= nvme_plat_reg_read64,
	.free_ctrl		= nvme_plat_free_ctrl,
	.submit_async_event	= nvme_plat_submit_async_event,
	.get_address		= nvme_plat_get_address,
	.get_virt_boundary	= nvme_get_virt_boundary,
};

/*
 * Probe and removal
 */

static void nvme_plat_async_probe(void *data, async_cookie_t cookie)
{
	struct nvme_plat_dev *pdev = data;

	flush_work(&pdev->ctrl.reset_work);
	flush_work(&pdev->ctrl.scan_work);
	nvme_put_ctrl(&pdev->ctrl);
}

static void devm_nvme_plat_mempool_destroy(void *data)
{
	mempool_destroy(data);
}

static int nvme_plat_alloc_queues(struct nvme_plat_dev *pdev)
{
	u64 cap = lo_hi_readq(pdev->bar + NVME_REG_CAP);
	unsigned int i;
	int ret;

	pdev->ctrl.cap = cap;
	pdev->db_stride = 1 << NVME_CAP_STRIDE(cap);
	pdev->io_depth = min_t(u32, NVME_CAP_MQES(cap) + 1, NVME_PLAT_IO_QUEUE_DEPTH);
	pdev->ctrl.sqsize = pdev->io_depth - 1;	/* 0's based queue depth */

	pdev->max_io_queues = min_t(unsigned int, num_possible_cpus(),
				    NVME_PLAT_MAX_IO_QUEUES);
	pdev->queues = devm_kcalloc(pdev->dev, pdev->max_io_queues + 1,
				    sizeof(*pdev->queues), GFP_KERNEL);
	if (!pdev->queues)
		return -ENOMEM;

	ret = nvme_plat_alloc_queue(pdev, 0, NVME_AQ_DEPTH);
	if (ret)
		return ret;
	for (i = 1; i <= pdev->max_io_queues; i++) {
		ret = nvme_plat_alloc_queue(pdev, i, pdev->io_depth);
		if (ret)
			return ret;
	}
	return 0;
}

static int nvme_plat_request_irqs(struct nvme_plat_dev *pdev,
				  struct platform_device *plat)
{
	int i, ret;

	pdev->nr_irqs = platform_irq_count(plat);
	if (pdev->nr_irqs < 0)
		return pdev->nr_irqs;
	if (pdev->nr_irqs == 0)
		return -ENXIO;

	pdev->irqs = devm_kcalloc(pdev->dev, pdev->nr_irqs, sizeof(*pdev->irqs), GFP_KERNEL);
	if (!pdev->irqs)
		return -ENOMEM;

	for (i = 0; i < pdev->nr_irqs; i++) {
		pdev->irqs[i] = platform_get_irq(plat, i);
		if (pdev->irqs[i] < 0)
			return pdev->irqs[i];
		ret = devm_request_irq(pdev->dev, pdev->irqs[i], nvme_plat_irq, 0,
				       dev_name(pdev->dev), pdev);
		if (ret)
			return ret;
	}
	return 0;
}

static struct nvme_plat_dev *nvme_plat_alloc(struct platform_device *plat)
{
	struct device *dev = &plat->dev;
	struct nvme_plat_dev *pdev;
	int ret;

	pdev = devm_kzalloc(dev, sizeof(*pdev), GFP_KERNEL);
	if (!pdev)
		return ERR_PTR(-ENOMEM);

	pdev->dev = get_device(dev);
	platform_set_drvdata(plat, pdev);

	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64))) {
		ret = -ENXIO;
		goto put_dev;
	}

	pdev->bar = devm_platform_ioremap_resource(plat, 0);
	if (IS_ERR(pdev->bar)) {
		ret = PTR_ERR(pdev->bar);
		goto put_dev;
	}

	INIT_WORK(&pdev->ctrl.reset_work, nvme_plat_reset_work);
	INIT_WORK(&pdev->remove_work, nvme_plat_remove_dead_ctrl_work);

	ret = nvme_plat_alloc_queues(pdev);
	if (ret)
		goto put_dev;

	pdev->prp_page_pool = dmam_pool_create("prp list page", dev, NVME_CTRL_PAGE_SIZE,
					       NVME_CTRL_PAGE_SIZE, 0);
	pdev->prp_small_pool = dmam_pool_create("prp list 256", dev, 256, 256, 0);
	if (!pdev->prp_page_pool || !pdev->prp_small_pool) {
		ret = -ENOMEM;
		goto put_dev;
	}

	WARN_ON_ONCE(nvme_plat_iod_alloc_size() > PAGE_SIZE);
	pdev->iod_mempool = mempool_create_kmalloc_pool(1, nvme_plat_iod_alloc_size());
	if (!pdev->iod_mempool) {
		ret = -ENOMEM;
		goto put_dev;
	}
	ret = devm_add_action_or_reset(dev, devm_nvme_plat_mempool_destroy, pdev->iod_mempool);
	if (ret)
		goto put_dev;

	ret = nvme_plat_request_irqs(pdev, plat);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request the interrupts\n");
		goto put_dev;
	}

	ret = nvme_init_ctrl(&pdev->ctrl, dev, &nvme_plat_ctrl_ops, 0);
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialize nvme_ctrl\n");
		goto put_dev;
	}

	return pdev;
put_dev:
	put_device(pdev->dev);
	return ERR_PTR(ret);
}

static int nvme_plat_probe(struct platform_device *plat)
{
	struct nvme_plat_dev *pdev;
	int ret;

	pdev = nvme_plat_alloc(plat);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	ret = nvme_add_ctrl(&pdev->ctrl);
	if (ret)
		goto out_put_ctrl;

	ret = nvme_alloc_admin_tag_set(&pdev->ctrl, &pdev->admin_tagset,
				       &nvme_plat_mq_admin_ops, sizeof(struct nvme_plat_iod));
	if (ret)
		goto out_uninit_ctrl;

	dev_info(pdev->dev, "NVMe controller, %d interrupt lines\n", pdev->nr_irqs);

	nvme_reset_ctrl(&pdev->ctrl);
	async_schedule(nvme_plat_async_probe, pdev);
	return 0;

out_uninit_ctrl:
	nvme_uninit_ctrl(&pdev->ctrl);
out_put_ctrl:
	nvme_put_ctrl(&pdev->ctrl);
	return ret;
}

static void nvme_plat_remove(struct platform_device *plat)
{
	struct nvme_plat_dev *pdev = platform_get_drvdata(plat);

	nvme_change_ctrl_state(&pdev->ctrl, NVME_CTRL_DELETING);
	flush_work(&pdev->ctrl.reset_work);
	nvme_stop_ctrl(&pdev->ctrl);
	nvme_remove_namespaces(&pdev->ctrl);
	nvme_plat_disable(pdev, true);
	nvme_remove_admin_tag_set(&pdev->ctrl);
	nvme_uninit_ctrl(&pdev->ctrl);
}

static void nvme_plat_shutdown(struct platform_device *plat)
{
	struct nvme_plat_dev *pdev = platform_get_drvdata(plat);

	nvme_plat_disable(pdev, true);
}

static int nvme_plat_suspend(struct device *dev)
{
	nvme_plat_disable(dev_get_drvdata(dev), true);
	return 0;
}

static int nvme_plat_resume(struct device *dev)
{
	struct nvme_plat_dev *pdev = dev_get_drvdata(dev);

	return nvme_reset_ctrl(&pdev->ctrl);
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvme_plat_pm_ops, nvme_plat_suspend, nvme_plat_resume);

static const struct of_device_id nvme_plat_of_match[] = {
	{ .compatible = "chipsit,nvme-ssd" },
	{},
};
MODULE_DEVICE_TABLE(of, nvme_plat_of_match);

static struct platform_driver nvme_plat_driver = {
	.driver = {
		.name		= "nvme-platform",
		.of_match_table	= nvme_plat_of_match,
		.pm		= pm_sleep_ptr(&nvme_plat_pm_ops),
	},
	.probe		= nvme_plat_probe,
	.remove		= nvme_plat_remove,
	.shutdown	= nvme_plat_shutdown,
};
module_platform_driver(nvme_plat_driver);

MODULE_AUTHOR("Lorenzo Zuolo <lorenzo.zuolo@chips.it>");
MODULE_DESCRIPTION("NVM Express driver for controllers on the SoC interconnect");
MODULE_LICENSE("GPL");
