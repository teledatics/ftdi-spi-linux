/*
 * FTDI FT232H SPI Host Driver
 * GPL-2.0
 * Copyright James Ewing <james@teledatics.com>
 * Copyright Yuji Sasaki <sasaki@silexamerica.com>
 * Based on work by Anatolij Gustschin <agust@denx.de>
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/idr.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/string.h>
#include <linux/scatterlist.h>
#include <linux/err.h>
#if IS_ENABLED(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#include "ft232h-intf.h"

#undef FTDI_IRQ_SPI_POLL

static int param_latency = 1;
module_param_named(latency, param_latency, int, 0600);
MODULE_PARM_DESC(latency, "latency timer value (1ms ~ 255ms, default 1ms)");

int usb_wait_msec = 0;
module_param(usb_wait_msec, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(usb_wait_msec, "Wait after USB transfer in msec");

static int param_gpio_base = -1;
module_param_named(gpio_base_num, param_gpio_base, int, 0600);
MODULE_PARM_DESC(gpio_base_num, "GPIO controller base number (if negative, dynamic allocation)");

static int param_bus_num = -1;
module_param_named(spi_bus_num, param_bus_num, int, 0600);
MODULE_PARM_DESC(spi_bus_num, "SPI controller bus number (if negative, dynamic allocation)");

/*
 * Performance tuning knobs – defaults keep legacy behaviour but allow
 * opt-in batching/latency trade-offs when explicitly configured.
 */
enum ftdi_perf_profile {
	PERF_PROFILE_LEGACY = 0,
	PERF_PROFILE_BALANCED = 1,
	PERF_PROFILE_AGGRESSIVE = 2,
};

static int param_perf_profile = PERF_PROFILE_AGGRESSIVE;
module_param_named(perf_profile, param_perf_profile, int, 0600);
MODULE_PARM_DESC(perf_profile, "Performance profile: 0=legacy, 1=balanced, 2=aggressive");

static unsigned int param_max_block;
module_param_named(max_block, param_max_block, uint, 0600);
MODULE_PARM_DESC(max_block, "Maximum SPI payload bytes per FTDI burst (0=auto)");

static unsigned int param_bulk_in_buf_kb;
module_param_named(bulk_in_buf_kb, param_bulk_in_buf_kb, uint, 0600);
MODULE_PARM_DESC(bulk_in_buf_kb, "Bulk-in buffer size in KiB (0=endpoint max)");

static bool param_flush_per_block = false;
static bool param_flush_overridden;

static int ftdi_param_set_flush(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret)
		param_flush_overridden = true;
	return ret;
}

static int ftdi_param_get_flush(char *buffer, const struct kernel_param *kp)
{
	return param_get_bool(buffer, kp);
}

static const struct kernel_param_ops ftdi_flush_param_ops = {
	.set = ftdi_param_set_flush,
	.get = ftdi_param_get_flush,
};

module_param_cb(flush_per_block, &ftdi_flush_param_ops,
		      &param_flush_per_block, 0600);
MODULE_PARM_DESC(flush_per_block,
		      "Force SEND_IMMEDIATE after every SPI payload block");

static unsigned int param_rx_retry_us;
module_param_named(rx_retry_us, param_rx_retry_us, uint, 0600);
MODULE_PARM_DESC(rx_retry_us, "Delay in usec between bulk-in polls when no data is returned");

static bool param_enable_stats = true;
module_param_named(enable_stats, param_enable_stats, bool, 0600);
MODULE_PARM_DESC(enable_stats, "Enable debugfs performance statistics (default disabled)");

static unsigned int param_pipeline_depth;
static bool param_pipeline_overridden;

static int ftdi_param_set_pipeline_depth(const char *val,
					     const struct kernel_param *kp)
{
	int ret = param_set_uint(val, kp);

	if (!ret)
		param_pipeline_overridden = true;
	return ret;
}

static int ftdi_param_get_pipeline_depth(char *buffer,
					     const struct kernel_param *kp)
{
	return param_get_uint(buffer, kp);
}

static const struct kernel_param_ops ftdi_pipeline_param_ops = {
	.set = ftdi_param_set_pipeline_depth,
	.get = ftdi_param_get_pipeline_depth,
};

module_param_cb(pipeline_depth, &ftdi_pipeline_param_ops,
		      &param_pipeline_depth, 0600);
MODULE_PARM_DESC(pipeline_depth,
		      "Number of in-flight read URBs (0=auto per perf profile)");

#ifdef FTDI_IRQ_SPI_POLL
static unsigned int irq_poll_period = 0;
module_param(irq_poll_period, uint, 0644);
MODULE_PARM_DESC(irq_poll_period, "GPIO polling period in ms (default 5 ms)");
#endif

#define SPI_INTF_DEVNAME	"spi-ft232h"

/* SPI controller/master Compatibility Layer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)
#define spi_controller spi_master
#endif

/* Device info struct used for device specific init. */
struct ft232h_intf_info {
	int (*probe)(struct usb_interface *intf, const void *plat_data);
	int (*remove)(struct usb_interface *intf);
	const void *plat_data; /* optional, passed to probe() */
};

struct ft232h_intf_priv {
	struct usb_interface	*intf;
	struct usb_device	*udev;
	struct mutex		io_mutex; /* sync I/O with disconnect */
	struct mutex		ops_mutex;
	int			id;
	int			index;
	u8			bulk_in;
	u8			bulk_out;
	size_t			bulk_in_sz;
	size_t			bulk_in_pkt_sz;
	void			*bulk_in_buf;

	const struct usb_device_id	*usb_dev_id;
	struct ft232h_intf_info		*info;
	struct platform_device		*spi_pdev;
	struct gpiod_lookup_table	*lookup_gpios;
	struct task_struct		*gpio_thread;
	struct completion		gpio_thread_complete;

	struct gpio_chip	mpsse_gpio;
	u8			gpiol_mask;
	u8			gpioh_mask;
	u8			gpiol_dir;
	u8			gpioh_dir;
	u8			tx_buf[4];
	
	struct irq_chip		mpsse_irq;
	int			irq_base;
	bool              	irq_enabled[FTDI_MPSSE_GPIOS];
	int			irq_type[FTDI_MPSSE_GPIOS];
};

/* Cumulative transfer counters exported through debugfs for benchmarking. */
struct ftdi_spi_stats {
	u64 tx_bytes;
	u64 rx_bytes;
	u64 total_transfers;
	u64 xfers_full_duplex;
	u64 xfers_tx_only;
	u64 xfers_rx_only;
	u64 read_timeouts;
	u64 read_empty_loops;
	u64 transfer_errors;
	u64 total_latency_ns;
	u64 max_latency_ns;
	u64 tx_copyfree_bytes;
	u64 tx_copyfree_chunks;
	u64 tx_copyfree_fallbacks;
	u64 pipeline_max_inflight;
	u64 pipeline_wait_events;
	u64 tx_chunks_total;
	u64 tx_partial_chunks;
	u64 tx_timeout_errors;
	u64 bytes_since_tune;
	u64 latency_min_ns;
	u64 latency_max_ns;
};

#define FTDI_PIPELINE_MAX	4

struct ftdi_urb_ctx {
	struct urb *urb;
	u8 *buf;
	struct completion done;
	u8 *dst;
	size_t expected;
	size_t actual;
	int status;
};

struct ftdi_pipeline {
	struct ftdi_urb_ctx *ctxs;
	unsigned int depth;
	unsigned int head;
	unsigned int tail;
	unsigned int in_flight;
};

#if IS_ENABLED(CONFIG_DEBUG_FS)
static struct dentry *ftdi_debugfs_root;
#endif

struct ftdi_spi {
	struct platform_device *pdev;
	struct usb_interface *intf;
	struct spi_controller *master;
	const struct ft232h_intf_ops *iops;
	u8 txrx_cmd;
	u8 rx_cmd;
	u8 tx_cmd;
	u8 xfer_buf[SZ_64K];
	u16 last_mode;
	u32 last_speed_hz;
	size_t max_burst_bytes;
	bool flush_per_block;
	u32 rx_retry_delay_us;
	unsigned int rx_max_loops;
	bool stats_enabled;
	spinlock_t stats_lock;
	struct ftdi_spi_stats stats;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry *debugfs_root;
#endif
	struct ftdi_pipeline pipeline;
	unsigned long tuning_jiffies;
	unsigned int tuning_rounds;
	bool flush_forced;
	unsigned int pipeline_forced;
	bool copyfree_disabled;
};

static DEFINE_IDA(ftdi_devid_ida);

static int ftdi_spi_pipeline_setup(struct ftdi_spi *priv, unsigned int depth);
static void ftdi_spi_pipeline_teardown(struct ftdi_spi *priv);
static void ftdi_spi_stats_copy_free(struct ftdi_spi *priv, size_t bytes,
				     bool success);
static void ftdi_spi_stats_pipeline_submit(struct ftdi_spi *priv,
					     unsigned int in_flight);
static void ftdi_spi_stats_pipeline_wait(struct ftdi_spi *priv);
static int ftdi_spi_push_buf(struct ftdi_spi *priv, const void *buf, size_t len);

enum gpiol {
	MPSSE_SK	= BIT(0),
	MPSSE_DO	= BIT(1),
	MPSSE_DI	= BIT(2),
	MPSSE_CS	= BIT(3),
};

#define MPSSE_DIR_OUTPUT        (MPSSE_SK | MPSSE_DO | MPSSE_CS)

/* SPI controller/master Compatibility Layer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 0)

static inline void *spi_controller_get_devdata(struct spi_controller *ctlr)
{
	return dev_get_drvdata(&ctlr->dev);
}

static inline void spi_controller_set_devdata(struct spi_controller *ctlr,
                                              void *data)
{
	dev_set_drvdata(&ctlr->dev, data);
}

static inline struct spi_controller *spi_controller_get(struct spi_controller *ctlr)
{
	if (!ctlr || !get_device(&ctlr->dev))
		return NULL;
	return ctlr;
}

static inline void spi_controller_put(struct spi_controller *ctlr)
{
	if (ctlr)
		put_device(&ctlr->dev);
}

#define spi_register_controller(_ctlr) spi_register_master(_ctlr)
#define spi_unregister_controller(_ctlr) spi_unregister_master(_ctlr)
#endif

/* Handle scalar chip_select (<= v6.7) and array+mask (v6.8+). */
#if defined(SPI_DEVICE_CS_CNT_MAX)
static inline unsigned int ftdi_spi_chip_select(struct spi_device *spi)
{
	u8 idx = spi->cs_index_mask ? ffs(spi->cs_index_mask) - 1 : 0;
	u8 num = spi->num_chipselect ? spi->num_chipselect : 1;

	if (idx >= num)
		idx = 0;

	return spi_get_chipselect(spi, idx);
}
#elif defined(SPI_CS_CNT_MAX)
static inline unsigned int ftdi_spi_chip_select(struct spi_device *spi)
{
	u8 idx = spi->cs_index_mask ? ffs(spi->cs_index_mask) - 1 : 0;

	if (idx >= SPI_CS_CNT_MAX)
		idx = 0;

	return spi_get_chipselect(spi, idx);
}
#else
static inline unsigned int ftdi_spi_chip_select(struct spi_device *spi)
{
	return spi->chip_select;
}
#endif

static void ftdi_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(spi->controller);
	unsigned int cs = ftdi_spi_chip_select(spi);

	priv->iops->gpio_set(priv->intf, cs, enable);
}

static inline u8 ftdi_spi_txrx_byte_cmd(struct spi_device *spi)
{
	u8 mode = spi->mode & (SPI_CPOL | SPI_CPHA);
	u8 cmd;

	if (spi->mode & SPI_LSB_FIRST) {
		switch (mode) {
		case SPI_MODE_0:
		case SPI_MODE_1:
			cmd = TXF_RXR_BYTES_LSB;
			break;
		case SPI_MODE_2:
		case SPI_MODE_3:
			cmd = TXR_RXF_BYTES_LSB;
			break;
		}
	} else {
		switch (mode) {
		case SPI_MODE_0:
		case SPI_MODE_1:
			cmd = TXF_RXR_BYTES_MSB;
			break;
		case SPI_MODE_2:
		case SPI_MODE_3:
			cmd = TXR_RXF_BYTES_MSB;
			break;
		}
	}
	return cmd;
}

static inline int ftdi_spi_loopback_cfg(struct ftdi_spi *priv, int on)
{
	int ret;

	priv->xfer_buf[0] = on ? LOOPBACK_ON : LOOPBACK_OFF;

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 1);
	if (ret < 0)
		dev_warn(&priv->pdev->dev, "loopback %d failed\n", on);
	return ret;
}

static void ftdi_spi_stats_account_rx(struct ftdi_spi *priv,
				     unsigned int empty_loops,
				     unsigned int timeouts)
{
	unsigned long flags;

	if (!priv->stats_enabled || (!empty_loops && !timeouts))
		return;

	/* Accumulate the amount of polling the fast path required. */
	spin_lock_irqsave(&priv->stats_lock, flags);
	priv->stats.read_empty_loops += empty_loops;
	priv->stats.read_timeouts += timeouts;
	spin_unlock_irqrestore(&priv->stats_lock, flags);
}

static void ftdi_spi_stats_complete(struct ftdi_spi *priv,
				       struct spi_transfer *xfer,
				       int status,
				       u64 duration_ns)
{
	unsigned long flags;
	u64 tx_bytes = xfer->tx_buf ? xfer->len : 0;
	u64 rx_bytes = xfer->rx_buf ? xfer->len : 0;

	if (!priv->stats_enabled)
		return;

	spin_lock_irqsave(&priv->stats_lock, flags);
	priv->stats.total_transfers++;
	if (xfer->tx_buf && xfer->rx_buf)
		priv->stats.xfers_full_duplex++;
	else if (xfer->tx_buf)
		priv->stats.xfers_tx_only++;
	else if (xfer->rx_buf)
		priv->stats.xfers_rx_only++;

	priv->stats.tx_bytes += tx_bytes;
	priv->stats.rx_bytes += rx_bytes;
	priv->stats.bytes_since_tune += tx_bytes + rx_bytes;
	priv->stats.total_latency_ns += duration_ns;
	if (duration_ns > priv->stats.max_latency_ns)
		priv->stats.max_latency_ns = duration_ns;
	if (!priv->stats.latency_min_ns || duration_ns < priv->stats.latency_min_ns)
		priv->stats.latency_min_ns = duration_ns;
	if (duration_ns > priv->stats.latency_max_ns)
		priv->stats.latency_max_ns = duration_ns;
	if (status < 0)
		priv->stats.transfer_errors++;
	spin_unlock_irqrestore(&priv->stats_lock, flags);
}

#define FTDI_READ_TIMEOUT_JIFFIES	msecs_to_jiffies(FTDI_USB_READ_TIMEOUT)

static void ftdi_spi_stats_copy_free(struct ftdi_spi *priv, size_t bytes,
				     bool success)
{
	unsigned long flags;

	if (!priv->stats_enabled)
		return;

	spin_lock_irqsave(&priv->stats_lock, flags);
	if (success) {
		priv->stats.tx_copyfree_bytes += bytes;
		priv->stats.tx_copyfree_chunks++;
	} else {
		priv->stats.tx_copyfree_fallbacks++;
	}
	spin_unlock_irqrestore(&priv->stats_lock, flags);
}
static void ftdi_spi_stats_tx_chunk(struct ftdi_spi *priv, size_t bytes,
				   bool partial, bool timeout)
{
	unsigned long flags;

	if (!priv->stats_enabled)
		return;

	spin_lock_irqsave(&priv->stats_lock, flags);
	priv->stats.tx_chunks_total++;
	if (partial)
		priv->stats.tx_partial_chunks++;
	if (timeout)
		priv->stats.tx_timeout_errors++;
	spin_unlock_irqrestore(&priv->stats_lock, flags);
}

static void ftdi_spi_stats_pipeline_submit(struct ftdi_spi *priv,
                                           unsigned int in_flight)
{
	unsigned long flags;

	if (!priv->stats_enabled)
		return;

	spin_lock_irqsave(&priv->stats_lock, flags);
	if (in_flight > priv->stats.pipeline_max_inflight)
		priv->stats.pipeline_max_inflight = in_flight;
	spin_unlock_irqrestore(&priv->stats_lock, flags);
}

static void ftdi_spi_stats_pipeline_wait(struct ftdi_spi *priv)
{
	unsigned long flags;

	if (!priv->stats_enabled)
		return;

	spin_lock_irqsave(&priv->stats_lock, flags);
	priv->stats.pipeline_wait_events++;
	spin_unlock_irqrestore(&priv->stats_lock, flags);
}

#define FTDI_TUNE_INTERVAL_JIFFIES	msecs_to_jiffies(5000)

static void ftdi_spi_maybe_retune(struct ftdi_spi *priv)
{
	unsigned long now = jiffies;
	struct ftdi_spi_stats snapshot;
	unsigned long flags;
	bool need_retune = false;
	bool reset_stats = false;
	bool increase_burst = false;
	u32 copyfree_fallback_ratio = 0;
	u32 pipeline_util_pct = 0;
	bool consider_copyfree = false;
	bool consider_pipeline = false;

	if (!priv->stats_enabled)
		return;

	if (time_before(now, priv->tuning_jiffies + FTDI_TUNE_INTERVAL_JIFFIES))
		return;

	spin_lock_irqsave(&priv->stats_lock, flags);
	snapshot = priv->stats;
	spin_unlock_irqrestore(&priv->stats_lock, flags);

	priv->tuning_jiffies = now;
	priv->tuning_rounds++;

	/*
	 * If copy-free TX keeps falling back, disable it by forcing flushes
	 * to avoid repeated SG retries.
	 */
	if (snapshot.tx_copyfree_chunks)
		copyfree_fallback_ratio =
			DIV_ROUND_UP(snapshot.tx_copyfree_fallbacks * 100,
				     snapshot.tx_copyfree_chunks);
	if (snapshot.tx_copyfree_chunks >= 32)
		consider_copyfree = true;

	if (consider_copyfree && !priv->copyfree_disabled &&
	    copyfree_fallback_ratio > 12) {
		priv->copyfree_disabled = true;
		priv->flush_per_block = true;
		priv->flush_forced = true;
		need_retune = true;
		reset_stats = true;
	}

	/* If the pipeline never uses more than one URB, shrink depth. */
	if (snapshot.pipeline_wait_events) {
		u64 util = DIV_ROUND_UP(snapshot.pipeline_max_inflight * 100ULL,
				       max_t(unsigned int, priv->pipeline.depth, 1U));

		pipeline_util_pct = min_t(u32, 100U, (u32)util);
	}
	if (snapshot.pipeline_wait_events > 8)
		consider_pipeline = true;

	if (!priv->pipeline_forced && priv->pipeline.depth > 1 &&
	    consider_pipeline && pipeline_util_pct <= 25) {
		priv->pipeline.depth = 1;
		priv->pipeline_forced = 1;
		need_retune = true;
		reset_stats = true;
	}

	/* Grow burst size when transfers are healthy. */
	if (snapshot.tx_chunks_total >= 64 && snapshot.tx_timeout_errors == 0 &&
	    snapshot.tx_partial_chunks == 0 && priv->max_burst_bytes >= SZ_8K &&
	    priv->max_burst_bytes < SZ_64K) {
		size_t new_burst = priv->max_burst_bytes + SZ_4K;
		if (new_burst > SZ_64K)
			new_burst = SZ_64K;
		priv->max_burst_bytes = new_burst;
		need_retune = true;
		reset_stats = true;
		increase_burst = true;
	}

	if (snapshot.tx_timeout_errors > 4 || snapshot.tx_partial_chunks > 16) {
		size_t new_burst = max(priv->max_burst_bytes / 2, (size_t)SZ_4K);
		priv->max_burst_bytes = new_burst;
		need_retune = true;
		reset_stats = true;
	}

	/* Try re-enabling features after sustained stability. */
	if (priv->copyfree_disabled && snapshot.tx_copyfree_fallbacks == 0 &&
	    snapshot.tx_copyfree_chunks >= 128) {
		priv->copyfree_disabled = false;
		priv->flush_forced = false;
		priv->flush_per_block = false;
		need_retune = true;
		reset_stats = true;
	}

	if (priv->pipeline_forced && priv->pipeline.depth < FTDI_PIPELINE_MAX &&
	    snapshot.pipeline_wait_events == 0 &&
	    snapshot.pipeline_max_inflight >= priv->pipeline.depth) {
		priv->pipeline.depth =
			min_t(unsigned int, priv->pipeline.depth + 1,
			      FTDI_PIPELINE_MAX);
		priv->pipeline_forced = priv->pipeline.depth;
		need_retune = true;
		reset_stats = true;
	}

	if (need_retune)
		dev_info(&priv->pdev->dev,
			 "adaptive tuning applied: flush=%d depth=%u burst=%zu%s\n",
			 priv->flush_per_block, priv->pipeline.depth,
			 priv->max_burst_bytes,
			 increase_burst ? " (increased)" : "");

	if (reset_stats) {
		spin_lock_irqsave(&priv->stats_lock, flags);
		priv->stats.tx_copyfree_fallbacks = 0;
		priv->stats.tx_copyfree_chunks = 0;
		priv->stats.tx_copyfree_bytes = 0;
		priv->stats.pipeline_max_inflight = 0;
		priv->stats.pipeline_wait_events = 0;
		priv->stats.tx_chunks_total = 0;
		priv->stats.tx_partial_chunks = 0;
		priv->stats.tx_timeout_errors = 0;
		priv->stats.bytes_since_tune = 0;
		priv->stats.latency_min_ns = 0;
		priv->stats.latency_max_ns = 0;
		spin_unlock_irqrestore(&priv->stats_lock, flags);
	}
}

static void ftdi_pipeline_read_complete(struct urb *urb)
{
	struct ftdi_urb_ctx *ctx = urb->context;

	ctx->status = urb->status;
	ctx->actual = urb->actual_length;
	complete(&ctx->done);
}

static size_t ftdi_extract_payload(struct ft232h_intf_priv *priv,
				       u8 *dst, const u8 *src,
				       size_t act_len, size_t max_copy)
{
	size_t maxp = priv->bulk_in_pkt_sz;
	size_t offset = 0, out_len = 0;

	if (!maxp) {
		maxp = usb_maxpacket(priv->udev,
				     usb_rcvbulkpipe(priv->udev, priv->bulk_in));
		if (!maxp)
			maxp = SZ_512;
	}

	while (offset < act_len && out_len < max_copy) {
		size_t pkt_len = min(maxp, act_len - offset);
		size_t data_len;

		if (pkt_len <= 2) {
			offset += pkt_len;
			continue;
		}

		data_len = min(max_copy - out_len, pkt_len - 2);
		memcpy(dst + out_len, src + offset + 2, data_len);
		out_len += data_len;
		offset += pkt_len;
	}

	return out_len;
}

static void ftdi_pipeline_reset(struct ftdi_pipeline *pipe)
{
	pipe->head = 0;
	pipe->tail = 0;
	pipe->in_flight = 0;
}

static void ftdi_pipeline_kill(struct ftdi_pipeline *pipe)
{
	unsigned int i;

	if (!pipe->ctxs)
		return;

	for (i = 0; i < pipe->depth; i++)
		if (pipe->ctxs[i].urb)
			usb_kill_urb(pipe->ctxs[i].urb);
}

static int ftdi_spi_pipeline_setup(struct ftdi_spi *priv, unsigned int depth)
{
	struct ftdi_pipeline *pipe = &priv->pipeline;
	struct ft232h_intf_priv *priv_intf = usb_get_intfdata(priv->intf);
	struct device *dev = &priv->pdev->dev;
	unsigned int i;
	int ret = 0;

	memset(pipe, 0, sizeof(*pipe));
	depth = clamp_t(unsigned int, depth, 1, FTDI_PIPELINE_MAX);
	pipe->depth = depth;
	if (depth <= 1)
		return 0;

	pipe->ctxs = devm_kcalloc(dev, depth, sizeof(*pipe->ctxs), GFP_KERNEL);
	if (!pipe->ctxs)
		return -ENOMEM;

	for (i = 0; i < depth; i++) {
		struct ftdi_urb_ctx *ctx = &pipe->ctxs[i];

		ctx->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!ctx->urb) {
			ret = -ENOMEM;
			break;
		}

		ctx->buf = kmalloc(priv_intf->bulk_in_sz, GFP_KERNEL);
		if (!ctx->buf) {
			ret = -ENOMEM;
			break;
		}

		init_completion(&ctx->done);
		ctx->urb->context = ctx;
	}

	if (ret) {
		for (; i > 0; i--) {
			struct ftdi_urb_ctx *ctx = &pipe->ctxs[i - 1];

			kfree(ctx->buf);
			ctx->buf = NULL;
			usb_free_urb(ctx->urb);
			ctx->urb = NULL;
		}
		pipe->ctxs = NULL;
		pipe->depth = 1;
	}

	ftdi_pipeline_reset(pipe);
	return ret;
}

static void ftdi_spi_pipeline_teardown(struct ftdi_spi *priv)
{
	struct ftdi_pipeline *pipe = &priv->pipeline;
	unsigned int i;

	ftdi_pipeline_kill(pipe);

	if (!pipe->ctxs)
		return;

	for (i = 0; i < pipe->depth; i++) {
		struct ftdi_urb_ctx *ctx = &pipe->ctxs[i];

		kfree(ctx->buf);
		ctx->buf = NULL;
		if (ctx->urb) {
			usb_free_urb(ctx->urb);
			ctx->urb = NULL;
		}
	}

	pipe->ctxs = NULL;
	pipe->depth = 1;
	pipe->head = pipe->tail = pipe->in_flight = 0;
}

static int ftdi_pipeline_wait_one(struct ftdi_spi *priv,
				    struct ft232h_intf_priv *priv_intf)
{
	struct ftdi_pipeline *pipe = &priv->pipeline;
	struct device *dev = &priv->pdev->dev;
	struct ftdi_urb_ctx *ctx;
	long timeleft;
	int ret = 0;

	if (!pipe->in_flight)
		return 0;

	ctx = &pipe->ctxs[pipe->tail];
	timeleft = wait_for_completion_timeout(&ctx->done,
						   FTDI_READ_TIMEOUT_JIFFIES);
	if (!timeleft) {
		usb_kill_urb(ctx->urb);
		wait_for_completion(&ctx->done);
		ctx->status = -ETIMEDOUT;
	}

	pipe->tail = (pipe->tail + 1) % pipe->depth;
	pipe->in_flight--;
	ftdi_spi_stats_pipeline_wait(priv);

	if (ctx->status) {
		ret = ctx->status;
		goto out;
	}

	if (ftdi_extract_payload(priv_intf, ctx->dst, ctx->buf,
				    ctx->actual, ctx->expected) < ctx->expected) {
		dev_err(dev, "short read: expected %zu, got %zu\n",
			ctx->expected, ctx->actual);
		ret = -EIO;
	}

out:
	reinit_completion(&ctx->done);
	return ret;
}

static int ftdi_pipeline_wait_all(struct ftdi_spi *priv,
				     struct ft232h_intf_priv *priv_intf)
{
	int ret = 0;

	while (!ret && priv->pipeline.in_flight)
		ret = ftdi_pipeline_wait_one(priv, priv_intf);

	return ret;
}

static void ftdi_pipeline_abort(struct ftdi_spi *priv,
			      struct ft232h_intf_priv *priv_intf)
{
	ftdi_pipeline_kill(&priv->pipeline);
	ftdi_pipeline_wait_all(priv, priv_intf);
	ftdi_pipeline_reset(&priv->pipeline);
}

static int ftdi_pipeline_submit_read(struct ftdi_spi *priv,
				 struct ft232h_intf_priv *priv_intf,
				 u8 *dst, size_t expected)
{
	struct ftdi_pipeline *pipe = &priv->pipeline;
	struct ftdi_urb_ctx *ctx = &pipe->ctxs[pipe->head];
	struct usb_device *udev = priv_intf->udev;
	int ret;

	reinit_completion(&ctx->done);
	ctx->dst = dst;
	ctx->expected = expected;
	ctx->actual = 0;
	ctx->status = 0;

	usb_fill_bulk_urb(ctx->urb, udev,
			usb_rcvbulkpipe(udev, priv_intf->bulk_in),
			ctx->buf, priv_intf->bulk_in_sz,
			ftdi_pipeline_read_complete, ctx);
	ctx->urb->transfer_flags = 0;

	ret = usb_submit_urb(ctx->urb, GFP_KERNEL);
	if (ret)
		return ret;

	pipe->head = (pipe->head + 1) % pipe->depth;
	pipe->in_flight++;
	ftdi_spi_stats_pipeline_submit(priv, pipe->in_flight);
	return 0;
}
#if IS_ENABLED(CONFIG_DEBUG_FS)

static int ftdi_stats_show(struct seq_file *s, void *unused)
{
	struct ftdi_spi *priv = s->private;
	struct ftdi_spi_stats snapshot;
	unsigned long flags;
	u64 avg = 0;

	if (!priv->stats_enabled) {
		seq_puts(s, "{}\n");
		return 0;
	}

	spin_lock_irqsave(&priv->stats_lock, flags);
	snapshot = priv->stats;
	spin_unlock_irqrestore(&priv->stats_lock, flags);

	if (snapshot.total_transfers)
		avg = div64_u64(snapshot.total_latency_ns,
				 snapshot.total_transfers);

	seq_printf(s,
		   "{ \"tx_bytes\": %llu, \"rx_bytes\": %llu, \"transfers\": %llu, "
		   "\"full_duplex\": %llu, \"tx_only\": %llu, \"rx_only\": %llu, "
		   "\"read_timeouts\": %llu, \"read_empty_loops\": %llu, "
		   "\"transfer_errors\": %llu, \"max_latency_ns\": %llu, "
		   "\"avg_latency_ns\": %llu, \"min_latency_ns\": %llu, "
		   "\"tx_copyfree_bytes\": %llu, "
		   "\"tx_copyfree_chunks\": %llu, \"tx_copyfree_fallbacks\": %llu, "
		   "\"pipeline_max_inflight\": %llu, \"pipeline_wait_events\": %llu, "
		   "\"tx_chunks_total\": %llu, \"tx_partial_chunks\": %llu, "
		   "\"tx_timeout_errors\": %llu, \"bytes_since_tune\": %llu }\n",
		   snapshot.tx_bytes, snapshot.rx_bytes, snapshot.total_transfers,
		   snapshot.xfers_full_duplex, snapshot.xfers_tx_only,
		   snapshot.xfers_rx_only, snapshot.read_timeouts,
		   snapshot.read_empty_loops, snapshot.transfer_errors,
		   snapshot.max_latency_ns, avg,
		   snapshot.latency_min_ns ? snapshot.latency_min_ns : 0,
		   snapshot.tx_copyfree_bytes, snapshot.tx_copyfree_chunks,
		   snapshot.tx_copyfree_fallbacks, snapshot.pipeline_max_inflight,
		   snapshot.pipeline_wait_events, snapshot.tx_chunks_total,
		   snapshot.tx_partial_chunks, snapshot.tx_timeout_errors,
		   snapshot.bytes_since_tune);

	return 0;
}

static int ftdi_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, ftdi_stats_show, inode->i_private);
}

static const struct file_operations ftdi_stats_fops = {
	.owner		= THIS_MODULE,
	.open		= ftdi_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int ftdi_stats_reset_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t ftdi_stats_reset_write(struct file *file,
					 const char __user *buf,
					 size_t len, loff_t *ppos)
{
	struct ftdi_spi *priv = file->private_data;
	unsigned long flags;

	if (!priv->stats_enabled)
		return len;

	spin_lock_irqsave(&priv->stats_lock, flags);
	memset(&priv->stats, 0, sizeof(priv->stats));
	spin_unlock_irqrestore(&priv->stats_lock, flags);

	return len;
}

static const struct file_operations ftdi_stats_reset_fops = {
	.owner	= THIS_MODULE,
	.open	= ftdi_stats_reset_open,
	.write	= ftdi_stats_reset_write,
	.llseek	= no_llseek,
};

#endif /* CONFIG_DEBUG_FS */

static int ftdi_spi_txrx_send_chunk(struct ftdi_spi *priv,
				       struct ft232h_intf_priv *priv_intf,
				       const u8 *tx_src, size_t stride,
				       bool last, bool allow_copyfree,
				       bool *copyfree_used,
				       bool *copyfree_fallback,
				       bool *chunk_timeout)
{
	struct usb_device *udev = priv_intf->udev;
	bool used = false;
	bool fallback = false;
	int ret;
	size_t send_len;

	if (copyfree_used)
		*copyfree_used = false;
	if (copyfree_fallback)
		*copyfree_fallback = false;
	if (chunk_timeout)
		*chunk_timeout = false;

	if (allow_copyfree && stride >= SZ_1K) {
		struct scatterlist sg[3];
		struct usb_sg_request io;
		u8 hdr[3];
		u8 flush = SEND_IMMEDIATE;
		int nents = 2;
		size_t sg_len;

		hdr[0] = priv->txrx_cmd;
		hdr[1] = (stride - 1) & 0xff;
		hdr[2] = ((stride - 1) >> 8) & 0xff;

		sg_init_table(sg, ARRAY_SIZE(sg));
		sg_set_buf(&sg[0], hdr, sizeof(hdr));
		sg_set_buf(&sg[1], tx_src, stride);

		if (priv->flush_per_block || last) {
			sg_set_buf(&sg[2], &flush, sizeof(flush));
			nents = 3;
		}

		sg_len = sizeof(hdr) + stride + (nents == 3 ? sizeof(flush) : 0);

		ret = usb_sg_init(&io, udev,
				 usb_sndbulkpipe(udev, priv_intf->bulk_out),
				 0, sg, nents, sg_len,
				 GFP_KERNEL);
		used = true;
		if (ret) {
			fallback = true;
			goto copy_path;
		}

		usb_sg_wait(&io);
		ret = io.status;
		if (ret == -EPIPE || ret == -EAGAIN) {
			fallback = true;
			goto copy_path;
		}
		if (ret) {
			if (chunk_timeout && ret == -ETIMEDOUT)
				*chunk_timeout = true;
			return ret;
		}
		goto success;
	}

copy_path:

	priv->xfer_buf[0] = priv->txrx_cmd;
	priv->xfer_buf[1] = (stride - 1) & 0xff;
	priv->xfer_buf[2] = ((stride - 1) >> 8) & 0xff;
	memcpy(&priv->xfer_buf[3], tx_src, stride);
	send_len = stride + 3;
	if (priv->flush_per_block || last)
		priv->xfer_buf[send_len++] = SEND_IMMEDIATE;

	print_hex_dump_debug("WR: ", DUMP_PREFIX_OFFSET, 16, 1,
			     priv->xfer_buf, send_len, 1);

	ret = ftdi_spi_push_buf(priv, priv->xfer_buf, send_len);
	if (ret < 0) {
		if (chunk_timeout && ret == -ETIMEDOUT)
			*chunk_timeout = true;
		return ret;
	}

success:
	if (copyfree_used)
		*copyfree_used = used;
	if (copyfree_fallback)
		*copyfree_fallback = fallback;
	return 0;
}

static int ftdi_spi_tx_rx_legacy(struct ftdi_spi *priv, struct spi_device *spi,
			  struct spi_transfer *t)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct ft232h_intf_priv *priv_intf = usb_get_intfdata(priv->intf);
	struct device *dev = &priv->pdev->dev;
	void *rx_offs;
	const void *tx_offs;
	size_t remaining, stride;
	size_t rx_remaining;
	int ret;
	unsigned int polls;
	/* Capture how often we had to poll the USB FIFO when batching. */
	unsigned int empty_loops = 0;
	unsigned int timeouts = 0;
	bool loop_enabled = false;
	const u8 *tx_data = t->tx_buf;
	u8 *rx_data = t->rx_buf;
	bool copyfree_enabled;

	if (!priv_intf)
		return -ENODEV;

	ops->lock(priv->intf);

	if (spi->mode & SPI_LOOP) {
		ret = ftdi_spi_loopback_cfg(priv, 1);
		if (ret < 0)
			goto out;
		loop_enabled = true;
	}

	remaining = t->len;
	rx_offs = rx_data;
	tx_offs = tx_data;
	copyfree_enabled = !priv->copyfree_disabled &&
		(!t->bits_per_word || t->bits_per_word == 8) &&
		priv->max_burst_bytes >= SZ_4K;

	while (remaining) {
		bool last;
		bool copyfree_used = false;
		bool copyfree_fallback = false;
		bool chunk_timeout = false;

		stride = min_t(size_t, remaining, priv->max_burst_bytes);
		stride = min_t(size_t, stride, sizeof(priv->xfer_buf) - 4);
		if (!stride) {
			ret = -EINVAL;
			goto out;
		}

		last = (stride == remaining);

		ret = ftdi_spi_txrx_send_chunk(priv, priv_intf, tx_offs, stride,
					 last, copyfree_enabled,
					 &copyfree_used, &copyfree_fallback,
					 &chunk_timeout);
		if (ret < 0) {
			dev_err(dev, "%s: xfer failed %d\n", __func__, ret);
			goto out;
		}

		if (copyfree_used)
			ftdi_spi_stats_copy_free(priv, stride, !copyfree_fallback);
		ftdi_spi_stats_tx_chunk(priv, stride, copyfree_fallback,
				 chunk_timeout);
		if (copyfree_fallback)
			copyfree_enabled = false;

		dev_dbg(dev, "%s: WR %zu byte(s), TXRX CMD 0x%02x\n",
			__func__, stride, priv->txrx_cmd);

		rx_remaining = stride;
		polls = priv->rx_max_loops;
		do {
			size_t rx_stride = min_t(size_t, rx_remaining,
						priv_intf->bulk_in_sz);
			ret = ops->read_data(priv->intf, rx_offs, rx_stride);
			if (ret < 0)
				goto out;
			if (!ret) {
				empty_loops++;
				if (--polls) {
					if (priv->rx_retry_delay_us)
						usleep_range(priv->rx_retry_delay_us,
							     priv->rx_retry_delay_us + 50);
					continue;
				}
				timeouts++;
				dev_err(dev, "Read timeout\n");
				ret = -ETIMEDOUT;
				goto out;
			}

			print_hex_dump_debug("RD: ", DUMP_PREFIX_OFFSET, 16, 1,
				     rx_offs, ret, 1);
			rx_offs += ret;
			rx_remaining -= ret;
			polls = priv->rx_max_loops;
		} while (rx_remaining);

		remaining -= stride;
		tx_offs += stride;
		dev_dbg(dev, "%s: WR remains %zu\n", __func__, remaining);
	}

	ret = 0;

out:
	if (loop_enabled)
		ftdi_spi_loopback_cfg(priv, 0);
	ops->unlock(priv->intf);
	ftdi_spi_stats_account_rx(priv, empty_loops, timeouts);
	return ret;
}

static int ftdi_spi_tx_rx_pipeline(struct ftdi_spi *priv, struct spi_device *spi,
				    struct spi_transfer *t)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct ft232h_intf_priv *priv_intf = usb_get_intfdata(priv->intf);
	struct device *dev = &priv->pdev->dev;
	const u8 *tx_offs = t->tx_buf;
	u8 *rx_offs = t->rx_buf;
	size_t remaining;
	int ret = 0;
	bool loop_enabled = false;
	bool copyfree_enabled;

	if (!priv_intf)
		return -ENODEV;

	ftdi_pipeline_reset(&priv->pipeline);

	ops->lock(priv->intf);

	if (spi->mode & SPI_LOOP) {
		ret = ftdi_spi_loopback_cfg(priv, 1);
		if (ret < 0)
			goto out;
		loop_enabled = true;
	}

	remaining = t->len;
	copyfree_enabled = !priv->copyfree_disabled &&
		(!t->bits_per_word || t->bits_per_word == 8) &&
		priv->max_burst_bytes >= SZ_4K;

	while (remaining) {
		size_t stride = min_t(size_t, remaining, priv->max_burst_bytes);
		bool last;
		bool copyfree_used = false;
		bool copyfree_fallback = false;
		bool chunk_timeout = false;

		stride = min_t(size_t, stride, sizeof(priv->xfer_buf) - 4);
		if (priv_intf->bulk_in_sz > 2)
			stride = min_t(size_t, stride, priv_intf->bulk_in_sz - 2);
		if (!stride) {
			ret = -EINVAL;
			goto out;
		}

		while (priv->pipeline.in_flight >= priv->pipeline.depth) {
			ret = ftdi_pipeline_wait_one(priv, priv_intf);
			if (ret)
				goto out;
		}

		ret = ftdi_pipeline_submit_read(priv, priv_intf, rx_offs, stride);
		if (ret)
			goto out;

		last = (stride == remaining);

		ret = ftdi_spi_txrx_send_chunk(priv, priv_intf, tx_offs, stride,
					 last, copyfree_enabled,
					 &copyfree_used, &copyfree_fallback,
					 &chunk_timeout);
		if (ret < 0) {
			dev_err(dev, "%s: xfer failed %d\n", __func__, ret);
			goto out;
		}

		if (copyfree_used)
			ftdi_spi_stats_copy_free(priv, stride, !copyfree_fallback);
		ftdi_spi_stats_tx_chunk(priv, stride, copyfree_fallback,
				 chunk_timeout);
		if (copyfree_fallback)
			copyfree_enabled = false;

		tx_offs += stride;
		rx_offs += stride;
		remaining -= stride;
	}

	ret = ftdi_pipeline_wait_all(priv, priv_intf);

out:
	if (loop_enabled)
		ftdi_spi_loopback_cfg(priv, 0);
	ops->unlock(priv->intf);

	if (ret)
		ftdi_pipeline_abort(priv, priv_intf);
	else
		ftdi_pipeline_reset(&priv->pipeline);

	ftdi_spi_stats_account_rx(priv, 0, ret == -ETIMEDOUT ? 1 : 0);
	return ret;
}

static int ftdi_spi_tx_rx(struct ftdi_spi *priv, struct spi_device *spi,
			  struct spi_transfer *t)
{
	struct ft232h_intf_priv *priv_intf = usb_get_intfdata(priv->intf);
	bool use_pipeline = false;

	if (priv->pipeline.depth > 1 && priv->pipeline.ctxs && priv_intf) {
		size_t maxp = priv_intf->bulk_in_pkt_sz;

		if (!maxp && priv_intf->udev)
			maxp = usb_maxpacket(priv_intf->udev,
					 usb_rcvbulkpipe(priv_intf->udev,
					 priv_intf->bulk_in));
		if (!maxp)
			maxp = SZ_512;

		/*
		 * Pipeline depth only helps on sizeable transfers.  Smaller control
		 * messages complete more reliably through the legacy synchronous path.
		 */
		if (t->len >= max_t(size_t, maxp * 2, SZ_1K))
			use_pipeline = true;
	}

	if (use_pipeline)
		return ftdi_spi_tx_rx_pipeline(priv, spi, t);

	return ftdi_spi_tx_rx_legacy(priv, spi, t);
}

static int ftdi_spi_push_buf(struct ftdi_spi *priv, const void *buf, size_t len)
{
	size_t bytesleft = len;
	int ret;

	do {
		ret = priv->iops->write_data(priv->intf, buf, bytesleft);
		if (ret < 0)
			return ret;

		buf += ret;
		bytesleft -= ret;
	} while (bytesleft);

	return len;
}

static int ftdi_spi_tx(struct ftdi_spi *priv, struct spi_transfer *xfer)
{
	const void *tx_offs;
	size_t remaining, stride;
	int ret;
	bool copy_free = false;
	bool last;
	const size_t hdr_len = 3;
	struct ft232h_intf_priv *priv_intf = usb_get_intfdata(priv->intf);
	struct usb_device *udev;

	if (!priv_intf)
		return -ENODEV;

	udev = priv_intf->udev;

	/* Copy-free path only handles 8-bit words with large payloads. */
	if (!priv->copyfree_disabled &&
	    (!xfer->bits_per_word || xfer->bits_per_word == 8) &&
	    priv->max_burst_bytes >= SZ_4K)
		copy_free = true;

	priv->iops->lock(priv->intf);

	tx_offs = xfer->tx_buf;
	remaining = xfer->len;

	do {
		bool copy_free_used = false;
		bool copy_free_fallback = false;
		/* Respect both FTDI command limits and caller supplied burst size. */
		stride = min_t(size_t, remaining, priv->max_burst_bytes);
		stride = min_t(size_t, stride, sizeof(priv->xfer_buf) - 3);
		if (!stride) {
			ret = -EINVAL;
			goto err;
		}
		last = (stride == remaining);

		priv->xfer_buf[0] = priv->tx_cmd;
		priv->xfer_buf[1] = stride - 1;
		priv->xfer_buf[2] = (stride - 1) >> 8;

		if (copy_free && stride >= SZ_1K) {
			struct scatterlist sg[3];
			struct usb_sg_request io;
			u8 hdr[3];
			u8 flush = SEND_IMMEDIATE;
			int nents = 2;
			size_t sg_len;

			copy_free_used = true;

			hdr[0] = priv->xfer_buf[0];
			hdr[1] = priv->xfer_buf[1];
			hdr[2] = priv->xfer_buf[2];

			sg_init_table(sg, ARRAY_SIZE(sg));
			sg_set_buf(&sg[0], hdr, hdr_len);
			sg_set_buf(&sg[1], tx_offs, stride);

			if (priv->flush_per_block || last) {
				sg_set_buf(&sg[2], &flush, 1);
				nents = 3;
			}

			sg_len = hdr_len + stride + (nents == 3 ? 1 : 0);

			ret = usb_sg_init(&io, udev,
					 usb_sndbulkpipe(udev, priv_intf->bulk_out),
					 0, sg, nents, sg_len,
					 GFP_KERNEL);
			if (ret) {
				copy_free = false;
				copy_free_fallback = true;
				goto copy_path;
			}

			usb_sg_wait(&io);
			/* usb_sg_wait() drains and frees the request on completion. */
		ret = io.status;
		if (ret == -EPIPE || ret == -EAGAIN) {
			/*
			 * The host may NAK a long burst; fall back to the
			 * memcpy path for this and subsequent chunks.
			 */
			copy_free = false;
			copy_free_fallback = true;
			goto copy_path;
		}
		if (ret)
			goto err;
		} else {
copy_path:
			memcpy(&priv->xfer_buf[3], tx_offs, stride);

			ret = ftdi_spi_push_buf(priv, priv->xfer_buf, stride + 3);
			if (ret < 0) {
				dev_dbg(&priv->pdev->dev, "%s: tx failed %d\n",
					__func__, ret);
				goto err;
			}
		}

		if (copy_free_used)
			ftdi_spi_stats_copy_free(priv, stride, !copy_free_fallback);
		ftdi_spi_stats_tx_chunk(priv, stride, copy_free_fallback,
				ret == -ETIMEDOUT);
		dev_dbg(&priv->pdev->dev, "%s: %zu byte(s) done\n",
			__func__, stride);
		remaining -= stride;
		tx_offs += stride;
	} while (remaining);

	ret = 0;
err:
	priv->iops->unlock(priv->intf);
	return ret;
}

static int ftdi_spi_rx(struct ftdi_spi *priv, struct spi_transfer *xfer)
{
	const struct ft232h_intf_ops *ops = priv->iops;
	struct device *dev = &priv->pdev->dev;
	struct ft232h_intf_priv *priv_intf = usb_get_intfdata(priv->intf);
	size_t remaining;
	size_t rx_remaining;
	int ret;
	void *rx_offs;
	unsigned int polls;
	/* Keep per-transfer poll statistics for debugfs counters. */
	unsigned int empty_loops = 0;
	unsigned int timeouts = 0;

	dev_dbg(dev, "%s: CMD 0x%02x, len %u\n",
		__func__, priv->rx_cmd, xfer->len);

	if (!priv_intf)
		return -ENODEV;

	ops->lock(priv->intf);

	remaining = xfer->len;
	rx_offs = xfer->rx_buf;

	while (remaining) {
		/* RX path shares the same burst sizing constraints as TX. */
		size_t stride = min_t(size_t, remaining, priv->max_burst_bytes);
		size_t cmd_len = 3;
		bool last;

		stride = min_t(size_t, stride, sizeof(priv->xfer_buf) - 4);
		if (!stride) {
			ret = -EINVAL;
			goto out;
		}

		last = (stride == remaining);

		priv->xfer_buf[0] = priv->rx_cmd;
		priv->xfer_buf[1] = (stride - 1) & 0xff;
		priv->xfer_buf[2] = ((stride - 1) >> 8) & 0xff;
		if (priv->flush_per_block || last)
			priv->xfer_buf[cmd_len++] = SEND_IMMEDIATE;

		print_hex_dump_debug("WR-RX: ", DUMP_PREFIX_OFFSET, 16, 1,
				     priv->xfer_buf, cmd_len, 1);

		ret = ops->write_data(priv->intf, priv->xfer_buf, cmd_len);
		if (ret < 0)
			goto out;

		rx_remaining = stride;
		polls = priv->rx_max_loops;

		do {
			size_t rx_stride = min_t(size_t, rx_remaining,
						priv_intf->bulk_in_sz);
			ret = ops->read_data(priv->intf, rx_offs, rx_stride);
			if (ret < 0)
				goto out;

			if (!ret) {
				empty_loops++;
				if (--polls) {
					if (priv->rx_retry_delay_us)
						usleep_range(priv->rx_retry_delay_us,
							     priv->rx_retry_delay_us + 50);
					continue;
				}
				timeouts++;
				dev_dbg(dev, "read timeout...\n");
				ret = -ETIMEDOUT;
				goto out;
			}

			print_hex_dump_debug("RD: ", DUMP_PREFIX_OFFSET, 16, 1,
				     rx_offs, ret, 1);
			rx_offs += ret;
			rx_remaining -= ret;
			polls = priv->rx_max_loops;
		} while (rx_remaining);

		remaining -= stride;
		dev_dbg(dev, "%s: chunk %zu done, remaining %zu\n",
			__func__, stride, remaining);
	}

	ret = 0;
out:
	ops->unlock(priv->intf);
	ftdi_spi_stats_account_rx(priv, empty_loops, timeouts);
	return ret;
}

static int ftdi_spi_transfer_one(struct spi_controller *ctlr,
				 struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	struct ftdi_spi *priv = spi_controller_get_devdata(ctlr);
	struct device *dev = &priv->pdev->dev;
	int ret = 0;
	ktime_t start = 0;

	if (!xfer->len)
		return 0;

	if (priv->last_speed_hz != xfer->speed_hz) {
		dev_dbg(dev, "%s: new speed %u\n", __func__, (int)xfer->speed_hz);
		ret = priv->iops->set_clock(priv->intf, xfer->speed_hz);
		if (ret < 0) {
			dev_err(dev, "Set clock(%u) failed: %d\n", xfer->speed_hz, ret);
			return ret;
		}
		priv->last_speed_hz = xfer->speed_hz;
	}

	if (priv->last_mode != spi->mode) {
		u8 spi_mode = spi->mode & (SPI_CPOL | SPI_CPHA);
		u8 pins = 0;

		dev_dbg(dev, "%s: MODE 0x%x\n", __func__, spi->mode);

		if (spi->mode & SPI_LSB_FIRST) {
			switch (spi_mode) {
			case SPI_MODE_0:
			case SPI_MODE_3:
				priv->tx_cmd = TX_BYTES_FE_LSB;
				priv->rx_cmd = RX_BYTES_RE_LSB;
				break;
			case SPI_MODE_1:
			case SPI_MODE_2:
				priv->tx_cmd = TX_BYTES_RE_LSB;
				priv->rx_cmd = RX_BYTES_FE_LSB;
				break;
			}
		} else {
			switch (spi_mode) {
			case SPI_MODE_0:
			case SPI_MODE_3:
				priv->tx_cmd = TX_BYTES_FE_MSB;
				priv->rx_cmd = RX_BYTES_RE_MSB;
				break;
			case SPI_MODE_1:
			case SPI_MODE_2:
				priv->tx_cmd = TX_BYTES_RE_MSB;
				priv->rx_cmd = RX_BYTES_FE_MSB;
				break;
			}
		}

		priv->txrx_cmd = ftdi_spi_txrx_byte_cmd(spi);

		switch (spi_mode) {
		case SPI_MODE_2:
		case SPI_MODE_3:
			pins |= MPSSE_SK;
			break;
		}

		ret = priv->iops->cfg_bus_pins(priv->intf,
					       MPSSE_DIR_OUTPUT, pins);
		if (ret < 0) {
			dev_err(dev, "IO cfg failed: %d\n", ret);
			return ret;
		}
		priv->last_mode = spi->mode;
	}

	dev_dbg(dev, "%s: mode 0x%x, CMD RX/TX 0x%x/0x%x\n",
		__func__, spi->mode, priv->rx_cmd, priv->tx_cmd);

	if (priv->stats_enabled)
		/* Timestamps are only captured when debug stats are in use. */
		start = ktime_get();

	if (xfer->tx_buf && xfer->rx_buf)
		ret = ftdi_spi_tx_rx(priv, spi, xfer);
	else if (xfer->tx_buf)
		ret = ftdi_spi_tx(priv, xfer);
	else if (xfer->rx_buf)
		ret = ftdi_spi_rx(priv, xfer);

	dev_dbg(dev, "%s: xfer ret %d\n", __func__, ret);

	if (priv->stats_enabled) {
		u64 duration = ktime_to_ns(ktime_sub(ktime_get(), start));
		ftdi_spi_stats_complete(priv, xfer, ret, duration);
	}

	ftdi_spi_maybe_retune(priv);

	spi_finalize_current_transfer(ctlr);
	return ret;
}

static int ftdi_mpsse_init(struct ftdi_spi *priv)
{
	struct platform_device *pdev = priv->pdev;
	int ret;

	dev_dbg(&pdev->dev, "MPSSE init\n");

	/* Setup and send off the Hi-Speed specific commands for the FTx232H */
	priv->xfer_buf[0] = DIS_DIV_5;      /* Use 60MHz master clock */
	priv->xfer_buf[1] = DIS_ADAPTIVE;   /* Turn off adaptive clocking */
	priv->xfer_buf[2] = DIS_3_PHASE;    /* Disable three-phase clocking */

	priv->iops->lock(priv->intf);

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clk cfg failed: %d\n", ret);
		priv->iops->unlock(priv->intf);
		return ret;
	}

	priv->xfer_buf[0] = TCK_DIVISOR;
	priv->xfer_buf[1] = div_value(60000000);
	priv->xfer_buf[2] = div_value(60000000) >> 8;
	dev_dbg(&pdev->dev, "TCK_DIVISOR: 0x%04x 0x%04x\n",
		priv->xfer_buf[1], priv->xfer_buf[2]);

	ret = priv->iops->write_data(priv->intf, priv->xfer_buf, 3);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clk cfg failed: %d\n", ret);
		priv->iops->unlock(priv->intf);
		return ret;
	}

	priv->iops->unlock(priv->intf);

	ret = priv->iops->cfg_bus_pins(priv->intf, MPSSE_DIR_OUTPUT, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "Can't init SPI bus pins: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ftdi_mpsse_gpio_probe(struct usb_interface *intf);
static int ftdi_mpsse_irq_probe(struct usb_interface *intf);

static int ftdi_spi_probe(struct platform_device *pdev)
{
	const struct mpsse_spi_platform_data *pd;
	struct device *dev = &pdev->dev;
	struct spi_controller *master;
	struct ftdi_spi *priv;
	int ret;

	pd = dev->platform_data;
	if (!pd) {
		dev_err(dev, "Missing platform data.\n");
		return -EINVAL;
	}

	if (!pd->ops ||
	    !pd->ops->read_data || !pd->ops->write_data ||
	    !pd->ops->lock || !pd->ops->unlock ||
	    !pd->ops->set_bitmode ||
	    !pd->ops->cfg_bus_pins ||
	    !pd->ops->set_clock ||
	    !pd->ops->set_latency ||
	    !pd->ops->gpio_get ||
	    !pd->ops->gpio_set ||
	    !pd->ops->gpio_direction_input ||
	    !pd->ops->gpio_direction_output)
		return -EINVAL;

#if defined(SPI_DEVICE_CS_CNT_MAX)
		master = spi_alloc_host(&pdev->dev, sizeof(*priv));
#else
		master = spi_alloc_master(&pdev->dev, sizeof(*priv));
#endif
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	priv = spi_controller_get_devdata(master);
	priv->master = master;
	priv->pdev = pdev;
	priv->intf = to_usb_interface(dev->parent);
	priv->iops = pd->ops;

	{
		struct ft232h_intf_priv *intf_priv = usb_get_intfdata(priv->intf);
		size_t burst_limit = sizeof(priv->xfer_buf) - 4;
		size_t burst = min_t(size_t, SZ_512 - 3, burst_limit);

		if (param_perf_profile == PERF_PROFILE_BALANCED)
			burst = max_t(size_t, burst, SZ_4K);
		else if (param_perf_profile == PERF_PROFILE_AGGRESSIVE)
			burst = max_t(size_t, burst, SZ_32K);

		if (param_max_block)
			burst = param_max_block;

		burst = clamp_t(size_t, burst, 1, burst_limit);
		priv->max_burst_bytes = burst;
	priv->flush_per_block = param_flush_per_block;
	if (!param_flush_overridden) {
		switch (param_perf_profile) {
		case PERF_PROFILE_LEGACY:
		case PERF_PROFILE_BALANCED:
			priv->flush_per_block = true;
			break;
		case PERF_PROFILE_AGGRESSIVE:
		default:
			priv->flush_per_block = false;
			break;
		}
	}
		priv->rx_retry_delay_us = param_rx_retry_us;
		if (priv->rx_retry_delay_us) {
			unsigned int loops;

			/* Target roughly 2.5ms of total wait before timing out. */
			loops = (2500 + priv->rx_retry_delay_us - 1) /
				priv->rx_retry_delay_us;
			loops = clamp_t(unsigned int, loops, 2, 1000);
			priv->rx_max_loops = loops;
		} else {
			priv->rx_max_loops = 10;
		}

		if (intf_priv && intf_priv->bulk_in_pkt_sz) {
			/* Align bursts to whole USB packets when possible. */
			size_t max_align = intf_priv->bulk_in_pkt_sz;
			if (max_align && priv->max_burst_bytes > max_align) {
				size_t aligned = priv->max_burst_bytes -
					(priv->max_burst_bytes % max_align);
				if (!aligned)
					aligned = max_align;
				priv->max_burst_bytes = min_t(size_t, aligned,
							      burst_limit);
			}
		}

		dev_dbg(dev, "SPI burst=%zu flush=%d rx_retry_us=%u loops=%u\n",
			priv->max_burst_bytes, priv->flush_per_block,
			priv->rx_retry_delay_us, priv->rx_max_loops);
	}

	{
		unsigned int depth = param_pipeline_depth;

		if (!param_pipeline_overridden || !depth) {
			/* Auto-tune pipeline depth when the user leaves it unspecified. */
			switch (param_perf_profile) {
			case PERF_PROFILE_LEGACY:
				depth = 1;
				break;
			case PERF_PROFILE_BALANCED:
			depth = min_t(unsigned int, 2U, FTDI_PIPELINE_MAX);
				break;
			case PERF_PROFILE_AGGRESSIVE:
			default:
				depth = FTDI_PIPELINE_MAX;
				break;
			}
		}

	/* Configure the RX pipeliner; aggressive profile starts at max depth. */
	ret = ftdi_spi_pipeline_setup(priv, depth);
	}
	if (ret) {
		dev_warn(dev, "Failed to enable URB pipeline (%d), falling back to synchronous mode\n",
			 ret);
	}

	/* Metrics are on by default; users can still opt out via module param. */
	priv->stats_enabled = param_enable_stats;
	spin_lock_init(&priv->stats_lock);
	memset(&priv->stats, 0, sizeof(priv->stats));
	priv->tuning_jiffies = jiffies;
	priv->tuning_rounds = 0;
	priv->flush_forced = param_flush_overridden;
	priv->pipeline_forced = param_pipeline_overridden ? priv->pipeline.depth : 0;
	priv->copyfree_disabled = param_flush_overridden && param_flush_per_block;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (priv->stats_enabled) {
		const char *dbg_name = dev_name(&master->dev);
		char *alloc_name = NULL;
		if (!ftdi_debugfs_root)
			ftdi_debugfs_root = debugfs_create_dir("ftdi_spi", NULL);
		if (IS_ERR(ftdi_debugfs_root)) {
			dev_warn(dev, "debugfs root unavailable: %ld\n",
				 PTR_ERR(ftdi_debugfs_root));
			ftdi_debugfs_root = NULL;
		} else {
			if (!dbg_name || !dbg_name[0]) {
				alloc_name = devm_kasprintf(dev, GFP_KERNEL, "%s.%d",
						  SPI_INTF_DEVNAME, priv->pdev->id);
				if (!alloc_name) {
					dev_warn(dev, "debugfs name alloc failed\n");
					goto debugfs_done;
				}
				dbg_name = alloc_name;
			}

			priv->debugfs_root = debugfs_create_dir(dbg_name,
							ftdi_debugfs_root);
			if (IS_ERR(priv->debugfs_root)) {
				dev_warn(dev, "debugfs dir creation failed: %ld\n",
					 PTR_ERR(priv->debugfs_root));
				priv->debugfs_root = NULL;
			} else {
				debugfs_create_file("stats", 0444, priv->debugfs_root,
						priv, &ftdi_stats_fops);
				debugfs_create_file("stats_reset", 0200,
						priv->debugfs_root, priv,
						&ftdi_stats_reset_fops);
			}
		debugfs_done:
		}
	}
#endif

	master->bus_num = (param_bus_num >= 0) ? param_bus_num : -1;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP |
			    SPI_CS_HIGH | SPI_LSB_FIRST;
	master->num_chipselect = 1;
	master->min_speed_hz = 450;
	master->max_speed_hz = 30000000;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->set_cs = ftdi_spi_set_cs;
	master->transfer_one = ftdi_spi_transfer_one;
	master->auto_runtime_pm = false;

	ret = spi_register_controller(master);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register spi master\n");
		spi_controller_put(master);
		return ret;
	}
	
	dev_info(dev, "spi_master: bus_num=%d\n", master->bus_num);

	ret = priv->iops->set_bitmode(priv->intf, 0x00, BITMODE_MPSSE);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to set MPSSE mode\n");
		goto err;
	}

	priv->last_mode = 0xffff;

	ret = ftdi_mpsse_init(priv);
	if (ret < 0) {
		dev_err(&pdev->dev, "MPSSE init failed\n");
		goto err;
	}

	ret = priv->iops->set_latency(priv->intf, param_latency);
	if (ret < 0) {
		dev_err(&pdev->dev, "Set latency failed\n");
		goto err;
	}
	
	ret = ftdi_mpsse_gpio_probe(priv->intf);
	if (ret < 0)
		goto err;
	
	ret = ftdi_mpsse_irq_probe(priv->intf);
	if (ret < 0)
		goto err;

	return 0;
err:
	platform_set_drvdata(pdev, NULL);
	ftdi_spi_pipeline_teardown(priv);
	spi_unregister_controller(master);
	return ret;
}

static int ftdi_spi_slave_release(struct device *dev, void *data)
{
	spi_unregister_device(to_spi_device(dev));
	return 0;
}

static void ftdi_mpsse_irq_remove(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct gpio_chip *chip = &priv->mpsse_gpio;
	
	if (priv->irq_base >= 0)
		irq_free_descs(priv->irq_base, chip->ngpio);
}

static void ftdi_mpsse_gpio_remove(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	if (priv->lookup_gpios)
		gpiod_remove_lookup_table(priv->lookup_gpios);
}

static int ftdi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *master;
	struct ftdi_spi *priv;

	master = platform_get_drvdata(pdev);
	priv = spi_controller_get_devdata(master);

	device_for_each_child(&master->dev, priv, ftdi_spi_slave_release);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (priv->debugfs_root)
		debugfs_remove_recursive(priv->debugfs_root);
#endif

	ftdi_spi_pipeline_teardown(priv);

	spi_unregister_controller(master);
	return 0;
}

/*
 * ftdi_ctrl_xfer - FTDI control endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for control transfer
 *
 * Return:
 * Return: If successful, the number of bytes transferred. Otherwise,
 * a negative error number.
 */
static int ftdi_ctrl_xfer(struct usb_interface *intf, struct ctrl_desc *desc)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (!desc->data && desc->size)
		desc->data = priv->bulk_in_buf;

	if (desc->dir_out)
		pipe = usb_sndctrlpipe(udev, 0);
	else
		pipe = usb_rcvctrlpipe(udev, 0);

	ret = usb_control_msg(udev, pipe, desc->request, desc->requesttype,
			      desc->value, desc->index, desc->data, desc->size,
			      desc->timeout);
	if (ret < 0)
		dev_dbg(&udev->dev, "ctrl msg failed: %d\n", ret);
exit:
	mutex_unlock(&priv->io_mutex);
	return ret;
}

/*
 * ftdi_bulk_xfer - FTDI bulk endpoint transfer
 * @intf: USB interface pointer
 * @desc: pointer to descriptor struct for bulk-in or bulk-out transfer
 *
 * Return:
 * If successful, 0. Otherwise a negative error number. The number of
 * actual bytes transferred will be stored in the @desc->act_len field
 * of the descriptor struct.
 */
static int ftdi_bulk_xfer(struct usb_interface *intf, struct bulk_desc *desc)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct usb_device *udev = priv->udev;
	unsigned int pipe;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		ret = -ENODEV;
		goto exit;
	}

	if (desc->dir_out)
		pipe = usb_sndbulkpipe(udev, priv->bulk_out);
	else
		pipe = usb_rcvbulkpipe(udev, priv->bulk_in);

	ret = usb_bulk_msg(udev, pipe, desc->data, desc->len,
			   &desc->act_len, desc->timeout);
	if (ret)
		dev_dbg(&udev->dev, "bulk msg failed: %d\n", ret);

exit:
	mutex_unlock(&priv->io_mutex);
	if (usb_wait_msec > 0) {
		usleep_range(usb_wait_msec * 1000, usb_wait_msec * 1000 + 1000);
	}
	return ret;
}

/*
 * ftdi_set_clock - set the device clock (NOT UART baudrate)
 * @intf: USB interface pointer
 * @clock_freq_hz: clock value to set
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_clock(struct usb_interface *intf, int clock_freq_hz)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	uint8_t *buf = priv->tx_buf;
	uint32_t value = 0;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	switch (priv->usb_dev_id->idProduct) {
	case 0x6001: /* FT232 */
		if (clock_freq_hz >= FTDI_CLK_6MHZ) {
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		break;

	case 0x6010: /* FT2232 */
	case 0x6011: /* FT4232 */
	case 0x6014: /* FT232H */
	case 0x0146: /* GW16146 */
		desc.len = 1;
		if (clock_freq_hz <= (FTDI_CLK_30MHZ/65535)) {
			buf[0] = EN_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_6MHZ/clock_freq_hz) - 1;
		}
		else {
			buf[0] = DIS_DIV_5;
			ret = ftdi_bulk_xfer(intf, &desc);
			if (ret) {
				return ret;
			}
			value = (FTDI_CLK_30MHZ/clock_freq_hz) - 1;
		}

		break;
	}

	buf[0] = TCK_DIVISOR;
	buf[1] = (uint8_t)(value & 0xff);
	buf[2] = (uint8_t)(value >> 8);
	desc.act_len = 0;
	desc.len = 3;
	ret = ftdi_bulk_xfer(intf, &desc);

	return ret;
}

/*
 * ftdi_set_latency - set the device latency (Bulk-In interval)
 * @intf: USB interface pointer
 * @latency_msec: latency value to set, 1-255
 *
 * Return: If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_latency(struct usb_interface *intf, int latency_msec)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.request = FTDI_SIO_SET_LATENCY_TIMER_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.value = latency_msec;
	desc.index = priv->index;
	desc.data = NULL;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	if (ret < 0) {
		dev_dbg(&intf->dev, "failed to set latency: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * ftdi_read_data - read from FTDI bulk-in endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to read
 *
 * The two modem status bytes transferred in every read will
 * be removed and will not appear in the data buffer.
 *
 * Return:
 * If successful, the number of data bytes received (can be 0).
 * Otherwise, a negative error number.
 */
static int ftdi_read_data(struct usb_interface *intf, void *buf, size_t len)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = false;
	desc.data = priv->bulk_in_buf;
	/* Device sends 2 additional status bytes, read at least len + 2 */
	desc.len = min_t(size_t, len + 2, priv->bulk_in_sz);
	desc.timeout = FTDI_USB_READ_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret)
		return ret;

	/* Only status bytes and no data? */
	if (desc.act_len <= 2)
		return 0;

	/* Remove 2-byte status from each 512-byte packet */
	size_t maxp = priv->bulk_in_pkt_sz;
	if (!maxp)
		maxp = usb_maxpacket(priv->udev,
				 usb_rcvbulkpipe(priv->udev, priv->bulk_in));
	if (!maxp)
		maxp = SZ_512;
	size_t offset = 0;
	size_t out_len = 0;
	while (offset < desc.act_len) {
		size_t pkt_len = min(maxp, desc.act_len - offset);
		if (pkt_len <= 2) {
			offset += pkt_len;   // packet contains only status
			break;
		}
		/* Copy packet payload (skip status bytes) */
		size_t data_len = pkt_len - 2;
		if (out_len + data_len > len)
			data_len = len - out_len;
		memcpy(buf + out_len, desc.data + offset + 2, data_len);
		out_len += data_len;
		offset += pkt_len;
	}
	
	return out_len;
}

/*
 * ftdi_write_data - write to FTDI bulk-out endpoint
 * @intf: USB interface pointer
 * @buf:  pointer to data buffer
 * @len:  length in bytes of the data to send
 *
 * Return:
 * If successful, the number of bytes transferred. Otherwise a negative
 * error number.
 */
static int ftdi_write_data(struct usb_interface *intf,
			   const char *buf, size_t len)
{
	struct bulk_desc desc;
	int ret;

	desc.act_len = 0;
	desc.dir_out = true;
	desc.data = (char *)buf;
	desc.len = len;
	desc.timeout = FTDI_USB_WRITE_TIMEOUT;

	ret = ftdi_bulk_xfer(intf, &desc);
	if (ret < 0)
		return ret;

	return desc.act_len;
}

/*
 * ftdi_set_bitmode - configure bitbang mode
 * @intf: USB interface pointer
 * @bitmask: line configuration bitmask
 * @mode: bitbang mode to set
 *
 * Return:
 * If successful, 0. Otherwise a negative error number.
 */
static int ftdi_set_bitmode(struct usb_interface *intf, unsigned char bitmask,
			    unsigned char mode)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct ctrl_desc desc;
	int ret;

	desc.dir_out = true;
	desc.data = NULL;
	desc.request = FTDI_SIO_SET_BITMODE_REQUEST;
	desc.requesttype = USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT;
	desc.index = priv->index;
	desc.value = (mode << 8) | bitmask;
	desc.size = 0;
	desc.timeout = USB_CTRL_SET_TIMEOUT;

	ret = ftdi_ctrl_xfer(intf, &desc);
	return ret;
}

/*
 * MPSSE CS and GPIO-L/-H support
 */
#define SET_BITS_LOW	0x80
#define GET_BITS_LOW	0x81
#define SET_BITS_HIGH	0x82
#define GET_BITS_HIGH	0x83

static int ftdi_mpsse_get_port_pins(struct ft232h_intf_priv *priv, bool low)
{
	struct device *dev = &priv->intf->dev;
	int ret, tout = 10;
	u8 rxbuf[4];

	if (low)
		priv->tx_buf[0] = GET_BITS_LOW;
	else
		priv->tx_buf[0] = GET_BITS_HIGH;

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 1);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Writing port pins cmd failed: %d\n",
				    ret);
		return ret;
	}

	rxbuf[0] = 0;
	do {
		usleep_range(5000, 5200);
		ret = ftdi_read_data(priv->intf, rxbuf, 1);
		tout--;
		if (!tout) {
			dev_err(dev, "Timeout when getting port pins\n");
			return -ETIMEDOUT;
		}
	} while (ret == 0);

	if (ret < 0)
		return ret;

	if (ret != 1)
		return -EINVAL;

	if (low)
		priv->gpiol_mask = rxbuf[0];
	else
		priv->gpioh_mask = rxbuf[0];

	return 0;
}

static int ftdi_mpsse_set_port_pins(struct ft232h_intf_priv *priv, bool low)
{
	struct device *dev = &priv->intf->dev;
	int ret;

	if (low) {
		priv->tx_buf[0] = SET_BITS_LOW;
		priv->tx_buf[1] = priv->gpiol_mask;
		priv->tx_buf[2] = priv->gpiol_dir;
	} else {
		priv->tx_buf[0] = SET_BITS_HIGH;
		priv->tx_buf[1] = priv->gpioh_mask;
		priv->tx_buf[2] = priv->gpioh_dir;
	}

	ret = ftdi_write_data(priv->intf, priv->tx_buf, 3);
	if (ret < 0) {
		dev_dbg_ratelimited(dev, "Failed to set GPIO pins: %d\n",
				    ret);
		return ret;
	}

	return 0;
}

static int ftdi_mpsse_init_pins(struct usb_interface *intf, bool low,
				u8 bits, u8 direction)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	if (low) {
		priv->gpiol_mask = bits;
		priv->gpiol_dir = direction;
	} else {
		priv->gpioh_mask = bits;
		priv->gpioh_dir = direction;
	}
	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

#define MPSSE_GPIO_MASK 0x0F

static int ftdi_mpsse_cfg_bus_pins(struct usb_interface *intf,
				   u8 dir_bits, u8 value_bits)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	int ret;

	mutex_lock(&priv->ops_mutex);

	priv->gpiol_dir &= ~MPSSE_GPIO_MASK;
	priv->gpiol_dir |= (dir_bits & MPSSE_GPIO_MASK);

	priv->gpiol_mask &= ~MPSSE_GPIO_MASK;
	priv->gpiol_mask |= (value_bits & MPSSE_GPIO_MASK);

	ret = ftdi_mpsse_set_port_pins(priv, true);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

int ftdi_gpio_get(struct usb_interface *intf, unsigned int offset)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	int ret, val;
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d\n", __func__, offset);

	low = offset < 5;

	mutex_lock(&priv->ops_mutex);

	ret = ftdi_mpsse_get_port_pins(priv, low);
	if (ret < 0) {
		mutex_unlock(&priv->ops_mutex);
		return ret;
	}

	if (low)
		val = priv->gpiol_mask & (BIT(offset) << 3);
	else
		val = priv->gpioh_mask & BIT(offset - 5);

	mutex_unlock(&priv->ops_mutex);

	return !!val;
}

void ftdi_gpio_set(struct usb_interface *intf, unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		if (value)
			priv->gpiol_mask |= (BIT(offset) << 3);
		else
			priv->gpiol_mask &= ~(BIT(offset) << 3);
	} else {
		low = false;
		if (value)
			priv->gpioh_mask |= BIT(offset - 5);
		else
			priv->gpioh_mask &= ~BIT(offset - 5);
	}

	ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);
}

int ftdi_gpio_direction_input(struct usb_interface *intf, unsigned int offset)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d\n", __func__, offset);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		priv->gpiol_dir &= ~(BIT(offset) << 3);
	} else {
		low = false;
		priv->gpioh_dir &= ~BIT(offset - 5);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

int ftdi_gpio_direction_output(struct usb_interface *intf, unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &priv->intf->dev;
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(dev, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 5) {
		low = true;
		priv->gpiol_dir |= BIT(offset) << 3;

		if (value)
			priv->gpiol_mask |= BIT(offset) << 3;
		else
			priv->gpiol_mask &= ~(BIT(offset) << 3);
	} else {
		low = false;
		priv->gpioh_dir |= BIT(offset - 5);

		if (value)
			priv->gpioh_mask |= BIT(offset - 5);
		else
			priv->gpioh_mask &= ~BIT(offset - 5);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static void ftdi_lock(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	mutex_lock(&priv->ops_mutex);
}

static void ftdi_unlock(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);

	mutex_unlock(&priv->ops_mutex);
}

static const struct ft232h_intf_ops ft232h_intf_ops = {
	.ctrl_xfer = ftdi_ctrl_xfer,
	.bulk_xfer = ftdi_bulk_xfer,
	.read_data = ftdi_read_data,
	.write_data = ftdi_write_data,
	.lock = ftdi_lock,
	.unlock = ftdi_unlock,
	.set_bitmode = ftdi_set_bitmode,
	.init_pins = ftdi_mpsse_init_pins,
	.cfg_bus_pins = ftdi_mpsse_cfg_bus_pins,
	.set_clock = ftdi_set_clock,
	.set_latency = ftdi_set_latency,
	.gpio_get = ftdi_gpio_get,
	.gpio_set = ftdi_gpio_set,
	.gpio_direction_input = ftdi_gpio_direction_input,
	.gpio_direction_output = ftdi_gpio_direction_output,
};

/*
 * SPI Master device information
 */

static const struct mpsse_spi_platform_data ft232h_spi_cfg_plat_data = {
	.ops		= &ft232h_intf_ops,
};

static struct platform_device *mpsse_dev_register(struct ft232h_intf_priv *priv,
				const struct mpsse_spi_platform_data *pd)
{
	struct device *parent = &priv->intf->dev;
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(SPI_INTF_DEVNAME, 0);
	if (!pdev)
		return NULL;

	pdev->dev.parent = parent;
	pdev->dev.fwnode = NULL;
	priv->spi_pdev = pdev;

	ret = platform_device_add_data(pdev, pd, sizeof(*pd));
	if (ret)
		goto err;
	pdev->id = priv->id;

	ret = platform_device_add(pdev);
	if (ret < 0)
		goto err;

	dev_dbg(&pdev->dev, "%s done\n", __func__);

	ret = ftdi_spi_probe(pdev);
	if (ret < 0)
		goto err;

	return pdev;

err:
	platform_device_put(pdev);
	return ERR_PTR(ret);
}

static int ft232h_intf_spi_probe(struct usb_interface *intf,
				 const void *plat_data)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;
	struct platform_device *pdev;

	pdev = mpsse_dev_register(priv, plat_data);
	if (IS_ERR(pdev)) {
		dev_err(dev, "%s: Can't create MPSSE SPI device %ld\n",
			__func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	priv->spi_pdev = pdev;
	return 0;
}

static int ft232h_intf_spi_remove(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct device *dev = &intf->dev;

	dev_dbg(dev, "%s: spi pdev %p\n", __func__, priv->spi_pdev);
	platform_device_unregister(priv->spi_pdev);
	return 0;
}

static const struct ft232h_intf_info ft232h_spi_cfg_intf_info = {
	.probe  = ft232h_intf_spi_probe,
	.remove  = ft232h_intf_spi_remove,
	.plat_data  = &ft232h_spi_cfg_plat_data,
};

static int ft232h_intf_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct ft232h_intf_priv *priv;
	struct device *dev = &intf->dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	const struct ft232h_intf_info *info;
	unsigned int i;
	int ret = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->irq_base = -1;

	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_out(endpoint))
			priv->bulk_out = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_in(endpoint)) {
			priv->bulk_in = endpoint->bEndpointAddress;
			priv->bulk_in_pkt_sz = usb_endpoint_maxp(endpoint);
		}
	}

	priv->usb_dev_id = id;
	/* Identify the FTDI channel from the alternate-setting (0=A,1=B,2=C,3=D) */
	priv->index = intf->cur_altsetting->desc.bAlternateSetting + 1;
	priv->intf = intf;
	priv->info = (struct ft232h_intf_info *)id->driver_info;

	info = priv->info;
	if (!info) {
		dev_err(dev, "Missing device specific driver info...\n");
		return -ENODEV;
	}

	mutex_init(&priv->io_mutex);
	mutex_init(&priv->ops_mutex);
	usb_set_intfdata(intf, priv);

	if (!priv->bulk_in_pkt_sz) {
		dev_err(dev, "Missing bulk-in endpoint\n");
		return -ENODEV;
	}

	priv->bulk_in_sz = priv->bulk_in_pkt_sz;

	if (param_perf_profile == PERF_PROFILE_BALANCED)
		priv->bulk_in_sz = max_t(size_t, priv->bulk_in_sz, SZ_4K);
	else if (param_perf_profile == PERF_PROFILE_AGGRESSIVE)
		priv->bulk_in_sz = max_t(size_t, priv->bulk_in_sz, SZ_32K);

	if (param_bulk_in_buf_kb)
		priv->bulk_in_sz = max_t(size_t, priv->bulk_in_sz,
				       param_bulk_in_buf_kb * SZ_1K);

	/* Keep buffers packet-aligned while allowing large HS bursts (<=64 KiB). */
	priv->bulk_in_sz = clamp_t(size_t, priv->bulk_in_sz,
				     priv->bulk_in_pkt_sz, (size_t)SZ_64K);
	priv->bulk_in_sz = roundup(priv->bulk_in_sz, priv->bulk_in_pkt_sz);

	priv->bulk_in_buf = devm_kmalloc(dev, priv->bulk_in_sz, GFP_KERNEL);
	if (!priv->bulk_in_buf)
		return -ENOMEM;

	dev_dbg(dev, "bulk-in cfg: pkt=%zu buf=%zu\n",
		priv->bulk_in_pkt_sz, priv->bulk_in_sz);

	priv->udev = usb_get_dev(interface_to_usbdev(intf));

	priv->id = ida_simple_get(&ftdi_devid_ida, 0, 0, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;

	if (info->probe) {
		ret = info->probe(intf, info->plat_data);
		if (ret < 0)
			goto err;
		return 0;
	}

	return 0;
err:
	ida_simple_remove(&ftdi_devid_ida, priv->id);
	return ret;
}

static int ftdi_mpsse_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	int ret, val;
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d\n", __func__, offset);

	low = offset < 5;

	mutex_lock(&priv->ops_mutex);

	ret = ftdi_mpsse_get_port_pins(priv, low);
	if (ret < 0) {
		mutex_unlock(&priv->ops_mutex);
		return ret;
	}

	if (low)
		val = priv->gpiol_mask & (BIT(offset) << 3);
	else
		val = priv->gpioh_mask & BIT(offset - 5);

	mutex_unlock(&priv->ops_mutex);

	return !!val;
}

static void ftdi_mpsse_gpio_set(struct gpio_chip *chip, unsigned int offset,
				int value)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	bool low;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 4) {
		low = true;
		if (value)
			priv->gpiol_mask |= (BIT(offset) << 4);
		else
			priv->gpiol_mask &= ~(BIT(offset) << 4);
	} else {
		low = false;
		if (value)
			priv->gpioh_mask |= BIT(offset - 4);
		else
			priv->gpioh_mask &= ~BIT(offset - 4);
	}

	ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);
}

static int ftdi_mpsse_gpio_direction_input(struct gpio_chip *chip,
					   unsigned int offset)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	bool low;
	int ret;
	
	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d\n", __func__, offset);

	mutex_lock(&priv->ops_mutex);

	if (offset < 4) {
		low = true;
		priv->gpiol_dir &= ~(BIT(offset) << 4);
	} else {
		low = false;
		priv->gpioh_dir &= ~BIT(offset - 4);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static int ftdi_mpsse_gpio_direction_output(struct gpio_chip *chip,
					    unsigned int offset, int value)
{
	struct ft232h_intf_priv *priv = gpiochip_get_data(chip);
	bool low;
	int ret;

	mutex_lock(&priv->io_mutex);
	if (!priv->intf) {
		mutex_unlock(&priv->io_mutex);
		return -ENODEV;
	}
	mutex_unlock(&priv->io_mutex);

	dev_dbg(chip->parent, "%s: offset %d, val %d\n",
		__func__, offset, value);

	mutex_lock(&priv->ops_mutex);

	if (offset < 4) {
		low = true;
		priv->gpiol_dir |= BIT(offset) << 4;

		if (value)
			priv->gpiol_mask |= BIT(offset) << 4;
		else
			priv->gpiol_mask &= ~(BIT(offset) << 4);
	} else {
		low = false;
		priv->gpioh_dir |= BIT(offset - 4);

		if (value)
			priv->gpioh_mask |= BIT(offset - 4);
		else
			priv->gpioh_mask &= ~BIT(offset - 4);
	}

	ret = ftdi_mpsse_set_port_pins(priv, low);

	mutex_unlock(&priv->ops_mutex);

	return ret;
}

static void mpsse_irq_enable_disable(struct irq_data *data, bool enable)
{
	struct ft232h_intf_priv *priv;
	struct gpio_chip *chip;
	int irq;
	
	priv = (struct ft232h_intf_priv*) irq_data_get_irq_chip_data(data);
	
	if(!priv)
		return;
		
	irq = data->irq - priv->irq_base;

	chip = &priv->mpsse_gpio;

	if (irq < 0 || irq >= chip->ngpio) 
		return;

	priv->irq_enabled[irq] = enable;
}

static void mpsse_irq_enable(struct irq_data *data)
{
	mpsse_irq_enable_disable (data, true);
}

static void mpsse_irq_disable(struct irq_data *data)
{
	mpsse_irq_enable_disable (data, false);
}

static int mpsse_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct ft232h_intf_priv *priv;
	struct gpio_chip *chip;
	int irq;
	
	priv = (struct ft232h_intf_priv*) irq_data_get_irq_chip_data(data);
	
	if(!priv)
		return -EINVAL;
	
	irq = data->irq - priv->irq_base;
	
	chip = &priv->mpsse_gpio;
	
	if (irq < 0 || irq >= chip->ngpio) 
		return -EINVAL;

	priv->irq_type[irq] = type;

	return 0;
}

static int ftdi_mpsse_gpio_to_irq(struct gpio_chip *chip,
                  unsigned offset)
{
	struct ft232h_intf_priv *priv = (struct ft232h_intf_priv*)gpiochip_get_data(chip);
	
	ftdi_mpsse_gpio_direction_input(chip, offset);
	
	priv->irq_enabled[offset] = true;
	
	return priv->irq_base + offset;
}

#ifdef FTDI_IRQ_SPI_POLL
static void ftdi_mpsse_gpio_check(struct ft232h_intf_priv *priv)
{
	struct gpio_chip *chip = &priv->mpsse_gpio;
	unsigned int offset = chip->ngpio;
	int gpio_val = 0;
	
	while(offset--)
	{
		if(!priv->irq_enabled[offset])
			continue;
		
		gpio_val = ftdi_gpio_get(priv->intf, offset);
		
		if (!gpio_val && 
		    (priv->irq_type[offset] == IRQ_TYPE_EDGE_FALLING || 
		     priv->irq_type[offset] == IRQ_TYPE_LEVEL_LOW)) {
			handle_nested_irq(priv->irq_base+offset);
		}
		else if (gpio_val) {
			handle_nested_irq(priv->irq_base+offset);
		}
	}
}

#define FTDI_IRQ_POLL_PERIOD_MS  5
static int ftdi_irq_poll_function(void* argument)
{
	struct ft232h_intf_priv *priv = (struct ft232h_intf_priv*)argument;
	unsigned int next_poll_ms = jiffies_to_msecs(jiffies);
	unsigned int jiffies_ms;
	int drift_ms = 0;
	int corr_ms  = 0;
	int sleep_ms = 0;
	
	while (!kthread_should_stop())
	{
		jiffies_ms = jiffies_to_msecs(jiffies);
		drift_ms   = jiffies_ms - next_poll_ms;
		
		if(!irq_poll_period)
			irq_poll_period = FTDI_IRQ_POLL_PERIOD_MS;
		
		if (drift_ms < 0) {
			corr_ms = (corr_ms > 0) ? corr_ms - 1 : 0;
		}
		else if (drift_ms > 0 && drift_ms < irq_poll_period) {
			corr_ms = (corr_ms < irq_poll_period) ? corr_ms + 1 : 0;
		}
		
		next_poll_ms = jiffies_ms + irq_poll_period;
		
		ftdi_mpsse_gpio_check(priv);
		
		if(kthread_should_stop())
			break;
		
		jiffies_ms = jiffies_to_msecs(jiffies);
		
		// if gpio read > poll period, do not sleep
		if (jiffies_ms <= next_poll_ms) {
			sleep_ms = next_poll_ms - jiffies_ms - corr_ms;
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies((sleep_ms <= 0) ? 1 : sleep_ms));
		}
	}
	
	__set_current_state(TASK_RUNNING);
	
	complete(&priv->gpio_thread_complete);
	
	return 0;
}
#endif /* FTDI_IRQ_SPI_POLL */

static int ftdi_mpsse_irq_probe(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct gpio_chip *chip = &priv->mpsse_gpio;
	struct irq_chip *irqc = &priv->mpsse_irq;
	int i;
	
	chip->to_irq = ftdi_mpsse_gpio_to_irq;
	
	irqc->name         = "mpsse-irq";
	irqc->irq_enable   = mpsse_irq_enable;
	irqc->irq_disable  = mpsse_irq_disable;
	irqc->irq_set_type = mpsse_irq_set_type;

	priv->irq_base = irq_alloc_descs(-1, 0, chip->ngpio, 0);
	if (priv->irq_base < 0)
		return priv->irq_base;
	
	for (i = 0; i < chip->ngpio; i++) {
		priv->irq_enabled[i] = false;
		priv->irq_type[i] = IRQ_TYPE_NONE;
		irq_set_chip(priv->irq_base + i, &priv->mpsse_irq);
		irq_set_chip_data(priv->irq_base + i, priv);
		irq_clear_status_flags(priv->irq_base + i, IRQ_NOREQUEST | IRQ_NOPROBE);
	}
	
	return 0;
}

static int ftdi_mpsse_gpio_probe(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	struct gpio_chip *chip = &priv->mpsse_gpio;
	struct device *parent = &intf->dev;
	struct gpiod_lookup_table *lookup;
	size_t lookup_size;
	char **names, *label;
	int i, ret;

	label = devm_kasprintf(parent, GFP_KERNEL, "ftdi-mpsse-gpio.%d", priv->id);
	if (!label)
		return -ENOMEM;

	chip->label = label;
	chip->parent = parent;
	chip->owner = THIS_MODULE;
	chip->base = (param_gpio_base >= 0) ? param_gpio_base : -1;
	chip->ngpio = FTDI_MPSSE_GPIOS;
	chip->can_sleep = true;
	chip->set = ftdi_mpsse_gpio_set;
	chip->get = ftdi_mpsse_gpio_get;
	chip->direction_input = ftdi_mpsse_gpio_direction_input;
	chip->direction_output = ftdi_mpsse_gpio_direction_output;
	
	names = devm_kcalloc(parent, chip->ngpio, sizeof(char *),
			     GFP_KERNEL);
	if (!names)
		return -ENOMEM;

	for (i = 0; i < chip->ngpio; i++) {
		int offs;

		offs = i < 4 ? 0 : 4;
		names[i] = devm_kasprintf(parent, GFP_KERNEL,
					  "mpsse.%d-GPIO%c%d", priv->id,
					  i < 4 ? 'L' : 'H', i - offs);
		if (!names[i])
			return -ENOMEM;
	}

	chip->names = (const char *const *)names;

	ret = devm_gpiochip_add_data(parent, chip, priv);
	if (ret < 0) {
		dev_err(parent, "Failed to add MPSSE GPIO chip: %d\n", ret);
		return ret;
	}

	dev_info(parent, "gpiochip: label=%s base=%d ngpio=%d\n", 
			chip->label, chip->base, chip->ngpio);

	lookup_size = sizeof(struct gpiod_lookup_table);
   	lookup_size += (chip->ngpio + 1) * sizeof(struct gpiod_lookup);

	lookup = devm_kzalloc(parent, lookup_size, GFP_KERNEL);
	if (!lookup)
		return -ENOMEM;

	lookup->dev_id = devm_kasprintf(parent, GFP_KERNEL, "%s.%d",
						priv->spi_pdev->name, priv->spi_pdev->id);
	if (!lookup->dev_id)
		return -ENOMEM;

	for (i = 0; i < chip->ngpio ; i++) {
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,7,19)
		lookup->table[i].chip_label = chip->label;
#else
		lookup->table[i].key = chip->label;
#endif
		lookup->table[i].chip_hwnum = i;
		if (i < 4) {
			lookup->table[i].idx = i;
			lookup->table[i].con_id = "gpiol";
		} else {
			lookup->table[i].idx = i - 4;
			lookup->table[i].con_id = "gpioh";
		}

		lookup->table[i].flags = GPIOD_IN;
	}

	priv->lookup_gpios = lookup;
	gpiod_add_lookup_table(priv->lookup_gpios);

#ifdef FTDI_IRQ_SPI_POLL	
	init_completion(&priv->gpio_thread_complete);
	priv->gpio_thread = kthread_run(&ftdi_irq_poll_function, priv, "ftdi-irq-poll");
#else
	priv->gpio_thread = NULL;
#endif
	
	return 0;
}

static void ft232h_intf_disconnect(struct usb_interface *intf)
{
	struct ft232h_intf_priv *priv = usb_get_intfdata(intf);
	const struct ft232h_intf_info *info;

	if(priv->gpio_thread){
		kthread_stop(priv->gpio_thread);
		wake_up_process (priv->gpio_thread);
		wait_for_completion(&priv->gpio_thread_complete);
	}
	
	ftdi_mpsse_irq_remove(intf);
        ftdi_mpsse_gpio_remove(intf);
	ftdi_spi_remove(priv->spi_pdev);

	info = (struct ft232h_intf_info *)priv->usb_dev_id->driver_info;
	if (info && info->remove)
		info->remove(intf);

	mutex_lock(&priv->io_mutex);
	priv->intf = NULL;
	usb_set_intfdata(intf, NULL);
	mutex_unlock(&priv->io_mutex);

	usb_put_dev(priv->udev);
	ida_simple_remove(&ftdi_devid_ida, priv->id);

	mutex_destroy(&priv->io_mutex);
	mutex_destroy(&priv->ops_mutex);
}

/*
 * USB device information
 */
static struct usb_device_id ft232h_intf_table[] = {
#ifndef CONFIG_USB_SERIAL_FTDI_SIO
	{ USB_DEVICE(0x0403, 0x6011), .driver_info = (kernel_ulong_t)&ft232h_spi_cfg_intf_info },
	{ USB_DEVICE(0x0403, 0x6014), .driver_info = (kernel_ulong_t)&ft232h_spi_cfg_intf_info },
#endif
	{ USB_DEVICE(0x2beb, 0x0146), .driver_info = (kernel_ulong_t)&ft232h_spi_cfg_intf_info },
	{}
};
MODULE_DEVICE_TABLE(usb, ft232h_intf_table);

static struct usb_driver ft232h_intf_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= ft232h_intf_table,
	.probe		= ft232h_intf_probe,
	.disconnect	= ft232h_intf_disconnect,
};

module_usb_driver(ft232h_intf_driver);

MODULE_ALIAS("spi-ft232h");
MODULE_AUTHOR("Yuji Sasaki <sasaki@silexamerica.com>");
MODULE_AUTHOR("Anatolij Gustschin <agust@denx.de>");
MODULE_AUTHOR("James Ewing <james@teledatics.com>");
MODULE_DESCRIPTION("FT232H USB-SPI host driver");
MODULE_LICENSE("GPL v2");
