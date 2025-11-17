# Teledatics FTDI SPI Linux driver

This kernel module exposes FTDI FT232H/FT2232H/FT4232H devices as a high-speed SPI master. It is the default SPI transport for the Teledatics TD-XPAH development platform and should be loaded before the `nrc.ko` Wi-Fi driver.

## Module parameters

Load-time parameters allow the data path to be tuned per target. Parameters are supplied with `modprobe spi-ft232h <name>=<value>`.

| Parameter | Default | Notes |
|-----------|---------|-------|
| `latency` | `1` | USB latency timer in milliseconds. |
| `usb_wait_msec` | `0` | Optional post-USB transfer delay (legacy). |
| `spi_bus_num` | auto | Override registered SPI bus number. |
| `gpio_base_num` | auto | Override GPIO base. |
| `perf_profile` | `2` | Performance presets: `0` legacy, `1` balanced, `2` aggressive. Controls default burst sizes, buffering, and pipelining. |
| `max_block` | auto | Maximum SPI payload bytes per USB burst. When unset the driver derives a safe value from the profile. |
| `bulk_in_buf_kb` | profile dependent | Overrides the bulk-IN buffer size (KiB). Must be divisible by endpoint max packet (512 bytes on HS links). |
| `flush_per_block` | profile | When left unset the driver picks based on `perf_profile` (legacy/balanced: on, aggressive: off). Controls whether SEND_IMMEDIATE is issued after each block. |
| `rx_retry_us` | `0` | Optional sleep between empty USB reads. Setting `25`–`75` reduces CPU burn during long bursts. |
| `enable_stats` | `1` | Enable debugfs statistics collection (see below). |
| `pipeline_depth` | profile | Number of concurrent bulk-IN URBs. If left at `0`, the driver picks based on profile (legacy:1, balanced:2, aggressive:4). |

The legacy behaviour (profile `0`) matches earlier releases. Profiles `1` and `2` raise internal burst sizes and buffer allocations without requiring explicit `max_block`/`bulk_in_buf_kb` overrides.

## Performance presets

Use module parameters to switch between predictable legacy behaviour and high-throughput modes.

* **Legacy** – Compatible with previous releases. Example: `modprobe spi-ft232h perf_profile=0 flush_per_block=1 pipeline_depth=1`
* **Balanced profile** – Moderate batching with conservative flushing: `modprobe spi-ft232h perf_profile=1 rx_retry_us=25`
* **Aggressive profile (default)** – Maximum throughput with manual flush control:
  ```bash
  modprobe spi-ft232h perf_profile=2 max_block=32768 bulk_in_buf_kb=64 \
      flush_per_block=0 rx_retry_us=50 enable_stats=1 pipeline_depth=3
  ```

The aggressive preset expects a stable HS USB link and enables statistics by default so that tuning can be verified. When left at the default settings the driver collects runtime metrics and may automatically scale back zero-copy TX or pipeline depth if the host link indicates repeated stalls.

Full-duplex messages now share the same zero-copy fast path that previously applied only to TX-only workloads. Transfers using 8-bit words and >=4 KiB bursts avoid an extra memcpy, and the chunk-level stats exported via debugfs capture both TX-only and duplex pipelines so that adaptive autotuning decisions are visible in a single set of counters.

## Runtime stats and benchmarking

When `enable_stats=1` (default) the driver exports cumulative counters under debugfs (one directory per registered controller):

```
/sys/kernel/debug/ftdi_spi/<controller>/stats
```

Each read returns a JSON object containing byte counters, transfer counts, aggregate latencies, and error totals. Writing to the companion `stats_reset` file clears the counters.

The helper `tools/ftdi_spi_workload.py` script snapshots these stats around an arbitrary workload and records the result as JSON:

```bash
sudo python3 tools/ftdi_spi_workload.py \
  --label full-duplex-10MHz \
  --command "spidev_test -D /dev/spidev0.0 -s 10000000 -l 8192" \
  --iterations 8 \
  --stats-path /sys/kernel/debug/ftdi_spi/spi-ft232h.0/stats \
  --output ../../reports/benchmarks/ftdi_spi_latest.json --append
```

Store generated benchmark artefacts under `reports/benchmarks/` to keep an audit trail. The repository includes `reports/benchmarks/ftdi_spi_sample.json` as a template for expected fields.

## Building

Cross-compilation:

```
ARCH=<arch> CROSS_COMPILE=<prefix> INSTALL_MOD_PATH=<dest> KDIR=<kernel> make
ARCH=<arch> CROSS_COMPILE=<prefix> INSTALL_MOD_PATH=<dest> KDIR=<kernel> make modules_install
```

Native builds:

```
KDIR=<kernel> make
KDIR=<kernel> make modules_install
```

On load the driver reports the assigned SPI bus number and GPIO base in dmesg. The TD-XPAH design only uses the first SPI bus/GPIO block; please consult "NRC7292 Application Note (FT232H_USB_SPI)" for the downstream `nrc7292` driver parameters.
