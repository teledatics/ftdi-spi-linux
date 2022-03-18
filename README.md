# Teledatics FTDI SPI Linux driver

This module is for use with the Teledatics TD-XPAH development platform. Load the module before the nrc.ko Wi-Fi driver module.

Parameters (optional):

modprobe spi-ft232h usb_wait_msec=(ms delay after USB xfer) param_bus_num=(spi bus number) param_gpio_base=(gpio controller base number) latency=(latency in ms)


Cross compiling:

ARCH=<architecture> CROSS_COMPILE=<compiler_prefix> INSTALL_MOD_PATH=<destination_dir> KDIR=<kernel_source_dir> make

ARCH=<architecture> CROSS_COMPILE=<compiler_prefix> INSTALL_MOD_PATH=<destination_dir> KDIR=<kernel_source_dir> make modules_install

Native compiling:

KDIR=<kernel_source_dir> make

KDIR=<kernel_source_dir> make modules_install

On loading the driver will print the SPI bus number values and the GPIO base number values. These are used as parameters when loading the nrc.ko module.

The TD-XPAH uses only the first SPI bus and GPIO base numbers. These should not change on a given platform. Please see the Newracom PDF document "NRC7292 Application Note (FT232H_USB_SPI)" for the
parameters necessary for the nrc7292 Linux driver.
