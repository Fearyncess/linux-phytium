/*
 * Phytium SPI core controller driver.
 *
 * Copyright (c) 2019, Phytium Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/acpi.h>

#define CTRL0			0x00
#define SSIENR			0x08
#define SER			0x10
#define BAUDR			0x14
#define TXFLTR			0x18
#define TXFLR			0x20
#define RXFLR			0x24
#define IMR			0x2c
#define ISR			0x30
#define ICR			0x48
#define DR			0x60

#define FRF_OFFSET		4
#define MODE_OFFSET		6
#define TMOD_OFFSET		8

#define TMOD_MASK		(0x3 << TMOD_OFFSET)
#define	TMOD_TR			0x0
#define TMOD_TO			0x1
#define TMOD_RO			0x2

#define INT_TXEI		(1 << 0)
#define INT_TXOI		(1 << 1)
#define INT_RXUI		(1 << 2)
#define INT_RXOI		(1 << 3)

struct ft_spi {
	struct spi_master	*master;
	char			name[16];

	void __iomem		*regs;
	unsigned long		paddr;
	int			irq;
	u32			fifo_len;
	u32			max_freq;

	u32			reg_io_width;
	u16			bus_num;
	u16			num_cs;

	size_t			len;
	void			*tx;
	void			*tx_end;
	void			*rx;
	void			*rx_end;
	u8			n_bytes;
	irqreturn_t		(*transfer_handler)(struct ft_spi *fts);
};

static inline u32 ft_readl(struct ft_spi *fts, u32 offset)
{
	return __raw_readl(fts->regs + offset);
}

static inline u16 ft_readw(struct ft_spi *fts, u32 offset)
{
	return __raw_readw(fts->regs + offset);
}

static inline void ft_writel(struct ft_spi *fts, u32 offset, u32 val)
{
	__raw_writel(val, fts->regs + offset);
}

static inline void ft_writew(struct ft_spi *fts, u32 offset, u16 val)
{
	__raw_writew(val, fts->regs + offset);
}

static inline u32 ft_read_io_reg(struct ft_spi *fts, u32 offset)
{
	switch (fts->reg_io_width) {
	case 2:
		return ft_readw(fts, offset);
	case 4:
	default:
		return ft_readl(fts, offset);
	}
}

static inline void ft_write_io_reg(struct ft_spi *fts, u32 offset, u32 val)
{
	switch (fts->reg_io_width) {
	case 2:
		ft_writew(fts, offset, val);
		break;
	case 4:
	default:
		ft_writel(fts, offset, val);
		break;
	}
}

static inline void spi_enable_chip(struct ft_spi *fts, int enable)
{
	ft_writel(fts, SSIENR, (enable ? 1 : 0));
}

static inline void spi_set_clk(struct ft_spi *fts, u16 div)
{
	ft_writel(fts, BAUDR, div);
}

static inline void spi_mask_intr(struct ft_spi *fts, u32 mask)
{
	u32 new_mask;

	new_mask = ft_readl(fts, IMR) & ~mask;
	ft_writel(fts, IMR, new_mask);
}

static inline void spi_umask_intr(struct ft_spi *fts, u32 mask)
{
	u32 new_mask;

	new_mask = ft_readl(fts, IMR) | mask;
	ft_writel(fts, IMR, new_mask);
}

static inline void spi_reset_chip(struct ft_spi *fts)
{
	spi_enable_chip(fts, 0);
	spi_mask_intr(fts, 0xff);
	spi_enable_chip(fts, 1);
}

static inline void spi_shutdown_chip(struct ft_spi *fts)
{
	spi_enable_chip(fts, 0);
	spi_set_clk(fts, 0);
}

struct ft_spi_chip {
	u8 poll_mode;
	u8 type;
	void (*cs_control)(u32 command);
};

struct chip_data {
	u8 cs;
	u8 tmode;
	u8 type;

	u8 poll_mode;

	u16 clk_div;
	u32 speed_hz;
	void (*cs_control)(u32 command);
};

static void ft_spi_set_cs(struct spi_device *spi, bool enable)
{
	struct ft_spi *fts = spi_master_get_devdata(spi->master);
	struct chip_data *chip = spi_get_ctldata(spi);

	if (chip && chip->cs_control)
		chip->cs_control(!enable);

	if (!enable)
		ft_writel(fts, SER, BIT(spi->chip_select));
}

static inline u32 tx_max(struct ft_spi *fts)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (fts->tx_end - fts->tx) / fts->n_bytes;
	tx_room = fts->fifo_len - ft_readl(fts, TXFLR);

	rxtx_gap =  ((fts->rx_end - fts->rx) - (fts->tx_end - fts->tx))
			/ fts->n_bytes;

	return min3(tx_left, tx_room, (u32) (fts->fifo_len - rxtx_gap));
}

static inline u32 rx_max(struct ft_spi *fts)
{
	u32 rx_left = (fts->rx_end - fts->rx) / fts->n_bytes;

	return min_t(u32, rx_left, ft_readl(fts, RXFLR));
}

static void ft_writer(struct ft_spi *fts)
{
	u32 max = tx_max(fts);
	u16 txw = 0;

	while (max--) {
		if (fts->tx_end - fts->len) {
			if (fts->n_bytes == 1)
				txw = *(u8 *)(fts->tx);
			else
				txw = *(u16 *)(fts->tx);
		}
		ft_write_io_reg(fts, DR, txw);
		fts->tx += fts->n_bytes;
	}
}

static void ft_reader(struct ft_spi *fts)
{
	u32 max = rx_max(fts);
	u16 rxw;

	while (max--) {
		rxw = ft_read_io_reg(fts, DR);
		if (fts->rx_end - fts->len) {
			if (fts->n_bytes == 1)
				*(u8 *)(fts->rx) = rxw;
			else
				*(u16 *)(fts->rx) = rxw;
		}
		fts->rx += fts->n_bytes;
	}
}

static void int_error_stop(struct ft_spi *fts, const char *msg)
{
	spi_reset_chip(fts);

	dev_err(&fts->master->dev, "%s\n", msg);
	fts->master->cur_msg->status = -EIO;
	spi_finalize_current_transfer(fts->master);
}

static irqreturn_t interrupt_transfer(struct ft_spi *fts)
{
	u16 irq_status = ft_readl(fts, ISR);

	if (irq_status & (INT_TXOI | INT_RXOI | INT_RXUI)) {
		ft_readl(fts, ICR);
		int_error_stop(fts, "interrupt_transfer: fifo overrun/underrun");
		return IRQ_HANDLED;
	}

	ft_reader(fts);
	if (fts->rx_end == fts->rx) {
		spi_mask_intr(fts, INT_TXEI);
		spi_finalize_current_transfer(fts->master);
		return IRQ_HANDLED;
	}
	if (irq_status & INT_TXEI) {
		spi_mask_intr(fts, INT_TXEI);
		ft_writer(fts);
		spi_umask_intr(fts, INT_TXEI);
	}

	return IRQ_HANDLED;
}

static irqreturn_t ft_spi_irq(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct ft_spi *fts = spi_master_get_devdata(master);
	u16 irq_status = ft_readl(fts, ISR) & 0x3f;

	if (!irq_status)
		return IRQ_NONE;

	if (!master->cur_msg) {
		spi_mask_intr(fts, INT_TXEI);
		return IRQ_HANDLED;
	}

	return fts->transfer_handler(fts);
}

static int poll_transfer(struct ft_spi *fts)
{
	do {
		ft_writer(fts);
		ft_reader(fts);
		cpu_relax();
	} while (fts->rx_end > fts->rx);

	return 0;
}

static int ft_spi_transfer_one(struct spi_master *master,
		struct spi_device *spi, struct spi_transfer *transfer)
{
	struct ft_spi *fts = spi_master_get_devdata(master);
	struct chip_data *chip = spi_get_ctldata(spi);
	u8 imask = 0;
	u16 txlevel = 0;
	u16 clk_div;
	u32 cr0;

	fts->tx = (void *)transfer->tx_buf;
	fts->tx_end = fts->tx + transfer->len;
	fts->rx = transfer->rx_buf;
	fts->rx_end = fts->rx + transfer->len;
	fts->len = transfer->len;

	spi_enable_chip(fts, 0);

	if (transfer->speed_hz != chip->speed_hz) {
		clk_div = (fts->max_freq / transfer->speed_hz + 1) & 0xfffe;

		chip->speed_hz = transfer->speed_hz;
		chip->clk_div = clk_div;

		spi_set_clk(fts, chip->clk_div);
	}

	if (transfer->bits_per_word == 8) {
		fts->n_bytes = 1;
	} else if (transfer->bits_per_word == 16) {
		fts->n_bytes = 2;
	} else {
		return -EINVAL;
	}

	cr0 = (transfer->bits_per_word - 1)
		| (chip->type << FRF_OFFSET)
		| (spi->mode << MODE_OFFSET)
		| (chip->tmode << TMOD_OFFSET);

	if (chip->cs_control) {
		if (fts->rx && fts->tx)
			chip->tmode = TMOD_TR;
		else if (fts->rx)
			chip->tmode = TMOD_RO;
		else
			chip->tmode = TMOD_TO;

		cr0 &= ~TMOD_MASK;
		cr0 |= (chip->tmode << TMOD_OFFSET);
	}

	ft_writel(fts, CTRL0, cr0);

	spi_mask_intr(fts, 0xff);

	if (!chip->poll_mode) {
		txlevel = min_t(u16, fts->fifo_len / 2, fts->len / fts->n_bytes);
		ft_writel(fts, TXFLTR, txlevel);

		imask |= INT_TXEI | INT_TXOI |
			 INT_RXUI | INT_RXOI;
		spi_umask_intr(fts, imask);

		fts->transfer_handler = interrupt_transfer;
	}

	spi_enable_chip(fts, 1);

	if (chip->poll_mode)
		return poll_transfer(fts);

	return 1;
}

static void ft_spi_handle_err(struct spi_master *master,
		struct spi_message *msg)
{
	struct ft_spi *fts = spi_master_get_devdata(master);

	spi_reset_chip(fts);
}

static int ft_spi_setup(struct spi_device *spi)
{
	struct ft_spi_chip *chip_info = NULL;
	struct chip_data *chip;
	int ret;

	chip = spi_get_ctldata(spi);
	if (!chip) {
		chip = kzalloc(sizeof(struct chip_data), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;
		spi_set_ctldata(spi, chip);
	}

	chip_info = spi->controller_data;

	if (chip_info) {
		if (chip_info->cs_control)
			chip->cs_control = chip_info->cs_control;

		chip->poll_mode = chip_info->poll_mode;
		chip->type = chip_info->type;
	}

	chip->tmode = 0;

	if (gpio_is_valid(spi->cs_gpio)) {
		ret = gpio_direction_output(spi->cs_gpio,
				!(spi->mode & SPI_CS_HIGH));
		if (ret)
			return ret;
	}

	return 0;
}

static void ft_spi_cleanup(struct spi_device *spi)
{
	struct chip_data *chip = spi_get_ctldata(spi);

	kfree(chip);
	spi_set_ctldata(spi, NULL);
}

static void spi_hw_init(struct device *dev, struct ft_spi *fts)
{
	spi_reset_chip(fts);

	if (!fts->fifo_len) {
		u32 fifo;

		for (fifo = 1; fifo < 256; fifo++) {
			ft_writel(fts, TXFLTR, fifo);
			if (fifo != ft_readl(fts, TXFLTR))
				break;
		}
		ft_writel(fts, TXFLTR, 0);

		fts->fifo_len = (fifo == 1) ? 0 : fifo;
		dev_dbg(dev, "Detected FIFO size: %u bytes\n", fts->fifo_len);
	}
}

int ft_spi_add_host(struct device *dev, struct ft_spi *fts)
{
	struct spi_master *master;
	int ret;

	BUG_ON(fts == NULL);

	master = spi_alloc_master(dev, 0);
	if (!master)
		return -ENOMEM;

	fts->master = master;
	snprintf(fts->name, sizeof(fts->name), "ft_spi%d", fts->bus_num);

	ret = request_irq(fts->irq, ft_spi_irq, IRQF_SHARED, fts->name, master);
	if (ret < 0) {
		dev_err(dev, "can not get IRQ\n");
		goto err_free_master;
	}

	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);
	master->bus_num = fts->bus_num;
	master->num_chipselect = fts->num_cs;
	master->setup = ft_spi_setup;
	master->cleanup = ft_spi_cleanup;
	master->set_cs = ft_spi_set_cs;
	master->transfer_one = ft_spi_transfer_one;
	master->handle_err = ft_spi_handle_err;
	master->max_speed_hz = fts->max_freq;
	master->dev.of_node = dev->of_node;

	spi_hw_init(dev, fts);

	spi_master_set_devdata(master, fts);
	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&master->dev, "problem registering spi master\n");
		goto err_exit;
	}

	return 0;

err_exit:
	spi_enable_chip(fts, 0);
	free_irq(fts->irq, master);
err_free_master:
	spi_master_put(master);
	return ret;
}

void ft_spi_remove_host(struct ft_spi *fts)
{
	spi_shutdown_chip(fts);

	free_irq(fts->irq, fts->master);
}

#define DRIVER_NAME "phytium_spi"

struct ft_spi_clk {
	struct ft_spi  fts;
	struct clk     *clk;
};

static int ft_spi_probe(struct platform_device *pdev)
{
	struct ft_spi_clk *ftsc;
	struct ft_spi *fts;
	struct resource *mem;
	int ret;
	int num_cs;

	ftsc = devm_kzalloc(&pdev->dev, sizeof(struct ft_spi_clk),
			GFP_KERNEL);
	if (!ftsc)
		return -ENOMEM;

	fts = &ftsc->fts;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -EINVAL;
	}

	fts->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(fts->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(fts->regs);
	}

	fts->irq = platform_get_irq(pdev, 0);
	if (fts->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return fts->irq; /* -ENXIO */
	}

	if (pdev->dev.of_node) {
		ftsc->clk = devm_clk_get(&pdev->dev, NULL);

		if (IS_ERR(ftsc->clk))
			return PTR_ERR(ftsc->clk);
		ret = clk_prepare_enable(ftsc->clk);
		if (ret)
			return ret;

		fts->max_freq = clk_get_rate(ftsc->clk);
	} else if (has_acpi_companion(&pdev->dev)) {
		fts->max_freq = 48000000;
	}

	fts->bus_num = pdev->id;
	device_property_read_u32(&pdev->dev, "reg-io-width", &fts->reg_io_width);

	num_cs = 4;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	fts->num_cs = num_cs;

	if (pdev->dev.of_node) {
		int i;

		for (i = 0; i < fts->num_cs; i++) {
			int cs_gpio = of_get_named_gpio(pdev->dev.of_node,
					"cs-gpios", i);

			if (cs_gpio == -EPROBE_DEFER) {
				ret = cs_gpio;
				goto out;
			}

			if (gpio_is_valid(cs_gpio)) {
				ret = devm_gpio_request(&pdev->dev, cs_gpio,
						dev_name(&pdev->dev));
				if (ret)
					goto out;
			}
		}
	}

	ret = ft_spi_add_host(&pdev->dev, fts);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, ftsc);
	return 0;

out:
	clk_disable_unprepare(ftsc->clk);
	return ret;
}

static int ft_spi_remove(struct platform_device *pdev)
{
	struct ft_spi_clk *ftsc = platform_get_drvdata(pdev);

	ft_spi_remove_host(&ftsc->fts);
	clk_disable_unprepare(ftsc->clk);

	return 0;
}

static const struct of_device_id ft_spi_of_match[] = {
	{ .compatible = "phytium,spi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, ft_spi_of_match);

static const struct acpi_device_id ft_spi_acpi_match[] = {
	{"PHTY000E", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, ft_spi_acpi_match);

static struct platform_driver ft_spi_driver = {
	.probe		= ft_spi_probe,
	.remove		= ft_spi_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(ft_spi_of_match),
		.acpi_match_table = ACPI_PTR(ft_spi_acpi_match),
	},
};
module_platform_driver(ft_spi_driver);

MODULE_AUTHOR("Mingshuai Zhu <zhumingshuai@phytium.com.cn>");
MODULE_AUTHOR("Chen Baozi <chenbaozi@phytium.com.cn>");
MODULE_DESCRIPTION("Driver for Phytium SPI controller core");
MODULE_LICENSE("GPL v2");
