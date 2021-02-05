/* Phytium CAN device driver
 *
 * Copyright (C) 2018-2019, Phytium Technology Co., Ltd.
 *
 * Author: Leo Hou <houyuefei@phytium.com.cn>
 * Author: Chen Baozi <chenbaozi@phytium.com.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed as is WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>

#define DRIVER_NAME "phytium_can"

/* CAN registers offset */
enum ftcan_reg {
	FTCAN_CTRL_OFFSET		= 0x00,  /* Global control register */
	FTCAN_INTR_OFFSET		= 0x04,  /* Interrupt register */
	FTCAN_ARB_RATE_CTRL_OFFSET	= 0x08,  /* Arbitration rate control register */
	FTCAN_DAT_RATE_CTRL_OFFSET	= 0x0C,  /* Data rate control register */
	FTCAN_ACC_ID0_OFFSET	  	= 0x10,  /* Acceptance identifier0 register */
	FTCAN_ACC_ID1_OFFSET	  	= 0x14,  /* Acceptance identifier1 register */
	FTCAN_ACC_ID2_OFFSET	  	= 0x18,  /* Acceptance identifier2 register */
	FTCAN_ACC_ID3_OFFSET	  	= 0x1C,  /* Acceptance identifier3 register */
	FTCAN_ACC_ID0_MASK_OFFSET 	= 0x20,  /* Acceptance identifier0 mask register */
	FTCAN_ACC_ID1_MASK_OFFSET 	= 0x24,  /* Acceptance identifier1 mask register */
	FTCAN_ACC_ID2_MASK_OFFSET 	= 0x28,  /* Acceptance identifier2 mask register */
	FTCAN_ACC_ID3_MASK_OFFSET 	= 0x2C,  /* Acceptance identifier3 mask register */
	FTCAN_XFER_STS_OFFSET	  	= 0x30,  /* Transfer status register */
	FTCAN_ERR_CNT_OFFSET	  	= 0x34,  /* Error counter register */
	FTCAN_FIFO_CNT_OFFSET	  	= 0x38,  /* FIFO counter register */
	FTCAN_DMA_CTRL_OFFSET	  	= 0x3C,  /* DMA request control register */
	FTCAN_TX_FIFO_OFFSET	  	= 0x100, /* TX FIFO shadow register */
	FTCAN_RX_FIFO_OFFSET	  	= 0x200, /* RX FIFO shadow register */
};


/*---------------------------------------------------------------------------*/
/* CAN register bit masks - FTCAN_<REG>_<BIT>_MASK                           */
/*---------------------------------------------------------------------------*/
/* FTCAN_CTRL mask */
#define FTCAN_CTRL_XFER_MASK   (0x1 << 0)  /*Transfer enable*/
#define FTCAN_CTRL_TXREQ_MASK  (0x1 << 1)  /*Transmit request*/
#define FTCAN_CTRL_AIME_MASK   (0x1 << 2)  /*Acceptance identifier mask enable*/

/* FTCAN_INTR mask */
#define FTCAN_INTR_STATUS_MASK (0xFF << 0) /*the interrupt status*/
#define FTCAN_INTR_BOIS_MASK   (0x1 << 0)  /*Bus off interrupt status*/
#define FTCAN_INTR_PWIS_MASK   (0x1 << 1)  /*Passive warning interrupt status*/
#define FTCAN_INTR_PEIS_MASK   (0x1 << 2)  /*Passive error interrupt status*/
#define FTCAN_INTR_RFIS_MASK   (0x1 << 3)  /*RX FIFO full interrupt status*/
#define FTCAN_INTR_TFIS_MASK   (0x1 << 4)  /*TX FIFO empty interrupt status*/
#define FTCAN_INTR_REIS_MASK   (0x1 << 5)  /*RX frame end interrupt status*/
#define FTCAN_INTR_TEIS_MASK   (0x1 << 6)  /*TX frame end interrupt status*/
#define FTCAN_INTR_EIS_MASK    (0x1 << 7)  /*Error interrupt status*/

#define FTCAN_INTR_EN_MASK     (0xFF << 8) /*the interrupt enable*/
#define FTCAN_INTR_BOIE_MASK   (0x1 << 8)  /*Bus off interrupt enable*/
#define FTCAN_INTR_PWIE_MASK   (0x1 << 9)  /*Passive warning interrupt enable*/
#define FTCAN_INTR_PEIE_MASK   (0x1 << 10) /*Passive error interrupt enable*/
#define FTCAN_INTR_RFIE_MASK   (0x1 << 11) /*RX FIFO full interrupt enable*/
#define FTCAN_INTR_TFIE_MASK   (0x1 << 12) /*TX FIFO empty interrupt enable*/
#define FTCAN_INTR_REIE_MASK   (0x1 << 13) /*RX frame end interrupt enable*/
#define FTCAN_INTR_TEIE_MASK   (0x1 << 14) /*TX frame end interrupt enable*/
#define FTCAN_INTR_EIE_MASK    (0x1 << 15) /*Error interrupt enable*/

#define FTCAN_INTR_BOIC_MASK   (0x1 << 16) /*Bus off interrupt clear*/
#define FTCAN_INTR_PWIC_MASK   (0x1 << 17) /*Passive warning interrupt clear*/
#define FTCAN_INTR_PEIC_MASK   (0x1 << 18) /*Passive error interrupt clear*/
#define FTCAN_INTR_RFIC_MASK   (0x1 << 19) /*RX FIFO full interrupt clear*/
#define FTCAN_INTR_TFIC_MASK   (0x1 << 20) /*TX FIFO empty interrupt clear*/
#define FTCAN_INTR_REIC_MASK   (0x1 << 21) /*RX frame end interrupt clear*/
#define FTCAN_INTR_TEIC_MASK   (0x1 << 22) /*TX frame end interrupt clear*/
#define FTCAN_INTR_EIC_MASK    (0x1 << 23) /*Error interrupt clear*/

/* FTCAN_ACC_ID(0-3)_MASK mask */
#define FTCAN_ACC_IDN_MASK      0x1FFFFFFF /*don’t care the matching */

/* FTCAN_ERR_CNT_OFFSET mask */
#define FTCAN_ERR_CNT_RFN_MASK (0xFF << 0) /*Receive error counter*/
#define FTCAN_ERR_CNT_TFN_MASK (0xFF << 16)/*Transmit error counter*/

/* FTCAN_FIFO_CNT_OFFSET mask */
#define FTCAN_FIFO_CNT_RFN_MASK (0xFF << 0) /*Receive FIFO valid data number*/
#define FTCAN_FIFO_CNT_TFN_MASK (0xFF << 16)/*Transmit FIFO valid data number*/

#define FTCAN_ERR_CNT_TFN_SHIFT	  16  /* Tx Error Count shift */
#define FTCAN_FIFO_CNT_TFN_SHIFT  16  /* Tx FIFO Count shift*/
#define FTCAN_IDR_ID1_SHIFT       21  /* Standard Messg Identifier */
#define FTCAN_IDR_ID2_SHIFT       1   /* Extended Message Identifier */
#define FTCAN_IDR_SDLC_SHIFT      14
#define FTCAN_IDR_EDLC_SHIFT      26

#define FTCAN_IDR_ID2_MASK	0x0007FFFE /* Extended message ident */
#define FTCAN_IDR_ID1_MASK	0xFFE00000 /* Standard msg identifier */
#define FTCAN_IDR_IDE_MASK	0x00080000 /* Identifier extension */
#define FTCAN_IDR_SRR_MASK	0x00100000 /* Substitute remote TXreq */
#define FTCAN_IDR_RTR_MASK	0x00000001 /* Extended frames remote TX request */
#define FTCAN_IDR_DLC_MASK	0x0003C000 /* Standard msg dlc */
#define FTCAN_IDR_PAD_MASK	0x00003FFF /* Standard msg padding 1 */

#define FTCAN_INTR_EN		(FTCAN_INTR_TEIE_MASK | FTCAN_INTR_REIE_MASK)

#define FTCAN_INTR_DIS      0x00000000
#define FTCAN_NAPI_WEIGHT	64

/**
 * struct ftcan_priv - This definition define CAN driver instance
 * @can:		CAN private data structure.
 * @tx_head:		Tx CAN packets ready to send on the queue
 * @tx_tail:		Tx CAN packets successfully sended on the queue
 * @tx_max:		Maximum number packets the driver can send
 * @napi:		NAPI structure
 * @read_reg:		For reading data from CAN registers
 * @write_reg:		For writing data to CAN registers
 * @set_reg_bits:	For writing data to CAN registers bit
 * @clr_reg_bits:	For writing 0 to CAN registers bit
 * @dev:		Network device data structure
 * @reg_base:		Ioremapped address to registers
 * @irq_flags:		For request_irq()
 * @can_clk:		Pointer to struct clk
 * @lock:		The spin lock flag
 * @isr:		The interrupt status
 */
struct ftcan_priv {
	struct can_priv can;
	unsigned int tx_head;
	unsigned int tx_tail;
	unsigned int tx_max;
	u32 (*read_reg)(const struct ftcan_priv *priv,
			enum ftcan_reg reg);
	void (*write_reg)(const struct ftcan_priv *priv,
			  enum ftcan_reg reg, u32 val);
	void (*set_reg_bits)(const struct ftcan_priv *priv,
			     enum ftcan_reg reg, u32 bs);
	void (*clr_reg_bits)(const struct ftcan_priv *priv,
			     enum ftcan_reg reg, u32 bs);
	struct device *dev;
	void __iomem *reg_base;
	unsigned long irq_flags;
	struct clk *can_clk;
	spinlock_t lock;
	u32 isr;
};

struct ftcan_tasklet {
	struct net_device *ndev;
	struct tasklet_struct   *done_task;
};

/* CAN Bittiming constants as per Phytium CAN specs */
static const struct can_bittiming_const ftcan_bittiming_const = {
	.name = DRIVER_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 512,
	.brp_inc = 2,
};

/**
 * ftcan_write_reg - Write a value to the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 * @val:	Value to write at the Register offset
 *
 * Write data to the paricular CAN register
 */
static void ftcan_write_reg(const struct ftcan_priv *priv,
			    enum ftcan_reg reg, u32 val)
{
	writel(val, priv->reg_base + reg);
}

/**
 * ftcan_read_reg - Read a value from the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 *
 * Read data from the particular CAN register
 * Return: value read from the CAN register
 */
static u32 ftcan_read_reg(const struct ftcan_priv *priv, enum ftcan_reg reg)
{
	return readl(priv->reg_base + reg);
}

/**
 * ftcan_set_reg_bits - set a bit value to the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 * @bs:     The bit mask
 *
 * Read data from the particular CAN register
 * Return: value read from the CAN register
 */
static void ftcan_set_reg_bits(const struct ftcan_priv *priv,
			       enum ftcan_reg reg, u32 bs)
{
	u32 val = readl(priv->reg_base + reg);

	val |= bs;
	writel(val, priv->reg_base + reg);
}

/**
 * ftcan_clr_reg_bits - clear a bit value to the device register
 * @priv:	Driver private data structure
 * @reg:	Register offset
 * @bs:     The bit mask
 *
 * Read data from the particular CAN register
 * Return: value read from the CAN register
 */
static void ftcan_clr_reg_bits(const struct ftcan_priv *priv,
			       enum ftcan_reg reg, u32 bs)
{
	u32 val = readl(priv->reg_base + reg);

	val &= ~bs;
	writel(val, priv->reg_base + reg);
}

/**
 * ftcan_set_bittiming - CAN set bit timing routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver set bittiming  routine.
 * Return: 0 on success and failure value on error
 */
static int ftcan_set_bittiming(struct net_device *ndev)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;
	u32 btr;
	u32 is_config_mode;

	/* Check whether Phytium CAN is in configuration mode.
	 * It cannot set bit timing if Phytium CAN is not in configuration mode.
	 */
	is_config_mode = (priv->read_reg(priv, FTCAN_CTRL_OFFSET) &
			  FTCAN_CTRL_XFER_MASK);
	if (is_config_mode) {
		netdev_alert(ndev,
		     "BUG! Cannot set bittiming - CAN is not in config mode\n");
		return -EPERM;
	}

	/* Setting Baud Rate prescalar value in BRPR Register */
	btr = (bt->brp - 1) << 16;

	/* Setting Time Segment 1 in BTR Register */
	btr |= (bt->prop_seg - 1) << 2;

	btr |= (bt->phase_seg1 - 1) << 5;

	/* Setting Time Segment 2 in BTR Register */
	btr |= (bt->phase_seg2 - 1) << 8;

	/* Setting Synchronous jump width in BTR Register */
	btr |= (bt->sjw - 1);

	priv->write_reg(priv, FTCAN_DAT_RATE_CTRL_OFFSET, btr);
	priv->write_reg(priv, FTCAN_ARB_RATE_CTRL_OFFSET, btr);

	netdev_dbg(ndev, "DAT=0x%08x, ARB=0x%08x\n",
		   priv->read_reg(priv, FTCAN_DAT_RATE_CTRL_OFFSET),
		   priv->read_reg(priv, FTCAN_ARB_RATE_CTRL_OFFSET));

	return 0;
}

/**
 * ftcan_chip_start - This the drivers start routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the drivers start routine.
 * Based on the State of the CAN device it puts
 * the CAN device into a proper mode.
 *
 * Return: 0 on success and failure value on error
 */
static int ftcan_chip_start(struct net_device *ndev)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	int err;

	err = ftcan_set_bittiming(ndev);
	if (err < 0)
		return err;

	/* Identifier mask enable */
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_AIME_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID0_MASK_OFFSET, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID1_MASK_OFFSET, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID2_MASK_OFFSET, FTCAN_ACC_IDN_MASK);
	priv->write_reg(priv, FTCAN_ACC_ID3_MASK_OFFSET, FTCAN_ACC_IDN_MASK);

	/* Enable interrupts */
	priv->write_reg(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_EN);

	/*Enable Transfer*/
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);

	netdev_dbg(ndev, "status:#x%08x\n",
		   priv->read_reg(priv, FTCAN_XFER_STS_OFFSET));

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

/**
 * ftcan_do_set_mode - This sets the mode of the driver
 * @ndev:	Pointer to net_device structure
 * @mode:	Tells the mode of the driver
 *
 * This check the drivers state and calls the
 * the corresponding modes to set.
 *
 * Return: 0 on success and failure value on error
 */
static int ftcan_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		ret = ftcan_chip_start(ndev);
		if (ret < 0) {
			netdev_err(ndev, "xcan_chip_start failed!\n");
			return ret;
		}
		netif_wake_queue(ndev);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

/**
 * ftcan_start_xmit - Starts the transmission
 * @skb:	sk_buff pointer that contains data to be Txed
 * @ndev:	Pointer to net_device structure
 *
 * This function is invoked from upper layers to initiate transmission. This
 * function uses the next available free txbuff and populates their fields to
 * start the transmission.
 *
 * Return: 0 on success and failure value on error
 */
static int ftcan_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf = (struct can_frame *)skb->data;
	u32 id, dlc, frame_head[2] = {0, 0},  data[8] = {0, 0};
	u32 tx_fifo_cnt;
	unsigned long flags;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	/* Check if the TX buffer is full */
	tx_fifo_cnt = (priv->read_reg(priv, FTCAN_FIFO_CNT_OFFSET) >> FTCAN_FIFO_CNT_TFN_SHIFT);
	if (tx_fifo_cnt == priv->tx_max) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "BUG!, TX FIFO full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}

	if(priv->tx_head == priv->tx_tail){
		priv->tx_head = priv->tx_tail = 0;
	}

	/* Watch carefully on the bit sequence */
	if (cf->can_id & CAN_EFF_FLAG) {
		/* Extended CAN ID format */
		id = ((cf->can_id & CAN_EFF_MASK) << FTCAN_IDR_ID2_SHIFT) &
			FTCAN_IDR_ID2_MASK;
		id |= (((cf->can_id & CAN_EFF_MASK) >>
			(CAN_EFF_ID_BITS-CAN_SFF_ID_BITS)) <<
			FTCAN_IDR_ID1_SHIFT) & FTCAN_IDR_ID1_MASK;

		/* The substibute remote TX request bit should be "1"
		 * for extended frames as in the Xilinx CAN datasheet
		 */
		id |= FTCAN_IDR_IDE_MASK | FTCAN_IDR_SRR_MASK;

		if (cf->can_id & CAN_RTR_FLAG)
			/* Extended frames remote TX request */
			id |= FTCAN_IDR_RTR_MASK;

		dlc = cf->can_dlc << FTCAN_IDR_EDLC_SHIFT;

		frame_head[0] = cpu_to_be32p(&id);//id;
		frame_head[1] = cpu_to_be32p(&dlc);//dlc;

		/* Write the Frame to Phytium CAN TX FIFO */
		priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, frame_head[0]);
		priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, frame_head[1]);
	} else {
		/* Standard CAN ID format */
		id = ((cf->can_id & CAN_SFF_MASK) << FTCAN_IDR_ID1_SHIFT) &
		     FTCAN_IDR_ID1_MASK;

		if (cf->can_id & CAN_RTR_FLAG)
			/* Standard frames remote TX request */
			id |= FTCAN_IDR_SRR_MASK;

		dlc = ((cf->can_dlc << FTCAN_IDR_SDLC_SHIFT) | FTCAN_IDR_PAD_MASK);
		id |= dlc;

		frame_head[0] =  cpu_to_be32p(&id);

		/* Write the Frame to Xilinx CAN TX FIFO */
		priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, frame_head[0]);
	}

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		if (cf->can_dlc > 0) {
			data[0] = (*(__be32*)(cf->data + 0));
			priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, data[0]);
		}
		if (cf->can_dlc > 4) {
			data[1] = (*(__be32*)(cf->data + 4));
			priv->write_reg(priv, FTCAN_TX_FIFO_OFFSET, data[1]);
		}
		stats->tx_bytes += cf->can_dlc;
	}

	can_put_echo_skb(skb, ndev, priv->tx_head % priv->tx_max);
	priv->tx_head++;

	/* triggers tranmission */
	spin_lock_irqsave(&priv->lock, flags);
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_TXREQ_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_TXREQ_MASK|FTCAN_CTRL_XFER_MASK);
	spin_unlock_irqrestore(&priv->lock, flags);

	/* Check if the TX buffer is full */
	if ((priv->tx_head - priv->tx_tail) == priv->tx_max)
		netif_stop_queue(ndev);

	return NETDEV_TX_OK;
}

/**
 * ftcan_rx -  Is called from CAN isr to complete the received
 *		frame  processing
 * @ndev:	Pointer to net_device structure
 *
 * This function is invoked from the CAN isr(poll) to process the Rx frames. It
 * does minimal processing and invokes "netif_receive_skb" to complete further
 * processing.
 * Return: 1 on success and 0 on failure.
 */
static int ftcan_rx(struct net_device *ndev)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32 id_ftcan, dlc, data[2] = {0, 0};

	skb = alloc_can_skb(ndev, &cf);
	if (unlikely(!skb)) {
		stats->rx_dropped++;
		return 0;
	}

	/* Read a frame from Phytium CAN */
	id_ftcan = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);
	id_ftcan = be32_to_cpup(&id_ftcan);

	/* Change Phytium CAN ID format to socketCAN ID format */
	if (id_ftcan & FTCAN_IDR_IDE_MASK) {
		/* The received frame is an Extended format frame */
		dlc = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);

		cf->can_id = (id_ftcan & FTCAN_IDR_ID1_MASK) >> 3;
		cf->can_id |= (id_ftcan & FTCAN_IDR_ID2_MASK) >> FTCAN_IDR_ID2_SHIFT;
		cf->can_id |= CAN_EFF_FLAG;
		if (id_ftcan & FTCAN_IDR_RTR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	} else {
		dlc = ((id_ftcan & FTCAN_IDR_DLC_MASK ) >> FTCAN_IDR_SDLC_SHIFT);

		/* The received frame is a standard format frame */
		cf->can_id = (id_ftcan & FTCAN_IDR_ID1_MASK) >> FTCAN_IDR_ID1_SHIFT;
		if (id_ftcan & FTCAN_IDR_SRR_MASK)
			cf->can_id |= CAN_RTR_FLAG;
	}

	/* Change Phytium CAN data length format to socketCAN data format */
	cf->can_dlc = get_can_dlc(dlc);

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		/* Change Phytium CAN data format to socketCAN data format */
		if (cf->can_dlc > 0){
			data[0] = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);
			*(__be32 *)(cf->data) = (data[0]);
		}

		if (cf->can_dlc > 4){
			data[1] = priv->read_reg(priv, FTCAN_RX_FIFO_OFFSET);
			*(__be32 *)(cf->data + 4) = (data[1]);
		}

	}

	stats->rx_bytes += cf->can_dlc;
	stats->rx_packets++;
	netif_receive_skb(skb);

	return 1;
}

/**
 * ftcan_err_interrupt - error frame Isr
 * @ndev:	net_device pointer
 * @isr:	interrupt status register value
 *
 * This is the CAN error interrupt and it will
 * check the the type of error and forward the error
 * frame to upper layers.
 */
static void ftcan_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32  txerr = 0, rxerr = 0;

	skb = alloc_can_err_skb(ndev, &cf);

	rxerr = priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) & FTCAN_ERR_CNT_RFN_MASK;
	txerr = ((priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) &
		FTCAN_ERR_CNT_TFN_MASK) >> FTCAN_ERR_CNT_TFN_SHIFT);

	if (isr & FTCAN_INTR_BOIS_MASK) {
		priv->can.state = CAN_STATE_BUS_OFF;
		priv->can.can_stats.bus_off++;
		/* Leave device in Config Mode in bus-off state */
		can_bus_off(ndev);
		if (skb)
			cf->can_id |= CAN_ERR_BUSOFF;
	} else if ((isr & FTCAN_INTR_PEIS_MASK) == FTCAN_INTR_PEIS_MASK) {
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		priv->can.can_stats.error_passive++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (rxerr > 127) ?
					CAN_ERR_CRTL_RX_PASSIVE :
					CAN_ERR_CRTL_TX_PASSIVE;
			cf->data[6] = txerr;
			cf->data[7] = rxerr;
		}
	} else if (isr & FTCAN_INTR_PWIS_MASK) {
		priv->can.state = CAN_STATE_ERROR_WARNING;
		priv->can.can_stats.error_warning++;
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= (txerr > rxerr) ?
					CAN_ERR_CRTL_TX_WARNING :
					CAN_ERR_CRTL_RX_WARNING;
			cf->data[6] = txerr;
			cf->data[7] = rxerr;
		}
	}

	/* Check for RX FIFO Overflow interrupt */
	if (isr & FTCAN_INTR_RFIS_MASK) {
		stats->rx_over_errors++;
		stats->rx_errors++;

		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
		}
	}

	if (skb) {
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	}

	netdev_dbg(ndev, "%s: error status register:0x%x\n",
			__func__, (priv->read_reg(priv, FTCAN_INTR_OFFSET) & FTCAN_INTR_STATUS_MASK));
}

/**
 * ftcan_rx_poll - Poll routine for rx packets (NAPI)
 * @napi:	napi structure pointer
 * @quota:	Max number of rx packets to be processed.
 *
 * This is the poll routine for rx part.
 * It will process the packets maximux quota value.
 *
 * Return: number of packets received
 */

static void  ftcan_rx_poll(unsigned long data)
{
	struct ftcan_tasklet *t = (struct ftcan_tasklet *)data;
	struct net_device *ndev = t->ndev;
	int work_done = 0;

	struct tasklet_struct *pcan_task = NULL;

	pcan_task = t->done_task;

	if (t)
		kfree(t);

	if (ndev) {
		work_done += ftcan_rx(ndev);

		if (work_done)
			can_led_event(ndev, CAN_LED_EVENT_RX);
	}

	if (pcan_task) {
		kfree(pcan_task);
		pcan_task = NULL;
	}

	return ;
}

/**
 * ftcan_tx_interrupt - Tx Done Isr
 * @ndev:	net_device pointer
 * @isr:	Interrupt status register value
 */
static void ftcan_tx_interrupt(struct net_device *ndev, u32 isr)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	while ((priv->tx_head - priv->tx_tail > 0) &&
		(isr & FTCAN_INTR_TEIS_MASK)) {
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET,
				   FTCAN_INTR_TEIC_MASK|FTCAN_INTR_REIC_MASK);
		can_get_echo_skb(ndev, priv->tx_tail % priv->tx_max);
		priv->tx_tail++;
		stats->tx_packets++;
		isr = (priv->read_reg(priv, FTCAN_INTR_OFFSET) &
		       FTCAN_INTR_STATUS_MASK);
	}

	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_TXREQ_MASK);
	priv->set_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);

	can_led_event(ndev, CAN_LED_EVENT_TX);
	netif_wake_queue(ndev);
}

/**
 * ftcan_interrupt - CAN Isr
 * @irq:	irq number
 * @dev_id:	device id poniter
 *
 * This is the xilinx CAN Isr. It checks for the type of interrupt
 * and invokes the corresponding ISR.
 *
 * Return:
 * IRQ_NONE - If CAN device is in sleep mode, IRQ_HANDLED otherwise
 */
static irqreturn_t ftcan_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct ftcan_priv *priv = netdev_priv(ndev);
	u32 isr;
	struct ftcan_tasklet *pcan_tasklet_priv = NULL;
	struct tasklet_struct *pcan_tasklet = NULL;

	/* Get the interrupt status from Phytium CAN */
	isr = (priv->read_reg(priv, FTCAN_INTR_OFFSET) & FTCAN_INTR_STATUS_MASK);
	if (!isr)
		return IRQ_NONE;

	/* Check for Tx interrupt and Processing it */
	if ((isr & FTCAN_INTR_TEIS_MASK)) {
		isr &= (~FTCAN_INTR_REIS_MASK);
		ftcan_tx_interrupt(ndev, isr);
	}

	/* Check for the type of error interrupt and Processing it */
	if (isr & (FTCAN_INTR_EIS_MASK | FTCAN_INTR_RFIS_MASK |
			FTCAN_INTR_BOIS_MASK | FTCAN_INTR_PEIS_MASK)) {
		priv->clr_reg_bits(priv, FTCAN_INTR_OFFSET,
				   (FTCAN_INTR_EIC_MASK | FTCAN_INTR_RFIC_MASK |
				    FTCAN_INTR_BOIC_MASK | FTCAN_INTR_PEIC_MASK));
		ftcan_err_interrupt(ndev, isr);
	}

	/* Check for the type of receive interrupt and Processing it */
	if (isr & (FTCAN_INTR_REIS_MASK)) {
		priv->isr = (isr & FTCAN_INTR_REIS_MASK);
		priv->set_reg_bits(priv, FTCAN_INTR_OFFSET, FTCAN_INTR_REIC_MASK);

		pcan_tasklet_priv = (struct ftcan_tasklet *)kzalloc(sizeof(struct ftcan_tasklet), GFP_ATOMIC);
		pcan_tasklet = (struct tasklet_struct *)kzalloc(sizeof(struct tasklet_struct), GFP_ATOMIC);
		if (pcan_tasklet_priv && pcan_tasklet) {
			pcan_tasklet_priv->ndev = ndev;
			pcan_tasklet_priv->done_task = pcan_tasklet;
			tasklet_init(pcan_tasklet, ftcan_rx_poll, (unsigned long)pcan_tasklet_priv);
			tasklet_schedule(pcan_tasklet);
		} else {
			if(pcan_tasklet)
				kfree(pcan_tasklet);
			if(pcan_tasklet_priv)
				kfree(pcan_tasklet_priv);
		}
	}

	return IRQ_HANDLED;
}

/**
 * ftcan_chip_stop - Driver stop routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the drivers stop routine. It will disable the
 * interrupts and put the device into configuration mode.
 */
static void ftcan_chip_stop(struct net_device *ndev)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	u32 ier;

	/* Disable interrupts and leave the can in configuration mode */
	ier = (FTCAN_INTR_DIS & FTCAN_INTR_EN_MASK);
	priv->clr_reg_bits(priv, FTCAN_INTR_OFFSET, ier);

	/*Disable Transfer*/
	priv->clr_reg_bits(priv, FTCAN_CTRL_OFFSET, FTCAN_CTRL_XFER_MASK);
	priv->can.state = CAN_STATE_STOPPED;
}

/**
 * ftcan_open - Driver open routine
 * @ndev:	Pointer to net_device structure
 *
 * This is the driver open routine.
 * Return: 0 on success and failure value on error
 */
static int ftcan_open(struct net_device *ndev)
{
	struct ftcan_priv *priv = netdev_priv(ndev);
	int ret;

	ret = request_irq(ndev->irq, ftcan_interrupt, priv->irq_flags,
			  ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "irq allocation for CAN failed\n");
		goto err;
	}

	/* Common open */
	ret = open_candev(ndev);
	if (ret)
		goto err_irq;

	ret = ftcan_chip_start(ndev);
	if (ret < 0) {
		netdev_err(ndev, "ftcan_chip_start failed!\n");
		goto err_candev;
	}

	can_led_event(ndev, CAN_LED_EVENT_OPEN);

	netif_start_queue(ndev);

	return 0;

err_candev:
	close_candev(ndev);
err_irq:
	free_irq(ndev->irq, ndev);
err:
	return ret;
}

/**
 * ftcan_close - Driver close routine
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0 always
 */
static int ftcan_close(struct net_device *ndev)
{
	netif_stop_queue(ndev);
	ftcan_chip_stop(ndev);
	free_irq(ndev->irq, ndev);
	close_candev(ndev);

	can_led_event(ndev, CAN_LED_EVENT_STOP);

	return 0;
}

/**
 * ftcan_get_berr_counter - error counter routine
 * @ndev:	Pointer to net_device structure
 * @bec:	Pointer to can_berr_counter structure
 *
 * This is the driver error counter routine.
 * Return: 0 on success and failure value on error
 */
static int ftcan_get_berr_counter(const struct net_device *ndev,
					struct can_berr_counter *bec)
{
	struct ftcan_priv *priv = netdev_priv(ndev);

	bec->rxerr = priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) & FTCAN_ERR_CNT_RFN_MASK;
	bec->txerr = ((priv->read_reg(priv, FTCAN_ERR_CNT_OFFSET) &
			FTCAN_ERR_CNT_TFN_MASK) >> FTCAN_ERR_CNT_TFN_SHIFT);

	return 0;
}


static const struct net_device_ops ftcan_netdev_ops = {
	.ndo_open	= ftcan_open,
	.ndo_stop	= ftcan_close,
	.ndo_start_xmit	= ftcan_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};


#define ftcan_dev_pm_ops NULL

/**
 * ftcan_probe - Platform registration call
 * @pdev:	Handle to the platform device structure
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * Return: 0 on success and failure value on error
 */
static int ftcan_probe(struct platform_device *pdev)
{
	struct resource *res; /* IO mem resources */
	struct net_device *ndev;
	struct ftcan_priv *priv;
	void __iomem *addr;
	int ret, rx_max, tx_max;

	/* Get the virtual base address for the device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr)) {
		ret = PTR_ERR(addr);
		goto err;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth", &tx_max);
	if (ret < 0)
		goto err;

	ret = of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth", &rx_max);
	if (ret < 0)
		goto err;

	/* Create a CAN device instance */
	ndev = alloc_candev(sizeof(struct ftcan_priv), tx_max);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->dev = &pdev->dev;
	priv->can.bittiming_const = &ftcan_bittiming_const;
	priv->can.do_set_mode = ftcan_do_set_mode;
	priv->can.do_get_berr_counter = ftcan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING;
	priv->reg_base = addr;
	priv->tx_max = tx_max;
	priv->tx_head = 0;
	priv->tx_tail = 0;

	/* Get IRQ for the device */
	ndev->irq = platform_get_irq(pdev, 0);
	ndev->flags |= IFF_ECHO;	/* We support local echo */
	priv->irq_flags = IRQF_SHARED;

	spin_lock_init(&priv->lock);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &ftcan_netdev_ops;

	/* Getting the CAN can_clk info */
	priv->can_clk = devm_clk_get(&pdev->dev, "phytium_can_clk");
	if (IS_ERR(priv->can_clk)) {
		dev_err(&pdev->dev, "Device clock not found.\n");
		ret = PTR_ERR(priv->can_clk);
		goto err_free;
	}

	priv->can.clock.freq = clk_get_rate(priv->can_clk);

	ret = clk_prepare_enable(priv->can_clk);
	if (ret)
		return ret;

	priv->write_reg = ftcan_write_reg;
	priv->read_reg = ftcan_read_reg;
	priv->set_reg_bits = ftcan_set_reg_bits;
	priv->clr_reg_bits = ftcan_clr_reg_bits;

	ret = register_candev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "fail to register failed (err=%d)\n", ret);
		goto err_disableclks;
	}
	devm_can_led_init(ndev);
	netdev_dbg(ndev, "reg_base=0x%p irq=%d clock=%d, tx fifo depth:%d\n",
			priv->reg_base, ndev->irq, priv->can.clock.freq,
			priv->tx_max);

	return 0;

err_disableclks:

err_free:
	free_candev(ndev);
err:
	return ret;
}

/**
 * ftcan_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
static int ftcan_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	unregister_candev(ndev);
	free_candev(ndev);
	return 0;
}

/* Match table for OF platform binding */
static const struct of_device_id ftcan_of_match[] = {
	{ .compatible = "phytium,can", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, ftcan_of_match);

static struct platform_driver ftcan_driver = {
	.probe = ftcan_probe,
	.remove	= ftcan_remove,
	.driver	= {
		.name = "phytium-can",
		.pm = ftcan_dev_pm_ops,
		.of_match_table	= ftcan_of_match,
	},
};

module_platform_driver(ftcan_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leo Hou <houyuefei@phytium.com.cn>");
MODULE_AUTHOR("Chen Baozi <chenbaozi@phytium.com.cn>");
MODULE_DESCRIPTION("Phytium CAN Controller");
