/*
 * SPI ethernet driver
 * Copyright (C) 2017 Bertold Van den Bergh <vandenbergh@bertold.org>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/hrtimer.h>
#include <linux/crc32.h>

/*
 * The normal preamble is 8 bytes long, but if you use a capacitor to drive
 * the line a longer preamble may be required to ensure no packets are lost.
 */
static unsigned int spi10m_preamble_length = 8;
module_param(spi10m_preamble_length, int, 0444);
MODULE_PARM_DESC(spi10m_preamble_length, "Number of preamble bytes");

#define SPI10M_MIN_LEN 60
#define SPI10M_TXBUFFER_MAX_PACKET 1500

struct spi10m_dev {
	struct net_device *netdev;
	struct spi_device *spi;

	uint8_t buf[SPI10M_TXBUFFER_MAX_PACKET + 4];

	unsigned long tx_buffer_size;
	uint16_t *tx_buffer;
	dma_addr_t tx_buffer_dma;

	unsigned long spi_speed;

	uint64_t tx_packets;
	uint64_t tx_bytes;

	struct hrtimer nlp_timer;
	ktime_t nlp_period;
	uint8_t nlp_byte[1];

	struct work_struct tx_work;

	struct sk_buff *tx_skb;
	spinlock_t tx_skb_lock;

	uint16_t manchester_table[256];
};

static void spi10m_get_ethtool_stats(struct net_device *dev,
			struct ethtool_stats *stats, uint64_t *data)
{
	struct spi10m_dev *priv = netdev_priv(dev);
	unsigned long flags;

	memset(data, 0, 12 * sizeof(uint64_t));

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	data[0] = le64_to_cpu(priv->tx_packets);
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);
}


static const struct ethtool_ops spi10m_ethtool_ops = {
	.get_ethtool_stats = spi10m_get_ethtool_stats
};

static int spi10m_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);
	return 0;
}

static enum hrtimer_restart spi10m_send_nlp(struct hrtimer *timer)
{
	struct spi10m_dev *priv =
			container_of(timer, struct spi10m_dev, nlp_timer);
	unsigned long flags;
	bool need_link_pulse = false;

	hrtimer_forward(&priv->nlp_timer, ktime_get(), priv->nlp_period);

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	if (!priv->tx_skb)
		need_link_pulse = true;
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

	if (need_link_pulse)
		schedule_work(&priv->tx_work);

	return HRTIMER_RESTART;
}

static netdev_tx_t spi10m_send_packet(struct sk_buff *skb,
				      struct net_device *dev)
{
	struct spi10m_dev *priv = netdev_priv(dev);
	unsigned long flags;

	netif_stop_queue(dev);

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	priv->tx_skb = skb;
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;
}


static struct rtnl_link_stats64 *spi10m_get_stats64
		(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct spi10m_dev *priv = netdev_priv(dev);
	unsigned long flags;

	memset(stats, 0, sizeof(*stats));

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	stats->tx_packets = priv->tx_packets;
	stats->tx_bytes = priv->tx_bytes;
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

	return stats;
}


static const struct net_device_ops spi10m_netdev_ops = {
	.ndo_start_xmit		= spi10m_send_packet,
	.ndo_set_mac_address	= spi10m_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= spi10m_get_stats64
};

static void spi10m_tx_work_handler(struct work_struct *work)
{
	struct spi10m_dev *priv = container_of(work, struct spi10m_dev,
						tx_work);

	unsigned int i;
	unsigned long flags;

	struct sk_buff *skb;

	unsigned int len;
	uint16_t *tx_packet = priv->tx_buffer;

	uint32_t crc_add;
	uint16_t tmp;

	struct spi_message message = {};
	struct spi_transfer transfer = {};

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	skb = priv->tx_skb;
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

	/* If no SKB, send link pulse */
	if (!skb) {
		spi_message_init(&message);
		transfer.tx_buf = priv->nlp_byte;
		transfer.len = 1;
		spi_message_add_tail(&transfer, &message);
		transfer.speed_hz = priv->spi_speed;

		spi_sync(priv->spi, &message);
	} else{
		len = skb->len;

		if (len < SPI10M_TXBUFFER_MAX_PACKET) {
			/* Copy packet to local buffer */
			skb_copy_bits(skb, 0, priv->buf, len);
			dev_kfree_skb(skb);

			/* Skip preamble */
			tx_packet += spi10m_preamble_length;

			/* Add padding if needed */
			if (len < SPI10M_MIN_LEN) {
				memset(priv->buf + len,
				       0,
				       SPI10M_MIN_LEN - len);
				len = SPI10M_MIN_LEN;
			}

			/* Add checksum */
			crc_add = ~le32_to_cpu(ether_crc_le(len, priv->buf));
			memcpy(priv->buf + len, &crc_add, 4);
			len += 4;

			/* Convert the packet to manchester encoding */
			for (i = 0; i < len; i++) {
				tmp = priv->manchester_table[priv->buf[i]];
				tx_packet[i] = be16_to_cpu(tmp);
			}
			tx_packet += len;

			/* Encode TP_IDL */
			if (tmp & 1)
				tx_packet[0] = be16_to_cpu(0xFC00); /* 300ns */
			else
				tx_packet[0] = be16_to_cpu(0xFE00); /* 350ns */
			tx_packet++;

			/* Add interframe gap */
			for (i = 0; i < 12; i++)
				tx_packet[i] = 0;

			/* Send it over SPI */
			spi_message_init(&message);
			transfer.tx_buf = priv->tx_buffer;
			transfer.tx_dma = priv->tx_buffer_dma;

			transfer.rx_buf = NULL;
			transfer.rx_dma = 0;

			transfer.len = 2 * (spi10m_preamble_length + len +
					    12 + 1);

			spi_message_add_tail(&transfer, &message);

			message.is_dma_mapped = 1;

			transfer.speed_hz = priv->spi_speed;

			spi_sync(priv->spi, &message);
		} else
			dev_kfree_skb(skb);

		spin_lock_irqsave(&priv->tx_skb_lock, flags);
		priv->tx_skb = NULL;
		priv->tx_packets++;
		priv->tx_bytes += len;
		spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

		netif_wake_queue(priv->netdev);
	}
}

static uint16_t spi10m_calculate_manchester(uint8_t index)
{
	uint16_t out = 0;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		out <<= 2;
		if (index & (1<<i))
			out |= 0x1; /* Low, then high */
		else
			out |= 0x2; /* High, then low */
	}
	return out;
}

static int spi10m_probe(struct spi_device *spi)
{
	int ret = 0, i;

	struct spi10m_dev *dev;
	struct net_device *netdev;

	netdev = alloc_etherdev(sizeof(struct spi10m_dev));
	if (!netdev) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "Failed to allocate device\n");
		goto err_allocdev;
	}

	dev = netdev_priv(netdev);
	dev->netdev = netdev;

	spi_set_drvdata(spi, dev);
	SET_NETDEV_DEV(netdev, &spi->dev);

	dma_set_coherent_mask(&spi->dev, DMA_BIT_MASK(32));

	if (spi10m_preamble_length <= 0)
		spi10m_preamble_length = 1;

	spin_lock_init(&dev->tx_skb_lock);

	dev->tx_buffer_size = 2 * (spi10m_preamble_length +
				   SPI10M_TXBUFFER_MAX_PACKET + 4 + 1 + 12);
	dev->tx_buffer = dma_alloc_coherent(&spi->dev,
					    dev->tx_buffer_size,
					    &dev->tx_buffer_dma,
					    GFP_KERNEL);
	if (!dev->tx_buffer) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "TX Buffer allocation failed\n");
		goto err_allocbuf;
	}

	/* Initialize table */
	for (i = 0; i < 256; i++)
		dev->manchester_table[i] = spi10m_calculate_manchester(i);

	/* Add the preamble and SFD */
	for (i = 0; i < (spi10m_preamble_length - 1); i++)
		dev->tx_buffer[i] = be16_to_cpu(dev->manchester_table[0x55]);

	dev->tx_buffer[spi10m_preamble_length - 1] =
				be16_to_cpu(dev->manchester_table[0xD5]);

	/* This is a transmit only adapter so random is fine */
	eth_hw_addr_random(netdev);

	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = &spi10m_netdev_ops;
	netdev->watchdog_timeo = 10 * HZ; /* 10 seconds */
	netdev->ethtool_ops = &spi10m_ethtool_ops;

	/* Setup SPI to 20MHz to create a 10MHz manchester signal */
	dev->spi_speed = 20000000;
	dev->spi = spi;

	INIT_WORK(&dev->tx_work, spi10m_tx_work_handler);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(&spi->dev, "Netdev registration failed: %d\n", ret);

		goto err_register;
	}

	dev->nlp_period = ktime_set(0, 16000000); /* 16ms */
	hrtimer_init(&dev->nlp_timer, CLOCK_REALTIME, HRTIMER_MODE_REL);
	dev->nlp_timer.function = spi10m_send_nlp;
	dev->nlp_byte[0] = 0x3;
	hrtimer_start(&dev->nlp_timer, dev->nlp_period, HRTIMER_MODE_REL);

	netif_start_queue(netdev);

	return 0;

err_register:
	dma_free_coherent(&spi->dev,
			  dev->tx_buffer_size,
			  dev->tx_buffer,
			  dev->tx_buffer_dma);
err_allocbuf:
	free_netdev(dev->netdev);
err_allocdev:
	return ret;
}

static int spi10m_remove(struct spi_device *spi)
{
	struct spi10m_dev *dev = spi_get_drvdata(spi);

	hrtimer_cancel(&dev->nlp_timer);

	unregister_netdev(dev->netdev);
	free_netdev(dev->netdev);

	dma_free_coherent(&spi->dev,
			  dev->tx_buffer_size,
			  dev->tx_buffer,
			  dev->tx_buffer_dma);

	return 0;
}


static const struct of_device_id spi10m_of_match[] = {
	{ .compatible = "bertold,spi10m", },
	{ },
};
MODULE_DEVICE_TABLE(of, spi10m_of_match);

static struct spi_driver spi10m_driver = {
	.driver = {
		.name	= "spi10m",
		.owner	= THIS_MODULE,
	},
	.probe		= spi10m_probe,
	.remove		= spi10m_remove,
};
module_spi_driver(spi10m_driver);

MODULE_AUTHOR("Bertold Van den Bergh (vandenbergh@bertold.org)");
MODULE_DESCRIPTION("SPI10M");
MODULE_LICENSE("GPL");
