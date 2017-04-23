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
#include <linux/if_vlan.h>

#define SPI10M_FOOTER_LEN (1 + 12)

struct spi10m_dev {
	struct net_device *netdev;
	struct spi_device *spi;

	unsigned long tx_buffer_size;
	unsigned long tx_buffer_max_packet;
	uint16_t *tx_buffer;
	struct mutex tx_buffer_mutex;

	unsigned long check_buffer_size;
	uint16_t *check_buffer;

	uint64_t tx_packets;
	uint64_t tx_bytes;
	uint64_t tx_errors;
	uint64_t tx_nlp_sent;

	struct hrtimer flp_timer;
	ktime_t flp_period;
	uint8_t *flp_buffer;

	struct work_struct tx_work;

	struct sk_buff *tx_skb;
	spinlock_t tx_skb_lock;

	uint16_t manchester_table[256];

	bool closing;

	unsigned int spi_clock;
	unsigned int preamble_length;
	bool spi_readback;
	bool use_nlp;
};

static int spi10m_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, address->sa_data, dev->addr_len);

	return 0;
}

static enum hrtimer_restart spi10m_send_flp(struct hrtimer *timer)
{
	struct spi10m_dev *priv =
			container_of(timer, struct spi10m_dev, flp_timer);
	unsigned long flags;
	bool need_link_pulse = false;

	hrtimer_forward(&priv->flp_timer, ktime_get(), priv->flp_period);

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	if (!priv->tx_skb && !priv->closing)
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
	bool closing;

	netif_stop_queue(dev);

	spin_lock_irqsave(&priv->tx_skb_lock, flags);
	closing = priv->closing;
	priv->tx_skb = skb;
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

	if (closing)
		dev_kfree_skb(skb);
	else
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
	stats->tx_errors = priv->tx_errors;

	/* Put the number of link pulse transmissions in this field */
	stats->collisions = priv->tx_nlp_sent;
	spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

	return stats;
}

static void spi10m_free_dma_buffer(struct spi10m_dev *dev, int buffer)
{
	if (buffer == 0 && dev->tx_buffer) {
		kfree(dev->tx_buffer);

		dev->tx_buffer = NULL;
	}

	if (buffer == 1 && dev->check_buffer) {
		kfree(dev->check_buffer);

		dev->check_buffer = NULL;
	}

	if (buffer == 2 && dev->flp_buffer) {
		kfree(dev->flp_buffer);

		dev->flp_buffer = NULL;
	}
}

static bool spi10m_alloc_ptx_buffer(struct spi10m_dev *dev,
				    unsigned long desired_max_packet)
{
	bool ret = false;
	unsigned int i;

	unsigned long new_buffer_size = 2 * (dev->preamble_length +
					     desired_max_packet +
					     ETH_FCS_LEN + SPI10M_FOOTER_LEN);
	uint16_t *new_buffer_tx;

	if (dev->tx_buffer && (new_buffer_size == dev->tx_buffer_size))
		return true;

	mutex_lock(&dev->tx_buffer_mutex);

	new_buffer_tx = kmalloc(new_buffer_size, GFP_KERNEL | GFP_DMA);

	if (new_buffer_tx) {
		/* Deallocate current buffer, if there is one */
		spi10m_free_dma_buffer(dev, 0);

		dev->tx_buffer = new_buffer_tx;
		dev->tx_buffer_size = new_buffer_size;
		dev->tx_buffer_max_packet = desired_max_packet;

		/* Add the preamble and SFD */
		for (i = 0; i < (dev->preamble_length - 1); i++)
			new_buffer_tx[i] =
				be16_to_cpu(dev->manchester_table[0x55]);

		new_buffer_tx[dev->preamble_length - 1] =
				be16_to_cpu(dev->manchester_table[0xD5]);

		ret = true;
	}

	if (dev->spi_readback) {
		spi10m_free_dma_buffer(dev, 1);

		dev->check_buffer = kmalloc(new_buffer_size, GFP_KERNEL |
							     GFP_DMA);
		dev->check_buffer_size = new_buffer_size;

		if (!dev->check_buffer)
			dev_warn(&dev->netdev->dev,
				"Could not allocate verification buffer");
	}

	mutex_unlock(&dev->tx_buffer_mutex);

	return ret;
}

static int spi10m_change_mtu(struct net_device *dev, int new_mtu)
{
	struct spi10m_dev *priv = netdev_priv(dev);

#if IS_ENABLED(CONFIG_VLAN_8021Q)
	int vlan_bytes = VLAN_HLEN;
#else
	int vlan_bytes = 0;
#endif

	unsigned long desired_max_packet = vlan_bytes + ETH_HLEN +
					   ETH_FCS_LEN + new_mtu;

	if (spi10m_alloc_ptx_buffer(priv, desired_max_packet)) {
		dev->mtu = new_mtu;

		return 0;
	}

	return -ENOMEM;
}

static const struct net_device_ops spi10m_netdev_ops = {
	.ndo_start_xmit		= spi10m_send_packet,
	.ndo_change_mtu		= spi10m_change_mtu,
	.ndo_set_mac_address	= spi10m_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= spi10m_get_stats64
};

static void spi10m_tx_work_handler(struct work_struct *work)
{
	struct spi10m_dev *priv = container_of(work, struct spi10m_dev,
						tx_work);

	bool tx_ok = true;

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
		transfer.tx_buf = priv->flp_buffer;
		if (priv->use_nlp)
			transfer.len = 1;
		else
			transfer.len = 5009;
		transfer.speed_hz = priv->spi_clock;
		spi_message_add_tail(&transfer, &message);

		spi_sync(priv->spi, &message);

		spin_lock_irqsave(&priv->tx_skb_lock, flags);
		priv->tx_nlp_sent++;
		spin_unlock_irqrestore(&priv->tx_skb_lock, flags);
	} else{
		len = skb->len;

		/* Lock buffer mutex */
		mutex_lock(&priv->tx_buffer_mutex);

		if (len <= priv->tx_buffer_max_packet) {
			/* Skip preamble */
			tx_packet += priv->preamble_length;

			/* Manchester encoder packet */
			for (i = 0; i < len; i++) {
				tmp = priv->manchester_table[skb->data[i]];
				tx_packet[i] = be16_to_cpu(tmp);
			}

			/* Calculate CRC */
			crc_add = ether_crc_le(len, skb->data);

			/* Add padding if needed */
			if (len < ETH_ZLEN) {
				for (i = len; i < ETH_ZLEN; i++) {
					tmp = priv->manchester_table[0];
					tx_packet[i] = be16_to_cpu(tmp);
				}

				/* Extend CRC with padding */
				crc_add = crc32_le_combine(crc_add, 0,
							   ETH_ZLEN - len);

				len = ETH_ZLEN;
			}

			/* Manchester encode CRC */
			crc_add = ~le32_to_cpu(crc_add);
			for (i = 0; i < ETH_FCS_LEN; i++) {
				tmp = priv->manchester_table
						[((uint8_t *)&crc_add)[i]];
				tx_packet[len + i] = be16_to_cpu(tmp);
			}
			len += ETH_FCS_LEN;

			/* Encode TP_IDL */
			if (tmp & 1)
				tx_packet[len] = be16_to_cpu(0xFC00);
			else
				tx_packet[len] = be16_to_cpu(0xFE00);
			len++;

			/* Add interframe gap */
			for (i = 0; i < 12; i++)
				tx_packet[len + i] = 0;
			len += 12;

			/* Send it over SPI */
			spi_message_init(&message);
			transfer.tx_buf = priv->tx_buffer;
			transfer.rx_buf = priv->check_buffer;

			transfer.len = 2 * (priv->preamble_length + len);
			transfer.speed_hz = priv->spi_clock;
			spi_message_add_tail(&transfer, &message);

			spi_sync(priv->spi, &message);

			if (priv->check_buffer) {
				if (memcmp(priv->check_buffer +
					   2 * priv->preamble_length,
					   priv->tx_buffer +
					   2 * priv->preamble_length,
					   2 * len))
					tx_ok = false;
			}
		}

		spin_lock_irqsave(&priv->tx_skb_lock, flags);
		priv->tx_skb = NULL;
		priv->tx_packets++;
		priv->tx_bytes += skb->len;
		if (!tx_ok)
			priv->tx_errors++;

		spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

		dev_kfree_skb(skb);

		mutex_unlock(&priv->tx_buffer_mutex);

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

static void spi10m_encode_flp(uint8_t *flp, uint16_t data)
{
	unsigned int i, j, bit;
	unsigned int index = 0;

	for (i = 0; i < 16; i++) {
		if (data & (1 << i))
			bit = 0x3;
		else
			bit = 0;

		for (j = 0; j < 313; j++) {
			if (j == 0)
				flp[index] = 0x3;
			else if (j == 156)
				flp[index] = bit;
			else
				flp[index] = 0;

			index++;
		}
	}

	flp[index] = 0x3;
}

static int spi10m_probe(struct spi_device *spi)
{
	int ret = 0, i;

	struct spi10m_dev *dev;
	struct net_device *netdev;
	struct device_node *node = spi->dev.of_node;

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

	/* Read devicetree properties */
	dev->spi_readback = of_property_read_bool(node, "spi-readback");

	dev->use_nlp = of_property_read_bool(node, "use-nlp");

	ret = of_property_read_u32(node, "preamble-len", &dev->preamble_length);
	if (ret || dev->preamble_length <= 0)
		dev->preamble_length = 8;

	ret = of_property_read_u32(node, "spi-frequency", &dev->spi_clock);
	if (ret || dev->spi_clock <= 100000)
		dev->spi_clock = 20000000;

	spin_lock_init(&dev->tx_skb_lock);
	mutex_init(&dev->tx_buffer_mutex);

	/* Initialize table */
	for (i = 0; i < 256; i++)
		dev->manchester_table[i] = spi10m_calculate_manchester(i);

	ret = spi10m_change_mtu(netdev, netdev->mtu);
	if (ret) {
		dev_err(&spi->dev, "TX Buffer allocation failed\n");

		goto err_dealloc;
	}

	/* This is a transmit only adapter so random is fine */
	eth_hw_addr_random(netdev);

	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = &spi10m_netdev_ops;
	netdev->watchdog_timeo = HZ;

	dev->spi = spi;

	dev->flp_buffer = kmalloc(5009, GFP_KERNEL | GFP_DMA);
	if (!dev->flp_buffer) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "Could not allocate FLP buffer\n");

		goto err_dealloc;
	}

	spi10m_encode_flp(dev->flp_buffer, 0x4070);

	INIT_WORK(&dev->tx_work, spi10m_tx_work_handler);

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(&spi->dev, "Netdev registration failed: %d\n", ret);

		goto err_dealloc;
	}

	hrtimer_init(&dev->flp_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->flp_timer.function = spi10m_send_flp;
	dev->flp_period = ktime_set(0, 16000000); /* 16ms */
	hrtimer_start(&dev->flp_timer, dev->flp_period, HRTIMER_MODE_REL);

	netif_start_queue(netdev);

	return 0;

err_dealloc:
	spi10m_free_dma_buffer(dev, 0);
	spi10m_free_dma_buffer(dev, 1);
	spi10m_free_dma_buffer(dev, 2);

	free_netdev(dev->netdev);
err_allocdev:
	return ret;
}

static int spi10m_remove(struct spi_device *spi)
{
	struct spi10m_dev *dev = spi_get_drvdata(spi);

	dev->closing = true;
	hrtimer_cancel(&dev->flp_timer);
	cancel_work_sync(&dev->tx_work);

	unregister_netdev(dev->netdev);

	free_netdev(dev->netdev);

	spi10m_free_dma_buffer(dev, 0);
	spi10m_free_dma_buffer(dev, 1);
	spi10m_free_dma_buffer(dev, 2);

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
