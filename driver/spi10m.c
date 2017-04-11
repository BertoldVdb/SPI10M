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
 * The normal preamble is 8 bytes long, but if you use a capacitor to drive the line
 * a longer preamble may be required to ensure no packets are lost
 */
static unsigned int spi10m_preamble_length = 8;
module_param(spi10m_preamble_length, int, S_IRUGO);
MODULE_PARM_DESC(spi10m_preamble_length, "Number of preamble bytes");

#define SPI10M_MIN_LEN 60
#define SPI10M_TXBUFFER_MAX_PACKET 1500

static const uint16_t spi10m_manchester_table[] = {
        0xAAAA, 0x6AAA, 0x9AAA, 0x5AAA, 0xA6AA, 0x66AA, 0x96AA, 0x56AA, 0xA9AA, 0x69AA, 0x99AA, 0x59AA, 0xA5AA, 0x65AA, 0x95AA, 0x55AA,
        0xAA6A, 0x6A6A, 0x9A6A, 0x5A6A, 0xA66A, 0x666A, 0x966A, 0x566A, 0xA96A, 0x696A, 0x996A, 0x596A, 0xA56A, 0x656A, 0x956A, 0x556A,
        0xAA9A, 0x6A9A, 0x9A9A, 0x5A9A, 0xA69A, 0x669A, 0x969A, 0x569A, 0xA99A, 0x699A, 0x999A, 0x599A, 0xA59A, 0x659A, 0x959A, 0x559A,
        0xAA5A, 0x6A5A, 0x9A5A, 0x5A5A, 0xA65A, 0x665A, 0x965A, 0x565A, 0xA95A, 0x695A, 0x995A, 0x595A, 0xA55A, 0x655A, 0x955A, 0x555A,
        0xAAA6, 0x6AA6, 0x9AA6, 0x5AA6, 0xA6A6, 0x66A6, 0x96A6, 0x56A6, 0xA9A6, 0x69A6, 0x99A6, 0x59A6, 0xA5A6, 0x65A6, 0x95A6, 0x55A6,
        0xAA66, 0x6A66, 0x9A66, 0x5A66, 0xA666, 0x6666, 0x9666, 0x5666, 0xA966, 0x6966, 0x9966, 0x5966, 0xA566, 0x6566, 0x9566, 0x5566,
        0xAA96, 0x6A96, 0x9A96, 0x5A96, 0xA696, 0x6696, 0x9696, 0x5696, 0xA996, 0x6996, 0x9996, 0x5996, 0xA596, 0x6596, 0x9596, 0x5596,
        0xAA56, 0x6A56, 0x9A56, 0x5A56, 0xA656, 0x6656, 0x9656, 0x5656, 0xA956, 0x6956, 0x9956, 0x5956, 0xA556, 0x6556, 0x9556, 0x5556,
        0xAAA9, 0x6AA9, 0x9AA9, 0x5AA9, 0xA6A9, 0x66A9, 0x96A9, 0x56A9, 0xA9A9, 0x69A9, 0x99A9, 0x59A9, 0xA5A9, 0x65A9, 0x95A9, 0x55A9,
        0xAA69, 0x6A69, 0x9A69, 0x5A69, 0xA669, 0x6669, 0x9669, 0x5669, 0xA969, 0x6969, 0x9969, 0x5969, 0xA569, 0x6569, 0x9569, 0x5569,
        0xAA99, 0x6A99, 0x9A99, 0x5A99, 0xA699, 0x6699, 0x9699, 0x5699, 0xA999, 0x6999, 0x9999, 0x5999, 0xA599, 0x6599, 0x9599, 0x5599,
        0xAA59, 0x6A59, 0x9A59, 0x5A59, 0xA659, 0x6659, 0x9659, 0x5659, 0xA959, 0x6959, 0x9959, 0x5959, 0xA559, 0x6559, 0x9559, 0x5559,
        0xAAA5, 0x6AA5, 0x9AA5, 0x5AA5, 0xA6A5, 0x66A5, 0x96A5, 0x56A5, 0xA9A5, 0x69A5, 0x99A5, 0x59A5, 0xA5A5, 0x65A5, 0x95A5, 0x55A5,
        0xAA65, 0x6A65, 0x9A65, 0x5A65, 0xA665, 0x6665, 0x9665, 0x5665, 0xA965, 0x6965, 0x9965, 0x5965, 0xA565, 0x6565, 0x9565, 0x5565,
        0xAA95, 0x6A95, 0x9A95, 0x5A95, 0xA695, 0x6695, 0x9695, 0x5695, 0xA995, 0x6995, 0x9995, 0x5995, 0xA595, 0x6595, 0x9595, 0x5595,
        0xAA55, 0x6A55, 0x9A55, 0x5A55, 0xA655, 0x6655, 0x9655, 0x5655, 0xA955, 0x6955, 0x9955, 0x5955, 0xA555, 0x6555, 0x9555, 0x5555
};

struct spi10m_dev {
	struct net_device *netdev;
	struct spi_device *spi;

	uint8_t buf[SPI10M_TXBUFFER_MAX_PACKET + 4];

	unsigned long tx_buffer_size;
	uint16_t* tx_buffer;
	dma_addr_t tx_buffer_dma;

	unsigned long spi_speed;

	uint64_t tx_packets;
	uint64_t tx_bytes;

	struct hrtimer nlp_timer;
	ktime_t nlp_period;
	uint8_t nlp_byte[1];
	int nlp_busy;
	struct spi_message nlp_message;
	struct spi_transfer nlp_transfer;

	struct work_struct nlp_work;
	struct work_struct tx_work;
	
	struct sk_buff *tx_skb;
};

static void spi10m_get_ethtool_stats(struct net_device *dev,
                                      struct ethtool_stats *stats, uint64_t *data)
{
        struct spi10m_dev *priv = netdev_priv(dev);

	memset(data, 0, 12*sizeof(uint64_t));

        data[0] = le64_to_cpu(priv->tx_packets);
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

static enum hrtimer_restart spi10m_send_nlp(struct hrtimer * timer)
{
	struct spi10m_dev *priv = container_of(timer, struct spi10m_dev, nlp_timer);
	
	hrtimer_forward(&priv->nlp_timer, ktime_get(), priv->nlp_period);
	
	if(!priv->nlp_busy){
		priv->nlp_busy = 1;
		schedule_work(&priv->nlp_work);
	}

	return HRTIMER_RESTART;
}

static netdev_tx_t spi10m_send_packet (struct sk_buff *skb, struct net_device *dev)
{
        struct spi10m_dev *priv = netdev_priv(dev);

	netif_stop_queue(dev);

	priv->tx_skb = skb;
	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;
}


static struct rtnl_link_stats64 *spi10m_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct spi10m_dev *priv = netdev_priv(dev);

	memset(stats, 0, sizeof(*stats));

	stats->tx_packets = priv->tx_packets;
	stats->tx_bytes = priv->tx_bytes;

	return stats;
}


static const struct net_device_ops spi10m_netdev_ops = {
        .ndo_start_xmit         = spi10m_send_packet,
        .ndo_set_mac_address    = spi10m_set_mac_address,
        .ndo_validate_addr      = eth_validate_addr,
	.ndo_get_stats64        = spi10m_get_stats64
};

static void spi10m_nlp_work_handler(struct work_struct *work)
{
	struct spi10m_dev *priv = container_of(work, struct spi10m_dev, nlp_work);

	struct spi_message message = {};
	struct spi_transfer transfer = {};

	spi_message_init(&message);
	transfer.tx_buf = priv->nlp_byte;
	transfer.len = 1;
	spi_message_add_tail(&transfer, &message);
	transfer.speed_hz = priv->spi_speed;
	
	spi_sync(priv->spi, &message);

	priv->nlp_busy=0;
}

static void spi10m_tx_work_handler(struct work_struct *work)
{
	struct spi10m_dev *priv = container_of(work, struct spi10m_dev, tx_work);

	unsigned int i;
	struct sk_buff *skb = priv->tx_skb;
	
	unsigned int len = skb->len;
	uint16_t* tx_packet = priv->tx_buffer;

	uint32_t crc_add;
	uint16_t tmp;

	struct spi_message message = {};
	struct spi_transfer transfer = {};

        if (len < SPI10M_TXBUFFER_MAX_PACKET) {
		/* Copy packet to local buffer */
		skb_copy_bits(skb, 0, priv->buf, len);
		dev_kfree_skb(skb);

		/* Skip preamble */
		tx_packet += spi10m_preamble_length;

		/* Add padding if needed */
		if(len < SPI10M_MIN_LEN){
			memset(priv->buf+len, 0, SPI10M_MIN_LEN - len);
			len = SPI10M_MIN_LEN;
		}

		/* Add checksum */
		crc_add = ~le32_to_cpu(ether_crc_le(len, priv->buf));
		memcpy(priv->buf+len, &crc_add, 4);
		len += 4;

		/* Convert the packet to manchester encoding */
		for(i=0; i<len; i++){
			tmp = spi10m_manchester_table[priv->buf[i]];
			tx_packet[i] = be16_to_cpu(tmp);
		}
		tx_packet+=len;

		/* Encode TP_IDL */
		if(tmp & 1){
			tx_packet[0] = be16_to_cpu(0xFC00); /* 300ns */
		}else{
			tx_packet[0] = be16_to_cpu(0xFE00); /* 350ns */
		}
		tx_packet++;

		/* Add interframe gap */
		for (i=0; i<12; i++){
			tx_packet[i] = 0;
		}

		/* Send it over SPI */
		spi_message_init(&message);
		transfer.tx_buf = priv->tx_buffer;
		transfer.tx_dma = priv->tx_buffer_dma;

		transfer.rx_buf = NULL;
		transfer.rx_dma = 0;

		transfer.len = (spi10m_preamble_length + len + 1 + 12) * 2;
		spi_message_add_tail(&transfer, &message);

		message.is_dma_mapped = 1;
		
		transfer.speed_hz = priv->spi_speed;

		spi_sync(priv->spi, &message);
		
		priv->tx_packets++;
		priv->tx_bytes += len;

        } else {
		dev_kfree_skb(skb);
        }

	netif_wake_queue(priv->netdev);
}

static int spi10m_probe(struct spi_device *spi){
	int ret = 0, i;

	struct spi10m_dev *dev;
	struct net_device *netdev;
        netdev = alloc_etherdev(sizeof(struct spi10m_dev));
	if(!netdev){
		ret = -ENOMEM;
		dev_err(&spi->dev, "Failed to allocate device\n");
		goto err_allocdev;
	}

	dev = netdev_priv(netdev);
	dev->netdev = netdev;
	
	spi_set_drvdata(spi, dev);
	SET_NETDEV_DEV(netdev, &spi->dev);

	dma_set_coherent_mask(&spi->dev, DMA_BIT_MASK(32));

	if(spi10m_preamble_length<=0){
		spi10m_preamble_length = 1;
	}

	dev->tx_buffer_size = (spi10m_preamble_length + SPI10M_TXBUFFER_MAX_PACKET + 4 + 1 + 12) * 2;
	dev->tx_buffer = dma_alloc_coherent(&spi->dev, dev->tx_buffer_size, &dev->tx_buffer_dma, GFP_KERNEL);
	if(!dev->tx_buffer){
		ret = -ENOMEM;
		dev_err(&spi->dev, "TX Buffer allocation failed\n");
		goto err_allocbuf;
	}

	/* Add the preamble and SFD */
	for(i=0; i<(spi10m_preamble_length-1); i++){
		dev->tx_buffer[i]=be16_to_cpu(spi10m_manchester_table[0x55]);
	}
	dev->tx_buffer[spi10m_preamble_length-1]=be16_to_cpu(spi10m_manchester_table[0xD5]);

	/* This is a transmit only adapter so the MAC address doesn't really matter */
	eth_hw_addr_random(netdev);

	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = &spi10m_netdev_ops;
	netdev->watchdog_timeo = 10 * HZ; /* 10 seconds */
	netdev->ethtool_ops = &spi10m_ethtool_ops;

	/* Setup SPI to 20MHz to create a 10MHz manchester signal */
	dev->spi_speed = 20000000;
	dev->spi = spi;

	INIT_WORK(&dev->tx_work, spi10m_tx_work_handler);
	INIT_WORK(&dev->nlp_work, spi10m_nlp_work_handler);

	ret = register_netdev(netdev);
	if(ret){
		dev_err(&spi->dev, "Netdev registration failed: %d\n", ret);

		goto err_register;
	}

	dev->nlp_period = ktime_set(0, 16000000); /* 16ms */
	hrtimer_init (&dev->nlp_timer, CLOCK_REALTIME, HRTIMER_MODE_REL);
	dev->nlp_timer.function = spi10m_send_nlp;
	dev->nlp_byte[0]=0x3;
	hrtimer_start(&dev->nlp_timer, dev->nlp_period, HRTIMER_MODE_REL);
	
	netif_start_queue(netdev);

	return 0;

err_register:
	dma_free_coherent(&spi->dev, dev->tx_buffer_size, dev->tx_buffer, dev->tx_buffer_dma);
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

	dma_free_coherent(&spi->dev, dev->tx_buffer_size, dev->tx_buffer, dev->tx_buffer_dma);

	return 0;
}


static const struct of_device_id spi10m_of_match[] = {
	{ .compatible = "bertold,spi10m", },
	{ },
};
MODULE_DEVICE_TABLE(of, spi10m_of_match);

static struct spi_driver spi10m_driver = {
	.driver = {
		.name = "spi10m",
		.owner = THIS_MODULE,
	},
	.probe      = spi10m_probe,
	.remove     = spi10m_remove,
};
module_spi_driver(spi10m_driver);

MODULE_AUTHOR("Bertold Van den Bergh (vandenbergh@bertold.org)");
MODULE_DESCRIPTION("SPI10M");
MODULE_LICENSE("GPL");
