/*
 * (C) Copyright 2009 RF Code, Inc.
 * (C) Copyright 2005-2008 Analog Devices Inc.
 * (C) Copyright 2008 Atmel Corporation
 *
 * Based on
 * drivers/mtd/spi/atmel.c
 * 
 * Jedec SPI Flash support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 *
 */

#include <common.h>
#include <malloc.h>
#include <spi_flash.h>

#include "spi_flash_internal.h"

#define READY_STATUS	1
#define WREN_STATUS	2

enum {
	JED_MANU_SPANSION = 0x01,
	JED_MANU_ST       = 0x20,
	JED_MANU_WINBOND  = 0xEF,
};

struct flash_info {
	char     *name;
	uint16_t id;
	unsigned sector_size;
	unsigned num_sectors;
};

/* SPI Speeds: 50 MHz / 33 MHz */
static struct flash_info flash_spansion_serial_flash[] = {
	{ "S25FL016",	0x0215,  64 * 1024,  32 },
	{ "S25FL032",	0x0216,  64 * 1024,  64 },
	{ "S25FL064",	0x0217,  64 * 1024, 128 },
	{ "S25FL0128",	0x0218, 256 * 1024,  64 },
	{ NULL, 0, 0, 0 }
};

/* SPI Speeds: 50 MHz / 20 MHz */
static struct flash_info flash_st_serial_flash[] = {
	{ "M25P05",	0x2010,  32 * 1024,   2 },
	{ "M25P10",	0x2011,  32 * 1024,   4 },
	{ "M25P20",	0x2012,  64 * 1024,   4 },
	{ "M25P40",	0x2013,  64 * 1024,   8 },
	{ "M25P16",	0x2015,  64 * 1024,  32 },
	{ "M25P32",	0x2016,  64 * 1024,  64 },
	{ "M25P64",	0x2017,  64 * 1024, 128 },
	{ "M25P128",	0x2018, 256 * 1024,  64 },
	{ NULL, 0, 0, 0 }
};

/* SPI Speed: 50 MHz / 25 MHz or 40 MHz / 20 MHz */
static struct flash_info flash_winbond_serial_flash[] = {
	{ "W25X10",	0x3011,  16 * 256,  32 },
	{ "W25X20",	0x3012,  16 * 256,  64 },
	{ "W25X40",	0x3013,  16 * 256, 128 },
	{ "W25X80",	0x3014,  16 * 256, 256 },
	{ "W25P80",	0x2014, 256 * 256,  16 },
	{ "W25P16",	0x2015, 256 * 256,  32 },
	{ NULL, 0, 0, 0 }
};

struct flash_ops {
	uint8_t read;
	uint8_t write;
	uint8_t erase;
	uint8_t status;
	uint8_t enable_write;
};

#ifdef CFG_SPIFLASH_SLOW_READ
 #define OP_READ	CMD_READ_ARRAY_SLOW
#else
 #define OP_READ	CMD_READ_ARRAY_FAST
#endif

static struct flash_ops flash_st_ops = {
	.read = OP_READ,
	.write = 0x02,
	.erase = 0xD8,
	.status = 0x05,
	.enable_write = 0x06,
};

static struct flash_ops flash_winbond_ops = {
	.read = OP_READ,
	.write = 0x02,
	.erase = 0x20,
	.status = 0x05,
	.enable_write = 0x06,
};

struct manufacturer_info {
	const char *name;
	uint8_t id;
	struct flash_info *flashes;
	struct flash_ops *ops;
};

struct jedec_spi_flash {
	struct manufacturer_info *manufacturer;
	struct flash_info *info;
	struct flash_ops *ops;
	struct spi_flash flash;
	unsigned int write_length;
	unsigned long sector_size, num_sectors;
};

static struct manufacturer_info flash_manufacturers[] = {
	{
		.name = "Spansion",
		.id = JED_MANU_SPANSION,
		.flashes = flash_spansion_serial_flash,
		.ops = &flash_st_ops,
	},
	{
		.name = "ST",
		.id = JED_MANU_ST,
		.flashes = flash_st_serial_flash,
		.ops = &flash_st_ops,
	},
	{
		.name = "Winbond",
		.id = JED_MANU_WINBOND,
		.flashes = flash_winbond_serial_flash,
		.ops = &flash_winbond_ops,
	},
};

/* to_jedec_spi_flash
 * 
 * Wrapper for container_of in order to return our jedec_spi_flash pointer
 */
static inline struct jedec_spi_flash *
to_jedec_spi_flash(struct spi_flash *flash)
{
	return container_of(flash, struct jedec_spi_flash, flash);
}

/* jedec_wait_status
 * 
 * Poll the status register waiting for a given type of status
 */
static int jedec_wait_status(struct jedec_spi_flash *jsf, int type,
		unsigned long timeout)
{
	struct spi_slave *spi = jsf->flash.spi;
	unsigned long timebase;
	int ret;
	int done = 0;
	u8 cmd = jsf->ops->status;
	u8 status;

	timebase = get_timer(0);

	ret = spi_xfer(spi, 8, &cmd, NULL, SPI_XFER_BEGIN);
	if (ret) return ret;

	do {
		ret = spi_xfer(spi, 8, NULL, &status, 0);
		if (ret) return ret;
			
		if (type == READY_STATUS) {
			/* Wait for Write-in-progress bit to clear */
			if (!(status & 0x01)) {
				done = 1;
				break;
			}
		} else {
			/* Wait for Write Enable bit to set */
			if (status & 0x02) {
				done = 1;
				break;
			}
		}

		if (ctrlc()) {
			puts("\nAbort\n");
			return -1;
		}
	} while (get_timer(timebase) < timeout);

	/* Deactivate CS */
	spi_xfer(spi, 0, NULL, NULL, SPI_XFER_END);

	return done ? 0 : -1;
}
/* jedec_wait_ready - wait for a write operation to complete
 */
static int jedec_wait_ready(struct jedec_spi_flash *jsf, unsigned long timeout)
{
	return jedec_wait_status(jsf, READY_STATUS, timeout);
}

/* jedec_wait_wren - wait for the write enable bit to be set
 */
static int jedec_wait_wren(struct jedec_spi_flash *jsf, unsigned long timeout)
{
	return jedec_wait_status(jsf, WREN_STATUS, timeout);
}

/* jedec_spi_flash_enable_write
 * 
 * Send the enable-write command and then wait for the write enable bit to
 * be set in the status register. SPI bus is already claimed before 
 * calling this function.
 */
static int jedec_spi_flash_enable_write(struct jedec_spi_flash *jsf,
		unsigned long timeout)
{		
	u8 cmd = jsf->ops->enable_write;
	int ret;

	ret = spi_xfer(jsf->flash.spi, 8, &cmd, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret < 0) return ret;
	
	/* The status register will be polled to check the write enable latch "WREN" */
	ret = jedec_wait_wren(jsf, timeout);
	if (ret < 0) {
		puts("Timeout waiting for write enable\n");
		return -1;
	}
	return 0;
}

/* jedec_spi_flash_read
 */ 
static int jedec_spi_flash_read(struct spi_flash *flash,
		u32 offset, size_t len, void *buf)
{
	struct jedec_spi_flash *jsf = to_jedec_spi_flash(flash);
	u8 cmd[5];

	cmd[0] = jsf->ops->read;
	cmd[1] = (offset & 0x00FF0000) >> 16;
	cmd[2] = (offset & 0x0000FF00) >> 8;
	cmd[3] = (offset & 0x000000FF);
	cmd[4] = 0;

	if (cmd[0] == CMD_READ_ARRAY_FAST)
		return spi_flash_read_common(flash, cmd, 5, buf, len);
	else
		return spi_flash_read_common(flash, cmd, 4, buf, len);
}

/* jedec_spi_flash_write
 */
static int jedec_spi_flash_write(struct spi_flash *flash,
		u32 offset, size_t len, const void *buf)
{
	struct jedec_spi_flash *jsf = to_jedec_spi_flash(flash);
	unsigned long byte_addr;
	unsigned long page_size;
	u32 address = offset;
	size_t chunk_len;
	size_t actual;
	int ret;
	u8 cmd[4];

	page_size = jsf->write_length;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf("SF: Unable to claim SPI bus\n");
		return ret;
	}
	
	cmd[0] = jsf->ops->write;
	for (actual = 0; actual < len; actual += chunk_len) {
		/* We have to enable writing before each chunk */
		ret = jedec_spi_flash_enable_write(jsf, SPI_FLASH_PROG_TIMEOUT);
		if (ret) {
			printf("SF: Enabling write failed\n");
			goto out;
		}
		
		/* What address within the page are we starting with */
		byte_addr = address % page_size;
		
		/* Write the smaller of full amount or bytes left in the page */
		chunk_len = min(len - actual, page_size - byte_addr);

		/* Fill in the address portion of the command */
		cmd[1] = (address & 0x00FF0000) >> 16;
		cmd[2] = (address & 0x0000FF00) >> 8;
		cmd[3] = (address & 0x000000FF);

		ret = spi_flash_cmd_write(flash->spi, cmd, 4,
				buf + actual, chunk_len);
		if (ret < 0) {
			printf("SF: Loading write buffer failed\n");
			goto out;
		}

		ret = jedec_wait_ready(jsf, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			printf("SF: Page programming timed out\n");
			goto out;
		}

		address += chunk_len;
	}

	printf("SF: Successfully programmed %zu bytes @ 0x%x\n", len, offset);
	ret = 0;
out:
	spi_release_bus(flash->spi);
	return ret;
}

/* jedec_spi_flash_erase
 */
int jedec_spi_flash_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	struct jedec_spi_flash *jsf = to_jedec_spi_flash(flash);
	unsigned sector_size;
	u32 address = offset;
	size_t actual;
	int ret;
	u8 cmd[4];

	sector_size = jsf->sector_size;
	if (address % sector_size || len % sector_size) {
		printf("SF: Erase offset/length not multiple of sector size\n");
		return -1;
	}

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		printf("SF: Unable to claim SPI bus\n");
		return ret;
	}

	cmd[0] = jsf->ops->erase;
	for (actual = 0; actual < len; actual += sector_size) {
		/* We have to enable writing before each sector */
		ret = jedec_spi_flash_enable_write(jsf, SPI_FLASH_PROG_TIMEOUT);
		if (ret) {
			printf("SF: Enabling write failed\n");
			goto out;
		}
		
		/* Fill in the address portion of the command */
		cmd[1] = (address & 0x00FF0000) >> 16;
		cmd[2] = (address & 0x0000FF00) >> 8;
		cmd[3] = (address & 0x000000FF);

		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret < 0) {
			printf("SF: page erase command failed (%d)\n", ret);
			goto out;
		}

		ret = jedec_wait_ready(jsf, SPI_FLASH_SECTOR_ERASE_TIMEOUT);
		if (ret < 0) {
			printf("SF: page erase timed out\n");
			goto out;
		}
		address += sector_size;
	}

	printf("SF: Successfully erased %zu bytes @ 0x%x\n", len, offset);
	ret = 0;
out:
	spi_release_bus(flash->spi);
	return ret;
}

/* spi_flash_probe_jedec
 * 
 * See if this SPI device is one that we support and if so, claim it
 */
struct spi_flash *spi_flash_probe_jedec(struct spi_slave *spi, u8 *idcode)
{
	struct jedec_spi_flash *jsf = NULL;
	u16 dev_id;
	int i;
	
	/* Make sure this is a manufacturer we support */
	for (i = 0; i < ARRAY_SIZE(flash_manufacturers); ++i) {
		if (idcode[0] == flash_manufacturers[i].id)
			break;
	}
	if (i == ARRAY_SIZE(flash_manufacturers))
		goto unknown;

	/* Allocate a jedec_spi_flash object to store everything */
	jsf = malloc(sizeof(struct jedec_spi_flash));
	if (!jsf) {
		printf("spi_flash_probe_jedec: Failed to allocate memory\n");
		return NULL;
	}
	jsf->manufacturer = &flash_manufacturers[i];
	jsf->ops = flash_manufacturers[i].ops;

	/* Assemble the 16-bit device id */
	dev_id = (idcode[1] << 8) | idcode[2];

	/* Make sure this is a device id that we support */
	for (i = 0; jsf->manufacturer->flashes[i].name; ++i) {
		if (dev_id == jsf->manufacturer->flashes[i].id)
			break;
	}
	if (!jsf->manufacturer->flashes[i].name)
		goto unknown;

	jsf->info = &jsf->manufacturer->flashes[i];
	jsf->sector_size = jsf->info->sector_size;
	jsf->num_sectors = jsf->info->num_sectors;
	jsf->write_length = 256;
	
	jsf->flash.spi = spi;
	jsf->flash.name = jsf->info->name;
	jsf->flash.size = jsf->num_sectors * jsf->sector_size;
	jsf->flash.read = jedec_spi_flash_read;
	jsf->flash.write = jedec_spi_flash_write;
	jsf->flash.erase = jedec_spi_flash_erase;

	debug("SF: Detected %s with sector size %u, total %u bytes\n",
			jsf->flash.name, jsf->info->sector_size, jsf->flash.size);

	return &jsf->flash;

unknown:
	printf("Unsupported SPI flash device\n");
	if (jsf)
		free(jsf);
	return NULL;
}
