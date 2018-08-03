/*
 * BCM5357 NAND flash driver
 *
 * Copyright (C) 2012 Rafał Miłecki <zajec5@gmail.com>
 * Copyright (C) 2018 Erik Zachrisson <erik@zachrisson.info>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include "bcm47xxnflash.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bcma/bcma.h>

/* nand_cmd_start commands */
/* These are stolen from brcm/mips/include/nand.h, maybe these are already defined somewhere in the kernel source tree */
#define NCMD_NULL			0
#define NCMD_PAGE_RD			1
#define NCMD_SPARE_RD			2
#define NCMD_STATUS_RD			3
#define NCMD_PAGE_PROG			4
#define NCMD_SPARE_PROG			5
#define NCMD_COPY_BACK			6
#define NCMD_ID_RD			7
#define NCMD_BLOCK_ERASE		8
#define NCMD_FLASH_RESET		9
#define NCMD_LOCK			0xa
#define NCMD_LOCK_DOWN			0xb
#define NCMD_UNLOCK			0xc
#define NCMD_LOCK_STATUS		0xd

/*
 * Standard NAND flash commands
 */
/* #define NAND_CMD_READ0		0 */
/* #define NAND_CMD_READ1		1 */
/* #define NAND_CMD_RNDOUT		5 */
/* #define NAND_CMD_PAGEPROG	0x10 */
/* #define NAND_CMD_READOOB	0x50 */
/* #define NAND_CMD_ERASE1		0x60 */
/* #define NAND_CMD_STATUS		0x70 */
/* #define NAND_CMD_SEQIN		0x80 */
/* #define NAND_CMD_RNDIN		0x85 */
/* #define NAND_CMD_READID		0x90 */
/* #define NAND_CMD_ERASE2		0xd0 */
/* #define NAND_CMD_PARAM		0xec */
/* #define NAND_CMD_GET_FEATURES	0xee */
/* #define NAND_CMD_SET_FEATURES	0xef */
/* #define NAND_CMD_RESET		0xff */

/* nand_acc_control */
#define	NAC_RD_ECC_EN			0x80000000
#define	NAC_WR_ECC_EN			0x40000000
#define	NAC_RD_ECC_BLK0_EN		0x20000000
#define	NAC_FAST_PGM_RDIN		0x10000000
#define	NAC_RD_ERASED_ECC_EN		0x08000000
#define	NAC_PARTIAL_PAGE_EN		0x04000000
#define	NAC_PAGE_HIT_EN			0x01000000
#define	NAC_ECC_LEVEL0_MASK		0x00f00000
#define	NAC_ECC_LEVEL0_SHIFT		20
#define	NAC_ECC_LEVEL_MASK		0x000f0000
#define	NAC_ECC_LEVEL_SHIFT		16
#define	NAC_SPARE_SIZE0			0x00003f00
#define	NAC_SPARE_SIZE			0x0000003f


/* nand_intfc_status */
#define	NIST_CTRL_READY			0x80000000
#define	NIST_FLASH_READY		0x40000000
#define	NIST_CACHE_VALID		0x20000000
#define	NIST_SPARE_VALID		0x10000000
#define	NIST_ERASED			0x08000000
#define	NIST_STATUS			0x000000ff

/* FIXME: Lower this to 10000 but test first */
#define NF_RETRIES	1000000
#define NFL_SECTOR_SIZE			512


/* K9F1G08XD has 64K pages = 1024 blocks */
/* 1 Block is 64 pages (128k + 4k), 1 Page = 2K + 64 bytes */
/* 65536 pages i.e 16 bits */
/* Column address is A0 - A11 = 12 bits = 2**12 = 4096 bytes */
/* Highest bit 12 is used to access spare bits */
/* Row address is A12 - A27 = 16 bits */
/* FIXME: Double check this, hard coded for the WNR3500LV2 */
#define K9F1G08X0D_PAGE_SZ 2048
#define K9F1G08X0D_COL_SZ 11

/* Column is within a page */
/* Page addr is all pages */


/**************************************************
 * Various helpers
 **************************************************/
static inline int bcm47xxnflash_ops_bcm5357_ctl_cmd(struct bcma_drv_cc *cc, u32 code)
{
	bcma_cc_write32(cc, BCMA_CC_NAND_CMD_START, code);
	/* Read after write to flush the command */
	bcma_cc_read32(cc, BCMA_CC_NAND_CMD_START);

	/* FIXME: Add polling for completion here? */
	return 0;
}

static inline u8 bcm47xxnnflash_ops_bcm5357_check_status(struct bcma_drv_cc *cc)
{
	return bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS) & (NIST_STATUS & 1);
}

static inline void bcm47xxnflash_ops_bcm5357_calc_and_set_offset(struct bcma_drv_cc *cc, int page_addr, int column) {
	/* FIXME: Hardcoded page size for now */
	/* Do we need to account for spare area? */
	bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, (page_addr << K9F1G08X0D_COL_SZ) | column);
}

static void bcm47xxnflash_ops_bcm5357_enable(struct bcma_drv_cc *cc, bool enable)
{
	u32 mask;
	u32 val;

	mask = ~(BCMA_CHIPCTL_5357_NFLASH);
	val = 0;
	if (enable) {
		val = BCMA_CHIPCTL_5357_NFLASH;
	}

	bcma_pmu_maskset32(cc, BCMA_CC_PMU_CHIPCTL_DATA, mask, val);
}

/* Poll for command completion. Returns zero when complete. */
static int bcm47xxnflash_ops_bcm5357_poll(struct bcma_drv_cc *cc)
{
	int i;
	u32 val;

	for (i = 0; i < NF_RETRIES; i++) {
		val = bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS);

		if ((val & (NIST_CTRL_READY | NIST_FLASH_READY)) ==
		     (NIST_CTRL_READY | NIST_FLASH_READY)) {
			return 0;
		}
	}

	pr_err("Polling timeout!\n");
	return -EBUSY;
}

/**************************************************
 * R/W
 **************************************************/

static void bcm47xxnflash_ops_bcm5357_read(struct mtd_info *mtd, uint8_t *buf,
					   int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u64 offset;
	u32 *buf32;
	int i;
	int toread;

	buf32 = (u32 *) buf;
	offset = (b47n->curr_page_addr << K9F1G08X0D_COL_SZ) | b47n->curr_column;
	while (len > 0) {
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset);
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("Failed PAGE_RD\n");
		}

		if ((bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS) & NIST_CACHE_VALID) == 0) {
			pr_err("Read cache not valid\n");
			break;
		}

		/* FIXME: Can this be omitted and moved to the init part of the function? */
		bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_ADDR, 0);

		toread = min(len, NFL_SECTOR_SIZE);
		/* PIO Read, is DMA possible? Maybe not worth it if one only can read out 512 bytes in one go */
		for (i = 0; i < toread; i += sizeof(buf32), buf32++) {
			*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA);
		}
		len -= toread;
		offset += toread;
	}
}

static void bcm47xxnflash_ops_bcm5357_write(struct mtd_info *mtd,
					    const uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u32 reg;
	int i;
	u32 *from;
	int towrite;
	u64 offset;

        offset = (b47n->curr_page_addr << K9F1G08X0D_COL_SZ) | b47n->curr_column;
	from = (u32 *) buf;

	/* Disable partial page programming */
	reg = bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL);
	reg &= ~NAC_PARTIAL_PAGE_EN;
	bcma_cc_write32(cc, BCMA_CC_NAND_ACC_CONTROL, reg);

	while (len > 0) {
		/* FIXME: Do not rely on flash specific page size */
		towrite = min(len, K9F1G08X0D_PAGE_SZ);
		bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_ADDR, 0);
		/* Transfer data to nand controller cache */
		for (i = 0; i < towrite; i += sizeof(from), from++) {
			/* Reset cmd addr for each sector */
			/* I don't fully understand this logic */
			if (i % NFL_SECTOR_SIZE == 0) {
				bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, i);
			}

			bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_DATA, *from);
		}

		/* Set cmd address to end of page - last sector size */
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset + K9F1G08X0D_PAGE_SZ - NFL_SECTOR_SIZE);
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_PROG);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("Failed PAGE_PROG\n");
			break;
		}
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_STATUS_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("Failed STATUS_RD after PAGE_PROG\n");
			break;
		}

		if (bcm47xxnnflash_ops_bcm5357_check_status(cc) > 0) {
			pr_err("PAGE_PROG failed\n");
			break;
		}

		b47n->curr_column += towrite;
		len -= towrite;
		offset += towrite;
	}
}

/**************************************************
 * NAND chip ops
 **************************************************/

static void bcm47xxnflash_ops_bcm5357_cmd_ctrl(struct mtd_info *mtd, int cmd,
					       unsigned int ctrl)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	if (cmd == NAND_CMD_NONE)
		return;

	bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, cmd);
}

static int bcm47xxnflash_ops_bcm5357_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u32 val;

	val = bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS);

	if ((val & (NIST_CTRL_READY | NIST_FLASH_READY)) ==
	    (NIST_CTRL_READY | NIST_FLASH_READY)) {
	    return ~0;
	}
        return 0;
}

static void bcm47xxnflash_ops_bcm5357_cmdfunc(struct mtd_info *mtd,
					      unsigned command, int column,
					      int page_addr)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;
	u32 reg;

	if (column != -1)
		b47n->curr_column = column;
	if (page_addr != -1)
		b47n->curr_page_addr = page_addr;

	switch (command) {
	case NAND_CMD_RESET:
		/* FIXME: Double check this */
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_FLASH_RESET);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("Failed FLASH_RESET\n");
		}
		break;
	case NAND_CMD_READID:
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_ID_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("Failed to read id\n");
			break;
		}

		/* FIXME: Can surely be optimized with some casting cleverness */
		reg = bcma_cc_read32(cc, BCMA_CC_NAND_DEVID);
		memcpy(b47n->id_data, &reg, sizeof(reg));

	        reg = bcma_cc_read32(cc, BCMA_CC_NAND_DEVID_X);
		memcpy(b47n->id_data + sizeof(reg), &reg, sizeof(reg));
		break;
	case NAND_CMD_STATUS:
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_STATUS_RD);
		break;
	case NAND_CMD_READ0:
		/* FIXME: Nop this as the 4706, no clue what to do */
		break;
	case NAND_CMD_READOOB:
		if (page_addr != -1)
			b47n->curr_column += mtd->writesize;
		break;
	case NAND_CMD_ERASE1:
		/* FIXME: Do we need to check for non block aligned offsets */
		bcm47xxnflash_ops_bcm5357_calc_and_set_offset(cc, page_addr, column);
                pr_err("BLOCK_ERASE disabled for now\n");

/* bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_BLOCK_ERASE); */

		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("Failed BLOCK_ERASE cmd\n");
			break;
		}
		if (bcm47xxnnflash_ops_bcm5357_check_status(cc) > 0) {
			pr_err("Bad NIST_STATUS after block erase\n");
			break;
		}
	case NAND_CMD_ERASE2:
		break;
	case NAND_CMD_SEQIN:
		bcm47xxnflash_ops_bcm5357_calc_and_set_offset(cc, page_addr, column);

		/* FIXME: Double check this */
		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;
			bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READOOB);
		} else if (column < 256) {
			/* First 256 bytes --> READ0 */
			bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READ0);
		} else {
			column -= 256;
			bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READ1);
		}
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_BLOCK_ERASE);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("SEQIN failed\n");
		}
		break;
	case NAND_CMD_PAGEPROG:
		/* FIXME: Should we set page offset here */
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_PROG);
		if (bcm47xxnflash_ops_bcm5357_poll(cc) < 0) {
			pr_err("PAGE_PROG failed\n");
		}
		break;
	default:
		pr_err("Command 0x%X unsupported\n", command);
		break;
	}

	b47n->curr_command = command;
}

static u8 bcm47xxnflash_ops_bcm5357_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	u8 data;
	u32 tmp;

	data = 0;
	switch (b47n->curr_command) {
	case NAND_CMD_READID:
		if (b47n->curr_column >= ARRAY_SIZE(b47n->id_data)) {
			pr_err("Requested invalid id_data: %d\n",
			       b47n->curr_column);
			break;
		}
		data = b47n->id_data[b47n->curr_column++];
		break;

	/* FIXME: No clue why this is done this way, just mimicking the 4706 variant */
	case NAND_CMD_STATUS:
	case NAND_CMD_READOOB:
		bcm47xxnflash_ops_bcm5357_read(mtd, (u8 *)&tmp, 4);
		data = tmp & 0xFF;
		break;
	default:
		pr_err("Invalid command for byte read: 0x%X\n", b47n->curr_command);
	}

	return data;
}

static void bcm47xxnflash_ops_bcm5357_read_buf(struct mtd_info *mtd,
					       uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);

	switch (b47n->curr_command) {
	case NAND_CMD_READ0:
	case NAND_CMD_READOOB:
		bcm47xxnflash_ops_bcm5357_read(mtd, buf, len);
		return;
	}

	pr_err("Invalid command for read buf: 0x%X\n", b47n->curr_command);
}

static void bcm47xxnflash_ops_bcm5357_write_buf(struct mtd_info *mtd,
						const uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);

	switch (b47n->curr_command) {
	case NAND_CMD_SEQIN:
		bcm47xxnflash_ops_bcm5357_write(mtd, buf, len);
		return;
	}

	pr_err("Invalid command for buf write: 0x%X\n", b47n->curr_command);
}

/**************************************************
 * Init
 **************************************************/

int bcm47xxnflash_ops_bcm5357_init(struct bcm47xxnflash *b47n)
{
	struct bcma_drv_cc *cc = b47n->cc;
	struct nand_chip *nand_chip = (struct nand_chip *)&b47n->nand_chip;
	int err;

	pr_info("Initializing bcm5357 NAND controller\n");
	err = 0;

	nand_chip->cmd_ctrl = bcm47xxnflash_ops_bcm5357_cmd_ctrl;
	nand_chip->dev_ready = bcm47xxnflash_ops_bcm5357_dev_ready;

	b47n->nand_chip.cmdfunc = bcm47xxnflash_ops_bcm5357_cmdfunc;
	b47n->nand_chip.read_buf = bcm47xxnflash_ops_bcm5357_read_buf;
	b47n->nand_chip.write_buf = bcm47xxnflash_ops_bcm5357_write_buf;
	b47n->nand_chip.read_byte = bcm47xxnflash_ops_bcm5357_read_byte;

	b47n->nand_chip.onfi_set_features = nand_onfi_get_set_features_notsupp;
	b47n->nand_chip.onfi_get_features = nand_onfi_get_set_features_notsupp;

	/* As per K9F1G08U0D data sheet Rev 0.0 Dec 9 2009, page 13 tR */
	nand_chip->chip_delay = 35;
	b47n->nand_chip.bbt_options = NAND_BBT_NO_OOB_BBM | NAND_BBT_USE_FLASH;

	/* FIXME: Just hoping for the best */
	b47n->nand_chip.ecc.mode = NAND_ECC_NONE;

	/* FIXME: Must enable ecc configuration here */

	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	/* Scan NAND */
	pr_err("Scanning nand\n");
	err = nand_scan(nand_to_mtd(&b47n->nand_chip), 1);
	pr_err("Page shift is %d\n", b47n->nand_chip.page_shift);

	if (err) {
		pr_err("Could not scan NAND flash: %d\n", err);
		goto exit;
	}

	/* Return negative for now to stop further nand booting */
	err = -1;
	goto exit;

exit:
	/* Disable flash */
	bcm47xxnflash_ops_bcm5357_enable(cc, false);
        return err;
}
