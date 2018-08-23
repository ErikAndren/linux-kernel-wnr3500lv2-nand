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
#define NCMD_NULL			0
#define NCMD_PAGE_RD			1
#define NCMD_SPARE_RD			2
/* Used to update the INTFC register */
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
/* Spare sizes are per sub page */
#define	NAC_SPARE_SIZE0			0x00003f00
/* SPARE_SIZE0 cannot be set to 0x40 */
#define NAC_SPARE_SIZE0_SHIFT           8
#define	NAC_SPARE_SIZE			0x0000003f
/* SPARE_SIZE cannot be set to 0x40 */
#define NAC_SPARE_SIZE_SHIFT            0

/* nand_intfc_status */
/* What is the difference between ctrl and flash ready? */
/* Current hypothesis is that ctrl ready means that flash is ready for a new command */
#define	NIST_CTRL_READY			0x80000000
#define	NIST_FLASH_READY		0x40000000
#define	NIST_CACHE_VALID		0x20000000
#define	NIST_SPARE_VALID		0x10000000
#define	NIST_ERASED			0x08000000
#define	NIST_STATUS			0x000000ff

/* #define NAND_STATUS_FAIL	            0x01 */
/* #define NAND_STATUS_FAIL_N1              0x02 */
/* #define NAND_STATUS_TRUE_READY	    0x20 */
/* #define NAND_STATUS_READY	            0x40 */
/* #define NAND_STATUS_WP		    0x80 */

#define NF_RETRIES	                1000000
#define NFL_SECTOR_SIZE			512

#define BCM5357_CMD_DEBUG 1
#define BCM5357_DATA_DEBUG 0
#define BCM5357_NAND_ENABLE_DEBUG 0
#define BCM5357_POLL_DEBUG 1

#define BRCMNAND_FLASH_STATUS_ERROR         (-2)
#define BRCMNAND_TIMED_OUT                  (-3)
#define BRCMNAND_READING_ERASED_BLOCK       (-4)


/* K9F1G08XD has 64K pages = 1024 blocks */
/* 1 Block is 64 pages (128k + 4k), 1 Page = 2K + 64 bytes */
/* 65536 pages i.e 16 bits */
/* Column address is A0 - A11 = 12 bits = 2**12 = 2048 bytes */
/* Row address is A12 - A27 = 2**16 bits = 65536 */
/* Column is within a page */
/* Page addr is all pages */

/**************************************************
 * Various helpers
 **************************************************/
static void bcm47xxnflash_ops_bcm5357_dump_regs(struct bcma_drv_cc *cc)
{
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_REVISION, bcma_cc_read32(cc, BCMA_CC_NAND_REVISION));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_START, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_START));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_END_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_END_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CS_NAND_SELECT, bcma_cc_read32(cc, BCMA_CC_NAND_CS_NAND_SELECT));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CS_NAND_XOR, bcma_cc_read32(cc, BCMA_CC_NAND_CS_NAND_XOR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD0, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD0));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD4, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD4));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD8, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD8));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD12, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD12));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR0, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR0));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR4, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR4));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR8, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR8));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR12, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR12));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_ACC_CONTROL, bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CONFIG, bcma_cc_read32(cc, BCMA_CC_NAND_CONFIG));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_1, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_1));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_2, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_2));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SEMAPHORE, bcma_cc_read32(cc, BCMA_CC_NAND_SEMAPHORE));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_DEVID, bcma_cc_read32(cc, BCMA_CC_NAND_DEVID));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_DEVID_X, bcma_cc_read32(cc, BCMA_CC_NAND_DEVID_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLOCK_LOCK_STATUS, bcma_cc_read32(cc, BCMA_CC_NAND_BLOCK_LOCK_STATUS));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_INTFC_STATUS, bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_CORR_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_CORR_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_CORR_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_CORR_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_UNC_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_UNC_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_UNC_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_UNC_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_READ_ERROR_COUNT, bcma_cc_read32(cc, BCMA_CC_NAND_READ_ERROR_COUNT));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CORR_STAT_THRESHOLD, bcma_cc_read32(cc, BCMA_CC_NAND_CORR_STAT_THRESHOLD));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_READ_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_READ_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_READ_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_READ_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_PAGE_PROGRAM_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_PAGE_PROGRAM_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_PAGE_PROGRAM_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_PAGE_PROGRAM_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_COPY_BACK_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_COPY_BACK_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_COPY_BACK_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_COPY_BACK_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLOCK_ERASE_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_BLOCK_ERASE_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLOCK_ERASE_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_BLOCK_ERASE_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_INV_READ_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_INV_READ_ADDR_X));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_INV_READ_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_INV_READ_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLK_WR_PROTECT, bcma_cc_read32(cc, BCMA_CC_NAND_BLK_WR_PROTECT));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_ACC_CONTROL_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL_CS1));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CONFIG_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_CONFIG_CS1));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_1_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_1_CS1));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_2_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_2_CS1));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD16, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD16));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD20, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD20));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD24, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD24));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD28, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD28));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CACHE_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_ADDR));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CACHE_DATA, bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CTRL_CONFIG, bcma_cc_read32(cc, BCMA_CC_NAND_CTRL_CONFIG));
	printk("0x%08x, 0x%08x\n", BCMA_CC_NAND_CTRL_STATUS, bcma_cc_read32(cc, BCMA_CC_NAND_CTRL_STATUS));
}

static inline void bcm47xxnflash_ops_bcm5357_ctl_cmd(struct bcma_drv_cc *cc, u32 code)
{
	bcma_cc_write32(cc, BCMA_CC_NAND_CMD_START, code);
	/* Read after write to flush the command */
	bcma_cc_read32(cc, BCMA_CC_NAND_CMD_START);
}

static inline void bcm47xxnflash_ops_bcm5357_calc_and_set_offset(struct bcm47xxnflash *b47n, int page_addr, int column) {
	if (column > 0) {
		panic("Tried to set column in cmd addr\n");
	}
        bcma_cc_write32(b47n->cc, BCMA_CC_NAND_CMD_ADDR, page_addr << b47n->nand_chip.page_shift);
}

static void bcm47xxnflash_ops_bcm5357_enable(struct bcma_drv_cc *cc, bool enable)
{
	u32 mask;
	u32 val;

	mask = ~(BCMA_CHIPCTL_5357_NFLASH);
	val = 0;
	if (enable) {
		val = BCMA_CHIPCTL_5357_NFLASH;
#if BCM5357_NAND_ENABLE_DEBUG == 1
		pr_err("Enabling BCM5357 NAND flash\n");
	} else {
		pr_err("Disabling BCM5357 NAND flash\n");
#endif
	}

	bcma_pmu_write32(cc, BCMA_CC_PMU_CHIPCTL_ADDR, 1);
	bcma_pmu_maskset32(cc, BCMA_CC_PMU_CHIPCTL_DATA, mask, val);
}

/* Poll for command completion. Returns zero when complete. */
static int bcm47xxnflash_ops_bcm5357_poll(struct bcma_drv_cc *cc, u32 pollmask)
{
	int i;
	u32 val;
	pollmask = NIST_CTRL_READY | NIST_FLASH_READY;

	for (i = 0; i < NF_RETRIES; i++) {
		val = bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS);

#if BCM5357_POLL_DEBUG == 1
		pr_err("INTFC_ST: 0x%08x\n", val);

		if (val & NIST_ERASED) {
			pr_err("Reading from erased block\n");
			return BRCMNAND_READING_ERASED_BLOCK;
		}

#endif
		/* FIXME: Need to handle when controller informs that block is erased */

		if ((val & pollmask) == pollmask) {
			return 0;
		}
	}

	pr_err("Polling timeout, Intfc status: 0x%08x\n", val);
	return BRCMNAND_TIMED_OUT;
}



/**************************************************
 * R/W
 **************************************************/
static void bcm47xxnflash_ops_bcm5357_read_oob(struct mtd_info *mtd, uint8_t *buf,
					   int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u32 offset;
	u32 *buf32;
	int i;
	int toread;

	buf32 = (u32 *) buf;

	offset = b47n->curr_page_addr << nand_chip->page_shift;

/* #if BCM5357_CMD_DEBUG == 1 */
/* 	pr_err("bcm5357_read_oob, offset: 0x%08x, len: %d\n", offset, len); */
/* #endif */

	mutex_lock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	while (len > 0) {
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset);
		bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR);

		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_SPARE_RD);
		udelay(nand_chip->chip_delay);

		if (bcm47xxnflash_ops_bcm5357_poll(cc, NIST_SPARE_VALID) < 0) {
			pr_err("Failed SPARE_RD\n");
			break;
		}

		toread = min(len, (int) (mtd->oobsize / (mtd->writesize / NFL_SECTOR_SIZE)));
		/* Current theory is that _rd0-3 regs expose the 16 bytes associated with each subpage. Not verified yet */
		for (i = 0; i < toread; i += sizeof(buf32), buf32++) {
			*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD0 + i);
#if BCM5357_DATA_DEBUG == 2
			printk("bcm5357_read_oob, addr: 0x%08x, 0x%08x\n", offset + i, *buf32);
#endif
		}
		offset += NFL_SECTOR_SIZE;
		len -= toread;
	}
	bcm47xxnflash_ops_bcm5357_enable(cc, false);
	mutex_unlock(&b47n->cmd_l);
}

static void bcm47xxnflash_ops_bcm5357_read(struct mtd_info *mtd, uint8_t *buf,
					   int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u32 offset;
	u32 *buf32;
	int i;
	int toread;
	u32 mask;
	int stuck;
	u32 reg;

	stuck = 0;
	mask = NFL_SECTOR_SIZE - 1;
	buf32 = (u32 *) buf;
	offset = (b47n->curr_page_addr << (nand_chip->page_shift)) | b47n->curr_column;

#if BCM5357_CMD_DEBUG == 1
	pr_err("bcm5357_read command, offset: 0x%08x, len: %d\n", offset, len);
#endif

	/* FIXME: Possible improvements are to allow non-aligned reads */
	if (offset & mask) {
		pr_err("Tried perform a non-aligned read\n");
		return;
	}

	if (len & mask) {
		pr_err("Tried to read an illegal length\n");
		return;
	}

	mutex_lock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	while (len > 0) {
		toread = min(len, NFL_SECTOR_SIZE);

#if BCM5357_DATA_DEBUG == 1
		pr_err("New read, offset: 0x%08x, toread: %d\n", offset, toread);
#endif
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset);
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, NIST_CACHE_VALID) < 0) {
			pr_err("Failed PAGE_RD\n");
			break;
		}

		bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_ADDR, 0);

		*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA);

		/* Awful hack, but sometimes reads fail for unknown reason, just repeat for now */
		if (*buf32 == 0x3c12b800) {
			stuck++;
			if (stuck >= 10) {
				panic("Read is stuck\n");
			}
			continue;
		}
		stuck = 0;

#if BCM5357_DATA_DEBUG == 1
			printk("b%08x %08x\n", offset, *buf32);
#endif

		for (i = 4; i < toread; i += sizeof(buf32), buf32++) {
			*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA);

#if BCM5357_DATA_DEBUG == 1
			printk("b%08x %08x\n", offset + i, *buf32);
#endif
		}

		len -= toread;
		offset += toread;
	}
	bcm47xxnflash_ops_bcm5357_enable(cc, false);
	mutex_unlock(&b47n->cmd_l);
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
	u32 mask;

	mask = NFL_SECTOR_SIZE - 1;
        offset = (b47n->curr_page_addr << (nand_chip->page_shift)) | b47n->curr_column;
	from = (u32 *) buf;

#if BCM5357_CMD_DEBUG == 1
	pr_err("bcm5357_write, offset: 0x%08llx, len: %d\n", offset, len);
#endif

	if ((offset & mask) > 0) {
		pr_err("Invalid address offset, must be page-aligned\n");
		return;
	}

	if ((len & mask) > 0) {
		pr_err("Must write full pages\n");
		return;
	}

	mutex_lock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	/* Disable partial page programming */
	reg = bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL);
	reg &= ~NAC_PARTIAL_PAGE_EN;
	bcma_cc_write32(cc, BCMA_CC_NAND_ACC_CONTROL, reg);

	while (len > 0) {
		towrite = min(len, (int) mtd->writesize);
		bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_ADDR, 0);
		/* Transfer data to nand controller cache */
		for (i = 0; i < towrite; i += sizeof(from), from++) {
			/* Reset cmd addr for each sector */
			/* I don't fully understand this logic, should not be needed */
			if (i % NFL_SECTOR_SIZE == 0) {
				bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, i);
				bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR);

			}

			bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_DATA, *from);
		}

		/* Set cmd address to end of page - last sector size */
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset + mtd->writesize - NFL_SECTOR_SIZE);
		bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR);

		pr_err("Page programming disabled for now\n");
		break;

		/* bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_PROG); */
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed PAGE_PROG\n");
			break;
		}
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_STATUS_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed STATUS_RD after PAGE_PROG\n");
			break;
		}

		b47n->curr_column += towrite;
		len -= towrite;
		offset += towrite;
	}
	bcm47xxnflash_ops_bcm5357_enable(cc, false);
	mutex_unlock(&b47n->cmd_l);
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

#if BCM5357_CMD_DEBUG == 1
	pr_err("bcm5357_cmd_ctrl, cmd: 0x%02x\n", cmd);
#endif
	bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, cmd);
}

static int bcm47xxnflash_ops_bcm5357_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u32 val;
	u32 ret;

	ret = 0;
	mutex_lock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_STATUS_RD);

	val = bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS);

	if ((val & (NIST_CTRL_READY | NIST_FLASH_READY)) ==
	    (NIST_CTRL_READY | NIST_FLASH_READY)) {
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_dev_ready, device is ready\n");
#endif

	    ret = ~0;
	} else {
#if BCM5357_CMD_DEBUG == 1
	pr_err("bcm5357_dev_ready, device is NOT ready: 0x%08x\n", val);
#endif
	}

	mutex_unlock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, false);

        return ret;
}

static void bcm47xxnflash_ops_bcm5357_erase(struct mtd_info *mtd,
					   int column,
					   int page_addr)
{
	u32 block_mask;
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;
	u32 block;

	if (column != -1) {
		pr_err("Column address not 0 when trying to erase\n");
		return;
	}

	block = page_addr >> (nand_chip->phys_erase_shift - nand_chip->page_shift);
	block_mask = nand_chip->phys_erase_shift - 1;

	if ((page_addr & block_mask) > 0) {
		pr_err("Page address is not aligned to block start when erasing\n");
		return;
	}

        bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, page_addr << nand_chip->page_shift);
	bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR);

	if (block != 1023) {
		panic("Tried to erase block: %u\n", block);
	}
	bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_BLOCK_ERASE);
	if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
		pr_err("Failed to erase block: %d\n", block);
	}
	if (bcm47xxnflash_ops_bcm5357_poll(cc, NAND_STATUS_READY) < 0) {
		pr_err("Failed to check for block erase ok: %d\n", block);
	}
}

static void bcm47xxnflash_ops_bcm5357_cmdfunc(struct mtd_info *mtd,
					      unsigned command, int column,
					      int page_addr)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;
	u32 reg;

	mutex_lock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	b47n->curr_command = command;

	if (column != -1)
		b47n->curr_column = column;
	if (page_addr != -1)
		b47n->curr_page_addr = page_addr;

	switch (command) {
	case NAND_CMD_RESET:
#if BCM5357_CMD_DEBUG == 1
		pr_err("bcm5357_cmdfunc, NAND_CMD_RESET, col: %d, page: %d\n", column, page_addr);
#endif

		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_FLASH_RESET);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed FLASH_RESET\n");
		}
		break;
	case NAND_CMD_READID:
#if BCM5357_CMD_DEBUG == 1
		pr_err("bcm5357_cmdfunc, NAND_CMD_READID, col: %d, page: %d\n", column, page_addr);
#endif
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_ID_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed to read id\n");
			bcm47xxnflash_ops_bcm5357_enable(cc, false);
			break;
		}

		/* FIXME: Can surely be optimized with some casting cleverness */
		reg = bcma_cc_read32(cc, BCMA_CC_NAND_DEVID);
		memcpy(b47n->id_data, &reg, sizeof(reg));
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READID, DEVID: 0x%08x\n", reg);
#endif

	        reg = bcma_cc_read32(cc, BCMA_CC_NAND_DEVID_X);
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READID, DEVID_X: 0x%08x\n", reg);
#endif

		memcpy(b47n->id_data + sizeof(reg), &reg, sizeof(reg));
		break;

	case NAND_CMD_STATUS:
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_STATUS, col: %d, page: %d\n", column, page_addr);
#endif
		break;
	case NAND_CMD_READ0:
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READ0, col: %d, page: %d\n", column, page_addr);
#endif

		break;
	case NAND_CMD_READOOB:
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READOOB, col: %d, page: %d, block: %d\n", column, page_addr, page_addr >> (nand_chip->phys_erase_shift - nand_chip->page_shift));
#endif

		break;
	case NAND_CMD_ERASE1:
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_ERASE1, block: %d\n", page_addr >> (nand_chip->phys_erase_shift - nand_chip->page_shift));
#endif
		bcm47xxnflash_ops_bcm5357_erase(mtd, column, page_addr);
		break;

	case NAND_CMD_ERASE2:
		break;
	case NAND_CMD_SEQIN:
#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_SEQIN, col: %d, page: %d\n", column, page_addr);
#endif

		bcm47xxnflash_ops_bcm5357_calc_and_set_offset(b47n, page_addr, column);

		if (column >= mtd->writesize) {
			bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READOOB);
		} else {
	                bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READ0);
		}

		break;
	case NAND_CMD_PAGEPROG:

#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_PAGEPROG, col: %d, page: %d\n", column, page_addr);
#endif

		/* FIXME: Should we set page offset here */
		/* bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_PROG); */
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("PAGE_PROG failed\n");
		}

		break;
	default:
		pr_err("Command 0x%X unsupported\n", command);
		break;
	}

	bcm47xxnflash_ops_bcm5357_enable(cc, false);
	mutex_unlock(&b47n->cmd_l);

}

static u8 bcm47xxnflash_ops_bcm5357_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);
	struct bcma_drv_cc *cc = b47n->cc;

	u8 data;
	u32 tmp;

	mutex_lock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	data = 0;
	switch (b47n->curr_command) {
	case NAND_CMD_READID:
		if (b47n->curr_column >= ARRAY_SIZE(b47n->id_data)) {
			pr_err("Requested invalid id_data: %d\n",
			       b47n->curr_column);
			break;
		}
		data = b47n->id_data[b47n->curr_column++];
#if BCM5357_CMD_DEBUG == 1
		pr_err("bcm5357_read_byte, NAND_CMD_READID, read col: %d, data: 0x%02x\n", b47n->curr_column-1, data);
#endif
		break;

	case NAND_CMD_STATUS:
		/* TODO: Add write protect bit checking routine */
		/* NAND_STATUS_WP is active low */
		data = NAND_STATUS_WP;
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) == 0) {
			data |= NAND_STATUS_READY;
		} else {
			data |= NAND_STATUS_FAIL;
		}

#if BCM5357_CMD_DEBUG
		pr_err("bcm5357_read_byte, NAND_CMD_STATUS: 0x%02x\n", data);
#endif

		break;

	case NAND_CMD_READOOB:
		bcm47xxnflash_ops_bcm5357_read_oob(mtd, (u8 *)&tmp, 4);
		data = tmp & 0xFF;
#if BCM5357_CMD_DEBUG == 1
		pr_err("bcm5357_read_byte, NAND_CMD_READOOB, page: %d, data: 0x%02x\n", nand_chip->page_shift, data);
#endif

		break;
	default:
		pr_err("Invalid command for byte read: 0x%X\n", b47n->curr_command);
	}

	mutex_unlock(&b47n->cmd_l);
	bcm47xxnflash_ops_bcm5357_enable(cc, false);

	return data;
}

static void bcm47xxnflash_ops_bcm5357_read_buf(struct mtd_info *mtd,
					       uint8_t *buf, int len)
{
	struct nand_chip *nand_chip = mtd_to_nand(mtd);
	struct bcm47xxnflash *b47n = nand_get_controller_data(nand_chip);

	switch (b47n->curr_command) {
	case NAND_CMD_READ0:
		bcm47xxnflash_ops_bcm5357_read(mtd, buf, len);
		break;
	case NAND_CMD_READOOB:
		bcm47xxnflash_ops_bcm5357_read_oob(mtd, buf, len);
		break;

	default:
		pr_err("Invalid command for read buf: 0x%X\n", b47n->curr_command);
	}
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
	u32 reg;

	pr_info("Initializing bcm5357 NAND controller\n");
	err = 0;

	mutex_init(&b47n->cmd_l);

	nand_chip->cmd_ctrl = bcm47xxnflash_ops_bcm5357_cmd_ctrl;
	nand_chip->dev_ready = bcm47xxnflash_ops_bcm5357_dev_ready;
	nand_chip->cmdfunc = bcm47xxnflash_ops_bcm5357_cmdfunc;
	nand_chip->read_buf = bcm47xxnflash_ops_bcm5357_read_buf;
	nand_chip->write_buf = bcm47xxnflash_ops_bcm5357_write_buf;
        nand_chip->read_byte = bcm47xxnflash_ops_bcm5357_read_byte;
	nand_chip->onfi_set_features = nand_onfi_get_set_features_notsupp;
	nand_chip->onfi_get_features = nand_onfi_get_set_features_notsupp;

	/* As per K9F1G08U0D data sheet Rev 0.0 Dec 9 2009, page 13 tR */
	nand_chip->chip_delay = 35;
	nand_chip->bbt_options = NAND_BBT_USE_FLASH;

	/* FIXME: Need to add ECC checks */
	nand_chip->ecc.mode = NAND_ECC_NONE;

	/* reg = bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL); */
	/* pr_err("BCMA_CC_NAND_ACC_CONTROL: 0x%08x\n", reg); */

	/* reg &= 0xFFFF0000; */
	/* Setting to 0x40 does not work. Set to closest */
	/* Is this per subpage, if so original value is correct */
	/* reg |= (0x3F << NAC_SPARE_SIZE0_SHIFT) | (0x3F << NAC_SPARE_SIZE_SHIFT); */
	/* pr_err("BCMA_CC_NAND_ACC_CONTROL: 0x%08x\n", reg); */
	/* bcma_cc_write32(cc, BCMA_CC_NAND_ACC_CONTROL, reg); */

	/* reg = bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL); */
	/* pr_err("BCMA_CC_NAND_ACC_CONTROL: 0x%08x\n", reg); */

	/* bcma_cc_write32(cc, BCMA_CC_NAND_CS_NAND_XOR, 0); */

	/* Scan NAND */
	err = nand_scan(nand_to_mtd(nand_chip), 1);

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
