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
/* Below is assumed */
#define NIST_FLASH_STATUS_ERROR         0x00000001

#define NF_RETRIES	                10000
#define NFL_SECTOR_SIZE			512

#define BCM5357_CMD_DEBUG 1
#define BCM5357_DATA_DEBUG 1

#define BRCMNAND_FLASH_STATUS_ERROR         (-2)
#define BRCMNAND_TIMED_OUT                  (-3)


/* K9F1G08XD has 64K pages = 1024 blocks */
/* 1 Block is 64 pages (128k + 4k), 1 Page = 2K + 64 bytes */
/* 65536 pages i.e 16 bits */
/* Column address is A0 - A11 = 12 bits = 2**12 = 4096 bytes */
/* Highest bit 12 is used to access spare bits */
/* Row address is A12 - A27 = 16 bits */

/* Column is within a page */
/* Page addr is all pages */

/**************************************************
 * Various helpers
 **************************************************/
static void bcm47xxnflash_ops_bcm5357_dump_regs(struct bcma_drv_cc *cc)
{
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_REVISION, bcma_cc_read32(cc, BCMA_CC_NAND_REVISION));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_START, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_START));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CMD_END_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_CMD_END_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CS_NAND_SELECT, bcma_cc_read32(cc, BCMA_CC_NAND_CS_NAND_SELECT));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CS_NAND_XOR, bcma_cc_read32(cc, BCMA_CC_NAND_CS_NAND_XOR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD0, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD0));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD4, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD4));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD8, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD8));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD12, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD12));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR0, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR0));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR4, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR4));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR8, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR8));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_WR12, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_WR12));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_ACC_CONTROL, bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CONFIG, bcma_cc_read32(cc, BCMA_CC_NAND_CONFIG));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_1, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_1));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_2, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_2));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SEMAPHORE, bcma_cc_read32(cc, BCMA_CC_NAND_SEMAPHORE));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_DEVID, bcma_cc_read32(cc, BCMA_CC_NAND_DEVID));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_DEVID_X, bcma_cc_read32(cc, BCMA_CC_NAND_DEVID_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLOCK_LOCK_STATUS, bcma_cc_read32(cc, BCMA_CC_NAND_BLOCK_LOCK_STATUS));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_INTFC_STATUS, bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_CORR_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_CORR_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_CORR_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_CORR_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_UNC_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_UNC_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_ECC_UNC_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_ECC_UNC_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_READ_ERROR_COUNT, bcma_cc_read32(cc, BCMA_CC_NAND_READ_ERROR_COUNT));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CORR_STAT_THRESHOLD, bcma_cc_read32(cc, BCMA_CC_NAND_CORR_STAT_THRESHOLD));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_READ_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_READ_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_READ_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_READ_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_PAGE_PROGRAM_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_PAGE_PROGRAM_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_PAGE_PROGRAM_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_PAGE_PROGRAM_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_COPY_BACK_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_COPY_BACK_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_COPY_BACK_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_COPY_BACK_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLOCK_ERASE_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_BLOCK_ERASE_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLOCK_ERASE_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_BLOCK_ERASE_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_INV_READ_ADDR_X, bcma_cc_read32(cc, BCMA_CC_NAND_INV_READ_ADDR_X));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_INV_READ_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_INV_READ_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_BLK_WR_PROTECT, bcma_cc_read32(cc, BCMA_CC_NAND_BLK_WR_PROTECT));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_ACC_CONTROL_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL_CS1));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CONFIG_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_CONFIG_CS1));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_1_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_1_CS1));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_TIMING_2_CS1, bcma_cc_read32(cc, BCMA_CC_NAND_TIMING_2_CS1));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD16, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD16));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD20, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD20));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD24, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD24));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_SPARE_RD28, bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD28));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CACHE_ADDR, bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_ADDR));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CACHE_DATA, bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CTRL_CONFIG, bcma_cc_read32(cc, BCMA_CC_NAND_CTRL_CONFIG));
	pr_err("0x%08x, 0x%08x\n", BCMA_CC_NAND_CTRL_STATUS, bcma_cc_read32(cc, BCMA_CC_NAND_CTRL_STATUS));
}

static inline int bcm47xxnflash_ops_bcm5357_ctl_cmd(struct bcma_drv_cc *cc, u32 code)
{
	bcma_cc_write32(cc, BCMA_CC_NAND_CMD_START, code);
	/* Read after write to flush the command */
	bcma_cc_read32(cc, BCMA_CC_NAND_CMD_START);

	__sync();
	return 0;
}

static inline void bcm47xxnflash_ops_bcm5357_calc_and_set_offset(struct bcm47xxnflash *b47n, int page_addr, int column) {
	/* Do we need to account for spare area? */
        bcma_cc_write32(b47n->cc, BCMA_CC_NAND_CMD_ADDR, (page_addr << (b47n->nand_chip.page_shift + 1)) | column);
}

static void bcm47xxnflash_ops_bcm5357_enable(struct bcma_drv_cc *cc, bool enable)
{
	u32 mask;
	u32 val;
	u32 reg;

	mask = ~(BCMA_CHIPCTL_5357_NFLASH);
	val = 0;
	if (enable) {
		val = BCMA_CHIPCTL_5357_NFLASH;
	}

	bcma_pmu_write32(cc, BCMA_CC_PMU_CHIPCTL_ADDR, 1);

#ifdef BCM5357_CMD_DEBUG
	reg = bcma_pmu_read32(cc, BCMA_CC_PMU_CHIPCTL_DATA);
	pr_err("Pre enable write: bcm5357_enable: 0x%08x\n", reg);
#endif

	bcma_pmu_maskset32(cc, BCMA_CC_PMU_CHIPCTL_DATA, mask, val);

#ifdef BCM5357_CMD_DEBUG
	reg = bcma_pmu_read32(cc, BCMA_CC_PMU_CHIPCTL_DATA);
	pr_err("Post enable write: bcm5357_enable: 0x%08x\n", reg);
#endif
}

/* Poll for command completion. Returns zero when complete. */
static int bcm47xxnflash_ops_bcm5357_poll(struct bcma_drv_cc *cc, u32 pollmask)
{
	int i;
	u32 val;
	pollmask |= NIST_CTRL_READY | NIST_FLASH_READY;

	for (i = 0; i < NF_RETRIES; i++) {
		val = bcma_cc_read32(cc, BCMA_CC_NAND_INTFC_STATUS);

		if ((val & pollmask) == pollmask) {
			return 0;
		}

		if (val & NIST_FLASH_STATUS_ERROR) {
			pr_err("Flash status error\n");
			return BRCMNAND_FLASH_STATUS_ERROR;
		}
		udelay(1);
	}

	pr_err("Polling timeout!\n");
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
	offset = (b47n->curr_page_addr << (b47n->nand_chip.page_shift + 1)) | b47n->curr_column;

#ifdef BCM5357_CMD_DEBUG
	pr_err("bcm5357_read_oob, offset: 0x%08x, len: %d\n", offset, len);
#endif

	bcm47xxnflash_ops_bcm5357_enable(cc, true);

	while (len > 0) {
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR_X, 0);
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset);

		__sync();
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_SPARE_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, NIST_SPARE_VALID) < 0) {
			pr_err("Failed SPARE_RD\n");
			break;
		}

		__sync();

		toread = min(len, (int) mtd->oobsize);
		for (i = 0; i < toread; i += 4, buf32++) {
			if (i < 16) {
				*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD0 + i);
			} else if (i < 32) {
				*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_SPARE_RD16 + (i - 16));
			}
#ifdef BCM5357_DATA_DEBUG
			pr_err("bcm5357_read_oob, addr: 0x%08x, 0x%08x\n", offset + i, *buf32);
#endif
		}
		offset += toread;
		len -= toread;

	}
	bcm47xxnflash_ops_bcm5357_enable(cc, false);

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
	/* u32 reg; */
	u32 mask;

	mask = NFL_SECTOR_SIZE - 1;
	buf32 = (u32 *) buf;
	offset = (b47n->curr_page_addr << (b47n->nand_chip.page_shift + 1)) | b47n->curr_column;

#ifdef BCM5357_CMD_DEBUG
	pr_err("bcm5357_read command, offset: 0x%08x, len: %d\n", offset, len);
#endif

	if (offset & mask) {
		pr_err("bcm5357: Tried perform a non-aligned read\n");
		return;
	}


	offset = 0x00400000;


	bcm47xxnflash_ops_bcm5357_enable(cc, true);
	while (len > 0) {
		toread = min(len, NFL_SECTOR_SIZE);

		pr_err("New read, offset: 0x%08x, toread: %d\n", offset, toread);
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset);

		__sync();
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, NIST_CACHE_VALID) < 0) {
			pr_err("Failed PAGE_RD\n");
			break;
		}

		bcma_cc_write32(cc, BCMA_CC_NAND_CACHE_ADDR, 0);

		*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA);

		if (*buf32 == 0x3c12b800) {
			pr_err("Read failed, retrying\n");
			continue;
		}

		__sync();
		for (i = 4; i < toread; i += sizeof(buf32), buf32++) {
			*buf32 = bcma_cc_read32(cc, BCMA_CC_NAND_CACHE_DATA);


#ifdef BCM5357_DATA_DEBUG
			/* pr_err("bcm5357_read: A: 0x%08x, 0x%08x\n", offset + i, *buf32); */
#endif
		}
		len -= toread;
		/* offset += toread; */
		offset += 0x00100000;
	}
	bcm47xxnflash_ops_bcm5357_enable(cc, false);

	/* bcm47xxnflash_ops_bcm5357_dump_regs(cc); */

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

        offset = (b47n->curr_page_addr << (b47n->nand_chip.page_shift + 1)) | b47n->curr_column;
	from = (u32 *) buf;

#ifdef BCM5357_CMD_DEBUG
	pr_err("bcm5357_write, offset: 0x%08llx, len: %d\n", offset, len);
#endif

	/* Disable partial page programming */
	reg = bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL);
	reg &= ~NAC_PARTIAL_PAGE_EN;
	bcma_cc_write32(cc, BCMA_CC_NAND_ACC_CONTROL, reg);

	bcm47xxnflash_ops_bcm5357_enable(cc, true);
	while (len > 0) {
		towrite = min(len, (int) mtd->writesize);
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
		bcma_cc_write32(cc, BCMA_CC_NAND_CMD_ADDR, offset + mtd->writesize - NFL_SECTOR_SIZE);
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

#ifdef BCM5357_CMD_DEBUG
	pr_err("bcm5357_cmd_ctl, cmd: 0x%02x\n", cmd);
#endif
	bcm47xxnflash_ops_bcm5357_enable(cc, true);
	bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, cmd);
	bcm47xxnflash_ops_bcm5357_enable(cc, false);
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
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_dev_ready, device is ready\n");
#endif

	    return ~0;
	}
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_dev_ready, device is NOT ready\n");
#endif

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
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_RESET, col: %d, page: %d\n", column, page_addr);
#endif

		/* FIXME: Double check this */
		bcm47xxnflash_ops_bcm5357_enable(cc, true);
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_FLASH_RESET);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed FLASH_RESET\n");
		}
		bcm47xxnflash_ops_bcm5357_enable(cc, false);

		break;
	case NAND_CMD_READID:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READID, col: %d, page: %d\n", column, page_addr);
#endif

		bcm47xxnflash_ops_bcm5357_enable(cc, true);
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_ID_RD);
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed to read id\n");
			bcm47xxnflash_ops_bcm5357_enable(cc, false);
			break;
		}

		/* FIXME: Can surely be optimized with some casting cleverness */
		reg = bcma_cc_read32(cc, BCMA_CC_NAND_DEVID);
		memcpy(b47n->id_data, &reg, sizeof(reg));
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READ_BYTE, DEVID: 0x%08x\n", reg);
#endif

	        reg = bcma_cc_read32(cc, BCMA_CC_NAND_DEVID_X);
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READ_BYTE, DEVID_X: 0x%08x\n", reg);
#endif

		memcpy(b47n->id_data + sizeof(reg), &reg, sizeof(reg));
		bcm47xxnflash_ops_bcm5357_enable(cc, false);

		break;
	case NAND_CMD_STATUS:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_STATUS, col: %d, page: %d\n", column, page_addr);
#endif

		bcm47xxnflash_ops_bcm5357_enable(cc, true);
		bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_STATUS_RD);
		bcm47xxnflash_ops_bcm5357_enable(cc, false);

		break;
	case NAND_CMD_READ0:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READ0, col: %d, page: %d\n", column, page_addr);
#endif

		/* FIXME: Nop this as the 4706, no clue what to do */
		break;
	case NAND_CMD_READOOB:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READOOB, col: %d, page: %d, inc curr_column by: %d\n", column, page_addr, mtd->writesize);
#endif

		if (page_addr != -1)
			b47n->curr_column += mtd->writesize;
		break;
	case NAND_CMD_ERASE1:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_ERASE1, col: %d, page: %d\n", column, page_addr);
#endif

		/* FIXME: Do we need to check for non block aligned offsets */
		bcm47xxnflash_ops_bcm5357_calc_and_set_offset(b47n, page_addr, column);
                pr_err("BLOCK_ERASE disabled for now\n");

		bcm47xxnflash_ops_bcm5357_enable(cc, true);

                /* bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_BLOCK_ERASE); */

		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("Failed BLOCK_ERASE cmd\n");
		}
		bcm47xxnflash_ops_bcm5357_enable(cc, false);

	case NAND_CMD_ERASE2:
		break;
	case NAND_CMD_SEQIN:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_SEQIN, col: %d, page: %d\n", column, page_addr);
#endif

		bcm47xxnflash_ops_bcm5357_calc_and_set_offset(b47n, page_addr, column);

		/* FIXME: Double check this */
		bcm47xxnflash_ops_bcm5357_enable(cc, true);
		if (column >= mtd->writesize) {
			/* OOB area */
			column -= mtd->writesize;

                        if (bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READOOB) < 0) {
			           pr_err("SEQIN, READ00B failed\n");
                        }
		} else {
	                if (bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NAND_CMD_READ0) < 0) {
			           pr_err("SEQIN, READ failed\n");
                        }
		}
		bcm47xxnflash_ops_bcm5357_enable(cc, false);
		break;
	case NAND_CMD_PAGEPROG:
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_PAGEPROG, col: %d, page: %d\n", column, page_addr);
#endif

		/* FIXME: Should we set page offset here */
		bcm47xxnflash_ops_bcm5357_enable(cc, true);
		/* bcm47xxnflash_ops_bcm5357_ctl_cmd(cc, NCMD_PAGE_PROG); */
		if (bcm47xxnflash_ops_bcm5357_poll(cc, 0) < 0) {
			pr_err("PAGE_PROG failed\n");
		}
		bcm47xxnflash_ops_bcm5357_enable(cc, false);
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

#ifdef BCM5357_CMD_DEBUG
	pr_err("bcm5357_cmdfunc, NAND_CMD_READ_BYTE, curr_command: 0x%x\n", b47n->curr_command);
#endif

	data = 0;
	switch (b47n->curr_command) {
	case NAND_CMD_READID:
		if (b47n->curr_column >= ARRAY_SIZE(b47n->id_data)) {
			pr_err("Requested invalid id_data: %d\n",
			       b47n->curr_column);
			break;
		}
		data = b47n->id_data[b47n->curr_column++];
#ifdef BCM5357_CMD_DEBUG
		pr_err("bcm5357_cmdfunc, NAND_CMD_READ_BYTE, read col: %d, data: 0x%02x\n", b47n->curr_column -1, data);
#endif
		break;

	/* FIXME: No clue why this is done this way, just mimicking the 4706 variant */
	case NAND_CMD_STATUS:
	case NAND_CMD_READOOB:
		bcm47xxnflash_ops_bcm5357_read_oob(mtd, (u8 *)&tmp, 4);
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
	int i;

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
	nand_chip->chip_delay = 50;

	b47n->nand_chip.bbt_options = NAND_BBT_USE_FLASH;

	/* FIXME: Just hoping for the best */
	b47n->nand_chip.ecc.mode = NAND_ECC_NONE;

	/* FIXME: Disable read ecc for now */
	/* reg = bcma_cc_read32(cc, BCMA_CC_NAND_ACC_CONTROL); */
	/* reg &= ~NAC_RD_ECC_EN; */
	/* bcma_cc_write32(cc, BCMA_CC_NAND_ACC_CONTROL, reg); */

	/* FIXME: Must enable ecc configuration here */
	/* bcm47xxnflash_ops_bcm5357_enable(cc, false); */
	/* bcm47xxnflash_ops_bcm5357_enable(cc, true); */

	/* bcm47xxnflash_ops_bcm5357_dump_regs(cc); */

	/* for (i = 0; i < 0xFFFF; i += 4) { */
	/* 	reg = bcma_cc_read32(cc, i); */
	/* 	pr_err("0x%08X, 0x%08x\n", i, reg); */
        /* } */

	/* Scan NAND */
	pr_err("Scanning nand\n");
	err = nand_scan(nand_to_mtd(&b47n->nand_chip), 1);

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
