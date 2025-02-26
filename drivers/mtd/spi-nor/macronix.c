// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

#define SPINOR_OP_READ_CR2		0x71	/* Read Configuration Register 2 */
#define SPINOR_OP_WRITE_CR2		0x72	/* Write Configuration Register 2 */
#define SPINOR_OP_MX_DTR_RD		0xee	/* Octa DTR Read Opcode */

#define SPINOR_REG_CR2_MODE_ADDR	0	/* Address of Mode Enable in CR2 */
#define SPINOR_REG_CR2_DTR_OPI_ENABLE	BIT(1)	/* DTR OPI Enable */
#define SPINOR_REG_CR2_SPI		0	/* SPI Enable */

/**
 * Macronix SPI NOR flash operations.
 */
#define SPI_NOR_MX_READ_CR2_OP(ndummy, buf, ndata)			\
	SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_READ_CR2, 0),		\
		   SPI_MEM_OP_ADDR(4, SPINOR_REG_CR2_MODE_ADDR, 0),	\
		   SPI_MEM_OP_DUMMY(ndummy, 0),				\
		   SPI_MEM_OP_DATA_IN(ndata, buf, 0))

#define SPI_NOR_MX_WRITE_CR2_OP(buf, ndata)				\
	SPI_MEM_OP(SPI_MEM_OP_CMD(SPINOR_OP_WRITE_CR2, 0),		\
		   SPI_MEM_OP_ADDR(4, SPINOR_REG_CR2_MODE_ADDR, 0),	\
		   SPI_MEM_OP_NO_DUMMY,					\
		   SPI_MEM_OP_DATA_OUT(ndata, buf, 0))

static int spi_nor_macronix_read_cr2(struct spi_nor *nor, u8 ndummy, void *sr,
				     unsigned int nbytes)
{
	struct spi_mem_op op = SPI_NOR_MX_READ_CR2_OP(ndummy, sr, nbytes);

	spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
	return spi_mem_exec_op(nor->spimem, &op);
}

static int spi_nor_macronix_write_cr2(struct spi_nor *nor, const void *sr,
				      unsigned int nbytes)
{
	struct spi_mem_op op = SPI_NOR_MX_WRITE_CR2_OP(sr, nbytes);
	int ret;

	spi_nor_spimem_setup_op(nor, &op, nor->reg_proto);
	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;
	return spi_mem_exec_op(nor->spimem, &op);
}

static int spi_nor_macronix_octal_dtr_en(struct spi_nor *nor)
{
	u8 *buf = nor->bouncebuf;
	int i, ret;

	buf[0] = SPINOR_REG_CR2_DTR_OPI_ENABLE;
	ret = spi_nor_macronix_write_cr2(nor, buf, 1);
	if (ret)
		return ret;

	/* Read flash ID to make sure the switch was successful. */
	ret = spi_nor_read_id(nor, 4, 4, buf, SNOR_PROTO_8_8_8_DTR);
	if (ret)
		return ret;

	for (i = 0; i < nor->info->id->len; i++)
		if (buf[i * 2] != nor->info->id->bytes[i])
			return -EINVAL;
	return 0;
}

static int spi_nor_macronix_octal_dtr_dis(struct spi_nor *nor)
{
	u8 *buf = nor->bouncebuf;
	int ret;

	/*
	 * One byte transactions are not allowed in 8D-8D-8D mode. mx66lm1g45g
	 * requires that undefined register addresses to keep their value
	 * unchanged. Its second CR2 byte value is not defined. Read the second
	 * byte value of CR2 so that we can write it back when disabling
	 * Octal DTR mode.
	 */
	ret = spi_nor_macronix_read_cr2(nor, 4, buf, 2);
	if (ret)
		return ret;
	/* Keep the value of buf[1] unchanged.*/
	buf[0] = SPINOR_REG_CR2_SPI;

	ret = spi_nor_macronix_write_cr2(nor, buf, 2);
	if (ret)
		return ret;

	ret = spi_nor_read_id(nor, 0, 0, buf, nor->reg_proto);
	if (ret)
		return ret;

	if (memcmp(buf, nor->info->id, nor->info->id->len)) {
		dev_dbg(nor->dev, "Failed to disable 8D-8D-8D mode.\n");
		return -EINVAL;
	}
	return 0;
}

static int spi_nor_macronix_set_octal_dtr(struct spi_nor *nor, bool enable)
{
	return enable ? spi_nor_macronix_octal_dtr_en(nor) :
			spi_nor_macronix_octal_dtr_dis(nor);
}

static int mx66lm1g45g_late_init(struct spi_nor *nor)
{
	nor->params->set_octal_dtr = spi_nor_macronix_set_octal_dtr;

	/* Set the Fast Read settings. */
	nor->params->hwcaps.mask |= SNOR_HWCAPS_READ_8_8_8_DTR;
	spi_nor_set_read_settings(&nor->params->reads[SNOR_CMD_READ_8_8_8_DTR],
				  0, 20, SPINOR_OP_MX_DTR_RD,
				  SNOR_PROTO_8_8_8_DTR);

	nor->cmd_ext_type = SPI_NOR_EXT_INVERT;
	nor->params->rdsr_dummy = 4;
	nor->params->rdsr_addr_nbytes = 4;

	return 0;
}

static struct spi_nor_fixups mx66lm1g45g_fixups = {
	.late_init = mx66lm1g45g_late_init,
};

static int
mx25l25635_post_bfpt_fixups(struct spi_nor *nor,
			    const struct sfdp_parameter_header *bfpt_header,
			    const struct sfdp_bfpt *bfpt)
{
	/*
	 * MX25L25635F supports 4B opcodes but MX25L25635E does not.
	 * Unfortunately, Macronix has re-used the same JEDEC ID for both
	 * variants which prevents us from defining a new entry in the parts
	 * table.
	 * We need a way to differentiate MX25L25635E and MX25L25635F, and it
	 * seems that the F version advertises support for Fast Read 4-4-4 in
	 * its BFPT table.
	 */
	if (bfpt->dwords[SFDP_DWORD(5)] & BFPT_DWORD5_FAST_READ_4_4_4)
		nor->flags |= SNOR_F_4B_OPCODES;

	return 0;
}

static const struct spi_nor_fixups mx25l25635_fixups = {
	.post_bfpt = mx25l25635_post_bfpt_fixups,
};

static const struct flash_info macronix_nor_parts[] = {
	{
		.id = SNOR_ID(0xc2, 0x20, 0x10),
		.name = "mx25l512e",
		.size = SZ_64K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x12),
		.name = "mx25l2005a",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x13),
		.name = "mx25l4005a",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x14),
		.name = "mx25l8005",
		.size = SZ_1M,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x15),
		.name = "mx25l1606e",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x16),
		.name = "mx25l3205d",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x17),
		.name = "mx25l6405d",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x18),
		.name = "mx25l12805d",
		.size = SZ_16M,
		.flags = SPI_NOR_HAS_LOCK | SPI_NOR_4BIT_BP,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x19),
		.name = "mx25l25635e",
		.size = SZ_32M,
		.no_sfdp_flags = SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixups = &mx25l25635_fixups
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x1a),
		.name = "mx66l51235f",
		.size = SZ_64M,
		.no_sfdp_flags = SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x20, 0x1b),
		.name = "mx66l1g45g",
		.size = SZ_128M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x23, 0x14),
		.name = "mx25v8035f",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x32),
		.name = "mx25u2033e",
		.size = SZ_256K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x33),
		.name = "mx25u4035",
		.size = SZ_512K,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x34),
		.name = "mx25u8035",
		.size = SZ_1M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x36),
		.name = "mx25u3235f",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x37),
		.name = "mx25u6435f",
		.size = SZ_8M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x38),
		.name = "mx25u12835f",
		.size = SZ_16M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x39),
		.name = "mx25u25635f",
		.size = SZ_32M,
		.no_sfdp_flags = SECT_4K,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3a),
		.name = "mx25u51245g",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3a),
		.name = "mx66u51235f",
		.size = SZ_64M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x25, 0x3c),
		.name = "mx66u2g45g",
		.size = SZ_256M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
		.fixup_flags = SPI_NOR_4B_OPCODES,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x18),
		.name = "mx25l12855e",
		.size = SZ_16M,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x19),
		.name = "mx25l25655e",
		.size = SZ_32M,
	}, {
		.id = SNOR_ID(0xc2, 0x26, 0x1b),
		.name = "mx66l1g55g",
		.size = SZ_128M,
		.no_sfdp_flags = SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x28, 0x15),
		.name = "mx25r1635f",
		.size = SZ_2M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x28, 0x16),
		.name = "mx25r3235f",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ,
	}, {
		.id = SNOR_ID(0xc2, 0x81, 0x3a),
		.name = "mx25uw51245g",
		.n_banks = 4,
		.flags = SPI_NOR_RWW,
	}, {
		.id = SNOR_ID(0xc2, 0x9e, 0x16),
		.name = "mx25l3255e",
		.size = SZ_4M,
		.no_sfdp_flags = SECT_4K,
	}, {
		.id = SNOR_ID(0xc2, 0x85, 0x3b),
		.name = "mx66lm1g45g",
		.size = SZ_128M,
		.no_sfdp_flags = SPI_NOR_SKIP_SFDP | SECT_4K | SPI_NOR_OCTAL_DTR_READ | SPI_NOR_OCTAL_DTR_PP | SPI_NOR_DTR_BSWAP16,
		.fixup_flags = SPI_NOR_4B_OPCODES | SPI_NOR_IO_MODE_EN_VOLATILE | SPI_NOR_SOFT_RESET,
		.fixups = &mx66lm1g45g_fixups,
	}
};

static void macronix_nor_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
}

static int macronix_nor_late_init(struct spi_nor *nor)
{
	if (!nor->params->set_4byte_addr_mode)
		nor->params->set_4byte_addr_mode = spi_nor_set_4byte_addr_mode_en4b_ex4b;

	return 0;
}

static const struct spi_nor_fixups macronix_nor_fixups = {
	.default_init = macronix_nor_default_init,
	.late_init = macronix_nor_late_init,
};

const struct spi_nor_manufacturer spi_nor_macronix = {
	.name = "macronix",
	.parts = macronix_nor_parts,
	.nparts = ARRAY_SIZE(macronix_nor_parts),
	.fixups = &macronix_nor_fixups,
};
