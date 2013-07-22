/*
 * Copyright (C) 2013 Spreadtrum Communications Inc.
 * Copyright (C) steve.zhan
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define SCX35_ALPHA_TAPOUT	(0x8300aaaa)
#define SCX35_ALPHA_TAPOUT_MASK	(0xFFFFFFFF)

#define SCX35_BETA_TAPOUT	(0x8300a001)
#define SCX35_BETA_TAPOUT_MASK	(0xFFFFFFFF)



/**
 * sci_get_chip_id - read chip id
 *
 *
 * Return read the chip id
 */
u32 sci_get_chip_id(void);



/**
 *
 * sc_init_chip_id(void) - read chip id from hardware and save it.
 *
 */
void __init sc_init_chip_id(void);



#define IS_CPU(name, id, mask)		\
static inline int soc_id_is_##name(void)	\
{						\
	return ((sci_get_chip_id() & mask) == (id & mask));	\
}

IS_CPU(sc8735v0, SCX35_ALPHA_TAPOUT, SCX35_ALPHA_TAPOUT_MASK)
IS_CPU(sc8735v1, SCX35_BETA_TAPOUT, SCX35_BETA_TAPOUT_MASK)

/*Driver can use this MACRO to distinguish different chip code */
#define soc_is_scx35_v0()	soc_id_is_sc8735v0()
#define soc_is_scx35_v1()	soc_id_is_sc8735v1()


/**
* read value from virtual address. Pls make sure val is not NULL.
* return 0 is successful
*/
int sci_read_va(u32 vreg, u32 *val);

/**
* write value to virtual address. if clear_msk is ~0, or_val will fully set to vreg.
* return 0 is successful
*/
int sci_write_va(u32 vreg, const u32 or_val, const u32 clear_msk);

/**
* read value from pysical address. Pls make sure val is not NULL.
* return 0 is successful
*/
int sci_read_pa(u32 preg, u32 *val);

/**
* write value to pysical address. if clear_msk is ~0, or_val will fully set to paddr.
* return 0 is successful
*/
int sci_write_pa(u32 paddr, const u32 or_val, const u32 clear_msk);

