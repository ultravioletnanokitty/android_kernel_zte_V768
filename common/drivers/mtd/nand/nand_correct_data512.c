/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/mtd/nand/nand_correct_data512.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

/* --------------------------------------------------------------------------- */
/* nand error control correction routines */
/* --------------------------------------------------------------------------- */
#include <linux/types.h>
#include <linux/mtd/nand_ecc512.h>


/*****************************************************************************/
/*                                                                           */
/* NAME                                                                      */
/*      nand_correct_data512                                                */
/* DESCRIPTION                                                               */
/*      This function compares two ECCs and indicates if there is an error.  */
/* PARAMETERS                                                                */
/*      read_ecc           one ECC to be compared                            */
/*      calc_ecc           the other ECC to be compared                      */
/*      dat                content of data page                              */
/* RETURN VALUES                                                             */
/*      Upon successful completion, compare_ecc returns 0.                   */
/*      Otherwise, corresponding error code is returned.                     */
/*                                                                           */
/*****************************************************************************/

int nand_correct_data512(struct mtd_info *mtd, u_char *dat, u_char *read_ecc,
			 u_char *calc_ecc)
{
	unsigned short row, col;
	u_char newval;
	unsigned long  orgecc;
	unsigned long  newecc;
	unsigned long  mylword;
        unsigned long  ecc_sum;

	orgecc = (read_ecc[2] << 16) | (read_ecc[1] << 8) | (read_ecc[0]);
	newecc = (calc_ecc[2] << 16) | (calc_ecc[1] << 8) | (calc_ecc[0]);
	

	mylword = orgecc ^ newecc;

	/* Quick check to avoid for loop below for normal no error case. */
	if (mylword == 0) {
		return 0;	/* No error */
	}

	// Fast ones count using in-place parallel addition of bits.
	ecc_sum = mylword;
	ecc_sum -= ((ecc_sum >> 1) & 0x55555555);
	ecc_sum = (((ecc_sum >> 2) & 0x33333333) + (ecc_sum & 0x33333333));
	ecc_sum = (((ecc_sum >> 4) + ecc_sum) & 0x0f0f0f0f);
	ecc_sum += (ecc_sum >> 8);
	ecc_sum += (ecc_sum >> 16);
	ecc_sum &= 0x0000003f;  

	if (ecc_sum == 1) {
		return 2;	/* ECC itself in error */
	}
	else if (ecc_sum == 12){
		col =  mylword & 0x7;            // bit position P4P2P1
        row = (mylword >> 3) & 0x1FF;  // byte position P2048..P8 	
		newval = dat[row] ^ (1 << col);
		/* printk("ECC: Replaced at offset 0x%x  old=0x%x new=0x%x\n", row, dat[row], newval); */
		dat[row] = newval;
		return 1;	/* Corrected 1 error */
	}
	return -1;		/* 2 or more errors - uncorrectible */
}

