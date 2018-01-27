/*
 * Copyright (c) 2014 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/****************************************************************************
 * Ralink Tech Inc.
 * Taiwan, R.O.C.
 *
 * (c) Copyright 2002, Ralink Technology, Inc.
 *
 * All rights reserved. Ralink's source code is an unpublished work and the
 * use of a copyright notice does not imply otherwise. This source code
 * contains confidential trade secret material of Ralink Tech. Any attemp
 * or participation in deciphering, decoding, reverse engineering or in any
 * way altering the source code is stricitly prohibited, unless the prior
 * written consent of Ralink Technology, Inc. is obtained.
 ***************************************************************************/

/****************************************************************************
  Module Name:
  BigInteger


  Revision History:
  Who         When            What
  --------    ----------      ------------------------------------------
  Eddy        2009/01/12      Create
 ***************************************************************************/

#ifndef __CRYPT_BIGINTEGER_H__
#define __CRYPT_BIGINTEGER_H__

#include "crypt_config.h"

/* BigInteger definition & structure */
#define SLIDING_WINDOW 16
typedef struct _BIG_INTEGER_STRUC {
	STRING Name[10];
	UINT32 *pIntegerArray;
	UINT AllocSize;
	UINT ArrayLength;
	UINT IntegerLength;
	INT  Signed;
} BIG_INTEGER, *PBIG_INTEGER;


/* BigInteger operations */
VOID BigInteger_Print(PBIG_INTEGER pBI);

VOID BigInteger_Init(PBIG_INTEGER *pBI);

VOID BigInteger_Free_AllocSize(PBIG_INTEGER *pBI);

VOID BigInteger_Free(PBIG_INTEGER *pBI);

VOID BigInteger_AllocSize(PBIG_INTEGER *pBI, UINT Length);

VOID BigInteger_ClearHighBits(PBIG_INTEGER pBI);

VOID BigInteger_BI2Bin(PBIG_INTEGER pBI, UINT8 *pValue, UINT *Length);

VOID BigInteger_Bin2BI(UINT8 *pValue, UINT Length, PBIG_INTEGER *pBI);

VOID BigInteger_BitsOfBI(PBIG_INTEGER pBI, UINT *Bits_Of_P);

INT BigInteger_GetBitValue(PBIG_INTEGER pBI, UINT Index);

UINT8 BigInteger_GetByteValue(PBIG_INTEGER pBI, UINT Index);

VOID BigInteger_Copy(PBIG_INTEGER pBI_Copied, PBIG_INTEGER *pBI_Result);

INT BigInteger_UnsignedCompare(PBIG_INTEGER pFirstOperand, PBIG_INTEGER pSecondOperand);

VOID BigInteger_Add(PBIG_INTEGER pFirstOperand, PBIG_INTEGER pSecondOperand, PBIG_INTEGER *pBI_Result);

VOID BigInteger_Sub(PBIG_INTEGER pFirstOperand, PBIG_INTEGER pSecondOperand, PBIG_INTEGER *pBI_Result);

VOID BigInteger_Mul(PBIG_INTEGER pFirstOperand, PBIG_INTEGER pSecondOperand, PBIG_INTEGER *pBI_Result);

VOID BigInteger_Square(PBIG_INTEGER pBI, PBIG_INTEGER *pBI_Result);

VOID BigInteger_Div(PBIG_INTEGER pFirstOperand, PBIG_INTEGER pSecondOperand, PBIG_INTEGER *pBI_Result, PBIG_INTEGER *pBI_Remainder);

VOID BigInteger_Montgomery_Reduction(PBIG_INTEGER pBI_A, PBIG_INTEGER pBI_P, PBIG_INTEGER pBI_R, PBIG_INTEGER *pBI_Result);

VOID BigInteger_Montgomery_ExpMod(PBIG_INTEGER pBI_G, PBIG_INTEGER pBI_E, PBIG_INTEGER pBI_P, PBIG_INTEGER *pBI_Result);

VOID BigInteger_ExpMod_32(PBIG_INTEGER pBI_G, PBIG_INTEGER pBI_E, PBIG_INTEGER pBI_P, PBIG_INTEGER *pBI_Result);

#endif /* __CRYPT_BIGINTEGER_H__ */

