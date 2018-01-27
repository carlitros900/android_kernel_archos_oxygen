/* linux/drivers/hwmon/stk8baxx.h
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * STK8BAXX driver for MT6573
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef STK8BAXX_H
#define STK8BAXX_H 
	 
#include <linux/ioctl.h>

#define IS_STK8BA50

#define STK8BAXX_I2C_SLAVE_ADDR		0x18 		//(SDO=GND or floating)
//#define STK8BAXX_I2C_SLAVE_ADDR		0x19	//(SDO=VDDIO)

	 /* STK8BAXX Register Map  (Please refer to STK8BA22 Specifications) */
#define 	STK8BAXX_XOUT1			0x02
#define 	STK8BAXX_XOUT2			0x03
#define 	STK8BAXX_YOUT1			0x04	
#define 	STK8BAXX_YOUT2			0x05
#define 	STK8BAXX_ZOUT1			0x06
#define 	STK8BAXX_ZOUT2			0x07
#define 	STK8BAXX_INTSTS1		0x09
#define 	STK8BAXX_INTSTS2		0x0A
#define 	STK8BAXX_EVENTINFO1	0x0B
#define 	STK8BAXX_EVENTINFO2	0x0C
#define 	STK8BAXX_RANGESEL		0x0F
#define 	STK8BAXX_BWSEL			0x10
#define 	STK8BAXX_POWMODE		0x11
#define  	STK8BAXX_DATASETUP		0x13
#define  	STK8BAXX_SWRST			0x14
#define  	STK8BAXX_INTEN1			0x16
#define  	STK8BAXX_INTEN2			0x17
#define  	STK8BAXX_INTMAP1		0x19
#define  	STK8BAXX_INTMAP2		0x1A
#define  	STK8BAXX_INTMAP3		0x1B
#define  	STK8BAXX_DATASRC		0x1E
#define  	STK8BAXX_INTCFG1		0x20
#define  	STK8BAXX_INTCFG2		0x21
#define  	STK8BAXX_LGDLY			0x22
#define  	STK8BAXX_LGTHD			0x23
#define  	STK8BAXX_HLGCFG		0x24
#define  	STK8BAXX_HGDLY			0x25
#define  	STK8BAXX_HGTHD			0x26
#define  	STK8BAXX_SLOPEDLY		0x27
#define  	STK8BAXX_SLOPETHD		0x28
#define  	STK8BAXX_TAPTIME		0x2A
#define  	STK8BAXX_TAPCFG		0x2B
#define  	STK8BAXX_ORIENTCFG		0x2C
#define  	STK8BAXX_ORIENTTHETA	0x2D
#define  	STK8BAXX_FLATTHETA		0x2E
#define  	STK8BAXX_FLATHOLD		0x2F
#define  	STK8BAXX_SLFTST			0x32	
#define  	STK8BAXX_INTFCFG		0x34
#define  	STK8BAXX_OFSTCOMP1	0x36
#define  	STK8BAXX_OFSTCOMP2	0x37
#define  	STK8BAXX_OFSTFILTX		0x38
#define  	STK8BAXX_OFSTFILTY		0x39
#define  	STK8BAXX_OFSTFILTZ		0x3A
#define  	STK8BAXX_OFSTUNFILTX	0x3B
#define  	STK8BAXX_OFSTUNFILTY	0x3C
#define  	STK8BAXX_OFSTUNFILTZ	0x3D

/*	SWRST register	*/
#define  	STK8BAXX_SWRST_VAL		0xB6

/*	STK8BAXX_POWMODE register	*/
#define STK8BAXX_MD_SUSPEND	0x80
#define STK8BAXX_MD_NORMAL		0x00
#define STK8BAXX_MD_SLP_MASK	0x1E

/*	RANGESEL register	*/
#define STK8BAXX_RANGE_MASK	0x0F
#define STK8BAXX_RNG_2G			0x3
#define STK8BAXX_RNG_4G			0x5
#define STK8BAXX_RNG_8G			0x8
#define STK8BAXX_RNG_16G		0xC

/*	STK8BAXX_OFSTCOMP1 register	*/
#define CAL_OFST_RST				0x80

#define STK8BAXX_SUCCESS						0
#define STK8BAXX_ERR_I2C						-1
#define STK8BAXX_ERR_STATUS					-3
#define STK8BAXX_ERR_SETUP_FAILURE			-4
#define STK8BAXX_ERR_GETGSENSORDATA			-5
#define STK8BAXX_ERR_IDENTIFICATION			-6

#define STK8BAXX_BUFSIZE				256
	 
#endif
