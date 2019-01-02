/*
 *  stk8baxx.c - Linux kernel modules for sensortek  stk8ba50 / stk8ba50-R /
 *  stk8ba53 accelerometer
 *
 *  Copyright (C) 2012~2016 Lex Hsieh / Sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
 
#ifndef STK8BAXX_H
#define STK8BAXX_H

#include <linux/ioctl.h>

#define STK8BAXX_BUFSIZE				256

#define STK8BAXX_SUCCESS					0
#define STK8BAXX_ERR_I2C						-1
#define STK8BAXX_ERR_STATUS					-3
#define STK8BAXX_ERR_SETUP_FAILURE			-4
#define STK8BAXX_ERR_GETGSENSORDATA		-5
#define STK8BAXX_ERR_IDENTIFICATION			-6

/*----------------------------------------------------------------------------*/
#define STK8BAXX_AXIS_X          0
#define STK8BAXX_AXIS_Y          1
#define STK8BAXX_AXIS_Z          2
#define STK8BAXX_AXES_NUM        3
#define STK8BAXX_DATA_LEN        6
#define STK8BAXX_DEV_NAME        "STK8BAXX"

/*----------------------------------------------------------------------------*/
enum CUST_ACTION {
	STK8BAXX_CUST_ACTION_SET_CUST = 1,
	STK8BAXX_CUST_ACTION_SET_CALI,
	STK8BAXX_CUST_ACTION_RESET_CALI
};
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
}STK8BAXX_CUST;
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
	uint16_t part;
	int32_t data[0];
}STK8BAXX_SET_CUST;
/*----------------------------------------------------------------------------*/
typedef struct {
	uint16_t action;
	int32_t data[STK8BAXX_AXES_NUM];
}STK8BAXX_SET_CALI;
/*----------------------------------------------------------------------------*/
union STK8BAXX_CUST_DATA {
	uint32_t data[10];
	STK8BAXX_CUST cust;
	STK8BAXX_SET_CUST setCust;
	STK8BAXX_SET_CALI setCali;
	STK8BAXX_CUST resetCali;
};
/*----------------------------------------------------------------------------*/


#endif
