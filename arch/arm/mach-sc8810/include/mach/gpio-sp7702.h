/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GPIO_SC8810_H__
#define __GPIO_SC8810_H__

#ifndef __ASM_ARCH_BOARD_H
#error  "Don't include this file directly, include <mach/board.h>"
#endif

#define GPIO_INVALID		0xFFFFFFFF

/*
 * GPIO NR:
 *   0   - 15  : D-Die EIC
 *   16  - 159 : D-Die GPIO
 *   160 - 175 : A-Die EIC
 *   176 - 207 : A-Die GPIO
 */

#define GPIO_BT_RESET       90
#define GPIO_WIFI_RESET     140
#define GPIO_WIFI_IRQ	    -1
#define GPIO_WIFI_SHUTDOWN  137

#define GPIO_TOUCH_RESET	16
#define GPIO_TOUCH_IRQ		17
#define GPIO_PLSENSOR_IRQ	28
#define MSENSOR_DRDY_GPIO       97

#define EIC_CHARGER_DETECT	162
#define EIC_KEY_POWER		163

#define GPIO_SENSOR_RESET	72
#define GPIO_MAIN_SENSOR_PWN    73
#define GPIO_SUB_SENSOR_PWN     74

#define HEADSET_DETECT_GPIO	141
#define HEADSET_PA_CTL_GPIO	93
#define GPIO_GPS_RESET          26
#define GPIO_GPS_ONOFF          27

#define GPIO_BK		143


#define GPIO_AP_TO_CP_RTS	38	/*cp gpio 0*/
#define GPIO_CP_TO_AP_RDY	37	/*cp gpio 1*/
#define GPIO_CP_TO_AP_RTS	99	/*cp gpio 2*/
#define GPIO_AP_STATUS		25	/*cp gpio 3*/

#define GPIO_MODEM_POWER	106
#define GPIO_MODEM_DETECT	36	/*cp gpio 94*/
#define GPIO_MODEM_BOOT		92	/*cp gpio 5*/
#define GPIO_MODEM_CRASH	40	/*cp gpio 6*/

#endif
