/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Horizon AI Inc.
 */
#ifndef HB_BOARD_ID_H
#define HB_BOARD_ID_H

#define HOBOT_X5_FPGA_ID   0xFFFF
#define HOBOT_X5_SOC_ID    0xFE01
#define HOBOT_X5_SVB_ID    0xFF10

#define HOBOT_X5_DVB_ID    0x0101
#define HOBOT_X5_EVB_ID    0x0201
#define HOBOT_X5_EVB_V2_ID 0x0202
#define HOBOT_X5_RDK_ID    0x0301

#if defined(CONFIG_HOBOT_X5_FPGA)
    #define HOBOT_X5_BOARD_ID HOBOT_X5_FPGA_ID
#elif defined(CONFIG_HOBOT_X5_SOC)
    #define HOBOT_X5_BOARD_ID HOBOT_X5_SOC_ID
#elif defined(CONFIG_HOBOT_X5_SVB)
    #define HOBOT_X5_BOARD_ID HOBOT_X5_SVB_ID
#elif defined(CONFIG_HOBOT_X5_EVB)
    #define HOBOT_X5_BOARD_ID HOBOT_X5_EVB_ID
#elif defined(CONFIG_HOBOT_X5_EVB_V2)
    #define HOBOT_X5_BOARD_ID HOBOT_X5_EVB_V2_ID
#elif !defined(CONFIG_HOBOT_ADC_BTYPE)
    #define CONFIG_HOBOT_ADC_BTYPE
#endif

#endif
