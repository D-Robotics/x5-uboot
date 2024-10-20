/*
 * Copyright (C) 2024, d-robotics, all rights reserved.
 * Author: Ming Yu <ming.yu@d-robotics.cc>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef HB_SIP_IPI_H
#define HB_SIP_IPI_H

#define HB_SIP_IPI_VERSION_MAGIC      0xdead0000
#define HB_SIP_IPI_VERSION_MASK       0xffff0000

#define HB_SIP_IPI_VERSION_MINOR(minor)   ((ver) & 0xff)
#define HB_SIP_IPI_VERSION_MAJOR(major)   (((ver) >> 8) & 0xff)
#define HB_SIP_IPI_CURRENT_VERSION	   (0xdead0001)

/**
 * @x0: function id
 * @x1:	command
 * @x2~@x7: parameters
 */
#define HB_SIP_IPI                    0xC200000c

/**
 * x1: Get IPI services version
 *
 * return: a0: status, a1: address to write OS state
 */
#define HB_SIP_IPI_VERSION            0x00000000

/**
 * x1: Get core context
 * @x2:	cpu index
 * @x3: reg index
 *
 * return: a0: status, a1: reg[31:0], a2: reg[63:32]
 */
#define HB_SIP_IPI_GET_CORE_CONTEXT   0x00000001

/**
 * x1: Trigger a IPI to get core context
 * @x2:	HB_SIP_IPI_CURRENT_VERSION for verify
 *
 * return: a0: status
 */
#define HB_SIP_IPI_TRIGER_CORE_CONTEXT   0x00000002

/**
 * x1: Clear core context
 * @x2:	HB_SIP_IPI_CURRENT_VERSION for verify
 *
 * return: a0: status
 */
#define HB_SIP_IPI_CLEAR_CORE_CONTEXT   0x00000003


#endif
