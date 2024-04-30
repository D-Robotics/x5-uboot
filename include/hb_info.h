/*
 *    COPYRIGHT NOTICE
 *   Copyright 2023 Horizon Robotics, Inc.
 *    All rights reserved.
 */

#ifndef __HB_INFO_H__
#define __HB_INFO_H__

#include <common.h>
#include <linux/types.h>
#include <dt-bindings/board/hb_board_id.h>

char *hb_hardware_name_get(void);
int hb_hardware_name_set(const char *name);
char *hb_ethact_name_get(void);
int hb_ethact_name_set(const char *name);
uint32_t hb_board_id_get(uint32_t *board_id);
int hb_board_id_set(uint32_t board_id);
char *hb_eth0_ip_get(void);
int hb_eth0_ip_set(const char *eth0_ip);
char *hb_board_name_get(void);
int hb_board_name_set(const char *name);
int hb_device_init(void);
char *hb_bootargs_console(void);
int32_t hb_get_socid(char *socid);
int32_t hb_pmic_type_set(const char *name);
char *hb_pmic_type_get(void);

static inline uint32_t hb_csum_calculate(void *buffer, size_t size)
{
	uint32_t csum = 0;
	int i;

	for (i = 0; i < size; i++) {
		csum += ((uint8_t *)buffer)[i];
	}

	return csum;
}

#endif /* __HB_INFO_H__ */
