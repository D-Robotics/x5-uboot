// SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright(C) 2026, D-Robotics Co., Ltd.
 *
 */

#ifndef __HB_QUICKSTART_H
#define __HB_QUICKSTART_H

#ifdef CONFIG_QUICKSTART

/* hb_quickstart_init_sequence_r[] entries (implemented in board_r.c / drivers) */
int initr_trace(void);
int initr_reloc(void);
#if defined(CONFIG_ARM) || defined(CONFIG_RISCV)
int initr_caches(void);
#endif
int initr_reloc_global_data(void);
int initr_malloc(void);
#if defined(CONFIG_DM)
int initr_dm(void);
#endif
#if defined(CONFIG_MMC)
int initr_mmc(void);
#endif
int hb_quickstart_boot(void);

extern uint32_t hb_get_uart_baud(void);
extern char *hb_get_ab_switch_reason(void);
extern char *hb_bootmedium_for_udev(void);
extern char *hb_bootmode(void);
static char hb_quickstart_env_data[ENV_SIZE];

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
extern int hb_setup_ion_size(void *blob);
int hb_fdt_set_board_info(void *fdt_blob);
extern void fdt_rm_by_env(void *fdt_blob);
#endif


//struct global_data;

#endif /* CONFIG_QUICKSTART */

#endif /* __HB_QUICKSTART_H */
