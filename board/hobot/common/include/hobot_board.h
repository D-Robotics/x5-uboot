/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2021
 * Zhaohui Shi <zhaohui.shi@horizon.ai>
 */
#ifndef HOBOT_BOARD_H
#define HOBOT_BOARD_H

#include <common.h>

struct board_driver {
	const char *name;
	/**
	 * test() - Test if the runnint board is this board type.
	 *
	 * @return 0 if the board appears to be this board type,
	 *	   -ve if not
	 */
	int (*test)(void);
	/**
	 * boot_hardware_get() - Return boot_hardware use in OS kernel
	 *
	 * @return NULL if there is no hook with boot_hardware.
	 *	   type, -ve if not
	 */
	char *(*hb_boot_hardware_get)(void);
	int (*hb_boot_mode_get)(void);
	int (*skip_pll_lock)(void);
#if CONFIG_IS_ENABLED(OF_CONTROL)
	void *(*board_fdt_blob_setup)(void);
	void (*board_fdt_blob_post_setup)(void *blob);
#endif
#ifndef CONFIG_SPL_BUILD
	char *(*fastboot_generic_flash_intf)(void);
	int (*fastboot_generic_flash_dev)(void);
	bool (*set_init_flags)(void);
	void (*board_type_init)(void);
	void (*chip_last_stage_init)(void);
	void (*board_env_setup)(void);
	int32_t (*pmic_hardware_set)(void);
#else
	u32 (*spl_boot_device)(void);
	int (*spl_dram_init)(void);
	void (*spl_board_init)(void);
	void (*spl_clock_init)(int fastboot_level);
	void (*mcore_load)(void);
#endif
	void (*hw_init)(void);
};

struct board_driver *board_driver_lookup_type(void);

/* Declare a new U-Boot partition 'driver' */
#define HOBOT_BOART_TYPE(__name)					\
	ll_entry_declare(struct board_driver, __name, board_driver)

#define CALL_BTYPE_DRIVERS(__func, ...)				\
	do {							\
		struct board_driver *bdriver =			\
			board_driver_lookup_type();		\
		if (bdriver && bdriver->__func)			\
			return bdriver->__func(## __VA_ARGS__);	\
	} while(0)

#define CALL_BTYPE_DRIVERS_ARGS(__func, arg, ...)			\
	do {								\
		struct board_driver *bdriver =				\
			board_driver_lookup_type();			\
		if (bdriver && bdriver->__func)				\
			return bdriver->__func(arg, ## __VA_ARGS__);	\
	} while(0)

#define CALL_BTYPE_DRIVERS_NORET(_ret, _func, ...)		\
	do {							\
		struct board_driver *bdriver =			\
			board_driver_lookup_type();		\
		if (bdriver && bdriver->_func)			\
			_ret = bdriver->_func(## __VA_ARGS__);	\
	} while(0)

#define CALL_BTYPE_DRIVERS_VOID(_func, ...)		\
	do {						\
		struct board_driver *bdriver =		\
			board_driver_lookup_type();	\
		if (bdriver && bdriver->_func)		\
			bdriver->_func(## __VA_ARGS__);	\
	} while(0)

#define CALL_BTYPE_DRIVERS_VOID_ARGS(_func, arg, ...)		\
	do {							\
		struct board_driver *bdriver =			\
			board_driver_lookup_type();		\
		if (bdriver && bdriver->_func)			\
			bdriver->_func(arg, ## __VA_ARGS__);	\
	} while(0)

#endif
