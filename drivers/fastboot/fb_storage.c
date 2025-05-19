#include <stdio.h>
#include <env.h>
#include <fastboot.h>
#include <fb_storage.h>
#include <net/fastboot.h>

#include <fb_mmc.h>
#include <fb_nand.h>
#include <fb_spinand.h>
#include <fb_ram.h>

static const struct fastboot_storage_ops *storage_ops[FLASH_TYPE_COUNT + 1];
// struct fastboot_storage_ops *current_storage_ops = NULL;

void fastboot_storage_init(void)
{
	#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
		fastboot_mmc_register();
	#endif
	#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_SPINAND)
		fastboot_spinand_register();
	#endif
}

void fastboot_storage_register(fb_flash_type type,
                               const struct fastboot_storage_ops *ops)
{
	if (type <= FLASH_TYPE_COUNT) {
		storage_ops[type] = ops;
	}
}

const struct fastboot_storage_ops *get_current_storage_ops(void)
{
	fb_flash_type type = fastboot_get_flash_type();

	if (type >= FLASH_TYPE_COUNT || !storage_ops[type]) {
		printf("No ops registered for flash type %d\n", type);
		return NULL;
	}
	return storage_ops[type];
}

void check_storage_registration(void)
{
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	if (!storage_ops[FLASH_TYPE_EMMC]) {
		printf("MMC ops not registered!");
	}
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_SPINAND)
	if (!storage_ops[FLASH_TYPE_SPINAND]) {
		printf("spinand ops not registered!");
	}
#endif
}
