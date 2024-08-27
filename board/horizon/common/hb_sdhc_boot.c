#include <common.h>
#include <mmc.h>
#include <malloc.h>
#include <dm/ofnode.h>

#if !defined(CONFIG_ENV_IS_IN_UBI)

static inline int mmc_offset_try_partition(const char *str, int copy, s64 *val)
{
	struct disk_partition info;
	struct blk_desc *desc;
	int len, i, ret;
	char dev_str[4];

	snprintf(dev_str, sizeof(dev_str), "%d", mmc_get_env_dev());
	ret = blk_get_device_by_str("mmc", dev_str, &desc);
	if (ret < 0)
		return (ret);

	for (i = 1;;i++) {
		ret = part_get_info(desc, i, &info);
		if (ret < 0)
			return ret;

		if (!strncmp((const char *)info.name, str, sizeof(info.name)))
			break;
	}

	/* round up to info.blksz */
	len = DIV_ROUND_UP(CONFIG_ENV_SIZE, info.blksz);

	/* use the bottom of the partion for the environment */
	*val = info.start * info.blksz;

	return 0;
}

static inline s64 mmc_offset(int copy)
{
	const struct {
		const char *offset_redund;
		const char *partition;
		const char *offset;
	} dt_prop = {
		.offset_redund = "u-boot,mmc-env-offset-redundant",
		.partition = "u-boot,mmc-env-partition",
		.offset = "u-boot,mmc-env-offset",
	};
	s64 val = 0, defvalue;
	const char *propname;
	const char *str;
	int err;

	/* look for the partition in mmc CONFIG_SYS_MMC_ENV_DEV */
	str = ofnode_conf_read_str(dt_prop.partition);
	if (str) {
		/* try to place the environment at end of the partition */
		err = mmc_offset_try_partition(str, copy, &val);
		if (!err)
			return val;
	}

	defvalue = 0x0;
	defvalue = 0;
	propname = dt_prop.offset;

	return ofnode_conf_read_int(propname, defvalue);
}

int mmc_get_env_addr(struct mmc *mmc, int copy, u32 *env_addr)
{
	s64 offset = mmc_offset(copy);

	if (offset < 0)
		offset += mmc->capacity;

	*env_addr = offset;

	return 0;
}

#endif
