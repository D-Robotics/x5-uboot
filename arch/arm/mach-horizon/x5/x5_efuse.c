#include <command.h>
#include <common.h>
#include <env.h>
#include <asm/io.h>
#include <errno.h>
#include <tee.h>
#include <linux/delay.h>
#include <display_options.h>
#include <asm/arch/hb_efuse.h>

#define PTA_EFUSE                                          \
	{                                                      \
		0x16c83a2b, 0xaae3, 0x4542,                        \
		{                                                  \
			0x9d, 0xdd, 0x40, 0x46, 0x51, 0xe0, 0x1e, 0xa2 \
		}                                                  \
	}

#define PTA_EFUSE_READ          0
#define PTA_EFUSE_WRITE         1
#define TEE_ERROR_ACCESS_DENIED 0xffff0001

int hb_read_efuse(uint32_t offset, uint32_t size, char *output_buffer)
{
	int rc = CMD_RET_SUCCESS;
	const struct tee_optee_ta_uuid uuid = PTA_EFUSE;
	struct tee_open_session_arg session;
	struct tee_invoke_arg invoke;
	struct tee_param param[2];
	struct udevice *tee_dev = NULL;
	struct tee_shm *shm_val;

	tee_dev = tee_find_device(NULL, NULL, NULL, NULL);
	if (!tee_dev) {
		rc = -ENODEV;
		goto exit;
	}

	rc = tee_shm_alloc(tee_dev, size, TEE_SHM_ALLOC, &shm_val);
	if (rc) {
		rc = -ENOMEM;
		goto exit;
	}

	memset(&session, 0, sizeof(session));
	tee_optee_ta_uuid_to_octets(session.uuid, &uuid);
	if (tee_open_session(tee_dev, &session, 0, NULL)) {
		rc = -ENXIO;
		goto free_shm;
	}

	memset(param, 0, sizeof(param));
	param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a     = offset;
	param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	param[1].u.memref.shm  = shm_val;
	param[1].u.memref.size = size;

	memset(&invoke, 0, sizeof(invoke));
	invoke.func    = PTA_EFUSE_READ;
	invoke.session = session.session;

	rc = tee_invoke_func(tee_dev, &invoke, 2, param);
	if (rc != 0) {
		printf("tee_invoke_func failed with error [0x%x]\n", rc);
		goto close_session;
	}
	if (invoke.ret) {
		rc = invoke.ret;
		if (invoke.ret == TEE_ERROR_ACCESS_DENIED)
			printf("efuse region access denied\n");
		else
			printf("optee read efuse failed with error [0x%x]\n", invoke.ret);
		goto close_session;
	}

	memcpy(output_buffer, shm_val->addr, size);

close_session:
	tee_close_session(tee_dev, session.session);
free_shm:
	tee_shm_free(shm_val);
exit:
	return rc;
}

bool is_secure_boot()
{
	int32_t value = 0;
	hb_read_efuse(MBEDTLS_OTP_USER_SECUR_FLAG_OFFSET,
				  4,
				  (char *)&value);
	return (value & 0x1);
}

int hb_get_socuid(uint32_t *socuid)
{
	int32_t ret = 0;
	ret = hb_read_efuse(MBEDTLS_OTP_MODEL_ID_OFFSET,
						MBEDTLS_OTP_MODEL_ID_SIZE,
						(char *)socuid);
	ret |= hb_read_efuse(MBEDTLS_OTP_DEVICE_ID_OFFSET,
						MBEDTLS_OTP_DEVICE_ID_SIZE,
						(char *)&socuid[1]);
	ret |= hb_read_efuse(MBEDTLS_OTP_USER_SOCID_HIGH_OFFSET,
						MBEDTLS_OTP_USER_SOCID_HIGH_SIZE,
						(char *)&socuid[2]);
	if (ret) {
		printf("get socuid failed\n");
	}
	return 0;
}

static int dump_efuse(void)
{
	uint32_t efuse_buf[14] = {0};
	int32_t ret;
	int32_t i = 0;

	ret = hb_get_socuid(efuse_buf);
	if (ret == CMD_RET_SUCCESS)
	{
		printf("socid[0]: 0x%x\n", efuse_buf[0]);
		printf("socid[1]: 0x%x\n", efuse_buf[1]);
		printf("socid[2]: 0x%x\n", efuse_buf[2]);
		printf("socid[3]: 0x%x\n", efuse_buf[3]);
	}
	ret = hb_read_efuse(MBEDTLS_OTP_USER_SEC_BL2PUB_OFFSET,
					MBEDTLS_OTP_USER_SEC_BL2PUB_SIZE,
					(char *)efuse_buf);
	for (i = 0; i < MBEDTLS_OTP_USER_SEC_BL2PUB_SIZE / 4; i++) {
		printf("public hash[%d]:0x%x\n", i, efuse_buf[i]);
	}
	ret = hb_read_efuse(MBEDTLS_OTP_USER_NON_SEC_REGION_OFFSET,
					MBEDTLS_OTP_USER_NON_SEC_REGION_SIZE,
					(char *)efuse_buf);
	for (i = 0; i < MBEDTLS_OTP_USER_NON_SEC_REGION_SIZE / 4; i++) {
		printf("non-secure bank[%d]:0x%x\n", i, efuse_buf[i]);
	}
	return ret;
}

static int is_valid_read_efuse_region(struct efuse_info *efuse)
{
	/* valid read region:
		all non secure region
		secure region[20:13] [0]
	*/
	if ((efuse->type == EFUSE_NONSECURE) && efuse->bank < 14)
		return 1;

	if ((efuse->type == EFUSE_SECURE) &&
		((efuse->bank > 12 && efuse->bank < 21) || efuse->bank == 0))
		return 1;

	printf("READ efuse type:%d, bank:%d invalid\n", efuse->type, efuse->bank);
	return 0;
}

static int hb_read_efuse_and_lock(struct efuse_info *efuse)
{
	int32_t ret = 0;
	uint32_t offset = 0;
	uint32_t size = 0;
	uint32_t value = 0;

	ret = is_valid_read_efuse_region(efuse);
	if (ret == 0) {
		return -1;
	}
	size = 4;
	/* read bank value */
	offset = efuse->bank * 4;
	offset = efuse->type == EFUSE_SECURE ?
	                        offset + MBEDTLS_OTP_USER_SEC_REGION_OFFSET : offset + MBEDTLS_OTP_USER_NON_SEC_REGION_OFFSET;
	ret = hb_read_efuse(offset, size, (char *)&value);
	if (ret) {
		printf("read type:%s bank:%d failed\n",
	            efuse->type == EFUSE_NONSECURE ? "NON-SECURE": "SECURE", efuse->bank);
		return ret;
	}
	efuse->value = value;

	/* read lock */
	offset = efuse->type == EFUSE_SECURE ? MBEDTLS_OTP_USER_SEC_REGION_OFFSET : MBEDTLS_OTP_USER_NON_SEC_REGION_OFFSET;
	ret = hb_read_efuse(offset, size, (char *)&value);
	if (ret) {
		printf("read type:%s  bank:%d lock status failed\n",
	            efuse->type == EFUSE_NONSECURE ? "NON-SECURE": "SECURE", efuse->bank);
		return ret;
	}
	efuse->lock = (value >> efuse->bank) & 0x1;
	return ret;
}

static int hb_write_efuse(u32 offset, u32 value)
{
	int rc = CMD_RET_SUCCESS;
	const struct tee_optee_ta_uuid uuid = PTA_EFUSE;
	struct tee_open_session_arg session;
	struct tee_invoke_arg invoke;
	struct tee_param param[2];
	struct udevice *tee_dev = NULL;
	struct tee_shm *shm_val;

	tee_dev = tee_find_device(NULL, NULL, NULL, NULL);
	if (!tee_dev) {
		rc = -ENODEV;
		goto exit;
	}

	rc = tee_shm_alloc(tee_dev, sizeof(value), TEE_SHM_ALLOC, &shm_val);
	if (rc) {
		rc = -ENOMEM;
		goto exit;
	}

	memcpy(shm_val->addr, &value, sizeof(value));

	memset(&session, 0, sizeof(session));
	tee_optee_ta_uuid_to_octets(session.uuid, &uuid);
	if (tee_open_session(tee_dev, &session, 0, NULL)) {
		rc = -ENXIO;
		goto free_shm;
	}

	memset(&param, 0, sizeof(param));
	param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a     = offset;
	param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	param[1].u.memref.shm  = shm_val;
	param[1].u.memref.size = sizeof(value);

	memset(&invoke, 0, sizeof(invoke));
	invoke.func    = PTA_EFUSE_WRITE;
	invoke.session = session.session;

	rc = tee_invoke_func(tee_dev, &invoke, 2, param);
	if (rc != 0)
		printf("tee_invoke_func failed with error [0x%x]\n", rc);

	if (invoke.ret) {
		rc = invoke.ret;
		if (invoke.ret == TEE_ERROR_ACCESS_DENIED)
			printf("efuse region access denied\n");
		else
			printf("optee write efuse failed with error [0x%x]\n", invoke.ret);
	}

	tee_close_session(tee_dev, session.session);
free_shm:
	tee_shm_free(shm_val);
exit:
	return rc;
}

static int is_valid_write_efuse_region(struct efuse_info *efuse)
{
	/* valid read region:
		non secure region[13:10]
		secure region[20:13]
	*/
	if ((efuse->type == EFUSE_NONSECURE) &&
		(efuse->bank > 9 && efuse->bank < 14))
		return 1;

	if ((efuse->type == EFUSE_SECURE) &&
		(efuse->bank > 12 && efuse->bank < 21))
		return 1;

	printf("WRITE efuse type:%d, bank:%d invalid\n", efuse->type, efuse->bank);
	return 0;
}

static void drobot_efuse_gpio_init(void)
{
	uint32_t value = 0;
	/*
	* 1, GPIO FUNC
	* 2. OUTPUT DIRECTION
	*/
	value = readl(HB_EFUSE_PINMUX);
	value &= ~HB_EFUSE_PINMUX_MSAK;
	value |= HB_EFUSE_FUNC_GPIO;
	writel(value, HB_EFUSE_PINMUX);

	value = readl(HB_EFUSE_GPIO_DIR_REG);
	value |= HB_EFUSE_GPIO_DIR_OUTPUT;
	writel(value, HB_EFUSE_GPIO_DIR_REG);
}

static inline void drobot_efuse_gpio_high(void)
{
	uint32_t value = 0;

	value = readl(HB_EFUSE_GPIO_VALUE_REG);
	value |= HB_EFUSE_GPIO_HIGH;
	writel(value, HB_EFUSE_GPIO_VALUE_REG);

}

static inline void drobot_efuse_gpio_low(void)
{
	uint32_t value = 0;

	value = readl(HB_EFUSE_GPIO_VALUE_REG);
	value &= ~HB_EFUSE_GPIO_HIGH;
	writel(value, HB_EFUSE_GPIO_VALUE_REG);
}


static int is_valid_write_efuse_condition(struct efuse_info *efuse)
{
	int ret = 0;
	struct efuse_info tmp = {0};

	memcpy(&tmp, efuse, sizeof(tmp));
	ret = hb_read_efuse_and_lock(&tmp);
	if (ret) {
		return 0;
	}
	if (tmp.lock != 0) {
		printf("type:%s bank:%d has been locked\n", tmp.type == EFUSE_NONSECURE ? "NON-SECURE": "SECURE", tmp.bank);
		return 0;
	}
	/* customer bank---non secure bank11/12/13 can be burned when it is not locked */
	if ((tmp.type == EFUSE_NONSECURE) && (tmp.bank > 10) && (tmp.bank < 14)) {
		return 1;
	}
	if (tmp.value != 0) {
		printf("type:%s bank:%d has been burned\n", tmp.type == EFUSE_NONSECURE ? "NON-SECURE": "SECURE", tmp.bank);
		return 0;
	}
	return 1;
}

static int hb_write_efuse_and_lock(struct efuse_info *efuse)
{
	int ret = 0;
	uint32_t offset = 0;
	uint32_t value = 0;

	ret = is_valid_write_efuse_region(efuse);
	if (ret == 0) {
		return -1;
	}

	ret = is_valid_write_efuse_condition(efuse);
	if (ret == 0) {
		return -1;
	}
	if (efuse->value == 0) {
		printf("efuse value is 0, skip\n");
		return -1;
	}
	drobot_efuse_gpio_init();
	drobot_efuse_gpio_high();
	mdelay(1000);
	/* update value */
	offset = efuse->bank * 4;
	offset = (efuse->type == EFUSE_SECURE) ?
	          offset + MBEDTLS_OTP_USER_SEC_REGION_OFFSET : offset + MBEDTLS_OTP_USER_NON_SEC_REGION_OFFSET;
	value = efuse->value;
	ret = hb_write_efuse(offset, value);
	if (ret) {
		printf("write %s bank: %d failed\n",
	           (efuse->type == EFUSE_SECURE) ? "SECURE" : "NON-SECURE",
	           efuse->bank);
		goto exit;
	}

	/* lock bank */
	if (efuse->lock == 0) {
		goto exit;
	}
	offset = (efuse->type == EFUSE_SECURE) ?
	          MBEDTLS_OTP_USER_SEC_REGION_OFFSET : MBEDTLS_OTP_USER_NON_SEC_REGION_OFFSET;
	value = 1 << efuse->bank;
	ret = hb_write_efuse(offset, value);
	if (ret) {
		printf("lock %s bank: %d failed\n",
	           (efuse->type == EFUSE_SECURE) ? "SECURE" : "NON-SECURE",
	           efuse->bank);
		goto exit;
	}
exit:
	drobot_efuse_gpio_low();
	return ret;
}

static uint32_t parse_efuse_type(char *type)
{
	if (strncmp(type, "secure", strlen("secure") + 1) == 0) {
		return EFUSE_SECURE;
	} else if (strncmp(type, "nonsecure", strlen("nonsecure") + 1) == 0) {
		return EFUSE_NONSECURE;
	}

	printf("parse efuse type failed,valid efuse type: 'secure' or 'nonsecure'\n");
	return -1;
}

static uint32_t parse_efuse_lock(char *lock)
{
	if (strncmp(lock, "lock", strlen("lock") + 1) == 0) {
		return EFUSE_LOCK;
	} else if (strncmp(lock, "unlock", strlen("unlock") + 1) == 0) {
		return EFUSE_UNLOCK;
	}

	printf("parse efuse lock status failed,valid efuse type: 'lock' or 'unlock'\n");
	return -1;
}

static int do_optee_efuse(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	const char *cmd;
	int ret;
	struct efuse_info info = {0};

	if (argc < 2)
		return CMD_RET_USAGE;

	cmd = argv[1];

	if (strcmp(cmd, "read") == 0) {
		if (argc < 4)
			return CMD_RET_USAGE;
		ret = parse_efuse_type(argv[2]);
		if (ret < 0) {
			return -1;
		}
		info.type = ret;
		info.bank = hextoul(argv[3], NULL);
		ret = hb_read_efuse_and_lock(&info);
		if (ret == 0) {
			printf("value:0x%x\n", info.value);
			printf("lock:%s\n", info.lock == 1 ? "true" : "false");
		}
		return ret;
	} else if (strcmp(cmd, "dump") == 0) {
		if (argc != 2)
			return CMD_RET_USAGE;
		return dump_efuse();
	} else if (strcmp(cmd, "write") == 0) {
		if (argc < 6)
			return CMD_RET_USAGE;
		ret = parse_efuse_type(argv[2]);
		if (ret < 0) {
			return -1;
		}
		info.type = ret;
		info.bank = hextoul(argv[3], NULL);
		info.value = hextoul(argv[4], NULL);
		ret = parse_efuse_lock(argv[5]);
		if (ret < 0) {
			return -1;
		}
		info.lock = ret;
		ret = hb_write_efuse_and_lock(&info);
		if (ret) {
			printf("write %s bank:0x%x failed\n", info.type == EFUSE_SECURE ? "SECURE" : "NONSECURE", info.bank);
		} else {
			printf("write %s bank:0x%x success\n", info.type == EFUSE_SECURE ? "SECURE" : "NONSECURE", info.bank);
		}
		return ret;
	}
	return CMD_RET_USAGE;
}

U_BOOT_CMD(efuse, 6, 1, do_optee_efuse, "Read/Dump efuse access via optee",
		   "read [type] [bank] - efuse read 'type' 'banks'\n"
		   "efuse write [type] [bank] [value] [lock]- efuse write 'type' 'banks' 'value' 'lock'\n"
		   "efuse dump - dump all banks\n");
