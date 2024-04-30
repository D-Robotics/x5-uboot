#include <command.h>
#include <common.h>
#include <env.h>
#include <errno.h>
#include <tee.h>
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

static int do_optee_efuse(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	const char *cmd;
	uint32_t offset;
	uint32_t size;
	char output_buffer[256];
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	cmd = argv[1];

	if (strcmp(cmd, "read") == 0) {
		if (argc < 4)
			return CMD_RET_USAGE;
		offset = hextoul(argv[2], NULL);
		size   = hextoul(argv[3], NULL) * 4;
		ret = hb_read_efuse(offset, size, output_buffer);
		if (ret == CMD_RET_SUCCESS)
		{
			printf("otpee read efuse values:\n");
			print_buffer(offset, output_buffer, 4, size / 4, 1);
		}
		return ret;
	}
	if (strcmp(cmd, "dump") == 0) {
		if (argc != 2)
			return CMD_RET_USAGE;
		return dump_efuse();
	}
	return CMD_RET_USAGE;
}

U_BOOT_CMD(efuse, 4, 1, do_optee_efuse, "Read/Dump efuse access via optee",
		   "read [offset] [dword_count] - read 'dword_count' banks\n"
		   "efuse dump - dump all banks\n");
