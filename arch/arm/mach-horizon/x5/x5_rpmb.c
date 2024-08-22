#include <command.h>
#include <stdlib.h>
#include <common.h>
#include <env.h>
#include <asm/io.h>
#include <errno.h>
#include <tee.h>
#include <linux/delay.h>
#include <display_options.h>
#include <asm/arch/hb_rpmb.h>

#define TEE_ERROR_ACCESS_DENIED 0xffff0001


#define TEE_STORAGE_PRIVATE_REE 0x80000000
#define TEE_STORAGE_PRIVATE_RPMB 0x80000100
#define TEE_STORAGE_BAD_PARAM	0xbadu

struct optee_ctx_st_host {
        int32_t inited;
        struct udevice *tee_dev;    /**< TEE device */
        struct tee_open_session_arg session;   /**< TEE session */
};

static struct optee_ctx_st_host s_ctx; //存储上下文

static uint32_t storage_type_trans(uint32_t storage_type) {
        uint32_t ret;

        if (storage_type == SECURE_STORAGE_PRIVATE_RPMB) {
                ret = TEE_STORAGE_PRIVATE_RPMB;
        } else {
                ret = TEE_STORAGE_BAD_PARAM;
        }
        return ret;
}
/**
 * @NO{S06E03C05I}
 * @brief establish connection between CA and secure storage TA
 *
 * @param[in] ctx: Represent the connection with TA
 *
 * @retval TEE_SUCCESS:Session open success
 * @retval !=TEE_SUCCESS:Session open fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_prepare_tee_session(void)
{
        const struct tee_optee_ta_uuid uuid = TA_SECSTOR_UUID;
        int res = 0;

        if (s_ctx.inited == 1)
                return 0;

        s_ctx.tee_dev = tee_find_device(NULL, NULL, NULL, NULL);
        if (!s_ctx.tee_dev) {
                res = -ENODEV;
                printf("%s:%d\n", __func__, __LINE__);
                goto exit;
        }
        memset(&s_ctx.session, 0, sizeof(s_ctx.session));
        tee_optee_ta_uuid_to_octets(s_ctx.session.uuid, &uuid);
        if (tee_open_session(s_ctx.tee_dev, &s_ctx.session, 0, NULL)) {
                res = -ENXIO;
                printf("%s:%d\n", __func__, __LINE__);
                goto exit;
        }

        s_ctx.inited = 1;

        exit:
        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief free resources and close the connection with TEE
 *
 * @param[in] ctx: Represent the connection with TA
 *
 * @return None
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_terminate_tee_session(void)
        {
        if (s_ctx.inited != 1)
                return -1;

        tee_close_session(s_ctx.tee_dev, s_ctx.session.session);

        s_ctx.inited = 0;

        return 0;
}

/**
 * @NO{S06E03C05I}
 * @brief secure storage object create
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] storage_type: indicates which Trusted Storage Space to access, SECURE_STORAGE_PRIVATE_REE or SECURE_STORAGE_PRIVATE_RPMB
 * @param[in] filename: name of the file
 * @param[in] len: length of the file name
 * @param[in] data: initial data of the created file
 * @param[in] data_size: initial data length of the created file
 * @param[in] flag: determine the settings of the created file, see DataFlag
 * @param[out] obj: contains the opened handle upon successful completion
 *
 * @retval TEE_SUCCESS: obj create success
 * @retval !=TEE_SUCCESS: obj create fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_create(uint32_t storage_type,
			const char *filename, uint32_t len,
			uint8_t *data, size_t data_size,
			uint32_t flag, uint32_t *obj)
{
        int res;
        uint32_t tee_storage_type;
        struct tee_param param[4] = {0};
        struct tee_shm *shm_filename = NULL;
        struct tee_shm *shm_data = NULL;
        struct tee_invoke_arg invoke;

        if (s_ctx.inited != 1) {
                res = -1;
                //coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1_01
                goto err_exit;
        }
        if (filename == NULL) {
                res = -1;
                printf("%s:%d filename is null\n", __func__, __LINE__);
                goto err_exit;
        }
        tee_storage_type = storage_type_trans(storage_type);
        if (TEE_STORAGE_BAD_PARAM == tee_storage_type) {
                res = TEE_ERROR_BAD_PARAMETERS;
                printf("%s:%d\n", __func__, __LINE__);
                goto err_exit;
        }
        res = tee_shm_alloc(s_ctx.tee_dev, strlen(filename) + 1, TEE_SHM_ALLOC, &shm_filename);
        if (res) {
                res = -ENOMEM;
                printf("%s:%d\n", __func__, __LINE__);
                goto err_exit;
        }
        strncpy(shm_filename->addr, filename, strlen(filename) + 1);

        if (data_size) {
                res = tee_shm_alloc(s_ctx.tee_dev, data_size, TEE_SHM_ALLOC, &shm_data);
                if (res) {
                        res = -ENOMEM;
                        printf("%s:%d\n", __func__, __LINE__);
                        goto free_file;
                }
                memcpy(shm_data->addr, data, data_size);
        }
        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = tee_storage_type;
        param[0].u.value.b     = flag;
        param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
        param[1].u.memref.shm  = shm_filename;
        param[1].u.memref.size = len;
        param[2].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
        param[2].u.memref.shm  = shm_data;
        param[2].u.memref.size = data_size;
        param[3].attr          = TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_CREATE;
        invoke.session = s_ctx.session.session;

        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 4, param);

        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        } else {
                *obj = param[3].u.value.a;
        }

        if (data_size)
                tee_shm_free(shm_data);
        free_file:
        tee_shm_free(shm_filename);
        err_exit:
        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief open the file, shoule be called before any operation
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] storage_type: indicates which Trusted Storage Space to access, SECURE_STORAGE_PRIVATE_REE or SECURE_STORAGE_PRIVATE_RPMB
 * @param[in] filename: name of the file
 * @param[in] len: length of the file name
 * @param[in] flag: determine the settings of the opened file, see DataFlag. if flag contains TEE_DATA_FLAG_CREATE, the file will be created
 * @param[out] obj: contains the opened handle upon successful completion
 *
 * @retval TEE_SUCCESS:obj open success
 * @retval !=TEE_SUCCESS:obj open fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_open(uint32_t storage_type,
			const char *filename, uint32_t len,
			uint32_t flag, uint32_t *obj)
{
        int res;
        uint32_t tee_storage_type;
        struct tee_shm *shm_filename = NULL;
        struct tee_param param[3] = {0};
        struct tee_invoke_arg invoke;

        if (s_ctx.inited != 1) {
                res = TEE_ERROR_ITEM_NOT_FOUND;
                //coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1_01
                goto err_exit;
        }

        memset(param, 0, sizeof(param));
        if ((flag & DATA_FLAG_CREATE) != 0u) {
                res = hb_secstor_create(storage_type, filename, len,
                                        NULL, 0, flag & (~DATA_FLAG_CREATE), obj);
        }
        else {
                tee_storage_type = storage_type_trans(storage_type);
                if (TEE_STORAGE_BAD_PARAM == tee_storage_type) {
                        res = TEE_ERROR_BAD_PARAMETERS;
                        //coverity[misra_c_2012_rule_15_1_violation:SUPPRESS], ## violation reason SYSSW_V_15.1_01
                        goto err_exit;
                }

                res = tee_shm_alloc(s_ctx.tee_dev, strlen(filename) + 1, TEE_SHM_ALLOC, &shm_filename);
                if (res) {
                        res = -ENOMEM;
                        goto err_exit;
                }
                strncpy(shm_filename->addr, filename, strlen(filename) + 1);

                memset(param, 0, sizeof(param));
                param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
                param[0].u.value.a     = tee_storage_type;
                param[0].u.value.b     = flag;
                param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
                param[1].u.memref.shm  = shm_filename;
                param[1].u.memref.size = len;
                param[2].attr          = TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT;

                memset(&invoke, 0, sizeof(invoke));
                invoke.func    = TA_SECSTOR_CMD_OPEN;
                invoke.session = s_ctx.session.session;

                res = tee_invoke_func(s_ctx.tee_dev, &invoke, 3, param);

                if (res != 0) {
                        printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
                } else if (invoke.ret) {
                        res = invoke.ret;
                        printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
                } else {
                        *obj = param[2].u.value.a;
                }
                tee_shm_free(shm_filename);

        }

        err_exit:
        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief close the file, shoule be called after completing the operation
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: Represent the opened file
 *
 * @retval TEE_SUCCESS:obj close success
 * @retval !=TEE_SUCCESS:obj close fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_close(uint32_t obj)
{
        int res;
        struct tee_param param[1] = {0};
        struct tee_invoke_arg invoke;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;
        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;


        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_CLOSE;
        invoke.session = s_ctx.session.session;


        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 1, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        }

        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief read data
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: Represent the opened file
 * @param[out] data: store the read data
 * @param[in] datalen: length to be read
 * @param[in] count: the lenth of data can store and the number of bytes read
 * @param[out] count: the lenth of data can store and the number of bytes read
 *
 * @retval TEE_SUCCESS:obj read success
 * @retval !=TEE_SUCCESS:obj read fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_read(uint32_t obj, void *data,
			size_t datalen, size_t *count)
{
        int res;
        struct tee_param param[2] = {0};
        struct tee_invoke_arg invoke;
        struct tee_shm *shm_data = NULL;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;

        if (*count > 0) {
                res = tee_shm_alloc(s_ctx.tee_dev, *count, TEE_SHM_ALLOC, &shm_data);
                if (res) {
                        res = -ENOMEM;
                        goto err_exit;
                }
        }

        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;
        param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
        param[1].u.memref.shm  = shm_data;
        param[1].u.memref.size = *count;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_READ;
        invoke.session = s_ctx.session.session;
        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 2, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret != TEE_SUCCESS && invoke.ret != TEE_ERROR_SHORT_BUFFER) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        } else {
                res = invoke.ret;
                *count = param[1].u.memref.size;
                if (data) {
                        memcpy(data, shm_data->addr, *count);
                }
        }
        if (*count) {
                tee_shm_free(shm_data);
        }

        err_exit:
        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief write data
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: Represent the opened file
 * @param[in] data: store the data to be writen
 * @param[in] datalen: length of the data to be writen
 *
 * @retval TEE_SUCCESS:obj write success
 * @retval !=TEE_SUCCESS:obj write fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_write(uint32_t obj, void *data, size_t datalen)
{
        int res;
        struct tee_param param[2] = {0};
        struct tee_invoke_arg invoke;
        struct tee_shm *shm_data = NULL;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;

        res = tee_shm_alloc(s_ctx.tee_dev, datalen, TEE_SHM_ALLOC, &shm_data);
        if (res) {
                res = -ENOMEM;
                goto err_exit;
        }
        memcpy(shm_data->addr, data, datalen);
        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;
        param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
        param[1].u.memref.shm  = shm_data;
        param[1].u.memref.size = datalen;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_WRITE;
        invoke.session = s_ctx.session.session;

        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 2, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d optee write efuse failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        }

        tee_shm_free(shm_data);
        err_exit:
        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief sets the data position
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: Represent the opened file
 * @param[in] offset: The number of bytes to move the data position.
 * @param[in] origin: The position in the data stream from which to calculate the new position, see DATA_Whence
 *
 * @retval TEE_SUCCESS:obj seek success
 * @retval !=TEE_SUCCESS:obj seek fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_seek(uint32_t obj, ssize_t offset, int32_t origin)
{
        int res;
        struct tee_param param[2] = {0};
        struct tee_invoke_arg invoke;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;

        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;
        param[0].u.value.b     = offset;
        param[1].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[1].u.value.a     = origin;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_SEEK;
        invoke.session = s_ctx.session.session;

        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 2, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        }

        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief delete the file and closes the object
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: Represent the opened file
 *
 * @retval TEE_SUCCESS:obj delete success
 * @retval !=TEE_SUCCESS:obj delete fail
 * @callgraph
 * @callergraph
 * @design
 */
int hb_secstor_delete(uint32_t obj)
{
        int res;
        struct tee_param param[1] = {0};
        struct tee_invoke_arg invoke;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;

        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_DELETE;
        invoke.session = s_ctx.session.session;

        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 2, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        }

        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief change the file name
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: contains the opened handle upon successful completion
 * @param[in] filename: new name of the file
 * @param[in] len: length of the new file name
 *
 * @retval TEE_SUCCESS:obj rename success
 * @retval !=TEE_SUCCESS:obj rename fail
 * @callgraph
 * @callergraph
 * @design
 */
//coverity[misra_c_2012_rule_8_7_violation:SUPPRESS], ## violation reason SYSSW_V_8.7_02
int hb_secstor_rename(uint32_t obj, const char *filename, uint32_t len)
{
        int res;
        struct tee_param param[2] = {0};
        struct tee_invoke_arg invoke;
        struct tee_shm *shm_filename = NULL;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;


        if (filename == NULL) {
                res = -1;
                printf("%s:%d filename is null\n", __func__, __LINE__);
                return res;
        }
        res = tee_shm_alloc(s_ctx.tee_dev, strlen(filename) + 1, TEE_SHM_ALLOC, &shm_filename);
        if (res) {
                res = -ENOMEM;
                printf("%s:%d\n", __func__, __LINE__);
                return res;
        }
        strncpy(shm_filename->addr, filename, strlen(filename) + 1);

        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;
        param[1].attr          = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
        param[1].u.memref.shm  = shm_filename;
        param[1].u.memref.size = len;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_RENAME;
        invoke.session = s_ctx.session.session;

        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 2, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        }

        return res;
}

/**
 * @NO{S06E03C05I}
 * @brief changes the size of a data stream. If size is less than the current size of the data stream then all bytes beyond size are removed.
 * If size is greater than the current size of the data stream then the data stream is extended by adding zero bytes at the end of the stream
 *
 * @param[in] ctx: Represent the connection with TA
 * @param[in] obj: contains the opened handle upon successful completion
 * @param[in] len: length of the new file name
 *
 * @retval TEE_SUCCESS:obj truncate success
 * @retval !=TEE_SUCCESS:obj truncate fail
 * @callgraph
 * @callergraph
 * @design
 */
//coverity[misra_c_2012_rule_8_7_violation:SUPPRESS], ## violation reason SYSSW_V_8.7_02
int hb_secstor_truncate(uint32_t obj, size_t len)
{
        int res;
        struct tee_param param[1] = {0};
        struct tee_invoke_arg invoke;

        if (s_ctx.inited != 1)
                return TEE_ERROR_ITEM_NOT_FOUND;


        memset(param, 0, sizeof(param));
        param[0].attr          = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
        param[0].u.value.a     = obj;

        memset(&invoke, 0, sizeof(invoke));
        invoke.func    = TA_SECSTOR_CMD_TRUNCATE;
        invoke.session = s_ctx.session.session;

        res = tee_invoke_func(s_ctx.tee_dev, &invoke, 2, param);
        if (res != 0) {
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, res);
        } else if (invoke.ret) {
                res = invoke.ret;
                printf("%s:%d tee_invoke_func failed with error [0x%x]\n", __func__, __LINE__, invoke.ret);
        }

        return res;
}

int64_t drobot_st_file_store(const char *file_name, char *input, uint32_t len, uint32_t store_mode)
{
        int res;
        uint32_t obj;

        res = hb_secstor_prepare_tee_session();
        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
        if (TEE_SUCCESS != res) {
                printf("%s:%d open failed! errno:%x\n", __func__, __LINE__, res);
                return HBST_ERR_OPEN;
        }

        res = hb_secstor_create(store_mode, file_name, (uint32_t)strlen(file_name), NULL, 0u,
                                        DATA_FLAG_ACCESS_READ |
                                        DATA_FLAG_ACCESS_WRITE |
                                        DATA_FLAG_ACCESS_WRITE_META |
                                        DATA_FLAG_OVERWRITE, &obj);
        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
        if (TEE_SUCCESS != res) {
                printf("%s:%d open %s failed! errno:%d\n", __func__, __LINE__, file_name, errno);
                hb_secstor_terminate_tee_session();
                return HBST_ERR_OPEN;
        }

        res = hb_secstor_write(obj, input, len);
        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
        if (TEE_SUCCESS != res) {
                printf("%s:%d write file failed! buflen:%d bufsize:%ld errno:%d\n", __func__, __LINE__, len, sizeof(input), errno);
                if (res == TEE_ERROR_STORAGE_NO_SPACE) {
                        //res = hb_secstor_delete(obj);
                        hb_secstor_terminate_tee_session();
                        return HBST_ERR_MALLOC;
                }
                if (res == TEE_ERROR_OUT_OF_MEMORY || res == TEE_ERROR_TARGET_DEAD) {
                        res = hb_secstor_prepare_tee_session();
                        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
                        if (TEE_SUCCESS != res) {
                                printf("%s:%d open failed! errno:%x\n", __func__, __LINE__, res);
                                return HBST_ERR_OPEN;
                        }
                        //res = hb_secstor_delete(obj);
                        hb_secstor_terminate_tee_session();
                        return HBST_ERR_MALLOC;
                }
                hb_secstor_close(obj);
                hb_secstor_terminate_tee_session();
                return HBST_ERR_WR;
        }
        hb_secstor_terminate_tee_session();
        return res;
}

int64_t drobot_st_file_load(const char *file_name, char **output, uint32_t *len, uint32_t store_mode)
{
        int res;
        uint32_t obj;
        unsigned char *encrypt_buf = NULL;
        size_t encrypt_len = 0;

        res = hb_secstor_prepare_tee_session();
        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
        if (TEE_SUCCESS != res) {
                printf("%s:%d open failed! errno:%d\n", __func__, __LINE__, errno);
                return HBST_ERR_OPEN;
        }

        res = hb_secstor_open(store_mode, file_name, (uint32_t)strlen(file_name),
                                        DATA_FLAG_ACCESS_WRITE_META | DATA_FLAG_ACCESS_READ | DATA_FLAG_SHARE_READ, &obj);
        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
        if (TEE_SUCCESS != res) {
                printf("%s:%d open %s failed! errno:%d\n", __func__, __LINE__, file_name, errno);
                hb_secstor_terminate_tee_session();
                return HBST_ERR_OPEN;
        }
        res = hb_secstor_read(obj, NULL, 0, &encrypt_len);
        if(TEE_ERROR_SHORT_BUFFER != res) {
                hb_secstor_close(obj);
                hb_secstor_terminate_tee_session();
                printf("%s:%d get file len failed! len:%ld errno:%d res:%u\n", __func__, __LINE__, encrypt_len, errno, res);
                return HBST_ERR_NULLOBJ;
        }

        encrypt_buf = malloc(encrypt_len);
        if (encrypt_buf == NULL) {
                printf("malloc read buffer failed\n");
                hb_secstor_close(obj);
                hb_secstor_terminate_tee_session();
                return HBST_ERR_RD;
        }
        res = hb_secstor_read(obj, encrypt_buf, 0, &encrypt_len);
        //coverity[misra_c_2012_rule_10_4_violation:SUPPRESS], ## violation reason SYSSW_V_10.4_01
        if (TEE_SUCCESS != res) {
                //coverity[misra_c_2012_rule_21_3_violation:SUPPRESS], ## violation reason SYSSW_V_21.3_02
                hb_secstor_close(obj);
                hb_secstor_terminate_tee_session();
                free(encrypt_buf);
                printf("%s:%d read file failed! errno:%d\n", __func__, __LINE__, errno);
                return HBST_ERR_RD;
        }
        hb_secstor_terminate_tee_session();
        *len = encrypt_len;
        *output = encrypt_buf;
        return HBST_OK;
}

#ifdef CONFIG_DROBOT_OPTEE_RPMB_DEBUG_CMD
static int do_x5_rpmb(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
        const char *cmd;
        int ret;
        char *tmp_buffer = NULL;
        uint64_t addr = 0;
        uint64_t write_size = 0;
        uint32_t len = 0;

        if (argc < 2)
                return CMD_RET_USAGE;

        cmd = argv[1];

        if (strcmp(cmd, "read") == 0) {
                if (argc < 3)
                        return CMD_RET_USAGE;
                addr = hextoul(argv[3], NULL);
                ret = drobot_st_file_load(argv[2], &tmp_buffer, &len, SECURE_STORAGE_PRIVATE_RPMB);
                if (ret) {
                        printf("read %s from rpmb fileed\n", argv[2]);
                } else {
                        memcpy((void *)addr, tmp_buffer, len);
                        free(tmp_buffer);
                }

                return ret;
        } else if (strcmp(cmd, "write") == 0) {
                if (argc < 5)
                        return CMD_RET_USAGE;

                addr = hextoul(argv[3], NULL);
                write_size = hextoul(argv[4], NULL);
                tmp_buffer = malloc(write_size);
                if (tmp_buffer == NULL) {
                        printf("malloc write buffer failed\n");
                        return -1;
                }
                memcpy(tmp_buffer, (void *)addr, write_size);
                ret = drobot_st_file_store(argv[2], (char *)tmp_buffer, write_size, SECURE_STORAGE_PRIVATE_RPMB);

                if (ret) {
                        printf("write %s failed\n", argv[2]);
                } else {
                        printf("write %s success\n", argv[2]);
                }
                free(tmp_buffer);
                return ret;
        }
        return CMD_RET_USAGE;
}

U_BOOT_CMD(x5_rpmb, 6, 1, do_x5_rpmb, "Read/write rpmb via optee",
		   "read file_name addr\n"
		   "write file_name addr len(hex)\n");
#endif
