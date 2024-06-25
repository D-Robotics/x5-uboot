// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for GUC IGAADCV04A ADC
 */

#include <common.h>
#include <command.h>
#include <adc.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <power/regulator.h>
#include <linux/delay.h>
#include <tee.h>
#include <asm/arch/hb_efuse.h>

/* SW Syscon Registers */
#define ADC_GOLDEN_PASSWD 0x00
#define ADC_REG_PROTECT   0x04
#define ADC_CTRL_HI_TRIG  0x08
#define ADC_CTRL_LO_TRIG  0x0c
#define ADC_ADR           0x10
#define ADC_MODE          0x14

#define PASSWD_MASK GENMASK(31, 0)
/* Interface Bridge Registers */
#define GUC_3IN1_APB_DISABLE 0x02
#define APB_DISABLE          BIT(7)
#define GUC_3IN1_ADDR0       0x03
#define GUC_3IN1_ADDR1       0x04
#define GUC_3IN1_WDATA0      0x05
#define GUC_3IN1_WDATA1      0x06
#define GUC_3IN1_WDATA2      0x07
#define GUC_3IN1_WDATA3      0x08
#define GUC_3IN1_RWCTRL      0x09
#define R_ENABLE             BIT(7)
#define W_ENABLE             BIT(0)
#define GUC_3IN1_RDATA0      0x0a
#define GUC_3IN1_RDATA1      0x0b
#define GUC_3IN1_RDATA2      0x0c
#define GUC_3IN1_RDATA3      0x0d
#define GUC_3IN1_RW_psel     0x0e
#define RW_psel              BIT(0)

/* IP Top Registers */
#define GUC_TOP_PWD_CTRL    0x0000
#define GUC_TOP_PWD_STS     0x0004
#define GUC_TOP_PR_CTRL     0x0008
#define GUC_TOP_ANA_CTRL0   0x0010
#define GUC_TOP_ANA_CTRL1   0x0014
#define GUC_TOP_ANA_CTRL2   0x0018
#define GUC_TOP_ANA_CTRL3   0x001c
#define GUC_TOP_ANA_CTRL4   0x0024
#define GUC_TOP_ANA_CTRL5   0x0028
#define GUC_TOP_ANA_STS0    0x0040
#define GUC_TOP_ANA_STS1    0x0048
#define GUC_TOP_ANA_STS2    0x004c
#define GUC_TOP_ANA_RSV_OUT 0x0058
#define GUC_TOP_DIG_CTRL0   0x0400
#define GUC_TOP_ANA_CTRL6   0x0468
#define GUC_TOP_ANA_CTRL7   0x0474
#define GUC_TOP_ANA_STS3    0x0488
#define GUC_TOP_ANA_STS4    0x048c
#define GUC_TOP_ANA_STS5    0x0490
#define GUC_TOP_ANA_STS6    0x049c

/* ADC Controller Registers */
#define GUC_CTRL_TOP_CTRL0  0x0804
#define TRIM_MASK           GENMASK(3, 0)
#define GUC_CTRL_TOP_CTRL1  0x0808
#define GUC_CTRL_TOP_CTRL2  0x0820
#define GUC_CTRL_TOP_STS0   0x0824
#define GUC_NOR_OVERFLOW    BIT(3)
#define GUC_NOR_FULL        BIT(2)
#define GUC_NOR_HALFFULL    BIT(1)
#define GUC_NOR_EMPTY       BIT(0)
#define GUC_CTRL_TOP_STS1   0x0828
#define GUC_CTRL_TOP_CTRL3  0x082c
#define GUC_CTRL_TOP_CTRL4  0x0830
#define GUC_CTRL_TOP_CTRL5  0x0840
#define GUC_CTRL_TOP_CTRL6  0x0844
#define GUC_CTRL_TOP_CTRL7  0x0848
#define GUC_CTRL_TOP_CTRL8  0x084c
#define GUC_CTRL_TOP_CTRL9  0x0850
#define GUC_CTRL_TOP_CTRL10 0x085c
#define GUC_CTRL_TOP_CTRL11 0x0858
#define GUC_CTRL_TOP_CTRL12 0x085c
#define GUC_CTRL_TOP_CTRL13 0x0860
#define GUC_CTRL_TOP_CTRL14 0x0864
#define GUC_CTRL_TOP_CTRL15 0x0868
#define GUC_CTRL_TOP_CTRL16 0x086c
#define H_THR_OFFSET        18
#define L_THR_OFFSET        2

#define GUC_CTRL_TOP_STS2 0x0900
#define SAMPLE_MASK       GENMASK(11, 2)
#define SAMPLE_OFFSET     2

#define GUC_CTRL_TOP_STS3      0x0904
#define GUC_CTRL_TOP_STS4      0x0908
#define GUC_CTRL_TOP_STS5      0x090c
#define GUC_CTRL_TOP_STS6      0x0910
#define GUC_CTRL_TOP_STS7      0x0914
#define GUC_CTRL_TOP_STS8      0x0918
#define GUC_CTRL_TOP_STS9      0x091c
#define GUC_CTRL_TOP_STS10     0x0920
#define GUC_CTRL_TOP_STS11     0x0924
#define GUC_CTRL_TOP_STS12     0x0928
#define GUC_CTRL_TOP_STS13     0x092c
#define GUC_CTRL_TOP_CTRL17    0x09c0
#define GUC_CTRL_TOP_CTRL18    0x09c4
#define GUC_CTRL_TOP_STS14     0x09c8
#define GUC_CTRL_NOR_CTRL0     0x0a00
#define GUC_CTRL_NOR_CTRL1     0x0a04
#define GUC_CTRL_NOR_CTRL2     0x0a08
#define GUC_CTRL_NOR_CTRL3     0x0a14
#define GUC_CTRL_NOR_CTRL4     0x0a18
#define GUC_CTRL_NOR_CTRL5     0x0a20
#define GUC_CTRL_NOR_STS0      0x0a24
#define GUC_NOR_FIFO_OVERFLOW  BIT(3)
#define GUC_NOR_FIFO_FULL      BIT(2)
#define GUC_NOR_FIFO_HALF_FULL BIT(1)
#define GUC_NOR_FIFO_EMPTY     BIT(0)

#define GUC_CTRL_NOR_CTRL6            0x0a28
#define GUC_CTRL_NOR_STS1             0x0a2c
#define GUC_CTRL_NOR_CTRL7            0x0a30
#define GUC_NOR_SAMPLE_ERR_INT_EN     BIT(5)
#define GUC_NOR_FIFO_OVERFLOW_INT_EN  BIT(4)
#define GUC_NOR_FIFO_FULL_INT_EN      BIT(3)
#define GUC_NOR_FIFO_HALF_FULL_INT_EN BIT(2)
#define GUC_NOR_FIFO_NOT_EMPTY_INT_EN BIT(1)
#define GUC_NOR_SAMPLE_DONE_INT_EN    BIT(0)
#define GUC_NOR_INT_MASK              GENMASK(5, 0)

#define GUC_CTRL_NOR_STS2          0x0a34
#define GUC_NOR_INT                BIT(31)
#define GUC_NOR_SAMPLE_ERR_INT     BIT(5)
#define GUC_NOR_FIFO_OVERFLOW_INT  BIT(4)
#define GUC_NOR_FIFO_FULL_INT      BIT(3)
#define GUC_NOR_FIFO_HALF_FULL_INT BIT(2)
#define GUC_NOR_FIFO_NOT_EMPTY_INT BIT(1)
#define GUC_NOR_SAMPLE_DONE_INT    BIT(0)

#define GUC_CTRL_NOR_STS3              0x0a38
#define GUC_CTRL_NOR_CTRL8             0x0a3c
#define GUC_CTRL_NOR_CTRL9             0x0a40
#define GUC_NOR_FIFO_FULL_DMA_REQ      BIT(3)
#define GUC_NOR_FIFO_HALF_FULL_DMA_REQ BIT(2)
#define GUC_NOR_FIFO_NOT_EMPTY_DMA_REQ BIT(1)
#define GUC_NOR_SAMPLE_DONE_DMA_REQ    BIT(0)

#define GUC_CTRL_NOR_STS4  0x0a80
#define GUC_NOR_SAMPLE_ERR BIT(0)

#define GUC_CTRL_NOR_STS5 0x0a84
#define EXPECTED_CHS_MASK GENMASK(4, 0)

#define GUC_CTRL_NOR_STS6 0x0a88
#define ACTUAL_CHS_MASK   GENMASK(4, 0)

#define GUC_CTRL_NOR_STS7 0x0a8c
#define ACTUAL_DATA_MASK  GENMASK(11, 2)

#define GUC_CTRL_INJ_CTRL0 0x0b00
#define GUC_CTRL_INJ_CTRL1 0x0b04
#define GUC_CTRL_INJ_CTRL2 0x0b08
#define GUC_CTRL_INJ_CTRL3 0x0b14
#define GUC_CTRL_INJ_CTRL4 0x0b20
#define GUC_CTRL_INJ_STS0  0x0b24
#define GUC_CTRL_INJ_CTRL5 0x0b28
#define GUC_CTRL_INJ_STS1  0x0b2c
#define GUC_CTRL_INJ_CTRL6 0x0b30
#define GUC_CTRL_INJ_STS2  0x0b34
#define GUC_CTRL_INJ_STS3  0x0b38
#define GUC_CTRL_INJ_CTRL7 0x0b3c
#define GUC_CTRL_INJ_CTRL8 0x0b40
#define GUC_CTRL_INJ_STS4  0x0b80
#define GUC_CTRL_INJ_STS5  0x0b84
#define GUC_CTRL_INJ_STS6  0x0b88
#define GUC_CTRL_INJ_STS7  0x0b8c

#define IGA_ADC_MAX_CHANNELS 8
#define GUC_ADC_SLEEP_US     1000
#define GUC_ADC_TIMEOUT_US   10000

/* adc calibration information in eFuse */
#define IGAV04_ADC_EFUSE_BYTES        6
#define IGAV04_ADC_EFUSE_CALIB_MASK   GENMASK(27, 22)
#define IGAV04_ADC_EFUSE_CALIB_OFFSET 22
#define IGAV04_ADC_EFUSE_TRIM_MASK    GENMASK(31, 28)
#define IGAV04_ADC_EFUSE_TRIM_OFFSET  28

#define IGAV04_ADC_EFUSE_NEG_FLAG  BIT(5)
#define IGAV04_ADC_EFUSE_NEG_MASK  GENMASK(5, 0)

struct guc_adc_data {
    int num_bits;
    int num_channels;
};

enum convert_mode { SCAN = 0, SINGLE };

struct guc_adc {
    void __iomem *base;
    void __iomem *passwd_reg;
    struct udevice *vref;
    struct clk clk;
    int passwd;
    u32 efuse_offset;
    int vref_uv;
    int active_channel;
    int32_t calibration_offset;
    u32 trimming_value;
    u8 realbits;
    const struct guc_adc_data *data;
    u32 last_val;
    enum convert_mode guc_convert_mode;
};

/*
 * Enable sample channel(n).
 */
static void guc_adc_nor_channel_enable(struct guc_adc *info, int channel, bool enable)
{
    u32 flag;

    flag = readl_relaxed(info->base + GUC_CTRL_NOR_CTRL3);
    if (enable)
        flag |= BIT(channel);
    else
        flag &= ~BIT(channel);
    writel_relaxed(flag, info->base + GUC_CTRL_NOR_CTRL3);
}

/*
 * Abort sample when sample error.
 */
static inline void guc_adc_error_abort(struct guc_adc *info, bool abort)
{
    writel_relaxed(abort ? 1 : 0, info->base + GUC_CTRL_TOP_CTRL18);
}

/*
 * Sample mode choose: 0:scan 1:single.
 */
static void guc_adc_nor_mode(struct guc_adc *info)
{
    u32 flag;

    flag = readl_relaxed(info->base + GUC_CTRL_NOR_CTRL4);
    if (info->guc_convert_mode == SINGLE)
        flag |= BIT(0);
    else
        flag &= ~BIT(0);
    writel_relaxed(flag, info->base + GUC_CTRL_NOR_CTRL4);
}

/*
 * ADC idle: 0.
 */
int guc_adc_status(struct guc_adc *info)
{
    int status;
    status = readl_relaxed(info->base + GUC_CTRL_TOP_STS14) & BIT(0);
    return status;
}

/*
 * Enable normal ADC controller from sw reg.
 */
static void guc_adc_nor_start(struct guc_adc *info, bool enable)
{
    if (enable)
        writel_relaxed(0x1, info->base + GUC_CTRL_NOR_CTRL2);
    else
        writel_relaxed(0x0, info->base + GUC_CTRL_NOR_CTRL2);
}

int guc_adc_start_channel(struct udevice *dev, int channel)
{
    int timeout = 0;
    struct guc_adc *info = dev_get_priv(dev);

    guc_adc_nor_channel_enable(info, channel, true);
    guc_adc_nor_start(info, true);

    while (1) {
        if (!guc_adc_status(info)) {
            timeout = 0;
            break;
        } else if (timeout++ > 100) {
            printf("Timed out. ");
            return 1;
        }
    }
    guc_adc_nor_channel_enable(info, channel, false);
    guc_adc_nor_start(info, false);


    info->active_channel = channel;

    return 0;
}

void guc_adc_nor_fifo_read(struct guc_adc *info, bool enable)
{
    u32 val;

    val = readl_relaxed(info->base + GUC_CTRL_NOR_CTRL6);
    if (enable)
        val |= BIT(0);
    else
        val &= ~BIT(0);
    writel_relaxed(val, info->base + GUC_CTRL_NOR_CTRL6);
    val           = readl_relaxed(info->base + GUC_CTRL_NOR_STS1) & SAMPLE_MASK;
    info->last_val = (val >> SAMPLE_OFFSET);
    info->last_val -= info->calibration_offset;
    //printf("FIFO read last value: %d\n", info->last_val);
}

static int guc_adc_channel_data(struct udevice *dev, int channel, unsigned int *data)
{
    int timeout = 0;
    struct guc_adc *info = dev_get_priv(dev);
    while (1) {
        if ((readl_relaxed(info->base + GUC_CTRL_NOR_STS0) & BIT(0)) != 1) {
            timeout = 0;
            break;
        } else if (timeout++ > 10000) {
            printf("Timed out. \n");
            return 1;
        }
    }

    guc_adc_nor_fifo_read(info, true);
    *data = info->last_val;

    return 0;
}

int guc_adc_stop(struct udevice *dev)
{
    struct guc_adc *info = dev_get_priv(dev);

    info->active_channel = -1;

    return 0;
}

/*
 * Verify golden passwd.
 */
static inline int guc_adc_passwd_verify(struct guc_adc *info)
{
	writel_relaxed(info->passwd, info->passwd_reg);
	writel_relaxed(info->passwd, info->base + GUC_TOP_PWD_CTRL);

	return readl_relaxed(info->base + GUC_TOP_PWD_STS) & BIT(0);
}

static int guc_adc_hw_init(struct udevice *dev, struct guc_adc *info)
{
    u32 val = 0;
    int ret;

    if (dev_read_bool(dev, "guc,scan"))
        info->guc_convert_mode = SCAN;
    else
        info->guc_convert_mode = SINGLE;

    /*passwd verify*/
    ret = guc_adc_passwd_verify(info);
    if (ret) {
        dev_err(dev, "failed verify password\n");
        return ret;
    }

    /* efuse value is optional */
    ret = hb_read_efuse(info->efuse_offset, 4, (char *)&val);
    if (ret != 0 || val == 0){
        info->calibration_offset = 0;
        info->trimming_value = 0;
    }
    else{
        info->calibration_offset =
            (val & IGAV04_ADC_EFUSE_CALIB_MASK) >> IGAV04_ADC_EFUSE_CALIB_OFFSET;
        info->trimming_value =
            (val & IGAV04_ADC_EFUSE_TRIM_MASK) >> IGAV04_ADC_EFUSE_TRIM_OFFSET;
    }

    if (info->calibration_offset & IGAV04_ADC_EFUSE_NEG_FLAG)
        info->calibration_offset |= ~IGAV04_ADC_EFUSE_NEG_MASK;

    printf("ADC calibration: %d\n", info->calibration_offset);
    printf("ADC trimming: %d\n", info->trimming_value);
    writel_relaxed(info->trimming_value, info->base + GUC_CTRL_TOP_CTRL0);

	//Voltage measure mode
	val = readl_relaxed(info->base + GUC_TOP_ANA_CTRL1);
	val &= ~BIT(8);
	writel_relaxed(val, info->base + GUC_TOP_ANA_CTRL1);

    //mode select
    guc_adc_nor_mode(info);
    guc_adc_error_abort(info, true);

	//sw ctrl
	writel_relaxed(0x1, info->base + GUC_CTRL_NOR_CTRL0);

	//init internal circuit
	writel_relaxed(0x33, info->base + GUC_TOP_ANA_CTRL1);

	return 0;

}

static int guc_adc_probe(struct udevice *dev)
{
    struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
    struct guc_adc *info             = dev_get_priv(dev);
    int ret;

    ret = dev_read_u32(dev, "guc,passwd", &info->passwd);
    if(ret) {
        dev_err(dev, "invalid or missing value for guc,passwd\n");
        return ret;
    }

    ret = dev_read_u32(dev, "efuse-offset", &info->efuse_offset);
    if(ret) {
        dev_err(dev, "invalid or missing value for efuse-offset\n");
        return ret;
    }
    /* adc controller */
    info->base = dev_remap_addr_index(dev, 0);
    if (!info->base) {
        dev_err(dev, "can't get address\n");
        return -ENOENT;
    }

    info->data = (const struct guc_adc_data *)dev_get_driver_data(dev);

    /* adc passwd register*/
    info->passwd_reg = dev_remap_addr_index(dev, 1);
    if (!info->passwd_reg) {
        dev_err(dev, "can't get password address\n");
        return -ENOENT;
    }

    /*clk init*/
    ret = clk_get_by_name(dev, "adc-clk", &info->clk);
    if (ret)
        return ret;

    /* regulator init */
    ret = device_get_supply_regulator(dev, "vref-supply", &info->vref);
    if (ret) {
        dev_err(dev, "can't get vref-supply: %d\n", ret);
        return ret;
    }

    ret = regulator_set_enable(info->vref, true);
    if (ret < 0) {
        dev_err(dev, "failed to enable vref regulator\n");
        return ret;
    }

    ret = regulator_get_value(info->vref);
    if (ret < 0) {
        dev_err(dev, "can't get vref-supply value: %d\n", ret);
        return ret;
    }
    info->vref_uv = ret;

    ret = guc_adc_hw_init(dev, info);
    if (ret) {
        dev_err(dev, "failed to init hardware %d\n", ret);
        return ret;
    }

    /* VDD supplied by common vref pin */
    uc_pdata->vdd_supply     = info->vref;
    uc_pdata->vdd_microvolts = info->vref_uv;
    uc_pdata->vss_microvolts = 0;

    mdelay(1);

    return 0;
}

int guc_adc_of_to_plat(struct udevice *dev)
{
	struct adc_uclass_plat *uc_pdata = dev_get_uclass_plat(dev);
	struct guc_adc *info = dev_get_priv(dev);
	struct guc_adc_data *data;

	data = (struct guc_adc_data *)dev_get_driver_data(dev);

	info->data = data;
	uc_pdata->data_mask = (1 << info->data->num_bits) - 1;
	uc_pdata->data_format = ADC_DATA_FORMAT_BIN;
	uc_pdata->data_timeout_us = GUC_ADC_TIMEOUT_US / 5;
	uc_pdata->channel_mask = (1 << info->data->num_channels) - 1;

	return 0;
}

static const struct adc_ops guc_adc_ops = {
    .start_channel = guc_adc_start_channel,
    .channel_data  = guc_adc_channel_data,
    .stop          = guc_adc_stop,
};

static const struct guc_adc_data guc_adc_data = {
    .num_bits     = 10,
    .num_channels = 8,
};

static const struct udevice_id guc_adc_ids[] = {
    {.compatible = "guc,igav04a", .data = (ulong)&guc_adc_data}, {}};

U_BOOT_DRIVER(guc_adc) = {
    .name      = "guc_adc",
    .id        = UCLASS_ADC,
    .of_match  = guc_adc_ids,
    .ops       = &guc_adc_ops,
    .probe     = guc_adc_probe,
    .of_to_plat= guc_adc_of_to_plat,
    .priv_auto = sizeof(struct guc_adc),
};
