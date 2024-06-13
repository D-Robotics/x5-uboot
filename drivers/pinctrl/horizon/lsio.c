// SPDX-License-Identifier: GPL-2.0
/*
 * horizon pinmux core definitions
 * Copyright (C) 2022 D-Robotics Holdings Co., Ltd.
 */

#include <common.h>
#include <dm.h>
#include <dm/pinctrl.h>
#include <dt-bindings/pinctrl/horizon-lsio-pinfunc.h>

#include "common.h"

/* Driving selector offset  */
#define DS_LSIO_I2C0_SDA 1
#define DS_LSIO_I2C0_SCL 9
#define DS_LSIO_I2C1_SDA 17
#define DS_LSIO_I2C1_SCL 25
#define DS_LSIO_I2C2_SDA 1
#define DS_LSIO_I2C2_SCL 9
#define DS_LSIO_I2C3_SDA 1
#define DS_LSIO_I2C3_SCL 9
#define DS_LSIO_I2C4_SDA 17
#define DS_LSIO_I2C4_SCL 25

#define DS_LSIO_SPI0_SCLK 1
#define DS_LSIO_SPI0_SSN  9
#define DS_LSIO_SPI0_MISO 17
#define DS_LSIO_SPI0_MOSI 25
#define DS_LSIO_SPI1_SSN_1 1
#define DS_LSIO_SPI1_SCLK 1
#define DS_LSIO_SPI1_SSN  9
#define DS_LSIO_SPI1_MISO 17
#define DS_LSIO_SPI1_MOSI 25
#define DS_LSIO_SPI2_SCLK 1
#define DS_LSIO_SPI2_SSN  9
#define DS_LSIO_SPI2_MISO 17
#define DS_LSIO_SPI2_MOSI 25
#define DS_LSIO_SPI3_SCLK 1
#define DS_LSIO_SPI3_SSN  9
#define DS_LSIO_SPI3_MISO 17
#define DS_LSIO_SPI3_MOSI 25
#define DS_LSIO_SPI4_SCLK 1
#define DS_LSIO_SPI4_SSN  9
#define DS_LSIO_SPI4_MISO 17
#define DS_LSIO_SPI4_MOSI 25
#define DS_LSIO_SPI5_SCLK 1
#define DS_LSIO_SPI5_SSN  9
#define DS_LSIO_SPI5_MISO 17
#define DS_LSIO_SPI5_MOSI 25

#define DS_LSIO_UART7_TX  9
#define DS_LSIO_UART7_RX  1
#define DS_LSIO_UART7_CTS 25
#define DS_LSIO_UART7_RTS 17
#define DS_LSIO_UART1_TX  9
#define DS_LSIO_UART1_RX  1
#define DS_LSIO_UART1_CTS 25
#define DS_LSIO_UART1_RTS 17
#define DS_LSIO_UART2_TX  9
#define DS_LSIO_UART2_RX  1
#define DS_LSIO_UART3_TX  9
#define DS_LSIO_UART3_RX  1
#define DS_LSIO_UART4_TX  9
#define DS_LSIO_UART4_RX  1
#define DS_LSIO_SPI1_SSN1 1

/* The bitfield of each pin in pull enable register */
#define PE_LSIO_I2C2_SDA BIT(6)
#define PE_LSIO_I2C2_SCL BIT(14)
#define PE_LSIO_I2C3_SDA BIT(6)
#define PE_LSIO_I2C3_SCL BIT(14)
#define PE_LSIO_I2C4_SDA BIT(22)
#define PE_LSIO_I2C4_SCL BIT(30)

#define PE_LSIO_SPI0_SCLK BIT(6)
#define PE_LSIO_SPI0_SSN  BIT(14)
#define PE_LSIO_SPI0_MISO BIT(22)
#define PE_LSIO_SPI0_MOSI BIT(30)
#define PE_LSIO_SPI1_SSN_1 BIT(6)
#define PE_LSIO_SPI1_SCLK BIT(6)
#define PE_LSIO_SPI1_SSN  BIT(14)
#define PE_LSIO_SPI1_MISO BIT(22)
#define PE_LSIO_SPI1_MOSI BIT(30)
#define PE_LSIO_SPI3_SCLK BIT(6)
#define PE_LSIO_SPI3_SSN  BIT(14)
#define PE_LSIO_SPI3_MISO BIT(22)
#define PE_LSIO_SPI3_MOSI BIT(30)
#define PE_LSIO_SPI4_SCLK BIT(6)
#define PE_LSIO_SPI4_SSN  BIT(14)
#define PE_LSIO_SPI4_MISO BIT(22)
#define PE_LSIO_SPI4_MOSI BIT(30)
#define PE_LSIO_SPI5_SCLK BIT(6)
#define PE_LSIO_SPI5_SSN  BIT(14)
#define PE_LSIO_SPI5_MISO BIT(22)
#define PE_LSIO_SPI5_MOSI BIT(30)
#define PE_LSIO_SPI1_SSN1 BIT(6)

/* The bitfield of each pin in pull select register */
#define PS_LSIO_I2C2_SDA BIT(5)
#define PS_LSIO_I2C2_SCL BIT(13)
#define PS_LSIO_I2C3_SDA BIT(5)
#define PS_LSIO_I2C3_SCL BIT(13)
#define PS_LSIO_I2C4_SDA BIT(21)
#define PS_LSIO_I2C4_SCL BIT(29)

#define PS_LSIO_SPI0_SCLK BIT(5)
#define PS_LSIO_SPI0_SSN  BIT(13)
#define PS_LSIO_SPI0_MISO BIT(21)
#define PS_LSIO_SPI0_MOSI BIT(29)
#define PS_LSIO_SPI1_SSN_1 BIT(5)
#define PS_LSIO_SPI1_SCLK BIT(5)
#define PS_LSIO_SPI1_SSN  BIT(13)
#define PS_LSIO_SPI1_MISO BIT(21)
#define PS_LSIO_SPI1_MOSI BIT(29)
#define PS_LSIO_SPI3_SCLK BIT(5)
#define PS_LSIO_SPI3_SSN  BIT(13)
#define PS_LSIO_SPI3_MISO BIT(21)
#define PS_LSIO_SPI3_MOSI BIT(29)
#define PS_LSIO_SPI4_SCLK BIT(5)
#define PS_LSIO_SPI4_SSN  BIT(13)
#define PS_LSIO_SPI4_MISO BIT(21)
#define PS_LSIO_SPI4_MOSI BIT(29)
#define PS_LSIO_SPI5_SCLK BIT(5)
#define PS_LSIO_SPI5_SSN  BIT(13)
#define PS_LSIO_SPI5_MISO BIT(21)
#define PS_LSIO_SPI5_MOSI BIT(29)
#define PS_LSIO_SPI1_SSN1 BIT(5)

/* The bitfield of each pin in schmitter trigger register */
#define ST_LSIO_I2C0_SDA BIT(0)
#define ST_LSIO_I2C0_SCL BIT(8)
#define ST_LSIO_I2C1_SDA BIT(16)
#define ST_LSIO_I2C1_SCL BIT(24)
#define ST_LSIO_I2C2_SDA BIT(0)
#define ST_LSIO_I2C2_SCL BIT(8)
#define ST_LSIO_I2C3_SDA BIT(0)
#define ST_LSIO_I2C3_SCL BIT(8)
#define ST_LSIO_I2C4_SDA BIT(16)
#define ST_LSIO_I2C4_SCL BIT(24)

#define ST_LSIO_UART7_RX  BIT(0)
#define ST_LSIO_UART7_TX  BIT(8)
#define ST_LSIO_UART7_RTS BIT(16)
#define ST_LSIO_UART7_CTS BIT(24)
#define ST_LSIO_UART1_RX  BIT(0)
#define ST_LSIO_UART1_TX  BIT(8)
#define ST_LSIO_UART1_RTS BIT(16)
#define ST_LSIO_UART1_CTS BIT(24)
#define ST_LSIO_UART2_RX  BIT(0)
#define ST_LSIO_UART2_TX  BIT(8)
#define ST_LSIO_UART3_RX  BIT(0)
#define ST_LSIO_UART3_TX  BIT(8)
#define ST_LSIO_UART4_RX  BIT(0)
#define ST_LSIO_UART4_TX  BIT(8)

#define ST_LSIO_SPI0_SCLK BIT(0)
#define ST_LSIO_SPI0_SSN  BIT(8)
#define ST_LSIO_SPI0_MISO BIT(16)
#define ST_LSIO_SPI0_MOSI BIT(24)
#define ST_LSIO_SPI1_SSN_1 BIT(0)
#define ST_LSIO_SPI1_SCLK BIT(0)
#define ST_LSIO_SPI1_SSN  BIT(8)
#define ST_LSIO_SPI1_MISO BIT(16)
#define ST_LSIO_SPI1_MOSI BIT(24)
#define ST_LSIO_SPI2_SCLK BIT(0)
#define ST_LSIO_SPI2_SSN  BIT(8)
#define ST_LSIO_SPI2_MISO BIT(16)
#define ST_LSIO_SPI2_MOSI BIT(24)
#define ST_LSIO_SPI3_SCLK BIT(0)
#define ST_LSIO_SPI3_SSN  BIT(8)
#define ST_LSIO_SPI3_MISO BIT(16)
#define ST_LSIO_SPI3_MOSI BIT(24)
#define ST_LSIO_SPI4_SCLK BIT(0)
#define ST_LSIO_SPI4_SSN  BIT(8)
#define ST_LSIO_SPI4_MISO BIT(16)
#define ST_LSIO_SPI4_MOSI BIT(24)
#define ST_LSIO_SPI5_SCLK BIT(0)
#define ST_LSIO_SPI5_SSN  BIT(8)
#define ST_LSIO_SPI5_MISO BIT(16)
#define ST_LSIO_SPI5_MOSI BIT(24)
#define ST_LSIO_SPI1_SSN1 BIT(0)

/* The bitfield of each pin in pull up enable register */
#define PU_LSIO_I2C0_SDA BIT(6)
#define PU_LSIO_I2C0_SCL BIT(14)
#define PU_LSIO_I2C1_SDA BIT(22)
#define PU_LSIO_I2C1_SCL BIT(30)

#define PU_LSIO_UART7_RX  BIT(6)
#define PU_LSIO_UART7_TX  BIT(14)
#define PU_LSIO_UART7_RTS BIT(22)
#define PU_LSIO_UART7_CTS BIT(30)
#define PU_LSIO_UART1_RX  BIT(6)
#define PU_LSIO_UART1_TX  BIT(14)
#define PU_LSIO_UART1_RTS BIT(22)
#define PU_LSIO_UART1_CTS BIT(30)
#define PU_LSIO_UART2_RX  BIT(6)
#define PU_LSIO_UART2_TX  BIT(14)
#define PU_LSIO_UART3_RX  BIT(6)
#define PU_LSIO_UART3_TX  BIT(14)
#define PU_LSIO_UART4_RX  BIT(6)
#define PU_LSIO_UART4_TX  BIT(14)

#define PU_LSIO_SPI2_SCLK BIT(6)
#define PU_LSIO_SPI2_SSN  BIT(14)
#define PU_LSIO_SPI2_MISO BIT(22)
#define PU_LSIO_SPI2_MOSI BIT(30)
/* The bitfield of each pin in pull down enable register */
#define PD_LSIO_I2C0_SDA BIT(5)
#define PD_LSIO_I2C0_SCL BIT(13)
#define PD_LSIO_I2C1_SDA BIT(21)
#define PD_LSIO_I2C1_SCL BIT(29)

#define PD_LSIO_UART7_RX  BIT(5)
#define PD_LSIO_UART7_TX  BIT(13)
#define PD_LSIO_UART7_RTS BIT(21)
#define PD_LSIO_UART7_CTS BIT(29)
#define PD_LSIO_UART1_RX  BIT(5)
#define PD_LSIO_UART1_TX  BIT(13)
#define PD_LSIO_UART1_RTS BIT(21)
#define PD_LSIO_UART1_CTS BIT(29)
#define PD_LSIO_UART2_RX  BIT(5)
#define PD_LSIO_UART2_TX  BIT(13)
#define PD_LSIO_UART3_RX  BIT(5)
#define PD_LSIO_UART3_TX  BIT(13)
#define PD_LSIO_UART4_RX  BIT(5)
#define PD_LSIO_UART4_TX  BIT(13)

#define PD_LSIO_SPI2_SCLK BIT(5)
#define PD_LSIO_SPI2_SSN  BIT(13)
#define PD_LSIO_SPI2_MISO BIT(21)
#define PD_LSIO_SPI2_MOSI BIT(29)

/* The bitfield of each pin in slew rate register */
#define SR_LSIO_SPI0_SCLK BIT(0)
#define SR_LSIO_SPI0_SSN  BIT(1)
#define SR_LSIO_SPI0_MISO BIT(2)
#define SR_LSIO_SPI0_MOSI BIT(3)
#define SR_LSIO_SPI1_SCLK BIT(4)
#define SR_LSIO_SPI1_SSN  BIT(5)
#define SR_LSIO_SPI1_MISO BIT(6)
#define SR_LSIO_SPI1_MOSI BIT(7)

/* The bitfield of each pin in mode select register */
#define MS_LSIO_I2C0   BIT(31)
#define MS_LSIO_I2C1   BIT(31)
#define MS_LSIO_UART7 BIT(31)
#define MS_LSIO_UART1 BIT(31)
#define MS_LSIO_UART2 BIT(31)
#define MS_LSIO_UART3 BIT(31)
#define MS_LSIO_UART4 BIT(31)
#define MS_LSIO_SPI2  BIT(31)

static const struct horizon_pin_desc horizon_lsio_pins_desc[] = {
	_PIN(LSIO_SPI0_SCLK, "lsio_spi0_sclk", LSIO_PINCTRL_10, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI0_SCLK, PE_LSIO_SPI0_SCLK, PS_LSIO_SPI0_SCLK, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI0_SCLK, INVALID_MS_BIT),
	_PIN(LSIO_SPI0_SSN, "lsio_spi0_ssn", LSIO_PINCTRL_10, INVALID_REG_DOMAIN, DS_LSIO_SPI0_SSN,
	     PE_LSIO_SPI0_SSN, PS_LSIO_SPI0_SSN, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_SPI0_SSN, INVALID_MS_BIT),
	_PIN(LSIO_SPI0_MISO, "lsio_spi0_miso", LSIO_PINCTRL_10, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI0_MISO, PE_LSIO_SPI0_MISO, PS_LSIO_SPI0_MISO, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI0_MISO, INVALID_MS_BIT),
	_PIN(LSIO_SPI0_MOSI, "lsio_spi0_mosi", LSIO_PINCTRL_10, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI0_MOSI, PE_LSIO_SPI0_MOSI, PS_LSIO_SPI0_MOSI, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI0_MOSI, INVALID_MS_BIT),
	_PIN(LSIO_SPI1_SSN_1, "lsio_spi1_ssn_1", LSIO_PINCTRL_14, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI1_SSN_1, PE_LSIO_SPI1_SSN_1, PS_LSIO_SPI1_SSN_1, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI1_SSN_1, INVALID_MS_BIT),
	_PIN(LSIO_SPI1_SCLK, "lsio_spi1_sclk", LSIO_PINCTRL_9, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI1_SCLK, PE_LSIO_SPI1_SCLK, PS_LSIO_SPI1_SCLK, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI1_SCLK, INVALID_MS_BIT),
	_PIN(LSIO_SPI1_SSN, "lsio_spi1_ssn", LSIO_PINCTRL_9, INVALID_REG_DOMAIN, DS_LSIO_SPI1_SSN,
	     PE_LSIO_SPI1_SSN, PS_LSIO_SPI1_SSN, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_SPI1_SSN, INVALID_MS_BIT),
	_PIN(LSIO_SPI1_MISO, "lsio_spi1_miso", LSIO_PINCTRL_9, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI1_MISO, PE_LSIO_SPI1_MISO, PS_LSIO_SPI1_MISO, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI1_MISO, INVALID_MS_BIT),
	_PIN(LSIO_SPI1_MOSI, "lsio_spi1_mosi", LSIO_PINCTRL_9, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI1_MOSI, PE_LSIO_SPI1_MOSI, PS_LSIO_SPI1_MOSI, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI1_MOSI, INVALID_MS_BIT),
	_PIN(LSIO_SPI2_SCLK, "lsio_spi2_sclk", LSIO_PINCTRL_8, LSIO_PINCTRL_8,
	     DS_LSIO_SPI2_SCLK, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_SPI2_SCLK,
	     PD_LSIO_SPI2_SCLK, ST_LSIO_SPI2_SCLK, MS_LSIO_SPI2),
	_PIN(LSIO_SPI2_SSN, "lsio_spi2_ssn", LSIO_PINCTRL_8, LSIO_PINCTRL_8, DS_LSIO_SPI2_SSN,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_SPI2_SSN, PD_LSIO_SPI2_SSN,
	     ST_LSIO_SPI2_SSN, MS_LSIO_SPI2),
	_PIN(LSIO_SPI2_MISO, "lsio_spi2_miso", LSIO_PINCTRL_8, LSIO_PINCTRL_8,
	     DS_LSIO_SPI2_MISO, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_SPI2_MISO,
	     PD_LSIO_SPI2_MISO, ST_LSIO_SPI2_MISO, MS_LSIO_SPI2),
	_PIN(LSIO_SPI2_MOSI, "lsio_spi2_mosi", LSIO_PINCTRL_8, LSIO_PINCTRL_8,
	     DS_LSIO_SPI2_MOSI, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_SPI2_MOSI,
	     PD_LSIO_SPI2_MOSI, ST_LSIO_SPI2_MOSI, MS_LSIO_SPI2),
	_PIN(LSIO_SPI3_SCLK, "lsio_spi3_sclk", LSIO_PINCTRL_7, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI3_SCLK, PE_LSIO_SPI3_SCLK, PS_LSIO_SPI3_SCLK, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI3_SCLK, INVALID_MS_BIT),
	_PIN(LSIO_SPI3_SSN, "lsio_spi3_ssn", LSIO_PINCTRL_7, INVALID_REG_DOMAIN, DS_LSIO_SPI3_SSN,
	     PE_LSIO_SPI3_SSN, PS_LSIO_SPI3_SSN, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_SPI3_SSN, INVALID_MS_BIT),
	_PIN(LSIO_SPI3_MISO, "lsio_spi3_miso", LSIO_PINCTRL_7, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI3_MISO, PE_LSIO_SPI3_MISO, PS_LSIO_SPI3_MISO, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI3_MISO, INVALID_MS_BIT),
	_PIN(LSIO_SPI3_MOSI, "lsio_spi3_mosi", LSIO_PINCTRL_7, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI3_MOSI, PE_LSIO_SPI3_MOSI, PS_LSIO_SPI3_MOSI, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI3_MOSI, INVALID_MS_BIT),
	_PIN(LSIO_SPI4_SCLK, "lsio_spi4_sclk", LSIO_PINCTRL_6, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI4_SCLK, PE_LSIO_SPI4_SCLK, PS_LSIO_SPI4_SCLK, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI4_SCLK, INVALID_MS_BIT),
	_PIN(LSIO_SPI4_SSN, "lsio_spi4_ssn", LSIO_PINCTRL_6, INVALID_REG_DOMAIN, DS_LSIO_SPI4_SSN,
	     PE_LSIO_SPI4_SSN, PS_LSIO_SPI4_SSN, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_SPI4_SSN, INVALID_MS_BIT),
	_PIN(LSIO_SPI4_MISO, "lsio_spi4_miso", LSIO_PINCTRL_6, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI4_MISO, PE_LSIO_SPI4_MISO, PS_LSIO_SPI4_MISO, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI4_MISO, INVALID_MS_BIT),
	_PIN(LSIO_SPI4_MOSI, "lsio_spi4_mosi", LSIO_PINCTRL_6, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI4_MOSI, PE_LSIO_SPI4_MOSI, PS_LSIO_SPI4_MOSI, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI4_MOSI, INVALID_MS_BIT),
	_PIN(LSIO_SPI5_SCLK, "lsio_spi5_sclk", LSIO_PINCTRL_5, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI5_SCLK, PE_LSIO_SPI5_SCLK, PS_LSIO_SPI5_SCLK, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI5_SCLK, INVALID_MS_BIT),
	_PIN(LSIO_SPI5_SSN, "lsio_spi5_ssn", LSIO_PINCTRL_5, INVALID_REG_DOMAIN, DS_LSIO_SPI5_SSN,
	     PE_LSIO_SPI5_SSN, PS_LSIO_SPI5_SSN, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_SPI5_SSN, INVALID_MS_BIT),
	_PIN(LSIO_SPI5_MISO, "lsio_spi5_miso", LSIO_PINCTRL_5, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI5_MISO, PE_LSIO_SPI5_MISO, PS_LSIO_SPI5_MISO, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI5_MISO, INVALID_MS_BIT),
	_PIN(LSIO_SPI5_MOSI, "lsio_spi5_mosi", LSIO_PINCTRL_5, INVALID_REG_DOMAIN,
	     DS_LSIO_SPI5_MOSI, PE_LSIO_SPI5_MOSI, PS_LSIO_SPI5_MOSI, INVALID_PULL_BIT,
	     INVALID_PULL_BIT, ST_LSIO_SPI5_MOSI, INVALID_MS_BIT),
	_PIN(LSIO_I2C0_SCL, "lsio_i2c0_scl", LSIO_PINCTRL_2, LSIO_PINCTRL_2, DS_LSIO_I2C0_SCL,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_I2C0_SCL, PD_LSIO_I2C0_SCL,
	     ST_LSIO_I2C0_SCL, MS_LSIO_I2C0),
	_PIN(LSIO_I2C0_SDA, "lsio_i2c0_sda", LSIO_PINCTRL_2, LSIO_PINCTRL_2, DS_LSIO_I2C0_SDA,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_I2C0_SDA, PD_LSIO_I2C0_SDA,
	     ST_LSIO_I2C0_SDA, MS_LSIO_I2C0),
	_PIN(LSIO_I2C1_SCL, "lsio_i2c1_scl", LSIO_PINCTRL_2, LSIO_PINCTRL_2, DS_LSIO_I2C1_SCL,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_I2C1_SCL, PD_LSIO_I2C1_SCL,
	     ST_LSIO_I2C1_SCL, MS_LSIO_I2C1),
	_PIN(LSIO_I2C1_SDA, "lsio_i2c1_sda", LSIO_PINCTRL_2, LSIO_PINCTRL_2, DS_LSIO_I2C1_SDA,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_I2C1_SDA, PD_LSIO_I2C1_SDA,
	     ST_LSIO_I2C1_SDA, MS_LSIO_I2C1),
	_PIN(LSIO_I2C2_SCL, "lsio_i2c2_scl", LSIO_PINCTRL_1, INVALID_REG_DOMAIN, DS_LSIO_I2C2_SCL,
	     PE_LSIO_I2C2_SCL, PS_LSIO_I2C2_SCL, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_I2C2_SCL, INVALID_MS_BIT),
	_PIN(LSIO_I2C2_SDA, "lsio_i2c2_sda", LSIO_PINCTRL_1, INVALID_REG_DOMAIN, DS_LSIO_I2C2_SDA,
	     PE_LSIO_I2C2_SDA, PS_LSIO_I2C2_SDA, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_I2C2_SDA, INVALID_MS_BIT),
	_PIN(LSIO_I2C3_SCL, "lsio_i2c3_scl", LSIO_PINCTRL_0, INVALID_REG_DOMAIN, DS_LSIO_I2C3_SCL,
	     PE_LSIO_I2C3_SCL, PS_LSIO_I2C3_SCL, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_I2C3_SCL, INVALID_MS_BIT),
	_PIN(LSIO_I2C3_SDA, "lsio_i2c3_sda", LSIO_PINCTRL_0, INVALID_REG_DOMAIN, DS_LSIO_I2C3_SDA,
	     PE_LSIO_I2C3_SDA, PS_LSIO_I2C3_SDA, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_I2C3_SDA, INVALID_MS_BIT),
	_PIN(LSIO_I2C4_SCL, "lsio_i2c4_scl", LSIO_PINCTRL_0, INVALID_REG_DOMAIN, DS_LSIO_I2C4_SCL,
	     PE_LSIO_I2C4_SCL, PS_LSIO_I2C4_SCL, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_I2C4_SCL, INVALID_MS_BIT),
	_PIN(LSIO_I2C4_SDA, "lsio_i2c4_sda", LSIO_PINCTRL_0, INVALID_REG_DOMAIN, DS_LSIO_I2C4_SDA,
	     PE_LSIO_I2C4_SDA, PS_LSIO_I2C4_SDA, INVALID_PULL_BIT, INVALID_PULL_BIT,
	     ST_LSIO_I2C4_SDA, INVALID_MS_BIT),
	_PIN(LSIO_UART7_RX, "lsio_uart7_rx", LSIO_PINCTRL_4, LSIO_PINCTRL_4, DS_LSIO_UART7_RX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART7_RX, PD_LSIO_UART7_RX,
	     ST_LSIO_UART7_RX, MS_LSIO_UART7),
	_PIN(LSIO_UART7_TX, "lsio_uart7_tx", LSIO_PINCTRL_4, LSIO_PINCTRL_4, DS_LSIO_UART7_TX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART7_TX, PD_LSIO_UART7_TX,
	     ST_LSIO_UART7_TX, MS_LSIO_UART7),
	_PIN(LSIO_UART7_RTS, "lsio_uart7_rts", LSIO_PINCTRL_4, LSIO_PINCTRL_4,
	     DS_LSIO_UART7_RTS, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART7_RTS,
	     PD_LSIO_UART7_RTS, ST_LSIO_UART7_RTS, MS_LSIO_UART7),
	_PIN(LSIO_UART7_CTS, "lsio_uart7_cts", LSIO_PINCTRL_4, LSIO_PINCTRL_4,
	     DS_LSIO_UART7_CTS, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART7_CTS,
	     PD_LSIO_UART7_CTS, ST_LSIO_UART7_CTS, MS_LSIO_UART7),
	_PIN(LSIO_UART1_RX, "lsio_uart1_rx", LSIO_PINCTRL_3, LSIO_PINCTRL_3, DS_LSIO_UART1_RX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART1_RX, PD_LSIO_UART1_RX,
	     ST_LSIO_UART1_RX, MS_LSIO_UART1),
	_PIN(LSIO_UART1_TX, "lsio_uart1_tx", LSIO_PINCTRL_3, LSIO_PINCTRL_3, DS_LSIO_UART1_TX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART1_TX, PD_LSIO_UART1_TX,
	     ST_LSIO_UART1_TX, MS_LSIO_UART1),
	_PIN(LSIO_UART1_RTS, "lsio_uart1_rts", LSIO_PINCTRL_3, LSIO_PINCTRL_3,
	     DS_LSIO_UART1_RTS, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART1_RTS,
	     PD_LSIO_UART1_RTS, ST_LSIO_UART1_RTS, MS_LSIO_UART1),
	_PIN(LSIO_UART1_CTS, "lsio_uart1_cts", LSIO_PINCTRL_3, LSIO_PINCTRL_3,
	     DS_LSIO_UART1_CTS, INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART1_CTS,
	     PD_LSIO_UART1_CTS, ST_LSIO_UART1_CTS, MS_LSIO_UART1),
	_PIN(LSIO_UART2_RX, "lsio_uart2_rx", LSIO_PINCTRL_11, LSIO_PINCTRL_11, DS_LSIO_UART2_RX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART2_RX, PD_LSIO_UART2_RX,
	     ST_LSIO_UART2_RX, MS_LSIO_UART2),
	_PIN(LSIO_UART2_TX, "lsio_uart2_tx", LSIO_PINCTRL_11, LSIO_PINCTRL_11, DS_LSIO_UART2_TX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART2_TX, PD_LSIO_UART2_TX,
	     ST_LSIO_UART2_TX, MS_LSIO_UART2),
	_PIN(LSIO_UART3_RX, "lsio_uart3_rx", LSIO_PINCTRL_12, LSIO_PINCTRL_11, DS_LSIO_UART3_RX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART3_RX, PD_LSIO_UART3_RX,
	     ST_LSIO_UART3_RX, MS_LSIO_UART3),
	_PIN(LSIO_UART3_TX, "lsio_uart3_tx", LSIO_PINCTRL_12, LSIO_PINCTRL_11, DS_LSIO_UART3_TX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART3_TX, PD_LSIO_UART3_TX,
	     ST_LSIO_UART3_TX, MS_LSIO_UART3),
	_PIN(LSIO_UART4_RX, "lsio_uart4_rx", LSIO_PINCTRL_13, LSIO_PINCTRL_11, DS_LSIO_UART4_RX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART4_RX, PD_LSIO_UART4_RX,
	     ST_LSIO_UART4_RX, MS_LSIO_UART4),
	_PIN(LSIO_UART4_TX, "lsio_uart4_tx", LSIO_PINCTRL_13, LSIO_PINCTRL_11, DS_LSIO_UART4_TX,
	     INVALID_PULL_BIT, INVALID_PULL_BIT, PU_LSIO_UART4_TX, PD_LSIO_UART4_TX,
	     ST_LSIO_UART4_TX, MS_LSIO_UART4),
};

static const struct horizon_pinctrl_priv horizon_lsio_pinctrl_info = {
	.pins  = horizon_lsio_pins_desc,
	.npins = ARRAY_SIZE(horizon_lsio_pins_desc),
};

static inline int horizon_lsio_pinctrl_set_state(struct udevice *dev, struct udevice *config)
{
	return horizon_pinctrl_set_state(dev, config, &horizon_lsio_pinctrl_info);
}

static inline int horizon_lsio_pinctrl_probe(struct udevice *dev)
{
	return horizon_pinctrl_probe(dev, &horizon_lsio_pinctrl_info);
}

static const struct pinctrl_ops horizon_pinctrl_lsio_ops = {
	.set_state		= horizon_lsio_pinctrl_set_state,
};

static const struct udevice_id horizon_lsio_pinctrl_of_match[] = {
	{
		.compatible = "d-robotics,horizon-lsio-iomuxc",
		.data = (ulong)&horizon_lsio_pinctrl_info,
	},
	{ }
};

U_BOOT_DRIVER(horizon_lsio_pinctrl) = {
	.name		= "horizon_lsio_pinctrl",
	.id		= UCLASS_PINCTRL,
	.of_match	= of_match_ptr(horizon_lsio_pinctrl_of_match),
	.priv_auto	= sizeof(struct horizon_pinctrl_priv),
	.ops		= &horizon_pinctrl_lsio_ops,
	.probe		= horizon_lsio_pinctrl_probe,
};
