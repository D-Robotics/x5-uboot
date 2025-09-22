/*
 *   Copyright 2024 D-Robotics, Inc.
 */
#ifndef BOARD_X5_SET_PIN_H_
#define BOARD_X5_SET_PIN_H_

#define gpio_data		(0x00)
#define gpio_dir		(0x04)

enum pull_state {
	NO_PULL = 0,
	PULL_UP,
	PULL_DOWN,
};

enum pin_dir {
	IN = 0,
	OUT,
};

static volatile unsigned int X5DspPinMuxBase = 0x31040000 ;
static volatile unsigned int X5LsioPinMuxBase = 0x34180000 ;
static volatile unsigned int X5LsioGPIO0Base = 0x34120000 ;
static volatile unsigned int X5LsioGPIO1Base = 0x34130000 ;
static volatile unsigned int X5DspGPIO0Base = 0x32150000 ;

enum gpio_group {
	LSIO_GPIO_0 = 0,
	LSIO_GPIO_1,
	DSP_GPIO_0,
};

struct x5_pin {
	int pin_index_40pin;
	int pin_index_bcm;
	int pin_index_x5;
	unsigned long long pin_iomux_offset;
	int pin_iomux_bit;
	unsigned long long pin_ctr_offset;
	int pin_schmit_bit;
	int pin_drvstrength_bit;
	int pin_pd_bit;
	int pin_pu_bit;
	int pin_pe_bit; //PULL 1: enable, 0: disable
	int pin_ps_bit; //1,PU 0,PD
	int pin_strong_pu_bit; //1:Enable, 0:Disable
	int pin_gpio_group;
	int pin_gpio_number;
	char func_name0[30];
	char func_name1[30];
	char func_name2[30];
	char func_name3[30];
};

#define PIN_MAX_NUMS 28

struct x5_pin pin_info_array[] = {
	{3,2,390,0x84,22,0x3C,8,9,13,14,-1,-1,-1,LSIO_GPIO_0,11,"LSIO_UART3_TXD","LSIO_I2C5_SDA","LSIO_GPIO0_PIN11","Reserved"}, //0
	{5,3,389,0x84,20,0x3C,0,1,5,6,-1,-1,-1,LSIO_GPIO_0,10,"LSIO_UART3_RXD","LSIO_I2C5_SCL","LSIO_GPIO0_PIN10","Reserved"},
	{7,4,420,0x34,18,0x28,8,9,13,14,-1,-1,-1,DSP_GPIO_0,9,"DSP_MCLK1","DSP_GPIO0_PIN09","Reserved","Reserved"},
	{11,17,380,0x84,6,0x1C,8,9,13,14,-1,-1,-1,LSIO_GPIO_0,1,"LSIO_UART7_TXD","Reserved","LSIO_GPIO0_PIN01","Reserved"},
	{13,27,379,0x84,4,0x1C,0,1,5,6,-1,-1,-1,LSIO_GPIO_0,0,"LSIO_UART7_RXD","Reserved","LSIO_GPIO0_PIN00","Reserved"},
	{15,22,388,0x84,18,0x38,8,9,13,14,-1,-1,-1,LSIO_GPIO_0,9,"LSIO_UART2_TXD","Reserved","LSIO_GPIO0_PIN09","Reserved"}, //5
	{19,10,398,0x78,8,0x30,24,25,-1,-1,29,30,31,LSIO_GPIO_0,19,"LSIO_SPI1_MOSI","LSIO_GPIO0_PIN19","JTG_TDO","Reserved"},
	{21,9,397,0x78,6,0x30,16,17,-1,-1,21,22,23,LSIO_GPIO_0,18,"LSIO_SPI1_MISO","LSIO_GPIO0_PIN18","JTG_TDI","Reserved"},
	{23,11,395,0x78,2,0x30,0,1,-1,-1,5,6,7,LSIO_GPIO_0,16,"LSIO_SPI1_SCLK","LSIO_GPIO0_PIN16","JTG_TCK","Reserved"},
	{27,0,355,0x80,18,0x10,0,1,5,6,-1,-1,-1,LSIO_GPIO_1,8,"LSIO_I2C0_SDA","LSIO_GPIO1_PIN08","Reserved","LSIO_PWM_OUT5"},
	{29,5,399,0x7C,16,0x2C,0,1,5,6,-1,-1,-1,LSIO_GPIO_0,20,"LSIO_SPI2_SCLK","LSIO_GPIO0_PIN20","Reserved","LSIO_PWM_OUT0"}, //10
	{31,6,400,0x7C,18,0x2C,8,9,13,14,-1,-1,-1,LSIO_GPIO_0,21,"LSIO_SPI2_SSN","LSIO_GPIO0_PIN21","Reserved","LSIO_PWM_OUT1"},
	{33,13,357,0x80,22,0x10,16,17,21,22,-1,-1,-1,LSIO_GPIO_1,10,"LSIO_I2C1_SDA","LSIO_GPIO1_PIN10","TIME_SYNC2","LSIO_PWM_OUT7"},
	{35,19,422,0x34,22,0x24,16,17,21,22,-1,-1,-1,DSP_GPIO_0,11,"DSP_I2S1_WS","DSP_GPIO0_PIN11","Reserved","Reserved"},
	{37,26,401,0x7C,20,0x2C,16,17,21,22,-1,-1,-1,LSIO_GPIO_0,22,"LSIO_SPI2_MISO","LSIO_GPIO0_PIN22","Reserved","LSIO_PWM_OUT2"},

	{8,14,383,0x84,14,0x18,8,9,13,14,-1,-1,-1,LSIO_GPIO_0,5,"LSIO_UART1_TXD","Reserved","LSIO_GPIO0_PIN05","Reserved"}, //15
	{10,15,384,0x84,12,0x18,0,1,5,6,-1,-1,-1,LSIO_GPIO_0,4,"LSIO_UART1_RXD","Reserved","LSIO_GPIO0_PIN04","Reserved"},
	{12,18,421,0x34,20,0x24,8,9,13,14,-1,-1,-1,DSP_GPIO_0,4,"DSP_I2S1_SCLK","DSP_GPIO0_PIN10","Reserved","Reserved"},
	{16,23,382,0x84,10,0x1C,16,27,21,22,-1,-1,-1,LSIO_GPIO_0,3,"LSIO_UART7_RTS_N","UART6_TXD","LSIO_GPIO0_PIN03","Reserved"},
	{18,24,402,0x7C,22,0x2C,24,25,29,30,-1,-1,-1,LSIO_GPIO_0,23,"LSIO_SPI2_MOSI","LSIO_GPIO0_PIN23","Reserved","LSIO_PWM_OUT3"},
	{22,25,387,0x84,16,0x38,0,1,5,6,-1,-1,-1,LSIO_GPIO_0,8,"LSIO_UART2_RXD","Reserved","LSIO_GPIO0_PIN08","Reserved"}, //20
	{24,8,394,0x78,0,0x44,0,1,-1,-1,5,6,7,LSIO_GPIO_0,15,"LSIO_SPI1_SSN_1","LSIO_GPIO0_PIN15","JTG_TMS","Reserved"},
	{26,7,396,0x78,4,0x30,8,9,-1,-1,13,14,15,LSIO_GPIO_0,17,"LSIO_SPI1_SSN","LSIO_GPIO0_PIN17","JTG_TRSTN","Reserved"},
	{28,1,354,0x80,16,0x10,8,9,13,14,-1,-1,-1,LSIO_GPIO_1,7,"LSIO_I2C0_SCL","LSIO_GPIO1_PIN07","Reserved","LSIO_PWM_OUT4"},
	{32,12,356,0x80,20,0x10,24,25,29,30,-1,-1,-1,LSIO_GPIO_1,9,"LSIO_I2C1_SCL","LSIO_GPIO1_PIN09","TIME_SYNC1","LSIO_PWM_OUT6"},
	{36,16,381,0x84,8,0x1C,24,25,29,30,-1,-1,-1,LSIO_GPIO_0,2,"LSIO_UART7_CTS_N","UART6_RXD","LSIO_GPIO0_PIN02","Reserved"}, //25
	{38,20,423,0x34,14,0x24,24,25,29,30,-1,-1,-1,DSP_GPIO_0,12,"DSP_I2S1_DI","DSP_GPIO0_PIN12","Reserved","Reserved"},
	{40,21,424,0x34,16,0x24,0,1,5,6,-1,-1,-1,DSP_GPIO_0,13,"DSP_I2S1_DO","DSP_GPIO0_PIN13","Reserved","Reserved"},
};

#endif // BOARD_X5_SET_PIN_H_
