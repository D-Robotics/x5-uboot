/*
 *   Copyright 2021 Horizon Robotics, Inc.
 */
#define ISB        asm volatile ("isb sy" : : : "memory")
#define DSB        asm volatile ("dsb sy" : : : "memory")
#define DMB        asm volatile ("dmb sy" : : : "memory")
#define isb()      ISB
#define dsb()      DSB
#define dmb()      DMB
/*
 *  * Generic virtual read/write.  Note that we don't support half-word
 *   * read/writes.  We define __arch_*[bl] here, and leave __arch_*w
 *    * to the architecture specific code.
 *     */
#define __arch_getl(a)                   (*(volatile unsigned int *)(a))
#define __arch_putl(v, a)                (*(volatile unsigned int *)(a) = (v))
/*
 *  * TODO: The kernel offers some more advanced versions of barriers, it might
 *   * have some advantages to use them instead of the simple one here.
 *    */
#define mb()             dsb()
#define __iormb()        dmb()
#define __iowmb()        dmb()

#define writel(v, c)    ({ unsigned int __v = v; __iowmb(); __arch_putl(__v, c); __v; })
#define readl(c)        ({ unsigned int __v = __arch_getl(c); __iormb(); __v; })


#define DDR_2G_END  (0xFFFFFFFF)
#define UBOOT_START (0x88000000) //128M
#define UBOOT_END (0x90000000)
#define OPTEE_KERNEL_START (0x80000000)
#define OPTEE_KERNEL_END (0x88000000)

typedef struct result{
	int ret_val;
}result_t;

/*
 *start: start address for test
 *size: test length
 * */
void do_test_in_sram(unsigned int start, unsigned int size)
{
	result_t* ret = (result_t*)0x1FF80100;
	volatile unsigned int addr;
	unsigned int i;
	unsigned int words = (size + 3) / 4;
	unsigned int temp_origin = 0;
	unsigned int flag = 0;

	if (size == 0 || start > DDR_2G_END || size > (DDR_2G_END - start + 1)) {
		ret->ret_val = -1;
		return;
	}

	ret->ret_val = 0;

	for (i = 0; i < words; i++) {
		addr = start + i * 4;

		if ((addr >= OPTEE_KERNEL_START && addr <= OPTEE_KERNEL_END) ||
			(addr > UBOOT_START && addr <= UBOOT_END)) {
			temp_origin = readl(addr);
			flag = 1;
		}

		writel(0x5A5A5A5A, addr);

		unsigned int temp_cur = readl(addr);
		if (temp_cur == 0x5A5A5A5A) {
			ret->ret_val++;
		} else {
			ret->ret_val--;
		}

		if (flag) {
			writel(temp_origin, addr);
			flag = 0;
		}
	}

	return;
}
