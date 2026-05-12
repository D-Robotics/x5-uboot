
#include <config.h>
#include <asm/psci.h>

/* 使用sram1地址，起始地址0x1FE80000 大小为256k */

MEMORY { .sram : ORIGIN = 0x1FF00000,
                LENGTH = 0x40000 }


OUTPUT_FORMAT("elf64-littleaarch64", "elf64-littleaarch64", "elf64-littleaarch64")
OUTPUT_ARCH(aarch64)
ENTRY(setup_c_runtime)
SECTIONS
{
	.text : {
		. = ALIGN(8);
		*(.__image_copy_start)
		setup.o (.text*)
		*(.text*)
		KEEP(*(SORT(.text*)));
	} >.sram

	.rodata : {
		. = ALIGN(8);
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
	} >.sram

	.data : {
		. = ALIGN(8);
		*(.data*)
			KEEP(*(SORT(.data*)));
	} >.sram

	.image_copy_end : {
		. = ALIGN(8);
		*(.__image_copy_end)
	} >.sram

	.end : {
		. = ALIGN(8);
		*(.__end)
	} >.sram

	_image_binary_end = .;

	.bss_start (NOLOAD) : {
		. = ALIGN(8);
		KEEP(*(.__bss_start));
	} >.sram

	.bss (NOLOAD) : {
		*(.bss*)
			. = ALIGN(8);
	} >.sram

	.bss_end (NOLOAD) : {
		KEEP(*(.__bss_end));
	} >.sram

	/DISCARD/ : { *(.dynsym) }
	/DISCARD/ : { *(.dynstr*) }
	/DISCARD/ : { *(.dynamic*) }
	/DISCARD/ : { *(.plt*) }
	/DISCARD/ : { *(.interp*) }
	/DISCARD/ : { *(.gnu*) }
}
