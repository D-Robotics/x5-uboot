

#include <asm/macro.h>
#include <linux/linkage.h>

ENTRY(setup_c_runtime)
	stp     x29, x30, [sp,#-96]!
	stp     x27, x28, [sp,#80]
	stp     x25, x26, [sp,#64]
	stp     x23, x24, [sp,#48]
	stp     x21, x22, [sp,#32]
	stp     x19, x20, [sp,#16]

	mov     x25, sp

	mov     x20, #0x0000
	movk    x20, #0x1FF8,LSL #16
	mov     sp, x20
	stp     x25, x0, [sp, #-16]!

	bl      do_test_in_sram

	ldp     x25, x0, [sp],#16
	mov     sp, x25
	ldp     x19, x20, [sp,#16]
	ldp     x21, x22, [sp,#32]
	ldp     x23, x24, [sp,#48]
	ldp     x25, x26, [sp,#64]
	ldp     x27, x28, [sp,#80]
	ldp     x29, x30, [sp],#96
	mov     w0, #0x0                        // #0
	ret
ENDPROC(setup_c_runtime)

ENTRY(_asm_invalidate_icache_all)
ic      ialluis
isb     sy
ret
ENDPROC(_asm_invalidate_icache_all)
