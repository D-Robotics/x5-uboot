/* SPDX-License-Identifier: GPL-2.0+ */

.global armv8_el2_to_aarch32
armv8_el2_to_aarch32:
	cmp     x0, #0
	bne     0f
	mov     x3, x2
	mov     x2, x1
	mov     x1, x4
	ldr x0, =0xc20000fd
0:
	smc #0
	ret
