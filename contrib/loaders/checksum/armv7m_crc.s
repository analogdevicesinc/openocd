/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/*
	parameters:
	r0 - address in - crc out
	r1 - char count

    description:
    Code runs from main until hitting a software break point. The nominal BP is
    at exit_point. BPs in the exception handlers indicate a catastrophic failure.
*/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

	.align	2

_start:
main:
	/* Setup VTOR to point to ISR vector */
	ldr		r3, SCB
	ldr		r2, =__isr_vector
	str		r2, [r3, #8] /* VTOR is at offset 8 into SCB */

	mov		r2, r0
	movs	r0, #0
	mvns	r0, r0
	ldr		r6, CRC32XOR
	mov		r3, r1
	movs	r4, #0
	b		ncomp
nbyte:
	ldrb	r1, [r2, r4]
	lsls	r1, r1, #24
	eors	r0, r0, r1
	movs	r5, #0
loop:
	cmp		r0, #0
	bge		notset
	lsls	r0, r0, #1
	eors	r0, r0, r6
	b		cont
notset:
	lsls	r0, r0, #1
cont:
	adds	r5, r5, #1
	cmp		r5, #8
	bne		loop
	adds	r4, r4, #1
ncomp:
	cmp		r4, r3
	bne		nbyte

.globl   exit_point
exit_point:
	bkpt	#0

	.align	2

CRC32XOR:	.word	0x04c11db7
SCB:		.word	0xe000ed00

    .align 9
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .align 1
    .thumb_func
    .type    \handler_name, %function
\handler_name :
    bkpt
    .size    \handler_name, . - \handler_name
    .endm

    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    Default_Handler

Reset_Handler:
    ldr     r0, =main
    blx     r0

	.end
