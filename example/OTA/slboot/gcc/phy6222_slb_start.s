
/****************************************************************************
 * Included Files
 ****************************************************************************/


/****************************************************************************
 * Public Symbols
 ****************************************************************************/
#define CONFIG_STACKSIZE 2048

	.file		"phy6222_start.s"



.text
	.align	2
	.code	16
	.globl		__start
	.thumb_func
	.type	__start, %function
__start:

	ldr     r1,  =g_top_irqstack
	msr		msp, r1					/* r2>>sp */
	bl		c_start				/* R0=IRQ, R1=register save area on stack */


	.size	__start, .-__start


.section .isr_vector
    .align  4
    .globl  __Vectors
    .type   __Vectors, %object
__Vectors:
    .long   0
    .long   __start

   .size   __Vectors, . - __Vectors


.section .int_stack

    .align  2
    .global g_intstackalloc
    .global g_intstackbase
    .global g_top_irqstack

g_intstackalloc:
g_intstackbase:
     .space 1024
g_top_irqstack:




.end


