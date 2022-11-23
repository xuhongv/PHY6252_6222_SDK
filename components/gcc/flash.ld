

MEMORY
{
  jumptbl (rwx) : ORIGIN = 0x1fff0000, LENGTH = 1K
  gcfgtbl  (rwx) : ORIGIN = 0x1fff0400, LENGTH = 1K
  flash (rx) : ORIGIN = 0x11009000, LENGTH = 384K
  sram (rwx) : ORIGIN = 0x1fff1880, LENGTH = 59264
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(__start)

SECTIONS
{

    .textentry : {
        _stext = ABSOLUTE(.);
        *(*.isr_vector)
        *phy6222_start.o(.text)
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

 
    ._sjtblsstore : {
       _sjtblss = ABSOLUTE(.);
    } > flash   

   ._eronlystore : {
       _eronly = ABSOLUTE(.);
   } > flash   

    .data : {
        _sdata = ABSOLUTE(.);
        _stextram = ABSOLUTE(.);
		*.o(_section_standby_code_)
		*.o(_section_sram_code_)
		*flash.o(.text .text.*)

		*patch.o(.text .text.*)

		*clock.o(.text.hal_xtal16m_cap_Set)
		*rf_phy_driver(.text .text.*)
		*libphy6222_host.a:l2cap_util.o(.text.L2CAP_Fragment_SendDataPkt)
		*libphy6222_host.a:l2cap_util.o(.text.l2capSegmentBuffToLinkLayer)
		*libphy6222_host.a:l2cap_util.o(.text.l2capPocessFragmentTxData)
		
        _etextram = ABSOLUTE(.);


        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash



    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        __heap_start__ = .;
        end = __heap_start__;
        _end = end;
        __end = end;
		
        _ebss = ABSOLUTE(.);
    } > sram

    .int_stack : {
        . = ALIGN(4);
        *(int_stack)
        . = ALIGN(4);
        _stack_top = ABSOLUTE(.);
    } > sram
   .common_text : {
       *(.text .text.*)
       *(.rodata .rodata.*)
       *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
   } > flash   
   
   
    /* Stabs debugging sections. */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}

