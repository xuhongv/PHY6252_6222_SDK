

MEMORY
{
  jumptbl (rwx) : ORIGIN = 0x1fff0000, LENGTH = 1K
  gcfgtbl (rwx) : ORIGIN = 0x1fff0400, LENGTH = 1K
  sram    (rwx) : ORIGIN = 0x1fff1838, LENGTH = 0x4700
  slbuf   (rwx) : ORIGIN = 0x1fffb300, LENGTH = 0x4100
  loader  (rwx) : ORIGIN = 0x1ffff400, LENGTH = 0x0c00
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
    } > sram

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > sram

    .ARM.extab : {
        *(.ARM.extab*)
    } > sram

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > sram
    __exidx_end = ABSOLUTE(.);
    
    
    .slbuffer : {
       *(ota_partition_buffer_area)
    } > slbuf
    
   .slbloader : {
       *(ota_app_loader_area)
       *(ota_app_loader_area_d)
        . = ALIGN(4);
        *(.int_stack)
        . = ALIGN(4);
        _stack_top = ABSOLUTE(.);
	} > loader   
   
 
    ._sjtblsstore : {
       _sjtblss = ABSOLUTE(.);
    } > jumptbl   

    .data : {
        _eronly = ABSOLUTE(.);
        _sdata = ABSOLUTE(.);
        _edata = ABSOLUTE(.);
        _stextram = ABSOLUTE(.);
	*.o(_section_standby_code_)
	*.o(_section_sram_code_)
		
        _etextram = ABSOLUTE(.);

        *(.data .data.*)
         *(.text .text.*)
       *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
    } > sram



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

