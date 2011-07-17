/* This is the size of the stack for early init and for all FreeRTOS IRQs */
_irq_stack_size = 0x400;
/* This is the size of the stack for early init: life span is until scheduler starts */
_init_stack_size = 0x400;
/* there is probably a way to get that from the MEMORY section */
_eram = ORIGIN(RAM) + LENGTH(RAM);

/* Check valid alignment for VTOR */
ASSERT(ORIGIN(FLASH) == ALIGN(ORIGIN(FLASH), 0x80), "Start of memory region flash not aligned for startup vector table");

/*PROVIDE(pios_board_info_blob = ORIGIN(BD_INFO));*/


/* Sections Definitions */

SECTIONS
{
    /* for Cortex devices, the beginning of the startup code is stored in the .isr_vector section, which goes to FLASH */
    .isr_vector :
    {
        PROVIDE (pios_isr_vector_table_base = .);
        KEEP(*(.isr_vector))            /* Startup code */
        . = ALIGN(4);
    } >FLASH

    /* for some STRx devices, the beginning of the startup code is stored in the .flashtext section, which goes to FLASH */
    .flashtext :
    {
        . = ALIGN(4);
        *(.flashtext)            /* Startup code */
        . = ALIGN(4);
    } >FLASH
 
    
    /* init sections */
    .initcalluavobj.init :
    {
        . = ALIGN(4);
	__uavobj_initcall_start = .;
        KEEP(*(.initcalluavobj.init))
	. = ALIGN(4);
	__uavobj_initcall_end   = .;
    } >FLASH

    /* module sections */
    .initcallmodule.init :
    {
        . = ALIGN(4);
	__module_initcall_start = .;
        KEEP(*(.initcallmodule.init))
	. = ALIGN(4);
	__module_initcall_end   = .;
    } >FLASH

    /* the program code is stored in the .text section, which goes to Flash */
    .text :
    {
            . = ALIGN(4);
            
        *(.text)                   /* remaining code */
        *(.text.*)                   /* remaining code */
        *(.rodata)                 /* read-only data (constants) */
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)

            . = ALIGN(4);
         _etext = .;
            /* This is used by the startup in order to initialize the .data secion */
         _sidata = _etext;
    } >FLASH
    
    /* 
     * This stack is used both as the initial sp during early init as well as ultimately
     * being used as the STM32's MSP (Main Stack Pointer) which is the same stack that
     * is used for _all_ interrupt handlers.  The end of this stack should be placed
     * against the lowest address in RAM so that a stack overrun results in a hard fault
     * at the first access beyond the end of the stack.
     */
    .irq_stack :
    {
         . = ALIGN(4);
         _irq_stack_end = . ;
         . = . + _irq_stack_size ;
         . = ALIGN(4);
         _irq_stack_top = . - 4 ;
         . = ALIGN(4);
    } >RAM
	

    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .data  : AT ( _sidata )
    {
            . = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;
        
        *(.data)
        *(.data.*)
            . = ALIGN(4);
            /* This is used by the startup in order to initialize the .data secion */
         _edata = . ;
    } >RAM
    
    

    /* This is the uninitialized data section */
    .bss :
    {
            . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;
        
        *(.bss)
        *(COMMON)
        
            . = ALIGN(4);
            /* This is used by the startup in order to initialize the .bss secion */
         _ebss = . ;
    } >RAM
    
    .heap (NOLOAD) :
    {
        _sheap = . ;
        _sheap_pre_rtos = . ;
        *(.heap)
        . = ALIGN(4);
        _eheap = . ;
        _eheap_pre_rtos = . ;
        _init_stack_end = . ; 
        _sheap_post_rtos = . ;
        . = . + _init_stack_size ;
        . = ALIGN(4);
        _eheap_post_rtos = . ;
        _init_stack_top = . - 4 ;
    } > RAM
    
    /* keep the heap section at the end of the SRAM */
    /* this will allow to claim the remaining bytes not used
    /* at run time! (done by the reset vector).

    PROVIDE ( end = _ebss );
    PROVIDE ( _end = _ebss );

    __exidx_start = .;
    __exidx_end = .;
    
    /* after that it's only debugging information. */
    
    /* remove the debugging information from the standard libraries */
    /DISCARD/ :
    {
     libc.a ( * )
     libm.a ( * )
     libgcc.a ( * )
     }

    /* Stabs debugging sections.  */
    .stab          0 : { *(.stab) }
    .stabstr       0 : { *(.stabstr) }
    .stab.excl     0 : { *(.stab.excl) }
    .stab.exclstr  0 : { *(.stab.exclstr) }
    .stab.index    0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment       0 : { *(.comment) }
    /* DWARF debug sections.
       Symbols in the DWARF debugging sections are relative to the beginning
       of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line) }
    .debug_frame    0 : { *(.debug_frame) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
}


