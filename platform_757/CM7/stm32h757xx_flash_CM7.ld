/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**
**  Abstract    : Linker script for STM32H7 series
**                1024Kbytes FLASH and 192Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2019 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
******************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)


/* Specify the memory areas */
MEMORY
{
FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K   /* A3 OTA: Bank1 lower half=CM7, upper half (0x08080000)=CM4, Bank2 whole=upgrade staging */
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
RAM_D2 (xrw)    : ORIGIN = 0x30020000, LENGTH = 128K   /* D2 SRAM2: ETH DMA descriptors + RX pool (SRAM1 0x30000000=CM4 alias area, already assigned to CM4) */
RAM_AXI (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K   /* D1 AXI SRAM: large buffers (offload DTCM 2026-07-08) */
OPENAMP_RSC_TAB (xrw) : ORIGIN = 0x38000200, LENGTH = 0x200   /* RPMsg resource table (example's original 0x38000000 collides with ping-pong+guard, relocated) */
OPEN_AMP_SHMEM (xrw)  : ORIGIN = 0x38000400, LENGTH = 31K     /* vring+buffers (SRAM4, MPU non-cacheable) */
BKPSRAM (xrw)   : ORIGIN = 0x38800000, LENGTH = 4K     /* battery domain: PF_RETAIN variables (app_pwrfail.h; NOLOAD not cleared at startup) */
SDRAM (xrw)     : ORIGIN = 0xC0000000, LENGTH = 32M    /* external W9825G6KH FMC SDRAM. NOLOAD only: the controller comes up
                                                          in app_p5_test_run() (first main-loop pass), so nothing here is loaded,
                                                          zeroed, or touchable before that. APP_SDRAM in app_platform.h */
}
__OPENAMP_region_start__ = ORIGIN(OPEN_AMP_SHMEM);
__OPENAMP_region_end__   = ORIGIN(OPEN_AMP_SHMEM) + LENGTH(OPEN_AMP_SHMEM);
/* Highest address of the user mode stack */
_estack = 0x20020000;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* firmware identity nameplate: fixed offset 0x400 (after vector table 0x298), OTA footer/tools read version and build date/time from here.
     Offset and structure defined in app_ota.h (FW_INFO_OFF/fw_info_t), changes must be synchronized on both sides */
  .fw_info ORIGIN(FLASH) + 0x400 :
  {
    KEEP(*(.fw_info))
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH
  .ARM : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH
  .init_array : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH
  .fini_array : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
  } >RAM AT> FLASH

  /* Initialized TLS data section */
  .tdata : ALIGN(4)
  {
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
    PROVIDE(__data_end = .);
    PROVIDE(__tdata_end = .);
  } >RAM AT> FLASH

  PROVIDE( __tdata_start = ADDR(.tdata) );
  PROVIDE( __tdata_size = __tdata_end - __tdata_start );

  PROVIDE( __data_start = ADDR(.data) );
  PROVIDE( __data_size = __data_end - __data_start );

  PROVIDE( __tdata_source = LOADADDR(.tdata) );
  PROVIDE( __tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata) );
  PROVIDE( __tdata_source_size = __tdata_source_end - __tdata_source );

  PROVIDE( __data_source = LOADADDR(.data) );
  PROVIDE( __data_source_end = __tdata_source_end );
  PROVIDE( __data_source_size = __data_source_end - __data_source );


  
  /* Uninitialized data section */
  /* Uninitialized TLS data section */
  .tbss (NOLOAD) : ALIGN(4)
  {
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.tbss .tbss.*)
    . = ALIGN(4);
    PROVIDE( __tbss_end = . );
  } >RAM

  PROVIDE( __tbss_start = ADDR(.tbss) );
  PROVIDE( __tbss_size = __tbss_end - __tbss_start );
  PROVIDE( __tbss_offset = ADDR(.tbss) - ADDR(.tdata) );

  PROVIDE( __tls_base = __tdata_start );
  PROVIDE( __tls_end = __tbss_end );
  PROVIDE( __tls_size = __tls_end - __tls_base );
  PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
  PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1) );
  PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );
  PROVIDE( __arm64_tls_tcb_offset = MAX(16, __tls_align) );

  .bss (NOLOAD) : ALIGN(4)
  {
    /* This is used by the startup in order to initialize the .bss section */
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
      PROVIDE( __bss_end = .);
  } >RAM
  PROVIDE( __non_tls_bss_start = ADDR(.bss) );

  PROVIDE( __bss_start = __tbss_start );
  PROVIDE( __bss_size = __bss_end - __bss_start );


  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  

  /* ETH DMA descriptors and zero-copy RX buffer pool -> D2 SRAM2 (ETH DMA cannot access DTCM, must go in D2) */
  .lwip_sec (NOLOAD) :
  {
    . = ALIGN(4);
    *(.RxDescripSection)
    . = ALIGN(4);
    *(.TxDescripSection)
    . = ALIGN(4);
    *(.Rx_PoolSection)
    . = ALIGN(4);
  } >RAM_D2

  /* power-fail retained variable area (NOLOAD = not zeroed at startup, retention by battery; validity decided by app_pwrfail) */
  .bkpsram (NOLOAD) :
  {
    . = ALIGN(4);
    __bkpsram_start = .;
    KEEP(*(.bkpsram))
    KEEP(*(.bkpsram*))
    . = ALIGN(4);
    __bkpsram_end = .;
  } >BKPSRAM

  .sdram (NOLOAD) :
  {
    . = ALIGN(4);
    __sdram_start = .;
    KEEP(*(.sdram))
    KEEP(*(.sdram*))
    . = ALIGN(4);
    __sdram_end = .;
  } >SDRAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a:* ( * )
    libm.a:* ( * )
    libgcc.a:* ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
  .axisram (NOLOAD) : {                /* CPU-only large buffers (mbedTLS pool/RTOS heap); NOLOAD = not flashed, not zeroed (user initializes it) */
    *(.axisram)
  } > RAM_AXI

  .resource_table (NOLOAD) : {
    . = ABSOLUTE(0x38000200);
    *(.resource_table)
  } > OPENAMP_RSC_TAB

}


