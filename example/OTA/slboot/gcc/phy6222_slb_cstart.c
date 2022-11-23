
/****************************************************************************
    Included Files
 ****************************************************************************/

#include "rom_sym_def.h"
#include <stddef.h>
#include "phy6222_cstart.h"
#include "clock.h"
#include "log.h"
#include "OSAL.h"
#include "flash.h"
#include "jump_function.h"
/* #include "rf_phy_driver.h" */

/****************************************************************************
    Pre-processor Definitions
 ****************************************************************************/

extern const uint32_t _sramscttexts;
extern const uint32_t _sramscttext;
extern const uint32_t _eramscttext;

extern const uint32_t _sjtblss;
extern const uint32_t _sjtbls;
extern const uint32_t _ejtbls;

extern const uint32_t _sbss;
extern const uint32_t _ebss;

extern const uint32_t _eronly;
extern const uint32_t _sdata;
extern const uint32_t _edata;

/****************************************************************************
    Name: c_start

    Description:
     This is the reset entry point.

 ****************************************************************************/
extern int  main(void);

void c_start(void)
{
    //const uint8_t *src;
    //uint8_t *dest;
    //uint8_t *edest;
    AP_PCR->CACHE_BYPASS = 1; //just bypass cache
    /*  Clear .bss.  We'll do this inline (vs. calling memset) just to be
        certain that there are no issues with the state of global variables.
    */
    /*  Move the initialized data section from his temporary holding spot in
        FLASH into the correct place in SRAM.  The correct place in SRAM is
        give by _sdata and _edata.  The temporary location is in FLASH at the
        end of all of the other read-only data (.text, .rodata) at _eronly.
    */
    main();

    /* Shouldn't get here */

    for (; ; );
}

