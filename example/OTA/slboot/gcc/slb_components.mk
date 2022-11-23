

#gcc components
VPATH += $(ROOT)/components/gcc/CMSIS/device/phyplus

CSRCS += phy6222_vectors.c

VPATH += $(ROOT)/components/osal/snv
VPATH += $(ROOT)/components/profiles/Roles
VPATH += $(ROOT)/components/driver/pwm
VPATH += $(ROOT)/components/driver/uart
VPATH += $(ROOT)/components/driver/clock
VPATH += $(ROOT)/components/driver/flash
VPATH += $(ROOT)/components/driver/gpio
VPATH += $(ROOT)/components/driver/pwrmgr
VPATH += $(ROOT)/components/driver/log
VPATH += $(ROOT)/components/driver/watchdog
VPATH += $(ROOT)/components/driver/led_light
VPATH += $(ROOT)/components/libraries/fs
VPATH += $(ROOT)/components/profiles/Roles
VPATH += $(ROOT)/components/profiles/GATT
VPATH += $(ROOT)/components/profiles/DevInfo
VPATH += $(ROOT)/components/profiles/ota_app
VPATH += $(ROOT)/misc
#VPATH += $(ROOT)/components/profiles/Keys
#VPATH += $(ROOT)/components/driver/dma
#VPATH += $(ROOT)/components/profiles/Batt

CSRCS += uart.c
CSRCS += clock.c
CSRCS += flash.c
CSRCS += gpio.c
CSRCS += pwrmgr.c
CSRCS += my_printf.c
CSRCS += gap.c
CSRCS += gapgattserver.c
CSRCS += peripheral.c
CSRCS += gattservapp.c
CSRCS += jump_table.c






#LIBS = 
LIBS += -lc
LIBS += -lm
LIBS += -lphy6222_rf
LIBS += -lphy6222_sec_boot
LIBS += -lphy6222_host
#LIBS += -ltest

LIBS_PATH += -L"$(PROJ_ROOT)"
LIBS_PATH += -L"$(ROOT)/lib"
LIBS_PATH += -L"$(ROOT)/components/ethermind/lib/meshlibs/phyos/armgcc"
