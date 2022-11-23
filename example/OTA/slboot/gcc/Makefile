BIN = ./output/slboot.bin
IHEX = ./output/slboot.ihex
ELF = ./output/slboot.elf
ASM = ./output/slboot.asm


ROOT := ../../../..
PROJ_ROOT := ..

VPATH = $(PROJ_ROOT)
VPATH = $(PROJ_ROOT)/gcc
VPATH += $(PROJ_ROOT)/Source

include $(PROJ_ROOT)/gcc/slb_components.cflags
include $(PROJ_ROOT)/gcc/slb_components.mk

DEF += -DMAX_NUM_LL_CONN=1
DEF += -DGATT_MAX_NUM_CONN=MAX_NUM_LL_CONN+1
DEF += -DCFG_MTU_23=0
DEF += -DCFG_FLASH=512
DEF += -DCFG_OTA_BANK_MODE=OTA_SINGLE_BANK
DEF += -DUSE_FCT=0
DEF += -DON_SLB_BOOTLOADER=1

RAMONLY_LD = $(PROJ_ROOT)/slb.ld

SRC_RAW = main.c
SRC_RAW += ota.c
SRC_RAW += ota_flash.c
SRC_RAW += slboot.c
SRC_RAW += slb.c

VPATH += $(ROOT)/components/profiles/ota
VPATH += $(ROOT)/components/profiles/slb
INC += -I$(ROOT)/components/profiles/ota
INC += -I$(ROOT)/components/profiles/slb
INC += -I$(ROOT)/components/libraries/crc16
INC += -I$(ROOT)/components/gcc/CMSIS/device/phyplus

STARTUP_RAW = phy6222_slb_start.s

CSRCS += phy6222_slb_cstart.c


SRC_RAW += $(CSRCS)

INC += -I$(PROJ_ROOT)/source



CFLAGS += -O1
CFLAGS += -g3
CFLAGS += --specs=nosys.specs
CFLAGS += -mcpu=cortex-m0
#CFLAGS += -mfloat-abi=soft
CFLAGS += -mthumb
CFLAGS += -ffunction-sections
CFLAGS += -Wall


LFLAGS += -Wl,--gc-sections
LFLAGS += -Wl,-Map=build.map
LFLAGS += $(ROOT)/misc/bb_rom_sym_m0.gdbsym
LFLAGS += $(LIBS_PATH)
LFLAGS += $(LIBS)

OBJ = $(SRC_RAW:%.c=%.o)
OBJ :=  $(notdir $(OBJ))

STARTUP_OBJ = $(STARTUP_RAW:%.s=%.o)
STARTUP_OBJ :=  $(notdir $(STARTUP_OBJ))



all:$(OBJ) $(STARTUP_OBJ)
	@echo $(STARTUP_OBJ)
	@echo $(OBJ)
	@echo $(BIN)

	mkdir -p output

	arm-none-eabi-gcc -o $(ELF) $(CFLAGS) $(OBJ) $(STARTUP_OBJ) $(LFLAGS) -T$(RAMONLY_LD) 
	arm-none-eabi-objcopy -O binary -S $(ELF) $(BIN)
	arm-none-eabi-objcopy -O ihex $(ELF) $(IHEX)
	arm-none-eabi-objdump -s -S $(ELF) > $(ASM)
	
%.o:%.s
	arm-none-eabi-gcc -c $(CFLAGS) $< -o $@
	@echo

%.o:%.c
	arm-none-eabi-gcc -c $(CFLAGS) $(DEF) $(INC) $< -o $@
	@echo

.PHONY: clean
clean :
	@-rm  $(BIN) $(ASM) $(ELF) $(IHEX)
	-rm $(OBJ)
	-rm $(STARTUP_OBJ)
	-rm *.map
