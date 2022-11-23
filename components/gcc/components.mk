

#gcc components
VPATH += $(ROOT)/components/gcc/CMSIS/device/phyplus

CSRCS += phy6222_cstart.c
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
VPATH += $(ROOT)/components/driver/adc
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

CSRCS += osal_snv.c
CSRCS += central.c
CSRCS += pwm.c
CSRCS += uart.c
CSRCS += clock.c
CSRCS += flash.c
CSRCS += gpio.c
CSRCS += pwrmgr.c
CSRCS += watchdog.c
CSRCS += led_light.c
CSRCS += my_printf.c
CSRCS += fs.c
CSRCS += gap.c
CSRCS += gapbondmgr.c
CSRCS += gapgattserver.c
CSRCS += peripheral.c
CSRCS += gattservapp.c
CSRCS += devinfoservice.c
CSRCS += ota_app_service.c
CSRCS += jump_table.c



ifdef CONFIG_PHY6222_PHY_MESH
	ifndef CONFIG_MIJIA_APIS
		VPATH += $(ROOT)/components/ethermind/external/crypto/sha256
		VAPTH += $(ROOT)/components/ethermind/mesh/export/climodel
		
		CSRCS += sha256.c
		CSRCS += cli_model.c
	endif

	VPATH += $(ROOT)/components/ethermind/platforms
	VPATH += $(ROOT)/components/ethermind/external/crypto/aes
	VPATH += $(ROOT)/components/ethermind/osal/src/phyos
	VPATH += $(ROOT)/components/ethermind/platforms/mesh
	VPATH += $(ROOT)/components/ethermind/platforms/interfaces/crypto
	VPATH += $(ROOT)/components/ethermind/mesh/export/bearer
	VPATH += $(ROOT)/components/ethermind/mesh/export/platforms/ext
	VPATH += $(ROOT)/components/ethermind/mesh/export/appl
	VPATH += $(ROOT)/components/ethermind/mesh/export/vendormodel/server
	VPATH += $(ROOT)/components/ethermind/mesh/export/climodel

	CSRCS += EM_platform.c
	CSRCS += aes.c
	CSRCS += aes-ccm.c
	CSRCS += EM_debug.c
	CSRCS += EM_os.c
	CSRCS += EM_timer.c
	CSRCS += blebrr_pl.c
	CSRCS += cry.c
	CSRCS += blebrr.c
	CSRCS += blebrr_gatt.c
	CSRCS += MS_common_pl.c
	CSRCS += prov_pl.c
	CSRCS += mesh_services.c
	CSRCS += model_state_handler_pl.c
	CSRCS += appl_prov.c
	CSRCS += vendormodel_server.c
	CSRCS += cli_model.c
endif

ifdef CONFIG_MIJIA_APIS
	VPATH += $(ROOT)/components/xiaomi/api
	VPATH += $(ROOT)/components/xiaomi/libs/third_party/mbedtls
	VPATH += $(ROOT)/components/xiaomi/libs/third_party/pt
	VPATH += $(ROOT)/components/xiaomi/libs/third_party/micro-ecc
	VPATH += $(ROOT)/components/xiaomi/libs


	CSRCS += mible_api.c
	CSRCS += mible_mcu.c
	CSRCS += mible_mesh_api.c
	CSRCS += phyplus_common.c
	CSRCS += phyplus_gap.c
	CSRCS += phyplus_gap_mesh.c
	CSRCS += phyplus_gatt.c

	CSRCS += phyplus_mesh.c
	CSRCS += phyplus_misc.c
	CSRCS += phyplus_nvm.c
	CSRCS += phyplus_os.c
	CSRCS += xmodem.c

	CSRCS += asn1parse.c
	CSRCS += base64.c
	CSRCS += ccm.c
	CSRCS += md.c
	CSRCS += md_wrap.c
	CSRCS += sha1.c
	CSRCS += sha256.c
	CSRCS += sha256_hkdf.c
	CSRCS += pt_misc.c
	CSRCS += uECC.c
	CSRCS += mi_config.c

endif

ifdef CONFIG_MIJIA_FCT
	VPATH += $(ROOT)/components/xiaomi
	VPATH += $(ROOT)/components/xiaomi
	VPATH += $(ROOT)/components/xiaomi

	CSRCS += mijia_mp_fct.c
	CSRCS += mijia_mp_cmd.c
	CSRCS += mijia_mp_cmd_parser.c
endif

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
