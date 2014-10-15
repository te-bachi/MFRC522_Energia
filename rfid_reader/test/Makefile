

COMPILER_PREFIX             = arm-none-eabi
BINARY_SUFFIX               = elf

# CPU = cortex-m4, cortex-m3, cortex-m1, cortex-m0, etc.
# FPU = fpv4-sp-d16 (ARMv7-M Floating-Point Extension v4, Single-Precision Floating-Point Arithmetic, 16 register)
CPU                         = cortex-m4
FPU                         = fpv4-sp-d16
FLOAT_ABI                   = softfp
CODE_GEN                    = thumb

# Stellaris
PART                        = LM4F120H5QR
ROM                         = TARGET_IS_BLIZZARD_RA1
STELLARISWARE               = /home/andreas/src/stellaris

global_CFLAGS               = -Iinclude -pipe -std=gnu99 \
                              -ffunction-sections -fdata-sections -fms-extensions \
                              -Wall -Wmissing-prototypes -Wno-uninitialized -Wstrict-prototypes

global_LDFLAGS              = --gc-sections 

debug_CFLAGS                = -O0 -ggdb
debug_LDFLAGS               = 

release_CFLAGS              = -O2
release_LDFLAGS             = 

### PROGRAMS ##################################################################

PROGRAMS                    = rfid_reader

### RFID READER ###############################################################

rfid_reader_LDSCRIPT        = LM4F.ld
rfid_reader_CFLAGS          = 
rfid_reader_LDFLAGS         = 
rfid_reader_SOURCE          = main.c \
                              mfrc522.c

include Makefile.inc

