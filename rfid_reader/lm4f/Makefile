
# Stellaris
PART                        = LM4F120H5QR
ROM                         = TARGET_IS_BLIZZARD_RA1
STELLARISWARE               = /home/andreas/src/stellaris

global_CFLAGS               = -I../include -pipe -std=gnu99 \
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
rfid_reader_SOURCE          = LM4F_startup.c \
                              main.c \
                              mfrc522.c \
                              uartstdio.c

include Makefile.inc
