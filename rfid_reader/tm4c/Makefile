
# Stellaris
PART                        = TM4C123GH6PM
ROM                         = TARGET_IS_TM4C123_RB1
TIVAWARE                    = /home/andreas/src/TivaWare

global_CFLAGS               = -I../include -pipe -std=gnu99 \
                              -ffunction-sections -fdata-sections -fms-extensions \
                              -Wall -Wmissing-prototypes -Wno-uninitialized -Wstrict-prototypes
global_LDFLAGS              = -g --gc-sections -static

debug_CFLAGS                = -ggdb
debug_LDFLAGS               = 

release_CFLAGS              = -flto -Os
release_LDFLAGS             = -flto -Os

### PROGRAMS ##################################################################

PROGRAMS                    = rfid_reader

### RFID READER ###############################################################

rfid_reader_LDSCRIPT        = hello.ld
rfid_reader_CFLAGS          = 
rfid_reader_LDFLAGS         = 
rfid_reader_SOURCE          = startup_gcc.c \
                              main.c \
                              mfrc522.c \
                              uartstdio.c

include Makefile.inc

