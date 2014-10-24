
# CPU = cortex-m4, cortex-m3, cortex-m1, cortex-m0, etc.
# FPU = fpv4-sp-d16 (ARMv7-M Floating-Point Extension v4, Single-Precision Floating-Point Arithmetic, 16 register)
CPU                 = cortex-m4
FPU                 = fpv4-sp-d16
FLOAT_ABI           = softfp
CODE_GEN            = thumb

COMPILER_PREFIX     = arm-none-eabi
CC                  = ${COMPILER_PREFIX}-gcc
LD                  = ${COMPILER_PREFIX}-ld
OBJCOPY             = ${COMPILER_PREFIX}-objcopy
OBJDUMP             = ${COMPILER_PREFIX}-objdump
BUILD               = debug release
ARCH                = -m$(CODE_GEN) -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)

LIB_GCC_PATH        = ${shell ${CC} ${CFLAGS} -print-libgcc-file-name}
LIBC_PATH           = ${shell ${CC} ${CFLAGS} -print-file-name=libc.a}
LIBM_PATH           = ${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

global_CFLAGS      += -DPART_$(PART) -D$(ROM) -I$(TIVAWARE)
global_LDFLAGS     += ${TIVAWARE}/driverlib/gcc/libdriver.a \
                      $(LIBM_PATH) $(LIBC_PATH) $(LIB_GCC_PATH)

OBJ_DIRECTORY       = obj
SRC_DIRECTORY       = ../src

ELF_SUFFIX          = .elf
HEX_SUFFIX          = .hex
BIN_SUFFIX          = .bin
LST_SUFFIX          = .lst

OBJCOPY_HEX_FLAGS   = -O ihex
OBJCOPY_BIN_FLAGS   = -O binary
OBJDUMP_FLAGS       = -S

### HELP ######################################################################
#
# target: dependencies
# [tab] system command
#
# $(foreach var,list,text)      For each newly created variable 'var' in 'list',
#                               do whatever in 'text'
# $(call variable,param,...)    Call a defined subroutine
# $(1), $(2)                    Arguments of defined subroutines
# $(var:pattern=replacement)    Substitutes 'var' with 'replacement'
# $(eval var)                   Expand 'var' and parse as makefile syntax
# $(info var)                   Debug Makefile
# $(dir filename)               Extracts the directory-part of each filename
# 

all:
	@echo Use 'make debug' or 'make release'

### MKDIR FOR EVERY PROGRAM ###################################################
# $(1) = build type, ex. debug, release
# $(2) = program name

define DIRECTORY_template

obj/$(1)/$(2):
	@echo "[MKDIR] $$@"
	@mkdir -p obj/$(1)/$(2)

endef

### VARIABLE FOR EVERY PROGRAM ################################################
# $(1) = build type, ex. debug, release
# $(2) = program name

define VARIABLE_template

$(1)_$(2)_OBJECT = $(addprefix obj/$(1)/$(2)/,$($(2)_SOURCE:%.c=%.o))

$(foreach folder,$(addprefix $(2)/,$(sort $(dir $($(2)_SOURCE)))),$(eval $(call DIRECTORY_template,$(1),$(folder))))

endef

### OBJECT FOR EVERY SOURCE FILE ##############################################
# $(1) = build type, ex. debug, release
# $(2) = program name
# $(3) = source directory
# $(4) = one single source file
# $(5) = specific CFLAGS

define OBJECT_template

obj/$(1)/$(2)/$($(4):%.c=%.o): obj/$(1)/$(2)/$(dir $($(4))) $(3)/$($(4))
	@echo "[CC   ] $(3)/$($(4))"
	$(CC) $(ARCH) -o $$@ -c $(3)/$($(4)) $(global_CFLAGS) $($(5)) $($(2)_CFLAGS)

endef

### PROGRAM ###################################################################
# $(1) = build type, ex. debug, release
# $(2) = program name
# $(3) = install directory
# $(4) = specific CFLAGS
# $(5) = specific LDFLAGS

define PROGRAM_template

$(foreach source,$($(2)_SOURCE),$(eval $(call OBJECT_template,$(1),$(2),$(SRC_DIRECTORY),source,$(4))))

$(3)/$(1)/$(2)$(ELF_SUFFIX): $($(1)_$(2)_OBJECT)
	@mkdir -p $(3)/$(1)
	
	@echo "[LD   ] $(3)/$(1)/$(2)$(ELF_SUFFIX)"
	$(LD) -T $($(2)_LDSCRIPT) -o $(3)/$(1)/$(2)$(ELF_SUFFIX) $($(1)_$(2)_OBJECT) $(global_LDFLAGS) $($(5)) $($(2)_LDFLAGS)
	
	@echo "[HEX  ] $(3)/$(1)/$(2)$(HEX_SUFFIX)"
	@$(OBJCOPY) $(OBJCOPY_HEX_FLAGS) $(3)/$(1)/$(2)$(ELF_SUFFIX) $(3)/$(1)/$(2)$(HEX_SUFFIX)
	
	@echo "[BIN  ] $(3)/$(1)/$(2)$(BIN_SUFFIX)"
	@$(OBJCOPY) $(OBJCOPY_BIN_FLAGS) $(3)/$(1)/$(2)$(ELF_SUFFIX) $(3)/$(1)/$(2)$(BIN_SUFFIX)
	
	@echo "[LST  ] $(3)/$(1)/$(2)$(LST_SUFFIX)"
	@$(OBJDUMP) $(OBJDUMP_FLAGS) $(3)/$(1)/$(2)$(ELF_SUFFIX) > $(3)/$(1)/$(2)$(LST_SUFFIX)

$(1)_$(2)_clean:
	@echo "[CLEAR $(2)]"
	@rm -rf $(2) $($(1)_$(2)_OBJECT) obj/$(2)

endef

### BUILD #####################################################################
# $(1) = build type, ex. debug, release

define BUILD_template

$(1): $(foreach prog, $(PROGRAMS), $(addprefix bin/$(1),/$(prog)$(ELF_SUFFIX)))
	@echo [DONE ]

$(foreach prog, $(PROGRAMS), $(eval $(call VARIABLE_template,$(1),$(prog),$(addprefix $(1),_CFLAGS))))
$(foreach prog, $(PROGRAMS), $(eval $(call PROGRAM_template,$(1),$(prog),bin,$(addprefix $(1),_CFLAGS),$(addprefix $(1),_LDFLAGS))))

endef

$(foreach build, $(BUILD), $(eval $(call BUILD_template,$(build))))

clean:
	@echo "[CLEAN]"
	@rm -rf bin obj
