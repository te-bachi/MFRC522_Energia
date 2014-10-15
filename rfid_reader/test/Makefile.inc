

CC                  = ${COMPILER_PREFIX}-gcc
LD                  = ${COMPILER_PREFIX}-ld
OBJCOPY             = ${COMPILER_PREFIX}-objcopy
OBJDUMP             = ${COMPILER_PREFIX}-objdump
BUILD               = debug release
ARCH                = -m$(CODE_GEN) -mcpu=$(CPU) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)

LIB_GCC_PATH        = ${shell ${CC} ${CFLAGS} -print-libgcc-file-name}
LIBC_PATH           = ${shell ${CC} ${CFLAGS} -print-file-name=libc.a}
LIBM_PATH           = ${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

global_CFLAGS      += -D$(PART) -D$(ROM) -I$(STELLARISWARE)
global_LDFLAGS     += ${STELLARISWARE}/driverlib/gcc-cm4f/libdriver-cm4f.a \
                      $(LIBM_PATH) $(LIBC_PATH) $(LIB_GCC_PATH)

OBJCOPY_FLAGS       = -Obinary
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

$(foreach source,$($(2)_SOURCE),$(eval $(call OBJECT_template,$(1),$(2),src,source,$(4))))

$(3)/$(1)/$(2).$(BINARY_SUFFIX): $($(1)_$(2)_OBJECT)
	@echo "[LD   ] $(3)/$(1)/$(2).$(BINARY_SUFFIX)"
	@mkdir -p $(3)/$(1)
	$(LD) -T $($(2)_LDSCRIPT) -o $(3)/$(1)/$(2).$(BINARY_SUFFIX) $($(1)_$(2)_OBJECT) $(global_LDFLAGS) $($(5)) $($(2)_LDFLAGS)
	@echo "[BIN  ] $(3)/$(1)/$(2).bin"
	@$(OBJCOPY) $(OBJCOPY_FLAGS) $(3)/$(1)/$(2).$(BINARY_SUFFIX) $(3)/$(1)/$(2).bin
	@echo "[LST  ] $(3)/$(1)/$(2).lst"
	@$(OBJDUMP) $(OBJDUMP_FLAGS) $(3)/$(1)/$(2).$(BINARY_SUFFIX) > $(3)/$(1)/$(2).lst

$(1)_$(2)_clean:
	@echo "[CLEAR $(2)]"
	@rm -rf $(2) $($(1)_$(2)_OBJECT) obj/$(2)

endef

### BUILD #####################################################################
# $(1) = build type, ex. debug, release

define BUILD_template

$(1): $(foreach prog, $(PROGRAMS), $(addprefix bin/$(1),/$(prog).$(BINARY_SUFFIX)))
	@echo [DONE ]

$(foreach prog, $(PROGRAMS), $(eval $(call VARIABLE_template,$(1),$(prog),$(addprefix $(1),_CFLAGS))))
$(foreach prog, $(PROGRAMS), $(eval $(call PROGRAM_template,$(1),$(prog),bin,$(addprefix $(1),_CFLAGS),$(addprefix $(1),_LDFLAGS))))

endef

$(foreach build, $(BUILD), $(eval $(call BUILD_template,$(build))))

clean:
	@echo "[CLEAN]"
	@rm -rf bin obj
