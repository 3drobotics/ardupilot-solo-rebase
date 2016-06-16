TOOLCHAIN = NATIVE

include $(MK_DIR)/find_tools.mk

#
# Tool options
#
DEFINES         =   -DF_CPU=$(F_CPU)
DEFINES        +=   -DSKETCH=\"$(SKETCH)\" -DSKETCHNAME="\"$(SKETCH)\"" -DSKETCHBOOK="\"$(SKETCHBOOK)\"" -DAPM_BUILD_DIRECTORY=APM_BUILD_$(SKETCH)
DEFINES        +=   $(EXTRAFLAGS)
DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) -DCONFIG_HAL_BOARD_SUBTYPE=$(HAL_BOARD_SUBTYPE)
WARNFLAGS       =   -Wall -Wextra -Wformat -Wshadow -Wpointer-arith -Wcast-align \
                    -Wlogical-op -Wwrite-strings -Wformat=2 -Wno-unused-parameter
WARNFLAGSCXX    = \
        -Wno-missing-field-initializers \
        -Wno-reorder \
        -Werror=format-security \
        -Werror=array-bounds \
        -Werror=unused-but-set-variable \
        -Werror=uninitialized \
        -Werror=init-self \
        -Wfatal-errors \
        -Wundef

DEPFLAGS        =   -MD -MP -MT $@

CXXOPTS         =   -ffunction-sections -fdata-sections -fno-exceptions -fsigned-char
COPTS           =   -ffunction-sections -fdata-sections -fsigned-char

ASOPTS          =   -x assembler-with-cpp 

# features: TODO detect dependecy and make them optional
HAVE_LTTNG=

ifeq ($(HAVE_LTTNG),1)
DEFINES        += -DPERF_LTTNG=1
LIBS           += -llttng-ust -ldl
endif

# disable as this breaks distcc
#ifneq ($(SYSTYPE),Darwin)
#LISTOPTS        =   -adhlns=$(@:.o=.lst)
#endif

CPUFLAGS     = -D_GNU_SOURCE
CPULDFLAGS   = -g
OPTFLAGS     ?= -O3 -g

CXXFLAGS        =   -g $(CPUFLAGS) $(DEFINES) $(OPTFLAGS)
CXXFLAGS       +=   -std=gnu++11 $(WARNFLAGS) $(WARNFLAGSCXX) $(DEPFLAGS) $(CXXOPTS)
CFLAGS          =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(OPTFLAGS)
CFLAGS         +=   $(WARNFLAGS) $(DEPFLAGS) $(COPTS)
ASFLAGS         =   -g $(CPUFLAGS) $(DEFINES) -Wa,$(LISTOPTS) $(DEPFLAGS)
ASFLAGS        +=   $(ASOPTS)
LDFLAGS         =   -g $(CPUFLAGS) $(OPTFLAGS) $(WARNFLAGS)

ifneq ($(SYSTYPE),Darwin)
LDFLAGS        +=   -Wl,--gc-sections -Wl,-Map -Wl,$(SKETCHMAP)
endif

LIBS ?= -lm -pthread -L/Users/georgehines/Code/PhysicalSimulation/lib/ -lVehicleBuild
ifneq ($(findstring CYGWIN, $(SYSTYPE)),)
LIBS += -lwinmm
endif

ifeq ($(VERBOSE),)
v = @
else
v =
endif

# Library object files
LIBOBJS			:=	$(SKETCHLIBOBJS)


################################################################################
# Built products
#

# The ELF file
SKETCHELF		=	$(BUILDROOT)/$(SKETCH).elf
BUILDELF                =       $(notdir $(SKETCHELF))

# HEX file
SKETCHHEX		=	$(BUILDROOT)/$(SKETCH).hex

# EEP file
SKETCHEEP		=	$(BUILDROOT)/$(SKETCH).eep

# Map file
SKETCHMAP		=	$(BUILDROOT)/$(SKETCH).map

# All of the objects that may be built
ALLOBJS			=	$(SKETCHOBJS) $(LIBOBJS)

# All of the dependency files that may be generated
ALLDEPS			=	$(ALLOBJS:%.o=%.d)

################################################################################
# Targets
#

all: $(SKETCHELF)

print-%:
	echo "$*=$($*)"

################################################################################
# Rules
#

# fetch dependency info from a previous build if any of it exists
-include $(ALLDEPS)

ifeq ($(HAL_BOARD_SUBTYPE),HAL_BOARD_SUBTYPE_LINUX_QFLIGHT)
include $(MK_DIR)/board_qflight.mk
else
# Link the final object
$(SKETCHELF): $(SKETCHOBJS) $(LIBOBJS)
	@echo "Building $(SKETCHELF)"
	$(RULEHDR)
	$(v)$(LD) $(LDFLAGS) -o $@ $(SKETCHOBJS) $(LIBOBJS) $(LIBS)
	$(v)cp $(SKETCHELF) .
	@echo "Firmware is in $(BUILDELF)"
endif

SKETCH_INCLUDES	=	$(SKETCHLIBINCLUDES)
SLIB_INCLUDES	=	-I$(dir $<)/utility $(SKETCHLIBINCLUDES) -I/Users/georgehines/Code/PhysicalSimulation/include/ -I/Users/georgehines/Code/PhysicalSimulation/MATLAB_dist/include/

include $(MK_DIR)/build_rules.mk
