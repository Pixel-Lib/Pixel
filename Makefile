################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=-I/opt/homebrew/Cellar/googletest/1.14.0/include -Isrc/pxl/yaml-cpp/include -Lsrc/pxl/yaml-cpp/lib -lyaml-cpp
EXTRA_CXXFLAGS=-I/opt/homebrew/Cellar/googletest/1.14.0/include -Isrc/pxl/yaml-cpp/include -Lsrc/pxl/yaml-cpp/lib -lyaml-cpp

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
# TODO: CHANGE THIS!
LIBNAME:=Pixel
VERSION:=0.1.0
# EX> baCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main $(SRCDIR)/pxl/yaml-cpp/util/parse $(SRCDIR)/pxl/yaml-cpp/util/read $(SRCDIR)/pxl/yaml-cpp/util/sandbox, $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/pxl/yaml-cpp/build/CMakeFiles/3.29.1/CompilerIdCXX/CMakeCXXCompilerId, $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))
# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported
TEMPLATE_FILES=$(INCDIR)/**/*.h $(INCDIR)/**/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
