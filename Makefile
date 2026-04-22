# =====================================================================
# Makefile — Vivado batch flow for arty_top (Arty A7-100T, I2S2 loopback)
#
#   make            build bitstream (default)
#   make project    create Vivado project only
#   make synth      build bitstream (alias: bitstream)
#   make program    load bitstream onto the board over JTAG
#   make gui        open the project in the Vivado GUI
#   make lint       RTL elaboration-only syntax check
#   make clean      remove build/ and stray Vivado artifacts
#   make help       show this list
# =====================================================================

VIVADO    ?= vivado
BUILD_DIR ?= build
PROJ_NAME ?= arty_top
PART      ?= xc7a100tcsg324-1
JOBS      ?= 8

PROJ_DIR := $(BUILD_DIR)/$(PROJ_NAME)
XPR      := $(PROJ_DIR)/$(PROJ_NAME).xpr
BIT      := $(PROJ_DIR)/$(PROJ_NAME).runs/impl_1/$(PROJ_NAME).bit

RTL_SRCS := $(wildcard rtl/*.sv) $(wildcard rtl/oversample/*.sv)
XDC_SRCS := $(wildcard constraints/*.xdc)
MEM_SRCS := $(wildcard lut_out/*.mem)

VIVADO_BATCH := $(VIVADO) -mode batch -nojournal -nolog -notrace

.DEFAULT_GOAL := all

.PHONY: all project synth bitstream program gui lint clean help

all: $(BIT)

project: $(XPR)

synth bitstream: $(BIT)

$(XPR): $(RTL_SRCS) $(XDC_SRCS) $(MEM_SRCS) tcl/project.tcl
	$(VIVADO_BATCH) -source tcl/project.tcl

$(BIT): $(XPR) $(RTL_SRCS) $(XDC_SRCS) tcl/synth.tcl
	$(VIVADO_BATCH) -source tcl/synth.tcl

program: $(BIT)
	$(VIVADO_BATCH) -source tcl/program.tcl

gui: $(XPR)
	$(VIVADO) $(XPR) &

lint: tcl/lint.tcl $(RTL_SRCS)
	$(VIVADO_BATCH) -source tcl/lint.tcl -tclargs $(PART) $(RTL_SRCS)

clean:
	rm -rf $(BUILD_DIR) .Xil vivado*.jou vivado*.log vivado*.str *.backup.jou *.backup.log

help:
	@echo 'Targets:'
	@echo '  all        build bitstream (default)  -> $(BIT)'
	@echo '  project    create Vivado project      -> $(XPR)'
	@echo '  synth      synth + impl + bitstream (alias: bitstream)'
	@echo '  program    program the Arty board over JTAG'
	@echo '  gui        open the project in the Vivado GUI'
	@echo '  lint       RTL elaboration-only syntax check'
	@echo '  clean      remove build artifacts'
	@echo '  help       show this message'
	@echo
	@echo 'Variables (override on command line, e.g. make JOBS=4):'
	@echo '  VIVADO=$(VIVADO)'
	@echo '  BUILD_DIR=$(BUILD_DIR)'
	@echo '  PROJ_NAME=$(PROJ_NAME)'
	@echo '  PART=$(PART)'
	@echo '  JOBS=$(JOBS)'
