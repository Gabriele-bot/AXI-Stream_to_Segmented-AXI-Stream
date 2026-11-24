# ----------------------------------------------------------------------------
# AXI-Stream <-> Segmented AXI-Stream : compile + regression
#
#   make                 -> compile and run everything on both simulators
#   make vivado          -> everything with Vivado xsim
#   make questa          -> everything with Questa
#   make vivado-sim      / make questa-sim      (same, explicit)
#   make vivado-compile  / make questa-compile  (compile only)
#   make <sim>_<tb>_N<nseg>_S<segb>             (single run, e.g.
#                                                 make questa_tb_axis_2_seg_axis_N8_S16)
#   make clean
#
# Override tool locations if they are not in PATH, e.g.
#   make VIVADO_BIN=/data/eda/Xilinx/Vivado/2024.2/bin QUESTA_BIN=/data/eda/questasim/linux_x86_64
# ----------------------------------------------------------------------------

SHELL := /bin/bash

# ---- Sources ---------------------------------------------------------------
AXIS_RTL_DIR := verilog-axis/rtl
HDL_DIR      := hdl
SIM_DIR      := sim

V_SRCS  := $(sort $(wildcard $(AXIS_RTL_DIR)/*.v))
SV_SRCS := $(sort $(wildcard $(HDL_DIR)/*.sv))
TB_SRCS := $(sort $(wildcard $(SIM_DIR)/*.sv))
TBS     := $(basename $(notdir $(TB_SRCS)))

# ---- Regression configurations (NSEG:SEGB) ---------------------------------
CONFIGS := 4:8 8:8 16:8  2:16 4:16 8:16

# Testbenches that are additionally run with SIMPLE=1 (simple TX module)
SIMPLE_TBS := tb_axis_2_seg_axis

# ---- Tools -----------------------------------------------------------------
VIVADO_BIN ?=
QUESTA_BIN ?=
XVLOG := $(if $(VIVADO_BIN),$(VIVADO_BIN)/)xvlog
XELAB := $(if $(VIVADO_BIN),$(VIVADO_BIN)/)xelab
XSIM  := $(if $(VIVADO_BIN),$(VIVADO_BIN)/)xsim
VLIB  := $(if $(QUESTA_BIN),$(QUESTA_BIN)/)vlib
VLOG  := $(if $(QUESTA_BIN),$(QUESTA_BIN)/)vlog
VSIM  := $(if $(QUESTA_BIN),$(QUESTA_BIN)/)vsim

BUILD := build
VIV_D := $(BUILD)/vivado
QST_D := $(BUILD)/questa
LOG_D := $(BUILD)/logs

# ---- Helpers ---------------------------------------------------------------
# $(call cfg_nseg,4:8) -> 4 ; $(call cfg_segb,4:8) -> 8
cfg_nseg = $(word 1,$(subst :, ,$1))
cfg_segb = $(word 2,$(subst :, ,$1))

# Names of every single run: <sim>_<tb>_N<nseg>_S<segb>[_SIMPLE1]
RUNS_VIV := $(foreach t,$(TBS),$(foreach c,$(CONFIGS),vivado_$(t)_N$(call cfg_nseg,$c)_S$(call cfg_segb,$c))) \
            $(foreach t,$(SIMPLE_TBS),$(foreach c,$(CONFIGS),vivado_$(t)_N$(call cfg_nseg,$c)_S$(call cfg_segb,$c)_SIMPLE1))
RUNS_QST := $(foreach t,$(TBS),$(foreach c,$(CONFIGS),questa_$(t)_N$(call cfg_nseg,$c)_S$(call cfg_segb,$c))) \
            $(foreach t,$(SIMPLE_TBS),$(foreach c,$(CONFIGS),questa_$(t)_N$(call cfg_nseg,$c)_S$(call cfg_segb,$c)_SIMPLE1))

# A run passes when the TB prints RESULT: PASS and no simulator error shows up
# (the second check catches $error calls such as the watchdog / drain timeout).
define check_log
@if grep -q "RESULT: PASS" $(1) && ! grep -qiE "^(\*\* )?(error|fatal)" $(1); then \
    echo "[PASS] $(2)"; \
else \
    echo "[FAIL] $(2)  (see $(1))"; exit 1; \
fi
endef

.PHONY: all vivado questa vivado-compile questa-compile vivado-sim questa-sim clean help
all: vivado questa

help:
	@sed -n '2,15p' Makefile

vivado: vivado-sim
questa: questa-sim
vivado-sim:  $(RUNS_VIV)
questa-sim:  $(RUNS_QST)
vivado-compile: $(VIV_D)/.compiled
questa-compile: $(QST_D)/.compiled

$(LOG_D) $(VIV_D) $(QST_D):
	@mkdir -p $@

# ---- Vivado (xsim) ---------------------------------------------------------
# xvlog -sv handles both .v and .sv; everything goes into library xil_defaultlib
$(VIV_D)/.compiled: $(V_SRCS) $(SV_SRCS) $(TB_SRCS) | $(VIV_D) $(LOG_D)
	cd $(VIV_D) && $(XVLOG) -sv -work xil_defaultlib $(addprefix ../../,$(V_SRCS) $(SV_SRCS) $(TB_SRCS)) -log ../logs/vivado_compile.log
	@grep -qE "^ERROR" $(LOG_D)/vivado_compile.log && { echo "Vivado compile failed"; exit 1; } || true
	@touch $@

# $1 = tb, $2 = nseg, $3 = segb, $4 = SIMPLE value (empty = TB default)
define vivado_run
.PHONY: vivado_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4))
vivado_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)): $(VIV_D)/.compiled
	@echo "[vivado] $(1) NSEG=$(2) SEGB=$(3) $(if $(4),SIMPLE=$(4))"
	@cd $(VIV_D) && $(XELAB) xil_defaultlib.$(1) -s $(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)) \
	    -timescale 1ns/1ps -debug off \
	    -generic_top NSEG=$(2) -generic_top SEGB=$(3) $(if $(4),-generic_top SIMPLE=$(4)) \
	    -log ../logs/vivado_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4))_elab.log > /dev/null
	@cd $(VIV_D) && $(XSIM) $(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)) -runall \
	    -log ../logs/vivado_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)).log > /dev/null
	$$(call check_log,$(LOG_D)/vivado_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)).log,vivado $(1) NSEG=$(2) SEGB=$(3) $(if $(4),SIMPLE=$(4)))
endef

# ---- Questa ----------------------------------------------------------------
$(QST_D)/.compiled: $(V_SRCS) $(SV_SRCS) $(TB_SRCS) | $(QST_D) $(LOG_D)
	cd $(QST_D) && rm -rf work && $(VLIB) work && \
	    $(VLOG) -sv -work work \
	    $(addprefix ../../,$(V_SRCS) $(SV_SRCS) $(TB_SRCS)) \
	    -l ../logs/questa_compile.log
	@touch $@

# $1 = tb, $2 = nseg, $3 = segb, $4 = SIMPLE value (empty = TB default)
define questa_run
.PHONY: questa_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4))
questa_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)): $(QST_D)/.compiled
	@echo "[questa] $(1) NSEG=$(2) SEGB=$(3) $(if $(4),SIMPLE=$(4))"
	@cd $(QST_D) && $(VSIM) -c -voptargs=+acc work.$(1) \
	    -GNSEG=$(2) -GSEGB=$(3) -GNP=100 $(if $(4),-GSIMPLE=$(4)) \
	    -l ../logs/questa_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)).log \
	    -do "run -all; quit -f" > /dev/null
	$$(call check_log,$(LOG_D)/questa_$(1)_N$(2)_S$(3)$(if $(4),_SIMPLE$(4)).log,questa $(1) NSEG=$(2) SEGB=$(3) $(if $(4),SIMPLE=$(4)))
endef

$(foreach t,$(TBS),$(foreach c,$(CONFIGS),\
  $(eval $(call vivado_run,$(t),$(call cfg_nseg,$c),$(call cfg_segb,$c))) \
  $(eval $(call questa_run,$(t),$(call cfg_nseg,$c),$(call cfg_segb,$c)))))

$(foreach t,$(SIMPLE_TBS),$(foreach c,$(CONFIGS),\
  $(eval $(call vivado_run,$(t),$(call cfg_nseg,$c),$(call cfg_segb,$c),1)) \
  $(eval $(call questa_run,$(t),$(call cfg_nseg,$c),$(call cfg_segb,$c),1))))

clean:
	rm -rf $(BUILD)
