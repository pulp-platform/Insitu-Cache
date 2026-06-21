# Makefile for the sram_forwarding_buffer unit testbench.
#
# Usage (from this directory or with `make -C <this_dir>`):
#   make            # default: compile + run, prints pass/fail
#   make run        # same as default
#   make compile    # compile only (incremental)
#   make gui        # open Questa GUI with the TB loaded
#   make wave       # run headless and dump vsim.wlf for offline waveform
#   make rerun      # re-run without recompile (after a tweak that doesn't change RTL/TB)
#   make clean      # delete all build artifacts
#   make help       # this list
#
# Override examples:
#   make BENDER=/path/to/bender VSIM="questa-2024.1 vsim"
#   make TB_TOP=tb_other_top
#

# ---------- Paths ----------
# Self-locate so `make -C` and `make` from anywhere both work.
MAKEFILE_DIR := $(abspath $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST)))))
PKG_DIR      := $(abspath $(MAKEFILE_DIR)/../..)
BUILD_DIR    := $(MAKEFILE_DIR)/build

# ---------- Tools ----------
BENDER       ?= bender
VSIM         ?= questa-2023.4-zr vsim
VLIB         ?= questa-2023.4-zr vlib
VSIM_OPTS    ?= -t 1ps -voptargs=+acc
VLOG_OPTS    ?=

# ---------- TB ----------
TB_TOP       ?= tb_sram_forwarding_buffer

# ---------- Inputs we depend on for "is recompile needed?" ----------
# Local test files
TEST_SRCS    := $(MAKEFILE_DIR)/sram_model.sv \
                $(MAKEFILE_DIR)/tb_sram_forwarding_buffer.sv
# DUT source(s) -- recompile if the buffer RTL changes
DUT_SRCS     := $(PKG_DIR)/src/utilities/sram_forwarding_buffer.sv \
                $(PKG_DIR)/src/utilities/sram_forwarding_buffer_multi.sv

# ---------- Generated ----------
COMPILE_TCL  := $(BUILD_DIR)/compile.tcl
COMPILE_DONE := $(BUILD_DIR)/.compile_done
RUN_LOG      := $(BUILD_DIR)/run.log
WAVE_WLF     := $(BUILD_DIR)/wave.wlf

# ---------- Phony targets ----------
.PHONY: all run rerun compile gui wave clean help

all: run

help:
	@echo "Targets:"
	@echo "  make            -- compile + run, print pass/fail summary"
	@echo "  make run        -- alias for default"
	@echo "  make compile    -- compile only (incremental)"
	@echo "  make rerun      -- run again without recompiling"
	@echo "  make gui        -- open Questa GUI"
	@echo "  make wave       -- run headless, dump $(WAVE_WLF) for inspection"
	@echo "  make clean      -- delete $(BUILD_DIR)"
	@echo ""
	@echo "Common overrides:"
	@echo "  make BENDER=/path/to/bender"
	@echo "  make VSIM=\"questa-2024.1 vsim\""

# ---------- Build dir ----------
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# ---------- Generate compile script via Bender ----------
# Re-generate whenever the package's Bender.yml changes.
$(COMPILE_TCL): $(PKG_DIR)/Bender.yml | $(BUILD_DIR)
	@echo "[bender] generating compile script -> $@"
	@cd $(PKG_DIR) && $(BENDER) script vsim -t rtl -t test_fwd_buf > $@

# ---------- Compile (incremental) ----------
# vsim -c with a sourced script runs the vlog commands.  We touch a
# sentinel file on success so subsequent `make` calls skip recompile
# unless RTL or TB sources have changed.
$(COMPILE_DONE): $(COMPILE_TCL) $(TEST_SRCS) $(DUT_SRCS) | $(BUILD_DIR)
	@echo "[vsim ] compiling..."
	@cd $(BUILD_DIR) && \
	    ($(VLIB) work >/dev/null 2>&1 || true) && \
	    $(VSIM) -c -do "source $(COMPILE_TCL); quit -f" \
	        > $(BUILD_DIR)/compile.log 2>&1
	@if grep -qE "^# Errors: [1-9]" $(BUILD_DIR)/compile.log; then \
	    echo "[vsim ] COMPILE FAILED -- see $(BUILD_DIR)/compile.log"; \
	    grep -aE "Error|error" $(BUILD_DIR)/compile.log | head -20; \
	    exit 1; \
	fi
	@echo "[vsim ] compile OK"
	@touch $@

compile: $(COMPILE_DONE)

# ---------- Run ----------
# Default headless run.  The TB calls $finish(0) on pass and $finish(1) on
# fail; we also grep for the "[PASS]" / "[FAIL]" markers and surface them.
run: compile
	@echo "[vsim ] running $(TB_TOP)..."
	@cd $(BUILD_DIR) && \
	    $(VSIM) -c -do "vsim -c $(VSIM_OPTS) $(TB_TOP); run -a; quit -f" \
	        > $(RUN_LOG) 2>&1 || true
	@grep -aE "TEST:|PASS:|FAIL:|Summary|Watchdog" $(RUN_LOG) || true
	@if grep -aq "^# \[PASS\]" $(RUN_LOG); then \
	    echo "[vsim ] OVERALL: PASS"; \
	    exit 0; \
	else \
	    echo "[vsim ] OVERALL: FAIL  (see $(RUN_LOG))"; \
	    exit 1; \
	fi

# rerun: skip the compile dependency check
rerun:
	@echo "[vsim ] re-running $(TB_TOP) without recompile..."
	@cd $(BUILD_DIR) && \
	    $(VSIM) -c -do "vsim -c $(VSIM_OPTS) $(TB_TOP); run -a; quit -f" \
	        > $(RUN_LOG) 2>&1 || true
	@grep -aE "TEST:|PASS:|FAIL:|Summary|Watchdog" $(RUN_LOG) || true
	@if grep -aq "^# \[PASS\]" $(RUN_LOG); then \
	    echo "[vsim ] OVERALL: PASS"; \
	else \
	    echo "[vsim ] OVERALL: FAIL  (see $(RUN_LOG))"; \
	    exit 1; \
	fi

# ---------- GUI for interactive debug ----------
gui: compile
	@cd $(BUILD_DIR) && \
	    $(VSIM) -do "vsim $(VSIM_OPTS) $(TB_TOP); add wave -r /tb_sram_forwarding_buffer/*; run -a"

# ---------- Wave dump (no GUI) ----------
wave: compile
	@echo "[vsim ] running $(TB_TOP) and dumping $(WAVE_WLF)..."
	@cd $(BUILD_DIR) && \
	    $(VSIM) -c -do "vsim -c $(VSIM_OPTS) $(TB_TOP) -wlf $(WAVE_WLF); log -r /*; run -a; quit -f" \
	        > $(RUN_LOG) 2>&1
	@grep -aE "TEST:|PASS:|FAIL:|Summary" $(RUN_LOG) || true
	@echo "[vsim ] wave: $(WAVE_WLF)"

clean:
	@rm -rf $(BUILD_DIR)
