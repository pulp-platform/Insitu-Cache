# Copyright 2025 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Chi Zhang <chizhang@iis.ee.ethz.ch>

# Basic Settings
LIBRARY                 ?= work
TB_TOP                  ?= tb_insitu_cache

# Bender
BENDER                  ?= bender
VLOG_ARGS               ?= -svinputport=compat -override_timescale 1ns/1ps -suppress 2583 -suppress 13314
BENDER_TARGETS          ?= -t insitu_test -t rtl
BENDER_VLOG_ARGS        ?= --vlog-arg="$(VLOG_ARGS)"

# Path to DRAMsyslib
DRAM_RTL_SIM_FOLDER     ?= $(abspath ./src/dep/dram_rtl_sim)
DRAMSYS_RESOUCES_PATH   ?= $(abspath ${DRAM_RTL_SIM_FOLDER}/dramsys_lib/DRAMSys/configs)
DRAMSYS_LIB_PATH        ?= $(abspath ${DRAM_RTL_SIM_FOLDER}/dramsys_lib/DRAMSys/build/lib)

# QuestaSim
QUESTAVSIM              ?= questa-2022.3 vsim
QUESTA_ARGS             ?=
QUESTA_ARGS             += +DRAMSYS_RES=$(DRAMSYS_RESOUCES_PATH)
QUESTA_ARGS             += -sv_lib $(DRAMSYS_LIB_PATH)/libsystemc
QUESTA_ARGS             += -sv_lib $(DRAMSYS_LIB_PATH)/libDRAMSys_Simulator
QUESTA_ARGS             += -suppress vsim-3999


# Design parameters are defined here! testbench only
## cache types setting ##
USE_CONVENTIONAL_CACHE  ?= 0
USE_BYPASS_CACHE        ?= 0
WRITE_THROUGH_MODE      ?= 0

## cache basic setting ##
CACHE_WORD_WIDTH        ?= 64
NUM_CACHE_ASSO          ?= 4
NUM_CACHE_LINE          ?= 2048

## cache bank setting ##
USE_DUAL_PORT_RF        ?= 0
USE_PSEUDO_DUAL_BANK    ?= 0
NUM_CACHE_BANK_FACTOR   ?= 2

## conventional cache setting ##
NUM_MSHR_ENTRY          ?= 256
NUM_MSHR_SUBARRAY       ?= 32
NUM_WRITE_BUFFER_ENTRY  ?= 128

## offchip link setting ##
OFFCHIP_LATENCY         ?= 0

## testbench setting ##
# 0->random read/write | 1->all read  | 2->all write
TEST_TYPE               ?= 1
# 0->random addresses  | 1->streaming | 2->sparse    | 3->traces
TEST_PATTERN            ?= 1
NUM_TEST                ?= 5000
ADDR_RANG               ?= 120
ACCESS_META_WIDTH       ?= 15
ACCESS_CORE_PORTS       ?= 4
COAL_EXT_FACTOR         ?= 1
TRAFFIC_LIMIT           ?= 0
# Log information
LOG_LIFE_CYCLE          ?= 0


BENDER_DEFS             += --define CACHE_WORD_WIDTH=$(CACHE_WORD_WIDTH) \
                           --define NUM_CACHE_LINE=$(NUM_CACHE_LINE) \
                           --define NUM_CACHE_ASSO=$(NUM_CACHE_ASSO) \
                           --define USE_DUAL_PORT_RF=$(USE_DUAL_PORT_RF) \
                           --define USE_PSEUDO_DUAL_BANK=$(USE_PSEUDO_DUAL_BANK) \
                           --define NUM_CACHE_BANK_FACTOR=$(NUM_CACHE_BANK_FACTOR) \
                           --define NUM_MSHR_ENTRY=$(NUM_MSHR_ENTRY) \
                           --define NUM_MSHR_SUBARRAY=$(NUM_MSHR_SUBARRAY) \
                           --define NUM_WRITE_BUFFER_ENTRY=$(NUM_WRITE_BUFFER_ENTRY) \
                           --define NUM_TEST=$(NUM_TEST) \
                           --define TEST_TYPE=$(TEST_TYPE) \
                           --define TEST_PATTERN=$(TEST_PATTERN) \
                           --define ADDR_RANG=$(ADDR_RANG) \
                           --define TRAFFIC_LIMIT=$(TRAFFIC_LIMIT) \
                           --define WRITE_THROUGH_MODE=$(WRITE_THROUGH_MODE) \
                           --define USE_CONVENTIONAL_CACHE=$(USE_CONVENTIONAL_CACHE) \
                           --define USE_BYPASS_CACHE=$(USE_BYPASS_CACHE) \
                           --define OFFCHIP_LATENCY=$(OFFCHIP_LATENCY) \
                           --define ACCESS_META_WIDTH=$(ACCESS_META_WIDTH) \
                           --define ACCESS_CORE_PORTS=$(ACCESS_CORE_PORTS) \
                           --define COAL_EXT_FACTOR=$(COAL_EXT_FACTOR) \
                           --define LOG_LIFE_CYCLE=$(LOG_LIFE_CYCLE)

vsim: compile
	cd vsim && $(QUESTAVSIM) -c $(LIBRARY).$(TB_TOP) -t 1ps $(QUESTA_ARGS) -do start.tcl

gui: compile
	cd vsim && $(QUESTAVSIM) $(LIBRARY).$(TB_TOP) -t 1ps $(QUESTA_ARGS) -do start.tcl

compile: vsim/compile.tcl
	echo "exit" >> vsim/compile.tcl
	cd vsim && $(QUESTAVSIM) -c -do compile.tcl

vsim/compile.tcl: Bender.yml Makefile $(shell find src -type f) $(shell find include -type f) 
	$(BENDER) script vsim $(BENDER_TARGETS) $(BENDER_VLOG_ARGS) $(BENDER_DEFS) > $@

clean:
	cd vsim && rm -rf work/ vsim.wlf  transcript  modelsim.ini compile.tcl vsim* DRAMSys*
