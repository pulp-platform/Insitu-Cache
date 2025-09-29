VERILATOR = verilator
BENDER ?= bender
VLOG_ARGS = -svinputport=compat -override_timescale 1ns/1ps -suppress 2583 -suppress 13314

library ?= work
# vsim_top_level ?= tb_seq_coalescer
# vsim_top_level ?= tb_insitu_cache
# vsim_top_level ?= tb_conventional_cache
# vsim_top_level ?= tb_insitu_cache_tcdm_wrapper
vsim_top_level ?= tb_flamingo_spatz_cache_ctrl

# Path to DRAMsyslib
dram_rtl_sim_folder ?= $(abspath ./src/dep/dram_rtl_sim)
dramsys_resouces_path ?= $(abspath ${dram_rtl_sim_folder}/dramsys_lib/DRAMSys/configs)
dramsys_lib_path ?= $(abspath ${dram_rtl_sim_folder}/dramsys_lib/DRAMSys/build/lib)
# QuestaSim arguments
questa_args    ?=
questa_args += +DRAMSYS_RES=$(dramsys_resouces_path)
questa_args += -sv_lib $(dramsys_lib_path)/libsystemc
questa_args += -sv_lib $(dramsys_lib_path)/libDRAMSys_Simulator
questa_args += -suppress vsim-3999


#Parameter define here! testbench only

## cache types setting ##
USE_CONVENTIONAL_CACHE 	?= 0
USE_BYPASS_CACHE 	  	?= 0
WRITE_THROUGH_MODE 		?= 0

## cache basic setting ##
CACHE_WORD_WIDTH 		?= 64
NUM_CACHE_ASSO 			?= 4
NUM_CACHE_LINE 			?= 2048

## cache bank setting ##
USE_DUAL_PORT_RF 		?= 0
USE_PSEUDO_DUAL_BANK 	?= 0
NUM_CACHE_BANK_FACTOR 	?= 2

## conventional cache setting ##
NUM_MSHR_ENTRY 			?= 256
NUM_MSHR_SUBARRAY 		?= 32
NUM_WRITE_BUFFER_ENTRY 	?= 128

## offchip link setting ##
OFFCHIP_LATENCY 		?= 0

## testbench setting ##
# 0->random read/write | 1->all read  | 2->all write
TEST_TYPE 				?= 1
# 0->random addresses  | 1->streaming | 2->sparse    | 3->traces
TEST_PATTERN 			?= 1
NUM_TEST 				?= 5000
ADDR_RANG 				?= 120
ACCESS_META_WIDTH 		?= 15
ACCESS_CORE_PORTS 		?= 4
COAL_EXT_FACTOR 		?= 1
TRAFFIC_LIMIT 			?= 0
# Log information
LOG_LIFE_CYCLE  		?= 0


bender_defs += 	--define CACHE_WORD_WIDTH=$(CACHE_WORD_WIDTH) \
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

# bender_defs +=  --define USE_DRAM_HBM
# bender_defs +=  --define USE_DRAM_DDR3
# bender_defs +=  --define USE_DRAM_DDR4
# bender_defs +=  --define USE_DRAM_LPDDR4
# bender_defs +=  --define SIM_STOP_AT=90000

ifeq ($(vsim_top_level), tb_flamingo_spatz_cache_ctrl)
    bender_defs += --define INSITU_CACHE_CORE_USE_MSHR_PADING
endif


all: vsim/compile.tcl

vsim: compile
	cd vsim && questa vsim -c $(library).$(vsim_top_level) -t 1ps -voptargs=+acc $(questa_args) -do start.tcl

gui: compile
	cd vsim && questa vsim $(library).$(vsim_top_level) -t 1ps -voptargs=+acc $(questa_args) -do start.tcl

compile: vsim/compile.tcl
	echo "exit" >> vsim/compile.tcl
	cd vsim && questa vsim -c -do compile.tcl

vsim/compile.tcl: Bender.yml Makefile $(shell find src -type f) $(shell find include -type f) 
	$(BENDER) script vsim -t insitu_test -t rtl --vlog-arg="$(VLOG_ARGS)" $(bender_defs) > $@

clean:
	cd vsim && rm -rf work/ vsim.wlf  transcript  modelsim.ini compile.tcl vsim* DRAMSys*


