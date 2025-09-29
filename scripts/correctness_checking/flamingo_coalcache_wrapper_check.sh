#!/bin/bash
set -e
set -x

###############################
# Basic Settings              #
###############################

results_folder="results/correctness_checking/flamingo_coalcache_wrapper"
timeout_second=1800

###############################
# Sweep Parameters Settings   #
###############################

list_core_ports=(4 5 8 10)
list_coal_factor=(1)
list_meta_width=(16)
list_bank_factor=(2)
list_set_asso=(4)
list_cache_lines=(4096)

###############################
# Setup Result Folder         #
###############################

mkdir -p ${results_folder}

###############################
# Design Parameter Sensitive  #
###############################

for cache_line in ${list_cache_lines[@]}; do
	for set_asso in ${list_set_asso[@]}; do
		for bank_factor in ${list_bank_factor[@]}; do
			for meta_width in ${list_meta_width[@]}; do
				for coal_factor in ${list_coal_factor[@]}; do
					for core_ports in ${list_core_ports[@]}; do
						log_file="${results_folder}/Log_${cache_line}cacheline_${set_asso}setAssociaty_${bank_factor}BankFactor_${meta_width}MetaWidth_${coal_factor}CoalFactor_${core_ports}CorePorts.txt"
                        echo "[Checking Flamingo CoalCache Wrapper]  ${cache_line} cacheline | ${set_asso} SetAssociaty | ${bank_factor} BankFactor | ${meta_width} MetaWidth | ${coal_factor} CoalFactor | ${core_ports} CorePorts"
                        make clean
                        NUM_CACHE_LINE=${cache_line} \
                        NUM_CACHE_ASSO=${set_asso} \
                        NUM_CACHE_BANK_FACTOR=${bank_factor} \
                        ACCESS_META_WIDTH=${meta_width} \
                        COAL_EXT_FACTOR=${coal_factor} \
                        ACCESS_CORE_PORTS=${core_ports} \
                        vsim_top_level=tb_flamingo_spatz_cache_ctrl \
                        timeout ${timeout_second} make vsim > ${log_file}
                        grep -q  "TEST PASSED" ${log_file}
					done
				done
			done
		done
	done
done