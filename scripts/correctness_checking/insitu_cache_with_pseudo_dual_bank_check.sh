#!/bin/bash
set -e
set -x

###############################
# Basic Settings              #
###############################

results_folder="results/correctness_checking/insitu_cache_pesudo_dual_bank"
timeout_second=1800

###############################
# Sweep Parameters Settings   #
###############################

list_cache_lines=(1024 2048 4096)
list_set_asso=(2 4 8)
list_bank_factor=(2 4 8)
list_word_width=(64)
list_write_through=(0)
list_test_type=(0 1 2)

###############################
# Setup Result Folder         #
###############################

mkdir -p ${results_folder}

###############################
# Design Parameter Sensitive  #
###############################

for cacheline in ${list_cache_lines[@]}; do
    for set_associaty in ${list_set_asso[@]}; do
        if [ "$cacheline" -gt "$set_associaty" ]; then
            for word_width in ${list_word_width[@]}; do
                for bank_factor in ${list_bank_factor[@]}; do
                    bank_depth=$(echo "$cacheline / ($set_associaty * $bank_factor)" | bc)
                    if [ "$bank_depth" -ge 2 ]; then
                        for test_type in ${list_test_type[@]}; do
                            for wt in ${list_write_through[@]}; do
                                log_file="${results_folder}/Log_${cacheline}cacheline_${set_associaty}setAssociaty_${word_width}wordWidth_TestType${test_type}_WriteThrough${wt}.txt"
                                echo "[Checking Insitu Cache with Pseudo-Dual-Port Banks] Bank factor = ${bank_factor}  IsWriteThrough = ${wt}   Cache lines = ${cacheline}   SetAsso = ${set_associaty}   WordWidth = ${word_width}   TestType = ${test_type}"
                                make clean
                                USE_PSEUDO_DUAL_BANK=1 \
                                NUM_CACHE_BANK_FACTOR=${bank_factor} \
                                NUM_CACHE_LINE=${cacheline} \
                                NUM_CACHE_ASSO=${set_associaty} \
                                CACHE_WORD_WIDTH=${word_width} \
                                TEST_TYPE=${test_type} \
                                WRITE_THROUGH_MODE=${wt} \
                                vsim_top_level=tb_insitu_cache \
                                timeout ${timeout_second} make vsim > ${log_file}
                                set +e
                                if (grep -q "Unexpected RData" ${log_file}); then exit 1; fi
                                set -e
                                grep -q  "TEST PASSED" ${log_file}
                            done
                        done
                    fi
                done
            done
        fi
    done
done
