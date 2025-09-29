#!/bin/bash
set -e
set -x

###############################
# Basic Settings              #
###############################

results_folder="results/correctness_checking/insitu_cache"
timeout_second=1800

###############################
# Sweep Parameters Settings   #
###############################

list_cache_lines=(4 16 64 256 1024 4096 8192)
list_set_asso=(2 4 8 16 32)
list_word_width=(32 64)
list_write_through=(0 1)
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
                for test_type in ${list_test_type[@]}; do
                    for wt in ${list_write_through[@]}; do
                        log_file="${results_folder}/Log_${cacheline}cacheline_${set_associaty}setAssociaty_${word_width}wordWidth_TestType${test_type}_WriteThrough${wt}.txt"
                        echo "[Checking Insitu Cache]  IsWriteThrough = ${wt}   Cache lines = ${cacheline}   SetAsso = ${set_associaty}   WordWidth = ${word_width}   TestType = ${test_type}"
                        make clean
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
            done
        fi
    done
done
