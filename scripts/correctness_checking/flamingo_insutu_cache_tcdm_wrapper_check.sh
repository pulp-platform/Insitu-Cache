#!/bin/bash
set -e
set -x

###############################
# Basic Settings              #
###############################

results_folder="results/correctness_checking/flamingo_insutu_cache_tcdm_wrapper_check"
timeout_second=1800

###############################
# Setup Result Folder         #
###############################

mkdir -p ${results_folder}

log_file="${results_folder}/log.txt"
make clean
vsim_top_level=tb_insitu_cache_tcdm_wrapper \
timeout ${timeout_second} make vsim > ${log_file}

set +e
if (grep -q "Unexpected RData" ${log_file}); then exit 1; fi
set -e
grep -q  "TEST PASSED" ${log_file}
