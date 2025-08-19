# Copyright 2025 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Chi Zhang <chizhang@iis.ee.ethz.ch>

git submodule update --init --recursive
CXX=g++-11.2.0 CC=gcc-11.2.0 CMAKE=cmake-3.28.3 make -C src/dep/dram_rtl_sim/ -j dramsys
