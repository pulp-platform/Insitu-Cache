git submodule update --init --recursive
CXX=g++-11.2.0 CC=gcc-11.2.0 CMAKE=cmake-3.28.3 make -C src/dep/dram_rtl_sim/ -j dramsys
