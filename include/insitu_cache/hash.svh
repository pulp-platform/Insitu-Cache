// Copyright 2025 ETH Zurich and 
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Chi Zhang <chizhang@iis.ee.ethz.ch>, ETH Zurich
// Date: 12.July.2024

`ifndef _HASH_H_
`define _HASH_H_

function automatic longint unsigned cache_addr_hashing(
    longint unsigned    addr,
    int                 field_start,
    int                 field_end,
    int                 length
);
    automatic longint unsigned addr_out = addr;
    automatic int dest_start = field_end - length;
    automatic int source_start = field_end;
    for (int i = 0; i<length; i++) begin
        addr_out[dest_start+i] = addr[dest_start+i] ^ addr[source_start+i];
    end
    return addr_out;
endfunction


`endif