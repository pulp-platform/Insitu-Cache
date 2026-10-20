// Copyright 2026 ETH Zurich and
// University of Bologna

// Solderpad Hardware License
// Version 0.51, see LICENSE for details.

// SPDX-License-Identifier: SHL-0.51

// Author: Diyou Shen <dishen@iis.ee.ethz.ch>, ETH Zurich

// Inverse of the tcdm_cache_interco address rotation: the interco moves the
// rot_bits_i routing bits found at ofst_i up to the MSBs; this moves them back.
//   rotated  : {rot_field, upper, lower}
//   original : {upper, rot_field, lower}, lower = addr[ofst_i-1:0]
module cachepool_addr_unrotate #(
  parameter int unsigned AddrWidth = 32,
  parameter int unsigned RotWidth  = 4,
  parameter int unsigned OfstWidth = $clog2(AddrWidth)
) (
  input  logic [AddrWidth-1:0] addr_i,
  input  logic [RotWidth-1:0]  rot_bits_i,
  input  logic [OfstWidth-1:0] ofst_i,
  output logic [AddrWidth-1:0] addr_o
);

  logic [AddrWidth-1:0] lower, rot_field, upper;

  assign lower     = addr_i & ((AddrWidth'(1) << ofst_i) - 1);
  assign rot_field = addr_i >> (AddrWidth - rot_bits_i);
  assign upper     = (addr_i >> ofst_i)
                   & ((AddrWidth'(1) << (AddrWidth - ofst_i - rot_bits_i)) - 1);

  assign addr_o = lower
                | (rot_field << ofst_i)
                | (upper     << (ofst_i + rot_bits_i));

endmodule : cachepool_addr_unrotate
