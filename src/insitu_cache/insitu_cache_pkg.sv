// Copyright 2023 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Matheus Cavalcante, ETH Zurich

package insitu_cache_pkg;

  `include "insitu_cache/hash.svh"

  //////////////////
  //  Parameters  //
  //////////////////


  //////////////////////
  // Type Definitions //
  //////////////////////
  typedef enum logic[1:0] { INVALID = '0, VALID, READ_PEND, WRITE_PEND } cache_status_t;


endpackage : insitu_cache_pkg
