/* -*- c++ -*- */
/*
 * Copyright 2022 Lukasz Wiecaszek <lukasz.wiecaszek@gmail.com>.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "skip_zeros_impl.h"
#include "vector3d.h"

namespace gr {
  namespace gnss {

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    skip_zeros::sptr
    skip_zeros::make()
    {
      return gnuradio::get_initial_sptr
        (new skip_zeros_impl<vector3d, vector3d>());
    }

    template<typename ITYPE, typename OTYPE>
    skip_zeros_impl<ITYPE, OTYPE>::skip_zeros_impl()
      : gr::sync_block("skip_zeros",
                       gr::io_signature::make(1, 1, sizeof(ITYPE) * IVLEN),
                       gr::io_signature::make(1, 1, sizeof(OTYPE) * OVLEN))
    {
    }

    template<typename ITYPE, typename OTYPE>
    skip_zeros_impl<ITYPE, OTYPE>::~skip_zeros_impl()
    {
    }

    template<typename ITYPE, typename OTYPE>
    int
    skip_zeros_impl<ITYPE, OTYPE>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE *iptr0 = (const ITYPE *) input_items[0];
      OTYPE* optr0 = (OTYPE*) output_items[0];
      int nproduced = 0;

      for (int n = 0; n < noutput_items; ++n) {
        if (is_null(iptr0[n])) {
          consume(0, 1);
          continue;
        }

        optr0[nproduced++] = iptr0[n];
      }

      // Tell runtime system how many output items we produced.
      return nproduced;
    }

  } /* namespace gnss */
} /* namespace gr */

