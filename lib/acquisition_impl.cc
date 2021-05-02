/* -*- c++ -*- */
/*
 * Copyright 2021 Lukasz Wiecaszek <lukasz.wiecaszek@gmail.com>.
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
#include "acquisition_impl.h"

namespace gr {
  namespace gnss {

    acquisition::sptr
    acquisition::make()
    {
      return gnuradio::get_initial_sptr
        (new acquisition_impl<gr_complex, gr_complex>());
    }

    template<typename ITYPE0, typename OTYPE0>
    acquisition_impl<ITYPE0, OTYPE0>::acquisition_impl()
      : gr::sync_block("acquisition",
                       gr::io_signature::make(1, 1, sizeof(ITYPE0)),
                       gr::io_signature::make(1, 1, sizeof(OTYPE0)))
    {
    }

    template<typename ITYPE0, typename OTYPE0>
    acquisition_impl<ITYPE0, OTYPE0>::~acquisition_impl()
    {
    }

    template<typename ITYPE0, typename OTYPE0>
    int
    acquisition_impl<ITYPE0, OTYPE0>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE0* iptr0 = (const ITYPE0*) input_items[0];
      OTYPE0* optr0 = (OTYPE0*) output_items[0];

      memcpy(optr0, iptr0, noutput_items * sizeof(OTYPE0));

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace gnss */
} /* namespace gr */

