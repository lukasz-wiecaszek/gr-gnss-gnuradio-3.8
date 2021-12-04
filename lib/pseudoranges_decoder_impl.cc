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

#include "vector3d.h"
#include "pseudoranges_decoder_impl.h"

#define DECIMATION_FACTOR 1

namespace gr {
  namespace gnss {

    pseudoranges_decoder::sptr
    pseudoranges_decoder::make()
    {
      return gnuradio::get_initial_sptr
        (new pseudoranges_decoder_impl<vector, vector>());
    }

    template<typename ITYPE, typename OTYPE>
    pseudoranges_decoder_impl<ITYPE, OTYPE>::pseudoranges_decoder_impl()
      : gr::block("pseudoranges_decoder",
                  gr::io_signature::make(1, 8, sizeof(ITYPE) * IVLEN),
                  gr::io_signature::make(1, 1, sizeof(OTYPE) * OVLEN))
    {
      set_relative_rate(1, DECIMATION_FACTOR);
    }

    template<typename ITYPE, typename OTYPE>
    pseudoranges_decoder_impl<ITYPE, OTYPE>::~pseudoranges_decoder_impl()
    {
    }

    template<typename ITYPE, typename OTYPE>
    void
    pseudoranges_decoder_impl<ITYPE, OTYPE>::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      // 1 output item requires DECIMATION_FACTOR items at input
      int nrequired = noutput_items * DECIMATION_FACTOR;

      for (auto&& element : ninput_items_required)
        element = nrequired;
    }

    template<typename ITYPE, typename OTYPE>
    int
    pseudoranges_decoder_impl<ITYPE, OTYPE>::general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const ITYPE* iptr0 = (const ITYPE*) input_items[0];
      OTYPE* optr0 = (OTYPE*) output_items[0];

      std::size_t ninputs = input_items.size();

      for (std::size_t i = 0; i < ninputs; ++i) {
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each(noutput_items * DECIMATION_FACTOR);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace gnss */
} /* namespace gr */

