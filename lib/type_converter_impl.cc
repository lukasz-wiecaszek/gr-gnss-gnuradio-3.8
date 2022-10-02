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
#include "type_converter_impl.h"

namespace gr {
  namespace gnss {

    template<typename ITYPE, typename OTYPE>
    typename type_converter<ITYPE, OTYPE>::sptr
             type_converter<ITYPE, OTYPE>::make(size_t vlen)
    {
      return gnuradio::get_initial_sptr
        (new type_converter_impl<ITYPE, OTYPE>(vlen));
    }

    template<typename ITYPE, typename OTYPE>
    type_converter_impl<ITYPE, OTYPE>::type_converter_impl(size_t vlen)
      : gr::sync_block("type_converter",
                       gr::io_signature::make(1, 1, sizeof(ITYPE) * vlen),
                       gr::io_signature::make(1, 1, sizeof(OTYPE) * vlen)),
        d_vlen{vlen}
    {
    }

    template<typename ITYPE, typename OTYPE>
    type_converter_impl<ITYPE, OTYPE>::~type_converter_impl()
    {
    }

    template<typename ITYPE, typename OTYPE>
    int
    type_converter_impl<ITYPE, OTYPE>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE* iptr0 = (const ITYPE*) input_items[0];
      OTYPE* optr0 = (OTYPE*) output_items[0];

      for (size_t i = 0; i < d_vlen; ++i)
        optr0[i] = static_cast<OTYPE>(iptr0[i]);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    template class type_converter<float, double>;
    template class type_converter<double, float>;
    template class type_converter<std::complex<float>, std::complex<double>>;
    template class type_converter<std::complex<double>, std::complex<float>>;

  } /* namespace gnss */
} /* namespace gr */

