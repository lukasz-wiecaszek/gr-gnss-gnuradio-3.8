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

#include <memory>
//#include <cmath>

#include <gnuradio/io_signature.h>
#include "ca_code_generator_impl.h"
#include "gnss_parameters.h"
#include "ca_code.h"

namespace gr {
  namespace gnss {

    template<typename T>
    typename ca_code_generator<T>::sptr
    ca_code_generator<T>::make(unsigned svid, double sampling_freq)
    {
      return gnuradio::get_initial_sptr
        (new ca_code_generator_impl<T>(svid, sampling_freq));
    }


    template<typename T>
    ca_code_generator_impl<T>::ca_code_generator_impl(unsigned svid, double sampling_freq)
      : gr::sync_block("ca_code_generator",
                       gr::io_signature::make(0, 0, 0),
                       gr::io_signature::make(1, 1, sizeof(T))),
        d_sampling_freq{sampling_freq},
        d_code{ca_code<T, GPS_CA_CODE_LENGTH>::get(svid)},
        d_n{0}
    {
      if (d_code == nullptr)
        throw std::runtime_error("invalid space vehicle id");
    }

    template<typename T>
    ca_code_generator_impl<T>::~ca_code_generator_impl()
    {
    }

    template<typename T>
    int
    ca_code_generator_impl<T>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      T* optr0 = (T*) output_items[0];

      for (int i = 0; i < noutput_items; ++i, ++d_n)
        optr0[i] = (*d_code)[d_n];

      d_n %= GPS_CA_CODE_LENGTH;

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    template class ca_code_generator<std::int8_t>;
    template class ca_code_generator<std::int16_t>;
    template class ca_code_generator<std::int32_t>;

  } /* namespace gnss */
} /* namespace gr */

