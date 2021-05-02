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
#include <cmath>

#include <gnuradio/io_signature.h>
#include "ca_code_generator_impl.h"
#include "gnss_parameters.h"
#include "ca_code.h"

namespace gr {
  namespace gnss {

    using gps_ca_code = ca_code<int, GPS_CA_CODE_LENGTH>;

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
        d_n_samples{static_cast<decltype(d_n_samples)>(ceil(sampling_freq / GPS_CA_CODE_RATE))},
        d_code_sampled(d_n_samples),
        d_n{0}
    {
      const std::shared_ptr<gps_ca_code> code = gps_ca_code::get(svid);
      if (code == nullptr)
        throw std::out_of_range("invalid space vehicle id");

      for (int i = 0; i < d_n_samples; ++i)
        d_code_sampled[i] = (*code)[((i + .5f) * GPS_CA_CODE_LENGTH) / d_n_samples] ? +1 : -1;
    }

    template<>
    ca_code_generator_impl<gr_complex>::ca_code_generator_impl(unsigned svid, double sampling_freq)
      : gr::sync_block("ca_code_generator",
                       gr::io_signature::make(0, 0, 0),
                       gr::io_signature::make(1, 1, sizeof(gr_complex))),
        d_n_samples{static_cast<decltype(d_n_samples)>(ceil(sampling_freq / GPS_CA_CODE_RATE))},
        d_code_sampled(d_n_samples),
        d_n{0}
    {
      const std::shared_ptr<gps_ca_code> code = gps_ca_code::get(svid);
      if (code == nullptr)
        throw std::out_of_range("invalid space vehicle id");

      for (int i = 0; i < d_n_samples; ++i)
        d_code_sampled[i] = (*code)[(i * GPS_CA_CODE_LENGTH) / d_n_samples] ?
          gr_complex{+1.0f, 0.0f} : gr_complex{-1.0f, 0.0f};
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
        optr0[i] = d_code_sampled[d_n % d_n_samples];

      d_n %= d_n_samples;

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    template class ca_code_generator<std::int8_t>;
    template class ca_code_generator<std::int16_t>;
    template class ca_code_generator<std::int32_t>;
    template class ca_code_generator<float>;
    template class ca_code_generator<gr_complex>;

  } /* namespace gnss */
} /* namespace gr */

