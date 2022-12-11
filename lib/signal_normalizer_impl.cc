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
#include "signal_normalizer_impl.h"

namespace gr {
  namespace gnss {

    template<typename ITYPE>
    typename signal_normalizer<ITYPE>::sptr
             signal_normalizer<ITYPE>::make(size_t vlen, size_t adc_resolution)
    {
      return gnuradio::get_initial_sptr
        (new signal_normalizer_impl<ITYPE, std::complex<double>>(vlen, adc_resolution));
    }

    template<typename ITYPE, typename OTYPE>
    signal_normalizer_impl<ITYPE, OTYPE>::signal_normalizer_impl(size_t vlen, size_t adc_resolution)
      : gr::block("signal_normalizer",
                  gr::io_signature::make(1, 1, sizeof(ITYPE) * vlen),
                  gr::io_signature::make(1, 1, sizeof(OTYPE) * vlen)),
        d_vlen{vlen},
        d_adc_resolution{adc_resolution}
    {
      if (d_adc_resolution > (sizeof(ITYPE) * 8))
        throw std::out_of_range("invalid space vehicle id");
    }

    template<typename ITYPE, typename OTYPE>
    signal_normalizer_impl<ITYPE, OTYPE>::~signal_normalizer_impl()
    {
    }

    template<typename ITYPE, typename OTYPE>
    void
    signal_normalizer_impl<ITYPE, OTYPE>::forecast(int noutput_items, gr_vector_int &ninput_items_required)
    {
      // 1 output item requires 2 input items
      int nrequired = 2 * noutput_items;

      for (auto&& element : ninput_items_required)
        element = nrequired;
    }

    template<>
    void
    signal_normalizer_impl<std::int8_t, std::complex<double>>::normalize(
      int noutput_items, const std::int8_t* iptr, std::complex<double>* optr)
    {
      for (int n = 0; n < noutput_items; ++n, iptr+=(2*d_vlen), optr+=d_vlen)
        for (size_t i = 0; i < d_vlen; ++i)
          optr[i] = std::complex<double>(
            static_cast<double>(iptr[2*i+0]) / (1U << (d_adc_resolution - 1)),
            static_cast<double>(iptr[2*i+1]) / (1U << (d_adc_resolution - 1)));
    }

    template<>
    void
    signal_normalizer_impl<std::uint8_t, std::complex<double>>::normalize(
      int noutput_items, const std::uint8_t* iptr, std::complex<double>* optr)
    {
      for (int n = 0; n < noutput_items; ++n, iptr+=(2*d_vlen), optr+=d_vlen)
        for (size_t i = 0; i < d_vlen; ++i)
          optr[i] = std::complex<double>(
            (static_cast<double>(iptr[2*i+0]) / (1U << (d_adc_resolution - 1))) - 1.0,
            (static_cast<double>(iptr[2*i+1]) / (1U << (d_adc_resolution - 1))) - 1.0);
    }

    template<>
    void
    signal_normalizer_impl<std::int16_t, std::complex<double>>::normalize(
      int noutput_items, const std::int16_t* iptr, std::complex<double>* optr)
    {
      for (int n = 0; n < noutput_items; ++n, iptr+=(2*d_vlen), optr+=d_vlen)
        for (size_t i = 0; i < d_vlen; ++i)
          optr[i] = std::complex<double>(
            static_cast<double>(iptr[2*i+0]) / (1U << (d_adc_resolution - 1)),
            static_cast<double>(iptr[2*i+1]) / (1U << (d_adc_resolution - 1)));
    }

    template<>
    void
    signal_normalizer_impl<std::uint16_t, std::complex<double>>::normalize(
      int noutput_items, const std::uint16_t* iptr, std::complex<double>* optr)
    {
      for (int n = 0; n < noutput_items; ++n, iptr+=(2*d_vlen), optr+=d_vlen)
        for (size_t i = 0; i < d_vlen; ++i)
          optr[i] = std::complex<double>(
            (static_cast<double>(iptr[2*i+0]) / (1U << (d_adc_resolution - 1))) - 1.0,
            (static_cast<double>(iptr[2*i+1]) / (1U << (d_adc_resolution - 1))) - 1.0);
    }

    template<typename ITYPE, typename OTYPE>
    int
    signal_normalizer_impl<ITYPE, OTYPE>::general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const ITYPE* iptr0 = (const ITYPE*) input_items[0];
      OTYPE* optr0 = (OTYPE*) output_items[0];

      normalize(noutput_items, iptr0, optr0);

      // Tell runtime system how many input items we consumed on
      // each input stream.
      this->consume_each(2 * noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    template class signal_normalizer<std::int8_t>;
    template class signal_normalizer<std::uint8_t>;
    template class signal_normalizer<std::int16_t>;
    template class signal_normalizer<std::uint16_t>;

  } /* namespace gnss */
} /* namespace gr */

