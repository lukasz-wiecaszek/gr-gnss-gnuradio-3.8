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

#ifndef INCLUDED_GNSS_CA_SYBMOLS_TO_NAV_BITS_IMPL_H
#define INCLUDED_GNSS_CA_SYBMOLS_TO_NAV_BITS_IMPL_H

#include <gnss/ca_sybmols_to_nav_bits.h>
#include <bitset>
#include <cmath>

#include "gnss_parameters.h"

#define IVLEN0 1
#define OVLEN0 1

#define STATES    \
  STATE(unlocked) \
  STATE(locked)   \

namespace gr {
  namespace gnss {

    template<typename ITYPE0, typename OTYPE0>
    class ca_sybmols_to_nav_bits_impl : public ca_sybmols_to_nav_bits
    {
    public:
      ca_sybmols_to_nav_bits_impl();
      ~ca_sybmols_to_nav_bits_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

    private:
      enum class state_e {
#define STATE(x) x,
        STATES
#undef STATE
      };

#define STATE(x) int state_handler_##x(int noutput_items,                      \
                                       gr_vector_int &ninput_items,            \
                                       gr_vector_const_void_star &input_items, \
                                       gr_vector_void_star &output_items);
        STATES
#undef STATE

      bool is_preamble_detected(const gr_complex* symbols)
      {
        int n = 0;

        for (std::size_t i = 0; i < d_preamble_sybmols.size(); ++i)
          if (d_preamble_sybmols[i])
            symbols[i].imag() > 0 ? n++ : n--;
          else
            symbols[i].imag() < 0 ? n++ : n--;

        if (std::abs(n) == d_preamble_sybmols.size())
          d_polarity = n > 0 ? +1 : -1;

        return std::abs(n) == d_preamble_sybmols.size();
      }

      int get_bit(const gr_complex* symbols)
      {
        int n = 0;

        for (int i = 0; i < GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT; ++i)
          symbols[i].imag() > 0 ? n++ : n--;

        if (std::abs(n) < d_thershold)
          return -1;

        return d_polarity * n > 0 ? 1 : 0;
      }

      state_e d_state;
      int d_polarity;
      int d_thershold;
      int d_subframe_bit;
      std::bitset<GPS_CA_TLM_PREAMBLE_BITS.size() * GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT> d_preamble_sybmols;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_CA_SYBMOLS_TO_NAV_BITS_IMPL_H */

