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

#include "ca_sybmols_to_nav_bits_impl.h"

namespace gr {
  namespace gnss {

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    ca_sybmols_to_nav_bits::sptr
    ca_sybmols_to_nav_bits::make()
    {
      return gnuradio::get_initial_sptr
        (new ca_sybmols_to_nav_bits_impl<gr_complex, uint8_t>());
    }

    template<typename ITYPE0, typename OTYPE0>
    ca_sybmols_to_nav_bits_impl<ITYPE0, OTYPE0>::ca_sybmols_to_nav_bits_impl()
      : gr::block("ca_sybmols_to_nav_bits",
                  gr::io_signature::make(1, 1, sizeof(ITYPE0)),
                  gr::io_signature::make(1, 1, sizeof(OTYPE0))),
        d_state{state_e::unlocked},
        d_polarity{0},
        d_thershold{GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT - 3 /* we accept 3 errors */},
        d_preamble_sybmols{}
    {
      set_tag_propagation_policy(TPP_DONT);

      set_relative_rate(1, GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT);
      set_output_multiple(GPS_CA_TLM_PREAMBLE_BITS.size());

      for (std::size_t i = 0; i < d_preamble_sybmols.size(); ++i)
        d_preamble_sybmols.set(i, GPS_CA_TLM_PREAMBLE_BITS[i / GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT]);
    }

    template<typename ITYPE0, typename OTYPE0>
    ca_sybmols_to_nav_bits_impl<ITYPE0, OTYPE0>::~ca_sybmols_to_nav_bits_impl()
    {
    }

    template<typename ITYPE0, typename OTYPE0>
    void
    ca_sybmols_to_nav_bits_impl<ITYPE0, OTYPE0>::forecast(int noutput_items, gr_vector_int &ninput_items_required)
    {
      int nrequired = noutput_items * GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT;

      for (auto&& element : ninput_items_required)
        element = nrequired;
    }

    template<typename ITYPE0, typename OTYPE0>
    int
    ca_sybmols_to_nav_bits_impl<ITYPE0, OTYPE0>::general_work(
      int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      int nproduced;

      switch (d_state)
      {
        case state_e::unlocked:
          nproduced = state_handler_unlocked(noutput_items, ninput_items, input_items, output_items);
          break;

        case state_e::locked:
          nproduced = state_handler_locked(noutput_items, ninput_items, input_items, output_items);
          break;

        default:
          consume_each(ninput_items[0]);
          nproduced = 0;
          break;
      }

      return nproduced;
    }

    /*===========================================================================*\
    * protected function definitions
    \*===========================================================================*/

    /*===========================================================================*\
    * private function definitions
    \*===========================================================================*/
    template<typename ITYPE0, typename OTYPE0>
    int
    ca_sybmols_to_nav_bits_impl<ITYPE0, OTYPE0>::state_handler_unlocked(
      int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      const ITYPE0* iptr0 = (const ITYPE0*) input_items[0];
      OTYPE0* optr0 = (OTYPE0*) output_items[0];
      int i = 0;

      while ((i < noutput_items) && !is_preamble_detected(iptr0 + i))
        i++;

      if (i < noutput_items)
        d_state = state_e::locked;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each(i);

      // Tell runtime system how many output items we produced.
      return 0;
    }

    template<typename ITYPE0, typename OTYPE0>
    int
    ca_sybmols_to_nav_bits_impl<ITYPE0, OTYPE0>::state_handler_locked(
      int noutput_items,
      gr_vector_int &ninput_items,
      gr_vector_const_void_star &input_items,
      gr_vector_void_star &output_items)
    {
      const ITYPE0* iptr0 = (const ITYPE0*) input_items[0];
      OTYPE0* optr0 = (OTYPE0*) output_items[0];
      int nproduced = 0;
      int nconsumed = 0;
      int bit_value;

      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + ninput_items[0], pmt::mp("rx_time"));

      while (nproduced < noutput_items) {
        if ((bit_value = get_bit(iptr0 + nconsumed)) == -1) {
          d_state = state_e::unlocked;
          break;
        }

        add_item_tag(0, tags[nconsumed]);
        optr0[nproduced] = bit_value;

        nproduced++;
        nconsumed += GPS_CA_SYMBOLS_PER_NAV_MESSAGE_BIT;
      }

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each(nconsumed);

      // Tell runtime system how many output items we produced.
      return nproduced;
    }

  } /* namespace gnss */
} /* namespace gr */

