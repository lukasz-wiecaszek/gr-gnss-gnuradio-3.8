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

#ifndef INCLUDED_GNSS_NAV_MESSAGE_DECODER_IMPL_H
#define INCLUDED_GNSS_NAV_MESSAGE_DECODER_IMPL_H

#include <gnss/nav_message_decoder.h>
#include <stdio.h>
#include <string>
#include <array>

#include "gps_nav_message_subframe.h"
#include "ephemeris.h"

#define IVLEN 1
#define OVLEN 1

namespace gr {
  namespace gnss {

    template<typename ITYPE, typename OTYPE>
    class nav_message_decoder_impl : public nav_message_decoder
    {
    public:
      nav_message_decoder_impl(const char* filename);
      ~nav_message_decoder_impl();

      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );

    private:
      void init_ephemeris(ephemeris& e, const gps_nav_message_subframe_2& subframe_2);
      void init_ephemeris(ephemeris& e, const gps_nav_message_subframe_3& subframe_3);
      void process_subframe(const gps_nav_message_subframe& subframe);

      FILE* d_fp;
      std::array<uint8_t, GPS_NAV_MESSAGE_BITS_PER_SUBFRAME> d_subframe_data;
      int d_subframe_data_cnt;
      gps_nav_message_subframe d_subframe;
      ephemeris d_ephemeris[2];
      int d_current_ephemeris_idx;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NAV_MESSAGE_DECODER_IMPL_H */

