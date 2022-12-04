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

#ifndef INCLUDED_GNSS_PSEUDORANGES_DECODER_IMPL_H
#define INCLUDED_GNSS_PSEUDORANGES_DECODER_IMPL_H

#include <cstdio>
#include <array>
#include <map>
#include <string>

#include <flatbuffer.hpp>
#include "vector3d.h"
#include "ephemeris.h"
#include "sv_clock_parameters.h"

#include <gnss/pseudoranges_decoder.h>

#define IVLEN 1
#define OVLEN 1
#define MAX_STREAMS 8

namespace gr {
  namespace gnss {

    template<typename ITYPE, typename OTYPE>
    class pseudoranges_decoder_impl : public pseudoranges_decoder
    {
    public:
      pseudoranges_decoder_impl(bool add_velocity_outputs);
      ~pseudoranges_decoder_impl();

      void set_acq_params(int port, navigation_system_e system, int id) override;
      void get_acq_params(int port, navigation_system_e& system, int& id) const override;

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

    private:
      double get_rx_time(const int N);
      double get_tx_time(const int n, double rx_time);

      bool d_add_velocity_outputs;
      std::array<std::tuple<navigation_system_e, int>, MAX_STREAMS> d_satelite_ids;
      std::array<lts::flatbuffer<vector3d>, MAX_STREAMS> d_flatbuffers;
      std::map<int, std::shared_ptr<sv_clock_parameters>> d_sv_clock_parameters;
      std::map<int, std::shared_ptr<ephemeris>> d_ephemerides;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_PSEUDORANGES_DECODER_IMPL_H */

