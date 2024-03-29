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

#include "gnss_channel_impl.h"
#include "vector3d.h"
#include "ports.h"

namespace gr {
  namespace gnss {

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    gnss_channel::sptr
    gnss_channel::make(double sampling_freq,
      double dll_bw_coarse, double pll_bw_coarse, double dll_bw_fine, double pll_bw_fine)
    {
      return gnuradio::get_initial_sptr
        (new gnss_channel_impl<gr_complex, vector3d, gr_complexd>(sampling_freq, dll_bw_coarse, pll_bw_coarse, dll_bw_fine, pll_bw_fine));
    }

    template<typename ITYPE0, typename OTYPE0, typename OTYPE1>
    gnss_channel_impl<ITYPE0, OTYPE0, OTYPE1>::gnss_channel_impl(double sampling_freq,
      double dll_bw_coarse, double pll_bw_coarse, double dll_bw_fine, double pll_bw_fine)
      : gr::hier_block2("gnss_channel",
                        gr::io_signature::make(1, 1, sizeof(ITYPE0) * IVLEN0),
                        gr::io_signature::make2(2, 2, sizeof(OTYPE0) * OVLEN0, sizeof(OTYPE1) * OVLEN1)),
        d_acquisition_and_tracking{nullptr},
        d_ca_sybmols_to_nav_bits{nullptr},
        d_nav_message_decoder{nullptr},
        d_navigation_system{NAVIGATION_SYSTEM_UNDEFINED},
        d_id{-1}
    {
      d_acquisition_and_tracking = gr::gnss::acquisition_and_tracking::make(sampling_freq, dll_bw_coarse, pll_bw_coarse, dll_bw_fine, pll_bw_fine);
      d_ca_sybmols_to_nav_bits = gr::gnss::ca_sybmols_to_nav_bits::make();
      d_nav_message_decoder = gr::gnss::nav_message_decoder::make();

      message_port_register_hier_out(pmt::mp(PORT_CLOCK));
      message_port_register_hier_out(pmt::mp(PORT_EPHEMERIS));

      connect(self(),                     0, d_acquisition_and_tracking, 0);
      connect(d_acquisition_and_tracking, 0, d_ca_sybmols_to_nav_bits,   0);
      connect(d_ca_sybmols_to_nav_bits,   0, d_nav_message_decoder,      0);
      connect(d_nav_message_decoder,      0, self(),                     0);
      connect(d_acquisition_and_tracking, 0, self(),                     1);

      msg_connect(d_nav_message_decoder, pmt::mp(PORT_CLOCK), self(), pmt::mp(PORT_CLOCK));
      msg_connect(d_nav_message_decoder, pmt::mp(PORT_EPHEMERIS), self(), pmt::mp(PORT_EPHEMERIS));
    }

    template<typename ITYPE0, typename OTYPE0, typename OTYPE1>
    gnss_channel_impl<ITYPE0, OTYPE0, OTYPE1>::~gnss_channel_impl()
    {
    }

    template<typename ITYPE0, typename OTYPE0, typename OTYPE1>
    void
    gnss_channel_impl<ITYPE0, OTYPE0, OTYPE1>::set_acq_params(navigation_system_e system, int id)
    {
      d_navigation_system = system;
      d_id = id;

      d_acquisition_and_tracking->set_acq_params(system, id);
      d_nav_message_decoder->set_acq_params(system, id);
    }

    template<typename ITYPE0, typename OTYPE0, typename OTYPE1>
    void
    gnss_channel_impl<ITYPE0, OTYPE0, OTYPE1>::get_acq_params(navigation_system_e& system, int& id) const
    {
      system = d_navigation_system;
      id = d_id;
    }

  } /* namespace gnss */
} /* namespace gr */

