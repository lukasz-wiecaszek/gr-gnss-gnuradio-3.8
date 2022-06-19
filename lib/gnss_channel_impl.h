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

#ifndef INCLUDED_GNSS_GNSS_CHANNEL_IMPL_H
#define INCLUDED_GNSS_GNSS_CHANNEL_IMPL_H

#include <gnss/gnss_channel.h>

#include <gnss/acquisition_and_tracking.h>
#include <gnss/ca_sybmols_to_nav_bits.h>
#include <gnss/nav_message_decoder.h>

#define IVLEN0 1
#define OVLEN0 1
#define OVLEN1 1

namespace gr {
  namespace gnss {

    template<typename ITYPE0, typename OTYPE0, typename OTYPE1>
    class gnss_channel_impl : public gnss_channel
    {
    public:
      gnss_channel_impl(double sampling_freq,
                        double dll_bw_coarse,
                        double pll_bw_coarse,
                        double dll_bw_fine,
                        double pll_bw_fine);
      ~gnss_channel_impl();

      void set_acq_params(navigation_system_e system, int id) override;
      void get_acq_params(navigation_system_e& system, int& id) const override;

    private:
      gr::gnss::acquisition_and_tracking::sptr d_acquisition_and_tracking;
      gr::gnss::ca_sybmols_to_nav_bits::sptr d_ca_sybmols_to_nav_bits;
      gr::gnss::nav_message_decoder::sptr d_nav_message_decoder;
      navigation_system_e d_navigation_system;
      int d_id;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_GNSS_CHANNEL_IMPL_H */

