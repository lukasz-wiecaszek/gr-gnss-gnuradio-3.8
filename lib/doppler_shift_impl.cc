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

#include "doppler_shift_impl.h"
#include "gnss_parameters.h"
#include "vector3d.h"
#include "tags.h"

namespace gr {
  namespace gnss {

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    doppler_shift::sptr
    doppler_shift::make(double x, double y, double z)
    {
      return gnuradio::get_initial_sptr
        (new doppler_shift_impl<vector3d, double>(x, y, z));
    }

    template<typename ITYPE, typename OTYPE>
    doppler_shift_impl<ITYPE, OTYPE>::doppler_shift_impl(double x, double y, double z)
      : gr::sync_block("doppler_shift",
                       gr::io_signature::make(1, 2, sizeof(ITYPE) * IVLEN),
                       gr::io_signature::make(1, 1, sizeof(OTYPE) * OVLEN)),
        d_receiver_position{{x, y, z}},
        d_t{0.0},
        d_r{{0.0, 0.0, 0.0}}
    {
      printf("doppler_shift: receiver_position: %s\n", to_string(d_receiver_position).c_str());
    }

    template<typename ITYPE, typename OTYPE>
    doppler_shift_impl<ITYPE, OTYPE>::~doppler_shift_impl()
    {
    }

    template<typename ITYPE, typename OTYPE>
    int
    doppler_shift_impl<ITYPE, OTYPE>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE *iptr0 = (const ITYPE *) input_items[0];
      const ITYPE *iptr1 = nullptr;
      OTYPE* optr0 = (OTYPE*) output_items[0];
      int nproduced = 0;
      bool skip;

      if (input_items.size() > 1)
        iptr1 = (const ITYPE *) input_items[1];

      std::vector<tag_t> tags;
      get_tags_in_range(tags, 0, nitems_read(0), nitems_read(0) + noutput_items, pmt::mp(TAG_RX_TIME));

      for (int n = 0; n < noutput_items; ++n) {
        skip = false;
        if (is_null(iptr0[n])) {
          consume(0, 1);
          skip = true; // we do not have valid satellite position yet
        }
        if (iptr1 && is_null(iptr1[n])) {
          consume(1, 1);
          skip = true; // we do not have valid receiver position yet
        }
        if (skip)
          continue;

        double t = pmt::to_double(tags[n].value);
        ITYPE r;
        if (iptr1)
          r = iptr0[n] - iptr1[n];
        else
          r = iptr0[n] - d_receiver_position;

        if ((d_t != 0.0) && (!is_null(d_r))) {
          double dt = t - d_t;
          ITYPE dr = r - d_r;
          ITYPE v = (dr / dt);
          double v_radial = (lts::transpose(v) * r) / abs(r);
          double gamma = v_radial / C;
          double df = GPS_L1_FREQ_HZ * (std::sqrt((1 - gamma) / (1 + gamma)) - 1.0);

          //printf("n: %d, %s, %.15e, %.15e\n", n, to_string(v).c_str(), v_radial, df);

          optr0[nproduced++] = df;
        }
        else
          consume_each(1);

        d_t = t;
        d_r = r;
      }

      // Tell runtime system how many output items we produced.
      return nproduced;
    }

  } /* namespace gnss */
} /* namespace gr */

