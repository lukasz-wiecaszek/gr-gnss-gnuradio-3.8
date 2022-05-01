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
#include "rft_impl.h"
#include "vector3d.h"
#include "reference_frames_transformations.h"

namespace gr {
  namespace gnss {

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    rft::sptr
    rft::make()
    {
      return gnuradio::get_initial_sptr
        (new rft_impl<vector3d, vector3d>());
    }

    template<typename ITYPE, typename OTYPE>
    rft_impl<ITYPE, OTYPE>::rft_impl()
      : gr::sync_block("rft",
                       gr::io_signature::make(1, 1, sizeof(ITYPE) * IVLEN),
                       gr::io_signature::make(1, 1, sizeof(OTYPE) * OVLEN)),
        d_rft_type{RFT_UNDEFINED}
    {
    }

    template<typename ITYPE, typename OTYPE>
    rft_impl<ITYPE, OTYPE>::~rft_impl()
    {
    }

    template<typename ITYPE, typename OTYPE>
    void
    rft_impl<ITYPE, OTYPE>::set_transformation(rft_type_e type)
    {
      gr::thread::scoped_lock lock(d_setlock);
      d_rft_type = type;
    }

    template<typename ITYPE, typename OTYPE>
    void
    rft_impl<ITYPE, OTYPE>::get_transformation(rft_type_e& type) const
    {
      type = d_rft_type;
    }

    template<typename ITYPE, typename OTYPE>
    int
    rft_impl<ITYPE, OTYPE>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE* iptr0 = (const ITYPE*) input_items[0];
      OTYPE* optr0 = (OTYPE*) output_items[0];

      gr::thread::scoped_lock lock(d_setlock);

      switch (d_rft_type)
      {
        case RFT_GCS_TO_ECEF:
          for (int n = 0; n < noutput_items; ++n)
            rftns::gcs_to_ecef(iptr0[n], optr0[n]);
          break;

        case RFT_ECEF_TO_GCS:
          for (int n = 0; n < noutput_items; ++n)
            rftns::ecef_to_gcs(iptr0[n], optr0[n]);
          break;

        default:
          /* do nothing */
          break;
      }

#if 0
      for (int n = 0; n < noutput_items; ++n) {
        printf("in: %s\n", to_string(iptr0[n]).c_str());
        printf("out: %s\n", to_string(optr0[n]).c_str());
      }
#endif

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace gnss */
} /* namespace gr */

