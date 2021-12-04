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
#include <time.h>
#include <chrono>

#include "nav_message_decoder_impl.h"

namespace gr {
  namespace gnss {

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    nav_message_decoder::sptr
    nav_message_decoder::make(const char* filename)
    {
      return gnuradio::get_initial_sptr
        (new nav_message_decoder_impl<uint8_t, vector>(filename));
    }

    template<typename ITYPE, typename OTYPE>
    nav_message_decoder_impl<ITYPE, OTYPE>::nav_message_decoder_impl(const char* filename)
      : gr::sync_block("nav_message_decoder",
                       gr::io_signature::make(1, 1, sizeof(ITYPE) * IVLEN),
                       gr::io_signature::make(1, 1, sizeof(OTYPE) * OVLEN)),
        d_fp{NULL},
        d_subframe_data{},
        d_subframe_data_cnt{0},
        d_subframe{},
        d_ephemeris{},
        d_current_ephemeris_idx{-1}
    {
      set_tag_propagation_policy(TPP_DONT);

      if (filename && filename[0] != '\0') {
        struct tm tm;
        char date_time_buf[128] = {};
        char date_time_filename_buf[256];

        auto current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        if (::gmtime_r(&current_time, &tm))
          ::strftime(date_time_buf, sizeof(date_time_buf), "%F_%H%M%S", &tm);

        snprintf(date_time_filename_buf, sizeof(date_time_filename_buf), "%s_%s", date_time_buf, filename);

        d_fp = fopen(date_time_filename_buf, "w");
      }
    }

    template<typename ITYPE, typename OTYPE>
    nav_message_decoder_impl<ITYPE, OTYPE>::~nav_message_decoder_impl()
    {
      if (d_fp != NULL)
        fclose(d_fp);
    }

    template<typename ITYPE, typename OTYPE>
    int
    nav_message_decoder_impl<ITYPE, OTYPE>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE* iptr0 = (const ITYPE*) input_items[0];
      OTYPE* optr0 = (OTYPE*) output_items[0];

      std::vector<tag_t> subframe_bit_tags;
      get_tags_in_range(subframe_bit_tags, 0, nitems_read(0), nitems_read(0) + noutput_items, pmt::mp("subframe_bit"));

      std::vector<tag_t> rx_time_tags;
      get_tags_in_range(rx_time_tags, 0, nitems_read(0), nitems_read(0) + noutput_items, pmt::mp("rx_time"));

      for (int i = 0; i < noutput_items; ++i) {
        int n = pmt::to_long(subframe_bit_tags[i].value);

        if (d_current_ephemeris_idx < 0) {
          optr0[i] = vector{0.0, 0.0, 0.0};
        }
        else {
          const ephemeris e = d_ephemeris[d_current_ephemeris_idx];
          vector position;
          double t = (d_subframe.tow_count_message() * 6) + n * 0.02;

          e.get_vectors(t, &position, NULL, NULL);
          double pseudorange = pmt::to_double(rx_time_tags[i].value) - t;

          add_item_tag(0, nitems_written(0) + i, pmt::mp("pseudorange"), pmt::mp(pseudorange), alias_pmt());
          optr0[i] = position;
        }

        if (d_subframe_data_cnt == n) {
          d_subframe_data[n] = iptr0[i];
          d_subframe_data_cnt++;
        }
        else
          d_subframe_data_cnt = 0;

        if (d_subframe_data_cnt == GPS_NAV_MESSAGE_BITS_PER_SUBFRAME) {
          const ITYPE *p = &d_subframe_data[0];
          d_subframe.init(&p);
          process_subframe(d_subframe);

          if (d_fp)
            fprintf(d_fp, "%s\n", d_subframe.to_string().c_str());

          d_subframe_data_cnt = 0;
        }
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    /*===========================================================================*\
    * protected function definitions
    \*===========================================================================*/

    /*===========================================================================*\
    * private function definitions
    \*===========================================================================*/
    template<typename ITYPE, typename OTYPE>
    void
    nav_message_decoder_impl<ITYPE, OTYPE>::init_ephemeris(ephemeris& e, const gps_nav_message_subframe_2& subframe_2)
    {
      e.IODE[0]               = subframe_2.IODE();
      e.correction_terms.C_RS = subframe_2.C_RS()       * GPS_SCALE_FACTOR_C_R;
      e.delta_n               = subframe_2.DELTA_N()    * GPS_SCALE_FACTOR_DELTA_N * M_PI;
      e.M_0                   = subframe_2.M_0()        * GPS_SCALE_FACTOR_M_0 * M_PI;
      e.correction_terms.C_UC = subframe_2.C_UC()       * GPS_SCALE_FACTOR_C_U;
      e.e                     = subframe_2.e()          * GPS_SCALE_FACTOR_E;
      e.correction_terms.C_US = subframe_2.C_US()       * GPS_SCALE_FACTOR_C_U;
      e.sqrt_a                = subframe_2.SQRT_A()     * GPS_SCALE_FACTOR_SQRT_A;
      e.t_oe                  = subframe_2.t_oe()       * GPS_SCALE_FACTOR_T_OE;
    }

    template<typename ITYPE, typename OTYPE>
    void
    nav_message_decoder_impl<ITYPE, OTYPE>::init_ephemeris(ephemeris& e, const gps_nav_message_subframe_3& subframe_3)
    {
      e.IODE[1]               = subframe_3.IODE();
      e.correction_terms.C_IC = subframe_3.C_IC()       * GPS_SCALE_FACTOR_C_I;
      e.OMEGA_0               = subframe_3.OMEGA_0()    * GPS_SCALE_FACTOR_OMEGA_0 * M_PI;
      e.correction_terms.C_IS = subframe_3.C_IS()       * GPS_SCALE_FACTOR_C_I;
      e.i_0                   = subframe_3.i_0()        * GPS_SCALE_FACTOR_I_0 * M_PI;
      e.correction_terms.C_RC = subframe_3.C_RC()       * GPS_SCALE_FACTOR_C_R;
      e.omega                 = subframe_3.omega()      * GPS_SCALE_FACTOR_OMEGA * M_PI;
      e.dOMEGA_dt             = subframe_3.dOMEGA_dt()  * GPS_SCALE_FACTOR_D_OMEGA_DT * M_PI;
      e.dI_dt                 = subframe_3.dI_dt()      * GPS_SCALE_FACTOR_IDOT * M_PI;
    }

    template<typename ITYPE, typename OTYPE>
    void
    nav_message_decoder_impl<ITYPE, OTYPE>::process_subframe(const gps_nav_message_subframe& subframe)
    {
      if (d_current_ephemeris_idx < 0) {
        ephemeris& e = d_ephemeris[0];

        if (subframe.subframe_id() == 2) {
          gps_nav_message_subframe_2 subframe_2 = subframe;
          init_ephemeris(e, subframe_2);
        }

        if (subframe.subframe_id() == 3) {
          gps_nav_message_subframe_3 subframe_3 = subframe;
          init_ephemeris(e, subframe_3);
        }

        if ((e.IODE[0] != -1) && (e.IODE[0] == e.IODE[1])) {
          d_current_ephemeris_idx = 0;
          if (d_fp)
            fprintf(d_fp, "%s\n", e.to_string().c_str());
        }
      }
      else {
        int complementary_ephemeris_idx = (d_current_ephemeris_idx == 0) ? 1 : 0;
        ephemeris e1 = d_ephemeris[d_current_ephemeris_idx];
        ephemeris e2 = d_ephemeris[complementary_ephemeris_idx];

        if (subframe.subframe_id() == 2) {
          gps_nav_message_subframe_2 subframe_2 = subframe;
          if (e1.IODE[0] != subframe_2.IODE()) {
            init_ephemeris(e2, subframe_2);
          }
        }

        if (subframe.subframe_id() == 3) {
          gps_nav_message_subframe_3 subframe_3 = subframe;
          if (e1.IODE[1] != subframe_3.IODE()) {
            init_ephemeris(e2, subframe_3);
          }
        }

        if ((e2.IODE[0] != -1) && (e2.IODE[0] == e2.IODE[1])) {
          e1.IODE[0] = e1.IODE[1] = -1;
          d_current_ephemeris_idx = complementary_ephemeris_idx;
          if (d_fp)
            fprintf(d_fp, "%s\n", e2.to_string().c_str());
        }
      }
    }

  } /* namespace gnss */
} /* namespace gr */

