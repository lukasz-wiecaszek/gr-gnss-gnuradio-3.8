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
#include "nav_message_sink_impl.h"
#include "gps_nav_message_subframe.h"

#include <string>
#include <sstream>

namespace gr {
  namespace gnss {

    nav_message_sink::sptr
    nav_message_sink::make(const char* filename)
    {
      return gnuradio::get_initial_sptr
        (new nav_message_sink_impl<uint8_t>(filename));
    }

    template<typename ITYPE0>
    nav_message_sink_impl<ITYPE0>::nav_message_sink_impl(const char* filename)
      : gr::sync_block("nav_message_sink",
                       gr::io_signature::make(1, 1, sizeof(ITYPE0) * IVLEN0),
                       gr::io_signature::make(0, 0, 0)),
        d_fp{NULL}
    {
      d_fp = fopen(filename, "w");
      if (d_fp == NULL) {
        std::stringstream s;
        s << "cannot open fifo file '" << filename << "': '" << strerror(errno) << "'" << std::endl;
        throw std::runtime_error(s.str());
      }
    }

    template<typename ITYPE0>
    nav_message_sink_impl<ITYPE0>::~nav_message_sink_impl()
    {
      if (d_fp != NULL)
        fclose(d_fp);
    }

    template<typename ITYPE0>
    int
    nav_message_sink_impl<ITYPE0>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE0* iptr0 = (const ITYPE0*) input_items[0];
      int nproduced = 0;
      gps_nav_message_subframe subframe;

      while (nproduced < noutput_items) {
        subframe.init(&iptr0);
        std::string str = subframe.to_string();

        fprintf(stdout, "%s\n", str.c_str());
        fprintf(d_fp, "%s\n", str.c_str());

        nproduced++;
      }

      // Tell runtime system how many output items we produced.
      return nproduced;
    }

  } /* namespace gnss */
} /* namespace gr */

