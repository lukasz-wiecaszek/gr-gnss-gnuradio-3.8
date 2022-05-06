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

#ifndef INCLUDED_GNSS_GEOJSON_FILE_SINK_IMPL_H
#define INCLUDED_GNSS_GEOJSON_FILE_SINK_IMPL_H

#include <stdio.h>

#include <gnss/geojson_file_sink.h>

#define IVLEN 1

namespace gr {
  namespace gnss {

    template<typename ITYPE>
    class geojson_file_sink_impl : public geojson_file_sink
    {
    public:
      geojson_file_sink_impl(const char *filename);
      ~geojson_file_sink_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);

    private:
      FILE *d_fp;
      std::size_t d_line_cnt;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_GEOJSON_FILE_SINK_IMPL_H */

