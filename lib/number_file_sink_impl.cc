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
#include "number_file_sink_impl.h"

namespace gr {
  namespace gnss {

    static void on_exit_handler(int code, void *arg)
    {
      FILE *fp = (FILE*)arg;

      if (fp) {
        fprintf(stderr, "number_file_sink_impl::on_exit_handler: closing file descriptor\n");
        fclose(fp);
      }
    }

    template<typename ITYPE>
    typename number_file_sink<ITYPE>::sptr
             number_file_sink<ITYPE>::make(const char *filename, const char *delimiter)
    {
      return gnuradio::get_initial_sptr
        (new number_file_sink_impl<ITYPE>(filename, delimiter));
    }

    template<typename ITYPE>
    number_file_sink_impl<ITYPE>::number_file_sink_impl(const char *filename, const char *delimiter)
      : gr::sync_block("number_file_sink",
              gr::io_signature::make(1, -1, sizeof(ITYPE) * IVLEN),
              gr::io_signature::make(0, 0, 0)),
      d_fp{NULL},
      d_delimiter{delimiter}
    {
      if (filename)
        d_fp = fopen(filename, "w");
    }

    template<typename ITYPE>
    number_file_sink_impl<ITYPE>::~number_file_sink_impl()
    {
      if (d_fp) {
        fprintf(stderr, "number_file_sink_impl::on_exit_handler: closing file descriptor\n");
        fclose(d_fp);
      }
    }

    template<typename ITYPE>
    void
    number_file_sink_impl<ITYPE>::print(const ITYPE *values, std::size_t N)
    {
      for (std::size_t n = 0; n < N; ++n)
        fprintf(d_fp, "%s\t", std::to_string(values[n]).c_str());
      fprintf(d_fp, "%s", d_delimiter.c_str());
    }

    template<>
    void
    number_file_sink_impl<float>::print(const float *values, std::size_t N)
    {
      for (std::size_t n = 0; n < N; ++n)
        fprintf(d_fp, "%.15e\t", values[n]);
      fprintf(d_fp, "%s", d_delimiter.c_str());
    }

    template<>
    void
    number_file_sink_impl<double>::print(const double *values, std::size_t N)
    {
      for (std::size_t n = 0; n < N; ++n)
        fprintf(d_fp, "%.15e\t", values[n]);
      fprintf(d_fp, "%s", d_delimiter.c_str());
    }

    template<>
    void
    number_file_sink_impl<std::complex<float>>::print(const std::complex<float> *values, std::size_t N)
    {
      for (std::size_t n = 0; n < N; ++n)
        fprintf(d_fp, "%.15e %.15e\t", values[n].real(), values[n].imag());
      fprintf(d_fp, "%s", d_delimiter.c_str());
    }

    template<>
    void
    number_file_sink_impl<std::complex<double>>::print(const std::complex<double> *values, std::size_t N)
    {
      for (std::size_t n = 0; n < N; ++n)
        fprintf(d_fp, "%.15e %.15e\t", values[n].real(), values[n].imag());
      fprintf(d_fp, "%s", d_delimiter.c_str());
    }

    template<typename ITYPE>
    int
    number_file_sink_impl<ITYPE>::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      if (d_fp == NULL)
        return 0;

      const std::size_t N = input_items.size();
      ITYPE values[N];

      for (int i = 0; i < noutput_items; ++i) {
        for (std::size_t n = 0; n < N; ++n) {
          const ITYPE* iptrn = (const ITYPE*) input_items[n];
          values[n] = iptrn[i];
        }
        print(values, N);
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

    template class number_file_sink<std::int8_t>;
    template class number_file_sink<std::uint8_t>;
    template class number_file_sink<std::int16_t>;
    template class number_file_sink<std::uint16_t>;
    template class number_file_sink<std::int32_t>;
    template class number_file_sink<std::uint32_t>;
    template class number_file_sink<std::int64_t>;
    template class number_file_sink<std::uint64_t>;
    template class number_file_sink<float>;
    template class number_file_sink<double>;
    template class number_file_sink<std::complex<float>>;
    template class number_file_sink<std::complex<double>>;

  } /* namespace gnss */
} /* namespace gr */

