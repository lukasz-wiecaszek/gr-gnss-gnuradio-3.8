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

#ifndef INCLUDED_GNSS_RESAMPLER_IMPL_H
#define INCLUDED_GNSS_RESAMPLER_IMPL_H

#include <gnss/resampler.h>

#define IVLEN0 1
#define OVLEN0 1

namespace gr {
  namespace gnss {

    template<typename ITYPE0, typename OTYPE0>
    class resampler_impl : public resampler
    {
    private:
      const double d_fs_in;
      const double d_fs_out;
      const uint32_t d_phase_step;
      uint32_t d_phase;
      uint32_t d_lphase;

    public:
      resampler_impl(double fs_in, double fs_out);
      ~resampler_impl();

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_RESAMPLER_IMPL_H */

