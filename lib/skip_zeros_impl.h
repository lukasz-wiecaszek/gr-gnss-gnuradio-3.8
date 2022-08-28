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

#ifndef INCLUDED_GNSS_SKIP_ZEROS_IMPL_H
#define INCLUDED_GNSS_SKIP_ZEROS_IMPL_H

#include <gnss/skip_zeros.h>
#include "vector3d.h"

#define IVLEN 1
#define OVLEN 1

namespace gr {
  namespace gnss {

    template<typename ITYPE, typename OTYPE>
    class skip_zeros_impl : public skip_zeros
    {
    public:
      skip_zeros_impl();
      ~skip_zeros_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);

    private:
      bool is_null(const vector3d& v) {
        return
          (v.get({0}, {0}) == 0) &&
          (v.get({1}, {0}) == 0) &&
          (v.get({2}, {0}) == 0);
      }
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_SKIP_ZEROS_IMPL_H */

