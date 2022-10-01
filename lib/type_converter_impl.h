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

#ifndef INCLUDED_GNSS_TYPE_CONVERTER_IMPL_H
#define INCLUDED_GNSS_TYPE_CONVERTER_IMPL_H

#include <gnss/type_converter.h>

namespace gr {
  namespace gnss {

    template<typename ITYPE, typename OTYPE>
    class type_converter_impl : public type_converter<ITYPE, OTYPE>
    {
    public:
      type_converter_impl(size_t vlen);
      ~type_converter_impl();

      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);

    private:
      const size_t d_vlen;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_TYPE_CONVERTER_IMPL_H */

