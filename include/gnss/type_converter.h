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

#ifndef INCLUDED_GNSS_TYPE_CONVERTER_H
#define INCLUDED_GNSS_TYPE_CONVERTER_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace gnss {

    /*!
     * \brief <+description of block+>
     * \ingroup gnss
     *
     */
    template<typename ITYPE, typename OTYPE>
    class GNSS_API type_converter : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<type_converter> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gnss::type_converter.
       *
       * To avoid accidental use of raw pointers, gnss::type_converter's
       * constructor is in a private implementation
       * class. gnss::type_converter::make is the public interface for
       * creating new instances.
       */
      static sptr make(size_t vlen);
    };

    using fc32_to_fc64 = type_converter<std::complex<float>, std::complex<double>>;
    using fc64_to_fc32 = type_converter<std::complex<double>, std::complex<float>>;

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_TYPE_CONVERTER_H */

