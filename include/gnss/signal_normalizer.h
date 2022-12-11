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

#ifndef INCLUDED_GNSS_SIGNAL_NORMALIZER_H
#define INCLUDED_GNSS_SIGNAL_NORMALIZER_H

#include <gnss/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace gnss {

    /*!
     * \brief Converts different input IQ formats to
     *        std::complex<double> sample/item normalized
     *        in the range [-1.0 .. +1.0] for I and Q parts.
     * \ingroup gnss
     *
     */
    template<typename ITYPE>
    class GNSS_API signal_normalizer : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<signal_normalizer> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gnss::signal_normalizer.
       */
      static sptr make(size_t vlen, size_t adc_resolution);
    };

    using signal_normalizer_s8 = signal_normalizer<std::int8_t>;
    using signal_normalizer_u8 = signal_normalizer<std::uint8_t>;
    using signal_normalizer_s16 = signal_normalizer<std::int16_t>;
    using signal_normalizer_u16 = signal_normalizer<std::uint16_t>;

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_SIGNAL_NORMALIZER_H */

