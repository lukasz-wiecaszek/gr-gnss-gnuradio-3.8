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

#ifndef INCLUDED_GNSS_NUMBER_FILE_SINK_H
#define INCLUDED_GNSS_NUMBER_FILE_SINK_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>
#include <cstdint>

namespace gr {
  namespace gnss {

    /*!
     * \brief <+description of block+>
     * \ingroup gnss
     *
     */
    template<typename ITYPE>
    class GNSS_API number_file_sink : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<number_file_sink<ITYPE>> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gnss::number_file_sink.
       *
       * To avoid accidental use of raw pointers, gnss::number_file_sink's
       * constructor is in a private implementation
       * class. gnss::number_file_sink::make is the public interface for
       * creating new instances.
       */
      static sptr make(const char *filename, const char *delimiter);
    };

    using number_file_sink_s8 = number_file_sink<std::int8_t>;
    using number_file_sink_u8 = number_file_sink<std::uint8_t>;
    using number_file_sink_s16 = number_file_sink<std::int16_t>;
    using number_file_sink_u16 = number_file_sink<std::uint16_t>;
    using number_file_sink_s32 = number_file_sink<std::int32_t>;
    using number_file_sink_u32 = number_file_sink<std::uint32_t>;
    using number_file_sink_f = number_file_sink<float>;
    using number_file_sink_c = number_file_sink<gr_complex>;

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_NUMBER_FILE_SINK_H */

