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

#ifndef INCLUDED_GNSS_GPS_NAV_MESSAGE_SUBFRAME_HEADER_H
#define INCLUDED_GNSS_GPS_NAV_MESSAGE_SUBFRAME_HEADER_H

#include <array>
#include <string>
#include <sstream>
#include "gps_nav_message_word.h"

namespace gr {
  namespace gnss {

    class gps_nav_message_subframe_header
    {
    public:
      int tlm_message()            const { return words[0].field(9, 22); }
      bool integrity_status_flag() const { return words[0].flag(23); }

      int tow_count_message()      const { return words[1].field(1, 17); }
      bool alert_flag()            const { return words[1].flag(18); }
      bool anti_spoof_flag()       const { return words[1].flag(19); }
      int subframe_id()            const { return words[1].field(20, 22); }

      std::string to_string() const
      {
        std::ostringstream stream;

        stream << "subframe id: " << std::dec << subframe_id();
        stream << ", ";
        stream << "tow: " << std::dec << tow_count_message();

        return stream.str();
      }

      operator std::string () const
      {
        return to_string();
      }

    protected:
      std::array<gps_nav_message_word, GPS_NAV_MESSAGE_WORDS_PER_SUBFRAME> words;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_GPS_NAV_MESSAGE_SUBFRAME_HEADER_H */

