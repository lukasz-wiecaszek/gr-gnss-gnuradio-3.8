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

#ifndef INCLUDED_GNSS_CA_SYBMOLS_TO_NAV_BITS_H
#define INCLUDED_GNSS_CA_SYBMOLS_TO_NAV_BITS_H

#include <gnss/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace gnss {

    /*!
     * \brief <+description of block+>
     * \ingroup gnss
     *
     */
    class GNSS_API ca_sybmols_to_nav_bits : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<ca_sybmols_to_nav_bits> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gnss::ca_sybmols_to_nav_bits.
       *
       * To avoid accidental use of raw pointers, gnss::ca_sybmols_to_nav_bits's
       * constructor is in a private implementation
       * class. gnss::ca_sybmols_to_nav_bits::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_CA_SYBMOLS_TO_NAV_BITS_H */

