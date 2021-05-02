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

#ifndef INCLUDED_GNSS_CA_CODE_GENERATOR_H
#define INCLUDED_GNSS_CA_CODE_GENERATOR_H

#include <gnss/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace gnss {

    /*!
     * \brief <+description of block+>
     * \ingroup gnss
     *
     */
    template<typename T>
    class GNSS_API ca_code_generator : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<ca_code_generator> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gnss::ca_code_generator.
       *
       * To avoid accidental use of raw pointers, gnss::ca_code_generator's
       * constructor is in a private implementation
       * class. gnss::ca_code_generator::make is the public interface for
       * creating new instances.
       */
      static sptr make(unsigned svid, double sampling_freq);
    };

    using ca_code_generator_b = ca_code_generator<int8_t>;
    using ca_code_generator_s = ca_code_generator<int16_t>;
    using ca_code_generator_i = ca_code_generator<int32_t>;
    using ca_code_generator_f = ca_code_generator<float>;
    using ca_code_generator_c = ca_code_generator<gr_complex>;

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_CA_CODE_GENERATOR_H */

