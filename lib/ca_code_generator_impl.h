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

#ifndef INCLUDED_GNSS_CA_CODE_GENERATOR_IMPL_H
#define INCLUDED_GNSS_CA_CODE_GENERATOR_IMPL_H

#include <memory>

#include <gnss/ca_code_generator.h>
#include "gnss_parameters.h"
#include "ca_code.h"

#define IVLEN0 0
#define OVLEN0 1

namespace gr {
  namespace gnss {

    template<typename T>
    class ca_code_generator_impl : public ca_code_generator<T>
    {
    public:
      ca_code_generator_impl(size_t vlen, double sampling_freq, unsigned svid, ca_code_domain_e domain);
      ~ca_code_generator_impl();

      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );

    private:
      const size_t d_vlen;
      const int d_n_samples;
      std::vector<T> d_code_sampled;
      int d_n;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_CA_CODE_GENERATOR_IMPL_H */

