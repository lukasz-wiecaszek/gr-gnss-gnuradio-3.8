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

#ifndef INCLUDED_GNSS_ACQUISITION_AND_TRACKING_H
#define INCLUDED_GNSS_ACQUISITION_AND_TRACKING_H

#include <gnss/api.h>
#include <gnss/navigation_system.h>
#include <gnuradio/block.h>

namespace gr {
  namespace gnss {

    /*!
     * \brief <+description of block+>
     * \ingroup gnss
     *
     */
    class GNSS_API acquisition_and_tracking : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<acquisition_and_tracking> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of gnss::acquisition_and_tracking.
       *
       * To avoid accidental use of raw pointers, gnss::acquisition_and_tracking's
       * constructor is in a private implementation
       * class. gnss::acquisition_and_tracking::make is the public interface for
       * creating new instances.
       */
      static sptr make(double sampling_freq,
        double dll_bw_coarse, double pll_bw_coarse, double dll_bw_fine, double pll_bw_fine);

      virtual void set_acq_params(navigation_system_e system, int id) = 0;
      virtual void get_acq_params(navigation_system_e& system, int& id) const = 0;

    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_ACQUISITION_AND_TRACKING_H */

