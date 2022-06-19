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

#ifndef INCLUDED_GNSS_ACQUISITION_AND_TRACKING_IMPL_H
#define INCLUDED_GNSS_ACQUISITION_AND_TRACKING_IMPL_H

#include <gnss/acquisition_and_tracking.h>
#include <gnuradio/fft/fft.h>
#include <memory>
#include <array>
#include <numeric>

#include "gps_parameters.h"
#include "magnitude_stats.h"
#include "doppler_shifts.h"
#include "dll_loop_filter.h"
#include "pll_loop_filter.h"

#define IVLEN 1
#define OVLEN 1

namespace gr {
  namespace gnss {

    constexpr double doppler_f0 = -10000.0;
    constexpr double doppler_df = 200.0;
    constexpr int doppler_bins = 100 + 1;
    constexpr int correlation_taps = 3;
    constexpr double correlation_shift_coarse = 0.25;
    constexpr double correlation_shift_fine = 0.15;
    constexpr double correlation_shifts_coarse[correlation_taps] = {-correlation_shift_coarse, 0, +correlation_shift_coarse};
    constexpr double correlation_shifts_fine[correlation_taps] = {-correlation_shift_fine, 0, +correlation_shift_fine};

    template<typename ITYPE, typename OTYPE>
    class acquisition_and_tracking_impl : public acquisition_and_tracking
    {
    public:
      acquisition_and_tracking_impl(double sampling_freq,
                                    double dll_bw_coarse,
                                    double pll_bw_coarse,
                                    double dll_bw_fine,
                                    double pll_bw_fine);
      ~acquisition_and_tracking_impl();

      void set_acq_params(navigation_system_e system, int id) override;
      void get_acq_params(navigation_system_e& system, int& id) const override;

      void forecast(int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items);

    private:
      enum class state_e
      {
        UNLOCKED,
        ACQUISITION,
        TRACKING_COARSE,
        TRACKING_FINE
      };

      template<typename T, std::size_t N>
      struct ringbuffer
      {
        ringbuffer() { reset(); };
        void reset() { n = 0; samples.fill(0); }
        void add(const T& sample) { samples[n++ % N] = sample; }
        T avg() const { std::size_t k = n < N ? n : N;
          return k > 0
            ? std::accumulate(std::begin(samples), std::begin(samples) + k, 0.0) / static_cast<T>(k)
            : 0;
        }

        std::size_t n;
        std::array<T, N> samples;
      };

      void init_spreading_code();

      int work_acquisition(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );

      template<std::size_t CTAPS>
      int work_tracking(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items,
              const double (&correlation_shifts)[CTAPS]
      );

      const double d_sampling_freq;
      const double d_dll_bw_coarse;
      const double d_pll_bw_coarse;
      const double d_dll_bw_fine;
      const double d_pll_bw_fine;
      const int d_spreading_code_samples;
      state_e d_state;
      navigation_system_e d_navigation_system;
      int d_id;
      std::vector<gr_complex> d_spreading_code;
      std::unique_ptr<gr::fft::fft_complex> d_ffft;
      std::unique_ptr<gr::fft::fft_complex> d_ifft;
      doppler_shifts<doppler_bins> d_doppler_shifts;
      std::vector<float> d_magnitude;
      magnitude_stats<float> d_max_magnitude_stats;
      double d_code_chip_rate;
      double d_code_offset_chips;
      double d_code_offset_samples;
      double d_freq;
      double d_freq_offset;
      dll_loop_filter<1> d_dll_loop_filter;
      pll_loop_filter<2> d_pll_loop_filter;
      ringbuffer<double, GPS_CA_CODES_PER_NAV_MESSAGE_BIT> d_tracking_history;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_ACQUISITION_AND_TRACKING_IMPL_H */

