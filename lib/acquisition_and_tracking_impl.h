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

#include "doppler_shifts.h"
#include "dll_loop_filter.h"
#include "pll_loop_filter.h"

#define IVLEN0 1
#define OVLEN0 1

namespace gr {
  namespace gnss {

    constexpr double doppler_f0 = -10000.0;
    constexpr double doppler_df = 200.0;
    constexpr int doppler_bins = 100 + 1;
    constexpr int correlation_taps = 3;
    constexpr double correlation_shift = 0.25;
    constexpr double correlation_shifts[correlation_taps] = {-correlation_shift, 0, +correlation_shift};

    template<typename T>
    struct magnitude_stats
    {
      magnitude_stats(int freq_index = -1)
      {
        reset(freq_index);
      }

      void reset(int freq_index = -1)
      {
        d_freq_index = freq_index;
        d_max_index = -1;
        d_min = 0.0;
        d_max = 0.0;
        d_avg = 0.0;
      }

      std::string to_string() const
      {
        char buf[128];

        snprintf(buf, sizeof(buf), "idx: %d, freq_idx: %d, min: %e, max: %e, avg: %e",
          d_max_index, d_freq_index, d_min, d_max, d_avg);

        return std::string(buf);
      }

      static magnitude_stats get(const std::vector<T>& v, int freq_index)
      {
        magnitude_stats stats{freq_index};

        if (v.size() > 0) {
          stats.d_max_index = 0;
          stats.d_min = stats.d_max = stats.d_avg = v[0];

          for (int i = 1; i < v.size(); ++i) {
            if (v[i] < stats.d_min) {
              stats.d_min = v[i];
            }
            if (v[i] > stats.d_max) {
              stats.d_max = v[i];
              stats.d_max_index = i;
            }
            stats.d_avg += v[i];
          }

          if (v.size() > 1) {
            stats.d_avg -= stats.d_max;
            stats.d_avg /= (v.size() - 1);
          }
        }

        return stats;
      }

      int d_freq_index;
      int d_max_index;
      T d_min;
      T d_max;
      T d_avg;
    };

    template<typename ITYPE0, typename OTYPE0>
    class acquisition_and_tracking_impl : public acquisition_and_tracking
    {
    public:
      acquisition_and_tracking_impl(double sampling_freq);
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
        ACQUISITION,
        TRACKING,
        UNLOCKED
      };

      void init_spreading_code();

      int work_acquisition(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );

      int work_tracking(
              int noutput_items,
              gr_vector_int &ninput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );

      const double d_sampling_freq;
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
      double d_code_offset;
      double d_freq;
      double d_freq_offset;
      dll_loop_filter<1> d_dll_loop_filter;
      pll_loop_filter<2> d_pll_loop_filter;
    };

  } // namespace gnss
} // namespace gr

#endif /* INCLUDED_GNSS_ACQUISITION_AND_TRACKING_IMPL_H */

