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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <gnuradio/nco.h>
#include <volk/volk.h>

#include <numeric>

#include "acquisition_and_tracking_impl.h"
#include "gnss_parameters.h"
#include "ca_code.h"
#include "dll_discriminators.h"
#include "pll_discriminators.h"

namespace {

} // anonymous namespace

namespace gr {
  namespace gnss {

    using gps_ca_code = ca_code<int, GPS_CA_CODE_LENGTH>;

    /*===========================================================================*\
    * public function definitions
    \*===========================================================================*/
    acquisition_and_tracking::sptr
    acquisition_and_tracking::make(double sampling_freq)
    {
      return gnuradio::get_initial_sptr
        (new acquisition_and_tracking_impl<gr_complex, gr_complex>(sampling_freq));
    }

    template<typename ITYPE0, typename OTYPE0>
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::acquisition_and_tracking_impl(double sampling_freq)
      : gr::block("acquisition_and_tracking",
                  gr::io_signature::make(1, 1, sizeof(ITYPE0)),
                  gr::io_signature::make(1, 1, sizeof(OTYPE0))),
        d_sampling_freq{sampling_freq},
        d_spreading_code_samples{static_cast<decltype(d_spreading_code_samples)>(ceil(d_sampling_freq / GPS_CA_CODE_RATE))},
        d_state{state_e::ACQUISITION},
        d_navigation_system{NAVIGATION_SYSTEM_UNDEFINED},
        d_id{-1},
        d_spreading_code(d_spreading_code_samples),
        d_ffft{std::make_unique<gr::fft::fft_complex>(d_spreading_code_samples, true)},
        d_ifft{std::make_unique<gr::fft::fft_complex>(d_spreading_code_samples, false)},
        d_doppler_shifts{sampling_freq, doppler_f0, doppler_df, d_spreading_code_samples},
        d_magnitude(d_spreading_code_samples),
        d_max_magnitude_stats{},
        d_code_chip_rate{GPS_CA_CODE_CHIP_RATE},
        d_code_offset{0.0},
        d_freq{0.0},
        d_freq_offset{0.0},
        d_dll_loop_filter{1.0 / GPS_CA_CODE_RATE,  4.0},
        d_pll_loop_filter{1.0 / GPS_CA_CODE_RATE, 40.0}
    {
      set_relative_rate(1, d_spreading_code_samples);
      printf("%s: d_spreading_code_samples: %d\n", __func__, d_spreading_code_samples);
    }

    template<typename ITYPE0, typename OTYPE0>
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::~acquisition_and_tracking_impl()
    {
    }

    template<typename ITYPE0, typename OTYPE0>
    void
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::set_acq_params(navigation_system_e system, int id)
    {
      gr::thread::scoped_lock lock(d_setlock);

      d_navigation_system = system;
      d_id = id;

      init_spreading_code();
      d_max_magnitude_stats.reset();
    }

    template<typename ITYPE0, typename OTYPE0>
    void
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::get_acq_params(navigation_system_e& system, int& id) const
    {
      system = d_navigation_system;
      id = d_id;
    }

    template<typename ITYPE0, typename OTYPE0>
    void
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::forecast(int noutput_items, gr_vector_int &ninput_items_required)
    {
      // 1 output item requires d_spreading_code_samples (integration period)
      int nrequired = noutput_items * d_spreading_code_samples;

      for (auto&& element : ninput_items_required)
        element = nrequired;
    }

    template<typename ITYPE0, typename OTYPE0>
    int
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::general_work(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      int nproduced;

      gr::thread::scoped_lock lock(d_setlock);

      switch (d_state)
      {
        case state_e::ACQUISITION:
          nproduced = work_acquisition(noutput_items, ninput_items, input_items, output_items);
          break;

        case state_e::TRACKING:
          nproduced = work_tracking(noutput_items, ninput_items, input_items, output_items);
          break;

        case state_e::UNLOCKED: /* fall through */
        default:
          consume_each(ninput_items[0]);
          nproduced = 0;
          break;
      }

      return nproduced;
    }

    /*===========================================================================*\
    * protected function definitions
    \*===========================================================================*/

    /*===========================================================================*\
    * private function definitions
    \*===========================================================================*/
    template<typename ITYPE0, typename OTYPE0>
    void
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::init_spreading_code()
    {
      const std::shared_ptr<gps_ca_code> code = gps_ca_code::get(d_id);
      if (code == nullptr)
        throw std::out_of_range("invalid space vehicle id");

      for (int i = 0; i < d_spreading_code_samples; ++i)
        d_spreading_code[i] = (*code)[(i * GPS_CA_CODE_LENGTH) / d_spreading_code_samples] ?
          gr_complex{+1.0f, 0.0f} : gr_complex{-1.0f, 0.0f};

      memcpy(d_ffft->get_inbuf(), d_spreading_code.data(), d_spreading_code_samples * sizeof(gr_complex));
      d_ffft->execute();

      volk_32fc_conjugate_32fc(d_spreading_code.data(), d_ffft->get_outbuf(), d_spreading_code_samples);
    }

    template<typename ITYPE0, typename OTYPE0>
    int
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::work_acquisition(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE0* iptr0 = (const ITYPE0*) input_items[0];
      OTYPE0* optr0 = (OTYPE0*) output_items[0];

      if (ninput_items[0] < d_spreading_code_samples)
        return 0;

      for (int n = 0; n < d_doppler_shifts.bins(); ++n) {
        // Doppler shift
        volk_32fc_x2_multiply_32fc(d_ffft->get_inbuf(), iptr0, d_doppler_shifts[n].data(), d_doppler_shifts[n].size());

        // Compute the forward fft of the doppler shifted incoming signal
        d_ffft->execute();

        // Multiply doppler shifted incoming signal with the spreading code
        volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_ffft->get_outbuf(), d_spreading_code.data(), d_spreading_code.size());

        // Compute the inverse fft
        d_ifft->execute();

        // Compute the magnitude^2
        volk_32fc_magnitude_squared_32f(d_magnitude.data(), d_ifft->get_outbuf(), d_magnitude.size());

        // Get statistics
        magnitude_stats<float> stats = magnitude_stats<float>::get(d_magnitude, n);
        if (n > 0) {
          if (stats.d_max > d_max_magnitude_stats.d_max)
            d_max_magnitude_stats = stats;
        } else
          d_max_magnitude_stats = stats;
      }

      if (d_max_magnitude_stats.d_max > (d_max_magnitude_stats.d_avg * 2)) { // this needs to be reworked
        printf("id: %d, f: %.1f Hz [%s]\n",
          d_id, d_doppler_shifts.n_to_freq(d_max_magnitude_stats.d_freq_index), d_max_magnitude_stats.to_string().c_str());
        d_freq = d_doppler_shifts.n_to_freq(d_max_magnitude_stats.d_freq_index);
        d_dll_loop_filter.reset(0.0);
        d_pll_loop_filter.reset(d_freq);

        d_state = state_e::TRACKING;
      }
      else
        d_state = state_e::UNLOCKED;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each(d_max_magnitude_stats.d_max_index);

      // Tell runtime system how many output items we produced.
      return 0;
    }

    template<typename ITYPE0, typename OTYPE0>
    int
    acquisition_and_tracking_impl<ITYPE0, OTYPE0>::work_tracking(int noutput_items,
        gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const ITYPE0* iptr0 = (const ITYPE0*) input_items[0];
      OTYPE0* optr0 = (OTYPE0*) output_items[0];
      int freq_index = d_max_magnitude_stats.d_freq_index;

      if (ninput_items[0] < d_spreading_code_samples)
        return 0;

      const std::shared_ptr<gps_ca_code> code = gps_ca_code::get(d_id);
      if (code == nullptr)
        throw std::out_of_range("invalid space vehicle id");

      // Doppler shift
      gr::fxpt_nco nco;
      std::vector<gr_complex> doppler_shift(d_spreading_code_samples);
      std::vector<gr_complex> input_stream(d_spreading_code_samples);

      nco.set_freq(GR_M_TWOPI * d_freq / d_sampling_freq);
      nco.set_phase(d_freq_offset);
      nco.sincos(doppler_shift.data(), doppler_shift.size(), 1);
      d_freq_offset = nco.get_phase();

      volk_32fc_x2_multiply_32fc(input_stream.data(), iptr0, doppler_shift.data(), d_spreading_code_samples);

      std::array<std::vector<gr_complex>, correlation_taps> spreading_codes;
      std::array<gr_complex, correlation_taps> correlations;
      std::vector<gr_complex> output_stream(d_spreading_code_samples);

      double code_phase_increment = d_code_chip_rate / d_sampling_freq;

      for (int tap = 0; tap < correlation_taps; ++tap) {
        std::vector<gr_complex>& spreading_code = spreading_codes[tap];
        spreading_code.resize(d_spreading_code_samples);
        for (int i = 0; i < d_spreading_code_samples; ++i) {
          int idx = static_cast<int>(i * code_phase_increment + d_code_offset + correlation_shifts[tap]);
          idx += GPS_CA_CODE_LENGTH;
          idx %= GPS_CA_CODE_LENGTH;
          spreading_code[i] = (*code)[idx] ? gr_complex{+1.0f, 0.0f} : gr_complex{-1.0f, 0.0f};
        }

        // Multiply doppler shifted incoming signal with the spreading code
        volk_32fc_x2_multiply_32fc(output_stream.data(), input_stream.data(), spreading_code.data(), d_spreading_code_samples);

        correlations[tap] = std::accumulate(output_stream.begin(), output_stream.end(), gr_complex{0.0f, 0.0f});
      }

      gr_complex E = correlations[0];
      gr_complex P = correlations[1];
      gr_complex L = correlations[2];

      double dll_discriminator = dll_discriminator_noncoherent_e_minus_l_power(E, L);
      double dll_discriminator_filtered = d_dll_loop_filter[(1.0 - correlation_shift) * dll_discriminator];

      double pll_discriminator = pll_discriminator_two_quadrant_arctangent(P);
      double pll_discriminator_filtered = d_pll_loop_filter[pll_discriminator / GR_M_TWOPI];

      d_freq = pll_discriminator_filtered;

      d_code_chip_rate = GPS_CA_CODE_CHIP_RATE - dll_discriminator_filtered;
      d_code_chip_rate -= d_freq * GPS_CA_CODE_CHIP_RATE / GPS_L1_FREQ_HZ;

      d_code_offset += GPS_CA_CODE_LENGTH * d_code_chip_rate / GPS_CA_CODE_CHIP_RATE - GPS_CA_CODE_LENGTH;
      if (d_code_offset < 0)
        d_code_offset += GPS_CA_CODE_LENGTH;
      if (d_code_offset > GPS_CA_CODE_LENGTH)
        d_code_offset -= GPS_CA_CODE_LENGTH;

      *optr0 = P;

      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each(d_spreading_code_samples);

      // Tell runtime system how many output items we produced.
      return 1;
    }

  } /* namespace gnss */
} /* namespace gr */

