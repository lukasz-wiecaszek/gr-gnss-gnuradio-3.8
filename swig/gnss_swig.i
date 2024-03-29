/* -*- c++ -*- */

#define GNSS_API

%include "gnuradio.i"           // the common stuff

//load generated python docstrings
%include "gnss_swig_doc.i"

%{
#include "gnss/navigation_system.h"
#include "gnss/acquisition_and_tracking.h"
#include "gnss/ca_code_domain.h"
#include "gnss/ca_code_generator.h"
#include "gnss/resampler.h"
#include "gnss/ca_sybmols_to_nav_bits.h"
#include "gnss/nav_message_decoder.h"
#include "gnss/rft_type.h"
#include "gnss/rft.h"
#include "gnss/geojson_file_sink.h"
#include "gnss/gnss_channel.h"
#include "gnss/pseudoranges_decoder.h"
#include "gnss/pvt.h"
#include "gnss/number_file_sink.h"
#include "gnss/doppler_shift.h"
#include "gnss/skip_zeros.h"
#include "gnss/type_converter.h"
#include "gnss/signal_normalizer.h"
%}

%include "gnss/navigation_system.h"
%include "gnss/rft_type.h"
%include "gnss/ca_code_domain.h"

%include "gnss/acquisition_and_tracking.h"
GR_SWIG_BLOCK_MAGIC2(gnss, acquisition_and_tracking);

%include "gnss/ca_code_generator.h"
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, ca_code_generator_b, ca_code_generator<std::int8_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, ca_code_generator_s, ca_code_generator<std::int16_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, ca_code_generator_i, ca_code_generator<std::int32_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, ca_code_generator_f, ca_code_generator<float>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, ca_code_generator_c, ca_code_generator<gr_complex>);

%include "gnss/resampler.h"
GR_SWIG_BLOCK_MAGIC2(gnss, resampler);

%include "gnss/ca_sybmols_to_nav_bits.h"
GR_SWIG_BLOCK_MAGIC2(gnss, ca_sybmols_to_nav_bits);

%include "gnss/nav_message_decoder.h"
GR_SWIG_BLOCK_MAGIC2(gnss, nav_message_decoder);

%include "gnss/rft.h"
GR_SWIG_BLOCK_MAGIC2(gnss, rft);

%include "gnss/geojson_file_sink.h"
GR_SWIG_BLOCK_MAGIC2(gnss, geojson_file_sink);

%include "gnss/gnss_channel.h"
GR_SWIG_BLOCK_MAGIC2(gnss, gnss_channel);

%include "gnss/pseudoranges_decoder.h"
GR_SWIG_BLOCK_MAGIC2(gnss, pseudoranges_decoder);

%include "gnss/pvt.h"
GR_SWIG_BLOCK_MAGIC2(gnss, pvt);

%include "gnss/number_file_sink.h"
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_s8, number_file_sink<std::int8_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_u8, number_file_sink<std::uint8_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_s16, number_file_sink<std::int16_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_u16, number_file_sink<std::uint16_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_s32, number_file_sink<std::int32_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_u32, number_file_sink<std::uint32_t>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_f32, number_file_sink<float>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_f64, number_file_sink<double>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_fc32, number_file_sink<std::complex<float>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, number_file_sink_fc64, number_file_sink<std::complex<double>>);

%include "gnss/skip_zeros.h"
GR_SWIG_BLOCK_MAGIC2(gnss, skip_zeros);

%include "gnss/doppler_shift.h"
GR_SWIG_BLOCK_MAGIC2(gnss, doppler_shift);

%include "gnss/type_converter.h"
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, f32_to_f64, type_converter<float, double>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, f64_to_f32, type_converter<double, float>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, fc32_to_fc64, type_converter<std::complex<float>, std::complex<double>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, fc64_to_fc32, type_converter<std::complex<double>, std::complex<float>>);

%include "gnss/signal_normalizer.h"
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_s8_fc32, signal_normalizer<std::int8_t, std::complex<float>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_u8_fc32, signal_normalizer<std::uint8_t, std::complex<float>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_s16_fc32, signal_normalizer<std::int16_t, std::complex<float>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_u16_fc32, signal_normalizer<std::uint16_t, std::complex<float>>);

GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_s8_fc64, signal_normalizer<std::int8_t, std::complex<double>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_u8_fc64, signal_normalizer<std::uint8_t, std::complex<double>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_s16_fc64, signal_normalizer<std::int16_t, std::complex<double>>);
GR_SWIG_BLOCK_MAGIC2_TMPL(gnss, signal_normalizer_u16_fc64, signal_normalizer<std::uint16_t, std::complex<double>>);
