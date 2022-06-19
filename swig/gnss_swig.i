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
#include "gnss/pseudoranges_decoder.h"
#include "gnss/rft_type.h"
#include "gnss/rft.h"
#include "gnss/geojson_file_sink.h"
#include "gnss/gnss_channel.h"
%}

%include "gnss/navigation_system.h"
%include "gnss/rft_type.h"

%include "gnss/acquisition_and_tracking.h"
GR_SWIG_BLOCK_MAGIC2(gnss, acquisition_and_tracking);

%include "gnss/ca_code_domain.h"

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

%include "gnss/pseudoranges_decoder.h"
GR_SWIG_BLOCK_MAGIC2(gnss, pseudoranges_decoder);

%include "gnss/rft.h"
GR_SWIG_BLOCK_MAGIC2(gnss, rft);
%include "gnss/geojson_file_sink.h"
GR_SWIG_BLOCK_MAGIC2(gnss, geojson_file_sink);
%include "gnss/gnss_channel.h"
GR_SWIG_BLOCK_MAGIC2(gnss, gnss_channel);
