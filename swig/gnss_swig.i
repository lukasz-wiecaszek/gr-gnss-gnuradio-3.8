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
%}

%include "gnss/navigation_system.h"

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
