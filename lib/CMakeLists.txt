# Copyright 2011,2012,2016,2018,2019 Free Software Foundation, Inc.
#
# This file was generated by gr_modtool, a tool from the GNU Radio framework
# This file is a part of gr-gnss
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX
list(APPEND gnss_sources
    acquisition_and_tracking_impl.cc
    ca_code_generator_impl.cc
    resampler_impl.cc
    ca_sybmols_to_nav_bits_impl.cc
    nav_message_decoder_impl.cc
    rft_impl.cc
    geojson_file_sink_impl.cc
    gnss_channel_impl.cc
    pseudoranges_decoder_impl.cc
    pvt_impl.cc
    number_file_sink_impl.cc
    skip_zeros_impl.cc
    doppler_shift_impl.cc
    type_converter_impl.cc
    signal_normalizer_impl.cc )

set(gnss_sources "${gnss_sources}" PARENT_SCOPE)
if(NOT gnss_sources)
    MESSAGE(STATUS "No C++ sources... skipping lib/")
    return()
endif(NOT gnss_sources)

add_library(gnuradio-gnss SHARED ${gnss_sources})
target_link_libraries(gnuradio-gnss gnuradio-runtime gnuradio-fft)
target_include_directories(gnuradio-gnss
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
  )
set_target_properties(gnuradio-gnss PROPERTIES DEFINE_SYMBOL "gnuradio_gnss_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-gnss PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

add_subdirectory(utils)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-gnss)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

# If your unit tests require special include paths, add them here
#include_directories()
# List all files that contain Boost.UTF unit tests here
list(APPEND test_gnss_sources
)
# Anything we need to link to for the unit tests go here
list(APPEND GR_TEST_TARGET_DEPS gnuradio-gnss)

if(NOT test_gnss_sources)
    MESSAGE(STATUS "No C++ unit tests... skipping")
    return()
endif(NOT test_gnss_sources)

foreach(qa_file ${test_gnss_sources})
    GR_ADD_CPP_TEST("gnss_${qa_file}"
        ${CMAKE_CURRENT_SOURCE_DIR}/${qa_file}
    )
endforeach(qa_file)
