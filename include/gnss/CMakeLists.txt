# Copyright 2011,2012 Free Software Foundation, Inc.
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
# Install public header files
########################################################################
install(FILES
    api.h
    navigation_system.h
    acquisition_and_tracking.h
    ca_code_domain.h
    ca_code_generator.h
    resampler.h
    ca_sybmols_to_nav_bits.h
    nav_message_decoder.h
    rft.h
    geojson_file_sink.h
    gnss_channel.h
    pseudoranges_decoder.h
    pvt.h
    number_file_sink.h
    skip_zeros.h
    doppler_shift.h
    type_converter.h
    signal_normalizer.h DESTINATION include/gnss
)
