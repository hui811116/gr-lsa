# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
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

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND lsa_sources
    eng_det_cc_impl.cc
    interference_energy_detector_cc_impl.cc
    modified_costas_loop_cc_impl.cc
    burst_tagger_cc_impl.cc
    correlate_sync_cc_impl.cc
    preamble_prefixer.cc
    su_ctrl.cc
    chip_mapper.cc
    moving_average_cc_impl.cc
    moving_average_ff_impl.cc 
    coarse_sync_cc_impl.cc
    prou_packet_sink_f_impl.cc
    interference_tagger_cc_impl.cc
    block_tagger_cc_impl.cc
    su_sr_transmitter_bb_impl.cc
    su_tx_helper.cc
    su_packet_sink_c_impl.cc
    throughput_report.cc
    utils.cc
    stop_n_wait_tx_bb_impl.cc
    stop_n_wait_rx_ctrl_cc_impl.cc
    stop_n_wait_ack.cc
    sns_tx_helper.cc
    burst_gate_cc_impl.cc
    message_file_sink.cc
    sfd_tagger_cc_impl.cc
    simple_tx.cc
    simple_rx.cc
    phy_crc.cc
    byte_to_symbol_bc_impl.cc
    throughput_file_sink.cc
    ic_resync_cc_impl.cc
    su_block_receiver_c_impl.cc
    ic_ncfo_cc_impl.cc
    arq_tx.cc
    sns_tx.cc )

set(lsa_sources "${lsa_sources}" PARENT_SCOPE)
if(NOT lsa_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT lsa_sources)

add_library(gnuradio-lsa SHARED ${lsa_sources})
target_link_libraries(gnuradio-lsa ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES})
set_target_properties(gnuradio-lsa PROPERTIES DEFINE_SYMBOL "gnuradio_lsa_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-lsa PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-lsa RUNTIME_COMPONENT "lsa_runtime" DEVEL_COMPONENT "lsa_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})
list(APPEND test_lsa_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_lsa.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_lsa.cc
    )

add_executable(test-lsa ${test_lsa_sources})

target_link_libraries(
  test-lsa
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-lsa
)

GR_ADD_TEST(test_lsa test-lsa)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

