# Copyright 2011 Free Software Foundation, Inc.
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
# Include python install macros
########################################################################
include(GrPython)
if(NOT PYTHONINTERP_FOUND)
    return()
endif()

########################################################################
# Install python sources
########################################################################
GR_PYTHON_INSTALL(
    FILES
    __init__.py
    DESTINATION ${GR_PYTHON_DIR}/lsa
)

########################################################################
# Handle the unit tests
########################################################################
include(GrTest)

set(GR_TEST_TARGET_DEPS gnuradio-lsa)
set(GR_TEST_PYTHON_DIRS ${CMAKE_BINARY_DIR}/swig)
GR_ADD_TEST(qa_eng_det_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_eng_det_cc.py)
GR_ADD_TEST(qa_header_payload_parser_cb ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_header_payload_parser_cb.py)
GR_ADD_TEST(qa_su_header_prefix ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_su_header_prefix.py)
GR_ADD_TEST(qa_interference_energy_detector_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_interference_energy_detector_cc.py)
GR_ADD_TEST(qa_modified_polyphase_time_sync_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_modified_polyphase_time_sync_cc.py)
GR_ADD_TEST(qa_modified_costas_loop_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_modified_costas_loop_cc.py)
GR_ADD_TEST(qa_prou_sample_queue_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_prou_sample_queue_cc.py)
GR_ADD_TEST(qa_interference_canceller_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_interference_canceller_cc.py)
GR_ADD_TEST(qa_burst_tagger_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_burst_tagger_cc.py)
GR_ADD_TEST(qa_protocol_parser_b ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_protocol_parser_b.py)
GR_ADD_TEST(qa_correlate_sync_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_correlate_sync_cc.py)
GR_ADD_TEST(qa_symbol_queue_receiver_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_symbol_queue_receiver_cc.py)
GR_ADD_TEST(qa_symbol_level_ic_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_symbol_level_ic_cc.py)
GR_ADD_TEST(qa_expand_symbal_to_sample_ff ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_expand_symbal_to_sample_ff.py)
GR_ADD_TEST(qa_symbol_sync_receiver_cf ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_symbol_sync_receiver_cf.py)
GR_ADD_TEST(qa_burst_synchronizer_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_burst_synchronizer_cc.py)
GR_ADD_TEST(qa_burst_parser_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_burst_parser_cc.py)
GR_ADD_TEST(qa_packet_parser_b ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_packet_parser_b.py)
GR_ADD_TEST(qa_symbol_corrector_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_symbol_corrector_cc.py)
GR_ADD_TEST(qa_control_source_b ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_control_source_b.py)
GR_ADD_TEST(qa_packet_sink ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_packet_sink.py)
GR_ADD_TEST(qa_su_transmitter_bb ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_su_transmitter_bb.py)
GR_ADD_TEST(qa_chip_mapper_bb ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_chip_mapper_bb.py)
GR_ADD_TEST(qa_moving_average_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_moving_average_cc.py)
GR_ADD_TEST(qa_moving_average_ff ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_moving_average_ff.py)
GR_ADD_TEST(qa_coarse_sync_cc ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/qa_coarse_sync_cc.py)
