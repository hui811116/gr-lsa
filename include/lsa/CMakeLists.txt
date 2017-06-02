# Copyright 2011,2012 Free Software Foundation, Inc.
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
# Install public header files
########################################################################
install(FILES
    api.h
    eng_det_cc.h
    interference_energy_detector_cc.h
    modified_polyphase_time_sync_cc.h
    modified_costas_loop_cc.h
    prou_sample_queue_cc.h
    interference_canceller_cc.h
    burst_tagger_cc.h
    protocol_parser_b.h
    correlate_sync_cc.h
    expand_symbal_to_sample_ff.h
    symbol_sync_receiver_cf.h
    preamble_prefixer.h
    symbol_corrector_cc.h
    control_source_b.h
    app_simple.h
    block_mac.h
    su_ctrl.h
    chip_mapper.h
    packet_sink.h
    su_transmitter_bb.h
    chip_mapper_bb.h
    moving_average_cc.h
    moving_average_ff.h
    coarse_sync_cc.h
    stat_report.h
    prou_packet_sink_f.h
    interference_tagger_cc.h
    su_mac.h
    prou_ring_queue_cc.h
    interference_cancellation_core_cc.h
    block_tagger_cc.h
    su_sr_transmitter_bb.h
    su_tx_helper.h
    ic_critical_cc.h DESTINATION include/lsa
)
