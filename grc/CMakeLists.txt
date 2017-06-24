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
install(FILES
    lsa_eng_det_cc.xml
    lsa_interference_energy_detector_cc.xml
    lsa_modified_polyphase_time_sync_cc.xml
    lsa_modified_costas_loop_cc.xml
    lsa_burst_tagger_cc.xml
    lsa_correlate_sync_cc.xml
    lsa_expand_symbol_to_sample_ff.xml
    lsa_symbol_sync_receiver_cf.xml
    lsa_preamble_prefixer.xml
    lsa_su_ctrl.xml
    lsa_chip_mapper.xml
    lsa_packet_sink.xml
    lsa_chip_mapper_bb.xml
    lsa_moving_average_cc.xml
    lsa_moving_average_ff.xml
    lsa_coarse_sync_cc.xml
    lsa_stat_report.xml
    lsa_prou_packet_sink_f.xml
    lsa_interference_tagger_cc.xml
    lsa_block_tagger_cc.xml
    lsa_su_sr_transmitter_bb.xml
    lsa_su_tx_helper.xml
    lsa_ic_critical_cc.xml
    lsa_su_packet_sink_c.xml
    lsa_throughput_report.xml
    lsa_sinr_helper.xml
    lsa_stop_n_wait_tx_bb.xml
    lsa_stop_n_wait_rx_ctrl_cc.xml
    lsa_stop_n_wait_ack.xml
    lsa_sns_tx_helper.xml
    lsa_burst_gate_cc.xml
    lsa_message_file_sink.xml
    lsa_sfd_tagger_cc.xml
    lsa_simple_tx.xml
    lsa_simple_rx.xml
    lsa_phy_crc.xml
    lsa_signal_power_tagger_cc.xml DESTINATION share/gnuradio/grc/blocks
)
