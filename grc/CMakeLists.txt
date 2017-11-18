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
    lsa_modified_costas_loop_cc.xml
    lsa_correlate_sync_cc.xml
    lsa_preamble_prefixer.xml
    lsa_su_ctrl.xml
    lsa_chip_mapper.xml
    lsa_moving_average_cc.xml
    lsa_moving_average_ff.xml
    lsa_coarse_sync_cc.xml
    lsa_prou_packet_sink_f.xml
    lsa_interference_tagger_cc.xml
    lsa_block_tagger_cc.xml
    lsa_su_sr_transmitter_bb.xml
    lsa_su_tx_helper.xml
    lsa_su_packet_sink_c.xml
    lsa_throughput_report.xml
    lsa_stop_n_wait_tx_bb.xml
    lsa_stop_n_wait_rx_ctrl_cc.xml
    lsa_stop_n_wait_ack.xml
    lsa_sns_tx_helper.xml
    lsa_message_file_sink.xml
    lsa_sfd_tagger_cc.xml
    lsa_simple_tx.xml
    lsa_simple_rx.xml
    lsa_phy_crc.xml
    lsa_byte_to_symbol_bc.xml
    lsa_throughput_file_sink.xml
    lsa_ic_resync_cc.xml
    lsa_su_block_receiver_c.xml
    lsa_ic_ncfo_cc.xml
    lsa_arq_tx.xml
    lsa_dump_tx.xml
    lsa_burst_tagger_cc.xml
    lsa_stop_n_wait_tag_gate_cc.xml
    lsa_file_downloader_tx.xml
    lsa_su_pwrCtrl_stream_tx_bb.xml
    lsa_pwr_rx_state_ctrl_cc.xml
    lsa_pwr_sense_tx_bb.xml
    lsa_pwr_ctrl.xml DESTINATION share/gnuradio/grc/blocks
)
