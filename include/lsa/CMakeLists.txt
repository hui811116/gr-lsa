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
    modified_costas_loop_cc.h
    correlate_sync_cc.h
    preamble_prefixer.h
    su_ctrl.h
    chip_mapper.h
    moving_average_cc.h
    moving_average_ff.h
    coarse_sync_cc.h
    prou_packet_sink_f.h
    interference_tagger_cc.h
    block_tagger_cc.h
    su_sr_transmitter_bb.h
    su_tx_helper.h
    su_packet_sink_c.h
    throughput_report.h
    stop_n_wait_tx_bb.h
    stop_n_wait_rx_ctrl_cc.h
    stop_n_wait_ack.h
    sns_tx_helper.h
    message_file_sink.h
    sfd_tagger_cc.h
    simple_tx.h
    simple_rx.h
    phy_crc.h
    byte_to_symbol_bc.h
    throughput_file_sink.h
    ic_resync_cc.h
    su_block_receiver_c.h
    ic_ncfo_cc.h
    arq_tx.h
    dump_tx.h
    burst_tagger_cc.h
    stop_n_wait_tag_gate_cc.h
    file_downloader_tx.h DESTINATION include/lsa
)
