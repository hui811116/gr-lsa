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
    lsa_header_payload_parser_cb.xml
    lsa_su_header_prefix.xml
    lsa_su_sample_receiver_cb.xml
    lsa_su_transmitter_bc.xml
    lsa_prou_sample_receiver_cb.xml
    lsa_interference_energy_detector_cc.xml
    lsa_poly_costas_sync_cc.xml
    lsa_modified_polyphase_time_sync_cc.xml
    lsa_modified_costas_loop_cc.xml
    lsa_prou_sample_queue_cc.xml
    lsa_symbol_receiver_c.xml DESTINATION share/gnuradio/grc/blocks
)
