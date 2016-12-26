#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2016 <+YOU OR YOUR COMPANY+>.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

from gnuradio import gr, gr_unittest
from gnuradio import blocks, digital
import pmt
import lsa_swig as lsa
from numpy import array
import numpy as np

class qa_header_payload_parser_cb (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()
        self.tsb_key = "length"

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        # set up fg
        accesscode = "0110110110111011"
        src_data = array([0x00,0xff,0x0f])
        src = blocks.vector_source_b(src_data)
        prefix =  lsa.su_header_prefix(accesscode,self.tsb_key,False)
        #const = digital.psk_2;
        #repack = blocks.repack_bits_bb(8,1,"",False,endianness=GR_MSB_FIRST);
        #const = .
        self.tb.run ()
        # check data


if __name__ == '__main__':
    gr_unittest.run(qa_header_payload_parser_cb, "qa_header_payload_parser_cb.xml")
