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
from gnuradio import blocks
import lsa_swig as lsa
from numpy import array

class qa_lsa_queue_cc (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        # set up fg
        cap=4
        iphase=array(range(20))
        qphase=array(range(20))
        src_data=iphase + 1j*qphase
        expected=array(range(20))
        src = blocks.vector_source_c(src_data)
        queue_cc=lsa.lsa_queue_cc(cap)
        #queue_cc.set_status(False)
        dst=blocks.vector_sink_c()
        self.tb.connect(src,queue_cc)
        self.tb.connect(queue_cc,dst)
        self.tb.run ()
        # check data
        result_data=dst.data()
        #print(src_data)
        print(result_data)
        self.assertEqual(len(expected),len(result_data))


if __name__ == '__main__':
    gr_unittest.run(qa_lsa_queue_cc, "qa_lsa_queue_cc.xml")
