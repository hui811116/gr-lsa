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
import math

from numpy import array
class qa_eng_det_cc (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        thresh=1e-3
        bin_size=5;
    	iphase = array([1,1,1,1,1,2])
    	qphase = array([-1,1,-1,1,-1,-2])
    	src_data = iphase + 1j*qphase
    	#expected=(10.0,16.0)
    	#src = blocks.vector_source_c(src_data)
    	#eng_cc = lsa.eng_det_cc(thresh,bin_size)
    	#dst = blocks.vector_sink_f()
    	#self.tb.connect(src,eng_cc)
    	#self.tb.connect(eng_cc,dst)

        # set up fg
        self.tb.run ()
        #result_data=dst.data()
        #print(src_data)
        #print(result_data)
        # check data
        #self.assertFloatTuplesAlmostEqual(expected, result_data, 5)
        #self.assertEqual(len(expected), len(result_data))


if __name__ == '__main__':
    gr_unittest.run(qa_eng_det_cc, "qa_eng_det_cc.xml")
