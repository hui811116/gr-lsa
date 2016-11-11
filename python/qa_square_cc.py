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
# to import array module form numpy
from numpy import array

class qa_square_cc (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
    	iphase= array([1, 2, 3, 4])
    	qphase= array([1, 2, 3 ,4])
    	# creating complex number array
    	src_data=iphase+1j*qphase
    	expected_result=(2.00,8.0,18.0,32.0)
    	src=blocks.vector_source_c(src_data)
    	sqr_cc=lsa.square_cc()
    	# the is just declaring the data type of dst
    	dst=blocks.vector_sink_f()
    	self.tb.connect(src,sqr_cc)
    	self.tb.connect(sqr_cc,dst)
        # set up fg
        self.tb.run ()
        result_data=dst.data()
        #
        self.assertFloatTuplesAlmostEqual(expected_result, result_data, 5)
        # count the differences in places
        self.assertEqual(len(expected_result), len(result_data))
        # check data


if __name__ == '__main__':
    gr_unittest.run(qa_square_cc, "qa_square_cc.xml")
