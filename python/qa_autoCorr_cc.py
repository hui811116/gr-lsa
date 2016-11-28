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

class qa_autoCorr_cc (gr_unittest.TestCase):

    def setUp (self):
        self.tb = gr.top_block ()

    def tearDown (self):
        self.tb = None

    def test_001_t (self):
        # set up fg
        iphase=array([1,2,3,4,5,6,7,8])
        qphase=array([1,2,3,4,5,6,7,8])
        expect=array([22+0j,46+0j,78+0j,118+0j,166+0j])
        src_data=iphase+1j*qphase
        src=blocks.vector_source_c(src_data)
        atc_cc=lsa.autoCorr_cc(2)
        dst=blocks.vector_sink_c()
        self.tb.connect(src,atc_cc)
        self.tb.connect(atc_cc,dst)

        self.tb.run ()
        result=dst.data()
        self.assertComplexTuplesAlmostEqual(expect,result,7)
        # print(result)
        # check data


if __name__ == '__main__':
    gr_unittest.run(qa_autoCorr_cc, "qa_autoCorr_cc.xml")
