/* -*- c++ -*- */
/* 
 * Copyright 2017 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_LSA_INTERFERENCE_ENERGY_DETECTOR_CC_IMPL_H
#define INCLUDED_LSA_INTERFERENCE_ENERGY_DETECTOR_CC_IMPL_H

#include <lsa/interference_energy_detector_cc.h>

namespace gr {
  namespace lsa {

    class interference_energy_detector_cc_impl : public interference_energy_detector_cc
    {
     private:
      // Nothing to declare in this block.
      float* d_energy_reg;
      float* v_stddev;
      float* v_mean;
      int d_blocklength;

      float d_voe_thres;
      pmt::pmt_t d_src_id;
      const pmt::pmt_t d_voe_tagname;

      bool d_debug;
      bool d_state_voe;

     public:
      interference_energy_detector_cc_impl(
        int blocklength,
        bool debug);
      ~interference_energy_detector_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      void set_blocklength(int blocklength);
      int blocklength() const;
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_INTERFERENCE_ENERGY_DETECTOR_CC_IMPL_H */

