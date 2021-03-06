/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_LSA_ENG_DET_CC_IMPL_H
#define INCLUDED_LSA_ENG_DET_CC_IMPL_H

#include <lsa/eng_det_cc.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class eng_det_cc_impl : public eng_det_cc
    {
     private:
      // Nothing to declare in this block.
      double d_threshold;
      const pmt::pmt_t d_ed_tagname;
      const pmt::pmt_t d_src_id;
      bool d_state_reg;
      bool d_tag_power;
      int d_ed_cnt;
      int d_burst_cnt;
      
      double d_eng_acc;

     public:
      eng_det_cc_impl(float threshold, bool tag_power);
      ~eng_det_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      void set_threshold(float thres_db);
      float threshold() const;

    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_ENG_DET_CC_IMPL_H */

