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

#ifndef INCLUDED_LSA_INTERFERENCE_TAGGER_CC_IMPL_H
#define INCLUDED_LSA_INTERFERENCE_TAGGER_CC_IMPL_H

#include <lsa/interference_tagger_cc.h>

namespace gr {
  namespace lsa {

    class interference_tagger_cc_impl : public interference_tagger_cc
    {
     private:
      const pmt::pmt_t d_msg_port;
      float d_voe_thres;

      bool d_intf_state;
      int d_intf_cnt;
      uint64_t d_duration_cnt;
      
      void report_interference();

     public:
      interference_tagger_cc_impl(const float& voe_thres);
      ~interference_tagger_cc_impl();

      void set_voe_threshold(const float& voe_thres);
      float voe_threshold()const;

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_INTERFERENCE_TAGGER_CC_IMPL_H */

