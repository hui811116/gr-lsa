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

#ifndef INCLUDED_LSA_COARSE_SYNC_CC_IMPL_H
#define INCLUDED_LSA_COARSE_SYNC_CC_IMPL_H

#include <lsa/coarse_sync_cc.h>

namespace gr {
  namespace lsa {

    class coarse_sync_cc_impl : public coarse_sync_cc
    {
     private:
      //int d_state;
      int d_auto_cnt;
      int d_copy_cnt;
      int d_delay;
      float d_threshold;
      float d_phase;
      const int d_valid_len;
      const int d_mingap;
      const int d_maxlen;
      const pmt::pmt_t d_cfo_key;
      float d_coarse_cfo;
      uint32_t d_voe_duration_cnt;
      bool d_voe_state;
      std::vector<tag_t> d_tags;

      void update_tag_state(int idx);

     public:
      coarse_sync_cc_impl(float threshold, int delay);
      ~coarse_sync_cc_impl();

      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_COARSE_SYNC_CC_IMPL_H */

