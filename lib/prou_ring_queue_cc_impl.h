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

#ifndef INCLUDED_LSA_PROU_RING_QUEUE_CC_IMPL_H
#define INCLUDED_LSA_PROU_RING_QUEUE_CC_IMPL_H

#include <lsa/prou_ring_queue_cc.h>

namespace gr {
  namespace lsa {

    class prou_ring_queue_cc_impl : public prou_ring_queue_cc
    {
     private:
      gr_complex * d_ring_mem;
      const int d_mem_cap;
      int d_ring_idx;
      int d_ring_cnt;
      int d_state;
      int d_cpy_cnt;
      int d_voe_cnt;
      float d_voe_thres;

      void enter_search();
      void enter_ring();
      void enter_copy();

      bool d_debug;
      uint32_t d_debug_cnt;

     public:
      prou_ring_queue_cc_impl(float thres, bool debug);
      ~prou_ring_queue_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
      float voe_threshold()const {return d_voe_thres;}
      void set_voe_threshold(float voe_thres){d_voe_thres = voe_thres;}
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROU_RING_QUEUE_CC_IMPL_H */

