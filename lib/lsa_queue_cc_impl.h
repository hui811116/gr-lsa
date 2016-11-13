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

#ifndef INCLUDED_LSA_LSA_QUEUE_CC_IMPL_H
#define INCLUDED_LSA_LSA_QUEUE_CC_IMPL_H

#include <lsa/lsa_queue_cc.h>
#include <vector>
#include <complex>

namespace gr {
  namespace lsa {

    class lsa_queue_cc_impl : public lsa_queue_cc
    {
     private:
      //int d_queue_size;
      int d_queue_capacity;
      // testing status TODO 
      bool d_status;
      std::vector<std::complex<float> >* d_q_ptr;
      //std::vector<std::complex<float>>* d_q_pending_ptr;
      // Nothing to declare in this block.
      //void flush_queue();
      void dequeue_n(int nSample);
      void enqueue(const std::complex<float>* in,int nin);
      bool check_status();

     public:
      lsa_queue_cc_impl();
      ~lsa_queue_cc_impl();

      // Where all the action really happens
      //void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
      size_t get_queue_size() const;
      void set_capacity(int capacity);
      void set_status(bool new_sta);

    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_LSA_QUEUE_CC_IMPL_H */

