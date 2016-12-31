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

#ifndef INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_IMPL_H
#define INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_IMPL_H

#include <lsa/prou_sample_receiver_cb.h>
#include <gnuradio/filter/fir_filter.h>

namespace gr {
  namespace lsa {

    class prou_sample_receiver_cb_impl : public prou_sample_receiver_cb
    {
     private:
      // Nothing to declare in this block.
      // filter for proU signal
      std::vector<gr::filter::kernel::fir_filter_ccf*> d_pu_filters;
      int d_pu_nfilters;
      std::vector< std::vector<float> > d_pu_taps;
      float d_pu_loop_bw;
      double d_pu_sps;


      // filter for su signal
      std::vector<gr::filter::kernel::fir_filter_ccf*> d_su_filters;
      int d_su_nfilters;
      std::vector< std::vector<float> > d_su_taps;
      float d_su_loop_bw;
      double d_su_sps;


     public:
      prou_sample_receiver_cb_impl(
        int pu_nfilts,
        int su_nfilts);
      ~prou_sample_receiver_cb_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_IMPL_H */

