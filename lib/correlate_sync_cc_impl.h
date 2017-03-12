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

#ifndef INCLUDED_LSA_CORRELATE_SYNC_CC_IMPL_H
#define INCLUDED_LSA_CORRELATE_SYNC_CC_IMPL_H

#include <lsa/correlate_sync_cc.h>
#include <gnuradio/filter/fft_filter.h>

using namespace gr::filter;

namespace gr {
  namespace lsa {

    class correlate_sync_cc_impl : public correlate_sync_cc
    {
     private:
      // Nothing to declare in this block.
      kernel::fft_filter_ccc *d_filter;

      gr_complex *d_corr;
      float *d_corr_mag;
      float *d_eng;
      float *d_norm_corr;
      float d_samples_eng;
      std::vector<gr_complex> d_samples;

      float d_threshold;

      float d_thres_log;
      float d_eng_log;
     public:
      correlate_sync_cc_impl(const std::vector<gr_complex>& samples,
        float threshold);
      ~correlate_sync_cc_impl();

      void set_threshold(float thres);

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_CORRELATE_SYNC_CC_IMPL_H */

