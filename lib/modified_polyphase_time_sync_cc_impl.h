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

#ifndef INCLUDED_LSA_MODIFIED_POLYPHASE_TIME_SYNC_CC_IMPL_H
#define INCLUDED_LSA_MODIFIED_POLYPHASE_TIME_SYNC_CC_IMPL_H

#include <lsa/modified_polyphase_time_sync_cc.h>

using namespace gr::filter;

namespace gr {
  namespace lsa {

    class modified_polyphase_time_sync_cc_impl : public modified_polyphase_time_sync_cc
    {
     private:

      pmt::pmt_t d_intf_tagname;
      // Nothing to declare in this block.
      bool   d_updated;
      double d_sps;
      double d_sample_num;
      float  d_loop_bw;
      float  d_damping;
      float  d_alpha;
      float  d_beta;

      int                                  d_nfilters;
      int                                  d_taps_per_filter;
      std::vector<kernel::fir_filter_ccf*> d_filters;
      std::vector<kernel::fir_filter_ccf*> d_diff_filters;
      std::vector< std::vector<float> >    d_taps;
      std::vector< std::vector<float> >    d_dtaps;
      std::vector<float>                   d_updated_taps;

      float d_k;
      float d_rate;
      float d_rate_i;
      float d_rate_f;
      float d_max_dev;
      int   d_filtnum;
      int   d_osps;
      float d_error;
      int   d_out_idx;

      bool d_intf_state;

      uint64_t d_old_in, d_new_in, d_last_out;

      void create_diff_taps(const std::vector<float> &newtaps,
          std::vector<float> &difftaps);
      
      void set_taps(const std::vector<float> &taps,
        std::vector< std::vector<float> > &ourtaps,
        std::vector<kernel::fir_filter_ccf*> &ourfilter);

     public:
      modified_polyphase_time_sync_cc_impl(double sps, float loop_bw,
            const std::vector<float> &taps,
            unsigned int filter_size,
            float init_phase,
            float max_rate_deviation,
            int osps,
            const std::string& intf_tagname);
      ~modified_polyphase_time_sync_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_MODIFIED_POLYPHASE_TIME_SYNC_CC_IMPL_H */

