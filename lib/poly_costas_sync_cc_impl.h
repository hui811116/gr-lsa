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

#ifndef INCLUDED_LSA_POLY_COSTAS_SYNC_CC_IMPL_H
#define INCLUDED_LSA_POLY_COSTAS_SYNC_CC_IMPL_H

#include <lsa/poly_costas_sync_cc.h>
#include <gnuradio/filter/fir_filter.h>

namespace gr {
  namespace lsa {

    class poly_costas_sync_cc_impl : public poly_costas_sync_cc
    {
     private:
      // Nothing to declare in this block.

      pmt::pmt_t d_sensing_tag;
      bool d_sensing_state;
      float d_prev_plf_k;
      float d_prev_plf_rate_f;
      //float d_prev_plf_error;
      //int d_prev_plf_filtnum;

      float d_prev_cos_phase;
      float d_prev_cos_freq;
      //float d_prev_cos_error;

      gr_complex* d_time_sync_symbol;
      float* d_error;

      double d_plf_sps;
      //double d_plf_sample_num;
      float d_plf_loop_bw;
      float d_plf_damping;
      float d_plf_alpha;
      float d_plf_beta;

      int d_plf_nfilts;
      int d_plf_taps_per_filter;
      std::vector<gr::filter::kernel::fir_filter_ccf*> d_plf_filters;
      std::vector<gr::filter::kernel::fir_filter_ccf*> d_plf_diff_filters;
      std::vector< std::vector<float> > d_plf_taps;
      std::vector< std::vector<float> > d_plf_dtaps;
      float d_plf_k; // phase;
      float d_plf_rate; // samping rate;
      float d_plf_rate_i; // int part;
      float d_plf_rate_f; //fraction part;
      float d_plf_max_dev; // max deviation
      int d_plf_filtnum;
      int d_plf_osps;
      float d_plf_error;
      int d_plf_out_idx;

      void plf_create_diff_taps(
        const std::vector<float>& newtaps,
        std::vector<float>& difftaps);

      void plf_set_taps(
        const std::vector<float> &newtaps,
        std::vector< std::vector<float> >& ourtaps,
        std::vector< gr::filter::kernel::fir_filter_ccf*>& ourfilter);

      int plf_core(
        gr_complex* out,
        float* error,
        const gr_complex* in,
        int nsample,
        int& nconsume,
        std::vector<tag_t>& tags,
        std::vector<tag_t>& out_tags);

      //Costas loop
      int d_costas_order;
      float d_costas_error;
      //float d_costas_noise;
      float costas_phase_detector_2(const gr_complex& sample) const;
      float costas_phase_detector_4(const gr_complex& sample) const;
      float costas_phase_detector_8(const gr_complex& sample) const;

      int costas_core(
        gr_complex* out,
        float* error,
        const gr_complex* in,
        int nsample,
        std::vector<tag_t>& tags);
      //function pointer;
      float (poly_costas_sync_cc_impl::*d_costas_phase_detector)(const gr_complex& sample) const;


     public:
      poly_costas_sync_cc_impl(
        double plf_sps,
        float plf_loop_bw,
        const std::vector<float>& plf_taps,
        int plf_nfilts,
        float costas_loop_bw,
        int costas_order,
        const std::string& sense_tagname);
      ~poly_costas_sync_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_POLY_COSTAS_SYNC_CC_IMPL_H */

