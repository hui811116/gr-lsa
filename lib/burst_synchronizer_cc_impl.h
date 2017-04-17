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

#ifndef INCLUDED_LSA_BURST_SYNCHRONIZER_CC_IMPL_H
#define INCLUDED_LSA_BURST_SYNCHRONIZER_CC_IMPL_H

#include <lsa/burst_synchronizer_cc.h>
#include <gnuradio/fft/fft.h>

namespace gr {
  namespace lsa {

    class burst_synchronizer_cc_impl : public burst_synchronizer_cc
    {
     private:
      // integrate FFT modules
      gr::fft::fft_complex* d_fft;
      std::vector<float> d_window;
      // other members
      gr_complex* d_sample_buffer;
      gr_complex* d_fft_out;
      // squaring loop
      gr_complex* d_in_pwr;
      // decimation filter
      std::vector<float> d_taps;
      float* d_volk_taps;
      unsigned int d_ntaps;
      gr_complex* d_dec_out;
      
      // clock recovery
      float d_mu;
      float d_omega;
      float d_omega_mid;
      float d_gain_mu;
      float d_gain_omega;
      float d_omega_relative_limit;

      gr_complex d_p_2T, d_p_1T, d_p_0T, d_c_2T, d_c_1T, d_c_0T;
      gr_complex * d_interp_out;
      unsigned int d_interp_size;

      const unsigned int d_cap;
      unsigned int d_samp_size;
      int d_min_len;
      int d_sps;
      int d_decimate;
      int d_arity;
      bool d_state;
      int d_burst_status;

      int d_out_counter;

      gr_complex interp_3(const gr_complex* in, const float& mu);
      void mm_time_recovery(gr_complex* out, const gr_complex* in, int size);
      
      float coarse_cfo_estimation(const gr_complex* in, int input_data_size, gr_complex* fft_out);
      void squaring_core(const gr_complex* in, int size);
      void decimation_filter(gr_complex* out, const gr_complex* in, int size);
     public:
      burst_synchronizer_cc_impl(int min_len, int sps, 
      const std::vector<float>& window, int arity,
      const std::vector<float>& taps, int decimate, float loop_bw);
      ~burst_synchronizer_cc_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_BURST_SYNCHRONIZER_CC_IMPL_H */

