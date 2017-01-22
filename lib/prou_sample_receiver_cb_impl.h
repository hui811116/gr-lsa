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
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    class prou_sample_receiver_cb_impl : public prou_sample_receiver_cb
    {
     private:
      // output buffer
      gr_complex* d_output_buffer;
      size_t d_output_buffer_cap;
      size_t d_output_buffer_size;
      size_t d_output_buffer_idx;

      // state registers
      int d_mode;
      int d_state;
      int d_intf_state;

      // message port name
      pmt::pmt_t d_debug_port;
      bool d_debug;
      
      // sample or symbol buffers

      std::vector< std::vector<gr_complex> >* d_sig_buffer;
      gr_complex* d_process_buffer;
      size_t d_process_cap;
      size_t d_cap_init;
      size_t d_process_size;
      size_t d_process_idx;

      // interference cancellation mode: su synchronizer
      gr::digital::constellation_sptr d_su_hdr_const;
      uint64_t d_su_sync_reg;
      size_t d_su_code_len;
      size_t d_su_hdr_bits_len;
      uint64_t d_su_accesscode;
      uint64_t d_su_code_mask;
      int d_su_bps;
      int d_su_pld_bps;
      int d_su_pld_counter;

      size_t d_su_pkt_begin;
      std::vector<unsigned char> d_su_bit_input;
      
      //Volk memory for variance of energy calculation
      float * d_var_eng_buffer;
      float * d_eng_buffer;

      uint8_t d_qidx;
      uint8_t d_qsize;
      uint16_t d_pld_len;

      std::vector<int> d_retx_buf_idx;
      std::vector<int> d_retx_pkt_len;
      int d_retx_count;

      std::vector<int> d_cei_pkt_len;
      std::vector<int> d_cei_buf_idx;
      std::vector<int> d_cei_qidx;

      // interference cancellation mode: helper funtions
      bool intf_decision_maker();
      void reset_intf_reg();

      bool calc_var_energy(const gr_complex* array, size_t length, float threshold_db, int bin);
      void calc_cei_all();

      void update_retx_info(bool test_voe);
      void do_interference_cancellation();

      // buffer helper functions
      void reduce_sample(int nleft);
      void double_cap();
      void reset_buffer();

      bool append_samples (const gr_complex* in, int size, int& consume);


      //su sync helper functions
      bool process_symbols ();
      bool parse_su_header(uint8_t& qidx, uint8_t& qsize, uint16_t& pld_len, const std::vector<unsigned char>& input);
      uint16_t _get_bit16(int begin_idx,const std::vector<unsigned char>& input);
      uint8_t _get_bit8(int begin_idx, const std::vector<unsigned char>& input);

      //output buffer helper function
      int output_samples(gr_complex* out, int noutput_items);
      void extract_samples_ed(std::vector<gr_complex>& out, double ed_db);
      void _out_buffer_double_cap();
      void _out_buffer_reset_cap();

      //su sync---polyphase clock sync
      double d_plf_sps;
      double d_plf_sample_num;
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
      float d_plf_k;  //what
      float d_plf_rate; //what
      float d_plf_rate_i; // what
      float d_plf_rate_f; // what
      float d_plf_max_dev; // what
      int d_plf_filtnum;
      int d_plf_osps;
      float d_plf_error;
      int d_plf_out_idx;
      // copy from original polyphase filter, seems like using these vars to handle tags
      //uint64_t d_plf_old_in, d_plf_new_in, d_pld_last_out;

      void plf_create_diff_taps(
        const std::vector<float> &newtaps,
        std::vector<float> &difftaps);

      void plf_set_taps(
        const std::vector<float> &newtaps,
        std::vector< std::vector<float> > &ourtaps,
        std::vector< gr::filter::kernel::fir_filter_ccf*> &ourfilter);      
      
      int plf_core(
        gr_complex* out,
        float* error,
        const gr_complex* in,
        int nsample,
        int& nconsume);
      //su sync---costas loop
      int d_costas_order;
      float d_costas_error;
      float d_costas_noise;
      float d_costas_loop_bw;

      float phase_detector_8(gr_complex sample) const;
      float phase_detector_4(gr_complex sample) const;
      float phase_detector_2(gr_complex sample) const;

      float (prou_sample_receiver_cb_impl::*d_costas_phase_detector)(gr_complex sample) const;
      int costas_core(
        gr_complex* out,
        float* error_ang,
        int& out_count,
        const gr_complex* in,
        int nsample);

     public:
      prou_sample_receiver_cb_impl(
        const gr::digital::constellation_sptr& su_hdr_const,
        int su_pld_bps,
        const std::string& su_accesscode,
        double d_plf_sps,
        float d_plf_loop_bw,
        const std::vector<float>& plf_taps,
        int su_nfilts,
        float costas_loop_bw,
        int costas_order,
        bool mode,
        bool debug);
      ~prou_sample_receiver_cb_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      //set functions;
      bool set_su_accesscode(const std::string& su_accesscode);
      //get functinos;
      uint64_t su_accesscode() const;

    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_PROU_SAMPLE_RECEIVER_CB_IMPL_H */

