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

      // state registers
      int d_mode;
      int d_state;
      int d_intf_state;
      //int d_output_buffer_state;
      //int d_intf_sync_state;

      // message port name
      pmt::pmt_t d_debug_port;
      
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

      bool append_samples (const gr_complex* in, int size);


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

     public:
      prou_sample_receiver_cb_impl(
        const gr::digital::constellation_sptr& su_hdr_const,
        int su_pld_bps,
        int pu_nfilts,
        int su_nfilts,
        bool mode);
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

