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

#ifndef INCLUDED_LSA_SYMBOL_RECEIVER_C_IMPL_H
#define INCLUDED_LSA_SYMBOL_RECEIVER_C_IMPL_H

#include <lsa/symbol_receiver_c.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class symbol_receiver_c_impl : public symbol_receiver_c
    {
     private:
      // Nothing to declare in this block.
      bool d_debug;
      bool d_queue;
      pmt::pmt_t d_sensing_tagname;
      pmt::pmt_t d_hdr_port;
      pmt::pmt_t d_pkt_port;

      pmt::pmt_t d_pdu_vector;

      unsigned char* d_byte_reg;
      unsigned char* d_symbol_to_bytes;
      int d_byte_count;

      std::vector<unsigned char> d_input_bits;
      int d_state;
      int d_sps;
      int d_hdr_bps;
      int d_pld_bps;

      unsigned long long d_data_reg;
      unsigned long long d_mask;

      uint64_t d_accesscode;
      size_t d_accesscode_len;

      int d_payload_len;
      int d_counter;
      int d_qidx;
      int d_qsize;

      gr::digital::constellation_sptr d_hdr_const;
      gr::digital::constellation_sptr d_pld_const;

      std::vector<int> d_hdr_map;
      std::vector<int> d_pld_map;

      float d_corr_phase;

      bool insert_symbol(const gr_complex& symbol);
      void pub_byte_pkt();
      size_t header_nbits() const;
      bool set_accesscode(const std::string& accesscode);
      uint64_t accesscode()const;
      uint16_t _get_bit16(int begin_idx);
      uint8_t _get_bit8(int begin_idx);
      bool parse_header();
      void feedback_info(bool type, 
        const float* plf_freq, 
        const float* plf_phase,
        const float* cos_freq,
        const float* cos_phase,
        int index,
        bool have_sync);

      //feedback to queue mechanism
      long int d_prev_time;
      long int d_current_time;
      pmt::pmt_t d_time_tagname;
      int d_symbol_count;

     public:
      symbol_receiver_c_impl(const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        const std::string& sensing_tagname,
        int sps,
        bool queue_mode,
        bool debug);
      ~symbol_receiver_c_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SYMBOL_RECEIVER_C_IMPL_H */

