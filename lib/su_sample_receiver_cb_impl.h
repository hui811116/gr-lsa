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

#ifndef INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_IMPL_H
#define INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_IMPL_H

#include <lsa/su_sample_receiver_cb.h>
//#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/digital/constellation.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class su_sample_receiver_cb_impl : public su_sample_receiver_cb
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_src_id;
      pmt::pmt_t d_sensing_tag_id;
      pmt::pmt_t d_msg_port;
      pmt::pmt_t d_pkt_port;
      pmt::pmt_t d_debug_port;  //debug
      
      pmt::pmt_t d_pdu_vector;

      gr::digital::constellation_sptr d_hdr_sptr;
      gr::digital::constellation_sptr d_pld_sptr;

      int d_state;

      unsigned char* d_byte_reg;
      unsigned char* d_symbol_to_bytes;
      //gr_complex  d_samp_reg;
      size_t d_cap;
      size_t d_byte_count;
      int d_hdr_bps;
      int d_pld_bps;

      // used for accesscode sync
      std::vector<bool> d_input;
      unsigned long long d_data_reg;
      unsigned long long d_mask;

      uint64_t d_accesscode;
      size_t d_accesscode_len;

      int d_bit_state;
      uint16_t d_payload_len;
      uint16_t d_counter;
      uint8_t d_qidx;
      uint8_t d_qsize;

      //debug
      bool d_debug;
      
      bool parse_header();
      uint16_t _get_bit16(int begin_idx);
      uint8_t _get_bit8(int begin_idx);

      void pub_byte_pkt();

      void feedback_info(bool type);
      void data_reg_reset();

      bool insert_parse_byte(const gr_complex& sample);

      void check_tags(std::vector<tag_t>& out_tags, const std::vector<tag_t>& in_tags);

     public:
      su_sample_receiver_cb_impl(
        const std::string& sensing_tag_id,
        const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        bool debug);
      ~su_sample_receiver_cb_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      size_t header_nbits() const;

      bool set_accesscode(const std::string& accesscode);

      uint64_t accesscode() const;

    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_SAMPLE_RECEIVER_CB_IMPL_H */

