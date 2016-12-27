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

#ifndef INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_IMPL_H
#define INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_IMPL_H

#include <lsa/su_queued_transmitter_cc.h>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    class su_queued_transmitter_cc_impl : public su_queued_transmitter_cc
    {
     private:
      // Nothing to declare in this block.
      int d_state;
      int d_max_queue_size;
      
      // for header
      uint16_t d_payload_len;
      uint16_t d_pkt_counter;  //iterator
      uint8_t d_qiter;                //iterator

      //uint16_t d_counter;
      //uint8_t d_qidx;
      uint8_t d_qsize;
            
      pmt::pmt_t d_lengthtagname;
      pmt::pmt_t d_rx_sensing_tag;
      pmt::pmt_t d_rx_index_tag;
      pmt::pmt_t d_rx_info_port;
      pmt::pmt_t d_src_id;
      // output message port
      pmt::pmt_t d_debug_port;
      pmt::pmt_t d_hdr_port;


      std::vector<pmt::pmt_t> d_rx_tag_keys;  //temporary buffer for multiple rx message
      std::vector<pmt::pmt_t> d_rx_tag_values;//temporary buffer for multiple rx message value

      std::vector<gr_complex> d_hdr_points;                   //symbol map
      std::vector<gr_complex> d_pld_points;                   //symbol map

      // sensing information 
      unsigned char* d_hdr_buffer;
      size_t d_hdr_samp_len;                                  // should be const
      //size_t d_hdr_buffer_size;
      std::vector< std::vector<gr_complex> >* d_buffer_ptr;   // queue
      std::vector<int> d_pld_len_buffer;                      // queue
      std::vector<int> d_counter_buffer;                      // queue

      std::vector<int> d_retx_counter_buffer;                 // for retransmission purpose
      //size_t d_retx_count;

      std::vector<unsigned char> d_accesscode;                 //store in bytes

      void queue_size_adapt();

      void generate_hdr(int pld_len, bool type);

      // Where all the action really happens
      void receiver_msg_handler(pmt::pmt_t rx_msg);

      void _repack(unsigned char* out,const unsigned char* in, int size, int const_m);
      void _map_sample(gr_complex* out, const unsigned char* in, int size, const std::vector<gr_complex>& mapper);

      void store_to_queue(gr_complex* samp, int pld_samp_len, int pld_bytes_len);

     protected:
        int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_queued_transmitter_cc_impl(
        int max_queue_size,
        const std::string & sensing_tag,
        const std::string & index_tag,
        const std::string & accesscode,
        const std::string & lengthtagname,
        const std::vector<gr_complex>& hdr_const_points,
        const std::vector<gr_complex>& pld_const_points);

      ~su_queued_transmitter_cc_impl();

      //void forecast (int noutput_items, gr_vector_int &ninput_items_required);


      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      bool set_accesscode(const std::string & accesscode);
      size_t header_nbits() const;
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_QUEUED_TRANSMITTER_CC_IMPL_H */

