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

#ifndef INCLUDED_LSA_SU_TRANSMITTER_BB_IMPL_H
#define INCLUDED_LSA_SU_TRANSMITTER_BB_IMPL_H

#include <lsa/su_transmitter_bb.h>

namespace gr {
  namespace lsa {

    class su_transmitter_bb_impl : public su_transmitter_bb
    {
     private:
      pmt::pmt_t d_msg_in;
      pmt::pmt_t d_lentag;
      unsigned char d_buf[256];     // temporary buffer for bytes
      int d_mode;                   // SU transmission schemes
      bool d_debug;
      int d_state;                  // SU TX state 
      
      long int d_current_time;      // naive idea, use time to track ack
      long int d_update_time;       // the time of latest feedback

      std::vector<unsigned char*> d_queue_buf; // pkt queue
      std::vector<size_t> d_pkt_len_buf;
      std::vector<long int> d_time_buf;
      int d_latest_idx;             // use this to save searching time
      int d_current_idx;            // current index of buf
      const int d_queue_cap;        // maximum capacity of the queue, wrap around if pass the cap

      std::vector<int> d_retx_idx_buf; // use for tracking retransmission status
      int d_retx_cnt;
      int d_retx_idx;

      void msg_in(pmt::pmt_t msg);
      void prepare_retx();
      void reset_queue();

      // statistic fields
      // long int d_transmit_pkts;
      // long int d_round_pkts;

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_transmitter_bb_impl(const std::string& tagname, int mode, bool debug);
      ~su_transmitter_bb_impl();

      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_TRANSMITTER_BB_IMPL_H */

