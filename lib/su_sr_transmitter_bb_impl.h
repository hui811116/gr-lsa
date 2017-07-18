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

#ifndef INCLUDED_LSA_SU_SR_TRANSMITTER_BB_IMPL_H
#define INCLUDED_LSA_SU_SR_TRANSMITTER_BB_IMPL_H

#include <lsa/su_sr_transmitter_bb.h>
#include <ctime>
#include "utils.h"
#include <fstream>

namespace gr {
  namespace lsa {

    class su_sr_transmitter_bb_impl : public su_sr_transmitter_bb
    {
     private:
      gr::thread::mutex d_mutex;
      std::list<srArq_t> d_arq_queue;
      std::vector<srArq_t> d_retx_queue;
      std::vector<bool> d_retx_table;
      uint16_t d_retx_cnt;
      uint16_t d_retx_idx;
      uint16_t d_retx_size;
      uint16_t d_seq;
      const std::string& d_tagname;      
      const pmt::pmt_t d_msg_in;
      bool d_prou_present;
      unsigned char d_buf[1024];
      bool d_usef;
      std::fstream d_file;
      long int d_pkt_success_cnt;
      long int d_pkt_failed_cnt;
      bool d_verb;
      boost::shared_ptr<gr::thread::thread> d_thread;
      boost::posix_time::ptime d_start_time;
      bool d_finished;

      // thread functions for d_arq_queue;
      void clear_queue();
      void enqueue(const srArq_t& arq);
      bool dequeue(int seq);
      pmt::pmt_t check_timeout();
      bool peek_front(int& len);
      // thread functions for retransmission
      bool create_retx_queue();
      pmt::pmt_t get_retx(int idx);
      bool retx_peek_front(int& len);
      bool check_retx_table(int idx);
      bool update_retx_table(int idx);
      void reset_retx_retry();

      // file operation
      std::vector< std::vector<unsigned char> > d_data_src;
      bool read_data(const std::string& filename);
      void run();
     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_sr_transmitter_bb_impl(const std::string& tagname, const std::string& filename, bool usef, bool verb);
      ~su_sr_transmitter_bb_impl();

      void msg_in(pmt::pmt_t);
      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
      bool start();
      bool stop();
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_SR_TRANSMITTER_BB_IMPL_H */

