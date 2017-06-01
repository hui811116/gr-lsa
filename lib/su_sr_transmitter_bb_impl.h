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
#define time_t long int
#define LSARETRYLIM 10
#define LSATIMEOUT 2*CLOCKS_PER_SEC
namespace gr {
  namespace lsa {

    class srArq_t{
      public:
      srArq_t(){ d_noseq=0; d_time = std::clock();d_retry=0;d_msg = pmt::PMT_NIL;}
      srArq_t(const srArq_t& aq){d_noseq = aq.d_noseq; d_time = aq.d_time; d_retry =aq.d_retry;d_msg = aq.d_msg;}
      srArq_t(uint16_t noseq,pmt::pmt_t msg){d_noseq = noseq; d_time = std::clock(); d_retry = 0;d_msg= msg;}
      ~srArq_t(){}
      const srArq_t& operator=(const srArq_t& aq){
        d_noseq = aq.d_noseq; d_time = aq.d_time; d_retry = aq.d_retry; d_msg=aq.d_msg;
        return *this;
      }
      const srArq_t& operator*()const{return *this;}
      time_t time()const {return d_time;}
      uint16_t seq()const {return d_noseq;}
      uint32_t retry()const{return d_retry;}
      pmt::pmt_t msg()const{return d_msg;}
      bool inc_retry(){d_retry++; return d_retry>LSARETRYLIM;}
      bool timeout(){return (std::clock()-d_time) >=LSATIMEOUT;}
      void reset(){d_retry = 0; d_time=std::clock();}
      void update_time(){d_time = std::clock();}
      void set_retry(uint32_t re){d_retry = re;}
      void set_time(time_t time){d_time = time;}
      void set_seq(uint16_t seq){d_noseq = seq;}
      void set_msg(pmt::pmt_t msg){d_msg = msg;}
      size_t blob_length(){return pmt::blob_length(d_msg);}
      private:
        uint16_t d_noseq;
        time_t d_time;
        uint32_t d_retry;
        pmt::pmt_t d_msg;
    };

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
      bool d_debug;

      // thread functions for d_arq_queue;
      void clear_queue();
      void enqueue(const srArq_t& arq);
      bool dequeue(int seq);
      pmt::pmt_t check_timeout();
      
      // thread functions for retransmission
      bool create_retx_queue();
      pmt::pmt_t get_retx(int idx);

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_sr_transmitter_bb_impl(const std::string& tagname, bool debug);
      ~su_sr_transmitter_bb_impl();

      void msg_in(pmt::pmt_t);
      // Where all the action really happens
      int work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
    };

  } // namespace lsa
} // namespace gr

#endif /* INCLUDED_LSA_SU_SR_TRANSMITTER_BB_IMPL_H */

