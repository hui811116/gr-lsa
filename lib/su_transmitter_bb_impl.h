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
#include <list>

namespace gr {
  namespace lsa {

    // helper class: an object to store a block of message
    class myBlock_t{
      public:
      friend class su_transmitter_bb_impl;
      myBlock_t(){}
      myBlock_t(const myBlock_t& myblock){
        mem_addr = myblock.mem_addr;
        block_len = myblock.block_len;
        base_point = myblock.base_point;
      }
      ~myBlock_t(){}
      const myBlock_t& operator = (const myBlock_t& block){
        mem_addr = block.mem_addr;
        block_len = block.block_len;
        base_point = block.base_point;
        return *this;
      }
      int mem_addr;
      int block_len;
      unsigned int base_point;
    };
    // helper class: an object to store ACK information
    class myACK_t{
      public:
      friend class su_transmitter_bb_impl;
      myACK_t(){}
      myACK_t(const myACK_t& ack){
        for(int i=0;i<64;++i){
          ack_table[i] = ack.ack_table[i];
        }
        ack_cnt = ack.ack_cnt;
        npkt = ack.npkt;
        base_point = ack.base_point;
      }
      ~myACK_t(){}
      const myACK_t& operator =(const myACK_t& ack)
      {
        for(int i=0;i<64;++i){
          ack_table[i] = ack.ack_table[i];
        }
        ack_cnt = ack.ack_cnt;
        npkt = ack.npkt;
        base_point = ack.base_point;
        return *this;
      }
      bool ack_table[64];
      int ack_cnt;
      int npkt;
      unsigned int base_point;
    };
    // main class
    class su_transmitter_bb_impl : public su_transmitter_bb
    {
     private:
      const pmt::pmt_t d_msg_in;               // message port tagname
      const pmt::pmt_t d_lentag;               // length tag for tagged stream block
      unsigned char d_mem_pool[256][64][256];  // 8M memory
      unsigned char d_buf[256];                // temporary buffer to add preamble
      bool d_debug;                            // debugging switch
      const int d_pkt_cap;                     // maximum capacity of packets
      const int d_block_cap;                   // maximum capacity of a block, wrap around if pass the cap
      int d_pkt_circ_cnt;                      // used to recored the memory location
      unsigned int d_base_cnt;                 // transmitter base count
      int d_base_rx;                           // acked base, note: initialize to -1
      int d_bytes_per_packet;                  // bytes per packet
      std::list<myBlock_t> d_blist;            // list for block
      std::list<myACK_t> d_acklist;            // list for acks
      unsigned char d_prev_block[64][256];     // previous block in case retransmission required
      int d_prev_npkt;                         // previous number of packets

      // helper functions
      void msg_in(pmt::pmt_t msg);
      void insert_block(const unsigned char* in, int nin);
      int output_block(unsigned char* out,int noutput_items);
      void parse_pdu(int& qidx,int& qsize,unsigned int& base,const uint8_t* uvec);
      void record_block(int cir_mem_idx, int npkt);

     protected:
      int calculate_output_stream_length(const gr_vector_int &ninput_items);

     public:
      su_transmitter_bb_impl(const std::string& tagname, int bytes_per_packet, bool debug);
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

