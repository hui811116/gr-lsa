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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "su_transmitter_bb_impl.h"

namespace gr {
  namespace lsa {

    // LSA SU MAC HEADER DEFINITION
    // -------------------------------------
    //              BIT FIELD
    // -------------------------------------
    // queue idx| queue size| base point   |
    //    1byte |    1byte  |    4bytes
    // -------------------------------------
    // STATES: 1) CLEAR_TO_SEND, 2) RETRANSMISSION
    // BEHAVIOR:     queue size =0,    queue_size = buf size

    // for simplicity, PHY preamble and SFD are also prefixed on received stream
    // PHY FIELD
    // ------------------------------------
    // preamble| signal frame dilimeter| pkt length|
    //  4 bytes|        1 byte         |   1 byte

#define DEBUG d_debug && std::cerr

    static const int MAX_PLD_LEN = 125;  //(127-2);
    static const unsigned char LSAPHYHDR[] ={0x00,0x00,0x00,0x00,0xE6,0x00}; // last bytes for length field
    static const unsigned char LSAMACHDR[] ={0x00,0x00,0x00,0x00,0x00,0x00}; // qidx,qsize,base x 4
    // the last feild is for packet length
    static const int PHYLEN = 6;
    static const int MACLEN = 6;
    

    su_transmitter_bb::sptr
    su_transmitter_bb::make(const std::string& tagname,int bytes_per_packet, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new su_transmitter_bb_impl(tagname,bytes_per_packet,debug));
    }

    /*
     * The private constructor
     */
    su_transmitter_bb_impl::su_transmitter_bb_impl(
      const std::string& tagname, int bytes_per_packet,bool debug)
      : gr::tagged_stream_block("su_transmitter_bb",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(char)), tagname),
              d_lentag(pmt::intern(tagname)),
              d_block_cap(64),
              d_pkt_cap(256),
              d_msg_in(pmt::mp("msg_in"))
    {
      message_port_register_in(d_msg_in);
      set_msg_handler(d_msg_in,boost::bind(&su_transmitter_bb_impl::msg_in,this,_1));

      if(bytes_per_packet<0 || bytes_per_packet>125){
        throw std::invalid_argument("invalid bytes per packet");
      }
      d_bytes_per_packet = bytes_per_packet;
      d_debug = debug;
      d_pkt_circ_cnt = 0;
      d_base_cnt=0;
      d_base_rx = -1;
    }

    /*
     * Our virtual destructor.
     */
    su_transmitter_bb_impl::~su_transmitter_bb_impl()
    {
      
    }

    int
    su_transmitter_bb_impl::calculate_output_stream_length(const gr_vector_int &ninput_items)
    {
      int nopkt;
      myBlock_t* bptr = &d_blist.front();
      if(!d_blist.empty()){
        nopkt = bptr->block_len();
      }
      else{
        nopkt =ninput_items[0]/d_bytes_per_packet;
      }
      return nopkt*(d_bytes_per_packet+MACLEN+PHYLEN);
    }

    void
    su_transmitter_bb_impl::msg_in(pmt::pmt_t msg)
    {
      assert(pmt::is_pair(msg));
      pmt::pmt_t k = pmt::car(msg);
      pmt::pmt_t v = pmt::cdr(msg);
      if(!pmt::is_blob(v)){
        throw std::runtime_error("<SU TX DEBUG>msg_in: message in is not a pair(x,blob)");
      }
      // receive blob from LSA CTRL channel
      size_t io(0);
      const uint8_t* uvec = pmt::u8vector_elements(v,io);
      myACK_t* ack_ptr;
      //myACK_t tmp_ack;
      myBlock_t* tmp_block;
      if(io == 2){
        DEBUG<<"<SU TX DEBUG>msg_in:receiving positive sensing information, reset ack table"<<std::endl;
        // TODO
        // do somthing
        // insert previous base block back to the begin of queue!!
      }
      else if(io==6){
        int qidx,qsize;
        unsigned int base;
        parse_pdu(qidx,qsize,base,uvec);
        assert(!d_acklist.empty());
        ack_ptr = &d_acklist.front();
        //DEBUG<<"Receive a control message:"<<qidx<<" ,"<<qsize<<" ,"<<base<<std::endl;
        if( (ack_ptr->get_npkt() == qsize) && (ack_ptr->get_npkt()>qidx) ){
          if(ack_ptr->set_ack(qidx,base)){
            DEBUG<<"<SU TX DEBUG>msg_in: ACK----base:"<<base<<" ,block size="<<qsize<<" ,block idx="<<qidx<<std::endl;
            if(ack_ptr->check_ack()){
              DEBUG<<"<SU TX DEBUG>msg_in:base point--"<<ack_ptr->get_base()<<" ACKed!"<<std::endl;
              tmp_block = &d_blist.front();
              // record previous acked block in case causing interference
              record_block(tmp_block->mem_addr(),tmp_block->block_len());
              // dequeue ACKed element
              d_blist.pop_front();
              d_acklist.pop_front();
              // update queue buffers
              d_base_rx++; // increment acked base
              d_base_cnt++;
            }
          }
        }
      }
      else if(io>6){
        // valid MAC HDR
        int qidx,qsize;
        unsigned int base;
        parse_pdu(qidx,qsize,base,uvec);
        DEBUG<<"<SU TX DEBUG>mag_in:receiving pdu: abort..."<<std::endl;
        return;
      }
      else{
        // invalid Size for current implementation
        DEBUG<<"<SU TX DEBUG>msg_in: invalid payload length, do nothing"<<std::endl;
      }
    }

    void
    su_transmitter_bb_impl::insert_block(const unsigned char* in, int nin)
    {
      int npkt = nin/d_bytes_per_packet;
      assert(npkt < d_pkt_cap);
      myBlock_t tmp_block(d_pkt_circ_cnt,npkt,d_base_cnt);
      for(int i=0;i<npkt;++i){
        memcpy(d_mem_pool[d_pkt_circ_cnt][i],LSAPHYHDR,sizeof(char)*PHYLEN);
        memcpy(d_mem_pool[d_pkt_circ_cnt][i]+PHYLEN,LSAMACHDR,sizeof(char)*MACLEN);
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN-1] = (unsigned char)d_bytes_per_packet + MACLEN;
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN] = (unsigned char)i;
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN+1] = (unsigned char)npkt;
        uint8_t * base_u8 = (uint8_t*)&d_base_cnt;
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN+2] = base_u8[3];
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN+3] = base_u8[2];
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN+4] = base_u8[1];
        d_mem_pool[d_pkt_circ_cnt][i][PHYLEN+5] = base_u8[0];
        memcpy(d_mem_pool[d_pkt_circ_cnt][i]+PHYLEN+MACLEN,in+i*d_bytes_per_packet,sizeof(char)*d_bytes_per_packet);
      }
      // insert into list
      d_blist.push_back(tmp_block);
      // also, create a ack list
      myACK_t tmp_ack(npkt,d_base_cnt);
      d_acklist.push_back(tmp_ack);
      // update buffer info
      d_base_cnt++;
      d_pkt_circ_cnt = (d_pkt_circ_cnt+1)% d_pkt_cap; // wrap around for circular memory
    }

    int
    su_transmitter_bb_impl::output_block(unsigned char* out,int noutput_items)
    {
      assert(!d_blist.empty());
      myBlock_t* tmp_block = &d_blist.front();
      int mem_addr = tmp_block->mem_addr();
      int block_len= tmp_block->block_len();
      int base = tmp_block->base_point();
      int eq_pkt_len = d_bytes_per_packet+MACLEN+PHYLEN;
      int nout = (eq_pkt_len) * block_len;
      assert(noutput_items>=nout);
      for(int i=0;i<block_len;++i){
        memcpy(out+i*(eq_pkt_len),d_mem_pool[mem_addr][i],sizeof(char)*(eq_pkt_len) );
      }
      return nout;
    }

    void
    su_transmitter_bb_impl::parse_pdu(int& qidx,int& qsize,unsigned int& base,const uint8_t* uvec)
    {
      qidx = uvec[0];
      qsize= uvec[1];
      base = uvec[2]<<24;
      base |= uvec[3]<<16;
      base |= uvec[4]<<8;
      base |= uvec[5];
    }

    void
    su_transmitter_bb_impl::record_block(int cir_mem_idx, int npkt)
    {
      int eq_len = d_bytes_per_packet + MACLEN+PHYLEN;
      for(int i=0;i<npkt;++i){
        memcpy(d_prev_block[i],d_mem_pool[cir_mem_idx][i],sizeof(char)*eq_len);
      }
      d_prev_npkt = npkt;
    }

    int
    su_transmitter_bb_impl::work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      unsigned char *out = (unsigned char *) output_items[0];
      int nout = 0;
      // input handling
      if(d_blist.size()<d_pkt_cap){
        // there still capacity for new block
        insert_block(in, ninput_items[0]);
      }
      // output handling
      // for first implementation, only transmit one block until it is received!!
      nout = output_block(out,noutput_items);
      return nout;
    }

  } /* namespace lsa */
} /* namespace gr */

