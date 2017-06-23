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
#include <lsa/simple_rx.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {
    #define d_debug false
    #define DEBUG d_debug && std::cout
    #define SEQLEN 4
    #define RESETLIMIT 5
    #define MAXLEN 123
    #define MINLEN 4

    class simple_rx_impl : public simple_rx
    {
      public:
        simple_rx_impl():block("simple_rx",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0)),
                d_in_port(pmt::mp("pdu_in")),
                d_out_port(pmt::mp("ack_out")),
                d_pdu_port(pmt::mp("pdu_out"))
        {
          reset();
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          message_port_register_out(d_pdu_port);
          set_msg_handler(d_in_port,boost::bind(&simple_rx_impl::msg_in,this,_1));
        }
        ~simple_rx_impl(){}
        void msg_in(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          assert(pmt::is_blob(v));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          if(io<=MAXLEN && io >MINLEN){
            // crc
            uint16_t base1,base2;
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1 == base2){
              DEBUG<<"<SIMPLE RX>Valid seq:"<<base1<<std::endl;
              // crc passed
              d_reset_cnt++;
              if(d_reset_cnt>=RESETLIMIT || base1 == d_expect_seq){
                DEBUG<<"<SIMPLE RX>Acknowledge seq:"<<base1<<" ,reset_cnt:"<<d_reset_cnt<<std::endl;
                d_reset_cnt =0;
                // ack this pkt
                d_rx_seq = base1;
                d_expect_seq = (base1==0xffff)? 0:base1+1;
                const uint8_t* u8 = (const uint8_t*) &d_rx_seq;
                d_buf[0] = u8[1];
                d_buf[1] = u8[0];
                d_buf[2] = u8[1];
                d_buf[3] = u8[0];
                pmt::pmt_t msg_out = pmt::make_blob(d_buf,SEQLEN);
                pmt::pmt_t pdu_out = pmt::make_blob(uvec+SEQLEN,io-SEQLEN);
                message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,msg_out));
                // export valid pdu only...
                message_port_pub(d_pdu_port,pmt::cons(pmt::PMT_NIL,pdu_out));
              }
            }
          }
        }
      private:
        void reset()
        {
          d_expect_seq =0;
          d_rx_seq=0;
          d_reset_cnt=0;
        }
        gr::thread::mutex d_mutex;
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_out_port;
        const pmt::pmt_t d_pdu_port;
        uint16_t d_rx_seq;
        uint16_t d_expect_seq;
        int d_reset_cnt;
        unsigned char d_buf[256];
    };
    simple_rx::sptr 
    simple_rx::make()
    {
      return gnuradio::get_initial_sptr(new simple_rx_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

