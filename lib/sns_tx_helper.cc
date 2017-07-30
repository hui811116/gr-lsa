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
#include <lsa/sns_tx_helper.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {
    #define d_debug true
    #define DEBUG d_debug && std::cout
    #define SNS_COLLISION 2
    #define SNS_CLEAR 3
    #define SNS_ACK 4
    class sns_tx_helper_impl : public sns_tx_helper
    {
      public:
        sns_tx_helper_impl(): block("sns_tx_helper",
                    gr::io_signature::make(0,0,0),
                    gr::io_signature::make(0,0,0)),
                    d_in_port(pmt::mp("pdu_in")),
                    d_out_port(pmt::mp("msg_out"))
        {
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          set_msg_handler(d_in_port,boost::bind(&sns_tx_helper_impl::msg_handler,this,_1));
        }
        ~sns_tx_helper_impl(){}
        void
        msg_handler(pmt::pmt_t msg)
        {
          gr::thread::scoped_lock guard(d_mutex);
          assert(pmt::is_pair(msg));
          pmt::pmt_t k = pmt::car(msg);
          pmt::pmt_t v = pmt::cdr(msg);
          assert(pmt::is_blob(v));
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(v,io);
          pmt::pmt_t msg_out = pmt::make_dict();
          if(io==SNS_COLLISION){
            // SNS sensing positive
            if(!sns_crc(uvec,io,SNS_COLLISION))
              return;
            msg_out = pmt::dict_add(msg_out,pmt::intern("SNS_ctrl"),pmt::from_long(SNS_COLLISION));
            message_port_pub(d_out_port,msg_out);
          }else if(io==SNS_CLEAR){
            if(!sns_crc(uvec,io,SNS_CLEAR))
              return;
            msg_out = pmt::dict_add(msg_out,pmt::intern("SNS_ctrl"),pmt::from_long(SNS_CLEAR));
            message_port_pub(d_out_port,msg_out);
          }else if(io==SNS_ACK){
            uint16_t base1 = 0x0000, base2 = 0x0000;
            base1 = uvec[0]<<8;
            base1|= uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base1!=base2){
              return;
            }
            msg_out = pmt::dict_add(msg_out,pmt::intern("SNS_ctrl"),pmt::from_long(SNS_ACK));
            msg_out = pmt::dict_add(msg_out,pmt::intern("base"),pmt::from_long(base1));
            message_port_pub(d_out_port,msg_out);
          }else{
            // tx should not recive other type of pdu length
            return;
          }
        }
      private:
        bool sns_crc(const uint8_t* uvec, size_t io, int type){
          if(type == SNS_CLEAR){
            return uvec[0]==0x00 && uvec[1]==0xff && uvec[2]==0x0f;
          }else if(type == SNS_COLLISION){
            return uvec[0]==0xff && uvec[1]==0x00;
          }else if(type == SNS_ACK){
            return uvec[0]==uvec[2] && uvec[1]==uvec[3];
          }else{
            return false;
          }
        }
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_out_port;
        gr::thread::mutex d_mutex;
    };

    sns_tx_helper::sptr 
    sns_tx_helper::make()
    {
      return gnuradio::get_initial_sptr(new sns_tx_helper_impl());
    }
  } /* namespace lsa */
} /* namespace gr */

