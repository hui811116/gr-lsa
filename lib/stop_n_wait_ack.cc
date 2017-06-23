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
#include <lsa/stop_n_wait_ack.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {
    #define d_debug true
    #define DEBUG d_debug && std::cout
    #define SNS_COLLISION 2
    #define SNS_CLEAR 3
    #define SNS_ACK 4
    #define MACLEN 4
    #define MAXLEN 127
    
    static unsigned char d_sns_collision[] = {0x00,0xff};
    static unsigned char d_sns_clear[] = {0x00,0xff,0x0f};

    class stop_n_wait_ack_impl : public stop_n_wait_ack
    {
      public:
        stop_n_wait_ack_impl() : block("stop_n_wait_ack",
                      gr::io_signature::make(0,0,0),
                      gr::io_signature::make(0,0,0)),
                      d_in_port(pmt::mp("pdu_in")),
                      d_out_port(pmt::mp("msg_out"))
        {
          message_port_register_in(d_in_port);
          message_port_register_out(d_out_port);
          set_msg_handler(d_in_port,boost::bind(&stop_n_wait_ack_impl::msg_handler,this,_1));
        }
        
        ~stop_n_wait_ack_impl(){}
        
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
          if(!sns_crc(uvec,io)){
            return;
          }
          if(io == SNS_COLLISION){
            // sensing true
            message_port_pub(d_out_port,msg);
            DEBUG<<"\033[31;1m"<<"<SNS ACK> positive sensing"<<"\033[0m"<<std::endl;
          }else if(io == SNS_CLEAR){
            // resume signal
            message_port_pub(d_out_port,msg);
            DEBUG<<"\033[31;1m"<<"<SNS ACK> clear signal"<<"\033[0m"<<std::endl;
          }else if(io == SNS_ACK){
            // receive an ACK signal
            pmt::pmt_t msg_out = pmt::make_dict();
            msg_out = pmt::dict_add(msg_out,pmt::intern("SNS_ctrl"),pmt::from_long(SNS_ACK));
            msg_out = pmt::dict_add(msg_out,pmt::intern("base"),pmt::from_long(d_base));
            message_port_pub(d_out_port,msg_out);
            DEBUG<<"\033[31;1m"<<"<SNS ACK> Receiving an ACK--base="<<d_base<<"\033[0m"<<std::endl;
          }else{
            // normal payload
            memcpy(d_buf,uvec,sizeof(char)*MACLEN);
            pmt::pmt_t blob_out = pmt::cons(pmt::intern("SNS_hdr"),pmt::make_blob(d_buf,MACLEN));
            message_port_pub(d_out_port,blob_out);
            DEBUG<<"\033[31;1m"<<"<SNS ACK> Receive an valid header, sending ack--"<<d_base<<"\033[0m"<<std::endl;
          }          
        }
      
      private:
        bool sns_crc(const uint8_t* uvec, size_t io)
        {
          if(io>MAXLEN || io==1 || io==0){
            return false;
          }else if(io==SNS_COLLISION){
            if(d_sns_collision[0]!=uvec[0] || d_sns_collision[1]!=uvec[1]){
              return false;
            }
          }else if(io==SNS_CLEAR){
            if(d_sns_clear[0]!=uvec[0]||d_sns_clear[1]!=uvec[1]||d_sns_clear[2]!=uvec[2]){
              return false;
            }
          }else if(io>=SNS_ACK){
            uint16_t base = 0x0000;
            uint16_t base2= 0x0000;
            base = uvec[0]<<8;
            base |=uvec[1];
            base2 = uvec[2]<<8;
            base2|= uvec[3];
            if(base==base2){
              d_base = base;
            }else{
              return false;
            }
          }else{
            return false;
          }
          return true;
        }
        unsigned char d_buf[256];
        gr::thread::mutex d_mutex;
        const pmt::pmt_t d_in_port;
        const pmt::pmt_t d_out_port;
        uint16_t d_base;
    };


    stop_n_wait_ack::sptr 
    stop_n_wait_ack::make()
    {
      return gnuradio::get_initial_sptr(new stop_n_wait_ack_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

