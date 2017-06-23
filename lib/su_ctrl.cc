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
#include <lsa/su_ctrl.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

    #define d_debug false
    #define DEBUG d_debug && std::cerr
    #define LSAMACLEN 8
    #define LSASENLEN 2
    static const unsigned char LSA_SEN = 0x02;
    static const unsigned char LSA_CTRL= 0x08;
    static const unsigned int BASEMAX = 16777216;

    class su_ctrl_impl: public su_ctrl{
      public:
      su_ctrl_impl():block("su_ctrl",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0)),
                        d_msg_in(pmt::mp("ctrl_in")),
                        d_msg_out(pmt::mp("ctrl_out"))
      {
        message_port_register_in(d_msg_in);
        message_port_register_out(d_msg_out);
        set_msg_handler(d_msg_in,boost::bind(&su_ctrl_impl::msg_in,this,_1));
        d_prou_present = false;
      }
      ~su_ctrl_impl(){}

      void msg_in(pmt::pmt_t msg)
      {
        gr::thread::scoped_lock guard(d_mutex);
        pmt::pmt_t key = pmt::car(msg);
        pmt::pmt_t value = pmt::cdr(msg);
        pmt::pmt_t blob;
        uint16_t qsize,qidx;
        uint16_t base1, base2;
        if(pmt::is_blob(value)){
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(value,io);
          if(io==LSASENLEN){
            DEBUG<<"<SU CTRL DEBUG>recieved a sensing positive tag"<<std::endl;
            blob = generate_header(qidx,qsize,base1,LSA_SEN);
          }else if(io==LSAMACLEN){
            parse_pdu(qidx,qsize,base1,base2,uvec);
            if(base1==base2){
              DEBUG<<"<SU CTRL DEBUG>received a LSA Control frame:"
              <<"block_idx="<<qidx<<" ,block_size="<<qsize<<" ,base="<<base1<<std::endl;
            }
            return;
          }else if(io>LSAMACLEN){
            parse_pdu(qidx,qsize,base1,base2,uvec);
            DEBUG<<"<SU CTRL DEBUG>received PHY packet--block_idx="
            <<qidx<<" ,block_size="<<qsize<<" ,base="<<base1<<std::endl;
            blob = generate_header(qidx,qsize,base1,LSA_CTRL);  
          }else{
            return;
          }
        }
        else if(pmt::dict_has_key(msg,pmt::intern("LSA_sensing"))){
          blob = generate_header(qidx,qsize,base1,LSA_SEN);
          if(!d_prou_present){
            DEBUG<<"<SU CTRL>Receive interference signal! send information back to TX!"<<std::endl;
            d_prou_present = true;
          }
        }
        else{
          return;
        }
        // can support more feedback types
        if(!pmt::is_blob(blob)){
          return;
        }
        message_port_pub(d_msg_out,pmt::cons(pmt::PMT_NIL,blob));
      }
      
      private:
      void parse_pdu(uint16_t& qidx,uint16_t& qsize,uint16_t& base1,uint16_t& base2, const uint8_t* uvec)
      {
        qidx = uvec[0]<<8;
        qidx|= uvec[1];
        qsize= uvec[2]<<8;
        qsize|=uvec[3];
        base1 =uvec[4]<<8;
        base1|=uvec[5];
        base2 = uvec[6]<<8;
        base2|= uvec[7];
      }
      pmt::pmt_t generate_header(int qidx,int qsize,unsigned int base,unsigned char type)
      {
        if(type == LSA_SEN){
          d_ctrl_buf[0]=0xff;
          d_ctrl_buf[1]=0x00;
          return pmt::make_blob(d_ctrl_buf,LSA_SEN);
        }else if(type==LSA_CTRL){
          uint8_t * qi8 = (uint8_t*) &qidx;
          uint8_t * qs8 = (uint8_t*) &qsize;
          uint8_t * bs8 = (uint8_t*) &base;
          d_ctrl_buf[0] = qi8[1];
          d_ctrl_buf[1] = qi8[0];
          d_ctrl_buf[2] = qs8[1];
          d_ctrl_buf[3] = qs8[0];
          d_ctrl_buf[4] = bs8[1];
          d_ctrl_buf[5] = bs8[0];
          d_ctrl_buf[6] = bs8[1];
          d_ctrl_buf[7] = bs8[0];
          return pmt::make_blob(d_ctrl_buf,LSA_CTRL);  
        }else{
          return pmt::PMT_NIL;
        }
      }
      const pmt::pmt_t d_msg_in;
      const pmt::pmt_t d_msg_out;
      unsigned char d_ctrl_buf[256];
      bool d_prou_present;
      gr::thread::mutex d_mutex;
    };
    su_ctrl::sptr
    su_ctrl::make(){
      return gnuradio::get_initial_sptr(new su_ctrl_impl());
    }
  } /* namespace lsa */
} /* namespace gr */

