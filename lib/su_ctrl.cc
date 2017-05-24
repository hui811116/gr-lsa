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

#define d_debug true
#define DEBUG d_debug && std::cerr

static const unsigned char LSA_ACK = 0x01;
//static const unsigned char LSA_NACK= 0x00;
static const unsigned char LSA_SEN = 0x02;
static const unsigned char LSA_CTRL= 0x06;

    static const unsigned char SU_PHY[] = {0x00,0x00,0x00,0x00,0xE6,0x00}; // including length
    static const unsigned char SU_MAC[] = {0x00,0x00,0x00,0x00,0x00,0x00}; // q1,q2, base x4
    static const int PHY_LEN = 6;
    static const int MAC_LEN = 6;
    static const unsigned int BASEMAX = 16777216;

    class su_ctrl_impl: public su_ctrl{
      public:
      su_ctrl_impl():block("su_ctrl",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0))
      {
        d_msg_in = pmt::mp("ctrl_in");
        d_msg_out= pmt::mp("ctrl_out");
        message_port_register_in(d_msg_in);
        message_port_register_out(d_msg_out);
        set_msg_handler(d_msg_in,boost::bind(&su_ctrl_impl::msg_in,this,_1));
        memcpy(d_ctrl_buf,SU_PHY,sizeof(unsigned char)*PHY_LEN);
      }
      ~su_ctrl_impl(){

      }
      void
      msg_in(pmt::pmt_t msg)
      {
        pmt::pmt_t key = pmt::car(msg);
        pmt::pmt_t value = pmt::cdr(msg);
        pmt::pmt_t blob;
        int qsize,qidx;
        unsigned int base;
        if(pmt::eqv(key,pmt::intern("LSA_hdr")) && pmt::is_blob(value)){
          //pmt::pmt_t k = pmt::car(msg);
          //pmt::pmt_t v = pmt::cdr(msg);
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(value,io);
          if(io==2){
            DEBUG<<"<SU CTRL DEBUG>recieved a sensing positive tag"<<std::endl;
            d_ctrl_buf[PHY_LEN-1]=LSA_SEN;
            d_ctrl_buf[PHY_LEN+0]=0xff;
            d_ctrl_buf[PHY_LEN+1]=0x00;
            blob = pmt::make_blob(d_ctrl_buf,PHY_LEN+LSA_SEN);
          }else if(io==6){
            parse_pdu(qidx,qsize,base,uvec);
            DEBUG<<"<SU CTRL DEBUG>received a LSA Control frame:"
            <<"block_idx="<<qidx<<" ,block_size="<<qsize<<" ,base="<<base<<std::endl;
            return;
          }else if(io>6){
            parse_pdu(qidx,qsize,base,uvec);
            if(crc_check(qidx,qsize,base)){
              DEBUG<<"<SU CTRL DEBUG>received PHY packet--block_idx="
              <<qidx<<" ,block_size="<<qsize<<" ,base="<<base<<std::endl;
              d_ctrl_buf[PHY_LEN-1]= LSA_CTRL;
              d_ctrl_buf[PHY_LEN]  =(unsigned char)qidx;
              d_ctrl_buf[PHY_LEN+1]=(unsigned char)qsize;
              unsigned char* base_u8 = (unsigned char*)&base;
              d_ctrl_buf[PHY_LEN+2]= base_u8[3];
              d_ctrl_buf[PHY_LEN+3]= base_u8[2];
              d_ctrl_buf[PHY_LEN+4]= base_u8[1];
              d_ctrl_buf[PHY_LEN+5]= base_u8[0];
              blob = pmt::make_blob(d_ctrl_buf,PHY_LEN+LSA_CTRL);  
            }else{
              return ;
            }
          }else{
            return;
          }
        }
        else if(pmt::dict_has_key(msg,pmt::intern("LSA_sensing"))){
          d_ctrl_buf[PHY_LEN-1] = LSA_SEN;
          d_ctrl_buf[PHY_LEN  ] = 0xff;
          d_ctrl_buf[PHY_LEN+1] = 0x00;
          blob = pmt::make_blob(d_ctrl_buf,PHY_LEN+LSA_SEN);
        }
        else{
          return;
        }
        // can support more feedback types
        assert(pmt::is_blob(blob));
        message_port_pub(d_msg_out,pmt::cons(pmt::PMT_NIL,blob));
      }
      
      private:
      void parse_pdu(int& qidx,int& qsize,unsigned int& base, const uint8_t* uvec)
      {
        qidx = uvec[0];
        qsize= uvec[1];
        base = uvec[2]<<24;
        base|= uvec[3]<<16;
        base|= uvec[4]<<8;
        base|= uvec[5];
      }
      bool crc_check(int qidx,int qsize,unsigned int base)
      {
        if(qidx>=qsize){
          return false;
        }else if(base>BASEMAX){
          return false;
        }else{
          return true;
        }
      }

      pmt::pmt_t d_msg_in;
      pmt::pmt_t d_msg_out;
      unsigned char d_ctrl_buf[256];
      // bits field
      // ---------------------------------------------------
      // | preamble | SFD  | LEN   | qidx | qsize |  base
      // ---------------------------------------------------
      // |   32     |   8  |   8   |  8   |  8    |   32
      // ---------------------------------------------------

    };
    su_ctrl::sptr
    su_ctrl::make(){
      return gnuradio::get_initial_sptr(new su_ctrl_impl());
    }
  } /* namespace lsa */
} /* namespace gr */

