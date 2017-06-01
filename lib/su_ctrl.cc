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

#define DEBUG d_debug && std::cerr

static const unsigned char LSA_ACK = 0x01;
static const unsigned char LSA_SEN = 0x02;
static const unsigned char LSA_CTRL= 0x06;

    static const int MAC_LEN = 6;
    static const unsigned int BASEMAX = 16777216;

    class su_ctrl_impl: public su_ctrl{
      public:
      su_ctrl_impl(bool debug):block("su_ctrl",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0))
      {
        d_msg_in = pmt::mp("ctrl_in");
        d_msg_out= pmt::mp("ctrl_out");
        message_port_register_in(d_msg_in);
        message_port_register_out(d_msg_out);
        set_msg_handler(d_msg_in,boost::bind(&su_ctrl_impl::msg_in,this,_1));
        d_prou_present = false;
        d_debug = debug;
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
        if(pmt::is_blob(value)){
          //DEBUG<<"<SU CTRL> received a blob"<<std::endl;
          size_t io(0);
          const uint8_t* uvec = pmt::u8vector_elements(value,io);
          if(io==2){
            DEBUG<<"<SU CTRL DEBUG>recieved a sensing positive tag"<<std::endl;
            d_ctrl_buf[0]=0xff;
            d_ctrl_buf[1]=0x00;
            blob = pmt::make_blob(d_ctrl_buf,LSA_SEN);
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
              /*d_ctrl_buf[0]  =(unsigned char)qidx;
              d_ctrl_buf[1]=(unsigned char)qsize;
              unsigned char* base_u8 = (unsigned char*)&base;
              d_ctrl_buf[2]= base_u8[3];
              d_ctrl_buf[3]= base_u8[2];
              d_ctrl_buf[4]= base_u8[1];
              d_ctrl_buf[5]= base_u8[0];*/
              uint8_t * qi8 = (uint8_t*) &qidx;
              uint8_t * qs8 = (uint8_t*) &qsize;
              uint8_t * bs8 = (uint8_t*) &base;
              d_ctrl_buf[0] = qi8[1];
              d_ctrl_buf[1] = qi8[0];
              d_ctrl_buf[2] = qs8[1];
              d_ctrl_buf[3] = qs8[0];
              d_ctrl_buf[4] = bs8[1];
              d_ctrl_buf[5] = bs8[0];
              blob = pmt::make_blob(d_ctrl_buf,LSA_CTRL);  
              if(d_prou_present){
                // receive a clean packet from interference state
                // reset the sensing state...
                d_prou_present = false;
              }
            }else{
              DEBUG<<"<SU CTRL DEBUG>CRC check failed:"<<qidx<<" ,"<<qsize<<" ,"<<base<<std::endl;
              return ;
            }
          }else{
            return;
          }
        }
        else if(pmt::dict_has_key(msg,pmt::intern("LSA_sensing"))){
          //d_ctrl_buf[PHY_LEN-1] = LSA_SEN;
          d_ctrl_buf[0] = 0xff;
          d_ctrl_buf[1] = 0x00;
          blob = pmt::make_blob(d_ctrl_buf,LSA_SEN);
          if(!d_prou_present){
            DEBUG<<"<SU CTRL>Receive interference signal! send information back to TX!"<<std::endl;
            d_prou_present = true;
          }
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
        qidx = uvec[0]<<8;
        qidx|= uvec[1];
        qsize= uvec[2]<<8;
        qsize|=uvec[3];
        base=0;
        base =uvec[4]<<8;
        base|=uvec[5];
        /*qidx = uvec[0];
        qsize= uvec[1];
        base = uvec[2]<<24;
        base|= uvec[3]<<16;
        base|= uvec[4]<<8;
        base|= uvec[5];*/
      }
      bool crc_check(int qidx,int qsize,unsigned int base)
      {
        if(qidx!=0 && qidx>=qsize){
          return false;
        }else if(base>BASEMAX){
          return false;
        }
        return true;
      }

      pmt::pmt_t d_msg_in;
      pmt::pmt_t d_msg_out;
      unsigned char d_ctrl_buf[256];
      //  MAC bits field
      // ---------------------------------------------------
      // | qidx | qsize |  base
      // ---------------------------------------------------
      // |  8   |  8    |   32
      // ---------------------------------------------------
      bool d_prou_present;
      bool d_debug;
    };
    su_ctrl::sptr
    su_ctrl::make(bool debug){
      return gnuradio::get_initial_sptr(new su_ctrl_impl(debug));
    }
  } /* namespace lsa */
} /* namespace gr */

