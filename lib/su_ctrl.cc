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

static const unsigned char LSA_ACK = 0x01;
static const unsigned char LSA_NACK= 0x00;
static const unsigned char LSA_SEN = 0x02;

    static const unsigned char SU_PHY[] = {0x00,0x00,0x00,0x00,0xE6,0x00,0x00,0x00}; // including length, qidx, qsize
    static const int PHY_LEN = 8;

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
        assert(pmt::is_dict(msg));
        pmt::pmt_t blob;
        uint8_t qsize,qidx;
        if(pmt::dict_has_key(msg,pmt::intern("LSA_hdr"))){
          uint8_t qidx,qsize;
          qidx= pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_index"),pmt::from_long(0)));
          qsize=pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_size"),pmt::from_long(0)));
          set_hdr(LSA_ACK,qidx,qsize);
          blob = pmt::make_blob(d_ctrl_buf,PHY_LEN);
        }
        else if(pmt::dict_has_key(msg,pmt::intern("LSA_sensing"))){
          set_hdr(LSA_SEN,0xff,0x00);
          blob = pmt::make_blob(d_ctrl_buf,PHY_LEN);
        }
        else{
          return;
        }
        // can support more feedback types
        assert(pmt::is_blob(blob));
        message_port_pub(d_msg_out,pmt::cons(pmt::PMT_NIL,blob));
      }
      void
      set_hdr(uint8_t len, uint8_t qidx,uint8_t qsize){
          d_ctrl_buf[5] = len;
          d_ctrl_buf[6] = qidx;
          d_ctrl_buf[7] = qsize;
      }
      private:
      pmt::pmt_t d_msg_in;
      pmt::pmt_t d_msg_out;
      unsigned char d_ctrl_buf[256];
      // bits field
      // ---------------------------------------------------
      // | preamble | SFD  | LEN   | qidx | qsize |
      // ---------------------------------------------------
      // |   32     |   8  |   8   |  8   |  8    |  
      // ---------------------------------------------------

    };
    su_ctrl::sptr
    su_ctrl::make(){
      return gnuradio::get_initial_sptr(new su_ctrl_impl());
    }
  } /* namespace lsa */
} /* namespace gr */

