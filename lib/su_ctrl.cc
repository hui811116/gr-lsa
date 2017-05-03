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
    enum FEEDBACKTYPE{
      ACK,
      SENSING
    };
    class su_ctrl_impl: public su_ctrl{
      public:
      su_ctrl_impl(const std::vector<unsigned char>& accesscode):block("su_ctrl",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0))
      {
        d_msg_in = pmt::mp("ctrl_in");
        d_msg_out= pmt::mp("ctrl_out");
        message_port_register_in(d_msg_in);
        message_port_register_out(d_msg_out);
        set_msg_handler(d_msg_in,boost::bind(&su_ctrl_impl::msg_in,this,_1));
        set_accesscode(accesscode);
      }
      ~su_ctrl_impl(){

      }
      void
      set_accesscode(const std::vector<unsigned char>& accesscode)
      {
        if(accesscode.empty()){
          throw std::invalid_argument("Preamble field cannot be empty");
        }
        d_accesscode = accesscode;
        assert(d_ctrl_buf!=NULL);
        memcpy(d_ctrl_buf,d_accesscode.data(),sizeof(char)*d_accesscode.size());
      }


      void
      msg_in(pmt::pmt_t msg)
      {
        assert(pmt::is_dict(msg));
        pmt::pmt_t blob;
        if(pmt::dict_has_key(msg,pmt::intern("LSA_hdr"))){
          uint16_t pld, cnt;
          uint8_t qidx,qsize;
          pld = 0x0001;
          //pld = pmt::to_long(pmt::dict_ref(msg,pmt::intern("payload"),pmt::from_long(0)));
          cnt = pmt::to_long(pmt::dict_ref(msg,pmt::intern("counter"),pmt::from_long(0)));
          //qidx= pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_index"),pmt::from_long(0)));
          //qsize=pmt::to_long(pmt::dict_ref(msg,pmt::intern("queue_size"),pmt::from_long(0)));
          set_hdr(ACK,pld,cnt);
          blob = pmt::make_blob(d_ctrl_buf,d_accesscode.size()+8);
        }
        else if(pmt::dict_has_key(msg,pmt::intern("LSA_sensing"))){
          set_hdr(SENSING,0,0);
          blob = pmt::make_blob(d_ctrl_buf,d_accesscode.size()+8);
        }
        else{
          return;
        }
        // can support more feedback types
        assert(pmt::is_blob(blob));
        message_port_pub(d_msg_out,pmt::cons(pmt::PMT_NIL,blob));
        
      }
      void
      set_hdr(FEEDBACKTYPE type,uint16_t pld_len, uint16_t cnt){
        int pre_len = d_accesscode.size();
        unsigned char* pld_u8 = (unsigned char*) &pld_len;
        unsigned char* cnt_u8 = (unsigned char*) &cnt;
        switch(type){
          case ACK:
            d_ctrl_buf[pre_len] = 0x00;
            d_ctrl_buf[pre_len+1]= 0x00;
            d_ctrl_buf[pre_len+2]= pld_u8[1];
            d_ctrl_buf[pre_len+3]= pld_u8[0];
            d_ctrl_buf[pre_len+4]= pld_u8[1];
            d_ctrl_buf[pre_len+5]= pld_u8[0];
            d_ctrl_buf[pre_len+6]= cnt_u8[1];
            d_ctrl_buf[pre_len+7]= cnt_u8[0];
          break;
          case SENSING:
            d_ctrl_buf[pre_len] = 0xff;
            d_ctrl_buf[pre_len+1] = 0x00;
            d_ctrl_buf[pre_len+2] = 0xff;
            d_ctrl_buf[pre_len+3] = 0xff;
            d_ctrl_buf[pre_len+4] = 0x00;
            d_ctrl_buf[pre_len+5] = 0x00;
            d_ctrl_buf[pre_len+6] = 0xff;
            d_ctrl_buf[pre_len+7] = 0x00;
          break;
          default:
            throw std::runtime_error("ERROR: unrecognized feedback type");
          break;
        }
      }
      private:
      pmt::pmt_t d_msg_in;
      pmt::pmt_t d_msg_out;
      unsigned char d_ctrl_buf[256];
      std::vector<unsigned char> d_accesscode;

      // bits field
      // ---------------------------------------------------
      // | preamble | qidx | qsize | len1 | len2 | counter |
      // ---------------------------------------------------
      // |   64     |   8  |   8   |  16  |  16  |   16    |
      // ---------------------------------------------------

    };
    su_ctrl::sptr
    su_ctrl::make(const std::vector<unsigned char>& accesscode){
      return gnuradio::get_initial_sptr(new su_ctrl_impl(accesscode));
    }
  } /* namespace lsa */
} /* namespace gr */

