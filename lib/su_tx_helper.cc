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
#include <lsa/su_tx_helper.h>
#include <gnuradio/block_detail.h>
#include <ctime>

namespace gr {
  namespace lsa {
    #define LSAMACLEN 8
    #define LSASENLEN 2
    
    class su_tx_helper_impl : public su_tx_helper{
      public:
       su_tx_helper_impl():block("su_tx_helper",
                          gr::io_signature::make(0,0,0),
                          gr::io_signature::make(0,0,0)),
                          d_in_port(pmt::mp("pdu_in")),
                          d_out_port(pmt::mp("msg_out"))
       {
        message_port_register_in(d_in_port);
        message_port_register_out(d_out_port);
        set_msg_handler(d_in_port,boost::bind(&su_tx_helper_impl::msg_in,this,_1));
       }
       ~su_tx_helper_impl(){}
       void msg_in(pmt::pmt_t msg)
       {
         gr::thread::scoped_lock guard(d_mutex);
         assert(pmt::is_pair(msg));
         pmt::pmt_t k = pmt::car(msg);
         pmt::pmt_t v = pmt::cdr(msg);
         assert(pmt::is_blob(v));
         size_t io(0);
         const uint8_t* uvec = pmt::u8vector_elements(v,io);
         if(io==LSASENLEN){
          // sensing
          pmt::pmt_t out = pmt::make_dict();
          out = pmt::dict_add(out,pmt::intern("LSA_sensing"),pmt::PMT_T);
          message_port_pub(d_out_port,out);
         }else if(io==LSAMACLEN){
           // only control message
           uint16_t base1,base2;
           d_qidx = uvec[0]<<8;
           d_qidx|= uvec[1];
           d_qsize= uvec[2]<<8;
           d_qsize|= uvec[3];
           base1 = uvec[4]<<8;
           base1|= uvec[5];
           base2 = uvec[6]<<8;
           base2|= uvec[7];
           if( (d_qidx!=0 && d_qidx>=d_qsize)||(base1!=base2) ){
             return;
           }
           d_base = base1;
           pmt::pmt_t out = pmt::make_dict();
           out = pmt::dict_add(out,pmt::intern("queue_index"),pmt::from_long(d_qidx));
           out = pmt::dict_add(out,pmt::intern("queue_size"),pmt::from_long(d_qsize));
           out = pmt::dict_add(out,pmt::intern("base"),pmt::from_long(d_base));
           message_port_pub(d_out_port,out);
         }else if(io>LSAMACLEN){
          // pdu
         }
       }
      private:
       gr::thread::mutex d_mutex;
       const pmt::pmt_t d_in_port;
       const pmt::pmt_t d_out_port;
       uint16_t d_qidx;
       uint16_t d_qsize;
       uint16_t d_base;
    };

    su_tx_helper::sptr 
    su_tx_helper::make(){
      return gnuradio::get_initial_sptr(new su_tx_helper_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

