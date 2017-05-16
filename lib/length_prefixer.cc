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
#include <lsa/length_prefixer.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

    class length_prefixer_impl : public length_prefixer
    {
      public:
      length_prefixer_impl() :block("length_prefixer",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0)),
                        d_out_port(pmt::mp("pdu_out")),
                        d_in_port(pmt::mp("pdu_in"))
      {
        message_port_register_in(d_in_port);
        message_port_register_out(d_out_port);
        set_msg_handler(d_in_port,boost::bind(&length_prefixer_impl::msg_handler,this,_1));
      }

      ~length_prefixer_impl()
      {

      }
      void
      msg_handler(pmt::pmt_t msg)
      {
        pmt::pmt_t v = pmt::cdr(msg);
        assert(pmt::is_blob(v));
        size_t io(0);
        const uint8_t* u8vec = pmt::u8vector_elements(v,io);
        if(io>127){
          return;
        }
        memcpy(d_buf+1,u8vec,sizeof(char)*io);
        d_buf[0] = (unsigned char)io;
        message_port_pub(d_out_port,pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_buf,io+1)));
      }
      private:
      unsigned char d_buf[1024];
      const pmt::pmt_t d_out_port;
      const pmt::pmt_t d_in_port;
    };

    length_prefixer::sptr
    length_prefixer::make()
    {
      return gnuradio::get_initial_sptr(new length_prefixer_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

