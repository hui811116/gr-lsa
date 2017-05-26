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
#include <lsa/preamble_prefixer.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

    
    static const unsigned char lsa_SFD = 0xE6;
    static const size_t d_reserved_len = 4;
    static const size_t LSA_PREAMBLE_LEN = 6;
    static const unsigned char lsa_preamble[] = {0x00,0x00,0x00,0x00,0xE6,0x00}; // last one for length
    static const int MAXIMUM_LEN = 127;

  class preamble_prefixer_impl : public preamble_prefixer
  {
    public:
    preamble_prefixer_impl():block("preamble_prefixer",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0))
    {
      d_buf = new unsigned char[256];
      memcpy(d_buf,lsa_preamble,sizeof(char)*LSA_PREAMBLE_LEN);
      d_in_port = pmt::mp("in");
      d_out_port = pmt::mp("out");
      message_port_register_in(d_in_port);
      message_port_register_out(d_out_port);
      set_msg_handler(d_in_port,boost::bind(&preamble_prefixer_impl::add_pre,this, _1));
      
    }
    ~preamble_prefixer_impl()
    {
      delete [] d_buf;
    }

    void
    add_pre(pmt::pmt_t msg)
    {
      if(pmt::is_eof_object(msg)){
        message_port_pub(d_out_port, pmt::PMT_EOF);
        detail().get()->set_done(true);
        return;
      }
      assert(pmt::is_pair(msg));
      pmt::pmt_t mac_tag = pmt::car(msg);
      pmt::pmt_t blob = pmt::cdr(msg);
      assert(pmt::is_blob(blob));
      size_t vlen = pmt::blob_length(blob);

      assert(vlen> d_reserved_len && vlen <= MAXIMUM_LEN);
      memcpy(d_buf+LSA_PREAMBLE_LEN,pmt::blob_data(blob),vlen);
      d_buf[LSA_PREAMBLE_LEN-1] = (unsigned char)vlen;
      pmt::pmt_t packet = pmt::make_blob(d_buf,vlen+LSA_PREAMBLE_LEN);
      message_port_pub(d_out_port, pmt::cons(pmt::PMT_NIL,packet));
      
    }

    private:
      pmt::pmt_t d_in_port;
      pmt::pmt_t d_out_port;
      unsigned char* d_buf;
  };


  preamble_prefixer::sptr
  preamble_prefixer::make()
  {
    return gnuradio::get_initial_sptr(new preamble_prefixer_impl());
  }

  } /* namespace lsa */
} /* namespace gr */

