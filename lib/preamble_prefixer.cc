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

    static const unsigned char d_default_preamble[] = {0xac,0xdd,0xa4,0xe2,0xf2,0x82,0x20,0xfc};
    static const unsigned char lsa_SFD[] = {0xa7,0xff};

  class preamble_prefixer_impl : public preamble_prefixer
  {
    public:
    preamble_prefixer_impl(const std::vector<unsigned char>& preamble):block("preamble_prefixer",
                        gr::io_signature::make(0,0,0),
                        gr::io_signature::make(0,0,0))
    {
      d_buf = new unsigned char[1024];
      set_preamble(preamble);
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
    set_preamble(const std::vector<unsigned char>& preamble)
    {
      if(preamble.empty()){
        throw std::runtime_error("Bad preamble");
      }
      d_preamble = preamble;
      memcpy(d_buf,preamble.data(),sizeof(char)*preamble.size());
      memcpy(d_buf+d_preamble.size(),lsa_SFD, sizeof(char)*2);
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
      pmt::pmt_t blob = pmt::cdr(msg);
      size_t vlen = pmt::blob_length(blob);
      size_t ulen = vlen/sizeof(char);
      uint8_t pld_MSB = (uint8_t*) &ulen;
      d_buf[d_preamble.size()+3] = pld_MSB[1];
      d_buf[d_preamble.size()+4] = pld_MSB[0];
      d_buf[d_preamble.size()+5] = pld_MSB[1];
      d_buf[d_preamble.size()+6] = pld_MSB[0];
      memcpy(d_buf+d_preamble.size()+6,pmt::blob_data(blob),vlen);
      pmt::pmt_t packet = pmt::make_blob(d_buf,vlen+d_preamble.size());
      message_port_pub(d_out_port, pmt::cons(pmt::PMT_NIL,packet));
    }

    private:
      pmt::pmt_t d_in_port;
      pmt::pmt_t d_out_port;
      unsigned char* d_buf;
      std::vector<unsigned char> d_preamble;
  };

/*
    preamble_prefixer::preamble_prefixer()
    {
    }

    preamble_prefixer::~preamble_prefixer()
    {
    }
*/

  preamble_prefixer::sptr
  preamble_prefixer::make(const std::vector<unsigned char>& preamble)
  {
    return gnuradio::get_initial_sptr(new preamble_prefixer_impl(preamble));
  }

  } /* namespace lsa */
} /* namespace gr */

