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
#include <lsa/chip_mapper.h>
#include <gnuradio/block_detail.h>

namespace gr {
  namespace lsa {

static const unsigned int CHIPSET[16] = {3653456430,
                                  3986437410,
                                  786023250,
                                  585997365,
                                  1378802115,
                                  891481500,
                                  3276943065,
                                  2620728045,
                                  2358642555,
                                  3100205175,
                                  2072811015,
                                  2008598880,
                                  125537430,
                                  1618458825,
                                  2517072780,
                                  3378542520};


class chip_mapper_impl :public chip_mapper
{
  public:
   chip_mapper_impl() : block("chip_mapper",
                gr::io_signature::make(0,0,0),
                gr::io_signature::make(0,0,0))
                //d_preamble_len(8)
  {
    d_pdu_in = pmt::mp("pdu_in");
    d_pdu_out= pmt::mp("pdu_out");
    message_port_register_in(d_pdu_in);
    message_port_register_out(d_pdu_out);
    set_msg_handler(d_pdu_in,boost::bind(&chip_mapper_impl::pdu_in,this,_1));
    //for(int i=0;i<8;++i)
      //d_u32_buf[i] = CHIPSET[0];
  }
   ~chip_mapper_impl()
   {

   }
   void
   pdu_in(pmt::pmt_t msg)
   {
    assert(pmt::is_pair(msg));
    pmt::pmt_t k,v;
    k=pmt::car(msg);
    v=pmt::cdr(msg);
    assert(pmt::is_blob(v));
    size_t io(0);
    const uint8_t* u8vec = pmt::u8vector_elements(v,io);
    assert(io>0 && io<256);
    //for(int i=0;i<io*2;++i){
      //d_u32_buf[i]=CHIPSET[ (u8vec[i/2] >> (4*(1-i%2))  )& 0x0f ];
    //}
    for(int i=0;i<io;++i){
      int idx1 = (u8vec[i] >> 4) & 0x0f;
      int idx2 = u8vec[i] & 0x0f;
      const uint8_t* u8_1 =(const uint8_t *)& CHIPSET[idx1];
      const uint8_t* u8_2 =(const uint8_t *)& CHIPSET[idx2];
      d_buf[8*i] = u8_1[3];
      d_buf[8*i+1] = u8_1[2];
      d_buf[8*i+2] = u8_1[1];
      d_buf[8*i+3] = u8_1[0];

      d_buf[8*i+4] = u8_2[3];
      d_buf[8*i+5] = u8_2[2];
      d_buf[8*i+6] = u8_2[1];
      d_buf[8*i+7] = u8_2[0];
    }
    message_port_pub(d_pdu_out,pmt::cons(pmt::PMT_NIL,pmt::make_blob(d_buf,io*8)));
   }

  private:
  pmt::pmt_t d_pdu_in;
  pmt::pmt_t d_pdu_out;
  unsigned char d_buf[2048]; //check
  //unsigned int d_u32_buf[1024];
  //const size_t d_preamble_len;
};
   chip_mapper::sptr
    chip_mapper::make(){
      return gnuradio::get_initial_sptr(new chip_mapper_impl());
    }

  } /* namespace lsa */
} /* namespace gr */

