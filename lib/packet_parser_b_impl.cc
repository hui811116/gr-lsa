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
#include "packet_parser_b_impl.h"

namespace gr {
  namespace lsa {

    enum HDRSTATE{
      SEARCH,
      WAIT_HDR,
      WAIT_DATA
    };

    packet_parser_b::sptr
    packet_parser_b::make()
    {
      return gnuradio::get_initial_sptr
        (new packet_parser_b_impl());
    }

    /*
     * The private constructor
     */
    packet_parser_b_impl::packet_parser_b_impl()
      : gr::sync_block("packet_parser_b",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(0, 0, 0)),
              d_cap(8192)
    {
      d_state = SEARCH;
      d_accesscode = 0xacdda4e2f28c20fc;
      d_accesscode_len=64;
      d_data_reg = 0x0000000000000000;

      d_hdr_count =0;
      d_pld_count =0;
      d_hdr_buf = new unsigned char[256];
      d_pld_buf = new unsigned char[d_cap];

      d_phy_port = pmt::mp("out");
      message_port_register_out(d_phy_port);
    }

    /*
     * Our virtual destructor.
     */
    packet_parser_b_impl::~packet_parser_b_impl()
    {
      delete [] d_hdr_buf;
      delete [] d_pld_buf;
    }

    int
    packet_parser_b_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      for(int i=0;i<noutput_items;++i){
        switch(d_state)
        {
          case SEARCH:
            d_data_reg = (d_data_reg<<1)| (in[i]&0x01);
            if(d_data_reg == d_accesscode){
              d_hdr_count =0;
              d_byte_count =0;
              d_state = WAIT_HDR;
            }
          break;
          case WAIT_HDR:
            d_hdr_buf[d_byte_count] = (d_hdr_buf[d_byte_count]<<1) | (in[i]&0x01);
            d_hdr_count++;
            if(d_hdr_count%8 == 0){
              d_byte_count++;
            }
            if(d_hdr_count == 32){
              if(parse_hdr()){
                // check sum passed
                if(d_hdr_rx == 0x0000){
                  //ack
                  pmt::pmt_t msg = pmt::cons(pmt::intern("LSA_ACK"),pmt::PMT_NIL);
                  message_port_pub(d_phy_port,msg);
                  d_state = SEARCH;
                }
                else if(d_hdr_rx == 0x0001){
                  //nack
                  pmt::pmt_t msg = pmt::cons(pmt::intern("LSA_NACK"),pmt::PMT_NIL);
                  message_port_pub(d_phy_port,msg);
                  d_state = SEARCH;
                }
                else{
                  //data
                  d_byte_count =0;
                  d_pld_count =0;
                  d_state = WAIT_DATA;
                }
              }//parse hdr
              else{
                // no information found
                d_state = SEARCH;
              }
            }
          break;
          case WAIT_DATA:
            d_pld_buf[d_byte_count] = (d_pld_buf[d_byte_count]<<1) | (in[i]&0x01);
            d_pld_count++;
            if(d_pld_count%8 == 0){
              d_byte_count++;
            }
            if(d_byte_count == d_hdr_rx){
              pub_data();
              d_state = SEARCH;
            }
          break;
        }
      }
      return noutput_items;
    }

    bool
    packet_parser_b_impl::parse_hdr()
    {
      uint16_t pld1=0x0000,pld2=0x0000;
      pld1 |= d_hdr_buf[0];
      pld1 = (pld1<<8) | d_hdr_buf[1];
      pld2 |= d_hdr_buf[2];
      pld2 = (pld2<<8) | d_hdr_buf[3];
      if(pld1==pld2){
        d_hdr_rx = pld1;
        return true;
      }
      return false;
    }

    void
    packet_parser_b_impl::pub_data()
    {
      pmt::pmt_t uvec = pmt::make_blob(d_pld_buf,d_byte_count);
      pmt::pmt_t packet = pmt::cons(pmt::intern("LSA_DATA"),uvec);
      message_port_pub(d_phy_port,packet);
    }

  } /* namespace lsa */
} /* namespace gr */

