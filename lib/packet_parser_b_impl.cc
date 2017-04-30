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
    enum LSAFLAG{
      ACK,
      NACK,
      DATA,
      SEN
    };

    packet_parser_b::sptr
    packet_parser_b::make(const std::string& accesscode)
    {
      return gnuradio::get_initial_sptr
        (new packet_parser_b_impl(accesscode));
    }

    /*
     * The private constructor
     */
    packet_parser_b_impl::packet_parser_b_impl(const std::string& accesscode)
      : gr::sync_block("packet_parser_b",
              gr::io_signature::make(1, 1, sizeof(char)),
              gr::io_signature::make(0, 0, 0)),
              d_cap(8192)
    {
      d_state = SEARCH;
      set_accesscode(accesscode);
      //d_accesscode = 0xacdda4e2f28c20fc;
      //d_accesscode_len=64;
      d_data_reg = 0x0000000000000000;

      d_pld_parse = 0;
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
            if(((d_data_reg ^ d_accesscode) & d_mask) == 0){
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
            if(d_hdr_count == (32) ){
              if(parse_hdr()){
                // check sum passed
                if( (d_hdr_rx == 0x0000) || (d_hdr_rx == 0x0001) || (d_hdr_rx == 0xffff) ){
                  //ack  // for addresses + block counter
                  d_pld_parse = 4;
                }
                else{
                  //data
                  d_pld_parse = d_hdr_rx;
                }
                d_byte_count =0;
                d_pld_count =0;
                d_state = WAIT_DATA;
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
            if(d_byte_count == d_pld_parse){
              LSAFLAG flag;
              switch(d_hdr_rx){
                case 0x0001:
                  flag = ACK;
                break;
                case 0x0000:
                  flag = NACK;
                break;
                case 0xffff:
                  flag = SEN;
                break;
                default:
                  flag = DATA;
                break;
              }
              pub_data(flag);
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
        if(pld1 <=1500 )
        d_hdr_rx = pld1;
        else{
          d_hdr_rx = 100;
          std::cout<<"<Warning> detecting packet with payload greater than 1500, force to 100"<<std::endl;
        }
        
        return true;
      }
      return false;
    }

    void
    packet_parser_b_impl::pub_data(size_t flag)
    {
      pmt::pmt_t phytag;
      size_t blob_len = 0;
      switch(flag)
      {
        case ACK:
          phytag = pmt::intern("ACK");
          blob_len = 4;
        break;
        case NACK:
          phytag = pmt::intern("NACK");
          blob_len = 4;
        break;
        case DATA:
          phytag = pmt::intern("LSA_DATA");
          blob_len = d_byte_count;
        break;
        case SEN:
          phytag = pmt::intern("SENSE");
          blob_len = 4;
        break;
        default:
          throw std::runtime_error("Bad phy flag");
        break;
      }
      pmt::pmt_t uvec = pmt::make_blob(d_pld_buf,blob_len);
      pmt::pmt_t packet = pmt::cons(phytag,uvec);
      message_port_pub(d_phy_port,packet);
    }

    void
    packet_parser_b_impl::set_accesscode(const std::string& accesscode)
    {
      if(accesscode.empty() || accesscode.length()>64)
        throw std::runtime_error("Invalid accesscode, length should between 0 and 64");
      d_accesscode_len = accesscode.length();
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (accesscode[i] & 1);
      }
    }

  } /* namespace lsa */
} /* namespace gr */

