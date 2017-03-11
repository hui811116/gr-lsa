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
#include "protocol_parser_b_impl.h"

namespace gr {
  namespace lsa {

    enum bitState{
      SEARCH,
      WAIT_HDR
    };

    protocol_parser_b::sptr
    protocol_parser_b::make(const std::string& code, bool debug)
    {
      return gnuradio::get_initial_sptr
        (new protocol_parser_b_impl(code,debug));
    }

    /*
     * The private constructor
     */
    protocol_parser_b_impl::protocol_parser_b_impl(const std::string& code, bool debug)
      : gr::block("protocol_parser_b",
              gr::io_signature::make(1, 1, sizeof(unsigned char)),
              gr::io_signature::make(0, 0, 0))
    {
      if(!set_accesscode(code)){
        throw std::runtime_error("Accesscode length should not be greater than 64");
      }
      d_hdr_len = 32+16+8+8;
      d_state = SEARCH;
      d_debug = debug;
      
      d_msg_port = pmt::mp("header");
      message_port_register_out(d_msg_port);
      d_input.clear();
    }

    /*
     * Our virtual destructor.
     */
    protocol_parser_b_impl::~protocol_parser_b_impl()
    {
    }

    bool
    protocol_parser_b_impl::set_accesscode(const std::string& code)
    {
      d_accesscode_len = code.length();
      if(d_accesscode_len >64){
        return false;
      }
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (code[i] & 1);
      }
      return true;
    }
    size_t
    protocol_parser_b_impl::header_nbits() const
    {
      return d_accesscode_len + 32+16+8+8;
    }

    unsigned int
    protocol_parser_b_impl::_get_bit32(int begin_idx)
    {
      unsigned int tmp =0;
      for(int i=0;i<32;++i)
      {
        tmp |= ((d_input[begin_idx+i])? 1 : 0) << (31-i);
      }
      return tmp;
    }

    uint16_t
    protocol_parser_b_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input[begin_idx+i])? 1 : 0) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }

    uint8_t
    protocol_parser_b_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input[begin_idx+i])? 1:0 ) << (7-i);
      }
      return tmp;
    }

    bool
    protocol_parser_b_impl::parse_header()
    {
      uint16_t len0,len1;    
      len0 = _get_bit16(16);
      len1 = _get_bit16(32);
      if(len0 == len1){
        d_payload_len=len0;
        d_counter = _get_bit16(48);
        d_qidx = _get_bit8(0);
        d_qsize = _get_bit8(8);
        if(d_debug){
          std::stringstream ss;
          ss<<"Packet found-->"<<"payload_len:"<<len0<<", counter:"<<d_counter<<", qidx:"<<(int)d_qidx<<", qsize:"<<(int)d_qsize;
          GR_LOG_DEBUG(d_logger,ss.str());
        }
        return true;
      }
      return false;
    }

    void
    protocol_parser_b_impl::output_msg()
    {
      pmt::pmt_t msg=pmt::make_dict();
      msg = pmt::dict_add(msg, pmt::intern("payload_len"),pmt::from_long(d_payload_len));
      msg = pmt::dict_add(msg, pmt::intern("counter"),pmt::from_long(d_counter));
      msg = pmt::dict_add(msg, pmt::intern("queue_index"),pmt::from_long((int)d_qidx));
      msg = pmt::dict_add(msg, pmt::intern("queue_size"),pmt::from_long((int)d_qsize));

      message_port_pub(d_msg_port,msg);
    }
    void
    protocol_parser_b_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[i] = noutput_items;
    }

    int
    protocol_parser_b_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const unsigned char *in = (const unsigned char *) input_items[0];
      //gr_complex *out = (<+OTYPE+> *) output_items[0];
      int nin = ninput_items[0];
      for(int i=0;i<nin;++i)
      {
        switch(d_state)
        {
          case SEARCH:
            d_data_reg = (d_data_reg << 1) | (in[i]& 0x01 );
            if(((d_data_reg ^ d_accesscode) & d_mask) == 0){
              d_state = WAIT_HDR;
              d_input.clear();
            }
          break;
          case WAIT_HDR:
            d_input.push_back( in[i] & 0x01);
            if(d_input.size()==d_hdr_len){
              if(parse_header()){
                output_msg();
              }
              else{
                if(d_debug){
                  GR_LOG_DEBUG(d_logger,"header accesscode found but checksum failed");
                }
              }
              d_state = SEARCH;
            }
          break;
          default:
            throw std::runtime_error("Entering wrong state");
          break;
        }
      }

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

