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
#include "symbol_receiver_c_impl.h"
#include <gnuradio/blocks/pdu.h>
#include <gnuradio/math.h>
#include <gnuradio/sincos.h>
#include <gnuradio/expj.h>
#include <sstream>
#include <ctime>


namespace gr {
  namespace lsa {

    enum bitState{
      SEARCH,
      WAIT_HDR,
      WAIT_PLD
    };

    symbol_receiver_c::sptr
    symbol_receiver_c::make(const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        const std::string& sensing_tagname,
        int sps,
        bool debug)
    {
      return gnuradio::get_initial_sptr
        (new symbol_receiver_c_impl(accesscode, hdr_const,pld_const,sensing_tagname,sps,debug));
    }

    /*
     * The private constructor
     */
    symbol_receiver_c_impl::symbol_receiver_c_impl(const std::string& accesscode,
        const gr::digital::constellation_sptr& hdr_const,
        const gr::digital::constellation_sptr& pld_const,
        const std::string& sensing_tagname,
        int sps,
        bool debug)
      : gr::sync_block("symbol_receiver_c",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0))
    {
      d_hdr_const = hdr_const->base();
      d_pld_const = pld_const->base();
      d_hdr_bps = hdr_const->bits_per_symbol();
      d_pld_bps = pld_const->bits_per_symbol();
      d_state = SEARCH;

      d_hdr_port = pmt::mp("info");
      d_pkt_port = pmt::mp("packet");
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      d_byte_count =0;
      if(!set_accesscode(accesscode)){
        throw std::runtime_error("Invalid accesscode!");
      }
      const int cap = 1024*64;
      d_byte_reg = new unsigned char[cap];//
      d_symbol_to_bytes = new unsigned char[cap];
      d_debug = debug;
      d_sps = sps;

      message_port_register_out(d_hdr_port);
      message_port_register_out(d_pkt_port);

      d_time_tagname  = pmt::string_to_symbol("ctime");
      d_prev_time  = 0;
      d_current_time = 0;
      d_symbol_count =0;
    }

    /*
     * Our virtual destructor.
     */
    symbol_receiver_c_impl::~symbol_receiver_c_impl()
    {
      delete [] d_byte_reg;
      delete [] d_symbol_to_bytes;
    }

    void
    symbol_receiver_c_impl::pub_byte_pkt()
    {  
        int bits_count = d_pld_bps * d_byte_count;
        int byte_count = bits_count/8;
        unsigned char tmp;   
        
        for(int i=0;i<bits_count;++i){
          if(i%8==0){
            d_symbol_to_bytes[i/8]=0x00;
          }
          tmp = (d_byte_reg[i/d_pld_bps] >> (d_pld_bps-1-i%d_pld_bps)) & 0x01;
          d_symbol_to_bytes[i/8] =  ((d_symbol_to_bytes[i/8] >> (7-i%8)) | tmp ) << 7-i%8;
        }

        pmt::pmt_t pdu_meta = pmt::make_dict();
        pdu_meta = pmt::dict_add(pdu_meta, pmt::intern("payload_len"), pmt::from_long(d_payload_len));
        pdu_meta = pmt::dict_add(pdu_meta, pmt::intern("counter"), pmt::from_long(d_counter));
        
        d_pdu_vector = gr::blocks::pdu::make_pdu_vector(gr::blocks::pdu::byte_t,d_symbol_to_bytes,byte_count);
        pmt::pmt_t msg = pmt::cons(pdu_meta, d_pdu_vector);
        message_port_pub(d_pkt_port, msg);
    }

    size_t
    symbol_receiver_c_impl::header_nbits() const
    {
      return d_accesscode_len + 32+16+8+8;
    }

    bool
    symbol_receiver_c_impl::set_accesscode(const std::string& accesscode)
    {
      d_accesscode_len = accesscode.length();
      if(d_accesscode_len >64){
        return false;
      }
      d_mask = ((~0ULL) >> (64- d_accesscode_len) );
      d_accesscode=0;
      for(unsigned i=0;i<d_accesscode_len;++i){
        d_accesscode = (d_accesscode << 1) | (accesscode[i] & 1);
      }
      return true;
    }

    uint64_t
    symbol_receiver_c_impl::accesscode() const
    {
      return d_accesscode;
    }
    //helper function for header parsing
    uint16_t
    symbol_receiver_c_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input_bits[begin_idx+i])? 1 : 0) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }
    uint8_t
    symbol_receiver_c_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input_bits[begin_idx+i])? 1:0 ) << (7-i);
      }
      return tmp;
    }

    bool
    symbol_receiver_c_impl::parse_header()
    {
      uint16_t len0,len1;    
      len0 = _get_bit16(0);
      len1 = _get_bit16(16);
      if(len0 == len1){
        d_payload_len=len0;
        d_counter = _get_bit16(32);
        d_qidx = _get_bit8(48);
        d_qsize = _get_bit8(56);
        if(d_debug){
          std::stringstream ss;
          ss<<"Packet found-->"<<"payload_len:"<<len0<<", counter:"<<d_counter<<", qidx:"<<(int)d_qidx<<", qsize:"<<(int)d_qsize;
          GR_LOG_DEBUG(d_logger,ss.str());
        }
        return true;
      }
      return false;
    }

    bool 
    symbol_receiver_c_impl::insert_symbol(const gr_complex& symbol)
    {
      unsigned char hold_byte;
      switch(d_state){
        case SEARCH:
          hold_byte = d_hdr_const->decision_maker(&symbol);
          for(int i=0;i<d_hdr_bps;++i){
            uint64_t check_bits = (~0ULL);
            d_data_reg = (d_data_reg << 1) | ((hold_byte >> (d_hdr_bps-1-i) )& 0x01 );
            check_bits = (d_data_reg ^ d_accesscode) & d_mask;
            if(check_bits == 0){
              d_state = WAIT_HDR;
              d_input_bits.clear();
            }
          }
        break;
        case WAIT_HDR:
          hold_byte = d_hdr_const->decision_maker(&symbol);
          for(int i=0;i<d_hdr_bps;++i){
            d_input_bits.push_back( (((hold_byte >> (d_hdr_bps-1-i)) & 0x01)==0x00 )? false : true );
          }
          if(d_input_bits.size() == (header_nbits()-d_accesscode_len) )
          {
            if(parse_header()){
              d_state = WAIT_PLD;
            }
            else{
              d_state = SEARCH;
            }
            d_byte_count=0;
          }
        break;
        case WAIT_PLD:
          //hold_byte = d_pld_const->decision_maker(&symbol);
          d_byte_reg[d_byte_count++] = d_pld_const->decision_maker(&symbol);
          if(d_payload_len*8 == d_byte_count*d_pld_bps)
          {
            pub_byte_pkt();//FIXME
            d_state = SEARCH;
            return true;
          }
        break;
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
      return false;
    }

    void
    symbol_receiver_c_impl::feedback_info(bool type)
    {
      pmt::pmt_t sen_back = pmt::make_dict();
      //interfering case
      sen_back = pmt::dict_add(sen_back,d_sensing_tagname,pmt::from_bool(type));
      if(!type)
      {
        //su pkt received
        //FIXME
        //payload length for feedback only matters in samples!!
        //sen_back = pmt::dict_add(sen_back,pmt::intern("payload"),pmt::from_long(d_payload_len));
        int sample_length = d_payload_len*8/ d_pld_bps *d_sps;
        sen_back = pmt::dict_add(sen_back,pmt::intern("payload"),pmt::from_long(sample_length));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_index"),pmt::from_long(d_qidx));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_size"),pmt::from_long(d_qsize));
        sen_back = pmt::dict_add(sen_back,pmt::intern("counter"),pmt::from_long(d_counter));
        //time index for feedback labeling in queue
        //FIXME
        // this index meant for the end of header
        sen_back = pmt::dict_add(sen_back,pmt::intern("buffer_offset"),pmt::from_long(d_symbol_count*d_sps));
        sen_back = pmt::dict_add(sen_back,pmt::intern("ctime"),pmt::from_long(d_current_time));
      }
      message_port_pub(d_hdr_port,sen_back);
    }

    int
    symbol_receiver_c_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      //should be symbol
      // Do <+signal processing+>
      std::vector<tag_t> time_tags;
      std::vector<tag_t> tags;
      //std::vector<tag_t> qindex_tags;
      get_tags_in_window(tags, 0,0,noutput_items, d_sensing_tagname);
      get_tags_in_window(time_tags, 0,0,noutput_items, d_time_tagname);
      
      for(int i=0;i<noutput_items;++i){
        if(!tags.empty()){
          //handling interference detection
          int offset = tags[0].offset- nitems_read(0);
          bool sense_result = pmt::to_bool(tags[0].value);
          if(offset==i){
            if(sense_result){
              feedback_info(true);
              d_byte_count = 0;
              d_state = SEARCH;
            }
            tags.erase(tags.begin());
          }
        }
        if(!time_tags.empty()){
          int offset = time_tags[0].offset - nitems_read(0); 
          if(offset == i){
            d_current_time = pmt::to_long(time_tags[0].value);
            d_symbol_count =0;
            d_prev_time = d_current_time;
            time_tags.erase(time_tags.begin());
          }
        }
        if(insert_symbol(in[i])){
          feedback_info(false);
          d_byte_count =0;
          d_state = SEARCH;
          if(d_debug){
            std::stringstream ss;
            ss<<"header found, current time:"<<d_current_time;
            ss<< " ,sample offset="<<d_sps * d_symbol_count;
            GR_LOG_DEBUG(d_logger,ss.str());
          }
        }
        d_symbol_count++;
      }
      // Tell runtime system how many output items we produced.

      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

