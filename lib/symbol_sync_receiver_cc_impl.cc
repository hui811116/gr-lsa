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
#include "symbol_sync_receiver_cc_impl.h"
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <ctime>

namespace gr {
  namespace lsa {

    enum syncState{
      SEARCH,
      WAIT_HDR
    };

    symbol_sync_receiver_cc::sptr
    symbol_sync_receiver_cc::make(
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new symbol_sync_receiver_cc_impl(
          accesscode,
          hdr_const,
          pld_const,
          debug));
    }

    /*
     * The private constructor
     */
    static int ios[]={sizeof(gr_complex), sizeof(float), sizeof(float), sizeof(float), sizeof(float)};
    static std::vector<int> iosig(ios,ios+sizeof(ios)/sizeof(int));
    static int os[]={sizeof(float), sizeof(float),sizeof(float),sizeof(float)};
    static std::vector<int> osig(os,os+sizeof(os)/sizeof(int));

    symbol_sync_receiver_cc_impl::symbol_sync_receiver_cc_impl(
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      bool debug)
      : gr::sync_block("symbol_sync_receiver_cc",
              gr::io_signature::makev(1, 5, iosig),
              gr::io_signature::makev(0, 4, osig))
    {
      d_hdr_const = hdr_const->base();
      d_pld_const = pld_const->base();

      d_pld_map = pld_const->pre_diff_code();
      d_hdr_map = hdr_const->pre_diff_code();

      d_hdr_bps = hdr_const->bits_per_symbol();
      d_pld_bps = pld_const->bits_per_symbol();

      d_symbol_count=0;
      d_timetag = pmt::string_to_symbol("ctime");

      d_debug = debug;

    }

    /*
     * Our virtual destructor.
     */
    symbol_sync_receiver_cc_impl::~symbol_sync_receiver_cc_impl()
    {
    }

    bool
    symbol_sync_receiver_cc_impl::set_accesscode(const std::string& accesscode)
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

    size_t
    symbol_sync_receiver_cc_impl::header_nbits() const
    {
      return d_accesscode_len + 8+8+16*2+16;
    }

    bool 
    symbol_sync_receiver_cc_impl::insert_symbol(const gr_complex& symbol)
    {
      unsigned char hold_byte;
      hold_byte = d_hdr_const->decision_maker(&symbol);
      hold_byte = d_hdr_map[hold_byte];
      switch(d_state){
        case SEARCH:
          for(int i=0;i<d_hdr_bps;++i){
            uint64_t check_bits = (~0ULL);
            d_data_reg = (d_data_reg << 1) | ((hold_byte >> (d_hdr_bps-1-i) )& 0x01 );
            check_bits = (d_data_reg ^ d_accesscode) & d_mask;
            if(check_bits == 0){
              d_state = WAIT_HDR;
              d_input.clear();
            }
          }
        break;
        case WAIT_HDR:
          for(int i=0;i<d_hdr_bps;++i){
            d_input.push_back( (hold_byte >> (d_hdr_bps-1-i)) & 0x01 );
          }
          if(d_input.size() == (header_nbits()-d_accesscode_len) )
          {
            if(parse_header()){
              d_state = SEARCH;
              return true;
            }
            if(d_debug){
              GR_LOG_DEBUG(d_logger, "HEADER CHUNKSUM NOT PASSED");
            }
            d_state = SEARCH;
          }
        break;
        
        default:
          std::runtime_error("Entering wrong state");
        break;
      }
      return false;
    }

    uint16_t
    symbol_sync_receiver_cc_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input[begin_idx+i])? 1 : 0) << (15-i);  
        // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }
    uint8_t
    symbol_sync_receiver_cc_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input[begin_idx+i])? 1:0 ) << (7-i);
      }
      return tmp;
    }

    bool
    symbol_sync_receiver_cc_impl::parse_header()
    {
      uint16_t len0,len1;    
      len0 = _get_bit16(16);
      len1 = _get_bit16(32);
      if(len0 == len1){
        d_pld_len=len0;
        d_counter = _get_bit16(48);
        d_qidx = _get_bit8(0);
        d_qsize = _get_bit8(8);
        if(d_debug){
          std::cout<<"Packet found-->"<<"payload_len:"<<len0
          <<", counter:"<<d_counter<<", qidx:"<<(int)d_qidx<<", qsize:"
          <<(int)d_qsize<<std::endl;
        }
        return true;
      }
      return false;
    }

    int
    symbol_sync_receiver_cc_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      bool have_sync = (input_items.size()>=5) && (output_items.size()>=4);
      const float* freq_in,*phase_in, *time_rate_in, *time_k_in;
      float* freq_out,*phase_out,*time_rate_out, *time_k_out;
      if(have_sync){
        freq_in = (const float*)input_items[1];
        phase_in = (const float*)input_items[2];
        time_rate_in = (const float*)input_items[3];
        time_k_in = (const float*)input_items[4];

        freq_out = (float*)output_items[0];
        phase_out = (float*)output_items[1];
        time_rate_out = (float*)output_items[2];
        time_k_out = (float*)output_items[3];
        memcpy(freq_out, freq_in,sizeof(float)*noutput_items);
        memcpy(phase_out, phase_in, sizeof(float)*noutput_items);
        memcpy(time_rate_out,time_rate_in,sizeof(float)*noutput_items);
        memcpy(time_k_out,time_k_in,sizeof(float)*noutput_items);
      }

      std::vector<tag_t> time_tag;
      get_tags_in_range(time_tag, 0, nitems_read(0),nitems_read(0)+noutput_items, d_timetag);

      for(int i=0;i<noutput_items;++i){
        while(!time_tag.empty()){
          uint64_t offset = time_tag[0].offset-nitems_read(0);
          if(i==offset){
            d_current_time = pmt::to_long(time_tag[0].value);
            d_symbol_count=0;
          }
          time_tag.erase(time_tag.begin());
        }
        if(insert_symbol(in[i])){
          if(d_debug){
            std::cout<<"<debug>counter:"<<d_counter<<" ,payload:"<<d_pld_len/d_pld_bps
            <<" ,queue_index:"<<d_qidx<<" ,queue_size:"<<d_qsize<<std::endl;
          }          
          if(have_sync){
            add_item_tag(0,nitems_written(0)+i,pmt::intern("LSA_hdr"),pmt::PMT_T);
            add_item_tag(0,nitems_written(0)+i,pmt::intern("queue_index"),pmt::from_long(d_qidx));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("queue_size"),pmt::from_long(d_qsize));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("counter"),pmt::from_long(d_counter));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("payload"),pmt::from_long(d_pld_len/d_pld_bps));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("ctime"),pmt::from_long(d_current_time));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("buffer_offset"),pmt::from_long(d_symbol_count));
          }
        }
        d_symbol_count++;
      }

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

