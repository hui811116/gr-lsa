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
#include "symbol_sync_receiver_cf_impl.h"
#include <gnuradio/math.h>
#include <gnuradio/expj.h>
#include <ctime>
#include <pmt/pmt.h>

namespace gr {
  namespace lsa {
    enum syncState{
      SEARCH,
      WAIT_HDR
    };

    symbol_sync_receiver_cf::sptr
    symbol_sync_receiver_cf::make(
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new symbol_sync_receiver_cf_impl(
          accesscode,
          hdr_const,
          pld_const,
          debug));
    }

    /*
     * The private constructor
     */
    symbol_sync_receiver_cf_impl::symbol_sync_receiver_cf_impl(
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const,
      const gr::digital::constellation_sptr& pld_const,
      bool debug)
      : gr::block("symbol_sync_receiver_cf",
              gr::io_signature::make3(1, 3, sizeof(gr_complex),sizeof(float),sizeof(float)),
              gr::io_signature::make2(0, 2, sizeof(float),sizeof(float)))
    {
      if(!set_accesscode(accesscode)){
        throw std::invalid_argument("Invalid accesscode!");
      }
      d_hdr_const = hdr_const->base();
      d_pld_const = pld_const->base();

      d_pld_map = pld_const->pre_diff_code();
      d_hdr_map = hdr_const->pre_diff_code();

      d_hdr_bps = hdr_const->bits_per_symbol();
      d_pld_bps = pld_const->bits_per_symbol();

      d_symbol_count=0;
      d_timetag = pmt::string_to_symbol("ctime");
      d_msg_port = pmt::mp("msg");

      d_debug = debug;
      d_state = SEARCH;

      set_tag_propagation_policy(TPP_DONT);
      message_port_register_out(d_msg_port);
    }

    /*
     * Our virtual destructor.
     */
    symbol_sync_receiver_cf_impl::~symbol_sync_receiver_cf_impl()
    {
    }
    bool
    symbol_sync_receiver_cf_impl::set_accesscode(const std::string& accesscode)
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
    symbol_sync_receiver_cf_impl::header_nbits() const
    {
      return d_accesscode_len + 8+8+16*2+16;
    }

    bool 
    symbol_sync_receiver_cf_impl::insert_symbol(const gr_complex& symbol)
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
    symbol_sync_receiver_cf_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= (d_input[begin_idx+i]) << (15-i);  
        // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }
    uint8_t
    symbol_sync_receiver_cf_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= (d_input[begin_idx+i]) << (7-i);
      }
      return tmp;
    }

    bool
    symbol_sync_receiver_cf_impl::parse_header()
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

    void
    symbol_sync_receiver_cf_impl::msg_out(int noutput_items, bool hdr)
    {
      pmt::pmt_t msg = pmt::make_dict();
      msg = pmt::dict_add(msg, pmt::intern("ctime"),pmt::from_long(d_current_time));
      msg = pmt::dict_add(msg, pmt::intern("buffer_offset"),pmt::from_long(d_symbol_count));
      if(!hdr){
        // for sync info
        msg = pmt::dict_add(msg, pmt::intern("output_items"),pmt::from_long(noutput_items));
      }
      else{
        // for header info
        msg = pmt::dict_add(msg, pmt::intern("LSA_hdr"),pmt::PMT_T);
        msg = pmt::dict_add(msg, pmt::intern("queue_index"),pmt::from_long(d_qidx));
        msg = pmt::dict_add(msg, pmt::intern("queue_size"),pmt::from_long(d_qsize));
        msg = pmt::dict_add(msg, pmt::intern("payload"),pmt::from_long(8*d_pld_len/d_pld_bps));
        //for debug
        msg = pmt::dict_add(msg, pmt::intern("counter"),pmt::from_long(d_counter));
      }
      message_port_pub(d_msg_port, msg);
    }

    void
    symbol_sync_receiver_cf_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i){
        ninput_items_required[i] = noutput_items;
      }
    }

    int
    symbol_sync_receiver_cf_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      const float* phase = NULL;
      const float* time = NULL;
      float *out_phase = NULL;
      float *out_time = NULL;
      
      bool have_sync = (input_items.size()>=3);
      bool out_sync = (output_items.size()>=2);

      // Do <+signal processing+>
      std::vector<tag_t> time_tag;
      get_tags_in_range(time_tag, 0, nitems_read(0),nitems_read(0)+noutput_items, d_timetag);
      for(int i=0;i<noutput_items;++i){
        if(!time_tag.empty()){
          int offset = time_tag[0].offset-nitems_read(0);
          if(i==offset){
            d_current_time = pmt::to_long(time_tag[0].value);
            d_symbol_count=0;
            //for sync purpose
            add_item_tag(0,nitems_written(0)+i,time_tag[0].key,time_tag[0].value);
            msg_out(noutput_items,false);
            time_tag.erase(time_tag.begin());
          }
        }
        if(insert_symbol(in[i])){          
          if(out_sync){
            add_item_tag(0,nitems_written(0)+i,pmt::intern("LSA_hdr"),pmt::PMT_T);
            add_item_tag(0,nitems_written(0)+i,pmt::intern("queue_index"),pmt::from_long(d_qidx));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("queue_size"),pmt::from_long(d_qsize));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("counter"),pmt::from_long(d_counter));
            add_item_tag(0,nitems_written(0)+i,pmt::intern("payload"),pmt::from_long(d_pld_len*8/d_pld_bps));
          }
          msg_out(noutput_items, true);
        }
        d_symbol_count++;
      }
      if(have_sync){
        phase = (const float*) input_items[1];
        time = (const float*) input_items[2];
        if(out_sync){
          out_phase = (float *) output_items[0];
          out_time = (float *) output_items[1];
          memcpy(out_phase,phase, sizeof(float)*noutput_items);
          memcpy(out_time,time,sizeof(float)*noutput_items);  
          produce(0,noutput_items);
          produce(1,noutput_items);
        }
        consume(1,noutput_items);
        consume(2,noutput_items);
      }
      consume(0,noutput_items);
      // Tell runtime system how many output items we produced.
      return WORK_CALLED_PRODUCE;
    }

  } /* namespace lsa */
} /* namespace gr */

