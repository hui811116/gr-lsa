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
#include "symbol_queue_receiver_cc_impl.h"
#include <gnuradio/digital/constellation.h>

namespace gr {
  namespace lsa {

    enum bitState{
      SEARCH,
      WAIT_HDR
      //WAIT_PLD
    };

    symbol_queue_receiver_cc::sptr
    symbol_queue_receiver_cc::make(
      const std::string& accesscode,
      const std::string& sensing_tagname,
      const gr::digital::constellation_sptr& hdr_const,
      int sps,
      int pld_bps,
      bool debug)
    {
      return gnuradio::get_initial_sptr
        (new symbol_queue_receiver_cc_impl(
          accesscode,
          sensing_tagname,
          hdr_const,
          sps,
          pld_bps,
          debug));
    }

    /*
     * The private constructor
     */
    static int ios[] = {sizeof(gr_complex), sizeof(float),sizeof(float),sizeof(float),sizeof(float)};
    static std::vector<int> iosig(ios, ios+ sizeof(ios)/sizeof(int));
    symbol_queue_receiver_cc_impl::symbol_queue_receiver_cc_impl(
      const std::string& accesscode,
      const std::string& sensing_tagname,
      const gr::digital::constellation_sptr& hdr_const,
      int sps,
      int pld_bps,
      bool debug)
      : gr::block("symbol_queue_receiver_cc",
              gr::io_signature::makev(1, 5, iosig),
              gr::io_signature::makev(1, 5, iosig))
    {
      if(!set_accesscode(accesscode)){
        throw std::invalid_argument("Accesscode too long or empty");
      }
      d_hdr_const = hdr_const->base();
      d_hdr_bps = hdr_const->bits_per_symbol();
      d_state = SEARCH;
      d_hdr_pre_code = hdr_const->pre_diff_code();
      d_hdr_port = pmt::mp("header_info");
      d_debug = debug;
      d_sps = sps;
      d_pld_bps = pld_bps;
      d_sensing_tagname = pmt::string_to_symbol(sensing_tagname);
      message_port_register_out(d_hdr_port);
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    symbol_queue_receiver_cc_impl::~symbol_queue_receiver_cc_impl()
    {
    }

    bool
    symbol_queue_receiver_cc_impl::set_accesscode(const std::string& accesscode)
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
    symbol_queue_receiver_cc_impl::header_nbits() const
    {
      return d_accesscode_len + 32+16+8+8;
    }
    uint16_t
    symbol_queue_receiver_cc_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input_bits[begin_idx+i])? 1 : 0) << (15-i);  // NOTE: this is how bits arrangement should be like
      }
      return tmp;
    }
    uint8_t
    symbol_queue_receiver_cc_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input_bits[begin_idx+i])? 1:0 ) << (7-i);
      }
      return tmp;
    }
    bool 
    symbol_queue_receiver_cc_impl::insert_symbol(const gr_complex& symbol)
    {
      unsigned char hold_byte;
      hold_byte = d_hdr_const->decision_maker(&symbol);
      hold_byte = d_hdr_pre_code[hold_byte];
      switch(d_state){
        case SEARCH:
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
          for(int i=0;i<d_hdr_bps;++i){
            d_input_bits.push_back( (((hold_byte >> (d_hdr_bps-1-i)) & 0x01)==0x00 )? false : true );
          }
          if(d_input_bits.size() == (header_nbits()-d_accesscode_len) )
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
    bool
    symbol_queue_receiver_cc_impl::parse_header()
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
    symbol_queue_receiver_cc_impl::pub_hdr()
    {
      pmt::pmt_t msg = pmt::make_dict();
      msg = pmt::dict_add(msg, pmt::intern("LSA_hdr"),pmt::PMT_T);
      msg = pmt::dict_add(msg, pmt::intern("counter"),pmt::from_long(d_counter));
      msg = pmt::dict_add(msg, pmt::intern("queue_index"),pmt::from_long((long)d_qidx));
      msg = pmt::dict_add(msg, pmt::intern("queue_size"),pmt::from_long((long)d_qsize));
      msg = pmt::dict_add(msg, pmt::intern("payload"),pmt::from_long(d_payload_len*8/d_pld_bps));
      message_port_pub(d_hdr_port, msg);
    }
    void
    symbol_queue_receiver_cc_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      for(int i=0;i<ninput_items_required.size();++i)
        ninput_items_required[i] = noutput_items;
    }

    int
    symbol_queue_receiver_cc_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      gr_complex *out = (gr_complex *) output_items[0];
      float* freq, *phase, *rate_f,*rate_k;
      bool have_sync = (ninput_items.size()>=5 && output_items.size()>=5);
      // Do <+signal processing+>
      memcpy(out,in,sizeof(gr_complex)*noutput_items);
      if(have_sync){
        freq = (float*)output_items[1];
        phase = (float*)output_items[2];
        rate_f = (float*)output_items[3];
        rate_k = (float*)output_items[4];
        memcpy(freq, input_items[1], sizeof(float)*noutput_items);
        memcpy(phase, input_items[2], sizeof(float)*noutput_items);
        memcpy(rate_f, input_items[3], sizeof(float)*noutput_items);
        memcpy(rate_k, input_items[4], sizeof(float)*noutput_items);
      }
      for(int i=0;i<noutput_items;++i){
        if(insert_symbol(in[i])){
          // output item tags;
          pub_hdr();
          add_item_tag(0,nitems_written(0)+i, pmt::intern("LSA_hdr"),pmt::PMT_T);
          add_item_tag(0,nitems_written(0)+i, pmt::intern("queue_index"), pmt::from_long((long)d_qidx));
          add_item_tag(0,nitems_written(0)+i, pmt::intern("queue_size"), pmt::from_long((long)d_qsize));
          add_item_tag(0,nitems_written(0)+i, pmt::intern("counter"), pmt::from_long(d_counter));
          add_item_tag(0,nitems_written(0)+i, pmt::intern("payload"), pmt::from_long(d_payload_len*8/d_pld_bps));
        }
      }
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each(noutput_items);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lsa */
} /* namespace gr */

