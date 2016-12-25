/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include "su_sample_receiver_cb_impl.h"
//#include <pmt/pmt.h>

namespace gr {
  namespace lsa {

    su_sample_receiver_cb::sptr
    su_sample_receiver_cb::make(
      const std::string& sensing_tag_id,
      const std::string& accesscode,
      const gr::digital::constellation_sptr& hdr_const)
    {
      return gnuradio::get_initial_sptr
        (new su_sample_receiver_cb_impl(sensing_tag_id,accesscode,hdr_const));
    }

    enum SuRxState{
      SU_ONLY,
      INTERFERING
    };

    enum SyncState{
      SEARCH_SYNC_CODE,
      SYNC_WAIT_HEADER,
      WAIT_PAYLOAD
    };
    /*
     * The private constructor
     */
    su_sample_receiver_cb_impl::su_sample_receiver_cb_impl(
      const std::string& sensing_tag_id,
      const std::string& accesscode, 
      const gr::digital::constellation_sptr& hdr_const)
      : gr::block("su_sample_receiver_cb",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 1, sizeof(char))),
        d_src_id(pmt::intern(alias()))
    {
      d_hdr_sptr = hdr_const->base();
      d_state = SU_ONLY;
      d_bit_state = SEARCH_SYNC_CODE;

      d_msg_port = pmt::mp("sensing_info");
      d_debug_port = pmt::mp("debug");
      d_sensing_tag_id = pmt::string_to_symbol(sensing_tag_id);

      d_cap = 16*1024;
      d_byte_count = 0;
      //const size_t nitems = 4096;
      set_max_noutput_items(d_cap);
      if(!set_accesscode(accesscode)){
        throw std::runtime_error("SU Receiver: Setting access code failed");
      }
      d_byte_reg = (unsigned char*) malloc(sizeof(char)*d_cap);
      // d_name = (gr_complex*) volk_malloc(sizeof(gr_complex)*nitems, volk_get_alignment());
      //int nsamples
      //set_output_multiple(nsamples);
      //set_history(1);
      //declare_sample_delay(port, delay);

      message_port_register_out(d_msg_port);
      message_port_register_out(d_debug_port);
      set_tag_propagation_policy(TPP_DONT);
    }

    /*
     * Our virtual destructor.
     */
    su_sample_receiver_cb_impl::~su_sample_receiver_cb_impl()
    {
      free(d_byte_reg);
    }

    void
    su_sample_receiver_cb_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0]=noutput_items;
      /*
      int hdr_symbol_len=(header_nbits()+d_accesscode_len)/d_hdr_sptr->bits_per_symbol();
      switch(d_bit_state)
      {
        case SEARCH_SYNC_CODE:
        case SYNC_WAIT_HEADER:
        case WAIT_PAYLOAD:
          ninput_items_required[0] = noutput_items;
        break;
        default:
        break;
      }
      */
    }

    bool
    su_sample_receiver_cb_impl::symbol_segment(
      std::vector<tag_t>& intf_idx,
      const std::vector<tag_t>& tags,
      int nsamples)
    {
      bool sensing_result;
      for(int i=0;i<tags.size();++i)
      {
        if(tags[i].key == d_sensing_tag_id){
          sensing_result = pmt::to_bool(tags[i].value);
          int offset_check=tags[i].offset-nitems_read(0);
          if( (offset_check>=0) && (offset_check < nsamples) )
            intf_idx.push_back(tags[i]);
        }
      }
      return intf_idx.empty();
    }

    bool
    su_sample_receiver_cb_impl::pub_byte_pkt(std::vector<unsigned char>& pub)
    {
      if(d_byte_count ==0){
        return false;
      }
      else{
        for(int i=0;i<d_byte_count;++i){
          pub.push_back(d_byte_reg[i]);
        }
        d_byte_count = 0;
        return true;
      }
      
    }


    void
    su_sample_receiver_cb_impl::feedback_info(bool type)
    {
      pmt::pmt_t sen_back;
      //interfering case
      if(type)
      {
        sen_back = pmt::make_dict();
        sen_back =  pmt::dict_add(sen_back,pmt::intern("sense"),pmt::from_bool(true));
      }
      //su packet received
      else
      {
        sen_back = pmt::make_dict();
        sen_back = pmt::dict_add(sen_back,pmt::intern("sense"),pmt::from_bool(false));
        sen_back = pmt::dict_add(sen_back,pmt::intern("payload"),pmt::from_long(d_payload_len));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_index"),pmt::from_long(d_qidx));
        sen_back = pmt::dict_add(sen_back,pmt::intern("queue_index"),pmt::from_long(d_qsize));
        sen_back = pmt::dict_add(sen_back,pmt::intern("counter"),pmt::from_long(d_counter));
      }
      message_port_pub(d_msg_port,sen_back);
    }

    void
    su_sample_receiver_cb_impl::data_reg_reset()
    {
      d_byte_count=0;
    }

    size_t
    su_sample_receiver_cb_impl::header_nbits() const
    {
      return d_accesscode_len + 32+16+8+8;
    }

    bool
    su_sample_receiver_cb_impl::set_accesscode(const std::string& accesscode)
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
    su_sample_receiver_cb_impl::accesscode() const
    {
      return d_accesscode;
    }
    //helper function for header parsing
    uint16_t
    su_sample_receiver_cb_impl::_get_bit16(int begin_idx)
    {
      unsigned long tmp=0UL;
      for(int i=0;i<16;++i){
        tmp |= ((d_input[begin_idx+i])? 1 : 0) << i;
      }
      return tmp;
    }
    uint8_t
    su_sample_receiver_cb_impl::_get_bit8(int begin_idx)
    {
      unsigned char tmp=0;
      for(int i=0;i<8;++i){
        tmp |= ((d_input[begin_idx+i])? 1:0 ) << i;
      }
      return tmp;
    }

    bool
    su_sample_receiver_cb_impl::parse_header()
    {
      uint16_t len0,len1;    
      len0 = _get_bit16(0);
      len1 = _get_bit16(16);
      if(len0 == len1){
        d_payload_len=len0;
        d_counter = _get_bit16(32);
        d_qidx = _get_bit8(48);
        d_qsize = _get_bit8(56);
        return true;
      }
      return false;
    }

    void
    su_sample_receiver_cb_impl::insert_parse_byte(std::vector<unsigned char>& out)
    {
      unsigned char byte_hold = d_byte_reg[d_byte_count-1];
      //MSB parsing
      int k_bits = d_hdr_sptr->bits_per_symbol();
      
      unsigned char bitstream[k_bits];
      for(int i=0;i < k_bits;++i){
        bitstream[i] = 0x01 & (byte_hold >> (k_bits-1-i));
      }
      for(int i=0;i<k_bits;++i){
        uint64_t check_bits = (~0ULL);
        switch(d_bit_state)
        {
          case SEARCH_SYNC_CODE:
            d_data_reg = (d_data_reg << 1) | (bitstream[i]);  
            check_bits = (d_data_reg ^ d_accesscode) & d_mask;
            if(check_bits == 0){
              d_bit_state = SYNC_WAIT_HEADER;
              d_input.clear();
              d_byte_count = 0;
            }      
          break;
          case SYNC_WAIT_HEADER:
            d_input.push_back( (bitstream[i]==0x00)? false : true);
            if(d_input.size() == (header_nbits()-d_accesscode_len) )
            {
              //int payload_len=0; // can construct a class variable
              if(parse_header()){
                d_bit_state = WAIT_PAYLOAD;
              }
              else{
                d_bit_state = SEARCH_SYNC_CODE;
              }
            }
          break;
          case WAIT_PAYLOAD:
            d_input.push_back( (bitstream[i]==0x00)? false : true);
            if(d_input.size() == (header_nbits() + d_payload_len*8)){
              d_bit_state = SEARCH_SYNC_CODE;
              for(int i=0;i<d_input.size();++i){
                out.push_back((d_input[i])? 0x01:0x00);
              }
            }
          break;
          default:
            std::runtime_error("SU Receiver: Entering wrong state");
          break;
        }
      }
    }
    int
    su_sample_receiver_cb_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      unsigned char out_bytes[noutput_items];
      unsigned char *out = out_bytes;
      if(!output_items.empty()){
        out = (unsigned char *) output_items[0];  
      }
      // assume input is complex symbols, with tags of sensing info to be feedback
      int out_count=0;
      
      std::vector<tag_t> intf_idx;
      std::vector<tag_t> tags;
      std::vector<unsigned char> hold_bits;
      get_tags_in_range(tags,0,nitems_read(0),nitems_read(0)+noutput_items);
      if(!symbol_segment(intf_idx,tags,noutput_items)){
        //this case represent no state info found
        if(d_state == SU_ONLY){
            for(int i=0;i<noutput_items;++i){
              d_byte_reg[d_byte_count] = d_hdr_sptr->decision_maker(&in[i]);
              
              if(!output_items.empty()){
                out[out_count++]=d_byte_reg[d_byte_count];
              }
              d_byte_count++;
              insert_parse_byte(hold_bits);
              if(d_byte_count == d_cap){
                GR_LOG_CRIT(d_logger, "SU Receiver: Reaching maximum capacity, reset to initial.");
                d_byte_count = 0;
              }
              if(!hold_bits.empty()){
                //publish message
                feedback_info(false);
                hold_bits.clear();
              }
            }
        }
      }
      else{
        //there are states to be checked
        int tag_count = 0;
        int offset_reg = intf_idx[tag_count].offset-nitems_read(0);
        bool next_info = pmt::to_bool(intf_idx[tag_count].value);
        for(int i=0;i<noutput_items;++i){
          if( (offset_reg == i)){
            if( tag_count < (intf_idx.size()-1)){
              tag_count++;
              next_info = pmt::to_bool(intf_idx[tag_count].value);
              offset_reg = intf_idx[tag_count].offset-nitems_read(0);
            }
            d_state = (next_info) ? INTERFERING : SU_ONLY;
            if((d_state == INTERFERING) && (d_byte_count != 0) ){
              // process current bytes
                feedback_info(true);
                //TODO a reset function to initialize data register
                data_reg_reset();
            }
          }
          switch(d_state){
            case SU_ONLY:
              d_byte_reg[d_byte_count] = d_hdr_sptr->decision_maker(&in[i]);
              if(!output_items.empty()){
                out[out_count++] = d_byte_reg[d_byte_count];
              }
              d_byte_count++;
              insert_parse_byte(hold_bits);
              if(!hold_bits.empty()){
                //publish message
                feedback_info(false);
                hold_bits.clear();
              }
              //input to data register and run searching code alg.
            break;
            case INTERFERING:
            default:
              std::runtime_error("SU Receiver: sensing info into wrong state");
            break;
          } // end switch
        }//end for

      }
      
      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (noutput_items);

      // Tell runtime system how many output items we produced.
      return out_count;
    }

  } /* namespace lsa */
} /* namespace gr */

